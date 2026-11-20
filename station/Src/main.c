/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Weather station firmware (STM32L152RE + X-NUCLEO-IKS01A3)
  ******************************************************************************
  * Three uC/OS-III tasks:
  *
  *   AppTaskStart  (prio 1)  board bring-up, spawns the others, heartbeat LED
  *   UartSend      (prio 2)  waits on its task queue, frames samples, UART TX
  *   SensorRead    (prio 3)  reads HTS221 / LPS22HH / STTS751 every 2 s and
  *                           posts a sample to UartSend's queue
  *
  * Wire format (one ASCII line per sample, 115200 8N1 on USART2 / ST-LINK VCP):
  *
  *   $WS,<station>,<seq>,<temp_cC>,<hum_c%>,<press_Pa>,<temp2_cC>,<status>*<XOR>\r\n
  *
  *   station   STATION_ID below, so several stations can share one receiver
  *   seq       per-station sample counter, lets the receiver spot lost frames
  *   temp_cC   HTS221 temperature in centi-degC   (2351  -> 23.51 C)
  *   hum_c%    HTS221 humidity in centi-percent   (4512  -> 45.12 %RH)
  *   press_Pa  LPS22HH pressure in pascal         (101325 -> 1013.25 hPa)
  *   temp2_cC  STTS751 temperature in centi-degC
  *   status    bit0 HTS221 fault, bit1 LPS22HH fault, bit2 STTS751 fault
  *   XOR       two hex digits, XOR of every byte between '$' and '*'
  *
  * Lines starting with '#' are informational and carry no data.
  *
  * Integers are sent instead of floats on purpose: newlib-nano printf has no
  * %f without extra linker flags, and fixed-point keeps frames short and exact.
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t seq;          /* increments every sample, lets the receiver spot drops */
  int32_t  temp_cC;      /* HTS221 temperature, 0.01 degC */
  int32_t  hum_cPct;     /* HTS221 relative humidity, 0.01 %RH */
  int32_t  press_Pa;     /* LPS22HH pressure, Pa */
  int32_t  temp2_cC;     /* STTS751 temperature, 0.01 degC */
  uint8_t  status;       /* SENSOR_FAULT_* bits */
} WeatherSample;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Station number in every frame. Override per board, e.g. -DSTATION_ID=2 */
#ifndef STATION_ID
#define STATION_ID 1u
#endif

/* Task Stack Size (in CPU_STK words, 4 bytes each) */
#define APP_TASK_START_STK_SIZE 128u
#define UART_SEND_STK_SIZE      512u   /* snprintf needs headroom */
#define SENSOR_READ_STK_SIZE    384u
/* Task Priority: the lower the number, the higher the priority */
#define APP_TASK_START_PRIO 1u
#define UART_SEND_PRIO      2u
#define SENSOR_READ_PRIO    3u

#define SAMPLE_PERIOD_MS  2000u
#define SAMPLE_POOL_SIZE  8u     /* must exceed UART_QUEUE_SIZE so a slot is
                                    never reused while still queued */
#define UART_QUEUE_SIZE   4u
#define FRAME_MAX         64u
#define UART_TIMEOUT_MS   100u

#define SENSOR_FAULT_HTS221   (1u << 0)
#define SENSOR_FAULT_LPS22HH  (1u << 1)
#define SENSOR_FAULT_STTS751  (1u << 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* huart2 lives in usart.c. It used to be defined here as well, which links
 * only with the legacy -fcommon behaviour (GCC 10+ rejects it). */

/* Task Control Block */
static OS_TCB AppTaskStartTCB;
static OS_TCB UartSendTCB;
static OS_TCB SensorReadTCB;
/* Task Stack */
static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static CPU_STK UartSendStk[UART_SEND_STK_SIZE];
static CPU_STK SensorReadStk[SENSOR_READ_STK_SIZE];

static WeatherSample samplePool[SAMPLE_POOL_SIZE];
static volatile uint8_t sensorFaults;  /* last status, drives the LED */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void AppTaskStart(void *p_arg);
static void UartSend(void *p_arg);
static void SensorRead(void *p_arg);
static void CreateTask(OS_TCB *tcb, CPU_CHAR *name, OS_TASK_PTR fn, OS_PRIO prio,
                       CPU_STK *stk, CPU_STK_SIZE stk_size, OS_MSG_QTY q_size);
static uint8_t SensorInit(uint32_t instance, uint32_t functions);
static uint8_t SensorReadCenti(uint32_t instance, uint32_t function, int32_t *out);
static uint16_t FormatFrame(const WeatherSample *s, char *buf, uint16_t len);
static void UartWrite(const char *buf, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int main(void)
{
  OS_ERR os_err;

  OSInit(&os_err);
  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  CreateTask(&AppTaskStartTCB, (CPU_CHAR *)"App Task Start", AppTaskStart,
             APP_TASK_START_PRIO, AppTaskStartStk, APP_TASK_START_STK_SIZE, 0u);

  OSStart(&os_err);  /* never returns */
  while (DEF_TRUE)
    ;
}

/*
 * HAL timebase on top of the RTOS tick.
 *
 * The vector table sends SysTick straight to OS_CPU_SysTickHandler, so the
 * HAL's own SysTick_Handler (stm32l1xx_it.c) never runs and HAL_IncTick() is
 * never called. In the original code that froze HAL_GetTick() at 0, so the
 * HAL_Delay(100) in the send loop never returned. The OS tick runs at 1 kHz
 * (OS_CFG_TICK_RATE_HZ), so it can stand in for the HAL millisecond counter.
 *
 * SysTick itself is still programmed by HAL_InitTick() during HAL_Init() and
 * SystemClock_Config(), at 1 kHz from the current core clock.
 */
uint32_t HAL_GetTick(void)
{
  OS_ERR os_err;
  return (uint32_t)OSTimeGet(&os_err);
}

void HAL_Delay(uint32_t Delay)
{
  OS_ERR os_err;

  if (Delay == 0u)
  {
    return;
  }
  /* Yield to other tasks instead of spinning... */
  OSTimeDly((OS_TICK)Delay, OS_OPT_TIME_DLY, &os_err);
  if (os_err != OS_ERR_NONE)
  {
    /* ...unless we're in an ISR or before OSStart(): fall back to busy-wait. */
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < Delay)
    {
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief System Clock Configuration: HSI 16 MHz -> PLL x6 /3 -> 32 MHz SYSCLK
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void CreateTask(OS_TCB *tcb, CPU_CHAR *name, OS_TASK_PTR fn, OS_PRIO prio,
                       CPU_STK *stk, CPU_STK_SIZE stk_size, OS_MSG_QTY q_size)
{
  OS_ERR os_err;

  OSTaskCreate(tcb, name, fn, (void *)0, prio,
               stk,
               stk_size / 10u,       /* stack limit watermark: 10 % */
               stk_size,
               q_size,               /* built-in task message queue depth */
               (OS_TICK)0u,          /* no round-robin time quanta */
               (void *)0,
               (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &os_err);

  if (os_err != OS_ERR_NONE)
  {
    Error_Handler();
  }
}

/*
 * Board bring-up happens exactly once, here. The original UartSend task
 * re-ran HAL_Init() / SystemClock_Config() / MX_USART2_UART_Init() while the
 * scheduler was live, reprogramming SysTick and the UART behind the other
 * tasks' backs.
 *
 * I2C1 is not initialised here: the IKS01A3 BSP calls BSP_I2C1_Init() itself
 * when a sensor is probed. The old MX_I2C1_Init(hi2c) call passed an
 * uninitialised pointer and wrote the I2C handle to a random address.
 */
static void AppTaskStart(void *p_arg)
{
  OS_ERR os_err;
  (void)p_arg;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Consumer first, so its queue exists before the producer posts. */
  CreateTask(&UartSendTCB, (CPU_CHAR *)"Uart send", UartSend,
             UART_SEND_PRIO, UartSendStk, UART_SEND_STK_SIZE, UART_QUEUE_SIZE);
  CreateTask(&SensorReadTCB, (CPU_CHAR *)"Sensor read", SensorRead,
             SENSOR_READ_PRIO, SensorReadStk, SENSOR_READ_STK_SIZE, 0u);

  /* Heartbeat: slow blink = healthy, fast blink = a sensor is faulted. */
  while (DEF_TRUE)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    OSTimeDly((sensorFaults != 0u) ? 100u : 500u, OS_OPT_TIME_DLY, &os_err);
  }
}

static void UartSend(void *p_arg)
{
  static const char banner[] = "# weather-station up, frame $WS,id,seq,t,h,p,t2,st*cs\r\n";
  char frame[FRAME_MAX];
  OS_ERR os_err;
  OS_MSG_SIZE size;
  (void)p_arg;

  UartWrite(banner, (uint16_t)(sizeof(banner) - 1u));

  while (DEF_TRUE)
  {
    WeatherSample *msg = (WeatherSample *)OSTaskQPend(0u, OS_OPT_PEND_BLOCKING,
                                                      &size, (CPU_TS *)0, &os_err);
    if ((os_err != OS_ERR_NONE) || (msg == NULL))
    {
      continue;
    }

    WeatherSample sample = *msg;  /* copy out before the slot can be reused */
    uint16_t len = FormatFrame(&sample, frame, sizeof(frame));
    if (len > 0u)
    {
      /* Send exactly the formatted bytes. The original sent sizeof(MSG) = 20
       * bytes every time: the text plus NULs and leftovers from the previous,
       * longer message, which showed up as garbage in the PC logs. */
      UartWrite(frame, len);
    }
  }
}

static void SensorRead(void *p_arg)
{
  OS_ERR os_err;
  uint32_t seq = 0u;
  uint8_t slot = 0u;
  uint8_t initFaults = 0u;
  (void)p_arg;

  if (!SensorInit(IKS01A3_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY))
  {
    initFaults |= SENSOR_FAULT_HTS221;
  }
  if (!SensorInit(IKS01A3_LPS22HH_0, ENV_PRESSURE))
  {
    initFaults |= SENSOR_FAULT_LPS22HH;
  }
  if (!SensorInit(IKS01A3_STTS751_0, ENV_TEMPERATURE))
  {
    initFaults |= SENSOR_FAULT_STTS751;
  }

  while (DEF_TRUE)
  {
    WeatherSample *s = &samplePool[slot];
    slot = (uint8_t)((slot + 1u) % SAMPLE_POOL_SIZE);

    memset(s, 0, sizeof(*s));
    s->seq = seq++;
    s->status = initFaults;

    if ((s->status & SENSOR_FAULT_HTS221) == 0u)
    {
      if (!SensorReadCenti(IKS01A3_HTS221_0, ENV_TEMPERATURE, &s->temp_cC) ||
          !SensorReadCenti(IKS01A3_HTS221_0, ENV_HUMIDITY, &s->hum_cPct))
      {
        s->status |= SENSOR_FAULT_HTS221;
      }
    }
    if ((s->status & SENSOR_FAULT_LPS22HH) == 0u)
    {
      /* BSP returns hPa; x100 gives Pa */
      if (!SensorReadCenti(IKS01A3_LPS22HH_0, ENV_PRESSURE, &s->press_Pa))
      {
        s->status |= SENSOR_FAULT_LPS22HH;
      }
    }
    if ((s->status & SENSOR_FAULT_STTS751) == 0u)
    {
      if (!SensorReadCenti(IKS01A3_STTS751_0, ENV_TEMPERATURE, &s->temp2_cC))
      {
        s->status |= SENSOR_FAULT_STTS751;
      }
    }

    sensorFaults = s->status;
    /* If the queue is full the sample is dropped; the receiver sees a seq gap. */
    OSTaskQPost(&UartSendTCB, s, (OS_MSG_SIZE)sizeof(*s), OS_OPT_POST_FIFO, &os_err);

    /* Periodic delay: the 2 s cadence does not drift with read time. */
    OSTimeDly(SAMPLE_PERIOD_MS, OS_OPT_TIME_PERIODIC, &os_err);
  }
}

static uint8_t SensorInit(uint32_t instance, uint32_t functions)
{
  uint32_t f;

  if (IKS01A3_ENV_SENSOR_Init(instance, functions) != BSP_ERROR_NONE)
  {
    return 0u;
  }
  /* Init only probes the part; each function must be enabled separately. */
  for (f = ENV_TEMPERATURE; f <= ENV_HUMIDITY; f <<= 1)
  {
    if (((functions & f) != 0u) &&
        (IKS01A3_ENV_SENSOR_Enable(instance, f) != BSP_ERROR_NONE))
    {
      return 0u;
    }
  }
  return 1u;
}

/*
 * The BSP hands back a float. The original code passed &data where data was
 * an int, so the float's raw IEEE-754 bits were printed as an integer:
 * 23.5 C came out as 1102839808. That was the gibberish reading.
 */
static uint8_t SensorReadCenti(uint32_t instance, uint32_t function, int32_t *out)
{
  float value;

  if (IKS01A3_ENV_SENSOR_GetValue(instance, function, &value) != BSP_ERROR_NONE)
  {
    return 0u;
  }
  value *= 100.0f;
  *out = (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
  return 1u;
}

static uint16_t FormatFrame(const WeatherSample *s, char *buf, uint16_t len)
{
  uint8_t checksum = 0u;
  int i;
  int n = snprintf(buf, len, "$WS,%u,%lu,%ld,%ld,%ld,%ld,%u",
                   (unsigned)STATION_ID,
                   (unsigned long)s->seq,
                   (long)s->temp_cC,
                   (long)s->hum_cPct,
                   (long)s->press_Pa,
                   (long)s->temp2_cC,
                   (unsigned)s->status);

  if ((n <= 0) || ((n + 6) > (int)len))  /* need room for "*XX\r\n\0" */
  {
    return 0u;
  }
  for (i = 1; i < n; i++)
  {
    checksum ^= (uint8_t)buf[i];
  }
  n += snprintf(&buf[n], len - (uint16_t)n, "*%02X\r\n", checksum);
  return (uint16_t)n;
}

static void UartWrite(const char *buf, uint16_t len)
{
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, UART_TIMEOUT_MS);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
