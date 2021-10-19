/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Task Stack Size */
#define APP_TASK_START_STK_SIZE 128u
#define UART_SEND_STK_SIZE 256u
#define SENSOR_READ_STK_SIZE 256u
/* Task Priority */
#define APP_TASK_START_PRIO 1u
#define UART_SEND_PRIO 2u
#define SENSOR_READ_PRIO 3u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
int data;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//float data;
static const uint8_t STTS751_ADDR = 0x94 << 1; //Use 8-bit address
static const uint8_t REG_TEMP = 0x00;

/* Task Control Block */
static OS_TCB AppTaskStartTCB;
static OS_TCB UartSendTCB;
static OS_TCB SensorReadTCB;
/* Task Stack */
static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static CPU_STK UartSendStk[UART_SEND_STK_SIZE];
static CPU_STK SensorReadStk[SENSOR_READ_STK_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void AppTaskStart(void *p_arg);
static void UartSend(void *p_arg);
static void SensorRead(void *p_arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

	/* To store error code */
  OS_ERR os_err;

  /* Initialize uC/OS-III */
  OSInit(&os_err);

  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  OSTaskCreate(
      /* pointer to task control block */
      (OS_TCB *)&AppTaskStartTCB,
      /* task name can be displayed by debuggers */
      (CPU_CHAR *)"App Task Start",
      /* pointer to the task */
      (OS_TASK_PTR)AppTaskStart,
      /* pointer to an OPTIONAL data area */
      (void *)0,
      /* task priority: the lower the number, the higher the priority */
      (OS_PRIO)APP_TASK_START_PRIO,
      /* pointer to task's stack base addr */
      (CPU_STK *)&AppTaskStartStk[0],
      /* task's stack limit to monitor and ensure that the stack 
       * doesn't overflow (10%) */
      (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
      /* task's stack size */
      (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
      /* max number of message that the task can receive through 
       * internal message queue (5) */
      (OS_MSG_QTY)5u,
      /* amount of clock ticks for the time quanta 
       * when round robin is enabled */
      (OS_TICK)0u,
      /* pointer to an OPTIONAL user-supplied memory location 
       * use as a TCB extension */
      (void *)0,
      /* contain task-specific option 
       * OS_OPT_TASK_STK_CHK: allow stack checking 
       * OS_OPT_TASK_STK_CLR: stack needs to be cleared */
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      /* pointer to a variable that will receive an error code */
      (OS_ERR *)&os_err);

    

  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  /* Start Mulitasking */
  OSStart(&os_err);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/**
  * @brief System Clock Configuration
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
static void AppTaskStart(void *p_arg)
{
  OS_ERR os_err;
  I2C_HandleTypeDef* hi2c;
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init(hi2c);
  
  OSTaskCreate(
      /* pointer to task control block */
      (OS_TCB *)&UartSendTCB,
      /* task name can be displayed by debuggers */
      (CPU_CHAR *)"Uart send",
      /* pointer to the task */
      (OS_TASK_PTR)UartSend,
      /* pointer to an OPTIONAL data area */
      (void *)0,
      /* task priority: the lower the number, the higher the priority */
      (OS_PRIO)UART_SEND_PRIO,
      /* pointer to task's stack base addr */
      (CPU_STK *)&UartSendStk[0],
      /* task's stack limit to monitor and ensure that the stack 
       * doesn't overflow (10%) */
      (CPU_STK_SIZE)UART_SEND_STK_SIZE / 10,
      /* task's stack size */
      (CPU_STK_SIZE)UART_SEND_STK_SIZE,
      /* max number of message that the task can receive through 
       * internal message queue (5) */
      (OS_MSG_QTY)5u,
      /* amount of clock ticks for the time quanta 
       * when round robin is enabled */
      (OS_TICK)0u,
      /* pointer to an OPTIONAL user-supplied memory location 
       * use as a TCB extension */
      (void *)0,
      /* contain task-specific option 
       * OS_OPT_TASK_STK_CHK: allow stack checking 
       * OS_OPT_TASK_STK_CLR: stack needs to be cleared */
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      /* pointer to a variable that will receive an error code */
      (OS_ERR *)&os_err);

     OSTaskCreate(
      /* pointer to task control block */
      (OS_TCB *)&SensorReadTCB,
      /* task name can be displayed by debuggers */
      (CPU_CHAR *)"Sensor read",
      /* pointer to the task */
      (OS_TASK_PTR)SensorRead,
      /* pointer to an OPTIONAL data area */
      (void *)0,
      /* task priority: the lower the number, the higher the priority */
      (OS_PRIO)SENSOR_READ_PRIO,
      /* pointer to task's stack base addr */
      (CPU_STK *)&SensorReadStk[0],
      /* task's stack limit to monitor and ensure that the stack 
       * doesn't overflow (10%) */
      (CPU_STK_SIZE)SENSOR_READ_STK_SIZE / 10,
      /* task's stack size */
      (CPU_STK_SIZE)SENSOR_READ_STK_SIZE,
      /* max number of message that the task can receive through 
       * internal message queue (5) */
      (OS_MSG_QTY)5u,
      /* amount of clock ticks for the time quanta 
       * when round robin is enabled */
      (OS_TICK)0u,
      /* pointer to an OPTIONAL user-supplied memory location 
       * use as a TCB extension */
      (void *)0,
      /* contain task-specific option 
       * OS_OPT_TASK_STK_CHK: allow stack checking 
       * OS_OPT_TASK_STK_CLR: stack needs to be cleared */
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      /* pointer to a variable that will receive an error code */
      (OS_ERR *)&os_err);
    
  while (DEF_TRUE)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

static void UartSend(void *p_arg)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  OS_ERR os_err;
  //unsigned char MSG[] = "\rTest message \n\r";
  unsigned char MSG[20] = "";
  unsigned char MSG1[] = "\rSensor checked\n\r";

if (IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE)==HAL_OK) {
	IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_TEMPERATURE);
}
  while (DEF_TRUE)
  {
  //HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
  // OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
   IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &data);
	 HAL_Delay(100);
   
	  sprintf(MSG, "\n%d\n\r", data);
	
	  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	  OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

static void SensorRead(void *p_arg)
{
  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
