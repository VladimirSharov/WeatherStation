# 2021 three-board firmware (original)

The master and slave firmware of the 2021 build, unmodified, kept for history. Each folder is a complete PlatformIO project with its own `.vscode`. Both still build with `pio run`.

| Folder | Board | Snapshot |
|---|---|---|
| [`slave/`](slave/) | Sensor station: reads an LM35 temperature sensor and sends the value to the master | Oct 28, 2021 |
| [`master/`](master/) | Central board: receives from the slave, shows it on a 16×2 LCD, forwards to the PC / Pi over USB | Nov 19, 2021 |

These are intermediate snapshots. They send temperature only. The humidity readings and the `T1 / H1 / T2 / H2` line format that [`../2021-pi/readdata.py`](../2021-pi/readdata.py) parsed came from a later version that hasn't survived.

## How it worked

- **Sensor:** an LM35 analog temperature sensor on ADC channel 7 (PA7). It outputs 10 mV per °C, so with a 3.3 V reference `temp = raw / 4096 * 330`.
- **Slave → master link:** USART3 (PB10 TX, PB11 RX) at 115200 baud, wired directly between the boards.
- **Display:** an HD44780-style 16×2 LCD in 4-bit mode: D4–D7 on PB12–PB15, RS on PB3, EN on PB4. The driver, Khaled Magdy's LCD16x2 library, times its pulses with the Cortex-M3 DWT cycle counter.
- **To the PC / Pi:** USART2 through the on-board ST-LINK USB port.
- **RTOS:** Micrium uC/OS-III, with a semaphore pair handing control between the worker task and an LED blink task.

Both projects started from a course template ("03-semaphore"). That's where the project file name, the unused BMP280 and soil-moisture drivers, and the `03-*.png` images come from. The images show the template exercise, not the weather station. The station's own wiring diagrams are in [`../../docs/hardware/`](../../docs/hardware/).

## Why the LCD showed gibberish

1. **The master read misaligned chunks.** The slave sent each reading as 3 bytes (`"23\0"`) with no start or end marker. The master asked for exactly 13 bytes with a 100 ms timeout. That timeout never fired: SysTick is routed straight to the RTOS, so `HAL_GetTick()` never advances, the same bug as in the prototype firmware. The master therefore blocked until 13 bytes had piled up, roughly four or five readings cut at an arbitrary point (`"3\023\023\023\023"`). The LCD showed "3", "23" or nothing, depending on the cut. The line was never cleared, so shorter values left old digits behind.
2. **Raw buffers went to the PC.** The master forwarded its whole 13-byte receive buffer, NULs and leftovers included, to the USB serial port. The slave sent its full 16-byte debug buffer the same way. That's the junk in the PC log.
3. **Buffer overflows.** The slave's send buffer `DATA_TO_SEND[]="xx"` only fits two digits. A disconnected or noisy sensor input can read up to 330 °C, and a three-digit value overflows into neighbouring memory. The master also copied the received bytes with an unbounded `strcpy`.

The 2026 rewrite avoids all of this with newline-terminated frames, an explicit length, a checksum and a working HAL timebase. See the [main README](../../README.md).
