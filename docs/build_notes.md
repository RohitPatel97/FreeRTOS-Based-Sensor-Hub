# CubeIDE / CubeMX Build Notes

These are the settings I would double-check before building the project.

## CubeMX peripherals

- Board: NUCLEO-F401RE
- USART2 asynchronous, 115200 baud, 8N1
- I2C1 on PB8/PB9, 400 kHz fast mode
- BMP280 address probing supports both `0x76` and `0x77`
- TIM2 base timer update interrupt at 100 Hz
- IWDG enabled, about 4 second timeout
- FreeRTOS enabled using CMSIS-RTOS v2

## Important FreeRTOS interrupt note

TIM2 releases an RTOS semaphore from its interrupt callback, so its NVIC priority must be safe for FreeRTOS API calls from ISR.

In the included MSP file I used:

```c
HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
```

That assumes the common CubeMX setting:

```c
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
```

If your FreeRTOS config is different, keep TIM2 at a numerically equal or larger priority value than the syscall threshold.

Also keep the HAL timebase valid after the scheduler starts. This firmware uses `HAL_GetTick()` for telemetry timestamps and `HAL_Delay()` inside I2C retry/recovery paths, so a CubeMX FreeRTOS project should either use the generated FreeRTOS-aware SysTick handler or move the HAL timebase to a separate timer such as TIM6/TIM7.

## File placement

Copy these into the CubeIDE project:

```text
Core/Inc/sensor_hw.h
Core/Inc/freertos_sensor_hub.h
Core/Src/sensor_hw.c
Core/Src/freertos_sensor_hub.c
Core/Src/main.c
Core/Src/stm32f4xx_hal_msp.c
```

For `stm32f4xx_it.c`, merge the TIM2 handler into your CubeMX-generated file if CubeMX already generated FreeRTOS SVC/PendSV/SysTick handlers.
