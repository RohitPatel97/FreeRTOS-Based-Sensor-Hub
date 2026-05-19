/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt handlers for FreeRTOS sensor hub.
  ******************************************************************************
  * Note:
  *   In a CubeMX FreeRTOS project, SVC/PendSV/SysTick are normally owned by
  *   the FreeRTOS port/CubeMX generated code. Keep those generated handlers
  *   unless your Cube project tells you to merge them manually.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;

void NMI_Handler(void)               { while (1) {} }
void HardFault_Handler(void)         { while (1) {} }
void MemManage_Handler(void)         { while (1) {} }
void BusFault_Handler(void)          { while (1) {} }
void UsageFault_Handler(void)        { while (1) {} }
void DebugMon_Handler(void)          { }

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}
