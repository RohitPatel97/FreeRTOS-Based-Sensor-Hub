/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos_sensor_hub.h
  * @brief   FreeRTOS task/queue/semaphore layer for the sensor hub project.
  ******************************************************************************
  */
/* USER CODE END Header */

#pragma once
#ifndef FREERTOS_SENSOR_HUB_H
#define FREERTOS_SENSOR_HUB_H

#include "stm32f4xx_hal.h"
#include "sensor_hw.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FSH_QUEUE_RAW_DEPTH       4U
#define FSH_QUEUE_FILTERED_DEPTH  1U

#define FSH_HEALTH_SENSOR_TASK    0x01U
#define FSH_HEALTH_FILTER_TASK    0x02U
#define FSH_HEALTH_UART_TASK      0x04U
#define FSH_HEALTH_ERROR_TASK     0x08U
#define FSH_HEALTH_CRITICAL_MASK  (FSH_HEALTH_SENSOR_TASK | FSH_HEALTH_FILTER_TASK | FSH_HEALTH_UART_TASK)

#define FSH_FAULT_TASK_STALL      0x00000001UL
#define FSH_FAULT_SENSOR_MISSING  0x00000002UL
#define FSH_FAULT_QUEUE_DROP      0x00000004UL

typedef struct {
    uint32_t sensor_read_us;
    uint32_t sensor_read_max_us;
    uint32_t filter_us;
    uint32_t filter_max_us;
    uint32_t uart_us;
    uint32_t uart_max_us;
    uint32_t error_monitor_us;
    uint32_t error_monitor_max_us;
    uint32_t sample_ticks;
    uint32_t telemetry_ticks;
    uint32_t raw_queue_drops;
    uint32_t filtered_queue_drops;
    uint32_t watchdog_refreshes;
    uint32_t health_mask_last;
    uint32_t fault_flags;
} FSH_RtosStatus_t;

HAL_StatusTypeDef FSH_RTOS_Init(UART_HandleTypeDef *huart2,
                                I2C_HandleTypeDef *hi2c1,
                                IWDG_HandleTypeDef *hiwdg,
                                TIM_HandleTypeDef *htim2);

void FSH_OnTim2TickFromISR(void);
const FSH_RtosStatus_t* FSH_RTOS_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_SENSOR_HUB_H */
