/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sensor_hw.h
  * @brief   Blocking sensor driver layer used by the FreeRTOS sensor hub.
  ******************************************************************************
  *
  * Target:
  *   - Board: NUCLEO-F401RE
  *   - Sensors: MPU6050 + BMP280 on I2C1 PB8/PB9
  *
  * This layer intentionally does not create tasks.  It just owns the hardware
  * reads, retry/error counters, fault detection, and I2C recovery.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#pragma once
#ifndef SENSOR_HW_H
#define SENSOR_HW_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FSH_SAMPLE_RATE_HZ       100U
#define FSH_TELEMETRY_RATE_HZ    10U
#define FSH_MA_WINDOW            8U
#define FSH_MAX_RETRIES          3U

#define FSH_I2C_TIMEOUT_MS       10U
#define FSH_UART_TIMEOUT_MS      50U

#define FSH_MPU6050_ADDR7        0x68U
#define FSH_BMP280_ADDR7         0x76U  /* primary address; driver also probes 0x77 */
#define FSH_BMP280_ADDR7_ALT     0x77U

#define FSH_VALID_MPU6050        0x01U
#define FSH_VALID_BMP280         0x02U

typedef struct {
    float accel_x, accel_y, accel_z;       /* g */
    float gyro_x,  gyro_y,  gyro_z;        /* deg/sec */
    float temperature_c;                   /* deg C from BMP280 when valid */
    float pressure_hpa;                    /* hPa */
    uint32_t timestamp_ms;
    uint8_t valid_mask;
} FSH_SensorSample_t;

typedef struct {
    uint32_t samples_taken;
    uint32_t comm_errors;
    uint32_t retries;
    uint32_t mpu_failures;
    uint32_t bmp_failures;
    uint32_t i2c_recoveries;
    uint8_t mpu_ready;
    uint8_t bmp_ready;
} FSH_HwStatus_t;

HAL_StatusTypeDef FSH_HW_Init(I2C_HandleTypeDef *hi2c1);
uint8_t FSH_HW_ReadSensors(FSH_SensorSample_t *out);
const FSH_HwStatus_t* FSH_HW_GetStatus(void);
void FSH_HW_I2CRecover(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_HW_H */
