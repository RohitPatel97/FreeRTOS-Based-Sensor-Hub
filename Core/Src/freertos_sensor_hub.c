/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos_sensor_hub.c
  * @brief   RTOS task pipeline for the STM32 sensor hub.
  ******************************************************************************
  *
  * Task layout:
  *   - SensorReadTask: waits on TIM2 semaphore, reads I2C sensors at 100 Hz
  *   - FilterTask: consumes raw samples and applies moving average filtering
  *   - UartTxTask: sends latest filtered telemetry at 10 Hz
  *   - ErrorMonitorTask: watches task health, faults, LED, and IWDG refresh
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "freertos_sensor_hub.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* =================== LOCAL TYPES =================== */
typedef struct {
    float buf[FSH_MA_WINDOW];
    uint16_t idx;
    uint16_t count;
    float sum;
} MA_t;

/* =================== HANDLES =================== */
static UART_HandleTypeDef *g_uart = NULL;
static I2C_HandleTypeDef  *g_i2c = NULL;
static IWDG_HandleTypeDef *g_iwdg = NULL;
static TIM_HandleTypeDef  *g_tim2 = NULL;

static osThreadId_t g_sensorTaskHandle = NULL;
static osThreadId_t g_filterTaskHandle = NULL;
static osThreadId_t g_uartTaskHandle = NULL;
static osThreadId_t g_errorTaskHandle = NULL;

static osMessageQueueId_t g_rawQueue = NULL;
static osMessageQueueId_t g_filteredQueue = NULL;
static osSemaphoreId_t g_sampleSem = NULL;
static osSemaphoreId_t g_telemetrySem = NULL;
static osMutexId_t g_i2cMutex = NULL;
static osMutexId_t g_latestMutex = NULL;

/* =================== STATE =================== */
static volatile uint32_t g_taskHeartbeat = 0U;
static volatile uint8_t g_timerStarted = 0U;
static FSH_RtosStatus_t g_rtos = {0};
static FSH_SensorSample_t g_latest = {0};

static MA_t g_ma_ax = {0};
static MA_t g_ma_ay = {0};
static MA_t g_ma_az = {0};

/* =================== FORWARD =================== */
static void SensorReadTask(void *argument);
static void FilteringTask(void *argument);
static void UartTxTask(void *argument);
static void ErrorMonitorTask(void *argument);

static void ma_update(MA_t *ma, float x);
static float ma_get(const MA_t *ma);
static uint8_t queue_put_latest(osMessageQueueId_t q, const void *item, uint32_t *drop_counter);

static void timing_init(void);
static uint32_t timing_start(void);
static uint32_t timing_elapsed_us(uint32_t start_cycles);
static void mark_task_alive(uint32_t bit);
static int32_t scale_i32(float value, float scale);

/* =================== PUBLIC API =================== */
HAL_StatusTypeDef FSH_RTOS_Init(UART_HandleTypeDef *huart2,
                                I2C_HandleTypeDef *hi2c1,
                                IWDG_HandleTypeDef *hiwdg,
                                TIM_HandleTypeDef *htim2)
{
    if (huart2 == NULL || hi2c1 == NULL || hiwdg == NULL || htim2 == NULL) {
        return HAL_ERROR;
    }

    g_uart = huart2;
    g_i2c = hi2c1;
    g_iwdg = hiwdg;
    g_tim2 = htim2;

    memset(&g_rtos, 0, sizeof(g_rtos));
    memset(&g_latest, 0, sizeof(g_latest));
    memset(&g_ma_ax, 0, sizeof(g_ma_ax));
    memset(&g_ma_ay, 0, sizeof(g_ma_ay));
    memset(&g_ma_az, 0, sizeof(g_ma_az));

    timing_init();

    if (FSH_HW_Init(g_i2c) != HAL_OK) {
        return HAL_ERROR;
    }

    g_rawQueue = osMessageQueueNew(FSH_QUEUE_RAW_DEPTH, sizeof(FSH_SensorSample_t), NULL);
    g_filteredQueue = osMessageQueueNew(FSH_QUEUE_FILTERED_DEPTH, sizeof(FSH_SensorSample_t), NULL);
    g_sampleSem = osSemaphoreNew(8U, 0U, NULL);
    g_telemetrySem = osSemaphoreNew(4U, 0U, NULL);
    g_i2cMutex = osMutexNew(NULL);
    g_latestMutex = osMutexNew(NULL);

    if (g_rawQueue == NULL || g_filteredQueue == NULL ||
        g_sampleSem == NULL || g_telemetrySem == NULL ||
        g_i2cMutex == NULL || g_latestMutex == NULL) {
        return HAL_ERROR;
    }

    const osThreadAttr_t sensorTaskAttr = {
        .name = "SensorReadTask",
        .priority = osPriorityHigh,
        .stack_size = 512U * 4U
    };
    const osThreadAttr_t filterTaskAttr = {
        .name = "FilteringTask",
        .priority = osPriorityAboveNormal,
        .stack_size = 384U * 4U
    };
    const osThreadAttr_t uartTaskAttr = {
        .name = "UartTxTask",
        .priority = osPriorityNormal,
        .stack_size = 512U * 4U
    };
    const osThreadAttr_t errorTaskAttr = {
        .name = "ErrorMonitorTask",
        .priority = osPriorityLow,
        .stack_size = 384U * 4U
    };

    g_sensorTaskHandle = osThreadNew(SensorReadTask, NULL, &sensorTaskAttr);
    g_filterTaskHandle = osThreadNew(FilteringTask, NULL, &filterTaskAttr);
    g_uartTaskHandle = osThreadNew(UartTxTask, NULL, &uartTaskAttr);
    g_errorTaskHandle = osThreadNew(ErrorMonitorTask, NULL, &errorTaskAttr);

    if (g_sensorTaskHandle == NULL || g_filterTaskHandle == NULL ||
        g_uartTaskHandle == NULL || g_errorTaskHandle == NULL) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void FSH_OnTim2TickFromISR(void)
{
    static uint16_t telemetry_div = 0U;

    if (g_sampleSem != NULL) {
        if (osSemaphoreRelease(g_sampleSem) == osOK) {
            g_rtos.sample_ticks++;
        } else {
            g_rtos.raw_queue_drops++;
            g_rtos.fault_flags |= FSH_FAULT_QUEUE_DROP;
        }
    }

    telemetry_div++;
    if (telemetry_div >= (FSH_SAMPLE_RATE_HZ / FSH_TELEMETRY_RATE_HZ)) {
        telemetry_div = 0U;
        if (g_telemetrySem != NULL) {
            if (osSemaphoreRelease(g_telemetrySem) == osOK) {
                g_rtos.telemetry_ticks++;
            }
        }
    }
}

const FSH_RtosStatus_t* FSH_RTOS_GetStatus(void)
{
    return &g_rtos;
}

/* =================== TASKS =================== */
static void SensorReadTask(void *argument)
{
    (void)argument;

    /* Start TIM2 from task context so the semaphore objects exist first. */
    if (!g_timerStarted && g_tim2 != NULL) {
        if (HAL_TIM_Base_Start_IT(g_tim2) == HAL_OK) {
            g_timerStarted = 1U;
        }
    }

    for (;;) {
        if (osSemaphoreAcquire(g_sampleSem, osWaitForever) == osOK) {
            const uint32_t t0 = timing_start();
            FSH_SensorSample_t raw;

            if (osMutexAcquire(g_i2cMutex, osWaitForever) == osOK) {
                (void)FSH_HW_ReadSensors(&raw);
                (void)osMutexRelease(g_i2cMutex);
            } else {
                memset(&raw, 0, sizeof(raw));
                raw.timestamp_ms = HAL_GetTick();
                g_rtos.fault_flags |= FSH_FAULT_SENSOR_MISSING;
            }

            if ((raw.valid_mask & (FSH_VALID_MPU6050 | FSH_VALID_BMP280)) !=
                (FSH_VALID_MPU6050 | FSH_VALID_BMP280)) {
                g_rtos.fault_flags |= FSH_FAULT_SENSOR_MISSING;
            }

            (void)queue_put_latest(g_rawQueue, &raw, &g_rtos.raw_queue_drops);

            g_rtos.sensor_read_us = timing_elapsed_us(t0);
            if (g_rtos.sensor_read_us > g_rtos.sensor_read_max_us) {
                g_rtos.sensor_read_max_us = g_rtos.sensor_read_us;
            }
            mark_task_alive(FSH_HEALTH_SENSOR_TASK);
        }
    }
}

static void FilteringTask(void *argument)
{
    (void)argument;

    for (;;) {
        FSH_SensorSample_t raw;
        if (osMessageQueueGet(g_rawQueue, &raw, NULL, osWaitForever) == osOK) {
            const uint32_t t0 = timing_start();
            FSH_SensorSample_t filtered = raw;

            if ((raw.valid_mask & FSH_VALID_MPU6050) != 0U) {
                ma_update(&g_ma_ax, raw.accel_x);
                ma_update(&g_ma_ay, raw.accel_y);
                ma_update(&g_ma_az, raw.accel_z);

                filtered.accel_x = ma_get(&g_ma_ax);
                filtered.accel_y = ma_get(&g_ma_ay);
                filtered.accel_z = ma_get(&g_ma_az);
            }

            (void)queue_put_latest(g_filteredQueue, &filtered, &g_rtos.filtered_queue_drops);

            if (osMutexAcquire(g_latestMutex, 0U) == osOK) {
                g_latest = filtered;
                (void)osMutexRelease(g_latestMutex);
            }

            g_rtos.filter_us = timing_elapsed_us(t0);
            if (g_rtos.filter_us > g_rtos.filter_max_us) {
                g_rtos.filter_max_us = g_rtos.filter_us;
            }
            mark_task_alive(FSH_HEALTH_FILTER_TASK);
        }
    }
}

static void UartTxTask(void *argument)
{
    (void)argument;
    uint8_t buf[320];
    FSH_SensorSample_t latest = {0};

    for (;;) {
        if (osSemaphoreAcquire(g_telemetrySem, osWaitForever) == osOK) {
            const uint32_t t0 = timing_start();

            FSH_SensorSample_t tmp;
            while (osMessageQueueGet(g_filteredQueue, &tmp, NULL, 0U) == osOK) {
                latest = tmp;
            }

            if (osMutexAcquire(g_latestMutex, 0U) == osOK) {
                latest = g_latest;
                (void)osMutexRelease(g_latestMutex);
            }

            const FSH_HwStatus_t *hw = FSH_HW_GetStatus();

            const int32_t ax_mg = scale_i32(latest.accel_x, 1000.0f);
            const int32_t ay_mg = scale_i32(latest.accel_y, 1000.0f);
            const int32_t az_mg = scale_i32(latest.accel_z, 1000.0f);
            const int32_t gx_cdps = scale_i32(latest.gyro_x, 100.0f);
            const int32_t gy_cdps = scale_i32(latest.gyro_y, 100.0f);
            const int32_t gz_cdps = scale_i32(latest.gyro_z, 100.0f);
            const int32_t t_cC = scale_i32(latest.temperature_c, 100.0f);
            const int32_t p_chPa = scale_i32(latest.pressure_hpa, 100.0f);

            const int n = snprintf((char*)buf, sizeof(buf),
                "TS:%" PRIu32 ",V:%u,AX_mg:%" PRId32 ",AY_mg:%" PRId32 ",AZ_mg:%" PRId32
                ",GX_cdps:%" PRId32 ",GY_cdps:%" PRId32 ",GZ_cdps:%" PRId32
                ",T_cC:%" PRId32 ",P_chPa:%" PRId32
                ",ERR:%" PRIu32 ",RET:%" PRIu32 ",MPUF:%" PRIu32 ",BMPF:%" PRIu32
                ",RD_us:%" PRIu32 ",FILT_us:%" PRIu32 ",UART_us:%" PRIu32 ",ERRMON_us:%" PRIu32
                ",QD:%" PRIu32 ",WDG:%" PRIu32 ",H:%" PRIu32 ",F:%" PRIu32 "\r\n",
                latest.timestamp_ms, (unsigned)latest.valid_mask,
                ax_mg, ay_mg, az_mg,
                gx_cdps, gy_cdps, gz_cdps,
                t_cC, p_chPa,
                hw->comm_errors, hw->retries, hw->mpu_failures, hw->bmp_failures,
                g_rtos.sensor_read_us, g_rtos.filter_us, g_rtos.uart_us, g_rtos.error_monitor_us,
                g_rtos.raw_queue_drops + g_rtos.filtered_queue_drops,
                g_rtos.watchdog_refreshes, g_rtos.health_mask_last, g_rtos.fault_flags);
            if (n > 0) {
                const uint16_t len = (uint16_t)((n >= (int)sizeof(buf)) ? (sizeof(buf) - 1U) : (uint32_t)n);
                if (HAL_UART_Transmit(g_uart, buf, len, FSH_UART_TIMEOUT_MS) != HAL_OK) {
                    (void)HAL_UART_Abort(g_uart);
                    (void)HAL_UART_DeInit(g_uart);
                    (void)HAL_UART_Init(g_uart);
                }
            }

            g_rtos.uart_us = timing_elapsed_us(t0);
            if (g_rtos.uart_us > g_rtos.uart_max_us) {
                g_rtos.uart_max_us = g_rtos.uart_us;
            }
            mark_task_alive(FSH_HEALTH_UART_TASK);
        }
    }
}

static void ErrorMonitorTask(void *argument)
{
    (void)argument;

    for (;;) {
        osDelay(250U);
        const uint32_t t0 = timing_start();

        const uint32_t health = g_taskHeartbeat;
        g_rtos.health_mask_last = health;

        const FSH_HwStatus_t *hw = FSH_HW_GetStatus();
        if (hw != NULL && (hw->mpu_ready == 0U || hw->bmp_ready == 0U)) {
            g_rtos.fault_flags |= FSH_FAULT_SENSOR_MISSING;
        }

        if ((health & FSH_HEALTH_CRITICAL_MASK) == FSH_HEALTH_CRITICAL_MASK) {
            if (g_iwdg != NULL && HAL_IWDG_Refresh(g_iwdg) == HAL_OK) {
                g_rtos.watchdog_refreshes++;
            }
        } else {
            g_rtos.fault_flags |= FSH_FAULT_TASK_STALL;
        }

        /* LD2 PA5: slow blink when healthy, faster blink once a fault is seen. */
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        if (g_rtos.fault_flags != 0U) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }

        g_taskHeartbeat = 0U;
        mark_task_alive(FSH_HEALTH_ERROR_TASK);

        g_rtos.error_monitor_us = timing_elapsed_us(t0);
        if (g_rtos.error_monitor_us > g_rtos.error_monitor_max_us) {
            g_rtos.error_monitor_max_us = g_rtos.error_monitor_us;
        }
    }
}

/* =================== HELPERS =================== */
static void ma_update(MA_t *ma, float x)
{
    if (ma->count < FSH_MA_WINDOW) ma->count++;
    ma->sum -= ma->buf[ma->idx];
    ma->buf[ma->idx] = x;
    ma->sum += x;
    ma->idx = (uint16_t)((ma->idx + 1U) % FSH_MA_WINDOW);
}

static float ma_get(const MA_t *ma)
{
    return (ma->count == 0U) ? 0.0f : (ma->sum / (float)ma->count);
}

static uint8_t queue_put_latest(osMessageQueueId_t q, const void *item, uint32_t *drop_counter)
{
    if (q == NULL || item == NULL) return 0U;

    if (osMessageQueuePut(q, item, 0U, 0U) == osOK) {
        return 1U;
    }

    FSH_SensorSample_t discard;
    (void)osMessageQueueGet(q, &discard, NULL, 0U);

    if (drop_counter != NULL) {
        (*drop_counter)++;
    }
    g_rtos.fault_flags |= FSH_FAULT_QUEUE_DROP;

    return (osMessageQueuePut(q, item, 0U, 0U) == osOK) ? 1U : 0U;
}

static void timing_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t timing_start(void)
{
    return DWT->CYCCNT;
}

static uint32_t timing_elapsed_us(uint32_t start_cycles)
{
    const uint32_t elapsed = DWT->CYCCNT - start_cycles;
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) return 0U;
    return elapsed / cycles_per_us;
}

static void mark_task_alive(uint32_t bit)
{
    taskENTER_CRITICAL();
    g_taskHeartbeat |= bit;
    taskEXIT_CRITICAL();
}

static int32_t scale_i32(float value, float scale)
{
    float scaled = value * scale;
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int32_t)scaled;
}
