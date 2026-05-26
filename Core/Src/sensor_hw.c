/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sensor_hw.c
  * @brief   MPU6050 + BMP280 hardware layer with retry counters and fault recovery.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sensor_hw.h"
#include <string.h>

/* =================== BMP280 CALIBRATION =================== */
typedef struct {
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3; int16_t dig_P4; int16_t dig_P5;
    int16_t dig_P6; int16_t dig_P7; int16_t dig_P8; int16_t dig_P9;
    int32_t t_fine;
} BMP280_Cal_t;

static I2C_HandleTypeDef *g_i2c = NULL;
static FSH_HwStatus_t g_hw = {0};
static BMP280_Cal_t g_bmp = {0};
static uint8_t g_bmp_addr7 = FSH_BMP280_ADDR7;

static HAL_StatusTypeDef mpu_init(void);
static uint8_t mpu_read(FSH_SensorSample_t *out);
static HAL_StatusTypeDef bmp_init(void);
static uint8_t bmp_read(FSH_SensorSample_t *out);

static uint8_t i2c_read_retry(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);
static uint8_t i2c_write_retry(uint8_t addr7, uint8_t reg, uint8_t val);
static void i2c_bus_clear_pb8_pb9(void);

static int32_t bmp_comp_t(int32_t adc_t);
static uint32_t bmp_comp_p(int32_t adc_p);

/* =================== PUBLIC API =================== */
HAL_StatusTypeDef FSH_HW_Init(I2C_HandleTypeDef *hi2c1)
{
    if (hi2c1 == NULL) return HAL_ERROR;

    g_i2c = hi2c1;
    memset(&g_hw, 0, sizeof(g_hw));
    memset(&g_bmp, 0, sizeof(g_bmp));
    g_bmp_addr7 = FSH_BMP280_ADDR7;

    g_hw.mpu_ready = (mpu_init() == HAL_OK) ? 1U : 0U;
    g_hw.bmp_ready = (bmp_init() == HAL_OK) ? 1U : 0U;

    /* Missing sensors are not fatal.  The RTOS project reports them in V/ERR. */
    return HAL_OK;
}

uint8_t FSH_HW_ReadSensors(FSH_SensorSample_t *out)
{
    static uint16_t mpu_reinit_ctr = 0;
    static uint16_t bmp_reinit_ctr = 0;

    if (out == NULL) return 0U;

    memset(out, 0, sizeof(*out));
    out->timestamp_ms = HAL_GetTick();
    out->valid_mask = 0U;

    uint8_t ok_mpu = 0U;
    uint8_t ok_bmp = 0U;
    uint8_t attempted_mpu = 0U;
    uint8_t attempted_bmp = 0U;

    if (g_hw.mpu_ready) {
        attempted_mpu = 1U;
        ok_mpu = mpu_read(out);
    } else if (++mpu_reinit_ctr >= FSH_SAMPLE_RATE_HZ) {
        mpu_reinit_ctr = 0U;
        attempted_mpu = 1U;
        g_hw.mpu_ready = (mpu_init() == HAL_OK) ? 1U : 0U;
        if (g_hw.mpu_ready) ok_mpu = mpu_read(out);
    }

    if (ok_mpu) {
        mpu_reinit_ctr = 0U;
        out->valid_mask |= FSH_VALID_MPU6050;
    } else if (attempted_mpu) {
        g_hw.mpu_ready = 0U;
        g_hw.comm_errors++;
        g_hw.mpu_failures++;
    }

    if (g_hw.bmp_ready) {
        attempted_bmp = 1U;
        ok_bmp = bmp_read(out);
    } else if (++bmp_reinit_ctr >= FSH_SAMPLE_RATE_HZ) {
        bmp_reinit_ctr = 0U;
        attempted_bmp = 1U;
        g_hw.bmp_ready = (bmp_init() == HAL_OK) ? 1U : 0U;
        if (g_hw.bmp_ready) ok_bmp = bmp_read(out);
    }

    if (ok_bmp) {
        bmp_reinit_ctr = 0U;
        out->valid_mask |= FSH_VALID_BMP280;
    } else if (attempted_bmp) {
        g_hw.bmp_ready = 0U;
        g_hw.comm_errors++;
        g_hw.bmp_failures++;
    }

    g_hw.samples_taken++;
    return out->valid_mask;
}

const FSH_HwStatus_t* FSH_HW_GetStatus(void)
{
    return &g_hw;
}

void FSH_HW_I2CRecover(void)
{
    if (g_i2c == NULL) return;

    i2c_bus_clear_pb8_pb9();
    (void)HAL_I2C_DeInit(g_i2c);
    (void)HAL_I2C_Init(g_i2c);
    g_hw.i2c_recoveries++;
}

/* =================== I2C RETRY HELPERS =================== */
static uint8_t i2c_read_retry(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (g_i2c == NULL || data == NULL || len == 0U) return 0U;

    for (uint8_t r = 0; r < FSH_MAX_RETRIES; r++) {
        if (HAL_I2C_Mem_Read(g_i2c, (uint16_t)(addr7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, data, len,
                             FSH_I2C_TIMEOUT_MS) == HAL_OK) {
            return 1U;
        }
        g_hw.retries++;
        HAL_Delay(1);
    }

    FSH_HW_I2CRecover();
    return 0U;
}

static uint8_t i2c_write_retry(uint8_t addr7, uint8_t reg, uint8_t val)
{
    if (g_i2c == NULL) return 0U;

    for (uint8_t r = 0; r < FSH_MAX_RETRIES; r++) {
        if (HAL_I2C_Mem_Write(g_i2c, (uint16_t)(addr7 << 1), reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1U,
                              FSH_I2C_TIMEOUT_MS) == HAL_OK) {
            return 1U;
        }
        g_hw.retries++;
        HAL_Delay(1);
    }

    FSH_HW_I2CRecover();
    return 0U;
}

static void i2c_bus_clear_pb8_pb9(void)
{
    if (g_i2c == NULL) return;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    (void)HAL_I2C_DeInit(g_i2c);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    for (uint8_t i = 0; i < 9U; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* STOP condition: SDA low while SCL high, then SDA high. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* =================== MPU6050 =================== */
static HAL_StatusTypeDef mpu_init(void)
{
    if (!i2c_write_retry(FSH_MPU6050_ADDR7, 0x6B, 0x00)) return HAL_ERROR; /* wake */
    if (!i2c_write_retry(FSH_MPU6050_ADDR7, 0x1B, 0x00)) return HAL_ERROR; /* gyro +/-250 dps */
    if (!i2c_write_retry(FSH_MPU6050_ADDR7, 0x1C, 0x00)) return HAL_ERROR; /* accel +/-2g */
    return HAL_OK;
}

static uint8_t mpu_read(FSH_SensorSample_t *out)
{
    uint8_t b[14];
    if (!i2c_read_retry(FSH_MPU6050_ADDR7, 0x3B, b, (uint16_t)sizeof(b))) return 0U;

    const int16_t ax = (int16_t)((uint16_t)b[0]  << 8 | b[1]);
    const int16_t ay = (int16_t)((uint16_t)b[2]  << 8 | b[3]);
    const int16_t az = (int16_t)((uint16_t)b[4]  << 8 | b[5]);
    const int16_t gx = (int16_t)((uint16_t)b[8]  << 8 | b[9]);
    const int16_t gy = (int16_t)((uint16_t)b[10] << 8 | b[11]);
    const int16_t gz = (int16_t)((uint16_t)b[12] << 8 | b[13]);

    out->accel_x = (float)ax / 16384.0f;
    out->accel_y = (float)ay / 16384.0f;
    out->accel_z = (float)az / 16384.0f;
    out->gyro_x  = (float)gx / 131.0f;
    out->gyro_y  = (float)gy / 131.0f;
    out->gyro_z  = (float)gz / 131.0f;

    return 1U;
}

/* =================== BMP280 =================== */
static HAL_StatusTypeDef bmp_init(void)
{
    uint8_t id = 0U;
    uint8_t addr = FSH_BMP280_ADDR7;

    if (!i2c_read_retry(addr, 0xD0, &id, 1U)) {
        addr = FSH_BMP280_ADDR7_ALT;
        if (!i2c_read_retry(addr, 0xD0, &id, 1U)) return HAL_ERROR;
    }

    if (id != 0x58U && id != 0x56U && id != 0x57U) return HAL_ERROR;
    g_bmp_addr7 = addr;

    uint8_t c[24];
    if (!i2c_read_retry(g_bmp_addr7, 0x88, c, (uint16_t)sizeof(c))) return HAL_ERROR;

    g_bmp.dig_T1 = (uint16_t)((uint16_t)c[1] << 8 | c[0]);
    g_bmp.dig_T2 = (int16_t)((uint16_t)c[3] << 8 | c[2]);
    g_bmp.dig_T3 = (int16_t)((uint16_t)c[5] << 8 | c[4]);
    g_bmp.dig_P1 = (uint16_t)((uint16_t)c[7] << 8 | c[6]);
    g_bmp.dig_P2 = (int16_t)((uint16_t)c[9] << 8 | c[8]);
    g_bmp.dig_P3 = (int16_t)((uint16_t)c[11] << 8 | c[10]);
    g_bmp.dig_P4 = (int16_t)((uint16_t)c[13] << 8 | c[12]);
    g_bmp.dig_P5 = (int16_t)((uint16_t)c[15] << 8 | c[14]);
    g_bmp.dig_P6 = (int16_t)((uint16_t)c[17] << 8 | c[16]);
    g_bmp.dig_P7 = (int16_t)((uint16_t)c[19] << 8 | c[18]);
    g_bmp.dig_P8 = (int16_t)((uint16_t)c[21] << 8 | c[20]);
    g_bmp.dig_P9 = (int16_t)((uint16_t)c[23] << 8 | c[22]);

    if (!i2c_write_retry(g_bmp_addr7, 0xF4, 0x00)) return HAL_ERROR; /* sleep while changing config */
    if (!i2c_write_retry(g_bmp_addr7, 0xF5, 0x08)) return HAL_ERROR; /* standby 0.5ms, filter x4 */
    if (!i2c_write_retry(g_bmp_addr7, 0xF4, 0x27)) return HAL_ERROR; /* x1 temp/press, normal */

    return HAL_OK;
}

static uint8_t bmp_read(FSH_SensorSample_t *out)
{
    uint8_t b[6];
    if (!i2c_read_retry(g_bmp_addr7, 0xF7, b, (uint16_t)sizeof(b))) return 0U;

    const int32_t adc_p = (int32_t)(((uint32_t)b[0] << 12) | ((uint32_t)b[1] << 4) | ((uint32_t)b[2] >> 4));
    const int32_t adc_t = (int32_t)(((uint32_t)b[3] << 12) | ((uint32_t)b[4] << 4) | ((uint32_t)b[5] >> 4));

    const int32_t t_x100 = bmp_comp_t(adc_t);
    const uint32_t p_q24_8 = bmp_comp_p(adc_p);

    out->temperature_c = (float)t_x100 / 100.0f;
    out->pressure_hpa  = ((float)p_q24_8 / 256.0f) / 100.0f;
    return 1U;
}

static int32_t bmp_comp_t(int32_t adc_t)
{
    const int32_t var1 = ((((adc_t >> 3) - ((int32_t)g_bmp.dig_T1 << 1))) * ((int32_t)g_bmp.dig_T2)) >> 11;
    const int32_t var2 = (((((adc_t >> 4) - ((int32_t)g_bmp.dig_T1)) *
                           ((adc_t >> 4) - ((int32_t)g_bmp.dig_T1))) >> 12) *
                           ((int32_t)g_bmp.dig_T3)) >> 14;
    g_bmp.t_fine = var1 + var2;
    return (g_bmp.t_fine * 5 + 128) >> 8;
}

static uint32_t bmp_comp_p(int32_t adc_p)
{
    int64_t var1 = (int64_t)g_bmp.t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)g_bmp.dig_P6;
    var2 = var2 + ((var1 * (int64_t)g_bmp.dig_P5) << 17);
    var2 = var2 + (((int64_t)g_bmp.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)g_bmp.dig_P3) >> 8) + ((var1 * (int64_t)g_bmp.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)g_bmp.dig_P1) >> 33;
    if (var1 == 0) return 0U;

    int64_t p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)g_bmp.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)g_bmp.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)g_bmp.dig_P7) << 4);
    return (uint32_t)p;
}
