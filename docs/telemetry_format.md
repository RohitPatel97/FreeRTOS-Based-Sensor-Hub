# Telemetry Format Notes

The UART line is intentionally integer-scaled so the STM32 does not need float `printf` support.

Example:

```text
TS:1234,V:3,AX_mg:10,AY_mg:-5,AZ_mg:1002,GX_cdps:0,GY_cdps:1,GZ_cdps:-2,T_cC:2450,P_chPa:101325,ERR:0,RET:0,MPUF:0,BMPF:0,RD_us:900,FILT_us:20,UART_us:2000,ERRMON_us:15,QD:0,WDG:5,H:7,F:0
```

## Sensor fields

| Field | Meaning | Scale |
|---|---|---|
| `TS` | HAL timestamp | ms |
| `V` | validity mask | bit0 = MPU6050, bit1 = BMP280 |
| `AX_mg`, `AY_mg`, `AZ_mg` | filtered acceleration | milli-g |
| `GX_cdps`, `GY_cdps`, `GZ_cdps` | gyro data | centi-deg/sec |
| `T_cC` | BMP280 temperature | centi-C |
| `P_chPa` | BMP280 pressure | centi-hPa |

## Fault / RTOS fields

| Field | Meaning |
|---|---|
| `ERR` | total sensor communication errors |
| `RET` | I2C retry counter |
| `MPUF` | MPU6050 failure counter |
| `BMPF` | BMP280 failure counter |
| `RD_us` | last sensor read task runtime |
| `FILT_us` | last filtering task runtime |
| `UART_us` | last UART transmit task runtime |
| `ERRMON_us` | last error monitor task runtime |
| `QD` | raw + filtered queue drop count |
| `WDG` | watchdog refresh count |
| `H` | task health heartbeat mask |
| `F` | fault flag mask |
