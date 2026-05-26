# Demo Video Test Plan

This is the shot list I would use to show the RTOS version clearly without making the video too long.

## Equipment

- NUCLEO-F401RE
- MPU6050 on I2C1
- BMP280 on I2C1
- USB cable for ST-LINK VCP UART
- PC running `dashboard/serial_dashboard.py`
- Optional: logic analyzer on PB8/PB9

## Test 1 — Normal operation

1. Show the board powered over USB.
2. Show MPU6050 and BMP280 wired to PB8/PB9.
3. Open the Python dashboard.
4. Confirm telemetry updates at 10 Hz.
5. Point out `V:3`, low/no `ERR`, and stable task timing fields.

Expected result:

```text
V:3
ERR steady or low
RET steady or low
RD_us/FILT_us/UART_us visible
```

## Test 2 — Disconnect BMP280

1. Pull BMP280 VCC or SDA carefully.
2. Keep the MPU6050 connected.
3. Show dashboard continuing to update.
4. Point out that `V` changes from `3` to `1`.
5. Point out `BMPF` and/or `RET` increasing on retry attempts.

Expected result:

```text
V:1
BMPF increases on retry attempts
Firmware keeps running
Watchdog continues refreshing
```

## Test 3 — Disconnect MPU6050

1. Reconnect BMP280.
2. Disconnect MPU6050 VCC or SDA.
3. Show telemetry continues.
4. Point out that `V` changes from `3` to `2`.
5. Point out `MPUF` and/or `RET` increasing on retry attempts.

Expected result:

```text
V:2
MPUF increases on retry attempts
No firmware lockup
```

## Test 4 — Noisy wiring / longer leads

1. Add longer I2C leads or slightly disturb the wiring.
2. Watch `RET` and `ERR`.
3. Show the dashboard still plotting data.
4. Mention that I2C retry + recovery prevents a simple wiring fault from killing the whole hub.

Expected result:

```text
RET increases
ERR may increase
RTOS tasks continue running
```

## Talking points for the video

- TIM2 only signals timing; sensor reads happen in RTOS task context.
- I2C hardware access is protected by a mutex.
- Raw data moves through queues.
- UART telemetry is separated from sensor reads.
- Error monitor refreshes the watchdog only when the critical tasks are alive.
