#!/usr/bin/env python3
"""
Live serial dashboard for the FreeRTOS-Based Sensor Hub.

Telemetry example:
TS:1234,V:3,AX_mg:10,AY_mg:-5,AZ_mg:1002,GX_cdps:0,GY_cdps:1,GZ_cdps:-2,
T_cC:2450,P_chPa:101325,ERR:0,RET:0,MPUF:0,BMPF:0,RD_us:900,FILT_us:20,
UART_us:2000,ERRMON_us:15,QD:0,WDG:5,H:7,F:0
"""

from __future__ import annotations

import argparse
import collections
import re
import time
from typing import Dict, Optional

import matplotlib.pyplot as plt
import serial

PAIR_RE = re.compile(r"([A-Za-z0-9_]+):(-?\d+)")


def parse_line(line: str) -> Optional[Dict[str, int]]:
    pairs = PAIR_RE.findall(line)
    if not pairs:
        return None
    return {k: int(v) for k, v in pairs}


def main() -> None:
    parser = argparse.ArgumentParser(description="Live plot STM32 FreeRTOS sensor hub telemetry.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--window", type=int, default=200, help="Number of telemetry samples to keep")
    args = parser.parse_args()

    xs = collections.deque(maxlen=args.window)
    ax = collections.deque(maxlen=args.window)
    ay = collections.deque(maxlen=args.window)
    az = collections.deque(maxlen=args.window)
    err = collections.deque(maxlen=args.window)
    retries = collections.deque(maxlen=args.window)
    rd_us = collections.deque(maxlen=args.window)
    filt_us = collections.deque(maxlen=args.window)
    uart_us = collections.deque(maxlen=args.window)
    valid = collections.deque(maxlen=args.window)

    plt.ion()
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    fig.canvas.manager.set_window_title("FreeRTOS Sensor Hub Dashboard")

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print(f"Connected to {args.port} @ {args.baud}. Close the plot window to stop.")
        start = time.time()

        while plt.fignum_exists(fig.number):
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw:
                plt.pause(0.01)
                continue

            data = parse_line(raw)
            if data is None:
                continue

            t = (data.get("TS", int((time.time() - start) * 1000))) / 1000.0
            xs.append(t)
            ax.append(data.get("AX_mg", 0) / 1000.0)
            ay.append(data.get("AY_mg", 0) / 1000.0)
            az.append(data.get("AZ_mg", 0) / 1000.0)
            err.append(data.get("ERR", 0))
            retries.append(data.get("RET", 0))
            rd_us.append(data.get("RD_us", 0))
            filt_us.append(data.get("FILT_us", 0))
            uart_us.append(data.get("UART_us", 0))
            valid.append(data.get("V", 0))

            for axis in axes:
                axis.cla()
                axis.grid(True, alpha=0.3)

            axes[0].plot(xs, ax, label="AX g")
            axes[0].plot(xs, ay, label="AY g")
            axes[0].plot(xs, az, label="AZ g")
            axes[0].set_ylabel("Accel (g)")
            axes[0].legend(loc="upper right")

            axes[1].plot(xs, valid, label="Validity mask V")
            axes[1].set_ylabel("V mask")
            axes[1].set_ylim(-0.2, 3.2)
            axes[1].legend(loc="upper right")

            axes[2].plot(xs, err, label="ERR")
            axes[2].plot(xs, retries, label="RET")
            axes[2].set_ylabel("Counters")
            axes[2].legend(loc="upper left")

            axes[3].plot(xs, rd_us, label="Sensor read us")
            axes[3].plot(xs, filt_us, label="Filter us")
            axes[3].plot(xs, uart_us, label="UART us")
            axes[3].set_ylabel("Task time (us)")
            axes[3].set_xlabel("Time (s)")
            axes[3].legend(loc="upper left")

            latest_v = data.get("V", 0)
            fault = data.get("F", 0)
            health = data.get("H", 0)
            fig.suptitle(
                f"FreeRTOS Sensor Hub | V={latest_v} | FaultFlags={fault} | HealthMask={health} | "
                f"MPUF={data.get('MPUF', 0)} BMPF={data.get('BMPF', 0)} QD={data.get('QD', 0)}"
            )

            plt.tight_layout()
            plt.pause(0.01)


if __name__ == "__main__":
    main()
