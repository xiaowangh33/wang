#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轻量串口嗅探：独占监听或 com0com 桥接，抓取上位机与 USB-CAN 的真实字节。

桥接示例（上位机改连 COM31，硬件仍为 COM4）:
  py -3 tools/serial_sniffer.py --bridge --port-a COM31 --port-b COM4 --log upper_pc.log
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    print("pip install pyserial")
    raise SystemExit(1)


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def log_line(fp, direction: str, data: bytes) -> None:
    line = f"[{ts()}] {direction} ({len(data)} B) {data.hex(' ')}\n"
    print(line, end="")
    if fp:
        fp.write(line)
        fp.flush()


def pump(src: serial.Serial, dst: serial.Serial, label: str, fp, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            n = src.in_waiting
            if n:
                chunk = src.read(n)
                if chunk:
                    log_line(fp, label, chunk)
                    dst.write(chunk)
                    dst.flush()
            else:
                time.sleep(0.002)
        except serial.SerialException:
            break


def bridge(port_a: str, port_b: str, baud: int, log_path: str) -> None:
    log_fp = open(log_path, "a", encoding="utf-8") if log_path else None
    a = serial.Serial(port_a, baud, timeout=0.05)
    b = serial.Serial(port_b, baud, timeout=0.05)
    stop = threading.Event()
    t1 = threading.Thread(target=pump, args=(a, b, f"{port_a}->{port_b}", log_fp, stop), daemon=True)
    t2 = threading.Thread(target=pump, args=(b, a, f"{port_b}->{port_a}", log_fp, stop), daemon=True)
    print(f"桥接 {port_a} <-> {port_b} @ {baud}. Ctrl+C 退出。")
    t1.start()
    t2.start()
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop.set()
    finally:
        a.close()
        b.close()
        if log_fp:
            log_fp.close()


def sniff(port: str, baud: int, log_path: str) -> None:
    log_fp = open(log_path, "a", encoding="utf-8") if log_path else None
    ser = serial.Serial(port, baud, timeout=0.05)
    print(f"独占监听 {port} @ {baud}（需先关闭上位机）。Ctrl+C 退出。")
    try:
        while True:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
                log_line(log_fp, f"{port} RX", chunk)
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if log_fp:
            log_fp.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="灵足 USB-CAN 串口嗅探")
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--log", default="")
    ap.add_argument("--bridge", action="store_true")
    ap.add_argument("--port-a", default="COM31")
    ap.add_argument("--port-b", default="COM4")
    args = ap.parse_args()
    if args.bridge:
        bridge(args.port_a, args.port_b, args.baud, args.log)
    else:
        sniff(args.port, args.baud, args.log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
