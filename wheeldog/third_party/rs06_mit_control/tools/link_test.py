#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""USB 转 CAN / SANPO 板链路测试：开串口、发 init、听原始回传。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("pip install pyserial")
    raise SystemExit(1)

from protocol import build_get_device_id_command, build_init_command, iter_at_frames


def open_ser(port: str, baud: int, dtr: bool = True, rts: bool = False) -> serial.Serial:
    ser = serial.Serial(
        port, baud, timeout=0.05, dsrdtr=False, rtscts=False
    )
    ser.dtr = dtr
    ser.rts = rts
    time.sleep(0.08)
    ser.reset_input_buffer()
    return ser


def listen_raw(ser: serial.Serial, ms: float) -> bytes:
    buf = bytearray()
    deadline = time.time() + ms / 1000.0
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        else:
            time.sleep(0.005)
    return bytes(buf)


def test_port(port: str, baud: int) -> bool:
    print(f"\n--- {port} @ {baud} ---")
    hit = False
    for dtr, rts, label in [(True, False, "DTR=1"), (False, False, "DTR=0"), (True, True, "DTR=1 RTS=1")]:
        try:
            ser = open_ser(port, baud, dtr, rts)
        except (serial.SerialException, OSError) as e:
            print(f"  无法打开: {e}")
            return False

        try:
            # 被动监听
            passive = listen_raw(ser, 200)
            if passive:
                print(f"  [{label}] 被动 RX {len(passive)}B: {passive[:64].hex()}")
                hit = True

            # 灵足 AT init（无通道前缀）
            for mid in (127, 1):
                frame = build_init_command(mid)
                ser.write(frame)
                ser.flush()
                time.sleep(0.08)
                rx = listen_raw(ser, 350)
                if rx:
                    print(f"  [{label}] init mid={mid} RX {len(rx)}B: {rx.hex()}")
                    at = list(iter_at_frames(rx))
                    if at:
                        print(f"    AT 帧数={len(at)} 首帧={at[0].hex()}")
                    hit = True

            # SANPO 风格：通道前缀 0~3 + init
            for ch in range(4):
                frame = bytes([ch]) + build_init_command(127)
                ser.write(frame)
                ser.flush()
                time.sleep(0.06)
                rx = listen_raw(ser, 250)
                if rx:
                    print(f"  [{label}] ch={ch} init RX {len(rx)}B: {rx.hex()}")
                    hit = True
        finally:
            ser.close()

    if not hit:
        print("  无回传")
    return hit


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="")
    ap.add_argument("--baudrates", default="921600,115200,2000000,1000000,460800")
    args = ap.parse_args()

    bauds = [int(x) for x in args.baudrates.split(",")]
    if args.port:
        ports = [args.port]
    else:
        ports = [p.device for p in list_ports.comports()]

    print("链路测试 — 端口:", ports)
    any_hit = False
    for port in ports:
        for baud in bauds:
            if test_port(port, baud):
                any_hit = True

    if not any_hit:
        print("\n所有端口均无回传。")
        print("若板子灯亮但无数据：可能是 SANPO 自有 USB 协议（非灵足 AT 4154...）")
        return 1
    print("\n至少一个端口有回传，请根据上方 COM 口与波特率继续 ping。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
