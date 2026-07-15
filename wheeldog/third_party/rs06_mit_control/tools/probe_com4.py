#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度探测 COM4：波特率扫描、被动监听、gamepad 完整 init 序列。"""

from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import serial
import serial.tools.list_ports as list_ports


def can2com(candata: str) -> bytes:
    mapping = {"00": "00", "01": "0c", "02": "14", "03": "18", "04": "20", "12": "90"}
    h = candata[:8]
    d = candata[8:]
    return bytes.fromhex(
        "4154" + mapping[h[0:2]] + "07e8" + mapping[h[6:8]] + d + "0d0a"
    )


def gamepad_init_sequence(motor_id: str) -> list[tuple[str, bytes]]:
    m = motor_id
    cmds = [
        ("init", can2com(f"0000fd{m}0100")),
        ("speed_mode", can2com(f"1200fd{m}080570000002000000")),
        ("current_2A", can2com(f"1200fd{m}081870000000004000")),
        ("accel_10", can2com(f"1200fd{m}082270000041200000")),
        ("enable", can2com(f"0400fd{m}0800c4000000000000")),
        ("start", can2com(f"0300fd{m}080000000000000000")),
    ]
    return cmds


def mit_init_sequence(motor_id: str) -> list[tuple[str, bytes]]:
    m = motor_id
    return [
        ("init", can2com(f"0000fd{m}0100")),
        ("run_mode_mit", can2com(f"1200fd{m}080570000000000000")),
        ("enable", can2com(f"0400fd{m}0800c4000000000000")),
        ("start", can2com(f"0300fd{m}080000000000000000")),
        ("mit", can2com("01007fff01" + "087fff7fff004100c4")),
    ]


def read_all(ser, seconds: float) -> bytes:
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.005)
    return bytes(buf)


def try_baud(port: str, baud: int) -> int:
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as exc:
        print(f"  baud {baud}: open fail {exc}")
        return -1
    try:
        ser.reset_input_buffer()
        passive = read_all(ser, 1.0)
        ser.write(can2com("0000fd010100"))
        ser.flush()
        active = read_all(ser, 1.0)
        total = len(passive) + len(active)
        print(
            f"  baud {baud:7d}: passive={len(passive)} active={len(active)} "
            f"total={total}"
        )
        if passive:
            print(f"    passive hex: {passive.hex()[:120]}")
        if active:
            print(f"    active  hex: {active.hex()[:120]}")
        return total
    finally:
        ser.close()


def run_sequence(port: str, baud: int, name: str, steps: list[tuple[str, bytes]]) -> int:
    print(f"\n=== {name} @ {baud} ===")
    ser = serial.Serial(
        port,
        baud,
        timeout=0.05,
        rtscts=False,
        dsrdtr=False,
    )
    total = 0
    try:
        ser.reset_input_buffer()
        pre = read_all(ser, 0.5)
        if pre:
            print(f"pre-listen: {pre.hex()}")
            total += len(pre)
        for step_name, frame in steps:
            ser.write(frame)
            ser.flush()
            rx = read_all(ser, 0.8)
            print(f"{step_name:12s} tx={frame.hex()} rx={len(rx)} {rx.hex()[:80] if rx else ''}")
            total += len(rx)
            time.sleep(0.05)
        post = read_all(ser, 2.0)
        if post:
            print(f"post-listen: {post.hex()}")
            total += len(post)
    finally:
        ser.close()
    return total


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    print("Ports:", [p.device for p in list_ports.comports()])

    print("\n--- Baud scan (init only) ---")
    best = 0
    best_baud = 921600
    for baud in [921600, 115200, 2000000, 1000000, 460800, 1500000, 57600]:
        n = try_baud(port, baud)
        if n > best:
            best = n
            best_baud = baud

    print(f"\nBest baud by RX bytes: {best_baud} ({best} bytes)")

    g_total = run_sequence(port, 921600, "gamepad_init_m1", gamepad_init_sequence("01"))
    m_total = run_sequence(port, 921600, "mit_init_m1", mit_init_sequence("01"))

    print("\n" + "=" * 60)
    if g_total == 0 and m_total == 0:
        print("仍无串口回传。高概率为硬件侧问题：")
        print("  - 电机/驱动器未上电")
        print("  - CAN_H/CAN_L 未接或接反")
        print("  - USB-CAN 模块 DIP 开关不在正常模式")
        print("  - COM4 不是 CAN 适配器（只是普通 CH340 串口）")
    else:
        print(f"收到数据: gamepad序列={g_total} B, MIT序列={m_total} B")
    print("=" * 60)


if __name__ == "__main__":
    main()
