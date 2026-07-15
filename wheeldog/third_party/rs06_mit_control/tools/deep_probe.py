#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度探测：多组上位机抓包顺序 + 原始字节监听（不依赖 AT 解析）。"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("请先安装: pip install pyserial")
    raise SystemExit(1)

from config import AppConfig, MitSetpoint
from protocol import (
    build_broadcast_run_mode_command,
    build_enable_command,
    build_init_command,
    build_mit_command,
    build_set_run_mode_command,
    build_start_command,
    decode_device_id_response,
    decode_feedback_from_can_id,
    parse_adapter_buffer,
)


def can2com(candata: str) -> bytes:
    mapping = {"00": "00", "01": "0c", "02": "14", "03": "18", "04": "20", "12": "90"}
    h, d = candata[:8], candata[8:]
    return bytes.fromhex(
        "4154" + mapping.get(h[0:2], h[0:2]) + "07e8" + mapping.get(h[6:8], h[6:8]) + d + "0d0a"
    )


def open_port(port: str, baud: int, dtr: bool, rts: bool) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=0.05, dsrdtr=False, rtscts=False)
    ser.dtr = dtr
    ser.rts = rts
    time.sleep(0.05)
    return ser


def listen_raw(ser: serial.Serial, seconds: float) -> bytes:
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        else:
            time.sleep(0.003)
    return bytes(buf)


def analyze(label: str, raw: bytes, limits) -> None:
    print(f"  [{label}] RX {len(raw)} B: {raw.hex() if raw else '(empty)'}")
    if not raw:
        return
    if b"AT" not in raw and raw[:2] != b"AT":
        print("  注意: 原始数据不含 AT 帧头，可能是其它串口格式")
    for can_id, data in parse_adapter_buffer(raw):
        print(f"    CAN 0x{can_id:08X} data={data.hex()}")
        dev = decode_device_id_response(can_id, data)
        if dev:
            print(f"    设备ID motor={dev[0]} uid={dev[1].hex()}")
        fb = decode_feedback_from_can_id(can_id, data, limits)
        if fb:
            print(
                f"    反馈 pos={fb.position:+.4f} vel={fb.velocity:+.4f} "
                f"torque={fb.torque:+.4f} temp={fb.temperature:.1f}"
            )


def run_sequence(name: str, ser: serial.Serial, frames: list[tuple[str, bytes]], listen: float, limits) -> int:
    print(f"\n=== 序列: {name} ===")
    total = 0
    for step, frame in frames:
        ser.reset_input_buffer()
        print(f">> {step}")
        print(f"   TX: {frame.hex()}")
        ser.write(frame)
        ser.flush()
        raw = listen_raw(ser, listen)
        analyze(step, raw, limits)
        total += len(raw)
        time.sleep(0.03)
    return total


def main() -> int:
    ap = argparse.ArgumentParser(description="RS06 深度串口探测")
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--listen", type=float, default=0.8)
    ap.add_argument("--motor-id", type=int, default=1)
    ap.add_argument("--list-ports", action="store_true")
    args = ap.parse_args()

    if args.list_ports:
        for p in list_ports.comports():
            print(f"  {p.device:8s}  {p.description}")
        return 0

    cfg = AppConfig()
    cfg.can.motor_id = args.motor_id
    limits = cfg.limits
    mid = args.motor_id
    sp = MitSetpoint(kp=5.0, kd=0.3)

    mit_seq = [
        ("broadcast_m0_mit", build_broadcast_run_mode_command(0)),
        ("init", build_init_command(mid)),
        ("run_mode=0", build_set_run_mode_command(mid, 0)),
        ("enable_04_c4", build_enable_command(mid)),
        ("start_03", build_start_command(mid)),
        ("mit", build_mit_command(mid, sp, limits)),
    ]

    gamepad_seq = [
        ("init", build_init_command(mid)),
        ("run_mode=2", build_set_run_mode_command(mid, 2)),
        ("enable_04_c4", build_enable_command(mid)),
        ("start_03", build_start_command(mid)),
    ]

    ws_motor_seq = [
        ("speed_mode_m0", can2com("1200fd00080570000000000000")),
        ("init", build_init_command(mid)),
        ("enable_04_c4", build_enable_command(mid)),
        ("run_mode=2", build_set_run_mode_command(mid, 2)),
        ("start_03", build_start_command(mid)),
    ]

    official_seq = [
        ("init", build_init_command(mid)),
        ("enable_type3", can2com(f"0300fd{mid:02x}080000000000000000")),
    ]

    dtr_rts = [(True, False), (True, True), (False, False)]
    grand = 0
    for dtr, rts in dtr_rts:
        print(f"\n######## DTR={dtr} RTS={rts} ########")
        ser = open_port(args.port, args.baud, dtr, rts)
        try:
            raw = listen_raw(ser, 0.5)
            analyze("open_passive", raw, limits)
            grand += len(raw)
            for seq_name, frames in [
                ("MIT+上位机前导", mit_seq),
                ("gamepad速度模式", gamepad_seq),
                ("WS_Motor抓包", ws_motor_seq),
                ("官方手册type3使能", official_seq),
            ]:
                grand += run_sequence(seq_name, ser, frames, args.listen, limits)
        finally:
            ser.close()

    print("\n" + "=" * 60)
    if grand == 0:
        print("全部探测均为 0 字节回传。")
        print("若上位机此时能连接，请：")
        print("  1) 保持电机上电，关闭上位机后立刻重跑本脚本")
        print("  2) 用 tools/serial_sniffer.py 桥接抓上位机真实收发")
        print("  3) 确认设备管理器里 USB-CAN 就是本 COM 口")
    else:
        print(f"共收到 {grand} 字节，请根据上方解析结果调整 motor_id / 序列。")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
