#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RS06 USB-CAN 通信诊断脚本。

对照 ESP32-S3-RS485-CAN-Demo 中已验证的帧格式，依次尝试：
  init -> run_mode=0(MIT) -> enable(0x04+C4) -> start(0x03) -> MIT
并打印所有串口回传原始数据。
"""

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
    iter_at_frames,
    parse_adapter_buffer,
)


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    print("可用串口:")
    if not ports:
        print("  (无)")
        return
    for p in ports:
        print(f"  {p.device:8s}  {p.description}")


def dump_rx(label: str, raw: bytes, limits) -> int:
    print(f"  [{label}] RX {len(raw)} bytes: {raw.hex() if raw else '(empty)'}")
    if not raw:
        return 0
    frames = list(iter_at_frames(raw))
    if frames:
        print(f"  [{label}] 解析到 {len(frames)} 个 AT 帧")
    for i, frame in enumerate(frames):
        print(f"    AT#{i+1}: {frame.hex()}")
    parsed = parse_adapter_buffer(raw)
    for can_id, data in parsed:
        print(f"    CAN id=0x{can_id:08X} data={data.hex()}")
        dev = decode_device_id_response(can_id, data)
        if dev:
            print(f"    设备ID motor={dev[0]} uid={dev[1].hex()}")
        fb = decode_feedback_from_can_id(can_id, data, limits)
        if fb:
            print(
                f"    反馈: pos={fb.position:+.4f} vel={fb.velocity:+.4f} "
                f"torque={fb.torque:+.4f} temp={fb.temperature:.1f}"
            )
    return len(raw)


def send_step(ser, name: str, frame: bytes, listen_s: float, limits) -> int:
    ser.reset_input_buffer()
    print(f"\n>> {name}")
    print(f"   TX ({len(frame)} B): {frame.hex()}")
    ser.write(frame)
    ser.flush()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < listen_s:
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.005)
    return dump_rx(name, bytes(buf), limits)


def run_sequence(port: str, baud: int, motor_id: int, listen_s: float) -> None:
    cfg = AppConfig()
    cfg.can.motor_id = motor_id
    limits = cfg.limits
    sp = MitSetpoint(position=0.0, velocity=0.0, kp=5.0, kd=0.3, torque=0.0)

    steps = [
        ("broadcast motor0 MIT", build_broadcast_run_mode_command(0)),
        ("init", build_init_command(motor_id)),
        ("run_mode=0 (MIT)", build_set_run_mode_command(motor_id, 0)),
        ("enable (0x04+C4)", build_enable_command(motor_id)),
        ("start (0x03)", build_start_command(motor_id)),
        ("MIT 运控", build_mit_command(motor_id, sp, limits)),
    ]

    print(f"打开 {port} @ {baud}, motor_id={motor_id}")
    ser = serial.Serial(port, baud, timeout=0.05)
    total_rx = 0
    try:
        for name, frame in steps:
            total_rx += send_step(ser, name, frame, listen_s, limits)
            time.sleep(0.05)
        print(f"\n被动监听 2s ...")
        ser.reset_input_buffer()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < 2.0:
            chunk = ser.read(512)
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.01)
        total_rx += dump_rx("passive", bytes(buf), limits)
    finally:
        ser.close()

    print("\n" + "=" * 60)
    if total_rx == 0:
        print("结果: 未收到任何串口回传。请检查：")
        print("  1) 电机是否上电、CAN 线是否接好")
        print("  2) COM 口是否为灵足 USB-CAN 模块")
        print("  3) motor_id 是否正确（可试 --motor-id 1/2/127）")
        print("  4) USB-CAN DIP 开关是否在正常工作模式")
    else:
        print(f"结果: 收到 {total_rx} 字节回传，通信链路有数据。")
    print("=" * 60)


def scan_motor_ids(port: str, baud: int, ids: list[int], listen_s: float) -> None:
    cfg = AppConfig()
    limits = cfg.limits
    print(f"扫描 motor_id: {ids}")
    for mid in ids:
        ser = serial.Serial(port, baud, timeout=0.05)
        try:
            ser.reset_input_buffer()
            frame = build_init_command(mid)
            ser.write(frame)
            ser.flush()
            time.sleep(listen_s)
            raw = ser.read(512)
            hit = "HIT" if raw else "----"
            print(f"  id={mid:3d} {hit} rx={len(raw)} {raw.hex()[:80] if raw else ''}")
            if raw:
                for can_id, data in parse_adapter_buffer(raw):
                    print(f"         can_id=0x{can_id:08X} data={data.hex()}")
                    fb = decode_feedback_from_can_id(can_id, data, limits)
                    if fb:
                        print(f"         pos={fb.position:+.3f} vel={fb.velocity:+.3f}")
        finally:
            ser.close()
        time.sleep(0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description="RS06 USB-CAN 通信诊断")
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--motor-id", type=int, default=1)
    ap.add_argument("--listen", type=float, default=0.5, help="每步等待回传秒数")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--scan-ids", action="store_true", help="对多个 motor_id 发 init 探测")
    args = ap.parse_args()

    if args.list_ports:
        list_serial_ports()
        return 0

    if args.scan_ids:
        scan_motor_ids(args.port, args.baud, [1, 2, 3, 4, 127], args.listen)
        return 0

    run_sequence(args.port, args.baud, args.motor_id, args.listen)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
