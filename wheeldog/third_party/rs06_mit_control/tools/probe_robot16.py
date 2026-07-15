#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""快速探测：多串口 × 多波特率 × 多传输模式，找有 CAN 回传的端口。"""

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
    print("请安装 pyserial")
    raise SystemExit(1)

from protocol import build_get_device_id_command, parse_adapter_buffer
from sanpo_transport import SanpoCanTransport


def probe_port(
    port: str,
    baud: int,
    mode: str,
    motor_ids: list[int],
    listen_ms: float = 400,
) -> int:
    transport = SanpoCanTransport(port, baud, mode=mode)  # type: ignore[arg-type]
    try:
        transport.open()
    except (serial.SerialException, OSError) as exc:
        print(f"  {port}@{baud} {mode}: 无法打开 ({exc})")
        return 0

    hits = 0
    try:
        for mid in motor_ids:
            for ch in range(4):
                frame = build_get_device_id_command(mid)
                if mode == "sanpo_prefix":
                    transport.send_on_channel(ch, frame, "", False)
                else:
                    transport.send_broadcast(frame, "", False)
                time.sleep(0.05)
                raw = transport.drain_rx(listen_ms / 4)
                if raw:
                    parsed = parse_adapter_buffer(raw)
                    if parsed:
                        hits += len(parsed)
                        print(
                            f"  [命中] {port}@{baud} mode={mode} ch={ch} mid={mid} "
                            f"rx={len(raw)}B frames={len(parsed)} raw={raw[:32].hex()}..."
                        )
    finally:
        transport.close()
    return hits


def main() -> int:
    ap = argparse.ArgumentParser(description="扫描串口找 CAN 电机回传")
    ap.add_argument("--ports", default="", help="逗号分隔，默认全部")
    ap.add_argument("--baudrates", default="921600,115200,2000000,1000000")
    ap.add_argument("--motor-ids", default="127,1")
    args = ap.parse_args()

    if args.ports.strip():
        ports = [p.strip() for p in args.ports.split(",") if p.strip()]
    else:
        ports = [p.device for p in list_ports.comports()]

    bauds = [int(x) for x in args.baudrates.split(",")]
    mids = [int(x) for x in args.motor_ids.split(",")]
    modes = ["sanpo_prefix", "lingzu"]

    print(f"扫描 {len(ports)} 个串口, motor_ids={mids}\n")
    total = 0
    for port in ports:
        for baud in bauds:
            for mode in modes:
                total += probe_port(port, baud, mode, mids)

    if total == 0:
        print("\n未发现任何 CAN 回传。请检查:")
        print("  1) 电机与 SANPO 板是否上电")
        print("  2) USB 线是否接对（STM VCP 多为 COM10/COM12）")
        print("  3) 是否被上位机占用")
        print("  4) SANPO 固件 USB 协议是否与灵足 AT 帧不同（需对照 usb_can.md）")
        return 1
    print(f"\n共 {total} 次有效回传")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
