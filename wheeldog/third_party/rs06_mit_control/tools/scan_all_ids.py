#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫描 SANPO 双 MCU 上所有总线，按正确物理接口名显示（CAN/RS485）。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protocol import (
    build_get_device_id_command,
    decode_device_id_response,
    parse_adapter_buffer_with_channel,
)
from sanpo_bus_map import MCU1_COM, MCU2_COM, bus_label, is_can_port
from transport import UsbCanTransport


def scan_port(
    port: str,
    baud: int,
    id_min: int,
    id_max: int,
    delay_ms: float,
    can_only: bool,
) -> dict[tuple[str, int, int, str], int]:
    transport = UsbCanTransport(port, baud)
    try:
        transport.open()
    except Exception as exc:
        print(f"[{port}] 无法打开: {exc}")
        return {}

    found: dict[tuple[str, int, int, str], int] = {}
    print(f"\n{'='*70}")
    print(f"扫描 {port} @ {baud}，ID {id_min}~{id_max}")
    print(f"{'probe':>6}  {'物理接口':>10}  {'bus_idx':>7}  {'motor_id':>8}  UUID")
    print("-" * 70)

    try:
        for mid in range(id_min, id_max + 1):
            transport.send(build_get_device_id_command(mid), "", False)
            time.sleep(delay_ms / 1000.0)
            raw = transport.drain_rx(max(150.0, delay_ms * 2))
            for bus_idx, _cid, data in parse_adapter_buffer_with_channel(raw):
                dev = decode_device_id_response(_cid, data)
                if not dev:
                    continue
                motor_id, uid = dev
                uid_hex = uid.hex()
                label = bus_label(port, bus_idx)
                if can_only and not is_can_port(port, bus_idx):
                    continue
                key = (port, bus_idx, motor_id, uid_hex)
                if key in found:
                    continue
                found[key] = mid
                print(f"{mid:6d}  {label:>10}  0x{bus_idx:02x}     {motor_id:8d}  {uid_hex}")
    finally:
        transport.close()
    return found


def main() -> int:
    ap = argparse.ArgumentParser(description="SANPO 双 MCU 电机扫描（正确 CAN/RS485 命名）")
    ap.add_argument("--ports", default=f"{MCU1_COM},{MCU2_COM}")
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--min", type=int, default=1, dest="id_min")
    ap.add_argument("--max", type=int, default=127, dest="id_max")
    ap.add_argument("--delay-ms", type=float, default=60.0)
    ap.add_argument("--can-only", action="store_true", help="只显示 CAN 口，隐藏 RS485")
    args = ap.parse_args()

    ports = [p.strip() for p in args.ports.split(",") if p.strip()]
    all_found: dict[tuple[str, int, int, str], int] = {}

    for port in ports:
        found = scan_port(
            port, args.baudrate, args.id_min, args.id_max, args.delay_ms, args.can_only
        )
        all_found.update(found)

    print(f"\n{'='*70}")
    print(f"汇总: 共 {len(all_found)} 个唯一电机")
    if not all_found:
        print("  无回传")
        return 1

    can_count = sum(1 for k in all_found if is_can_port(k[0], k[1]))
    rs485_count = len(all_found) - can_count
    print(f"  其中 CAN: {can_count}，RS485: {rs485_count}")

    for (port, bus_idx, mid, uid), probe in sorted(
        all_found.items(), key=lambda x: (x[0][0], x[0][1], x[0][2])
    ):
        label = bus_label(port, bus_idx)
        print(f"  [{port}] {label:>10} (idx=0x{bus_idx:02x})  id={mid}  uid={uid}  probe={probe}")

    print("\n说明: ET 帧第3字节 bus_idx 在 MCU1 上映射 CAN1/CAN2/RS485-1/RS485-2，")
    print("      在 MCU2 上映射 CAN3/CAN4/RS485-3/RS485-4。此前误标为 CAN1~4。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
