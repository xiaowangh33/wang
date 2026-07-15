#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 SANPO 手册 ET 协议，逐路扫描 CAN1~CAN4 上的电机 (ID 1~4, 127)。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    import serial
except ImportError:
    print("pip install pyserial")
    raise SystemExit(1)

from protocol import (
    COMM_GET_DEVICE_ID,
    build_standard_can_id,
    decode_device_id_response,
    iter_et_frames,
)
from sanpo_bus_map import MCU1_COM, MCU2_COM

ET_HEADER = bytes([0x45, 0x54])
ET_TAIL = bytes([0x0D, 0x0A])
ENTER_ET_MODE_COMMAND = b"AT+ET\r\n"

PROBE_IDS = [1, 2, 3, 4, 127]


def parse_ports(text: str) -> tuple[str, str]:
    ports = [p.strip() for p in text.split(",") if p.strip()]
    if len(ports) != 2:
        raise argparse.ArgumentTypeError("需要两个串口，格式如 /dev/ttyACM0,/dev/ttyACM1")
    return ports[0], ports[1]


def build_can_routes(mcu1_com: str, mcu2_com: str) -> list[tuple[str, str, int]]:
    # 手册: Channel 1~4 = CAN1~CAN4；MCU1 管 CAN1/2，MCU2 管 CAN3/4
    return [
        ("CAN1", mcu1_com, 0x01),
        ("CAN2", mcu1_com, 0x02),
        ("CAN3", mcu2_com, 0x03),
        ("CAN4", mcu2_com, 0x04),
    ]


def build_et_frame(channel: int, can_id: int, data: bytes) -> bytes:
    data = data[:8]
    frame = bytearray(ET_HEADER)
    frame.append(channel & 0xFF)
    frame.extend(can_id.to_bytes(4, byteorder="big"))
    frame.append(len(data))
    frame.extend(data)
    frame.extend(ET_TAIL)
    return bytes(frame)


def parse_et_response(raw: bytes) -> list[tuple[int, int, bytes]]:
    """返回 [(channel, can_id, data), ...]"""
    out = []
    for frame in iter_et_frames(raw):
        if len(frame) < 10:
            continue
        ch = frame[2]
        can_id = int.from_bytes(frame[3:7], byteorder="big")
        dlc = frame[7]
        data = frame[8 : 8 + dlc]
        out.append((ch, can_id, data))
    return out


def lingzu_ping_frame(motor_id: int) -> tuple[int, bytes]:
    """灵足 type0 查询设备 ID。"""
    can_id = build_standard_can_id(COMM_GET_DEVICE_ID, motor_id)
    data = bytes([0x01, 0x00])  # init 短数据，DLC=2
    return can_id, data


def scan_can_port(
    ser: serial.Serial,
    port_name: str,
    can_label: str,
    channel: int,
    wait_ms: float,
) -> list[dict]:
    hits = []
    seen = set()
    for mid in PROBE_IDS:
        can_id, data = lingzu_ping_frame(mid)
        ser.reset_input_buffer()
        ser.write(build_et_frame(channel, can_id, data))
        ser.flush()
        time.sleep(wait_ms / 1000.0)
        raw = b""
        deadline = time.time() + wait_ms / 1000.0 * 2
        while time.time() < deadline:
            if ser.in_waiting:
                raw += ser.read(ser.in_waiting)
            else:
                time.sleep(0.005)
        if ser.in_waiting:
            raw += ser.read(ser.in_waiting)

        for rx_ch, rx_cid, rx_data in parse_et_response(raw):
            dev = decode_device_id_response(rx_cid, rx_data)
            if not dev:
                continue
            motor_id, uid = dev
            key = (motor_id, uid.hex())
            if key in seen:
                continue
            seen.add(key)
            hits.append(
                {
                    "can_port": can_label,
                    "com": port_name,
                    "tx_channel": channel,
                    "rx_channel": rx_ch,
                    "probe_id": mid,
                    "motor_id": motor_id,
                    "uuid": uid.hex(),
                }
            )
    return hits


def main() -> int:
    ap = argparse.ArgumentParser(description="SANPO ET 协议扫描 CAN1~CAN4")
    ap.add_argument(
        "--ports",
        type=parse_ports,
        default=parse_ports(f"{MCU1_COM},{MCU2_COM}"),
        help="双 MCU 串口，顺序为 MCU1(CAN1/2),MCU2(CAN3/4)",
    )
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--wait-ms", type=float, default=120.0)
    args = ap.parse_args()
    mcu1_com, mcu2_com = args.ports
    can_routes = build_can_routes(mcu1_com, mcu2_com)

    print("SANPO CAN1~CAN4 扫描 (ET 协议 + 灵足 type0 ping)")
    print(f"探测 ID: {PROBE_IDS}\n")
    print(f"MCU1(CAN1/2): {mcu1_com}")
    print(f"MCU2(CAN3/4): {mcu2_com}\n")

    # 按 COM 分组打开串口
    ports_needed = {mcu1_com, mcu2_com}
    serials: dict[str, serial.Serial] = {}
    for port in ports_needed:
        try:
            s = serial.Serial(
                port, args.baudrate, timeout=0.05, dsrdtr=False, rtscts=False
            )
            s.dtr = True
            s.rts = False
            time.sleep(0.05)
            s.reset_input_buffer()
            s.write(ENTER_ET_MODE_COMMAND)
            s.flush()
            time.sleep(0.15)
            mode_response = s.read(s.in_waiting) if s.in_waiting else b""
            if b"OK" not in mode_response.upper():
                print(
                    f"{port} 未确认 AT+ET 模式，回包="
                    f"{mode_response.hex() if mode_response else '(empty)'}"
                )
                s.close()
                continue
            serials[port] = s
            print(f"已打开 {port}，ET 模式已确认")
        except (serial.SerialException, OSError) as e:
            print(f"无法打开 {port}: {e}")

    if not serials:
        return 1

    all_hits: dict[str, list[dict]] = {label: [] for label, _, _ in can_routes}

    try:
        for can_label, com, ch in can_routes:
            ser = serials.get(com)
            if not ser:
                print(f"\n[{can_label}] 跳过 — {com} 未打开")
                continue
            hits = scan_can_port(ser, com, can_label, ch, args.wait_ms)
            all_hits[can_label] = hits
    finally:
        for s in serials.values():
            s.close()

    print(f"\n{'='*72}")
    print(f"{'CAN口':<6} {'COM':<6} {'Channel':<8} {'motor_id':<8} {'UUID':<20} {'状态'}")
    print("-" * 72)

    total = 0
    for can_label, _, _ in can_routes:
        hits = all_hits[can_label]
        if not hits:
            print(f"{can_label:<6} {'—':<6} {'—':<8} {'—':<8} {'—':<20} 无电机响应")
            continue
        for i, h in enumerate(hits):
            total += 1
            ch_info = f"TX={h['tx_channel']} RX={h['rx_channel']}"
            com = h["com"] if i == 0 else ""
            label = can_label if i == 0 else ""
            print(
                f"{label:<6} {com:<6} {ch_info:<8} {h['motor_id']:<8} "
                f"{h['uuid']:<20} 在线 (probe={h['probe_id']})"
            )

    print("-" * 72)
    ok_ports = sum(1 for label, _, _ in can_routes if all_hits[label])
    print(f"汇总: {total} 台电机在线，{ok_ports}/4 路 CAN 有响应")

    for can_label, _, _ in can_routes:
        n = len(all_hits[can_label])
        mark = "OK" if n >= 1 else "—"
        print(f"  {can_label}: {n} 台  [{mark}]")

    if ok_ports == 4 and total >= 16:
        print("\n结论: CAN1~CAN4 均有电机，16 路配置完整。")
        return 0
    if ok_ports == 4:
        print(f"\n结论: 4 路 CAN 均有响应，但总数 {total} < 16，请检查每路是否 4 台。")
        return 0
    print(f"\n结论: 仅 {ok_ports}/4 路 CAN 有响应，请检查未响应路的接线与供电。")
    return 1 if ok_ports < 4 else 0


if __name__ == "__main__":
    raise SystemExit(main())
