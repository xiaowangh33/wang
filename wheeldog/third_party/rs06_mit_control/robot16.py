#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16 电机：探测连接、批量改 ID、MIT 控制。

用法:
  py robot16.py list-ports
  py tools/probe_robot16.py              # 扫描所有串口找回传
  py robot16.py ping --port COM10
  py robot16.py config --port COM10 --auto-all
  py robot16.py control --port COM10 --duration 3 --ping-only
"""

from __future__ import annotations

import argparse
import sys

try:
    import serial.tools.list_ports as list_ports
except ImportError:
    list_ports = None

from motors16_config import DEFAULT_FACTORY_ID, default_robot16
from multi_can16 import Robot16Controller


def cmd_list_ports(_: argparse.Namespace) -> int:
    if list_ports is None:
        print("请安装 pyserial: pip install pyserial")
        return 1
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口")
        return 1
    for p in ports:
        print(f"  {p.device}\t{p.description}")
    return 0


def _make_controller(args: argparse.Namespace) -> Robot16Controller:
    cfg = default_robot16()
    cfg.serial.port = args.port
    cfg.serial.baudrate = args.baudrate
    cfg.transport_mode = args.transport
    if hasattr(args, "hz"):
        cfg.loop_hz = args.hz
    return Robot16Controller(cfg)


def cmd_ping(args: argparse.Namespace) -> int:
    ctrl = _make_controller(args)
    try:
        ctrl.connect()
        print(f"\n探测 16 电机 (transport={args.transport})...\n")
        results = ctrl.ping_all(verbose=True)
        ctrl.ping_summary_by_channel()

        if args.scan_factory:
            print(f"\n扫描各通道出厂 ID={DEFAULT_FACTORY_ID} 的 UUID...")
            for ch in range(4):
                found = ctrl.discover_on_channel(ch, probe_ids=[DEFAULT_FACTORY_ID, 127], rounds=15)
                print(f"  CAN{ch + 1}: 发现 {len(found)} 个 UUID")
                for uid, m in found.items():
                    print(f"    id={m.motor_id} uid={uid.hex()}")

        online = sum(1 for r in results if r.online)
        return 0 if online > 0 else 1
    finally:
        ctrl.disconnect()


def cmd_config(args: argparse.Namespace) -> int:
    del args
    print(
        "已禁用项目内改ID功能：实机证明同一旧ID的多台电机会同时改号，"
        "UUID不能用于单台筛选。请使用电机开发者的原厂上位机。"
    )
    return 2


def cmd_control(args: argparse.Namespace) -> int:
    ctrl = _make_controller(args)
    try:
        ctrl.connect()
        if args.ping_only:
            ctrl.ping_all(verbose=True)
            return 0
        if args.enable_only:
            ctrl.enable_all()
            input("已使能 16 电机，按 Enter 失能...")
            ctrl.disable_all()
            return 0
        ctrl.run_loop(duration_s=args.duration)
        return 0
    finally:
        ctrl.disconnect()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="SANPO 16 电机探测 / 改 ID / 控制")
    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("list-ports", help="列出串口")

    ping_p = sub.add_parser("ping", help="探测 16 电机是否在线")
    ping_p.add_argument("--port", default="COM10")
    ping_p.add_argument("--baudrate", type=int, default=921600)
    ping_p.add_argument(
        "--transport",
        choices=["sanpo", "sanpo_prefix", "lingzu"],
        default="sanpo",
        help="sanpo=SANPO板(AT发/ET收); sanpo_prefix=发送加通道字节; lingzu=官方USB-CAN",
    )
    ping_p.add_argument(
        "--scan-factory",
        action="store_true",
        help="额外扫描各通道出厂 ID 的 UUID",
    )

    cfg_p = sub.add_parser("config", help="批量或单台改 CAN ID")
    cfg_p.add_argument("--port", default="COM10")
    cfg_p.add_argument("--baudrate", type=int, default=921600)
    cfg_p.add_argument("--transport", choices=["sanpo", "sanpo_prefix", "lingzu"], default="sanpo")
    cfg_p.add_argument("--auto-all", action="store_true", help="4 路 CAN 全自动改 ID")
    cfg_p.add_argument("--factory-id", type=int, default=DEFAULT_FACTORY_ID)
    cfg_p.add_argument("--channel", type=int, choices=[0, 1, 2, 3])
    cfg_p.add_argument("--old-id", type=int, default=DEFAULT_FACTORY_ID)
    cfg_p.add_argument("--new-id", type=int)
    cfg_p.add_argument("--uuid", help="16 进制 UUID，来自 ping --scan-factory")

    ctl_p = sub.add_parser("control", help="16 电机 MIT 控制")
    ctl_p.add_argument("--port", default="COM10")
    ctl_p.add_argument("--baudrate", type=int, default=921600)
    ctl_p.add_argument("--transport", choices=["sanpo", "sanpo_prefix", "lingzu"], default="sanpo")
    ctl_p.add_argument("--hz", type=float, default=50.0)
    ctl_p.add_argument("--duration", type=float, default=5.0)
    ctl_p.add_argument("--ping-only", action="store_true")
    ctl_p.add_argument("--enable-only", action="store_true")

    return p


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    handlers = {
        "list-ports": cmd_list_ports,
        "ping": cmd_ping,
        "config": cmd_config,
        "control": cmd_control,
    }
    return handlers[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
