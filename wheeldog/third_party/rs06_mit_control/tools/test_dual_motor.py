#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双电机同时力控测试。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import AppConfig
from multi_motor import RS06Bus, parse_ids


def main() -> int:
    ap = argparse.ArgumentParser(description="双电机力控测试")
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--motor-ids", default="1,127", help="逗号分隔，如 1,127")
    ap.add_argument("--iq1", type=float, default=1.0, help="电机1峰值电流 Apeak")
    ap.add_argument(
        "--iq2",
        type=float,
        default=-1.0,
        help="电机2峰值电流 Apeak（反向）",
    )
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--ping-only", action="store_true")
    args = ap.parse_args()

    ids = parse_ids(args.motor_ids)
    cfg = AppConfig()
    cfg.control.loop_hz = args.hz

    bus = RS06Bus(cfg.serial.port, ids, cfg.serial.baudrate, cfg.limits, cfg.control)
    try:
        bus.connect()
        print(f"探测电机 {ids} ...")
        hits = bus.ping_all()
        for mid, ok in hits.items():
            print(f"  ID={mid}: {'在线' if ok else '无响应'}")

        if args.ping_only:
            return 0

        if len(ids) >= 1:
            bus.channels[ids[0]].current.iq = args.iq1
        if len(ids) >= 2:
            bus.channels[ids[1]].current.iq = args.iq2

        bus.run_current_loop(duration_s=args.duration)
    finally:
        bus.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
