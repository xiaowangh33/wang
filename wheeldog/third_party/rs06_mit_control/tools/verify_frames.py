#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核对本工程生成的 AT 帧是否与 gamepad_hybrid_motor_control.py 一致。"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import AppConfig, MitSetpoint
from protocol import (
    build_enable_command,
    build_init_command,
    build_mit_command,
    build_set_run_mode_command,
    build_start_command,
    can2com,
    pack_candata_hex,
    build_standard_can_id,
)


def can2com_ref(candata: str) -> str:
    mapping = {"00": "00", "01": "0c", "02": "14", "03": "18", "04": "20", "12": "90"}
    h = candata[:8]
    d = candata[8:]
    return "4154" + mapping[h[0:2]] + "07e8" + mapping[h[6:8]] + d + "0d0a"


def check(name: str, got: bytes, expected_candata: str) -> None:
    exp = bytes.fromhex(can2com_ref(expected_candata))
    ok = got == exp
    print(f"[{'OK' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"  expected: {exp.hex()}")
        print(f"  got     : {got.hex()}")


def main() -> None:
    cfg = AppConfig()
    sp = MitSetpoint(kp=5.0, kd=0.3)

    check("init m1", build_init_command(1), "0000fd010100")
    check("enable m1", build_enable_command(1), "0400fd010800c4000000000000")
    check("start m1", build_start_command(1), "0300fd01080000000000000000")
    check("run_mode=0", build_set_run_mode_command(1, 0), "1200fd01" + "080570000000000000")
    check("run_mode=2", build_set_run_mode_command(1, 2), "1200fd01" + "080570000002000000")

    mit = build_mit_command(1, sp, cfg.limits)
    print(f"[INFO] MIT frame: {mit.hex()}")

    # 旧版错误 ID 对比
    wrong_enable_id = 0x0300FD00 | 0xFD00 | 1  # 曾经错误算法示意
    print(f"[INFO] 旧错误 enable id 示例: 0x{wrong_enable_id:08X}")
    print(f"[INFO] 正确 enable id: 0x{build_standard_can_id(0x04, 1):08X}")


if __name__ == "__main__":
    main()
