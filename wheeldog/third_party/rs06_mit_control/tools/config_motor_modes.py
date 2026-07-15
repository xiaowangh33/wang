#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""配置 CAN1~CAN4 上 16 台电机运行模式并保存到 Flash。

默认布局（与 motors16_config 一致）：
  每路 CAN：ID 1~3 (RS06) → run_mode=3 力矩/电流模式
            ID 4   (RS01) → run_mode=2 速度模式
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protocol import (
    PARAM_RUN_MODE,
    RUN_MODE_CURRENT,
    RUN_MODE_SPEED,
    build_disable_command,
    build_init_command,
    build_read_param_command,
    build_save_params_command,
    build_set_accel_command,
    build_set_iq_ref_command,
    build_set_run_mode_command,
    build_set_spd_ref_command,
    decode_read_param_u32,
)
from sanpo_bus_map import MCU1_COM, MCU2_COM
from sanpo_et import SanpoEtBus

MODE_NAMES = {
    RUN_MODE_CURRENT: "力矩/电流(3)",
    RUN_MODE_SPEED: "速度(2)",
}


def target_run_mode(motor_id: int) -> int:
    if motor_id in (1, 2, 3):
        return RUN_MODE_CURRENT
    if motor_id == 4:
        return RUN_MODE_SPEED
    raise ValueError(f"不支持的 motor_id={motor_id}")


def parse_ports(text: str) -> tuple[str, str]:
    ports = [p.strip() for p in text.split(",") if p.strip()]
    if len(ports) != 2:
        raise argparse.ArgumentTypeError("需要两个串口，格式如 /dev/ttyACM0,/dev/ttyACM1")
    return ports[0], ports[1]


def configure_one(
    bus: SanpoEtBus,
    can_label: str,
    channel: int,
    motor_id: int,
    save: bool,
    verify: bool,
) -> bool:
    run_mode = target_run_mode(motor_id)
    mode_name = MODE_NAMES[run_mode]
    prefix = f"{can_label} id={motor_id}"

    bus.send_can_label(can_label, build_disable_command(motor_id), wait_ms=60)
    time.sleep(0.03)
    bus.send_can_label(can_label, build_init_command(motor_id), wait_ms=60)
    time.sleep(0.03)
    bus.send_can_label(
        can_label, build_set_run_mode_command(motor_id, run_mode), wait_ms=80
    )
    time.sleep(0.03)

    if run_mode == RUN_MODE_CURRENT:
        bus.send_can_label(can_label, build_set_iq_ref_command(motor_id, 0.0), wait_ms=60)
    else:
        bus.send_can_label(can_label, build_set_spd_ref_command(motor_id, 0.0), wait_ms=60)
        bus.send_can_label(can_label, build_set_accel_command(motor_id, 10.0), wait_ms=60)

    if save:
        bus.send_can_label(can_label, build_save_params_command(motor_id), wait_ms=120)
        time.sleep(0.1)

    actual: int | None = None
    if verify:
        rx = bus.send_can_label(
            can_label, build_read_param_command(motor_id, PARAM_RUN_MODE), wait_ms=100
        )
        for _, can_id, data in rx:
            val = decode_read_param_u32(can_id, data, PARAM_RUN_MODE)
            if val is not None:
                actual = val
                break

    ok = actual == run_mode if verify and actual is not None else True
    status = "OK" if ok else "FAIL"
    read_info = f"读回 run_mode={actual}" if actual is not None else "读回超时"
    print(f"  [{prefix}] 设置 {mode_name}  save={'Y' if save else 'N'}  {read_info}  [{status}]")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="配置 16 电机 run_mode 并保存")
    ap.add_argument(
        "--ports",
        type=parse_ports,
        default=parse_ports(f"{MCU1_COM},{MCU2_COM}"),
        help=(
            "双 MCU 串口，顺序必须为 MCU1(CAN1/2),MCU2(CAN3/4)，"
            "例如 /dev/ttyACM0,/dev/ttyACM1"
        ),
    )
    ap.add_argument("--no-save", action="store_true", help="仅写入 RAM，不保存 Flash")
    ap.add_argument("--no-verify", action="store_true", help="不读回 run_mode 验证")
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--wait-ms", type=float, default=80.0)
    args = ap.parse_args()

    save = not args.no_save
    verify = not args.no_verify
    mcu1_com, mcu2_com = args.ports

    print("16 电机模式配置")
    print("  ID 1~3 (RS06): run_mode=3 力矩/电流模式")
    print("  ID 4   (RS01): run_mode=2 速度模式")
    print(f"  保存 Flash: {'是' if save else '否'}")
    print(f"  MCU1(CAN1/2): {mcu1_com}")
    print(f"  MCU2(CAN3/4): {mcu2_com}")
    print(f"  baudrate: {args.baudrate}")
    print()

    bus = SanpoEtBus(
        baudrate=args.baudrate,
        wait_ms=args.wait_ms,
        mcu1_com=mcu1_com,
        mcu2_com=mcu2_com,
    )
    try:
        bus.open()
        print(f"已打开 {mcu1_com} / {mcu2_com} @ {bus.baudrate}\n")

        ok_all = True
        for can_label, _com, channel in bus.can_routes:
            print(f"=== {can_label} (channel={channel}) ===")
            for mid in (1, 2, 3, 4):
                ok = configure_one(
                    bus, can_label, channel, mid, save=save, verify=verify
                )
                ok_all = ok_all and ok
                time.sleep(0.05)
            print()

        if ok_all:
            print("全部配置完成。")
            return 0
        print("部分电机配置或验证失败，请检查接线和供电。")
        return 1
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
