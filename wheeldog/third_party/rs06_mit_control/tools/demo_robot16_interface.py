#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""演示 Robot16Interface：400Hz 目标频率下发/接收，RS06 力矩 (N·m)，RS01 速度 (rad/s)。"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot16_interface import MotorCommand, Robot16Interface


def parse_ports(text: str) -> tuple[str, str]:
    ports = [p.strip() for p in text.split(",") if p.strip()]
    if len(ports) != 2:
        raise argparse.ArgumentTypeError("需要两个串口，格式如 /dev/ttyACM0,/dev/ttyACM1")
    return ports[0], ports[1]


def main() -> int:
    ap = argparse.ArgumentParser(description="16 电机控制/反馈接口演示")
    ap.add_argument(
        "--ports",
        type=parse_ports,
        default=parse_ports("COM10,COM12"),
        help="双 MCU 串口，顺序为 MCU1(CAN1/2),MCU2(CAN3/4)",
    )
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--hz", type=float, default=400.0)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument(
        "--torque",
        type=float,
        default=None,
        help="RS06 恒定力矩 N·m（默认正弦波；0.2 过小可能不转，建议 ≥0.6）",
    )
    ap.add_argument("--velocity", type=float, default=None, help="RS01 恒定速度 rad/s")
    ap.add_argument("--enable", action="store_true", help="运行前使能全部电机")
    ap.add_argument(
        "--verified-enable",
        action="store_true",
        help="使用 enable_all_verified（反馈验证+重试，无运动测试）",
    )
    ap.add_argument(
        "--motion-verify",
        action="store_true",
        help="使能时用 0.6N·m 做运动验证（电机会短暂动一下）",
    )
    ap.add_argument("--auto-recover", action="store_true", help="运行中自动单台恢复")
    args = ap.parse_args()
    mcu1_com, mcu2_com = args.ports

    robot = Robot16Interface(
        hz=args.hz,
        baudrate=args.baudrate,
        mcu1_com=mcu1_com,
        mcu2_com=mcu2_com,
    )
    robot.open()
    print(f"Robot16Interface 已连接，控制频率 {args.hz} Hz")
    print(f"  baudrate: {args.baudrate}")
    print(f"  MCU1(CAN1/2): {mcu1_com}")
    print(f"  MCU2(CAN3/4): {mcu2_com}")
    print("  RS06: 下发力矩 N·m（建议 ≥0.6 才能克服静摩擦）")
    print("  RS01: 下发速度 rad/s")

    if args.auto_recover:
        robot.set_auto_recover(True)

    if args.enable or args.verified_enable:
        if args.verified_enable:
            print("\n分批使能 + 反馈验证...\n")
            results = robot.enable_all_verified(motion_verify=args.motion_verify)
            if not all(results):
                print("警告: 部分电机使能验证未通过")
        else:
            print("使能 16 电机（无验证）...")
            robot.enable_all()
        time.sleep(0.3)

    t0 = time.time()
    n_print = 0
    try:
        while time.time() - t0 < args.duration:
            t = time.time() - t0
            if args.torque is not None:
                torque = args.torque
            else:
                torque = 0.6 * math.sin(2 * math.pi * 0.5 * t)

            vel = args.velocity if args.velocity is not None else 0.3

            cmds = []
            for fb in robot.get_all_feedback():
                if fb.model == "rs06":
                    cmds.append(MotorCommand(torque_nm=torque))
                else:
                    cmds.append(MotorCommand(velocity_rad_s=vel))
            robot.set_commands(cmds)

            fbs = robot.step()
            n_print += 1
            if n_print % int(max(1, args.hz)) == 0:
                online = sum(1 for f in fbs if f.online)
                enabled = sum(1 for f in fbs if f.enabled)
                print(
                    f"[{t:5.2f}s] cycle={robot.cycle_count} "
                    f"online={online}/16 enabled={enabled}/16  "
                    f"can1_m1: pos={fbs[0].position:+.3f} vel={fbs[0].velocity:+.3f} "
                    f"tau={fbs[0].torque:+.3f}  can1_m4: vel={fbs[3].velocity:+.3f}"
                )
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        if args.enable or args.verified_enable:
            robot.disable_all()
        robot.close()

    print(f"完成，共 {robot.cycle_count} 个控制周期")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
