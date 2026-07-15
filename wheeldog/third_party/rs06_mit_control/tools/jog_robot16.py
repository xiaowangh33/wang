#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16 电机逐台短脉冲运动测试。

默认只打印计划；必须加 --armed 才会真正使能并下发运动命令。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot16_interface import MOTOR_COUNT, MotorCommand, Robot16Interface


def parse_ports(text: str) -> tuple[str, str]:
    ports = [p.strip() for p in text.split(",") if p.strip()]
    if len(ports) != 2:
        raise argparse.ArgumentTypeError("需要两个串口，格式如 /dev/ttyACM0,/dev/ttyACM1")
    return ports[0], ports[1]


def parse_indices(text: str) -> list[int]:
    if text.strip().lower() == "all":
        return list(range(MOTOR_COUNT))
    out: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start = int(start_s)
            end = int(end_s)
            out.extend(range(start, end + 1))
        else:
            out.append(int(part))
    bad = [i for i in out if i < 0 or i >= MOTOR_COUNT]
    if bad:
        raise argparse.ArgumentTypeError(f"电机 index 须为 0~15，收到 {bad}")
    return list(dict.fromkeys(out))


def zero_commands() -> list[MotorCommand]:
    return [MotorCommand() for _ in range(MOTOR_COUNT)]


def jog_one(
    robot: Robot16Interface,
    index: int,
    torque_nm: float,
    velocity_rad_s: float,
    pulse_s: float,
    rest_s: float,
    hz: float,
) -> None:
    fb0 = robot.get_feedback(index)
    command = zero_commands()
    if fb0.model == "rs06":
        command[index] = MotorCommand(torque_nm=torque_nm)
        cmd_desc = f"torque={torque_nm:+.3f} N.m"
    else:
        command[index] = MotorCommand(velocity_rad_s=velocity_rad_s)
        cmd_desc = f"velocity={velocity_rad_s:+.3f} rad/s"

    print(f"\n[JOG] index={index:02d} {fb0.name} ({fb0.model})  {cmd_desc}")
    max_vel = 0.0
    pos_start = fb0.position
    deadline = time.time() + pulse_s
    while time.time() < deadline:
        robot.set_commands(command)
        fbs = robot.step()
        max_vel = max(max_vel, abs(fbs[index].velocity))

    robot.set_commands(zero_commands())
    rest_deadline = time.time() + rest_s
    fb_last = robot.get_feedback(index)
    while time.time() < rest_deadline:
        fbs = robot.step()
        fb_last = fbs[index]

    pos_delta = fb_last.position - pos_start
    print(
        f"      delta_pos={pos_delta:+.4f} rad, "
        f"max_vel={max_vel:.4f} rad/s, fault=0x{fb_last.fault_bits:02x}, "
        f"fresh={fb_last.fresh}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="16 电机逐台短脉冲运动测试")
    ap.add_argument(
        "--ports",
        type=parse_ports,
        default=parse_ports("COM10,COM12"),
        help="双 MCU 串口，顺序为 MCU1(CAN1/2),MCU2(CAN3/4)",
    )
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument(
        "--indices",
        type=parse_indices,
        default=parse_indices("0"),
        help="要测试的 index，如 0、0,1,3、0-3 或 all；index=CAN序号*4+(ID-1)",
    )
    ap.add_argument("--hz", type=float, default=400.0)
    ap.add_argument("--pulse", type=float, default=0.35, help="每台运动脉冲秒数")
    ap.add_argument("--rest", type=float, default=0.35, help="每台脉冲后的零指令等待秒数")
    ap.add_argument("--torque", type=float, default=0.45, help="RS06 测试力矩 N.m")
    ap.add_argument("--velocity", type=float, default=0.25, help="RS01 测试速度 rad/s")
    ap.add_argument("--reverse", action="store_true", help="反向测试")
    ap.add_argument("--armed", action="store_true", help="确认现场安全后，真正使能并下发运动命令")
    args = ap.parse_args()

    mcu1_com, mcu2_com = args.ports
    torque = -args.torque if args.reverse else args.torque
    velocity = -args.velocity if args.reverse else args.velocity

    print("16 电机逐台短脉冲测试")
    print(f"  MCU1(CAN1/2): {mcu1_com}")
    print(f"  MCU2(CAN3/4): {mcu2_com}")
    print(f"  baudrate: {args.baudrate}")
    print(f"  indices: {args.indices}")
    print(f"  RS06 torque: {torque:+.3f} N.m")
    print(f"  RS01 velocity: {velocity:+.3f} rad/s")
    print(f"  pulse/rest: {args.pulse:.2f}s / {args.rest:.2f}s @ {args.hz:.1f}Hz")

    if not args.armed:
        print("\n未加 --armed：只打印计划，不打开串口、不使能、不运动。")
        print("确认机械固定、运动范围清空、急停可用后，再加 --armed 运行。")
        return 0

    robot = Robot16Interface(
        hz=args.hz,
        baudrate=args.baudrate,
        mcu1_com=mcu1_com,
        mcu2_com=mcu2_com,
    )
    try:
        robot.open()
        print("\n分批使能 + 反馈验证...")
        enabled = robot.enable_all_verified(motion_verify=False)
        if not all(enabled):
            print("警告: 部分电机使能验证未通过；脚本只会对 enabled 的电机下发命令。")

        robot.set_commands(zero_commands())
        for _ in range(3):
            robot.step()

        for index in args.indices:
            if not robot.get_feedback(index).enabled:
                print(f"\n[SKIP] index={index:02d} 未使能验证通过")
                continue
            jog_one(robot, index, torque, velocity, args.pulse, args.rest, args.hz)

        robot.set_commands(zero_commands())
        for _ in range(3):
            robot.step()
    except KeyboardInterrupt:
        print("\n用户中断，发送零指令并失能")
    finally:
        try:
            robot.set_commands(zero_commands())
            robot.step()
        except Exception:
            pass
        robot.disable_all()
        robot.close()

    print("\n测试结束，已发送零指令并失能。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
