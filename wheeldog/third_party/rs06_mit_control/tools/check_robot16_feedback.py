#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16 电机反馈读数体检。

默认只打印计划；必须加 --armed 才会使能、发送零指令并采集反馈。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot16_interface import MOTOR_COUNT, MotorCommand, Robot16Interface


def parse_ports(text: str) -> tuple[str, str]:
    ports = [p.strip() for p in text.split(",") if p.strip()]
    if len(ports) != 2:
        raise argparse.ArgumentTypeError("需要两个串口，格式如 /dev/ttyACM0,/dev/ttyACM1")
    return ports[0], ports[1]


def zero_commands() -> list[MotorCommand]:
    return [MotorCommand() for _ in range(MOTOR_COUNT)]


@dataclass
class FeedbackStats:
    index: int
    name: str = ""
    model: str = ""
    samples: int = 0
    fresh: int = 0
    fault_or: int = 0
    min_pos: float = math.inf
    max_pos: float = -math.inf
    max_abs_vel: float = 0.0
    max_abs_torque: float = 0.0
    max_temp: float = 0.0
    online_seen: bool = False
    enabled_seen: bool = False
    finite: bool = True

    def add(self, fb) -> None:
        self.name = fb.name
        self.model = fb.model
        self.samples += 1
        self.fresh += 1 if fb.fresh else 0
        self.fault_or |= fb.fault_bits
        self.online_seen = self.online_seen or fb.online
        self.enabled_seen = self.enabled_seen or fb.enabled
        vals = (fb.position, fb.velocity, fb.torque, fb.temperature)
        self.finite = self.finite and all(math.isfinite(v) for v in vals)
        self.min_pos = min(self.min_pos, fb.position)
        self.max_pos = max(self.max_pos, fb.position)
        self.max_abs_vel = max(self.max_abs_vel, abs(fb.velocity))
        self.max_abs_torque = max(self.max_abs_torque, abs(fb.torque))
        self.max_temp = max(self.max_temp, fb.temperature)

    @property
    def fresh_rate(self) -> float:
        if self.samples <= 0:
            return 0.0
        return self.fresh / self.samples

    @property
    def pos_span(self) -> float:
        if self.min_pos == math.inf:
            return 0.0
        return self.max_pos - self.min_pos


def main() -> int:
    ap = argparse.ArgumentParser(description="16 电机反馈读数体检")
    ap.add_argument(
        "--ports",
        type=parse_ports,
        default=parse_ports("COM10,COM12"),
        help="双 MCU 串口，顺序为 MCU1(CAN1/2),MCU2(CAN3/4)",
    )
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--hz", type=float, default=400.0)
    ap.add_argument("--duration", type=float, default=2.0, help="采样秒数")
    ap.add_argument("--settle", type=float, default=0.5, help="零指令预稳定秒数")
    ap.add_argument("--min-fresh-rate", type=float, default=0.2, help="最低 fresh 比例")
    ap.add_argument("--max-temp", type=float, default=90.0, help="温度上限 °C")
    ap.add_argument("--idle-vel-warn", type=float, default=0.5, help="零指令速度提示阈值 rad/s")
    ap.add_argument("--idle-torque-warn", type=float, default=3.0, help="零指令力矩提示阈值 N.m")
    ap.add_argument("--armed", action="store_true", help="确认现场安全后，真正使能并采集反馈")
    args = ap.parse_args()

    mcu1_com, mcu2_com = args.ports
    print("16 电机反馈读数体检")
    print(f"  MCU1(CAN1/2): {mcu1_com}")
    print(f"  MCU2(CAN3/4): {mcu2_com}")
    print(f"  baudrate: {args.baudrate}")
    print(f"  采样: {args.duration:.2f}s @ {args.hz:.1f}Hz，预稳定 {args.settle:.2f}s")
    print("  单位: position=rad, velocity=rad/s, torque=N.m, temp=degC")

    if not args.armed:
        print("\n未加 --armed：只打印计划，不打开串口、不使能、不采样。")
        print("确认机械安全后运行同一命令并加 --armed。")
        return 0

    robot = Robot16Interface(
        hz=args.hz,
        baudrate=args.baudrate,
        mcu1_com=mcu1_com,
        mcu2_com=mcu2_com,
    )
    stats = [FeedbackStats(i) for i in range(MOTOR_COUNT)]
    protocol_fail = False

    try:
        robot.open()
        print("\n分批使能 + 反馈验证（无运动验证）...")
        enabled = robot.enable_all_verified(motion_verify=False)
        if not all(enabled):
            print("警告: 部分电机使能验证未通过，后续会在汇总中标为 FAIL。")

        robot.set_commands(zero_commands())
        settle_deadline = time.time() + args.settle
        while time.time() < settle_deadline:
            robot.step()

        print("\n采集零指令反馈...")
        cycles = 0
        sample_start = time.time()
        deadline = time.time() + args.duration
        while time.time() < deadline:
            robot.set_commands(zero_commands())
            fbs = robot.step()
            cycles += 1
            for i, fb in enumerate(fbs):
                stats[i].add(fb)
        sample_elapsed = time.time() - sample_start
        actual_hz = cycles / sample_elapsed if sample_elapsed > 0.0 else 0.0

        print(
            f"\n实际采样循环: {cycles} cycles / {sample_elapsed:.3f}s = "
            f"{actual_hz:.1f} Hz (target {args.hz:.1f} Hz)"
        )

        print("\nidx name      model samples fresh% pos_min(rad) pos_max(rad) span(rad) max|vel| max|tau| temp  status")
        print("-" * 112)
        for i, st in enumerate(stats):
            slot = robot.config.motors[i]
            pos_ok = abs(st.min_pos) <= slot.limits.p_max * 1.02 and abs(st.max_pos) <= slot.limits.p_max * 1.02
            vel_ok = st.max_abs_vel <= slot.limits.v_max * 1.02
            torque_ok = st.max_abs_torque <= slot.limits.t_max * 1.02
            temp_ok = st.max_temp <= args.max_temp
            fresh_ok = st.fresh_rate >= args.min_fresh_rate
            proto_ok = (
                enabled[i]
                and st.online_seen
                and st.enabled_seen
                and st.finite
                and st.fault_or == 0
                and fresh_ok
                and pos_ok
                and vel_ok
                and torque_ok
                and temp_ok
            )
            idle_warn = st.max_abs_vel > args.idle_vel_warn or st.max_abs_torque > args.idle_torque_warn
            if not proto_ok:
                protocol_fail = True
                status = "FAIL"
            elif idle_warn:
                status = "OK/WARN"
            else:
                status = "OK"
            print(
                f"{i:02d}  {st.name:<9s} {st.model:<5s} "
                f"{st.samples:7d} {st.fresh_rate * 100:6.1f} "
                f"{st.min_pos:+11.3f} {st.max_pos:+11.3f} {st.pos_span:9.4f} "
                f"{st.max_abs_vel:8.3f} {st.max_abs_torque:8.3f} {st.max_temp:5.1f}  {status}"
            )

        print("\n判定说明:")
        print("  OK      = 反馈在线、无 fault、数值有限，且在协议限幅内。")
        print("  OK/WARN = 协议反馈正常，但零指令下速度或力矩偏大；可能是负载/惯性/未静止。")
        print("  FAIL    = 反馈缺失、fault、数值越界或 fresh 比例过低。")

    except KeyboardInterrupt:
        print("\n用户中断，发送零指令并失能")
        protocol_fail = True
    finally:
        try:
            robot.set_commands(zero_commands())
            robot.step()
        except Exception:
            pass
        robot.disable_all()
        robot.close()

    print("\n已发送零指令并失能。")
    return 1 if protocol_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
