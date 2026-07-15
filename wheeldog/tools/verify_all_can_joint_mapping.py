#!/usr/bin/env python3
"""Verify four CAN buses and print the expected 16 joint/index mapping."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from read_all_motor_angles import (  # noqa: E402
    MOTOR_SLOTS,
    DualMcuAngleReader,
    parse_ports,
)


def evaluate_positions(
    positions: dict[tuple[int, int], float],
) -> tuple[bool, dict[int, list[int]]]:
    ids_by_can = {
        can_index: sorted(
            motor_id
            for (seen_can, motor_id) in positions
            if seen_can == can_index
        )
        for can_index in range(4)
    }
    passed = all(ids_by_can[index] == [1, 2, 3, 4] for index in range(4))
    return passed, ids_by_can


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="检查CAN1~CAN4的ID1~4，并打印逻辑序号和预期关节映射",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--ports",
        type=parse_ports,
        default=parse_ports("/dev/ttyACM0,/dev/ttyACM1"),
        help="两个 MCU 串口，A/B 顺序不限",
    )
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--response-wait-ms", type=float, default=120.0)
    parser.add_argument("--attempts", type=int, default=5, help="聚合只读轮询次数")
    parser.add_argument(
        "--confirm-supported",
        action="store_true",
        help="确认整机已可靠支撑，允许退出时发送急停",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not args.confirm_supported:
        raise ValueError("必须增加 --confirm-supported，确认整机已可靠支撑")
    if args.attempts < 1 or args.attempts > 50:
        raise ValueError("--attempts 必须在 1~50 之间")
    if not math.isfinite(args.response_wait_ms) or args.response_wait_ms <= 0.0:
        raise ValueError("--response-wait-ms 必须是大于0的有限数")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        validate_args(args)
        positions: dict[tuple[int, int], float] = {}
        with DualMcuAngleReader(
            args.ports,
            args.baudrate,
            args.response_wait_ms,
        ) as reader:
            for _ in range(args.attempts):
                positions.update(reader.read_positions())
                if len(positions) == len(MOTOR_SLOTS):
                    break
                time.sleep(0.02)
            bases = dict(reader.seen_bases)

        passed, ids_by_can = evaluate_positions(positions)
        identity_ok = set(bases) == {0, 2}
        print("WheelDog 四路 CAN / 16 电机映射检查")
        print(f"MCU identity: {bases if bases else '无反馈'}")
        print("序号  CAN/ID    预期关节       raw mech_pos      状态")
        print("-" * 66)
        for slot in MOTOR_SLOTS:
            key = (slot.can_index, slot.motor_id)
            value = positions.get(key)
            route = f"{slot.can_label}/ID{slot.motor_id}"
            if value is None:
                print(
                    f"{slot.logical_index:>4}  {route:<9} "
                    f"{slot.joint_name:<12} {'---':>14}      OFFLINE"
                )
            else:
                print(
                    f"{slot.logical_index:>4}  {route:<9} "
                    f"{slot.joint_name:<12} {value:>14.7f}      ONLINE"
                )

        print("\n每路唯一 ID：")
        for can_index in range(4):
            ids = ids_by_can[can_index]
            status = "OK" if ids == [1, 2, 3, 4] else "FAIL"
            shown = ",".join(str(item) for item in ids) if ids else "—"
            print(f"  CAN{can_index + 1}: [{shown}]  {status}")

        if passed and identity_ok:
            print("\nPASS: 双 MCU 身份正确，CAN1~CAN4 均有唯一 ID1~4，共16槽在线。")
            return 0
        print("\nFAIL: 地址或 MCU 身份不完整；不要进入标定或运动测试。")
        return 1
    except KeyboardInterrupt:
        print("\n已停止；退出前已发送急停。")
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
