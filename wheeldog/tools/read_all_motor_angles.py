#!/usr/bin/env python3
"""Monitor all 16 motors' raw single-turn absolute angles without enabling them.

The tool uses the WDP4 calibration-readonly path. Each MCU polls both of its
physical CAN buses and places the unmodified RobStride ``mech_pos`` (0x7019)
value in ``feedback.q``. No position, velocity, torque, run-mode, or enable
command is sent.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MOTOR_MODULE_ROOT = PROJECT_ROOT / "third_party" / "rs06_mit_control"
sys.path.insert(0, str(MOTOR_MODULE_ROOT))

from wd_mcu_protocol import (  # noqa: E402
    CONTROL_FLAG_CALIBRATION_READONLY,
    CONTROL_FLAG_DRY_RUN,
    PACKET_ESTOP,
    PACKET_FEEDBACK,
    PACKET_HELLO,
    PacketParser,
    ZERO_COMMANDS,
    build_packet,
    build_setpoint_packet,
    parse_feedback,
)


DEFAULT_PERIOD_RAD = 2.0 * math.pi


@dataclass(frozen=True)
class MotorSlot:
    logical_index: int
    joint_name: str
    can_index: int
    motor_id: int

    @property
    def can_label(self) -> str:
        return f"CAN{self.can_index + 1}"


def _make_motor_slots() -> tuple[MotorSlot, ...]:
    slots: list[MotorSlot] = []
    for leg_index, leg in enumerate(("FL", "FR", "HL", "HR")):
        for offset, suffix in enumerate(("HipX", "HipY", "Knee", "Ankle")):
            slots.append(
                MotorSlot(
                    logical_index=leg_index * 4 + offset,
                    joint_name=f"{leg}_{suffix}",
                    can_index=leg_index,
                    motor_id=offset + 1,
                )
            )
    return tuple(slots)


MOTOR_SLOTS = _make_motor_slots()


def parse_ports(text: str) -> tuple[str, str]:
    ports = tuple(part.strip() for part in text.split(",") if part.strip())
    if len(ports) != 2:
        raise argparse.ArgumentTypeError("需要恰好两个 MCU 串口")
    if ports[0] == ports[1]:
        raise argparse.ArgumentTypeError("两个 MCU 串口不能相同")
    return ports[0], ports[1]


def continuous_step(previous: float, current: float, period_rad: float) -> float:
    direct = current - previous
    if abs(direct) > period_rad * 0.5:
        return direct + round(-direct / period_rad) * period_rad
    return direct


@dataclass
class AngleState:
    raw_rad: float = 0.0
    baseline_raw_rad: float = 0.0
    delta_rad: float = 0.0
    update_count: int = 0
    last_update_monotonic: float = 0.0
    fresh: bool = False

    def update(self, raw_rad: float, now: float, period_rad: float) -> None:
        if self.update_count == 0:
            self.raw_rad = raw_rad
            self.baseline_raw_rad = raw_rad
            self.delta_rad = 0.0
        else:
            self.delta_rad += continuous_step(self.raw_rad, raw_rad, period_rad)
            self.raw_rad = raw_rad
        self.update_count += 1
        self.last_update_monotonic = now
        self.fresh = True


def positions_from_feedback(feedback: Any) -> dict[tuple[int, int], float]:
    """Extract local raw q values using the MCU's compiled CAN base."""

    positions: dict[tuple[int, int], float] = {}
    first_index = feedback.bus_base * 4
    for index in range(first_index, first_index + 8):
        joint = feedback.joints[index]
        if not joint.online or not math.isfinite(joint.q):
            continue
        can_index = index // 4
        motor_id = index % 4 + 1
        positions[(can_index, motor_id)] = joint.q
    return positions


class DualMcuAngleReader:
    def __init__(
        self,
        ports: tuple[str, str],
        baudrate: int,
        response_wait_ms: float,
    ) -> None:
        self.ports = ports
        self.baudrate = baudrate
        self.response_wait_ms = response_wait_ms
        self.serials: list[Any] = []
        self.parsers = [PacketParser(), PacketParser()]
        self.sequences = [1, 1]
        self.last_request_sequences = [0, 0]
        self.seen_bases: dict[int, str] = {}
        self.seen_feedback = False

    def open(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "缺少 pyserial；请使用 /home/gu/anaconda3/bin/python3，"
                "或安装 wheeldog/third_party/rs06_mit_control/requirements.txt"
            ) from exc

        opened: list[Any] = []
        try:
            for port in self.ports:
                kwargs = dict(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=0.0,
                    dsrdtr=False,
                    rtscts=False,
                )
                try:
                    handle = serial.Serial(exclusive=True, **kwargs)
                except (TypeError, ValueError):
                    handle = serial.Serial(**kwargs)
                handle.dtr = True
                handle.rts = False
                handle.reset_input_buffer()
                opened.append(handle)
            self.serials = opened
            for index, handle in enumerate(self.serials):
                handle.write(build_packet(PACKET_HELLO, self.sequences[index]))
                self.sequences[index] += 1
                handle.flush()
        except BaseException:
            for handle in opened:
                handle.close()
            raise

    def close(self) -> None:
        for index, handle in enumerate(self.serials):
            try:
                handle.write(build_packet(PACKET_ESTOP, self.sequences[index]))
                handle.flush()
            except OSError:
                pass
            handle.close()
        self.serials.clear()

    def __enter__(self) -> "DualMcuAngleReader":
        self.open()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def _send_readonly_setpoints(self) -> None:
        for index, handle in enumerate(self.serials):
            self.last_request_sequences[index] = self.sequences[index]
            packet = build_setpoint_packet(
                self.sequences[index],
                ZERO_COMMANDS,
                live_control=False,
                enable_request=False,
                timeout_ms=500,
                calibration_readonly=True,
            )
            self.sequences[index] += 1
            handle.write(packet)
            handle.flush()

    def read_positions(self) -> dict[tuple[int, int], float]:
        self._send_readonly_setpoints()
        positions: dict[tuple[int, int], float] = {}
        deadline = time.monotonic() + self.response_wait_ms / 1000.0
        while time.monotonic() < deadline:
            received = False
            for port_index, handle in enumerate(self.serials):
                count = handle.in_waiting
                if not count:
                    continue
                received = True
                for packet_type, _seq, payload in self.parsers[port_index].feed(
                    handle.read(count)
                ):
                    if packet_type != PACKET_FEEDBACK:
                        continue
                    feedback = parse_feedback(payload)
                    if feedback is None:
                        continue
                    self.seen_feedback = True
                    # HELLO and the preceding sweep may leave a valid feedback
                    # packet in flight. Only trust one that acknowledges this
                    # sweep's calibration-readonly setpoint.
                    if feedback.last_setpoint_seq != self.last_request_sequences[port_index]:
                        continue
                    required = CONTROL_FLAG_DRY_RUN | CONTROL_FLAG_CALIBRATION_READONLY
                    if (feedback.control_flags & required) != required:
                        raise RuntimeError(
                            f"{self.ports[port_index]} 的 MCU 固件不支持标定只读模式；"
                            "请先编译并重刷本次更新后的 MCU A/B 固件"
                        )
                    if feedback.enabled_mask:
                        raise RuntimeError(
                            f"{self.ports[port_index]} 仍报告 enabled_mask="
                            f"0x{feedback.enabled_mask:04x}，已发送急停并停止读取"
                        )
                    base = feedback.bus_base
                    previous_port = self.seen_bases.get(base)
                    if previous_port is not None and previous_port != self.ports[port_index]:
                        raise RuntimeError(
                            f"两个串口都报告 MCU CAN base={base}；请检查 MCU A/B 固件是否刷反"
                        )
                    self.seen_bases[base] = self.ports[port_index]
                    positions.update(positions_from_feedback(feedback))
            if not received:
                time.sleep(0.001)
        return positions


def render(
    states: dict[tuple[int, int], AngleState],
    now: float,
    sweep: int,
    clear_terminal: bool,
) -> None:
    if clear_terminal:
        print("\033[H\033[2J", end="")

    moving_key: Optional[tuple[int, int]] = None
    candidates = [
        (abs(state.delta_rad), key)
        for key, state in states.items()
        if state.update_count > 0
    ]
    if candidates:
        largest_delta, largest_key = max(candidates)
        if largest_delta >= 0.01:
            moving_key = largest_key

    online = sum(1 for state in states.values() if state.update_count > 0)
    fresh = sum(1 for state in states.values() if state.fresh)
    print(
        "WheelDog 16电机原始绝对角监视器  "
        f"sweep={sweep} online={online}/16 fresh={fresh}/16"
    )
    print("WDP标定只读：dry-run、未请求使能；Ctrl+C 发送急停后退出。单位 rad。")
    print("标记  CAN/ID    预期关节      raw mech_pos    相对首帧Δ    Δ(deg)    age")
    print("-" * 84)
    for slot in MOTOR_SLOTS:
        key = (slot.can_index, slot.motor_id)
        state = states[key]
        marker = " * " if key == moving_key else "   "
        route = f"{slot.can_label}/ID{slot.motor_id}"
        if state.update_count == 0:
            print(
                f"{marker}  {route:<9} {slot.joint_name:<12} "
                f"{'---':>13} {'---':>12} {'---':>9}   offline"
            )
            continue
        age = max(0.0, now - state.last_update_monotonic)
        print(
            f"{marker}  {route:<9} {slot.joint_name:<12} "
            f"{state.raw_rad:>13.7f} {state.delta_rad:>12.7f} "
            f"{math.degrees(state.delta_rad):>9.3f}   {age:>5.2f}s"
        )
    print("\n* = 自程序启动以来绝对变化量最大的 ID；关节名表示当前代码映射。")
    sys.stdout.flush()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="实时读取16台电机的原始单圈绝对角，不发送运动命令",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--ports",
        type=parse_ports,
        default=parse_ports("/dev/ttyACM0,/dev/ttyACM1"),
        help="两个 MCU 串口；程序根据 WDP base 自动识别 A/B，顺序不限",
    )
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--hz", type=float, default=5.0, help="终端刷新目标频率")
    parser.add_argument("--response-wait-ms", type=float, default=120.0)
    parser.add_argument("--period-rad", type=float, default=DEFAULT_PERIOD_RAD)
    parser.add_argument("--once", action="store_true", help="取得一份快照后退出")
    parser.add_argument("--no-clear", action="store_true", help="不清屏，逐次打印")
    parser.add_argument(
        "--confirm-supported",
        action="store_true",
        help="确认机器狗已可靠支撑，允许程序在退出时发送急停",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not args.confirm_supported:
        raise ValueError("必须增加 --confirm-supported，确认整机已可靠支撑")
    for name in ("hz", "response_wait_ms", "period_rad"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} 必须是大于0的有限数")
    if args.hz > 20.0:
        raise ValueError("--hz 不得超过20")


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
        states = {(slot.can_index, slot.motor_id): AngleState() for slot in MOTOR_SLOTS}
        period_s = 1.0 / args.hz
        sweep = 0
        consecutive_empty = 0
        clear_terminal = sys.stdout.isatty() and not args.no_clear

        print("正在打开两块 MCU，进入 WDP 标定只读模式……")
        with DualMcuAngleReader(args.ports, args.baudrate, args.response_wait_ms) as reader:
            while True:
                started = time.monotonic()
                for state in states.values():
                    state.fresh = False
                positions = reader.read_positions()
                now = time.monotonic()
                sweep += 1
                for key, raw_rad in positions.items():
                    if key in states:
                        states[key].update(raw_rad, now, args.period_rad)
                consecutive_empty = consecutive_empty + 1 if not positions else 0
                render(states, now, sweep, clear_terminal)

                if args.once and positions:
                    return 0
                if args.once and sweep >= 5:
                    raise RuntimeError("5次重试后仍未取得任何原始角度反馈")
                if consecutive_empty >= 5:
                    message = "连续5轮无任何电机角度反馈"
                    if not reader.seen_feedback:
                        message += "：两块 MCU 均无 WDP 回包，请检查 USB 端口并复位 MCU"
                    raise RuntimeError(message)
                elapsed = time.monotonic() - started
                if not args.once and elapsed < period_s:
                    time.sleep(period_s - elapsed)
    except KeyboardInterrupt:
        print("\n已停止；退出前已向两块 MCU 发送急停。")
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
