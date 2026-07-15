#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared-memory bridge between the C++ wheeldog controller and Robot16Interface.

The bridge owns the Python motor driver and runs the 16-motor bus loop at the
requested low-level frequency. The C++ side computes PD torques and writes raw
RS06 torque commands (N.m) plus RS01 wheel velocity commands (rad/s).
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import signal
import sys
import time
from multiprocessing import shared_memory
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from robot16_interface import MotorCommand, Robot16Interface  # noqa: E402

MOTOR_COUNT = 16
MAGIC = 0x57444D42
VERSION = 1

CONTROL_EXIT = 1 << 0

STATUS_READY = 1 << 0
STATUS_OPEN = 1 << 1
STATUS_ENABLED = 1 << 2
STATUS_FAULT = 1 << 3
DEFAULT_COMMAND_TORQUE_LIMIT_NM = 1.0
DEFAULT_COMMAND_SPEED_LIMIT_RAD_S = 4.0


def read_limit_env(name: str, default: float, upper: float) -> float:
    text = os.getenv(name)
    if not text:
        return default
    try:
        value = float(text)
    except ValueError:
        return default
    if not math.isfinite(value) or value < 0.0 or value > upper:
        return default
    return value


COMMAND_TORQUE_LIMIT_NM = read_limit_env(
    "WHEELDOG_MOTOR_MAX_TORQUE_NM",
    DEFAULT_COMMAND_TORQUE_LIMIT_NM,
    36.0,
)
COMMAND_SPEED_LIMIT_RAD_S = read_limit_env(
    "WHEELDOG_MOTOR_MAX_SPEED_RAD_S",
    DEFAULT_COMMAND_SPEED_LIMIT_RAD_S,
    50.0,
)


class BridgeShm(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("motor_count", ctypes.c_uint32),
        ("command_seq", ctypes.c_uint32),
        ("feedback_seq", ctypes.c_uint32),
        ("control_flags", ctypes.c_uint32),
        ("status_flags", ctypes.c_uint32),
        ("enabled_mask", ctypes.c_uint32),
        ("online_mask", ctypes.c_uint32),
        ("fresh_mask", ctypes.c_uint32),
        ("fault_mask", ctypes.c_uint32),
        ("command_torque", ctypes.c_float * MOTOR_COUNT),
        ("command_velocity", ctypes.c_float * MOTOR_COUNT),
        ("feedback_position", ctypes.c_float * MOTOR_COUNT),
        ("feedback_velocity", ctypes.c_float * MOTOR_COUNT),
        ("feedback_torque", ctypes.c_float * MOTOR_COUNT),
        ("feedback_temperature", ctypes.c_float * MOTOR_COUNT),
        ("feedback_fault_bits", ctypes.c_uint32 * MOTOR_COUNT),
        ("reserved0", ctypes.c_uint32),
        ("bridge_time_sec", ctypes.c_double),
        ("actual_hz", ctypes.c_double),
        ("cycle_count", ctypes.c_uint64),
        ("status_message", ctypes.c_char * 256),
    ]


assert ctypes.sizeof(BridgeShm) == 776


def parse_ports(text: str) -> tuple[str, str]:
    ports = [p.strip() for p in text.split(",") if p.strip()]
    if len(ports) != 2:
        raise argparse.ArgumentTypeError("需要两个串口，格式如 /dev/ttyACM0,/dev/ttyACM1")
    return ports[0], ports[1]


def set_status_message(shm: BridgeShm, text: str) -> None:
    data = text.encode("utf-8", errors="replace")[:255]
    shm.status_message = data + b"\0" * (256 - len(data))


def finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def clip_abs(value: float, limit: float) -> float:
    value = finite_or_zero(value)
    return max(-limit, min(limit, value))


def mask_bit(index: int) -> int:
    return 1 << index


def update_feedback(shm: BridgeShm, fbs) -> None:
    online_mask = 0
    fresh_mask = 0
    fault_mask = 0
    enabled_mask = 0
    for fb in fbs:
        i = fb.index
        shm.feedback_position[i] = float(fb.position)
        shm.feedback_velocity[i] = float(fb.velocity)
        shm.feedback_torque[i] = float(fb.torque)
        shm.feedback_temperature[i] = float(fb.temperature)
        shm.feedback_fault_bits[i] = int(fb.fault_bits)
        if fb.online:
            online_mask |= mask_bit(i)
        if fb.fresh:
            fresh_mask |= mask_bit(i)
        if fb.fault_bits:
            fault_mask |= mask_bit(i)
        if fb.enabled:
            enabled_mask |= mask_bit(i)
    shm.online_mask = online_mask
    shm.fresh_mask = fresh_mask
    shm.fault_mask = fault_mask
    shm.enabled_mask = enabled_mask
    shm.feedback_seq += 1


def build_commands(shm: BridgeShm) -> list[MotorCommand]:
    commands: list[MotorCommand] = []
    for i in range(MOTOR_COUNT):
        commands.append(
            MotorCommand(
                torque_nm=clip_abs(float(shm.command_torque[i]), COMMAND_TORQUE_LIMIT_NM),
                velocity_rad_s=clip_abs(
                    float(shm.command_velocity[i]),
                    COMMAND_SPEED_LIMIT_RAD_S,
                ),
            )
        )
    return commands


def zero_commands(robot: Robot16Interface, cycles: int = 6) -> None:
    commands = [MotorCommand() for _ in range(MOTOR_COUNT)]
    for _ in range(cycles):
        robot.set_commands(commands)
        robot.step()


def main() -> int:
    ap = argparse.ArgumentParser(description="wheeldog C++ <-> Robot16Interface bridge")
    ap.add_argument("--shm-name", required=True)
    ap.add_argument("--ports", type=parse_ports, default=parse_ports("/dev/ttyACM0,/dev/ttyACM1"))
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--hz", type=float, default=400.0)
    ap.add_argument("--enable", action="store_true")
    ap.add_argument("--verified-enable", action="store_true")
    ap.add_argument("--motion-verify", action="store_true")
    args = ap.parse_args()

    stop_requested = False

    def handle_signal(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    shm_obj = shared_memory.SharedMemory(name=args.shm_name)
    shm = BridgeShm.from_buffer(shm_obj.buf)
    robot: Robot16Interface | None = None

    if shm.magic != MAGIC or shm.version != VERSION or shm.motor_count != MOTOR_COUNT:
        raise RuntimeError("shared-memory header mismatch")

    t_start = time.time()
    last_hz_time = t_start
    last_hz_cycle = 0

    try:
        mcu1_com, mcu2_com = args.ports
        set_status_message(
            shm,
            "command safety: "
            f"tau<={COMMAND_TORQUE_LIMIT_NM:.3f}Nm "
            f"vel<={COMMAND_SPEED_LIMIT_RAD_S:.3f}rad/s",
        )
        set_status_message(shm, "opening Robot16Interface")
        robot = Robot16Interface(
            hz=args.hz,
            baudrate=args.baudrate,
            mcu1_com=mcu1_com,
            mcu2_com=mcu2_com,
        )
        robot.open()
        shm.status_flags |= STATUS_OPEN

        if args.enable or args.verified_enable:
            set_status_message(shm, "enabling motors")
            if args.verified_enable:
                results = robot.enable_all_verified(motion_verify=args.motion_verify)
            else:
                robot.enable_all()
                results = [True] * MOTOR_COUNT
            if not all(results):
                failed = [str(i) for i, ok in enumerate(results) if not ok]
                raise RuntimeError("motor enable failed: " + ",".join(failed))
            shm.status_flags |= STATUS_ENABLED

        set_status_message(shm, "bridge ready")
        shm.status_flags |= STATUS_READY

        while not stop_requested and not (shm.control_flags & CONTROL_EXIT):
            commands = build_commands(shm)
            robot.set_commands(commands)
            fbs = robot.step()
            update_feedback(shm, fbs)
            shm.bridge_time_sec = time.time() - t_start
            shm.cycle_count = robot.cycle_count

            now = time.time()
            if now - last_hz_time >= 1.0:
                cycles = robot.cycle_count - last_hz_cycle
                shm.actual_hz = cycles / max(1e-6, now - last_hz_time)
                last_hz_time = now
                last_hz_cycle = robot.cycle_count

        set_status_message(shm, "stopping bridge")
        if robot is not None:
            zero_commands(robot)
            if args.enable or args.verified_enable:
                robot.disable_all()
            robot.close()
        set_status_message(shm, "bridge stopped")
        return 0
    except Exception as exc:  # noqa: BLE001 - bridge must report all failures to C++.
        shm.status_flags |= STATUS_FAULT
        set_status_message(shm, f"bridge fault: {exc}")
        try:
            if robot is not None:
                zero_commands(robot)
                if args.enable or args.verified_enable:
                    robot.disable_all()
                robot.close()
        except Exception as cleanup_exc:  # noqa: BLE001
            set_status_message(shm, f"bridge fault: {exc}; cleanup: {cleanup_exc}")
        return 1
    finally:
        del shm
        shm_obj.close()


if __name__ == "__main__":
    raise SystemExit(main())
