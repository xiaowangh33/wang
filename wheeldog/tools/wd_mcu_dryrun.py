#!/usr/bin/env python3
"""Dry-run tester for the wheeled-dog PC<->MCU protocol.

This script sends HELLO and zero SETPOINT packets and prints MCU FEEDBACK
packets. The first firmware pass is dry-run only: motors are not enabled.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass

try:
    import serial
except ImportError as exc:  # pragma: no cover - user environment helper
    raise SystemExit("pyserial is required: pip install pyserial") from exc


MAGIC = 0x34504457
VERSION = 1
MOTOR_COUNT = 16
MAX_PAYLOAD = 1172

PACKET_HELLO = 1
PACKET_SETPOINT = 2
PACKET_ESTOP = 4
PACKET_FEEDBACK = 0x81

ACTUATOR_POSITION_PD = 0
ACTUATOR_VELOCITY = 1

LIMIT_STARTUP_RELATIVE = 0
LIMIT_CALIBRATED_ABSOLUTE = 1
TORQUE_JOINT_PHASES = {0, 1, 2}
VELOCITY_JOINT_PHASES = {3}
BENCH_TORQUE_LIMIT_NM = 36.0
BENCH_VELOCITY_LIMIT_RAD_S = 20.0
DEFAULT_SETPOINT_HZ = 200.0
MIN_SETPOINT_HZ = 10.0
MAX_SETPOINT_HZ = 200.0
QUALIFICATION_TORQUE_NM = 0.6
QUALIFICATION_VELOCITY_RAD_S = 1.0
RS01_MOTION_KD = 0.4
CONTROL_FLAG_ENABLE_REQUEST = 1 << 0
CONTROL_FLAG_DRY_RUN = 1 << 2
CONTROL_FLAG_READONLY_POLL = 1 << 3
CONTROL_FLAG_READONLY_POLL_CAN2 = 1 << 4
CONTROL_FLAG_LIVE_CONTROL = 1 << 5
CONTROL_FLAG_QUALIFICATION_EXCITATION = 1 << 6

STATUS_DRY_RUN = 1 << 0
STATUS_ACTIVE = 1 << 1
STATUS_COMMAND_TIMEOUT = 1 << 2
STATUS_ESTOP = 1 << 3
STATUS_CRC_ERROR = 1 << 4
STATUS_BAD_PACKET = 1 << 5
STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED = 1 << 6
STATUS_CONTROL_TASK_FALLBACK = 1 << 7
STATUS_CAN_RX_OVERFLOW = 1 << 8
STATUS_SETPOINT_CLIPPED = 1 << 9
STATUS_ENABLE_BLOCKED_DRY_RUN = 1 << 10
STATUS_TORQUE_SLEW_LIMITED = 1 << 11
STATUS_VELOCITY_GUARD = 1 << 12
STATUS_COMMAND_CLIPPED = 1 << 13
STATUS_READONLY_POLL = 1 << 14
STATUS_CAN_TX_ERROR = 1 << 15
STATUS_BENCH_CAN2_REMAP = 1 << 16
STATUS_LIVE_CONTROL_REQUESTED = 1 << 17
STATUS_LIVE_CONTROL_ACTIVE = 1 << 18
STATUS_LIVE_ENABLE_IN_PROGRESS = 1 << 19
STATUS_LIVE_ENABLE_READY = 1 << 20
STATUS_LIVE_CONTROL_BLOCKED = 1 << 21
STATUS_LIVE_SAFETY_STOP = 1 << 22
STATUS_MCU_CAN_BASE_2 = 1 << 23
STATUS_CAN_TX_DEADLINE_MISS = 1 << 24
STATUS_FAST_FEEDBACK_READY = 1 << 25
STATUS_FAST_FEEDBACK_UNQUALIFIED = 1 << 26
STATUS_BENCH_RELATIVE_LIMITS = 1 << 27
STATUS_QUALIFICATION_EXCITATION = 1 << 28
STATUS_RAW_POSITION_LIMIT = 1 << 29

LIVE_STOP_REASON_FAST_FEEDBACK_LOST = 1 << 0
LIVE_STOP_REASON_COMMAND_FEEDBACK_UNHEALTHY = 1 << 1
LIVE_STOP_REASON_COMMAND_MOTOR_NOT_ENABLED = 1 << 2
LIVE_STOP_REASON_FAST_STALE = 1 << 3
LIVE_STOP_REASON_FAST_RATE_LOST = 1 << 4
LIVE_STOP_REASON_FAST_REFERENCE_LOST = 1 << 5
LIVE_STOP_REASON_OVERSPEED = 1 << 6

HEADER = struct.Struct("<IBBHIHH")
JOINT_CMD = struct.Struct("<fffff")
FEEDBACK_PREFIX = struct.Struct("<IIIIIIIIIIIIII f B 3x")
JOINT_FEEDBACK = struct.Struct("<ffffI BBBB")
CAN_DIAGNOSTICS = struct.Struct("<IIII")
LAST_CAN = struct.Struct("<IIII")
MOTOR_COUNTERS = struct.Struct("<" + "I" * MOTOR_COUNT)
MOTOR_FLOATS = struct.Struct("<" + "f" * MOTOR_COUNT)
MOTOR_BYTES = struct.Struct("<" + "B" * MOTOR_COUNT)
OBSERVATION_DIAGNOSTICS = struct.Struct("<IIHH" + "H" * MOTOR_COUNT)
LIVE_STOP_DIAGNOSTICS = struct.Struct("<IIIIII")


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(packet_type: int, seq: int, payload: bytes = b"") -> bytes:
    header = bytearray(HEADER.pack(MAGIC, VERSION, packet_type, len(payload), seq, 0, 0))
    packet = header + payload
    crc = crc16_ccitt(packet)
    struct.pack_into("<H", packet, 12, crc)
    return bytes(packet)


def build_zero_setpoint(
    seq: int,
    timeout_ms: int,
    poll_readonly: bool = False,
    poll_bus: str = "can1",
    live_control: bool = False,
    enable_request: bool = False,
    torque_overrides: dict[int, float] | None = None,
    velocity_overrides: dict[int, float] | None = None,
    qualification_excitation: bool = False,
) -> bytes:
    modes = bytearray([ACTUATOR_POSITION_PD] * MOTOR_COUNT)
    for idx in (3, 7, 11, 15):
        modes[idx] = ACTUATOR_VELOCITY

    control_flags = 0
    if live_control:
        control_flags |= CONTROL_FLAG_LIVE_CONTROL
    else:
        control_flags |= CONTROL_FLAG_DRY_RUN
    if enable_request:
        control_flags |= CONTROL_FLAG_ENABLE_REQUEST
    if qualification_excitation:
        control_flags |= CONTROL_FLAG_QUALIFICATION_EXCITATION
    if poll_readonly:
        control_flags |= CONTROL_FLAG_READONLY_POLL
        if poll_bus == "can2":
            control_flags |= CONTROL_FLAG_READONLY_POLL_CAN2

    payload = bytearray()
    payload += struct.pack(
        "<IIIB",
        control_flags,
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        timeout_ms,
        LIMIT_CALIBRATED_ABSOLUTE,
    )
    payload += bytes(modes)
    payload += b"\x00" * 3
    torque_overrides = torque_overrides or {}
    velocity_overrides = velocity_overrides or {}
    for i in range(MOTOR_COUNT):
        if modes[i] == ACTUATOR_POSITION_PD:
            payload += JOINT_CMD.pack(
                0.0,
                0.0,
                0.0,
                0.0,
                float(torque_overrides.get(i, 0.0)),
            )
        else:
            velocity = float(velocity_overrides.get(i, 0.0))
            payload += JOINT_CMD.pack(
                0.0,
                0.0,
                RS01_MOTION_KD if velocity != 0.0 else 0.0,
                velocity,
                0.0,
            )
    return build_packet(PACKET_SETPOINT, seq, bytes(payload))


@dataclass
class JointFeedback:
    q: float
    dq: float
    tau: float
    temperature_c: float
    fault_bits: int
    online: int
    enabled: int
    actuator_mode: int
    mode_status: int


@dataclass
class Feedback:
    status_flags: int
    control_flags: int
    feedback_time_ms: int
    last_rx_seq: int
    last_setpoint_seq: int
    rx_packets: int
    rx_crc_errors: int
    rx_bad_packets: int
    control_ticks: int
    setpoint_age_ms: int
    command_timeout_ms: int
    online_mask: int
    enabled_mask: int
    fault_mask: int
    actual_control_hz: float
    limit_mode: int
    joints: list[JointFeedback]
    can_rx_frames: int = 0
    can_rx_bad_frames: int = 0
    can_rx_overflows: int = 0
    can_tx_errors: int = 0
    last_can_id: int = 0
    last_can_data0_3: int = 0
    last_can_data4_7: int = 0
    last_can_meta: int = 0
    live_command_tx_queued: tuple[int, ...] = (0,) * MOTOR_COUNT
    live_command_tx_completed: tuple[int, ...] = (0,) * MOTOR_COUNT
    live_command_tx_deferred: tuple[int, ...] = (0,) * MOTOR_COUNT
    operation_status_rx_count: tuple[int, ...] = (0,) * MOTOR_COUNT
    fast_feedback_valid_mask: int = 0
    fast_position_excited_mask: int = 0
    fast_velocity_excited_mask: int = 0
    fast_feedback_rate_hz: tuple[float, ...] = (0.0,) * MOTOR_COUNT
    fast_position_error_rad: tuple[float, ...] = (0.0,) * MOTOR_COUNT
    fast_velocity_error_radps: tuple[float, ...] = (0.0,) * MOTOR_COUNT
    observation_seq: int = 0
    observation_time_ms: int = 0
    observation_max_sample_age_ms: int = 0
    observation_flags: int = 0
    observation_sample_age_ms: tuple[int, ...] = (0xFFFF,) * MOTOR_COUNT
    live_stop_reason_flags: int = 0
    live_stop_motor_mask: int = 0
    live_stop_event_count: int = 0
    live_stop_trigger_time_ms: int = 0
    live_stop_fast_valid_mask: int = 0
    live_stop_fault_mask: int = 0
    operation_status_max_gap_ms: tuple[int, ...] = (0,) * MOTOR_COUNT
    live_stop_fast_age_ms: tuple[int, ...] = (0,) * MOTOR_COUNT
    final_joint_torque_cmd_nm: tuple[float, ...] = (0.0,) * MOTOR_COUNT
    final_torque_telemetry_present: bool = False
    supply_voltage_valid_mask: int = 0
    supply_voltage_v: tuple[float, ...] = (0.0,) * MOTOR_COUNT
    supply_voltage_telemetry_present: bool = False


def status_names(flags: int) -> str:
    names = []
    table = [
        (STATUS_DRY_RUN, "dry-run"),
        (STATUS_ACTIVE, "active"),
        (STATUS_COMMAND_TIMEOUT, "timeout"),
        (STATUS_ESTOP, "estop"),
        (STATUS_CRC_ERROR, "crc-error"),
        (STATUS_BAD_PACKET, "bad-packet"),
        (STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED, "abs-limit-not-calibrated"),
        (STATUS_CONTROL_TASK_FALLBACK, "control-fallback"),
        (STATUS_CAN_RX_OVERFLOW, "can-rx-overflow"),
        (STATUS_SETPOINT_CLIPPED, "setpoint-clipped"),
        (STATUS_ENABLE_BLOCKED_DRY_RUN, "enable-blocked-dry-run"),
        (STATUS_TORQUE_SLEW_LIMITED, "torque-slew-limited"),
        (STATUS_VELOCITY_GUARD, "velocity-guard"),
        (STATUS_COMMAND_CLIPPED, "command-clipped"),
        (STATUS_READONLY_POLL, "readonly-poll"),
        (STATUS_CAN_TX_ERROR, "can-tx-error"),
        (STATUS_BENCH_CAN2_REMAP, "bench-can2-remap"),
        (STATUS_LIVE_CONTROL_REQUESTED, "live-requested"),
        (STATUS_LIVE_CONTROL_ACTIVE, "live-active"),
        (STATUS_LIVE_ENABLE_IN_PROGRESS, "live-enable-progress"),
        (STATUS_LIVE_ENABLE_READY, "live-enable-ready"),
        (STATUS_LIVE_CONTROL_BLOCKED, "live-blocked"),
        (STATUS_LIVE_SAFETY_STOP, "live-safety-stop"),
        (STATUS_MCU_CAN_BASE_2, "mcu-base2"),
        (STATUS_CAN_TX_DEADLINE_MISS, "can-tx-deadline-miss"),
        (STATUS_FAST_FEEDBACK_READY, "fast-feedback-ready"),
        (STATUS_FAST_FEEDBACK_UNQUALIFIED, "fast-feedback-unqualified"),
        (STATUS_BENCH_RELATIVE_LIMITS, "bench-relative-limits"),
        (STATUS_QUALIFICATION_EXCITATION, "qualification-excitation"),
        (STATUS_RAW_POSITION_LIMIT, "raw-position-limit"),
    ]
    for mask, name in table:
        if flags & mask:
            names.append(name)
    return "|".join(names) if names else "0"


def parse_feedback(payload: bytes) -> Feedback | None:
    if len(payload) < FEEDBACK_PREFIX.size + MOTOR_COUNT * JOINT_FEEDBACK.size:
        return None
    values = FEEDBACK_PREFIX.unpack_from(payload, 0)
    joints: list[JointFeedback] = []
    offset = FEEDBACK_PREFIX.size
    for _ in range(MOTOR_COUNT):
        joint_values = JOINT_FEEDBACK.unpack_from(payload, offset)
        joints.append(JointFeedback(*joint_values))
        offset += JOINT_FEEDBACK.size
    can_diag = (0, 0, 0, 0)
    if len(payload) >= offset + CAN_DIAGNOSTICS.size:
        can_diag = CAN_DIAGNOSTICS.unpack_from(payload, offset)
        offset += CAN_DIAGNOSTICS.size
    last_can = (0, 0, 0, 0)
    if len(payload) >= offset + LAST_CAN.size:
        last_can = LAST_CAN.unpack_from(payload, offset)
        offset += LAST_CAN.size
    counters = [(0,) * MOTOR_COUNT for _ in range(3)]
    for i in range(3):
        if len(payload) >= offset + MOTOR_COUNTERS.size:
            counters[i] = MOTOR_COUNTERS.unpack_from(payload, offset)
            offset += MOTOR_COUNTERS.size
    operation_counts = (0,) * MOTOR_COUNT
    if len(payload) >= offset + MOTOR_COUNTERS.size:
        operation_counts = MOTOR_COUNTERS.unpack_from(payload, offset)
        offset += MOTOR_COUNTERS.size
    fast_valid_mask = 0
    if len(payload) >= offset + 4:
        (fast_valid_mask,) = struct.unpack_from("<I", payload, offset)
        offset += 4
    fast_position_excited_mask = 0
    if len(payload) >= offset + 4:
        (fast_position_excited_mask,) = struct.unpack_from("<I", payload, offset)
        offset += 4
    fast_velocity_excited_mask = 0
    if len(payload) >= offset + 4:
        (fast_velocity_excited_mask,) = struct.unpack_from("<I", payload, offset)
        offset += 4
    fast_arrays = [(0.0,) * MOTOR_COUNT for _ in range(3)]
    for i in range(3):
        if len(payload) >= offset + MOTOR_FLOATS.size:
            fast_arrays[i] = MOTOR_FLOATS.unpack_from(payload, offset)
            offset += MOTOR_FLOATS.size
    observation = (0, 0, 0, 0, *((0xFFFF,) * MOTOR_COUNT))
    if len(payload) >= offset + OBSERVATION_DIAGNOSTICS.size:
        observation = OBSERVATION_DIAGNOSTICS.unpack_from(payload, offset)
        offset += OBSERVATION_DIAGNOSTICS.size
    live_stop = (0, 0, 0, 0, 0, 0)
    if len(payload) >= offset + LIVE_STOP_DIAGNOSTICS.size:
        live_stop = LIVE_STOP_DIAGNOSTICS.unpack_from(payload, offset)
        offset += LIVE_STOP_DIAGNOSTICS.size
    fast_timing = [(0,) * MOTOR_COUNT for _ in range(2)]
    for i in range(2):
        if len(payload) >= offset + MOTOR_BYTES.size:
            fast_timing[i] = MOTOR_BYTES.unpack_from(payload, offset)
            offset += MOTOR_BYTES.size
    final_joint_torque_cmd_nm = (0.0,) * MOTOR_COUNT
    final_torque_telemetry_present = False
    if len(payload) >= offset + MOTOR_FLOATS.size:
        final_joint_torque_cmd_nm = MOTOR_FLOATS.unpack_from(payload, offset)
        offset += MOTOR_FLOATS.size
        final_torque_telemetry_present = True
    supply_voltage_valid_mask = 0
    supply_voltage_v = (0.0,) * MOTOR_COUNT
    supply_voltage_telemetry_present = False
    if len(payload) >= offset + 4 + MOTOR_FLOATS.size:
        (supply_voltage_valid_mask,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        supply_voltage_v = MOTOR_FLOATS.unpack_from(payload, offset)
        offset += MOTOR_FLOATS.size
        supply_voltage_telemetry_present = True
    return Feedback(
        *values,
        joints,
        *can_diag,
        *last_can,
        *counters,
        operation_counts,
        fast_valid_mask,
        fast_position_excited_mask,
        fast_velocity_excited_mask,
        *fast_arrays,
        *observation[:4],
        tuple(observation[4:]),
        *live_stop,
        *fast_timing,
        final_joint_torque_cmd_nm,
        final_torque_telemetry_present,
        supply_voltage_valid_mask,
        supply_voltage_v,
        supply_voltage_telemetry_present,
    )


def feed_parser(buffer: bytearray) -> list[tuple[int, int, bytes]]:
    packets: list[tuple[int, int, bytes]] = []
    while len(buffer) >= HEADER.size:
        start = buffer.find(b"WDP4")
        if start < 0:
            del buffer[:-3]
            break
        if start > 0:
            del buffer[:start]
        if len(buffer) < HEADER.size:
            break
        magic, version, packet_type, payload_size, seq, got_crc, _ = HEADER.unpack_from(buffer, 0)
        if magic != MAGIC or version != VERSION or payload_size > MAX_PAYLOAD:
            del buffer[0]
            continue
        total = HEADER.size + payload_size
        if len(buffer) < total:
            break
        packet = bytearray(buffer[:total])
        struct.pack_into("<H", packet, 12, 0)
        if crc16_ccitt(packet) == got_crc:
            packets.append((packet_type, seq, bytes(buffer[HEADER.size:total])))
            del buffer[:total]
        else:
            del buffer[0]
    return packets


def parse_index_values(items: list[str], label: str) -> dict[int, float]:
    values: dict[int, float] = {}
    for item in items:
        if ":" not in item:
            raise SystemExit(f"{label} must use index:value, got {item!r}")
        index_text, value_text = item.split(":", 1)
        try:
            index = int(index_text, 0)
            value = float(value_text)
        except ValueError as exc:
            raise SystemExit(f"bad {label} item {item!r}") from exc
        if not 0 <= index < MOTOR_COUNT:
            raise SystemExit(f"{label} index out of range: {index}")
        values[index] = value
    return values


def validate_bench_live_commands(
    torque_overrides: dict[int, float],
    velocity_overrides: dict[int, float],
) -> None:
    for index, value in torque_overrides.items():
        if index % 4 not in TORQUE_JOINT_PHASES:
            raise SystemExit(
                "--joint-torque is only allowed for each leg's first "
                "three joints: indices 0/1/2, 4/5/6, 8/9/10, 12/13/14"
            )
        if not math.isfinite(value) or abs(value) > BENCH_TORQUE_LIMIT_NM:
            raise SystemExit(
                "--joint-torque must be finite and within +/-"
                f"{BENCH_TORQUE_LIMIT_NM:.1f} Nm"
            )
    for index, value in velocity_overrides.items():
        if index % 4 not in VELOCITY_JOINT_PHASES:
            raise SystemExit(
                "--joint-velocity is only allowed for wheel joints "
                "3/7/11/15"
            )
        if not math.isfinite(value) or abs(value) > BENCH_VELOCITY_LIMIT_RAD_S:
            raise SystemExit(
                "--joint-velocity must be finite and within +/-"
                f"{BENCH_VELOCITY_LIMIT_RAD_S:.1f} rad/s"
            )


def validate_qualification_excitation(
    torque_overrides: dict[int, float],
    velocity_overrides: dict[int, float],
) -> None:
    if len(torque_overrides) + len(velocity_overrides) != 1:
        raise SystemExit(
            "--qualification-excitation requires exactly one motor command"
        )
    if torque_overrides:
        value = next(iter(torque_overrides.values()))
        if not math.isclose(abs(value), QUALIFICATION_TORQUE_NM, abs_tol=1e-6):
            raise SystemExit("qualification RS06 torque must be exactly +/-0.6 Nm")
    if velocity_overrides:
        value = next(iter(velocity_overrides.values()))
        if not math.isclose(abs(value), QUALIFICATION_VELOCITY_RAD_S, abs_tol=1e-6):
            raise SystemExit(
                "qualification RS01 motion target must be exactly +/-1.0 rad/s"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument(
        "--hz",
        type=float,
        default=DEFAULT_SETPOINT_HZ,
        help="PC-to-MCU setpoint refresh rate (default: 200 Hz)",
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument(
        "--poll-readonly",
        action="store_true",
        help="ask MCU to poll ID1-4 mech_pos/mech_vel with read-only CAN frames",
    )
    parser.add_argument(
        "--poll-bus",
        choices=("can1", "can2"),
        default="can1",
        help="CAN bus used by --poll-readonly on the flashed MCU",
    )
    parser.add_argument(
        "--print-joints",
        action="store_true",
        help="print online joint q/dq/tau/temp from feedback packets",
    )
    parser.add_argument(
        "--live-control",
        action="store_true",
        help="request non-dry-run live CAN control from MCU",
    )
    parser.add_argument(
        "--enable-request",
        action="store_true",
        help="set ENABLE_REQUEST; only meaningful with --live-control",
    )
    parser.add_argument(
        "--i-understand-live-can",
        action="store_true",
        help="required with --live-control because it can enable motors",
    )
    parser.add_argument(
        "--qualification-excitation",
        action="store_true",
        help="bench-only: allow exactly one +/-0.6 Nm RS06 or +/-1 rad/s RS01",
    )
    parser.add_argument(
        "--joint-torque",
        action="append",
        default=[],
        metavar="INDEX:NM",
        help="explicit torque feedforward for a position/PD joint",
    )
    parser.add_argument(
        "--joint-velocity",
        action="append",
        default=[],
        metavar="INDEX:RAD_S",
        help="explicit velocity target for a velocity joint",
    )
    args = parser.parse_args()

    if not MIN_SETPOINT_HZ <= args.hz <= MAX_SETPOINT_HZ:
        raise SystemExit(
            f"--hz must be between {MIN_SETPOINT_HZ:g} and {MAX_SETPOINT_HZ:g} Hz"
        )
    if args.timeout_ms < math.ceil(3000.0 / args.hz):
        raise SystemExit("--timeout-ms must allow at least three PC setpoint periods")

    if args.live_control:
        if not (args.enable_request and args.i_understand_live_can):
            raise SystemExit(
                "--live-control requires --enable-request and "
                "--i-understand-live-can"
            )
        if args.poll_readonly:
            raise SystemExit("--live-control and --poll-readonly are separate test modes")
    elif args.enable_request:
        raise SystemExit("--enable-request is only allowed with --live-control")

    torque_overrides = parse_index_values(args.joint_torque, "--joint-torque")
    velocity_overrides = parse_index_values(args.joint_velocity, "--joint-velocity")
    if (torque_overrides or velocity_overrides) and not args.live_control:
        raise SystemExit("nonzero joint commands require --live-control")
    validate_bench_live_commands(torque_overrides, velocity_overrides)
    if args.qualification_excitation:
        if not args.live_control:
            raise SystemExit("--qualification-excitation requires --live-control")
        validate_qualification_excitation(torque_overrides, velocity_overrides)

    period = 1.0 / args.hz
    seq = 1
    rx = bytearray()
    next_send = time.monotonic()
    next_print = 0.0
    deadline = time.monotonic() + args.duration

    with serial.Serial(args.port, args.baudrate, timeout=0.01) as ser:
        ser.write(build_packet(PACKET_HELLO, seq))
        seq += 1
        mode_name = "LIVE-CONTROL" if args.live_control else "dry-run"
        print(f"opened {args.port} @ {args.baudrate}, {mode_name} for {args.duration:.1f}s")
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                ser.write(
                    build_zero_setpoint(
                        seq,
                        args.timeout_ms,
                        args.poll_readonly,
                        args.poll_bus,
                        args.live_control,
                        args.enable_request,
                        torque_overrides,
                        velocity_overrides,
                        args.qualification_excitation,
                    )
                )
                seq += 1
                next_send += period

            data = ser.read(4096)
            if data:
                rx.extend(data)
                for packet_type, packet_seq, payload in feed_parser(rx):
                    if packet_type != PACKET_FEEDBACK:
                        continue
                    fb = parse_feedback(payload)
                    if fb is None:
                        continue
                    if now >= next_print:
                        print(
                            "feedback "
                            f"seq={packet_seq} last_setpoint={fb.last_setpoint_seq} "
                            f"age={fb.setpoint_age_ms}ms "
                            f"hz={fb.actual_control_hz:.1f} "
                            f"status={status_names(fb.status_flags)} "
                            f"online=0x{fb.online_mask:04x} "
                            f"fault=0x{fb.fault_mask:04x} "
                            f"canrx={fb.can_rx_frames} "
                            f"canbad={fb.can_rx_bad_frames} "
                            f"cantxerr={fb.can_tx_errors} "
                            f"txq={sum(fb.live_command_tx_queued)} "
                            f"txdone={sum(fb.live_command_tx_completed)} "
                            f"txdefer={sum(fb.live_command_tx_deferred)} "
                            f"oprx={sum(fb.operation_status_rx_count)} "
                            f"fast=0x{fb.fast_feedback_valid_mask:04x} "
                            f"pex=0x{fb.fast_position_excited_mask:04x} "
                            f"vex=0x{fb.fast_velocity_excited_mask:04x} "
                            f"obs={fb.observation_seq} "
                            f"obs_age={fb.observation_max_sample_age_ms}ms "
                            f"stop_reason=0x{fb.live_stop_reason_flags:x} "
                            f"stop_motors=0x{fb.live_stop_motor_mask:04x} "
                            f"max_gap={max(fb.operation_status_max_gap_ms)}ms"
                        )
                        if fb.can_rx_frames:
                            bus = fb.last_can_meta & 0xFF
                            dlc = (fb.last_can_meta >> 8) & 0xFF
                            data = (
                                fb.last_can_data0_3.to_bytes(4, "little") +
                                fb.last_can_data4_7.to_bytes(4, "little")
                            )
                            print(
                                "  last_can "
                                f"bus={bus} dlc={dlc} "
                                f"id=0x{fb.last_can_id:08x} "
                                f"data={data.hex()}"
                            )
                        if args.print_joints:
                            parts = []
                            first = 8 if fb.status_flags & STATUS_MCU_CAN_BASE_2 else 0
                            for index in range(first, first + 8):
                                joint = fb.joints[index]
                                voltage = (
                                    f"{fb.supply_voltage_v[index]:.1f}"
                                    if fb.supply_voltage_telemetry_present
                                    and fb.supply_voltage_valid_mask & (1 << index)
                                    else "--"
                                )
                                parts.append(
                                    f"{index}:on={joint.online} q={joint.q:.3f} "
                                    f"dq={joint.dq:.3f} "
                                    f"tau={joint.tau:.3f} "
                                    f"tau_cmd={fb.final_joint_torque_cmd_nm[index]:.3f} "
                                    f"T={joint.temperature_c:.1f} "
                                    f"V={voltage} "
                                    f"op={fb.fast_feedback_rate_hz[index]:.1f}Hz "
                                    f"gap={fb.operation_status_max_gap_ms[index]}ms "
                                    f"pe={fb.fast_position_error_rad[index]:.4f} "
                                    f"ve={fb.fast_velocity_error_radps[index]:.4f}"
                                )
                            if parts:
                                print("  joints " + " | ".join(parts))
                        if args.print_joints:
                            timing = [
                                f"{index}:gap={fb.operation_status_max_gap_ms[index]}ms"
                                f"/stop_age={fb.live_stop_fast_age_ms[index]}ms"
                                for index in range(first, first + 8)
                            ]
                            print("  fast_timing " + " | ".join(timing))
                        next_print = now + 0.5
            else:
                time.sleep(0.001)

        if args.live_control:
            print("sending zero/dry-run cleanup")
            for _ in range(10):
                ser.write(build_zero_setpoint(seq, args.timeout_ms))
                seq += 1
                ser.read(4096)
                time.sleep(0.02)

    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
