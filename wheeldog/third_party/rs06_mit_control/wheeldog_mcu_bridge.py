#!/usr/bin/env python3
"""Shared-memory bridge for the WDP4 dual-MCU local-PD firmware."""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from multiprocessing import resource_tracker, shared_memory

try:
    import serial
    SerialException = serial.SerialException
    SerialTimeoutException = serial.SerialTimeoutException
except ImportError:  # pragma: no cover - deployment dependency helper
    serial = None  # type: ignore

    class SerialException(Exception):
        """Test/import fallback used when pyserial is not installed."""

    class SerialTimeoutException(Exception):
        """Test/import fallback used when pyserial is not installed."""

from wd_mcu_protocol import (
    HEADER,
    LIMIT_CALIBRATED_ABSOLUTE,
    MOTOR_COUNT,
    PACKET_FEEDBACK,
    PACKET_SETPOINT,
    PACKET_HELLO,
    STATUS_CAN_RX_OVERFLOW,
    STATUS_CAN_TX_DEADLINE_MISS,
    STATUS_CAN_TX_ERROR,
    STATUS_BENCH_CAN2_REMAP,
    STATUS_BENCH_RELATIVE_LIMITS,
    STATUS_COMMAND_TIMEOUT,
    STATUS_CONTROL_TASK_FALLBACK,
    STATUS_ESTOP,
    STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED,
    STATUS_FAST_FEEDBACK_READY,
    STATUS_TORQUE_SLEW_LIMITED,
    STATUS_LIVE_CONTROL_ACTIVE,
    STATUS_LIVE_CONTROL_BLOCKED,
    STATUS_LIVE_ENABLE_READY,
    STATUS_LIVE_SAFETY_STOP,
    LIVE_STOP_REASON_COMMAND_FEEDBACK_UNHEALTHY,
    LIVE_STOP_REASON_COMMAND_MOTOR_NOT_ENABLED,
    LIVE_STOP_REASON_FAST_FEEDBACK_LOST,
    LIVE_STOP_REASON_FAST_RATE_LOST,
    LIVE_STOP_REASON_FAST_REFERENCE_LOST,
    LIVE_STOP_REASON_FAST_STALE,
    LIVE_STOP_REASON_OVERSPEED,
    JointCommand,
    McuFeedback,
    PacketParser,
    ZERO_COMMANDS,
    build_packet,
    build_setpoint_packet,
    parse_feedback,
)


def _nonnegative_env_float(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        return default
    return value if math.isfinite(value) and value >= 0.0 else default


MAGIC = 0x57444D42
VERSION = 4
CONTROL_EXIT = 1 << 0

BRIDGE_STATUS_READY = 1 << 0
BRIDGE_STATUS_OPEN = 1 << 1
BRIDGE_STATUS_ENABLED = 1 << 2
BRIDGE_STATUS_FAULT = 1 << 3

SERIAL_WRITE_TIMEOUT_S = 0.005
SERIAL_WRITE_MAX_ATTEMPTS = 3
SERIAL_ACK_RECOVERY_MAX_SETPOINT_AGE_MS = 250
SERIAL_ACK_POLL_WINDOW_S = 0.012
SERIAL_ACK_POLL_INTERVAL_S = 0.0005
SERIAL_PACKET_RECOVERY_DEADLINE_S = 0.050
SERIAL_SETPOINT_DELIVERY_GRACE_MS = 250
SERIAL_SETPOINT_WATCHDOG_MARGIN_MS = 25
MCU_FEEDBACK_LOSS_TIMEOUT_S = 0.500
# The C++ side has the same 500 ms final feedback latch.  A shorter bridge-only
# deadline used to turn one recoverable USB scheduling hiccup into a process
# restart even while both MCUs continued applying fresh setpoints.
COHERENT_OBSERVATION_LOSS_TIMEOUT_S = 0.500
OBSERVATION_HISTORY_DEPTH = 64
# Keep enough receive history to pair a temporarily leading MCU with the
# matching epoch from its peer, but never publish an observation that sat in
# userspace for an appreciable fraction of the final 500 ms safety deadline.
OBSERVATION_HISTORY_MAX_AGE_S = 0.100
DEFAULT_MAX_OBSERVATION_SKEW_MS = 30.0
DEFAULT_COMMAND_TIMEOUT_MS = 300
DEFAULT_MIN_MOTOR_RATE_HZ = 350.0
RUNTIME_LOW_RATE_CONSECUTIVE_WINDOWS = 5
FIRMWARE_COMM_ERROR_CONSECUTIVE_FEEDBACKS = 5
# Detailed USB timing is diagnostic-only; leave it off during normal walking.
USB_TIMING_REPORT_PERIOD_S = _nonnegative_env_float(
    "WHEELDOG_USB_TIMING_REPORT_S", 0.0
)
USB_RECOVERY_EVENT_LOG_PERIOD_S = 1.0
# Normal runtime telemetry: temperature comes from the drive status frame, while
# firmware reads RobStride VBUS (0x701C) from each motor once per period.
MOTOR_TELEMETRY_REPORT_PERIOD_S = _nonnegative_env_float(
    "WHEELDOG_MOTOR_TELEMETRY_REPORT_S", 3.0
)
# Final-torque telemetry remains decoded for fault snapshots and tests, but the
# old synchronous 5 Hz terminal dump is disabled by default. Set a positive
# period explicitly when re-running a torque-chain diagnostic.
TORQUE_TELEMETRY_REPORT_PERIOD_S = _nonnegative_env_float(
    "WHEELDOG_TORQUE_TELEMETRY_REPORT_S", 0.0
)
DEFAULT_SETPOINT_HZ = 200.0
MIN_SETPOINT_HZ = 10.0
MAX_SETPOINT_HZ = 200.0

# C++ publishes commands through an even/odd seqlock at 200 Hz.  A reader can
# legitimately arrive while the sequence is odd, so retry across the writer's
# short critical section instead of treating a handful of immediate probes as
# a fatal error.  If the writer is briefly preempted while odd, reuse only the
# most recent coherent command and never for longer than one 50 Hz policy
# period.  A genuinely stuck writer still trips well before the MCU command
# watchdog.
COMMAND_SNAPSHOT_RETRY_TIMEOUT_S = 0.001
COMMAND_SNAPSHOT_RETRY_SLEEP_S = 0.00005
# Holding the last coherent PC command for 50 ms covers scheduler preemption
# and one complete 50 Hz policy interval plus margin.  It is still six times
# shorter than the MCU's default command watchdog, so a genuinely stuck C++
# writer cannot keep live control alive indefinitely.
COMMAND_SNAPSHOT_HOLD_TIMEOUT_S = 0.050

JOINT_NAMES = (
    "FL_HipX", "FL_HipY", "FL_Knee", "FL_Wheel",
    "FR_HipX", "FR_HipY", "FR_Knee", "FR_Wheel",
    "HL_HipX", "HL_HipY", "HL_Knee", "HL_Wheel",
    "HR_HipX", "HR_HipY", "HR_Knee", "HR_Wheel",
)
WHEEL_INDICES = (3, 7, 11, 15)


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
        ("command_kp", ctypes.c_float * MOTOR_COUNT),
        ("command_q_des", ctypes.c_float * MOTOR_COUNT),
        ("command_kd", ctypes.c_float * MOTOR_COUNT),
        ("command_dq_des", ctypes.c_float * MOTOR_COUNT),
        ("command_tau_ff", ctypes.c_float * MOTOR_COUNT),
        ("feedback_position", ctypes.c_float * MOTOR_COUNT),
        ("feedback_velocity", ctypes.c_float * MOTOR_COUNT),
        ("feedback_torque", ctypes.c_float * MOTOR_COUNT),
        ("feedback_temperature", ctypes.c_float * MOTOR_COUNT),
        ("feedback_fault_bits", ctypes.c_uint32 * MOTOR_COUNT),
        ("mcu_status_flags", ctypes.c_uint32 * 2),
        ("mcu_control_hz", ctypes.c_float * 2),
        ("motor_command_hz", ctypes.c_float * MOTOR_COUNT),
        ("motor_feedback_hz", ctypes.c_float * MOTOR_COUNT),
        ("fast_feedback_valid_mask", ctypes.c_uint32),
        ("reserved0", ctypes.c_uint32),
        ("reserved1", ctypes.c_uint32),
        ("observation_seq", ctypes.c_uint32),
        ("observation_max_sample_age_ms", ctypes.c_uint32),
        ("observation_bridge_skew_us", ctypes.c_uint32),
        ("dropped_observation_epochs", ctypes.c_uint32),
        ("motor_sample_age_ms", ctypes.c_float * MOTOR_COUNT),
        ("bridge_time_sec", ctypes.c_double),
        ("actual_hz", ctypes.c_double),
        ("cycle_count", ctypes.c_uint64),
        ("status_message", ctypes.c_char * 256),
    ]


assert ctypes.sizeof(BridgeShm) == 1200


@dataclass
class PortState:
    name: str
    device: object
    parser: PacketParser
    feedback: McuFeedback | None = None
    last_feedback_time: float = 0.0
    write_timeout_events: int = 0
    short_write_events: int = 0
    recovered_write_events: int = 0
    acknowledged_write_events: int = 0
    write_window_attempts: int = 0
    write_window_total_ms: float = 0.0
    write_window_max_ms: float = 0.0
    unconfirmed_setpoint_events: int = 0
    consecutive_unconfirmed_setpoints: int = 0
    max_consecutive_unconfirmed_setpoints: int = 0
    read_error_events: int = 0
    consecutive_read_errors: int = 0
    max_consecutive_read_errors: int = 0
    last_read_error: str = ""
    communication_sample_initialized: bool = False
    last_communication_feedback_time_ms: int = 0
    last_can_rx_overflows: int = 0
    last_can_tx_errors: int = 0
    last_can_tx_deferred_total: int = 0
    consecutive_can_error_feedbacks: int = 0
    max_consecutive_can_error_feedbacks: int = 0
    consecutive_live_blocked_feedbacks: int = 0
    max_consecutive_live_blocked_feedbacks: int = 0
    last_recovery_log_time: float = 0.0
    suppressed_recovery_logs: int = 0
    # Latest-only pairing can remain one epoch out of phase forever after a
    # single CDC write/read delay.  Preserve a small bounded history so the two
    # ports can be joined by observation_seq instead.
    observation_history: dict[int, tuple[McuFeedback, float]] = field(
        default_factory=dict
    )

    def __post_init__(self) -> None:
        if (
            self.feedback is not None
            and self.feedback.observation_seq != 0
            and (self.feedback.observation_flags & 1) != 0
        ):
            self.observation_history[self.feedback.observation_seq] = (
                self.feedback,
                self.last_feedback_time,
            )


def parse_ports(text: str) -> tuple[str, str]:
    ports = tuple(item.strip() for item in text.split(",") if item.strip())
    if len(ports) != 2:
        raise argparse.ArgumentTypeError(
            "expected two MCU ports, for example /dev/ttyACM0,/dev/ttyACM1"
        )
    return ports[0], ports[1]


def finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def set_status_message(shm: BridgeShm, text: str) -> None:
    shm.status_message = text.encode("utf-8", errors="replace")[:255]


@dataclass(frozen=True)
class CommandSnapshot:
    commands: list[JointCommand]
    sequence: int
    attempts: int
    wait_s: float
    held: bool = False
    source_age_s: float = 0.0


class CommandSnapshotUnavailable(RuntimeError):
    def __init__(self, attempts: int, wait_s: float, sequence: int):
        self.attempts = attempts
        self.wait_s = wait_s
        self.sequence = sequence
        super().__init__(
            "C++ command snapshot remained inconsistent for "
            f"{wait_s * 1000.0:.3f} ms after {attempts} attempts "
            f"(last_seq={sequence})"
        )


def read_command_snapshot(
    shm: BridgeShm,
    retry_timeout_s: float = COMMAND_SNAPSHOT_RETRY_TIMEOUT_S,
) -> CommandSnapshot:
    if retry_timeout_s < 0.0:
        raise ValueError("command snapshot retry timeout must be non-negative")
    started = time.monotonic()
    deadline = started + retry_timeout_s
    attempts = 0
    last_sequence = int(shm.command_seq)
    while True:
        attempts += 1
        begin = int(shm.command_seq)
        last_sequence = begin
        if begin & 1:
            commands = None
        else:
            commands = [
                JointCommand(
                    kp=max(0.0, finite_or_zero(float(shm.command_kp[i]))),
                    q_des=finite_or_zero(float(shm.command_q_des[i])),
                    kd=max(0.0, finite_or_zero(float(shm.command_kd[i]))),
                    dq_des=finite_or_zero(float(shm.command_dq_des[i])),
                    tau_ff=finite_or_zero(float(shm.command_tau_ff[i])),
                )
                for i in range(MOTOR_COUNT)
            ]
            end = int(shm.command_seq)
            last_sequence = end
            if begin == end and not (end & 1):
                return CommandSnapshot(
                    commands=commands,
                    sequence=end,
                    attempts=attempts,
                    wait_s=time.monotonic() - started,
                )

        now = time.monotonic()
        if now >= deadline:
            raise CommandSnapshotUnavailable(
                attempts=attempts,
                wait_s=now - started,
                sequence=last_sequence,
            )
        # Sleeping briefly is intentional: eight tight Python probes can all
        # execute before a preempted C++ writer gets another CPU timeslice.
        time.sleep(min(COMMAND_SNAPSHOT_RETRY_SLEEP_S, deadline - now))


class CommandSnapshotReader:
    """Bounded seqlock reader with one-policy-period stale-frame tolerance."""

    def __init__(
        self,
        retry_timeout_s: float = COMMAND_SNAPSHOT_RETRY_TIMEOUT_S,
        hold_timeout_s: float = COMMAND_SNAPSHOT_HOLD_TIMEOUT_S,
    ) -> None:
        if retry_timeout_s < 0.0 or hold_timeout_s <= 0.0:
            raise ValueError("invalid command snapshot timing limits")
        self.retry_timeout_s = retry_timeout_s
        self.hold_timeout_s = hold_timeout_s
        self.last_snapshot: CommandSnapshot | None = None
        self.last_success_time: float | None = None
        self.contention_events = 0
        self.hold_cycles = 0
        self.consecutive_hold_cycles = 0
        self.max_consecutive_hold_cycles = 0
        self.max_wait_s = 0.0
        self.max_hold_age_s = 0.0

    def read(self, shm: BridgeShm) -> CommandSnapshot:
        try:
            snapshot = read_command_snapshot(shm, self.retry_timeout_s)
        except CommandSnapshotUnavailable as exc:
            now = time.monotonic()
            self.contention_events += 1
            self.max_wait_s = max(self.max_wait_s, exc.wait_s)
            self.consecutive_hold_cycles += 1
            self.max_consecutive_hold_cycles = max(
                self.max_consecutive_hold_cycles,
                self.consecutive_hold_cycles,
            )
            if self.last_snapshot is None or self.last_success_time is None:
                raise RuntimeError(
                    f"{exc}; no previous coherent C++ command is available"
                ) from exc
            source_age_s = now - self.last_success_time
            if source_age_s > self.hold_timeout_s:
                raise RuntimeError(
                    f"{exc}; last coherent C++ command is "
                    f"{source_age_s * 1000.0:.3f} ms old, exceeding the "
                    f"{self.hold_timeout_s * 1000.0:.1f} ms safety limit"
                ) from exc
            self.hold_cycles += 1
            self.max_hold_age_s = max(self.max_hold_age_s, source_age_s)
            return CommandSnapshot(
                commands=self.last_snapshot.commands,
                sequence=self.last_snapshot.sequence,
                attempts=exc.attempts,
                wait_s=exc.wait_s,
                held=True,
                source_age_s=source_age_s,
            )

        if snapshot.attempts > 1:
            self.contention_events += 1
        self.max_wait_s = max(self.max_wait_s, snapshot.wait_s)
        self.last_snapshot = snapshot
        self.last_success_time = time.monotonic()
        self.consecutive_hold_cycles = 0
        return snapshot

    def consume_timing_report(self) -> str:
        report = (
            f"command_snapshot_contentions={self.contention_events},"
            f"holds={self.hold_cycles},"
            f"max_consecutive_holds={self.max_consecutive_hold_cycles},"
            f"max_wait={self.max_wait_s * 1000.0:.3f}ms,"
            f"max_hold_age={self.max_hold_age_s * 1000.0:.3f}ms"
        )
        self.contention_events = 0
        self.hold_cycles = 0
        self.max_consecutive_hold_cycles = 0
        self.max_wait_s = 0.0
        self.max_hold_age_s = 0.0
        return report


def feedback_by_base(states: list[PortState]) -> dict[int, McuFeedback]:
    result: dict[int, McuFeedback] = {}
    for state in states:
        if state.feedback is None:
            continue
        base = state.feedback.bus_base
        if base in result:
            raise RuntimeError(
                "both MCU ports report the same WD_CAN_BUS_BASE; flash one MCU with "
                "release-mcu-a and the other with release-mcu-b"
            )
        result[base] = state.feedback
    return result


def remember_observation(
    state: PortState, feedback: McuFeedback, receive_time: float
) -> None:
    """Retain a bounded, first-arrival-timed observation history per MCU."""
    observation_seq = feedback.observation_seq
    if observation_seq == 0 or (feedback.observation_flags & 1) == 0:
        return
    previous = state.observation_history.get(observation_seq)
    # Repeated feedback for the same observation may contain newer diagnostic
    # counters.  Keep those fields while preserving the first arrival time used
    # by the inter-MCU USB skew check.
    first_receive_time = previous[1] if previous is not None else receive_time
    state.observation_history[observation_seq] = (feedback, first_receive_time)
    while len(state.observation_history) > OBSERVATION_HISTORY_DEPTH:
        del state.observation_history[next(iter(state.observation_history))]


def prune_observation_history(state: PortState, now: float) -> None:
    expired = [
        seq
        for seq, (_feedback, receive_time) in state.observation_history.items()
        if now - receive_time > OBSERVATION_HISTORY_MAX_AGE_S
    ]
    for seq in expired:
        del state.observation_history[seq]


def coherent_observation_by_base(
    states: list[PortState], now: float | None = None
) -> dict[int, McuFeedback]:
    by_base = feedback_by_base(states)
    if len(by_base) != 2:
        return {}
    if now is None:
        # Tests and diagnostic callers may use synthetic monotonic timestamps.
        now = max((state.last_feedback_time for state in states), default=0.0)

    states_by_base: dict[int, PortState] = {}
    for state in states:
        if state.feedback is None:
            continue
        prune_observation_history(state, now)
        states_by_base[state.feedback.bus_base] = state
    if set(states_by_base) != {0, 2}:
        return {}

    common_sequences = (
        set(states_by_base[0].observation_history)
        & set(states_by_base[2].observation_history)
    )
    if not common_sequences:
        return {}

    # Select the most recently completed common epoch by receive time rather
    # than integer ordering, which also behaves correctly across uint32 wrap.
    observation_seq = max(
        common_sequences,
        key=lambda seq: max(
            states_by_base[0].observation_history[seq][1],
            states_by_base[2].observation_history[seq][1],
        ),
    )
    result = {
        base: states_by_base[base].observation_history[observation_seq][0]
        for base in (0, 2)
    }
    if any((feedback.observation_flags & 1) == 0 for feedback in result.values()):
        return {}
    return result


def coherent_observation_skew_s(
    states: list[PortState], by_base: dict[int, McuFeedback]
) -> float:
    receive_times: list[float] = []
    if set(by_base) != {0, 2}:
        return float("inf")
    observation_seq = by_base[0].observation_seq
    if observation_seq == 0 or by_base[2].observation_seq != observation_seq:
        return float("inf")
    for state in states:
        if state.feedback is None:
            continue
        selected = by_base.get(state.feedback.bus_base)
        history_entry = state.observation_history.get(observation_seq)
        if selected is not None and history_entry is not None:
            receive_times.append(history_entry[1])
    if len(receive_times) != 2:
        return float("inf")
    return abs(receive_times[0] - receive_times[1])


def observation_rejection_diagnostic(
    states: list[PortState],
    coherent_by_base: dict[int, McuFeedback],
    observation_skew_s: float,
    max_observation_age_ms: float,
    max_observation_skew_ms: float,
    now: float,
) -> str:
    latest: list[str] = []
    for state in states:
        feedback = state.feedback
        if feedback is None:
            latest.append(f"{state.name}:no-feedback")
            continue
        latest.append(
            f"base{feedback.bus_base}:seq={feedback.observation_seq},"
            f"usb_age={max(0.0, now - state.last_feedback_time) * 1000.0:.1f}ms,"
            f"sample_age={feedback.observation_max_sample_age_ms}ms,"
            f"fast=0x{feedback.fast_feedback_valid_mask & feedback.local_mask:04x},"
            f"history={len(state.observation_history)}"
        )
    if not coherent_by_base:
        reason = "no-common-seq"
    elif observation_skew_s * 1000.0 > max_observation_skew_ms:
        reason = (
            f"usb-skew={observation_skew_s * 1000.0:.1f}ms>"
            f"{max_observation_skew_ms:.1f}ms"
        )
    elif not coherent_observation_is_fresh(
        coherent_by_base, max_observation_age_ms
    ):
        reason = f"sample-not-fresh>{max_observation_age_ms:.1f}ms-or-fast-mask"
    else:
        reason = "waiting-for-new-common-seq"
    return reason + "; " + "; ".join(latest)


def coherent_observation_is_fresh(
    by_base: dict[int, McuFeedback], max_observation_age_ms: float
) -> bool:
    return bool(by_base) and all(
        feedback.observation_max_sample_age_ms <= max_observation_age_ms
        and (feedback.fast_feedback_valid_mask & feedback.local_mask)
        == feedback.local_mask
        for feedback in by_base.values()
    )


def update_shared_feedback(
    shm: BridgeShm,
    by_base: dict[int, McuFeedback],
    motor_command_hz: list[float],
    motor_feedback_hz: list[float],
    observation_skew_s: float,
    dropped_observation_epochs: int,
    bridge_hz: float,
    bridge_start: float,
    cycle_count: int,
) -> None:
    seq = int(shm.feedback_seq)
    if seq & 1:
        seq += 1
    shm.feedback_seq = seq + 1

    online_mask = 0
    enabled_mask = 0
    fault_mask = 0
    fresh_mask = 0
    fast_feedback_valid_mask = 0
    observation_seq = 0
    observation_max_sample_age_ms = 0
    for base, mcu in by_base.items():
        mcu_slot = 1 if base == 2 else 0
        local_mask = mcu.local_mask
        shm.mcu_status_flags[mcu_slot] = mcu.status_flags
        shm.mcu_control_hz[mcu_slot] = mcu.actual_control_hz
        online_mask |= mcu.online_mask & local_mask
        enabled_mask |= mcu.enabled_mask & local_mask
        fault_mask |= mcu.fault_mask & local_mask
        fast_feedback_valid_mask |= mcu.fast_feedback_valid_mask & local_mask
        fresh_mask |= mcu.fast_feedback_valid_mask & local_mask
        observation_seq = mcu.observation_seq
        observation_max_sample_age_ms = max(
            observation_max_sample_age_ms, mcu.observation_max_sample_age_ms
        )
        first = 8 if base == 2 else 0
        for index in range(first, first + 8):
            joint = mcu.joints[index]
            shm.feedback_position[index] = finite_or_zero(joint.q)
            shm.feedback_velocity[index] = finite_or_zero(joint.dq)
            shm.feedback_torque[index] = finite_or_zero(joint.tau)
            shm.feedback_temperature[index] = finite_or_zero(joint.temperature_c)
            shm.feedback_fault_bits[index] = joint.fault_bits
            shm.motor_sample_age_ms[index] = float(
                mcu.observation_sample_age_ms[index]
            )

    shm.online_mask = online_mask
    shm.enabled_mask = enabled_mask
    shm.fault_mask = fault_mask
    shm.fresh_mask = fresh_mask
    shm.fast_feedback_valid_mask = fast_feedback_valid_mask
    shm.observation_seq = observation_seq
    shm.observation_max_sample_age_ms = observation_max_sample_age_ms
    shm.observation_bridge_skew_us = min(
        0xFFFFFFFF, int(max(0.0, observation_skew_s) * 1_000_000.0)
    )
    shm.dropped_observation_epochs = dropped_observation_epochs
    for index, hz in enumerate(motor_command_hz):
        shm.motor_command_hz[index] = hz
    for index, hz in enumerate(motor_feedback_hz):
        shm.motor_feedback_hz[index] = hz
    shm.bridge_time_sec = time.monotonic() - bridge_start
    shm.actual_hz = bridge_hz
    shm.cycle_count = cycle_count
    shm.feedback_seq = seq + 2


def read_feedback(states: list[PortState]) -> int:
    received = 0
    now = time.monotonic()
    for state in states:
        try:
            waiting = int(getattr(state.device, "in_waiting", 0))
            if waiting <= 0:
                state.consecutive_read_errors = 0
                continue
            data = state.device.read(waiting)
        except (OSError, SerialException) as exc:
            # A single CDC-ACM ioctl/read error is not proof that the device
            # disappeared.  Keep servicing the other MCU and let the existing
            # 500 ms fresh-feedback deadline decide whether this port is truly
            # lost.  Parser/programming exceptions deliberately still escape.
            state.read_error_events += 1
            state.consecutive_read_errors += 1
            state.max_consecutive_read_errors = max(
                state.max_consecutive_read_errors,
                state.consecutive_read_errors,
            )
            state.last_read_error = f"{type(exc).__name__}: {exc}"
            log_usb_recovery(
                state,
                f"[WDP4 bridge] transient USB read error: {state.name}; "
                f"consecutive={state.consecutive_read_errors}; "
                f"total={state.read_error_events}; last={state.last_read_error}",
            )
            continue

        state.consecutive_read_errors = 0
        for packet_type, _seq, payload in state.parser.feed(data):
            if packet_type != PACKET_FEEDBACK:
                continue
            feedback = parse_feedback(payload)
            if feedback is None:
                continue
            state.feedback = feedback
            state.last_feedback_time = now
            remember_observation(state, feedback, now)
            received += 1
    return received


def log_usb_recovery(state: PortState, message: str) -> None:
    """Rate-limit bursty CDC recovery logs; counters remain lossless."""
    now = time.monotonic()
    if now - state.last_recovery_log_time < USB_RECOVERY_EVENT_LOG_PERIOD_S:
        state.suppressed_recovery_logs += 1
        return
    state.last_recovery_log_time = now
    print(message, flush=True)


def send_packet(
    states: list[PortState],
    packet: bytes,
    max_attempts: int = SERIAL_WRITE_MAX_ATTEMPTS,
) -> None:
    """Write one frame to both MCUs with fair, acknowledgement-aware recovery.

    Linux CDC-ACM can report a write-completion timeout after the frame reached
    the MCU.  Ports are attempted once per round, feedback from both MCUs is
    drained between attempts, and an exact setpoint acknowledgement completes a
    timed-out write without another duplicate transfer.  If one individual
    setpoint still cannot be confirmed, a recent MCU-reported previous setpoint
    is allowed to bridge the gap.  Only continuous delivery loss approaching
    the MCU watchdog becomes fatal; a single dropped 200 Hz refresh does not.
    """
    if max_attempts < 1:
        raise ValueError("max_attempts must be at least one")

    packet_type: int | None = None
    packet_seq: int | None = None
    if len(packet) >= HEADER.size:
        try:
            _magic, _version, packet_type, _payload_size, packet_seq, _crc, _reserved = (
                HEADER.unpack_from(packet)
            )
        except Exception:
            packet_type = None
            packet_seq = None

    call_start = time.monotonic()
    recovery_deadline = call_start + SERIAL_PACKET_RECOVERY_DEADLINE_S
    state_count = len(states)
    completed = [False] * state_count
    terminal_failure = [False] * state_count
    attempts_used = [0] * state_count
    first_attempt_time = [0.0] * state_count
    last_error = ["unknown write failure"] * state_count

    def acknowledge_timed_out_setpoints() -> None:
        if packet_type != PACKET_SETPOINT or packet_seq is None:
            return
        try:
            read_feedback(states)
        except Exception:
            return
        now = time.monotonic()
        for index, state in enumerate(states):
            if completed[index] or terminal_failure[index] or attempts_used[index] == 0:
                continue
            feedback = state.feedback
            feedback_is_new = (
                feedback is not None
                and state.last_feedback_time >= first_attempt_time[index]
            )
            setpoint_acknowledged = (
                feedback_is_new
                and feedback is not None
                and feedback.last_setpoint_seq == packet_seq
                and feedback.setpoint_age_ms
                <= min(
                    SERIAL_ACK_RECOVERY_MAX_SETPOINT_AGE_MS,
                    max(0, feedback.command_timeout_ms - 1),
                )
            )
            if not setpoint_acknowledged:
                continue
            completed[index] = True
            state.acknowledged_write_events += 1
            total_ms = (now - first_attempt_time[index]) * 1000.0
            log_usb_recovery(
                state,
                "[WDP4 bridge] USB write completion timeout acknowledged "
                f"by MCU: {state.name}; attempts={attempts_used[index]}; "
                f"elapsed={total_ms:.1f}ms; seq={packet_seq}; "
                f"mcu_setpoint_age={feedback.setpoint_age_ms}ms",
            )

    for _round in range(max_attempts):
        # An acknowledgement may have arrived just after the previous polling
        # window.  Drain it before sending another duplicate frame.
        acknowledge_timed_out_setpoints()
        pending_indices = [
            index
            for index in range(state_count)
            if not completed[index] and not terminal_failure[index]
        ]
        if not pending_indices:
            break
        # Reserve one full configured write-timeout for every pending port.
        # This prevents starting a retry round that cannot finish inside the
        # packet-level recovery budget and preserves fairness between MCUs.
        if (
            time.monotonic()
            + len(pending_indices) * SERIAL_WRITE_TIMEOUT_S
            > recovery_deadline
        ):
            for index in pending_indices:
                last_error[index] = "recovery deadline exceeded"
            break
        for index, state in enumerate(states):
            if completed[index] or terminal_failure[index]:
                continue
            if time.monotonic() >= recovery_deadline:
                last_error[index] = "recovery deadline exceeded"
                continue
            attempts_used[index] += 1
            if first_attempt_time[index] == 0.0:
                first_attempt_time[index] = time.monotonic()
            attempt_start = time.monotonic()
            try:
                written = state.device.write(packet)
            except Exception as exc:
                elapsed_ms = (time.monotonic() - attempt_start) * 1000.0
                state.write_window_attempts += 1
                state.write_window_total_ms += elapsed_ms
                state.write_window_max_ms = max(
                    state.write_window_max_ms, elapsed_ms
                )
                if not isinstance(exc, SerialTimeoutException):
                    last_error[index] = f"{type(exc).__name__}: {exc}"
                    terminal_failure[index] = True
                    continue
                state.write_timeout_events += 1
                last_error[index] = "write timeout"
                # The OUT completion may have timed out even though this exact
                # frame is already being reported by the MCU's IN endpoint.
                acknowledge_timed_out_setpoints()
                continue

            elapsed_ms = (time.monotonic() - attempt_start) * 1000.0
            state.write_window_attempts += 1
            state.write_window_total_ms += elapsed_ms
            state.write_window_max_ms = max(state.write_window_max_ms, elapsed_ms)
            if written != len(packet):
                state.short_write_events += 1
                last_error[index] = f"short write {written}/{len(packet)} bytes"
                continue

            completed[index] = True
            if attempts_used[index] > 1:
                state.recovered_write_events += 1
                total_ms = (
                    time.monotonic() - first_attempt_time[index]
                ) * 1000.0
                log_usb_recovery(
                    state,
                    f"[WDP4 bridge] transient USB write recovered: {state.name}; "
                    f"attempts={attempts_used[index]}; elapsed={total_ms:.1f}ms; "
                    f"timeout_events={state.write_timeout_events}; "
                    f"short_write_events={state.short_write_events}",
                )

        if all(completed[index] or terminal_failure[index] for index in range(state_count)):
            break

        if packet_type != PACKET_SETPOINT or packet_seq is None:
            # HELLO and other packets do not have an application-level ACK in
            # feedback, so there is nothing useful to poll between rounds.
            continue

        # Give the USB feedback service time to expose an exact ACK before
        # starting the next retry round. Fast CAN feedback remains 500 Hz, but
        # the consolidated USB diagnostic packet is intentionally slower.
        poll_deadline = min(
            recovery_deadline,
            time.monotonic() + SERIAL_ACK_POLL_WINDOW_S,
        )
        while time.monotonic() < poll_deadline:
            acknowledge_timed_out_setpoints()
            if all(
                completed[index] or terminal_failure[index]
                for index in range(state_count)
            ):
                break
            remaining = poll_deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(SERIAL_ACK_POLL_INTERVAL_S, remaining))

        if time.monotonic() >= recovery_deadline:
            for index in range(state_count):
                if not completed[index] and not terminal_failure[index]:
                    last_error[index] = "recovery deadline exceeded"
            break

    # One final drain catches an ACK that arrived at the polling boundary.
    acknowledge_timed_out_setpoints()

    failures: list[str] = []
    for index, state in enumerate(states):
        if completed[index]:
            state.consecutive_unconfirmed_setpoints = 0
            continue
        start = first_attempt_time[index] or call_start
        diagnostic_now = time.monotonic()
        total_ms = (diagnostic_now - start) * 1000.0
        feedback_detail = ""
        effective_setpoint_age_ms = float("inf")
        delivery_grace_ms = 0.0
        if state.feedback is not None:
            feedback_age_ms = (
                diagnostic_now - state.last_feedback_time
            ) * 1000.0
            effective_setpoint_age_ms = (
                float(state.feedback.setpoint_age_ms) + feedback_age_ms
            )
            delivery_grace_ms = min(
                float(SERIAL_SETPOINT_DELIVERY_GRACE_MS),
                max(
                    0.0,
                    float(state.feedback.command_timeout_ms)
                    - float(SERIAL_SETPOINT_WATCHDOG_MARGIN_MS),
                ),
            )
            feedback_detail = (
                f", feedback_age={feedback_age_ms:.1f}ms"
                f", mcu_setpoint_age={state.feedback.setpoint_age_ms}ms"
                f", effective_setpoint_age={effective_setpoint_age_ms:.1f}ms"
                f", mcu_last_setpoint_seq={state.feedback.last_setpoint_seq}"
            )
        packet_detail = (
            f", pc_packet_seq={packet_seq}"
            if packet_type == PACKET_SETPOINT and packet_seq is not None
            else ""
        )

        # Neither a pyserial completion timeout nor one SerialException/OSError
        # proves that control stopped: the latest feedback can still show a
        # safely recent previous setpoint. Drop this refresh and let the next
        # 200 Hz packet resynchronize. A real disconnect stops refreshing
        # feedback, so effective_setpoint_age_ms reaches the 250 ms delivery
        # limit before the MCU's 300 ms watchdog and the bridge then fails.
        previous_setpoint_still_safe = (
            packet_type == PACKET_SETPOINT
            and state.feedback is not None
            and not (state.feedback.status_flags & STATUS_COMMAND_TIMEOUT)
            and delivery_grace_ms > 0.0
            and effective_setpoint_age_ms <= delivery_grace_ms
        )
        if previous_setpoint_still_safe:
            state.unconfirmed_setpoint_events += 1
            state.consecutive_unconfirmed_setpoints += 1
            state.max_consecutive_unconfirmed_setpoints = max(
                state.max_consecutive_unconfirmed_setpoints,
                state.consecutive_unconfirmed_setpoints,
            )
            continue

        failures.append(
            f"{state.name}: attempts={attempts_used[index]}, "
            f"elapsed={total_ms:.1f}ms, last={last_error[index]}, "
            f"timeout_events={state.write_timeout_events}, "
            f"short_write_events={state.short_write_events}"
            f"{packet_detail}{feedback_detail}"
        )

    if failures:
        raise RuntimeError("persistent USB write failure; " + "; ".join(failures))


def consume_usb_timing_report(states: list[PortState]) -> str:
    """Return and reset per-port host-write timing for the last report window."""
    details: list[str] = []
    for state in states:
        attempts = state.write_window_attempts
        average_ms = (
            state.write_window_total_ms / attempts if attempts > 0 else 0.0
        )
        firmware_transport = ""
        if state.feedback is not None:
            feedback = state.feedback
            first = 8 if feedback.bus_base == 2 else 0
            deferred_total = sum(
                feedback.live_command_tx_deferred[first : first + 8]
            )
            communication_flags = feedback.status_flags & (
                STATUS_CAN_RX_OVERFLOW
                | STATUS_CAN_TX_ERROR
                | STATUS_CAN_TX_DEADLINE_MISS
                | STATUS_LIVE_CONTROL_BLOCKED
            )
            firmware_transport = (
                f",mcu_can(rx_overflow_total={feedback.can_rx_overflows},"
                f"tx_error_total={feedback.can_tx_errors},"
                f"tx_deferred_total={deferred_total},"
                f"comm_flags=0x{communication_flags:08x})"
            )
        details.append(
            f"{state.name}:avg={average_ms:.3f}ms,"
            f"max={state.write_window_max_ms:.3f}ms,attempts={attempts},"
            f"timeouts_total={state.write_timeout_events},"
            f"recoveries_total={state.recovered_write_events},"
            f"mcu_ack_recoveries_total={state.acknowledged_write_events},"
            f"unconfirmed_setpoints_total={state.unconfirmed_setpoint_events},"
            "max_consecutive_unconfirmed="
            f"{state.max_consecutive_unconfirmed_setpoints},"
            f"read_errors_total={state.read_error_events},"
            "max_consecutive_read_errors="
            f"{state.max_consecutive_read_errors},"
            "max_consecutive_can_error_feedbacks="
            f"{state.max_consecutive_can_error_feedbacks},"
            "max_consecutive_live_blocked_feedbacks="
            f"{state.max_consecutive_live_blocked_feedbacks},"
            f"suppressed_recovery_logs={state.suppressed_recovery_logs}"
            f"{firmware_transport}"
        )
        state.write_window_attempts = 0
        state.write_window_total_ms = 0.0
        state.write_window_max_ms = 0.0
    return "; ".join(details)


def local_feedback_ready(
    feedback: McuFeedback, max_observation_age_ms: float = 5.0
) -> bool:
    mask = feedback.local_mask
    return (
        feedback.limit_mode == LIMIT_CALIBRATED_ABSOLUTE
        and not (feedback.status_flags & STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED)
        and not (feedback.status_flags & STATUS_BENCH_RELATIVE_LIMITS)
        and (feedback.online_mask & mask) == mask
        and (feedback.enabled_mask & mask) == mask
        and (feedback.fault_mask & mask) == 0
        and feedback.actual_control_hz >= 950.0
        and bool(feedback.status_flags & STATUS_LIVE_CONTROL_ACTIVE)
        and bool(feedback.status_flags & STATUS_LIVE_ENABLE_READY)
        and bool(feedback.status_flags & STATUS_FAST_FEEDBACK_READY)
        and (feedback.fast_feedback_valid_mask & mask) == mask
        and bool(feedback.observation_flags & 1)
        and feedback.observation_max_sample_age_ms <= max_observation_age_ms
    )


def fast_feedback_diagnostic(by_base: dict[int, McuFeedback]) -> str:
    details: list[str] = []
    for base, feedback in sorted(by_base.items()):
        first = 8 if base == 2 else 0
        for index in range(first, first + 8):
            if feedback.fast_feedback_valid_mask & (1 << index):
                continue
            details.append(
                f"{index}:rx={feedback.operation_status_rx_count[index]},"
                f"fw={feedback.fast_feedback_rate_hz[index]:.1f}Hz,"
                f"maxgap={feedback.operation_status_max_gap_ms[index]}ms,"
                f"pex={int(bool(feedback.fast_position_excited_mask & (1 << index)))},"
                f"vex={int(bool(feedback.fast_velocity_excited_mask & (1 << index)))},"
                f"pe={feedback.fast_position_error_rad[index]:.4f},"
                f"ve={feedback.fast_velocity_error_radps[index]:.4f}"
            )
    return "; ".join(details) if details else "no per-motor qualifier detail"


def final_torque_diagnostic(by_base: dict[int, McuFeedback]) -> str | None:
    """Compare the MCU's post-safety command with drive-reported joint torque."""
    if len(by_base) != 2 or not all(
        feedback.final_torque_telemetry_present for feedback in by_base.values()
    ):
        return None

    final_command = [0.0] * MOTOR_COUNT
    measured = [0.0] * MOTOR_COUNT
    slew_slots: list[str] = []
    for base, feedback in sorted(by_base.items()):
        first = 8 if base == 2 else 0
        for index in range(first, first + 8):
            final_command[index] = finite_or_zero(
                feedback.final_joint_torque_cmd_nm[index]
            )
            measured[index] = finite_or_zero(feedback.joints[index].tau)
        if feedback.status_flags & STATUS_TORQUE_SLEW_LIMITED:
            slew_slots.append("B" if base == 2 else "A")

    leg_indices = [index for index in range(MOTOR_COUNT) if index not in WHEEL_INDICES]
    worst_leg = max(
        leg_indices,
        key=lambda index: abs(final_command[index] - measured[index]),
    )
    wheel_command = " ".join(f"{final_command[index]:.1f}" for index in WHEEL_INDICES)
    wheel_measured = " ".join(f"{measured[index]:.1f}" for index in WHEEL_INDICES)
    return (
        f"leg_worst={JOINT_NAMES[worst_leg]} "
        f"final={final_command[worst_leg]:.1f} "
        f"motor_fb={measured[worst_leg]:.1f} "
        f"abs_err={abs(final_command[worst_leg] - measured[worst_leg]):.1f}; "
        f"wheel_final=({wheel_command}); wheel_fb=({wheel_measured}); "
        f"slew_this_feedback={'/'.join(slew_slots) if slew_slots else 'none'}"
    )


def motor_telemetry_report(by_base: dict[int, McuFeedback]) -> str:
    """Return compact per-leg temperature and bus-voltage lines for 16 motors."""
    temperatures: list[float | None] = [None] * MOTOR_COUNT
    voltages: list[float | None] = [None] * MOTOR_COUNT
    for base, feedback in sorted(by_base.items()):
        first = 8 if base == 2 else 0
        for index in range(first, first + 8):
            temperature = feedback.joints[index].temperature_c
            if (
                feedback.online_mask & (1 << index)
                and math.isfinite(temperature)
            ):
                temperatures[index] = temperature
            voltage = feedback.supply_voltage_v[index]
            if (
                feedback.supply_voltage_telemetry_present
                and feedback.supply_voltage_valid_mask & (1 << index)
                and math.isfinite(voltage)
            ):
                voltages[index] = voltage

    def format_groups(values: list[float | None]) -> str:
        groups: list[str] = []
        for leg, first in zip(("FL", "FR", "HL", "HR"), (0, 4, 8, 12)):
            samples = ",".join(
                "--" if value is None else f"{value:.1f}"
                for value in values[first:first + 4]
            )
            groups.append(f"{leg}[{samples}]")
        return " ".join(groups)

    return (
        "[Motor telemetry] temp_C (HipX,HipY,Knee,Wheel) "
        + format_groups(temperatures)
        + "\n[Motor telemetry] bus_V  (HipX,HipY,Knee,Wheel) "
        + format_groups(voltages)
    )


def critical_firmware_error(feedback: McuFeedback) -> str | None:
    # Communication-event bits are intentionally absent here. A single bxCAN
    # mailbox miss, deadline deferral or RX queue overflow is diagnostic, not
    # evidence that motor control stopped. Their real consequences are covered
    # independently by fresh 500 Hz feedback, per-motor zeroing, the 300 ms MCU
    # stop, the host feedback deadline and multi-window rate checks below.
    # LIVE_CONTROL_BLOCKED is also transiently asserted by a busy precheck
    # mailbox; LIVE_SAFETY_STOP remains an immediate hard failure.
    checks = (
        (STATUS_COMMAND_TIMEOUT, "command timeout"),
        (STATUS_ESTOP, "estop"),
        (STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED, "absolute limits invalid"),
        (STATUS_CONTROL_TASK_FALLBACK, "control task fallback"),
        (STATUS_LIVE_SAFETY_STOP, "live safety stop"),
    )
    active = [name for mask, name in checks if feedback.status_flags & mask]
    return " + ".join(active) if active else None


def persistent_firmware_communication_error(
    state: PortState,
    *,
    monitor_live_control: bool,
) -> str | None:
    """Debounce MCU communication diagnostics by distinct feedback packets."""
    feedback = state.feedback
    if feedback is None:
        return None

    feedback_time_ms = int(feedback.feedback_time_ms)
    first = 8 if feedback.bus_base == 2 else 0
    deferred_total = sum(feedback.live_command_tx_deferred[first : first + 8])
    if not state.communication_sample_initialized:
        state.communication_sample_initialized = True
        state.last_communication_feedback_time_ms = feedback_time_ms
        state.last_can_rx_overflows = int(feedback.can_rx_overflows)
        state.last_can_tx_errors = int(feedback.can_tx_errors)
        state.last_can_tx_deferred_total = deferred_total
        return None
    if feedback_time_ms == state.last_communication_feedback_time_ms:
        return None

    rx_overflow_delta = (
        int(feedback.can_rx_overflows) - state.last_can_rx_overflows
    ) & 0xFFFFFFFF
    tx_error_delta = (
        int(feedback.can_tx_errors) - state.last_can_tx_errors
    ) & 0xFFFFFFFF
    deferred_delta = (
        deferred_total - state.last_can_tx_deferred_total
    ) & 0xFFFFFFFF
    state.last_communication_feedback_time_ms = feedback_time_ms
    state.last_can_rx_overflows = int(feedback.can_rx_overflows)
    state.last_can_tx_errors = int(feedback.can_tx_errors)
    state.last_can_tx_deferred_total = deferred_total

    new_can_error = (
        rx_overflow_delta > 0
        or tx_error_delta > 0
        or (
            bool(feedback.status_flags & STATUS_CAN_TX_DEADLINE_MISS)
            and deferred_delta > 0
        )
    )
    state.consecutive_can_error_feedbacks = (
        state.consecutive_can_error_feedbacks + 1 if new_can_error else 0
    )
    state.max_consecutive_can_error_feedbacks = max(
        state.max_consecutive_can_error_feedbacks,
        state.consecutive_can_error_feedbacks,
    )

    live_blocked = monitor_live_control and bool(
        feedback.status_flags & STATUS_LIVE_CONTROL_BLOCKED
    )
    state.consecutive_live_blocked_feedbacks = (
        state.consecutive_live_blocked_feedbacks + 1 if live_blocked else 0
    )
    state.max_consecutive_live_blocked_feedbacks = max(
        state.max_consecutive_live_blocked_feedbacks,
        state.consecutive_live_blocked_feedbacks,
    )

    if (
        state.consecutive_can_error_feedbacks
        >= FIRMWARE_COMM_ERROR_CONSECUTIVE_FEEDBACKS
    ):
        return (
            "persistent CAN communication errors for "
            f"{state.consecutive_can_error_feedbacks} feedback packets "
            f"(rx_overflow_delta={rx_overflow_delta},"
            f"tx_error_delta={tx_error_delta},"
            f"tx_deferred_delta={deferred_delta})"
        )
    if (
        state.consecutive_live_blocked_feedbacks
        >= FIRMWARE_COMM_ERROR_CONSECUTIVE_FEEDBACKS
    ):
        return (
            "live control remained blocked for "
            f"{state.consecutive_live_blocked_feedbacks} feedback packets"
        )
    return None


def firmware_fault_diagnostic(
    feedback: McuFeedback,
    commands: list[JointCommand] | None = None,
) -> str:
    """Return a compact but actionable snapshot before cleanup clears the fault."""
    local_mask = feedback.local_mask
    first = 8 if feedback.bus_base == 2 else 0
    local_indices = range(first, first + 8)
    deferred = [
        f"{i}:{feedback.live_command_tx_deferred[i]}"
        for i in local_indices
        if feedback.live_command_tx_deferred[i]
    ]
    offline = [i for i in local_indices if not feedback.online_mask & (1 << i)]
    disabled = [i for i in local_indices if not feedback.enabled_mask & (1 << i)]
    faulted = [
        f"{i}:0x{feedback.joints[i].fault_bits:x}"
        for i in local_indices
        if feedback.joints[i].fault_bits
    ]
    unqualified = [
        i for i in local_indices
        if not feedback.fast_feedback_valid_mask & (1 << i)
    ]
    details = [
        f"flags=0x{feedback.status_flags:08x}",
        f"control=0x{feedback.control_flags:08x}",
        (
            "masks(online/enabled/fault/fast)="
            f"0x{feedback.online_mask & local_mask:04x}/"
            f"0x{feedback.enabled_mask & local_mask:04x}/"
            f"0x{feedback.fault_mask & local_mask:04x}/"
            f"0x{feedback.fast_feedback_valid_mask & local_mask:04x}"
        ),
        f"control_hz={feedback.actual_control_hz:.1f}",
        f"setpoint_age={feedback.setpoint_age_ms}ms",
        (
            f"obs(seq={feedback.observation_seq},"
            f"age={feedback.observation_max_sample_age_ms}ms,"
            f"flags=0x{feedback.observation_flags:x})"
        ),
        (
            f"can(rx_bad={feedback.can_rx_bad_frames},"
            f"overflow={feedback.can_rx_overflows},tx_error={feedback.can_tx_errors})"
        ),
    ]
    if feedback.live_stop_reason_flags:
        reason_table = (
            (LIVE_STOP_REASON_FAST_FEEDBACK_LOST, "fast-feedback-lost"),
            (LIVE_STOP_REASON_FAST_STALE, "fast-feedback-stale"),
            (LIVE_STOP_REASON_FAST_RATE_LOST, "fast-rate-lost"),
            (LIVE_STOP_REASON_FAST_REFERENCE_LOST, "fast-reference-lost"),
            (LIVE_STOP_REASON_OVERSPEED, "measured-overspeed"),
            (
                LIVE_STOP_REASON_COMMAND_FEEDBACK_UNHEALTHY,
                "command-feedback-unhealthy",
            ),
            (
                LIVE_STOP_REASON_COMMAND_MOTOR_NOT_ENABLED,
                "command-motor-not-enabled",
            ),
        )
        reasons = [
            name
            for mask, name in reason_table
            if feedback.live_stop_reason_flags & mask
        ]
        details.append(
            "live_stop("
            f"reason={'+'.join(reasons) or hex(feedback.live_stop_reason_flags)},"
            f"motors=0x{feedback.live_stop_motor_mask:04x},"
            f"events={feedback.live_stop_event_count},"
            f"time={feedback.live_stop_trigger_time_ms}ms,"
            f"fast_snapshot=0x{feedback.live_stop_fast_valid_mask:04x},"
            f"fault_snapshot=0x{feedback.live_stop_fault_mask:04x})"
        )
        stopped_ages = [
            f"{i}:{feedback.live_stop_fast_age_ms[i]}ms"
            for i in local_indices
            if feedback.live_stop_motor_mask & (1 << i)
        ]
        if stopped_ages:
            details.append("fast_age_at_stop=" + ",".join(stopped_ages))
        stopped_velocities = [
            f"{i}:{feedback.joints[i].dq:.3f}rad/s"
            for i in local_indices
            if feedback.live_stop_motor_mask & (1 << i)
        ]
        if stopped_velocities:
            details.append(
                "velocity_snapshot_after_stop=" + ",".join(stopped_velocities)
            )
        stopped_states = [
            f"{i}:q={feedback.joints[i].q:.3f},dq={feedback.joints[i].dq:.3f}"
            for i in local_indices
            if feedback.live_stop_motor_mask & (1 << i)
        ]
        if stopped_states:
            details.append("joint_state_after_stop=" + ",".join(stopped_states))
        if commands is not None and len(commands) == MOTOR_COUNT:
            command_states: list[str] = []
            for i in local_indices:
                if not feedback.live_stop_motor_mask & (1 << i):
                    continue
                cmd = commands[i]
                joint = feedback.joints[i]
                expected_pd = (
                    cmd.kp * (cmd.q_des - joint.q)
                    + cmd.kd * (cmd.dq_des - joint.dq)
                    + cmd.tau_ff
                )
                command_states.append(
                    f"{i}:kp={cmd.kp:.2f},qdes={cmd.q_des:.3f},"
                    f"kd={cmd.kd:.2f},dqdes={cmd.dq_des:.3f},"
                    f"tauff={cmd.tau_ff:.2f},pd_after_stop={expected_pd:.2f}Nm"
                )
            if command_states:
                details.append("pc_command_at_fault=" + ",".join(command_states))
        if feedback.final_torque_telemetry_present:
            final_commands = [
                f"{i}:{feedback.final_joint_torque_cmd_nm[i]:.2f}Nm"
                for i in local_indices
            ]
            if final_commands:
                details.append("mcu_final_torque_at_stop=" + ",".join(final_commands))
    if offline:
        details.append(f"offline={offline}")
    if disabled:
        details.append(f"disabled={disabled}")
    if faulted:
        details.append("motor_faults=" + ",".join(faulted))
    if deferred:
        details.append("tx_deferred=" + ",".join(deferred))
    if unqualified:
        details.append(f"unqualified={unqualified}")
        details.append(
            "fast=" + fast_feedback_diagnostic({feedback.bus_base: feedback})
        )
    return "; ".join(details)


def cleanup_to_dry_run(states: list[PortState], seq: int, timeout_ms: int) -> None:
    for _ in range(10):
        try:
            packet = build_setpoint_packet(
                seq,
                ZERO_COMMANDS,
                live_control=False,
                enable_request=False,
                timeout_ms=timeout_ms,
            )
            send_packet(states, packet)
            seq += 1
            read_feedback(states)
        except Exception:
            pass
        time.sleep(0.02)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shm-name", required=True)
    parser.add_argument(
        "--ports",
        type=parse_ports,
        default=parse_ports("/dev/ttyACM0,/dev/ttyACM1"),
    )
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument(
        "--hz",
        type=float,
        default=DEFAULT_SETPOINT_HZ,
        help="PC-to-MCU setpoint refresh rate; independent of 50 Hz policy inference",
    )
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=DEFAULT_COMMAND_TIMEOUT_MS,
        help="MCU local command watchdog; transient USB recovery remains bounded below it",
    )
    parser.add_argument("--parent-pid", type=int, default=0)
    parser.add_argument("--enable", action="store_true")
    parser.add_argument(
        "--verified-enable",
        action="store_true",
        help="enable drives while keeping every command zero until static checks pass",
    )
    parser.add_argument("--motion-verify", action="store_true")
    parser.add_argument(
        "--min-command-hz", type=float, default=DEFAULT_MIN_MOTOR_RATE_HZ
    )
    parser.add_argument(
        "--min-feedback-hz", type=float, default=DEFAULT_MIN_MOTOR_RATE_HZ
    )
    parser.add_argument("--max-observation-age-ms", type=float, default=10.0)
    parser.add_argument(
        "--max-observation-skew-ms",
        type=float,
        default=DEFAULT_MAX_OBSERVATION_SKEW_MS,
    )
    parser.add_argument("--rate-grace-s", type=float, default=3.0)
    parser.add_argument("--setup-timeout-s", type=float, default=60.0)
    args = parser.parse_args()

    if serial is None:
        raise SystemExit("pyserial is required: pip install pyserial")

    if not MIN_SETPOINT_HZ <= args.hz <= MAX_SETPOINT_HZ:
        raise SystemExit(
            f"--hz must be between {MIN_SETPOINT_HZ:g} and {MAX_SETPOINT_HZ:g} Hz"
        )
    if args.timeout_ms < math.ceil(3000.0 / args.hz):
        raise SystemExit("--timeout-ms must allow at least three PC setpoint periods")
    if args.min_command_hz <= 0.0 or args.min_feedback_hz <= 0.0:
        raise SystemExit("minimum command and feedback rates must be positive")
    if args.max_observation_age_ms <= 0.0 or args.max_observation_skew_ms <= 0.0:
        raise SystemExit("observation age and skew limits must be positive")
    if args.motion_verify:
        raise SystemExit(
            "automatic motion verification is intentionally unsupported by the MCU bridge"
        )

    enable = args.enable or args.verified_enable
    stop_requested = False

    def handle_signal(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    shm_obj = shared_memory.SharedMemory(name=args.shm_name)
    # The C++ parent owns shm_unlink(). Python's resource tracker otherwise
    # unlinks this attached segment when the bridge exits.
    resource_tracker.unregister(shm_obj._name, "shared_memory")
    shm = BridgeShm.from_buffer(shm_obj.buf)
    states: list[PortState] = []
    bridge_start = time.monotonic()
    seq = 1
    cycle_count = 0

    if shm.magic != MAGIC or shm.version != VERSION or shm.motor_count != MOTOR_COUNT:
        del shm
        shm_obj.close()
        raise RuntimeError("shared-memory header mismatch")

    try:
        for port in args.ports:
            device = serial.Serial(
                port,
                args.baudrate,
                timeout=0,
                write_timeout=SERIAL_WRITE_TIMEOUT_S,
            )
            states.append(PortState(port, device, PacketParser()))
        shm.status_flags = BRIDGE_STATUS_OPEN
        set_status_message(shm, "WDP4 ports open; waiting for both MCU identities")
        send_packet(states, build_packet(PACKET_HELLO, seq))
        seq += 1

        period = 1.0 / args.hz
        next_send = time.monotonic()
        last_hz_time = next_send
        last_hz_cycle = 0
        bridge_hz = 0.0
        motor_command_hz = [0.0] * MOTOR_COUNT
        motor_feedback_hz = [0.0] * MOTOR_COUNT
        command_rate_counts = [0] * MOTOR_COUNT
        feedback_rate_counts = [0] * MOTOR_COUNT
        rate_mcu_time_ms = [0] * MOTOR_COUNT
        rate_time = next_send
        live_enabled_time: float | None = None
        ready_time: float | None = None
        identity_time: float | None = None
        low_rate_samples = 0
        last_cpp_command_seq: int | None = None
        last_cpp_command_time = next_send
        last_published_observation_seq: int | None = None
        last_coherent_observation_time: float | None = None
        dropped_observation_epochs = 0
        last_setup_progress_time = next_send - 2.0
        last_usb_timing_report_time = next_send
        last_torque_telemetry_report_time = next_send
        last_motor_telemetry_report_time = next_send
        torque_telemetry_missing_reported = False
        last_setpoint_send_time: float | None = None
        setpoint_gap_window_max_ms = 0.0
        observation_age_window_max_ms = 0.0
        observation_skew_window_max_ms = 0.0
        observation_no_common_window = 0
        observation_skew_reject_window = 0
        observation_freshness_reject_window = 0
        commands = list(ZERO_COMMANDS)
        command_snapshot_reader = CommandSnapshotReader()

        while not stop_requested and not (shm.control_flags & CONTROL_EXIT):
            if args.parent_pid > 0 and os.getppid() != args.parent_pid:
                raise RuntimeError("C++ controller parent exited")
            now = time.monotonic()
            feedback_received = read_feedback(states)
            by_base = feedback_by_base(states)
            coherent_by_base = coherent_observation_by_base(states, now)
            observation_skew_s = (
                coherent_observation_skew_s(states, coherent_by_base)
                if coherent_by_base
                else float("inf")
            )
            observation_accepted = bool(coherent_by_base) and (
                observation_skew_s * 1000.0 <= args.max_observation_skew_ms
            ) and coherent_observation_is_fresh(
                coherent_by_base, args.max_observation_age_ms
            )
            if feedback_received:
                if not coherent_by_base:
                    observation_no_common_window += 1
                elif observation_skew_s * 1000.0 > args.max_observation_skew_ms:
                    observation_skew_reject_window += 1
                elif not coherent_observation_is_fresh(
                    coherent_by_base, args.max_observation_age_ms
                ):
                    observation_freshness_reject_window += 1
            if len(by_base) == 2 and identity_time is None:
                if any(
                    feedback.status_flags & STATUS_BENCH_CAN2_REMAP
                    for feedback in by_base.values()
                ):
                    raise RuntimeError(
                        "bench CAN2 remap firmware is not allowed for 16-motor deployment"
                    )
                if any(
                    feedback.limit_mode != LIMIT_CALIBRATED_ABSOLUTE
                    or feedback.status_flags
                    & (STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED |
                       STATUS_BENCH_RELATIVE_LIMITS)
                    for feedback in by_base.values()
                ):
                    raise RuntimeError(
                        "MCU absolute calibration/limit mode is not active"
                    )
                identity_time = now

            if (
                ready_time is None
                and now - last_setup_progress_time >= 2.0
            ):
                progress: list[str] = []
                for state in states:
                    feedback = state.feedback
                    if feedback is None:
                        progress.append(f"{state.name}:no-feedback")
                        continue
                    local_mask = feedback.local_mask
                    progress.append(
                        f"{state.name}:base={feedback.bus_base},"
                        f"online=0x{feedback.online_mask & local_mask:04x},"
                        f"enabled=0x{feedback.enabled_mask & local_mask:04x},"
                        f"fast=0x{feedback.fast_feedback_valid_mask & local_mask:04x},"
                        f"flags=0x{feedback.status_flags:08x}"
                    )
                print(
                    "[WDP4 bridge] setup waiting: " + "; ".join(progress),
                    flush=True,
                )
                last_setup_progress_time = now

            if now >= next_send:
                if last_setpoint_send_time is not None:
                    setpoint_gap_window_max_ms = max(
                        setpoint_gap_window_max_ms,
                        (now - last_setpoint_send_time) * 1000.0,
                    )
                last_setpoint_send_time = now
                commands = ZERO_COMMANDS
                command_snapshot: CommandSnapshot | None = None
                if enable:
                    # Keep a coherent snapshot warm during bridge setup, when
                    # zero commands are still sent.  This guarantees a safe
                    # previous frame is available when live control begins.
                    command_snapshot = command_snapshot_reader.read(shm)
                if enable and ready_time is not None and command_snapshot is not None:
                    commands = command_snapshot.commands
                    cpp_command_seq = command_snapshot.sequence
                    if cpp_command_seq != last_cpp_command_seq:
                        last_cpp_command_seq = cpp_command_seq
                        last_cpp_command_time = now
                    elif now - ready_time > 0.5 and now - last_cpp_command_time > 0.25:
                        raise RuntimeError("stale C++ setpoint publisher")
                packet = build_setpoint_packet(
                    seq,
                    commands,
                    live_control=enable and identity_time is not None,
                    enable_request=enable and identity_time is not None,
                    timeout_ms=args.timeout_ms,
                    qualification_excitation=False,
                )
                send_packet(states, packet)
                seq += 1
                cycle_count += 1
                next_send += period
                if now - next_send > period:
                    next_send = now + period

            if now - last_hz_time >= 1.0:
                bridge_hz = (cycle_count - last_hz_cycle) / (now - last_hz_time)
                last_hz_cycle = cycle_count
                last_hz_time = now

            if now - rate_time >= 1.0 and len(by_base) == 2:
                completed = [0] * MOTOR_COUNT
                operation_received = [0] * MOTOR_COUNT
                sample_mcu_time_ms = [0] * MOTOR_COUNT
                for feedback in by_base.values():
                    first = 8 if feedback.bus_base == 2 else 0
                    for index in range(first, first + 8):
                        completed[index] = feedback.live_command_tx_completed[index]
                        operation_received[index] = (
                            feedback.operation_status_rx_count[index]
                        )
                        sample_mcu_time_ms[index] = feedback.feedback_time_ms
                for index in range(MOTOR_COUNT):
                    command_delta = (
                        completed[index] - command_rate_counts[index]
                    ) & 0xFFFFFFFF
                    feedback_delta = (
                        operation_received[index] - feedback_rate_counts[index]
                    ) & 0xFFFFFFFF
                    dt_ms = (
                        sample_mcu_time_ms[index] - rate_mcu_time_ms[index]
                    ) & 0xFFFFFFFF
                    if dt_ms > 0:
                        motor_command_hz[index] = command_delta * 1000.0 / dt_ms
                        motor_feedback_hz[index] = feedback_delta * 1000.0 / dt_ms
                command_rate_counts = completed
                feedback_rate_counts = operation_received
                rate_mcu_time_ms = sample_mcu_time_ms
                rate_time = now

                if live_enabled_time is not None and ready_time is None:
                    low_command = [
                        hz for hz in motor_command_hz if hz < args.min_command_hz
                    ]
                    low_feedback = [
                        hz for hz in motor_feedback_hz if hz < args.min_feedback_hz
                    ]
                    if not low_command and not low_feedback:
                        ready_time = now
                        shm.status_flags |= BRIDGE_STATUS_READY | BRIDGE_STATUS_ENABLED
                        set_status_message(
                            shm,
                            "WDP4 ready; command/feedback rates "
                            f">={min(args.min_command_hz, args.min_feedback_hz):g} Hz verified",
                        )
                    elif now - live_enabled_time >= args.rate_grace_s:
                        slow = [
                            f"{i}:tx={motor_command_hz[i]:.1f}Hz/"
                            f"fb={motor_feedback_hz[i]:.1f}Hz"
                            for i in range(MOTOR_COUNT)
                            if motor_command_hz[i] < args.min_command_hz
                            or motor_feedback_hz[i] < args.min_feedback_hz
                        ]
                        raise RuntimeError(
                            "initial motor command/feedback rate below threshold: "
                            + ", ".join(slow)
                        )
                elif enable and ready_time is not None and now - ready_time >= args.rate_grace_s:
                    low = [
                        index
                        for index in range(MOTOR_COUNT)
                        if motor_command_hz[index] < args.min_command_hz
                        or motor_feedback_hz[index] < args.min_feedback_hz
                    ]
                    low_rate_samples = low_rate_samples + 1 if low else 0
                    if low_rate_samples >= RUNTIME_LOW_RATE_CONSECUTIVE_WINDOWS:
                        slow = [
                            f"{i}:tx={motor_command_hz[i]:.1f}Hz/"
                            f"fb={motor_feedback_hz[i]:.1f}Hz"
                            for i in low
                        ]
                        raise RuntimeError(
                            "motor command/feedback rate below threshold: "
                            + ", ".join(slow)
                        )

            if len(by_base) == 2:
                if enable:
                    all_ready = observation_accepted and all(
                        local_feedback_ready(fb, args.max_observation_age_ms)
                        for fb in coherent_by_base.values()
                    )
                else:
                    all_ready = True
                if all_ready and not enable and ready_time is None:
                    ready_time = now
                    shm.status_flags |= BRIDGE_STATUS_READY
                    set_status_message(shm, "WDP4 dual-MCU dry-run bridge ready")
                elif all_ready and enable and live_enabled_time is None:
                    live_enabled_time = now
                    rate_time = now
                    command_rate_counts = [0] * MOTOR_COUNT
                    feedback_rate_counts = [0] * MOTOR_COUNT
                    rate_mcu_time_ms = [0] * MOTOR_COUNT
                    for feedback in by_base.values():
                        first = 8 if feedback.bus_base == 2 else 0
                        for index in range(first, first + 8):
                            command_rate_counts[index] = (
                                feedback.live_command_tx_completed[index]
                            )
                            feedback_rate_counts[index] = (
                                feedback.operation_status_rx_count[index]
                            )
                            rate_mcu_time_ms[index] = feedback.feedback_time_ms
                    set_status_message(
                        shm,
                        "static zero-command cross-check passed; measuring TX/RX rates",
                    )

            if (
                enable
                and ready_time is not None
                and len(by_base) == 2
                and TORQUE_TELEMETRY_REPORT_PERIOD_S > 0.0
                and now - last_torque_telemetry_report_time
                >= TORQUE_TELEMETRY_REPORT_PERIOD_S
            ):
                diagnostic = final_torque_diagnostic(by_base)
                if diagnostic is not None:
                    print("[WDP4 torque actual] " + diagnostic, flush=True)
                elif not torque_telemetry_missing_reported:
                    print(
                        "[WDP4 torque actual] unavailable: flash the telemetry-enabled "
                        "firmware to both MCUs",
                        flush=True,
                    )
                    torque_telemetry_missing_reported = True
                last_torque_telemetry_report_time = now

            if (
                ready_time is not None
                and len(by_base) == 2
                and MOTOR_TELEMETRY_REPORT_PERIOD_S > 0.0
                and now - last_motor_telemetry_report_time
                >= MOTOR_TELEMETRY_REPORT_PERIOD_S
            ):
                print(motor_telemetry_report(by_base), flush=True)
                last_motor_telemetry_report_time = now

            if (
                identity_time is None
                and now - bridge_start > args.setup_timeout_s
            ):
                missing = [
                    state.name for state in states if state.feedback is None
                ]
                raise RuntimeError(
                    "dual-MCU identity timeout; no WDP4 feedback from: "
                    + ", ".join(missing or ["unknown/duplicate MCU identity"])
                )

            if (
                enable
                and identity_time is not None
                and ready_time is None
                and now - identity_time > args.setup_timeout_s
            ):
                raise RuntimeError(
                    "dual-MCU static 500 Hz fresh-feedback check timed out: "
                    + fast_feedback_diagnostic(by_base)
                )

            if identity_time is not None:
                for state in states:
                    if now - state.last_feedback_time > MCU_FEEDBACK_LOSS_TIMEOUT_S:
                        raise RuntimeError(f"stale MCU feedback on {state.name}")
                    if state.feedback is not None:
                        error = critical_firmware_error(state.feedback)
                        if (
                            error == "command timeout"
                            and identity_time is not None
                            and now - identity_time <= 0.25
                        ):
                            error = None
                        if error:
                            raise RuntimeError(
                                f"{state.name}: {error}; "
                                f"{firmware_fault_diagnostic(state.feedback, commands)}"
                            )
                        communication_error = (
                            persistent_firmware_communication_error(
                                state,
                                monitor_live_control=ready_time is not None,
                            )
                        )
                        if communication_error:
                            raise RuntimeError(
                                f"{state.name}: {communication_error}; "
                                f"{firmware_fault_diagnostic(state.feedback, commands)}"
                            )
                if (
                    ready_time is not None
                    and last_coherent_observation_time is not None
                    and now - last_coherent_observation_time
                    > COHERENT_OBSERVATION_LOSS_TIMEOUT_S
                ):
                    raise RuntimeError(
                        "no bounded-skew dual-MCU observation for "
                        f"{COHERENT_OBSERVATION_LOSS_TIMEOUT_S * 1000.0:.0f} ms; "
                        + observation_rejection_diagnostic(
                            states,
                            coherent_by_base,
                            observation_skew_s,
                            args.max_observation_age_ms,
                            args.max_observation_skew_ms,
                            now,
                        )
                        + "; reject_window="
                        f"no_common:{observation_no_common_window},"
                        f"skew:{observation_skew_reject_window},"
                        f"freshness:{observation_freshness_reject_window}"
                    )

            if observation_accepted:
                observation_age_window_max_ms = max(
                    observation_age_window_max_ms,
                    float(
                        max(
                            feedback.observation_max_sample_age_ms
                            for feedback in coherent_by_base.values()
                        )
                    ),
                )
                observation_skew_window_max_ms = max(
                    observation_skew_window_max_ms,
                    observation_skew_s * 1000.0,
                )
                observation_seq = next(iter(coherent_by_base.values())).observation_seq
                if observation_seq != last_published_observation_seq:
                    if last_published_observation_seq is not None:
                        delta = (
                            observation_seq - last_published_observation_seq
                        ) & 0xFFFFFFFF
                        if 1 < delta < 0x80000000:
                            dropped_observation_epochs += delta - 1
                    last_published_observation_seq = observation_seq
                    last_coherent_observation_time = now
                    update_shared_feedback(
                        shm,
                        coherent_by_base,
                        motor_command_hz,
                        motor_feedback_hz,
                        observation_skew_s,
                        dropped_observation_epochs,
                        bridge_hz,
                        bridge_start,
                        cycle_count,
                    )
            elif feedback_received and ready_time is None:
                # Identity/rate diagnostics continue, but never publish a mixed epoch.
                pass
            if (
                USB_TIMING_REPORT_PERIOD_S > 0.0
                and now - last_usb_timing_report_time
                >= USB_TIMING_REPORT_PERIOD_S
            ):
                print(
                    "[WDP4 bridge] communication timing: "
                    + consume_usb_timing_report(states)
                    + f"; setpoint_gap_max={setpoint_gap_window_max_ms:.3f}ms"
                    + f"; observation_age_max={observation_age_window_max_ms:.1f}ms"
                    + f"; dual_mcu_skew_max={observation_skew_window_max_ms:.3f}ms"
                    + "; observation_reject="
                    f"no_common:{observation_no_common_window},"
                    f"skew:{observation_skew_reject_window},"
                    f"freshness:{observation_freshness_reject_window}"
                    + "; " + command_snapshot_reader.consume_timing_report(),
                    flush=True,
                )
                last_usb_timing_report_time = now
                setpoint_gap_window_max_ms = 0.0
                observation_age_window_max_ms = 0.0
                observation_skew_window_max_ms = 0.0
                observation_no_common_window = 0
                observation_skew_reject_window = 0
                observation_freshness_reject_window = 0
            time.sleep(0.0005)

        set_status_message(shm, "stopping WDP4 bridge")
        cleanup_to_dry_run(states, seq, args.timeout_ms)
        set_status_message(shm, "WDP4 bridge stopped")
        return 0
    except Exception as exc:
        shm.status_flags |= BRIDGE_STATUS_FAULT
        set_status_message(shm, f"WDP4 bridge fault: {exc}")
        print(f"[WDP4 bridge] FATAL: {exc}", file=sys.stderr, flush=True)
        cleanup_to_dry_run(states, seq, args.timeout_ms)
        return 1
    finally:
        for state in states:
            try:
                state.device.close()
            except Exception:
                pass
        del shm
        shm_obj.close()


if __name__ == "__main__":
    raise SystemExit(main())
