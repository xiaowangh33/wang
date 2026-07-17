"""WDP4 PC<->MCU setpoint and feedback protocol helpers."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

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

CONTROL_FLAG_ENABLE_REQUEST = 1 << 0
CONTROL_FLAG_ESTOP = 1 << 1
CONTROL_FLAG_DRY_RUN = 1 << 2
CONTROL_FLAG_READONLY_POLL = 1 << 3
CONTROL_FLAG_READONLY_POLL_CAN2 = 1 << 4
CONTROL_FLAG_LIVE_CONTROL = 1 << 5
CONTROL_FLAG_QUALIFICATION_EXCITATION = 1 << 6
CONTROL_FLAG_CALIBRATION_READONLY = 1 << 7

STATUS_COMMAND_TIMEOUT = 1 << 2
STATUS_ESTOP = 1 << 3
STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED = 1 << 6
STATUS_CONTROL_TASK_FALLBACK = 1 << 7
STATUS_CAN_RX_OVERFLOW = 1 << 8
STATUS_TORQUE_SLEW_LIMITED = 1 << 11
STATUS_VELOCITY_GUARD = 1 << 12
STATUS_COMMAND_CLIPPED = 1 << 13
STATUS_CAN_TX_ERROR = 1 << 15
STATUS_BENCH_CAN2_REMAP = 1 << 16
STATUS_LIVE_CONTROL_ACTIVE = 1 << 18
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
JOINT_COMMAND = struct.Struct("<fffff")
FEEDBACK_PREFIX = struct.Struct("<IIIIIIIIIIIIII f B 3x")
JOINT_FEEDBACK = struct.Struct("<ffffI BBBB")
CAN_DIAGNOSTICS = struct.Struct("<IIII")
LAST_CAN = struct.Struct("<IIII")
MOTOR_COUNTERS = struct.Struct("<" + "I" * MOTOR_COUNT)
MOTOR_FLOATS = struct.Struct("<" + "f" * MOTOR_COUNT)
MOTOR_BYTES = struct.Struct("<" + "B" * MOTOR_COUNT)
FAST_VALID_MASK = struct.Struct("<I")
OBSERVATION_DIAGNOSTICS = struct.Struct("<IIHH" + "H" * MOTOR_COUNT)
LIVE_STOP_DIAGNOSTICS = struct.Struct("<IIIIII")


@dataclass(frozen=True)
class JointCommand:
    kp: float = 0.0
    q_des: float = 0.0
    kd: float = 0.0
    dq_des: float = 0.0
    tau_ff: float = 0.0


@dataclass(frozen=True)
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
class McuFeedback:
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
    live_command_tx_queued: tuple[int, ...] = field(
        default_factory=lambda: (0,) * MOTOR_COUNT
    )
    live_command_tx_completed: tuple[int, ...] = field(
        default_factory=lambda: (0,) * MOTOR_COUNT
    )
    live_command_tx_deferred: tuple[int, ...] = field(
        default_factory=lambda: (0,) * MOTOR_COUNT
    )
    operation_status_rx_count: tuple[int, ...] = field(
        default_factory=lambda: (0,) * MOTOR_COUNT
    )
    fast_feedback_valid_mask: int = 0
    fast_position_excited_mask: int = 0
    fast_velocity_excited_mask: int = 0
    fast_feedback_rate_hz: tuple[float, ...] = field(
        default_factory=lambda: (0.0,) * MOTOR_COUNT
    )
    fast_position_error_rad: tuple[float, ...] = field(
        default_factory=lambda: (0.0,) * MOTOR_COUNT
    )
    fast_velocity_error_radps: tuple[float, ...] = field(
        default_factory=lambda: (0.0,) * MOTOR_COUNT
    )
    observation_seq: int = 0
    observation_time_ms: int = 0
    observation_max_sample_age_ms: int = 0
    observation_flags: int = 0
    observation_sample_age_ms: tuple[int, ...] = field(
        default_factory=lambda: (0xFFFF,) * MOTOR_COUNT
    )
    live_stop_reason_flags: int = 0
    live_stop_motor_mask: int = 0
    live_stop_event_count: int = 0
    live_stop_trigger_time_ms: int = 0
    live_stop_fast_valid_mask: int = 0
    live_stop_fault_mask: int = 0
    operation_status_max_gap_ms: tuple[int, ...] = field(
        default_factory=lambda: (0,) * MOTOR_COUNT
    )
    live_stop_fast_age_ms: tuple[int, ...] = field(
        default_factory=lambda: (0,) * MOTOR_COUNT
    )
    final_joint_torque_cmd_nm: tuple[float, ...] = field(
        default_factory=lambda: (0.0,) * MOTOR_COUNT
    )
    final_torque_telemetry_present: bool = False
    supply_voltage_valid_mask: int = 0
    supply_voltage_v: tuple[float, ...] = field(
        default_factory=lambda: (0.0,) * MOTOR_COUNT
    )
    supply_voltage_telemetry_present: bool = False

    @property
    def bus_base(self) -> int:
        return 2 if self.status_flags & STATUS_MCU_CAN_BASE_2 else 0

    @property
    def local_mask(self) -> int:
        return 0xFF00 if self.bus_base == 2 else 0x00FF


def crc16_ccitt(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)) & 0xFFFF
    return crc


def build_packet(packet_type: int, seq: int, payload: bytes = b"") -> bytes:
    packet = bytearray(
        HEADER.pack(MAGIC, VERSION, packet_type, len(payload), seq & 0xFFFFFFFF, 0, 0)
    )
    packet += payload
    struct.pack_into("<H", packet, 12, crc16_ccitt(packet))
    return bytes(packet)


def build_setpoint_packet(
    seq: int,
    commands: list[JointCommand],
    *,
    live_control: bool,
    enable_request: bool,
    timeout_ms: int,
    estop: bool = False,
    qualification_excitation: bool = False,
    poll_readonly: bool = False,
    poll_can2: bool = False,
    calibration_readonly: bool = False,
) -> bytes:
    if len(commands) != MOTOR_COUNT:
        raise ValueError(f"expected {MOTOR_COUNT} joint commands, got {len(commands)}")

    flags = CONTROL_FLAG_DRY_RUN
    if live_control:
        flags = CONTROL_FLAG_LIVE_CONTROL
    if enable_request:
        flags |= CONTROL_FLAG_ENABLE_REQUEST
    if estop:
        flags |= CONTROL_FLAG_ESTOP
    if qualification_excitation:
        flags |= CONTROL_FLAG_QUALIFICATION_EXCITATION
    if poll_readonly or calibration_readonly:
        flags |= CONTROL_FLAG_READONLY_POLL
    if poll_can2:
        flags |= CONTROL_FLAG_READONLY_POLL_CAN2
    if calibration_readonly:
        if live_control or enable_request:
            raise ValueError("calibration_readonly requires dry-run with enable off")
        flags |= CONTROL_FLAG_CALIBRATION_READONLY

    modes = bytearray([ACTUATOR_POSITION_PD] * MOTOR_COUNT)
    for index in (3, 7, 11, 15):
        modes[index] = ACTUATOR_VELOCITY

    payload = bytearray(
        struct.pack("<IIIB", flags, 0, timeout_ms, LIMIT_CALIBRATED_ABSOLUTE)
    )
    payload += modes
    payload += b"\0" * 3
    for command in commands:
        payload += JOINT_COMMAND.pack(
            command.kp,
            command.q_des,
            command.kd,
            command.dq_des,
            command.tau_ff,
        )
    if len(payload) != 352:
        raise AssertionError(f"bad WDP4 setpoint payload size: {len(payload)}")
    return build_packet(PACKET_SETPOINT, seq, bytes(payload))


def parse_feedback(payload: bytes) -> McuFeedback | None:
    base_size = FEEDBACK_PREFIX.size + MOTOR_COUNT * JOINT_FEEDBACK.size
    if len(payload) < base_size:
        return None
    values = FEEDBACK_PREFIX.unpack_from(payload, 0)
    offset = FEEDBACK_PREFIX.size
    joints: list[JointFeedback] = []
    for _ in range(MOTOR_COUNT):
        joints.append(JointFeedback(*JOINT_FEEDBACK.unpack_from(payload, offset)))
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
    if len(payload) >= offset + FAST_VALID_MASK.size:
        (fast_valid_mask,) = FAST_VALID_MASK.unpack_from(payload, offset)
        offset += FAST_VALID_MASK.size
    fast_position_excited_mask = 0
    if len(payload) >= offset + FAST_VALID_MASK.size:
        (fast_position_excited_mask,) = FAST_VALID_MASK.unpack_from(payload, offset)
        offset += FAST_VALID_MASK.size
    fast_velocity_excited_mask = 0
    if len(payload) >= offset + FAST_VALID_MASK.size:
        (fast_velocity_excited_mask,) = FAST_VALID_MASK.unpack_from(payload, offset)
        offset += FAST_VALID_MASK.size
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
    if len(payload) >= offset + FAST_VALID_MASK.size + MOTOR_FLOATS.size:
        (supply_voltage_valid_mask,) = FAST_VALID_MASK.unpack_from(payload, offset)
        offset += FAST_VALID_MASK.size
        supply_voltage_v = MOTOR_FLOATS.unpack_from(payload, offset)
        offset += MOTOR_FLOATS.size
        supply_voltage_telemetry_present = True
    return McuFeedback(
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


class PacketParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.bad_packets = 0

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        self.buffer.extend(data)
        packets: list[tuple[int, int, bytes]] = []
        while len(self.buffer) >= HEADER.size:
            start = self.buffer.find(b"WDP4")
            if start < 0:
                del self.buffer[:-3]
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            magic, version, packet_type, payload_size, seq, got_crc, _ = HEADER.unpack_from(
                self.buffer, 0
            )
            if magic != MAGIC or version != VERSION or payload_size > MAX_PAYLOAD:
                self.bad_packets += 1
                del self.buffer[0]
                continue
            total = HEADER.size + payload_size
            if len(self.buffer) < total:
                break
            packet = bytearray(self.buffer[:total])
            struct.pack_into("<H", packet, 12, 0)
            if crc16_ccitt(packet) != got_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            packets.append((packet_type, seq, bytes(self.buffer[HEADER.size:total])))
            del self.buffer[:total]
        return packets


ZERO_COMMANDS = [JointCommand() for _ in range(MOTOR_COUNT)]
