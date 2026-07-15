"""灵足时代 RobStride 私有 CAN 协议与 MIT 运控编解码。

帧格式与 C:\\Users\\DELL\\Desktop\\new\\爬壁\\ESP32-S3-RS485-CAN-Demo 中
gamepad_hybrid_motor_control.py / WS_Motor.cpp 实测可用逻辑对齐：
- 标准命令 CAN ID: TT 00 FD MM（29bit 扩展帧，MSB 先发）
- USB 转 CAN 串口在 8 字节载荷前增加 08 前缀（init 等短帧除外）
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable, Optional

from config import MotorLimits, MitSetpoint

COMM_GET_DEVICE_ID = 0x00
COMM_OPERATION_CONTROL = 0x01
COMM_OPERATION_STATUS = 0x02
COMM_START = 0x03
COMM_ENABLE_DISABLE = 0x04
COMM_SET_ZERO = 0x06
COMM_SET_CAN_ID = 0x07
COMM_READ_PARAM = 0x11
COMM_WRITE_PARAM = 0x12
COMM_SAVE_PARAM = 0x16

PARAM_RUN_MODE = 0x7005
PARAM_IQ_REF = 0x7006
PARAM_SPD_REF = 0x700A
PARAM_I_LIMIT = 0x7018
PARAM_MECH_POS = 0x7019
PARAM_MECH_VEL = 0x701B
PARAM_VBUS = 0x701C
PARAM_ACC = 0x7022

RUN_MODE_MIT = 0
RUN_MODE_SPEED = 2
RUN_MODE_CURRENT = 3

CAN2COM_MODE_MAP = {
    0x00: 0x00,
    0x01: 0x0C,
    0x02: 0x14,
    0x03: 0x18,
    0x04: 0x20,
    0x12: 0x90,
}
CAN2COM_MOTOR_MAP = {
    0x01: 0x0C,
    0x02: 0x14,
    0x03: 0x1C,
    0x04: 0x24,
}


@dataclass
class MotorFeedback:
    position: float
    velocity: float
    torque: float
    temperature: float
    fault_bits: int = 0
    mode_status: int = 0
    raw_can_id: int = 0
    raw_data: bytes = b""


def build_standard_can_id(comm_type: int, motor_id: int) -> int:
    """构造 TT 00 FD MM 格式扩展帧 ID。"""
    return ((comm_type & 0x1F) << 24) | (0x00 << 16) | (0xFD << 8) | (motor_id & 0xFF)


def build_mit_can_id(motor_id: int, torque_extra: int) -> int:
    """MIT 运控：16 位数据区放力矩量化值。"""
    return (COMM_OPERATION_CONTROL << 24) | ((torque_extra & 0xFFFF) << 8) | (motor_id & 0xFF)


def _signed_offset_u16(value: float, limit: float) -> int:
    clamped = max(-limit, min(limit, value))
    raw = int(((clamped / limit) + 1.0) * 0x7FFF)
    return max(0, min(0xFFFF, raw))


def _unsigned_linear_u16(value: float, limit: float) -> int:
    clamped = max(0.0, min(limit, value))
    raw = int((clamped / limit) * 0xFFFF)
    return max(0, min(0xFFFF, raw))


def _signed_offset_to_float(raw: int, limit: float) -> float:
    return ((raw / 0x7FFF) - 1.0) * limit


def encode_mit_setpoint(setpoint: MitSetpoint, limits: MotorLimits) -> tuple[int, bytes]:
    pos_u = _signed_offset_u16(setpoint.position, limits.p_max)
    vel_u = _signed_offset_u16(setpoint.velocity, limits.v_max)
    kp_u = _unsigned_linear_u16(setpoint.kp, limits.kp_max)
    kd_u = _unsigned_linear_u16(setpoint.kd, limits.kd_max)
    tau_u = _signed_offset_u16(setpoint.torque, limits.t_max)
    payload = struct.pack(">HHHH", pos_u, vel_u, kp_u, kd_u)
    return tau_u, payload


def decode_mit_feedback(data: bytes, limits: MotorLimits) -> MotorFeedback:
    if len(data) < 8:
        raise ValueError(f"反馈数据长度不足: {len(data)}")
    pos_u, vel_u, tau_u, temp_u = struct.unpack(">HHHH", data[:8])
    return MotorFeedback(
        position=_signed_offset_to_float(pos_u, limits.p_max),
        velocity=_signed_offset_to_float(vel_u, limits.v_max),
        torque=_signed_offset_to_float(tau_u, limits.t_max),
        temperature=temp_u * 0.1,
        raw_data=data,
    )


def pack_candata_hex(can_id: int, bus_data: bytes = b"", use_usb_wrapper: bool = True) -> str:
    """打包逻辑 CAN 十六进制串（can2com 输入）。"""
    id_hex = f"{can_id:08x}"
    if not bus_data:
        return id_hex
    if len(bus_data) <= 2 and ((can_id >> 24) & 0xFF) == COMM_GET_DEVICE_ID:
        return id_hex + bus_data.hex()
    bus_data = bus_data.ljust(8, b"\x00")[:8]
    if use_usb_wrapper:
        return id_hex + "08" + bus_data.hex()
    return id_hex + bus_data.hex()


def can_id_to_at_header(can_id: int) -> str:
    """29bit CAN ID 左移 3 位 → AT 帧 4 字节扩展域（手册 3.3.5）。"""
    shifted = (can_id & 0xFFFFFFFF) << 3
    return (
        f"{(shifted >> 24) & 0xFF:02x}"
        f"{(shifted >> 16) & 0xFF:02x}"
        f"{(shifted >> 8) & 0xFF:02x}"
        f"{shifted & 0xFF:02x}"
    )


def can2com(candata: str) -> bytes:
    """逻辑 CAN 帧 → 灵足 USB-CAN AT 串口帧。"""
    can_header = candata[:8]
    data_part = candata[8:]
    comm_type = int(can_header[0:2], 16)
    motor_id = int(can_header[6:8], 16)
    can_id = int(can_header, 16)

    if motor_id in CAN2COM_MOTOR_MAP and comm_type in CAN2COM_MODE_MAP:
        xx = f"{CAN2COM_MODE_MAP[comm_type]:02x}"
        yy = f"{CAN2COM_MOTOR_MAP[motor_id]:02x}"
        header = xx + "07e8" + yy
    else:
        header = can_id_to_at_header(can_id)

    return bytes.fromhex("4154" + header + data_part + "0d0a")


def float_to_bytes(value: float) -> bytes:
    return struct.pack("<f", value)


def int_to_param_bytes(value: int) -> bytes:
    return struct.pack("<I", value & 0xFFFFFFFF)


def _wrap_serial(candata: str) -> bytes:
    return can2com(candata)


def build_init_command(motor_id: int) -> bytes:
    can_id = build_standard_can_id(COMM_GET_DEVICE_ID, motor_id)
    return _wrap_serial(pack_candata_hex(can_id, b"\x01\x00"))


def build_enable_command(motor_id: int) -> bytes:
    """类型 0x04 + 数据 00 C4 ...（与 ESP32/gamepad 一致）。"""
    can_id = build_standard_can_id(COMM_ENABLE_DISABLE, motor_id)
    bus_data = bytes([0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    return _wrap_serial(pack_candata_hex(can_id, bus_data))


def build_start_command(motor_id: int) -> bytes:
    """类型 0x03 启动运行。"""
    can_id = build_standard_can_id(COMM_START, motor_id)
    return _wrap_serial(pack_candata_hex(can_id, b"\x00" * 8))


def build_disable_command(motor_id: int, clear_error: bool = False) -> bytes:
    can_id = build_standard_can_id(COMM_ENABLE_DISABLE, motor_id)
    data = b"\x01\x00\x00\x00\x00\x00\x00\x00" if clear_error else b"\x00" * 8
    return _wrap_serial(pack_candata_hex(can_id, data))


def build_mit_command(motor_id: int, setpoint: MitSetpoint, limits: MotorLimits) -> bytes:
    torque_extra, payload = encode_mit_setpoint(setpoint, limits)
    can_id = build_mit_can_id(motor_id, torque_extra)
    return _wrap_serial(pack_candata_hex(can_id, payload))


def build_write_param_command(motor_id: int, param_index: int, value_bytes: bytes) -> bytes:
    if len(value_bytes) > 4:
        raise ValueError("单参数写入最多 4 字节")
    data = bytearray(8)
    struct.pack_into("<H", data, 0, param_index)
    data[4:4 + len(value_bytes)] = value_bytes
    can_id = build_standard_can_id(COMM_WRITE_PARAM, motor_id)
    return _wrap_serial(pack_candata_hex(can_id, bytes(data)))


def build_read_param_command(motor_id: int, param_index: int) -> bytes:
    data = bytearray(8)
    struct.pack_into("<H", data, 0, param_index)
    can_id = build_standard_can_id(COMM_READ_PARAM, motor_id)
    return _wrap_serial(pack_candata_hex(can_id, bytes(data)))


def build_set_run_mode_command(motor_id: int, run_mode: int) -> bytes:
    return build_write_param_command(motor_id, PARAM_RUN_MODE, int_to_param_bytes(run_mode))


def build_write_param_f32_command(motor_id: int, param_index: int, value: float) -> bytes:
    return build_write_param_command(motor_id, param_index, float_to_bytes(value))


def build_set_iq_ref_command(motor_id: int, iq_amps: float) -> bytes:
    """电流/力矩模式：写 iq_ref (0x7006)，单位 Apeak。"""
    return build_write_param_f32_command(motor_id, PARAM_IQ_REF, iq_amps)


def build_set_current_limit_command(motor_id: int, current_limit_a: float) -> bytes:
    """写电流限制 limit_cur (0x7018)，单位 Apeak。"""
    return build_write_param_f32_command(motor_id, PARAM_I_LIMIT, current_limit_a)


def build_set_spd_ref_command(motor_id: int, speed_rad_s: float) -> bytes:
    """速度模式：写 spd_ref (0x700A)，单位 rad/s。"""
    return build_write_param_f32_command(motor_id, PARAM_SPD_REF, speed_rad_s)


def build_set_accel_command(motor_id: int, accel_rad_s2: float) -> bytes:
    """速度模式：写加速度限制 acc (0x7022)，单位 rad/s²。"""
    return build_write_param_f32_command(motor_id, PARAM_ACC, accel_rad_s2)


def lingzu_serial_to_can_payload(serial_frame: bytes) -> Optional[tuple[int, bytes]]:
    """灵足 AT 串口帧 → (can_id, bus_data)，供 SANPO ET 网关发送。"""
    return at_frame_to_can_id_and_data(serial_frame)


def decode_read_param_u32(can_id: int, data: bytes, param_index: int) -> Optional[int]:
    """解析 type 0x11 读参数应答中的 32 位整型值。"""
    if (can_id >> 24) & 0x1F != COMM_READ_PARAM:
        return None
    if len(data) < 8:
        return None
    idx = struct.unpack_from("<H", data, 0)[0]
    if idx != param_index:
        return None
    return struct.unpack_from("<I", data, 4)[0]


def build_broadcast_run_mode_command(run_mode: int = RUN_MODE_MIT) -> bytes:
    """上位机抓包：先对 motor_id=0 写 run_mode（WS_Motor.cpp speed_mode_motor0）。"""
    return build_set_run_mode_command(0, run_mode)


def build_set_zero_command(motor_id: int) -> bytes:
    can_id = build_standard_can_id(COMM_SET_ZERO, motor_id)
    return _wrap_serial(pack_candata_hex(can_id, b"\x00" * 8))


def build_get_device_id_command(motor_id: int) -> bytes:
    return build_init_command(motor_id)


def iter_at_frames(raw: bytes) -> Iterable[bytes]:
    """从串口原始字节流中提取 AT 帧。"""
    start = 0
    while True:
        idx = raw.find(b"AT", start)
        if idx < 0:
            return
        end = raw.find(b"\x0d\x0a", idx)
        if end < 0:
            return
        yield raw[idx:end + 2]
        start = end + 2


def iter_et_frames(raw: bytes) -> Iterable[bytes]:
    """SANPO 等网关回传 ET 帧（4554...0d0a），结构与 AT 类似但 CAN ID 不左移。"""
    start = 0
    while True:
        idx = raw.find(b"ET", start)
        if idx < 0:
            return
        end = raw.find(b"\x0d\x0a", idx)
        if end < 0:
            return
        yield raw[idx:end + 2]
        start = end + 2


def et_frame_channel(frame: bytes) -> Optional[int]:
    """ET 帧第 3 字节为 CAN 通道号（0~3）。"""
    if len(frame) < 3 or frame[:2] != b"ET":
        return None
    return frame[2]


def et_frame_to_can_id_and_data(frame: bytes) -> Optional[tuple[int, bytes]]:
    """解析 ET 回传：ET + channel(1) + can_id(4, 未移位) + payload + CRLF。"""
    if len(frame) < 10 or frame[:2] != b"ET":
        return None
    can_id = (frame[3] << 24) | (frame[4] << 16) | (frame[5] << 8) | frame[6]
    payload = frame[7:-2]
    if payload.startswith(b"\x08"):
        bus_data = payload[1:9]
    else:
        bus_data = payload[:8]
    return can_id, bus_data


def iter_adapter_frames(raw: bytes) -> Iterable[tuple[bytes, str]]:
    """提取 AT/ET 帧，返回 (frame, kind)。"""
    seen = set()
    for kind, finder in (("AT", iter_at_frames), ("ET", iter_et_frames)):
        for frame in finder(raw):
            key = (kind, frame.hex())
            if key in seen:
                continue
            seen.add(key)
            yield frame, kind


def shifted_can_id_from_at(frame: bytes) -> Optional[int]:
    """按手册 3.3.5：AT 帧中 4 字节扩展 ID 右移 3 位得真实 CAN ID。"""
    if len(frame) < 6 or frame[:2] != b"AT":
        return None
    raw = (frame[2] << 24) | (frame[3] << 16) | (frame[4] << 8) | frame[5]
    return raw >> 3


def at_frame_to_can_id_and_data(frame: bytes) -> Optional[tuple[int, bytes]]:
    """解析 AT 回传帧，还原逻辑 CAN ID 与总线数据。"""
    if len(frame) < 8 or frame[:2] != b"AT":
        return None

    can_id = shifted_can_id_from_at(frame)
    if can_id is None:
        return None

    comm_type = (can_id >> 24) & 0x1F
    motor_id = can_id & 0xFF

    # 手册 4.1.1：type0 应答 ID 中 16 位域为 (motor_id<<8)|0xFE，与标准 TT 00 FD MM 不同
    if comm_type == COMM_GET_DEVICE_ID:
        mid = (can_id >> 8) & 0xFF
        if mid != 0xFE:
            motor_id = mid

    payload = frame[6:-2]
    if payload.startswith(b"\x08"):
        bus_data = payload[1:]
    else:
        bus_data = payload
    bus_data = bus_data[:8]
    return can_id, bus_data


def parse_adapter_buffer(raw: bytes) -> list[tuple[int, bytes]]:
    results = []
    for frame, kind in iter_adapter_frames(raw):
        if kind == "AT":
            parsed = at_frame_to_can_id_and_data(frame)
        else:
            parsed = et_frame_to_can_id_and_data(frame)
        if parsed:
            results.append(parsed)
    return results


def parse_adapter_buffer_with_channel(raw: bytes) -> list[tuple[int, int, bytes]]:
    """解析 AT/ET 帧，ET 帧附带 CAN 通道号；AT 帧 channel=-1。"""
    results: list[tuple[int, int, bytes]] = []
    for frame, kind in iter_adapter_frames(raw):
        if kind == "AT":
            parsed = at_frame_to_can_id_and_data(frame)
            if parsed:
                results.append((-1, parsed[0], parsed[1]))
        else:
            parsed = et_frame_to_can_id_and_data(frame)
            ch = et_frame_channel(frame)
            if parsed and ch is not None:
                results.append((ch, parsed[0], parsed[1]))
    return results


def decode_feedback_from_can_id(can_id: int, data: bytes, limits: MotorLimits) -> Optional[MotorFeedback]:
    comm_type = (can_id >> 24) & 0x1F
    if comm_type == COMM_GET_DEVICE_ID:
        return None
    if comm_type != COMM_OPERATION_STATUS:
        return None
    fault_mode = (can_id >> 16) & 0xFFFF
    feedback = decode_mit_feedback(data, limits)
    feedback.raw_can_id = can_id
    feedback.fault_bits = fault_mode & 0x3F
    feedback.mode_status = (fault_mode >> 6) & 0x03
    return feedback


def motor_id_from_can_id(can_id: int) -> int:
    """从 29bit CAN ID 提取目标电机 ID。"""
    comm_type = (can_id >> 24) & 0x1F
    if comm_type in (COMM_GET_DEVICE_ID, COMM_OPERATION_STATUS):
        return (can_id >> 8) & 0xFF
    return can_id & 0xFF


def decode_device_id_response(can_id: int, data: bytes) -> Optional[tuple[int, bytes]]:
    """解析 type0 获取设备 ID 应答，返回 (motor_id, 8字节唯一标识)。"""
    if (can_id >> 24) & 0x1F != COMM_GET_DEVICE_ID:
        return None
    motor_id = (can_id >> 8) & 0xFF
    if motor_id == 0xFE:
        return None
    return motor_id, data[:8]


def build_set_can_id_command(
    old_id: int,
    new_id: int,
    host_id: int = 0xFD,
    uuid: bytes = b"",
) -> bytes:
    """通信类型 7：修改电机 CAN ID。扩展 ID = 0x07 [new_id] [host_id] [old_id]。"""
    can_id = (
        (COMM_SET_CAN_ID << 24)
        | ((new_id & 0xFF) << 16)
        | ((host_id & 0xFF) << 8)
        | (old_id & 0xFF)
    )
    bus_data = (uuid[:8] if uuid else b"").ljust(8, b"\x00")
    return _wrap_serial(pack_candata_hex(can_id, bus_data))


def build_save_params_command(motor_id: int) -> bytes:
    """通信类型 22 (0x16)：保存参数到 Flash。"""
    can_id = build_standard_can_id(COMM_SAVE_PARAM, motor_id)
    return _wrap_serial(pack_candata_hex(can_id, bytes(range(1, 9))))
