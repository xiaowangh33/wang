"""SANPO 双 MCU ET 协议传输（默认 COM10=CAN1/2，COM12=CAN3/4）。"""

from __future__ import annotations

import time
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore

from protocol import (
    et_frame_to_can_id_and_data,
    iter_et_frames,
    lingzu_serial_to_can_payload,
)
from sanpo_bus_map import MCU1_COM, MCU2_COM

ET_HEADER = bytes([0x45, 0x54])
ET_TAIL = bytes([0x0D, 0x0A])

def build_can_routes(mcu1_com: str = MCU1_COM, mcu2_com: str = MCU2_COM) -> list[tuple[str, str, int]]:
    """返回 CAN1~CAN4 到双 MCU 串口的映射。"""
    return [
        ("CAN1", mcu1_com, 0x01),
        ("CAN2", mcu1_com, 0x02),
        ("CAN3", mcu2_com, 0x03),
        ("CAN4", mcu2_com, 0x04),
    ]


CAN_ROUTES: list[tuple[str, str, int]] = build_can_routes()

CAN_LABEL_TO_CHANNEL = {label: ch for label, _, ch in CAN_ROUTES}
CAN_CHANNEL_TO_LABEL = {ch: label for label, _, ch in CAN_ROUTES}

# ET 发送用全局 Channel 1~4；COM12 回传可能为本地 1/2
TX_CHANNEL_BY_CAN_INDEX = [0x01, 0x02, 0x03, 0x04]

def build_rx_channel_map(
    mcu1_com: str = MCU1_COM,
    mcu2_com: str = MCU2_COM,
) -> dict[tuple[str, int], int]:
    """返回 ET 回传 channel 到 0-based CAN index 的映射。"""
    return {
        (mcu1_com, 0x01): 0,
        (mcu1_com, 0x02): 1,
        (mcu2_com, 0x01): 2,
        (mcu2_com, 0x02): 3,
        (mcu2_com, 0x03): 2,
        (mcu2_com, 0x04): 3,
    }


RX_CHANNEL_TO_CAN_INDEX: dict[tuple[str, int], int] = build_rx_channel_map()


def build_et_frame(channel: int, can_id: int, data: bytes) -> bytes:
    data = data[:8]
    frame = bytearray(ET_HEADER)
    frame.append(channel & 0xFF)
    frame.extend(can_id.to_bytes(4, byteorder="big"))
    frame.append(len(data))
    frame.extend(data)
    frame.extend(ET_TAIL)
    return bytes(frame)


def parse_et_response(raw: bytes, com: str = "") -> list[tuple[str, int, int, bytes]]:
    """返回 [(com, channel, can_id, data), ...]"""
    out: list[tuple[str, int, int, bytes]] = []
    for frame in iter_et_frames(raw):
        if len(frame) < 10:
            continue
        ch = frame[2]
        parsed = et_frame_to_can_id_and_data(frame)
        if parsed:
            can_id, data = parsed
            out.append((com, ch, can_id, data))
    return out


def can_index_from_rx(
    com: str,
    rx_channel: int,
    mcu1_com: str = MCU1_COM,
    mcu2_com: str = MCU2_COM,
) -> Optional[int]:
    return build_rx_channel_map(mcu1_com, mcu2_com).get((com, rx_channel))


class SanpoEtBus:
    """双串口 SANPO ET 网关。"""

    def __init__(
        self,
        baudrate: int = 921600,
        wait_ms: float = 80.0,
        mcu1_com: str = MCU1_COM,
        mcu2_com: str = MCU2_COM,
    ):
        self.baudrate = baudrate
        self.wait_ms = wait_ms
        self.mcu1_com = mcu1_com
        self.mcu2_com = mcu2_com
        self.can_routes = build_can_routes(mcu1_com, mcu2_com)
        self.rx_channel_to_can_index = build_rx_channel_map(mcu1_com, mcu2_com)
        self._serials: dict[str, serial.Serial] = {}

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("请安装 pyserial")
        for com in (self.mcu1_com, self.mcu2_com):
            s = serial.Serial(
                com, self.baudrate, timeout=0.05, dsrdtr=False, rtscts=False
            )
            s.dtr = True
            s.rts = False
            time.sleep(0.05)
            self._serials[com] = s

    def close(self) -> None:
        for s in self._serials.values():
            s.close()
        self._serials.clear()

    def _com_for_channel(self, channel: int) -> str:
        for _, com, ch in self.can_routes:
            if ch == channel:
                return com
        raise ValueError(f"无效 CAN channel: {channel}")

    def can_index_from_rx(self, com: str, rx_channel: int) -> Optional[int]:
        return self.rx_channel_to_can_index.get((com, rx_channel))

    def tx_channel_for_can_index(self, can_index: int) -> int:
        if not 0 <= can_index < 4:
            raise ValueError(f"can_index 须为 0~3，收到 {can_index}")
        return TX_CHANNEL_BY_CAN_INDEX[can_index]

    def write_can(self, channel: int, can_id: int, data: bytes) -> None:
        """仅发送，不等待回传（批量控制用）。"""
        com = self._com_for_channel(channel)
        self._serials[com].write(build_et_frame(channel, can_id, data))

    def write_lingzu(self, channel: int, lingzu_frame: bytes) -> None:
        payload = lingzu_serial_to_can_payload(lingzu_frame)
        if not payload:
            raise ValueError("无法解析灵足串口帧")
        can_id, data = payload
        self.write_can(channel, can_id, data)

    def flush_tx(self) -> None:
        for ser in self._serials.values():
            ser.flush()

    def drain_all(self, wait_ms: float = 8.0) -> list[tuple[str, int, int, bytes]]:
        """收集两路 COM 上的 ET 回传。"""
        deadline = time.time() + wait_ms / 1000.0
        out: list[tuple[str, int, int, bytes]] = []
        while time.time() < deadline:
            got = False
            for com, ser in self._serials.items():
                n = ser.in_waiting
                if n:
                    raw = ser.read(n)
                    out.extend(parse_et_response(raw, com))
                    got = True
            if not got:
                time.sleep(0.001)
        for com, ser in self._serials.items():
            if ser.in_waiting:
                out.extend(parse_et_response(ser.read(ser.in_waiting), com))
        return out

    def send_lingzu(
        self,
        channel: int,
        lingzu_frame: bytes,
        wait_ms: Optional[float] = None,
    ) -> list[tuple[int, int, bytes]]:
        """发送灵足 AT 帧（自动转 CAN 载荷），返回 ET 回传列表。"""
        payload = lingzu_serial_to_can_payload(lingzu_frame)
        if not payload:
            raise ValueError("无法解析灵足串口帧")
        can_id, data = payload
        return self.send_can(channel, can_id, data, wait_ms)

    def send_can(
        self,
        channel: int,
        can_id: int,
        data: bytes,
        wait_ms: Optional[float] = None,
    ) -> list[tuple[int, int, bytes]]:
        com = self._com_for_channel(channel)
        wait = self.wait_ms if wait_ms is None else wait_ms
        self._serials[com].reset_input_buffer()
        self.write_can(channel, can_id, data)
        self.flush_tx()
        time.sleep(wait / 1000.0)
        raw = b""
        ser = self._serials[com]
        deadline = time.time() + wait / 1000.0 * 2
        while time.time() < deadline:
            if ser.in_waiting:
                raw += ser.read(ser.in_waiting)
            else:
                time.sleep(0.005)
        if ser.in_waiting:
            raw += ser.read(ser.in_waiting)
        parsed = parse_et_response(raw, com)
        return [(ch, cid, dat) for _c, ch, cid, dat in parsed]

    def send_can_label(
        self,
        can_label: str,
        lingzu_frame: bytes,
        wait_ms: Optional[float] = None,
    ) -> list[tuple[int, int, bytes]]:
        ch = CAN_LABEL_TO_CHANNEL[can_label]
        return self.send_lingzu(ch, lingzu_frame, wait_ms)
