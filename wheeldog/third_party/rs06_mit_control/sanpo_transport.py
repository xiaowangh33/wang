"""SANPO 四路 CAN 网关串口传输。

实测 SANPO 板（COM10）：
- 发送：灵足 AT 帧（4154...），无需通道前缀
- 接收：SANPO ET 帧（4554...），第 3 字节为 CAN 通道号
"""

from __future__ import annotations

from typing import Literal

from transport import UsbCanTransport

TransportMode = Literal["lingzu", "sanpo_prefix", "sanpo"]


class SanpoCanTransport(UsbCanTransport):
    """灵足 AT 发送 + 可选 SANPO 通道前缀。"""

    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        timeout: float = 0.05,
        mode: TransportMode = "sanpo",
    ):
        super().__init__(port, baudrate, timeout)
        self.mode = mode

    def send_on_channel(
        self,
        channel: int,
        frame: bytes,
        description: str = "",
        verbose: bool = True,
    ) -> None:
        if self.mode == "sanpo_prefix":
            payload = bytes([channel & 0x03]) + frame
        else:
            payload = frame
        self.send(payload, description, verbose)

    def send_broadcast(self, frame: bytes, description: str = "", verbose: bool = True) -> None:
        self.send(frame, description, verbose)
