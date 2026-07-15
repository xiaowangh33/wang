"""SANPO 开发板 ET 回传帧第 3 字节（bus_index）与物理接口映射。

COM10 = STM32 MCU1；COM12 = STM32 MCU2。
此前误将 bus_index 直接当作 CAN1~4，导致 RS485 被标成 CAN3/CAN4。
"""

from __future__ import annotations

from dataclasses import dataclass

# MCU1 (通常 COM10)
MCU1_PORTS = {
    0: "CAN1",
    1: "CAN2",
    2: "RS485-1",
    3: "RS485-2",
}

# MCU2 (通常 COM12)
MCU2_PORTS = {
    0: "CAN3",
    1: "CAN4",
    2: "RS485-3",
    3: "RS485-4",
}

MCU1_COM = "COM10"
MCU2_COM = "COM12"


@dataclass
class BusEndpoint:
    com: str
    bus_index: int
    port_name: str
    mcu: int

    @property
    def key(self) -> tuple[str, int]:
        return (self.com, self.bus_index)


def port_to_mcu_map(com: str) -> dict[int, str]:
    com_upper = com.upper()
    if com_upper == MCU2_COM.upper():
        return MCU2_PORTS
    return MCU1_PORTS


def bus_label(com: str, bus_index: int) -> str:
    mapping = port_to_mcu_map(com)
    return mapping.get(bus_index, f"unknown(0x{bus_index:02x})")


def is_can_port(com: str, bus_index: int) -> bool:
    name = bus_label(com, bus_index)
    return name.startswith("CAN")


def all_can_endpoints() -> list[BusEndpoint]:
    eps = []
    for idx, name in MCU1_PORTS.items():
        if name.startswith("CAN"):
            eps.append(BusEndpoint(MCU1_COM, idx, name, 1))
    for idx, name in MCU2_PORTS.items():
        if name.startswith("CAN"):
            eps.append(BusEndpoint(MCU2_COM, idx, name, 2))
    return eps
