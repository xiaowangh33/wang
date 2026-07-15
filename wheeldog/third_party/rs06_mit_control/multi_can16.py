"""4 路 CAN、16 电机：探测、改 ID、MIT 控制。"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from motors16_config import (
    DEFAULT_FACTORY_ID,
    MOTORS_PER_CHANNEL,
    MotorSlot,
    Robot16Config,
)
from protocol import (
    RUN_MODE_MIT,
    MotorFeedback,
    build_broadcast_run_mode_command,
    build_disable_command,
    build_enable_command,
    build_get_device_id_command,
    build_init_command,
    build_mit_command,
    build_set_run_mode_command,
    build_start_command,
    decode_device_id_response,
    decode_feedback_from_can_id,
    motor_id_from_can_id,
    parse_adapter_buffer,
    parse_adapter_buffer_with_channel,
)
from sanpo_transport import SanpoCanTransport, TransportMode


@dataclass
class DiscoveredMotor:
    can_channel: int
    motor_id: int
    uuid: bytes


@dataclass
class MotorState:
    slot: MotorSlot
    last_feedback: Optional[MotorFeedback] = None
    online: bool = False


@dataclass
class PingResult:
    slot: MotorSlot
    online: bool
    responder_id: Optional[int] = None
    uuid: Optional[bytes] = None


class Robot16Controller:
    def __init__(self, config: Robot16Config):
        self.config = config
        mode: TransportMode = (
            "sanpo_prefix"
            if config.transport_mode == "sanpo_prefix"
            else "sanpo"
            if config.transport_mode == "sanpo"
            else "lingzu"
        )
        self.transport = SanpoCanTransport(
            config.serial.port,
            config.serial.baudrate,
            mode=mode,
        )
        self.states = [MotorState(slot=m) for m in config.motors]

    def connect(self) -> None:
        self.transport.open()
        print(
            f"已连接 {self.config.serial.port} @ {self.config.serial.baudrate} "
            f"mode={self.config.transport_mode}"
        )

    def disconnect(self) -> None:
        self.transport.close()

    def _send_channel(self, channel: int, frame: bytes, desc: str, verbose: bool) -> None:
        # SANPO 实测：AT 发送不需通道前缀，网关负责路由；接收 ET 帧带通道号
        self.transport.send_broadcast(frame, desc, verbose)

    def _collect_device_ids(
        self,
        raw: bytes,
        channel: Optional[int] = None,
    ) -> list[DiscoveredMotor]:
        found: list[DiscoveredMotor] = []
        for rx_ch, can_id, data in parse_adapter_buffer_with_channel(raw):
            dev = decode_device_id_response(can_id, data)
            if not dev:
                continue
            mid, uid = dev
            ch = rx_ch if rx_ch >= 0 else (channel if channel is not None else -1)
            if channel is not None and ch >= 0 and ch != channel:
                continue
            found.append(DiscoveredMotor(ch, mid, uid))
        return found

    def discover_on_channel(
        self,
        channel: int,
        probe_ids: Optional[list[int]] = None,
        rounds: int = 25,
    ) -> dict[bytes, DiscoveredMotor]:
        """多轮 ping 收集同总线上不同 UUID（用于同 ID 批量改号）。"""
        probe_ids = probe_ids or [DEFAULT_FACTORY_ID, 1, 2, 3, 4, 127]
        seen: dict[bytes, DiscoveredMotor] = {}
        for probe in probe_ids:
            for _ in range(rounds):
                self._send_channel(
                    channel,
                    build_get_device_id_command(probe),
                    f"CAN{channel + 1} scan id={probe}",
                    False,
                )
                time.sleep(0.06)
                raw = self.transport.drain_rx(180)
                for item in self._collect_device_ids(raw, channel):
                    seen[item.uuid] = item
                if len(seen) >= MOTORS_PER_CHANNEL:
                    break
            if len(seen) >= MOTORS_PER_CHANNEL:
                break
        return seen

    def ping_slot(self, slot: MotorSlot, probe_id: Optional[int] = None) -> PingResult:
        mid = probe_id if probe_id is not None else slot.motor_id
        self._send_channel(
            slot.can_channel,
            build_get_device_id_command(mid),
            f"[{slot.name}] ping id={mid}",
            False,
        )
        time.sleep(0.08)
        raw = self.transport.drain_rx(200)
        for item in self._collect_device_ids(raw, slot.can_channel):
            if item.motor_id == mid or mid == slot.motor_id:
                return PingResult(slot, True, item.motor_id, item.uuid)
        return PingResult(slot, False)

    def ping_all(self, verbose: bool = True) -> list[PingResult]:
        results: list[PingResult] = []
        for st in self.states:
            r = self.ping_slot(st.slot)
            st.online = r.online
            results.append(r)
            if verbose:
                ch_info = ""
                # ping 时从 ET 回传可知真实 CAN 通道
                status = "在线" if r.online else "无响应"
                uid = r.uuid.hex() if r.uuid else "-"
                print(
                    f"  [{st.slot.name}] 期望CAN{st.slot.can_channel + 1} "
                    f"id={st.slot.motor_id} ({st.slot.model}): {status}  uid={uid}{ch_info}"
                )
            time.sleep(0.01)
        online = sum(1 for r in results if r.online)
        print(f"探测完成: {online}/{len(results)} 在线")
        return results

    def ping_summary_by_channel(self) -> None:
        print("\n按 CAN 通道汇总:")
        for ch in range(4):
            slots = [s for s in self.states if s.slot.can_channel == ch]
            online = sum(1 for s in slots if s.online)
            print(f"  CAN{ch + 1}: {online}/{len(slots)}")

    def set_motor_id(
        self,
        channel: int,
        old_id: int,
        new_id: int,
        uuid: bytes,
        host_id: Optional[int] = None,
    ) -> bool:
        raise RuntimeError(
            "项目内改电机ID功能已禁用：实机证明同一旧ID的多台电机会同时改号，"
            "数据区UUID不能用于单台筛选。请使用电机开发者的原厂上位机配置。"
        )

    def auto_config_channel(
        self,
        channel: int,
        target_ids: Optional[list[int]] = None,
        factory_id: int = DEFAULT_FACTORY_ID,
    ) -> bool:
        del channel, target_ids, factory_id
        raise RuntimeError(
            "项目内批量改ID功能已禁用；请使用电机开发者的原厂上位机。"
        )

    def auto_config_all(self, factory_id: int = DEFAULT_FACTORY_ID) -> bool:
        ok_all = True
        for ch in range(4):
            if not self.auto_config_channel(ch, factory_id=factory_id):
                ok_all = False
        return ok_all

    def _enable_one(self, slot: MotorSlot) -> None:
        mid = slot.motor_id
        ch = slot.can_channel
        self._send_channel(ch, build_init_command(mid), f"[{slot.name}] init", False)
        time.sleep(0.03)
        self._send_channel(
            ch, build_set_run_mode_command(mid, RUN_MODE_MIT), f"[{slot.name}] MIT", False
        )
        time.sleep(0.02)
        self._send_channel(ch, build_enable_command(mid), f"[{slot.name}] enable", False)
        time.sleep(0.03)
        self._send_channel(ch, build_start_command(mid), f"[{slot.name}] start", False)
        time.sleep(0.02)

    def enable_all(self) -> None:
        for ch in range(4):
            self._send_channel(
                ch,
                build_broadcast_run_mode_command(RUN_MODE_MIT),
                f"CAN{ch + 1} broadcast run_mode=0",
                False,
            )
            time.sleep(0.02)
        for st in self.states:
            self._enable_one(st.slot)
        self._collect_feedback("使能")

    def disable_all(self) -> None:
        for st in self.states:
            self._send_channel(
                st.slot.can_channel,
                build_disable_command(st.slot.motor_id),
                f"[{st.slot.name}] disable",
                False,
            )
            time.sleep(0.003)

    def send_mit_all(self) -> None:
        by_ch: dict[int, list[MotorState]] = {i: [] for i in range(4)}
        for st in self.states:
            by_ch[st.slot.can_channel].append(st)
        for ch in range(4):
            for st in by_ch[ch]:
                slot = st.slot
                frame = build_mit_command(slot.motor_id, slot.setpoint, slot.limits)
                self._send_channel(ch, frame, f"[{slot.name}] MIT", False)
                time.sleep(0.0003)
        self._collect_feedback("MIT")

    def _collect_feedback(self, context: str) -> None:
        raw = self.transport.drain_rx(400)
        for can_id, data in parse_adapter_buffer(raw):
            mid = motor_id_from_can_id(can_id)
            for st in self.states:
                if st.slot.motor_id != mid:
                    continue
                fb = decode_feedback_from_can_id(can_id, data, st.slot.limits)
                if fb:
                    st.last_feedback = fb
                    print(
                        f"[RX {st.slot.name}] {context} "
                        f"pos={fb.position:+.3f} vel={fb.velocity:+.3f} "
                        f"torque={fb.torque:+.3f} T={fb.temperature:.1f}"
                    )

    def run_loop(self, duration_s: float = 5.0) -> None:
        self.enable_all()
        dt = 1.0 / self.config.loop_hz
        t0 = time.time()
        n = 0
        try:
            while time.time() - t0 < duration_s:
                t_loop = time.time()
                self.send_mit_all()
                n += 1
                elapsed = time.time() - t_loop
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        finally:
            self.disable_all()
            print(f"控制结束，共 {n} 轮")
