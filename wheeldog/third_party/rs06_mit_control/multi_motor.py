"""多电机共享 CAN 总线控制（单 USB-CAN 串口）。"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from config import AppConfig, ControlConfig, CurrentSetpoint, MitSetpoint, MotorLimits
from protocol import (
    RUN_MODE_CURRENT,
    RUN_MODE_MIT,
    MotorFeedback,
    build_broadcast_run_mode_command,
    build_disable_command,
    build_enable_command,
    build_get_device_id_command,
    build_init_command,
    build_mit_command,
    build_set_iq_ref_command,
    build_set_run_mode_command,
    build_start_command,
    decode_device_id_response,
    decode_feedback_from_can_id,
    motor_id_from_can_id,
    parse_adapter_buffer,
)
from transport import UsbCanTransport


def parse_ids(text: str) -> list[int]:
    return [int(x.strip()) for x in text.split(",") if x.strip()]


@dataclass
class MotorChannel:
    motor_id: int
    setpoint: MitSetpoint = field(default_factory=MitSetpoint)
    current: CurrentSetpoint = field(default_factory=CurrentSetpoint)
    last_feedback: Optional[MotorFeedback] = None


class RS06Bus:
    """同一 CAN 总线上多电机控制器，共享一个串口。"""

    def __init__(
        self,
        port: str,
        motor_ids: list[int],
        baudrate: int = 921600,
        limits: Optional[MotorLimits] = None,
        control: Optional[ControlConfig] = None,
    ):
        if not motor_ids:
            raise ValueError("motor_ids 不能为空")
        self.transport = UsbCanTransport(port, baudrate)
        self.limits = limits or MotorLimits()
        self.control = control or ControlConfig()
        self.channels = {
            mid: MotorChannel(motor_id=mid) for mid in motor_ids
        }

    @property
    def motor_ids(self) -> list[int]:
        return list(self.channels.keys())

    def connect(self) -> None:
        self.transport.open()
        self._log(f"已连接 {self.transport.port}，电机 ID: {self.motor_ids}")

    def disconnect(self) -> None:
        self.transport.close()
        self._log("串口已关闭")

    def ping_all(self) -> dict[int, bool]:
        results = {}
        for mid in self.motor_ids:
            self.transport.send(
                build_get_device_id_command(mid),
                f"M{mid} 获取设备ID",
                self.control.verbose,
            )
            time.sleep(0.05)
        raw = self.transport.drain_rx(400)
        for can_id, data in parse_adapter_buffer(raw):
            dev = decode_device_id_response(can_id, data)
            if dev:
                found_id, uid = dev
                results[found_id] = True
                self._log(f"检测到电机 ID={found_id}, UID={uid.hex()}")
        for mid in self.motor_ids:
            results.setdefault(mid, False)
        return results

    def enable_all(self, run_mode: int = RUN_MODE_MIT) -> None:
        label = "MIT" if run_mode == RUN_MODE_MIT else "电流/力控"
        self.transport.send(
            build_broadcast_run_mode_command(run_mode),
            f"广播 motor0 run_mode={run_mode}",
            self.control.verbose,
        )
        time.sleep(0.02)
        for mid in self.motor_ids:
            self._enable_one(mid, run_mode, label)
        self._collect_feedback("使能完成")

    def _enable_one(self, motor_id: int, run_mode: int, label: str) -> None:
        prefix = f"M{motor_id}"
        self.transport.send(build_init_command(motor_id), f"{prefix} 初始化", self.control.verbose)
        time.sleep(0.03)
        self.transport.send(
            build_set_run_mode_command(motor_id, run_mode),
            f"{prefix} run_mode={run_mode} ({label})",
            self.control.verbose,
        )
        time.sleep(0.02)
        self.transport.send(build_enable_command(motor_id), f"{prefix} 使能", self.control.verbose)
        time.sleep(0.03)
        self.transport.send(build_start_command(motor_id), f"{prefix} 启动", self.control.verbose)
        time.sleep(0.02)

    def disable_all(self, clear_error: bool = False) -> None:
        for mid in self.motor_ids:
            self.transport.send(
                build_disable_command(mid, clear_error),
                f"M{mid} 失能",
                self.control.verbose,
            )
            time.sleep(0.02)

    def send_mit_all(self) -> dict[int, Optional[MotorFeedback]]:
        for mid, ch in self.channels.items():
            self.transport.send(
                build_mit_command(mid, ch.setpoint, self.limits),
                f"M{mid} MIT",
                self.control.verbose,
            )
            time.sleep(0.002)
        return self._collect_feedback("MIT")

    def send_current_all(self) -> dict[int, Optional[MotorFeedback]]:
        for mid, ch in self.channels.items():
            iq = ch.current.iq
            self.transport.send(
                build_set_iq_ref_command(mid, iq),
                f"M{mid} iq={iq:.3f}A",
                self.control.verbose,
            )
            time.sleep(0.002)
        return self._collect_feedback("力控")

    def run_current_loop(
        self,
        duration_s: float = 3.0,
        loop_count: Optional[int] = None,
    ) -> None:
        self.enable_all(run_mode=RUN_MODE_CURRENT)
        dt = 1.0 / self.control.loop_hz
        start = time.time()
        count = 0
        iq_info = ", ".join(f"M{mid}={ch.current.iq:.2f}A" for mid, ch in self.channels.items())
        self._log(f"开始双电机力控: {self.control.loop_hz:.0f}Hz, {iq_info}")
        try:
            while True:
                t0 = time.time()
                self.send_current_all()
                count += 1
                if loop_count is not None and count >= loop_count:
                    break
                if time.time() - start >= duration_s:
                    break
                elapsed = time.time() - t0
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        finally:
            for mid in self.motor_ids:
                self.transport.send(
                    build_set_iq_ref_command(mid, 0.0),
                    f"M{mid} iq=0",
                    self.control.verbose,
                )
            time.sleep(0.05)
            self.disable_all()
            self._log(f"力控结束，共 {count} 轮")

    def _collect_feedback(self, context: str) -> dict[int, Optional[MotorFeedback]]:
        raw = self.transport.drain_rx(350)
        if raw and self.control.verbose:
            self._log(f"[RX raw] {context}: {raw.hex()}")
        result = {mid: None for mid in self.motor_ids}
        for can_id, data in parse_adapter_buffer(raw):
            mid = motor_id_from_can_id(can_id)
            if mid not in self.channels:
                continue
            fb = decode_feedback_from_can_id(can_id, data, self.limits)
            if fb:
                self.channels[mid].last_feedback = fb
                result[mid] = fb
                if self.control.print_feedback:
                    self._print_feedback(mid, fb, context)
        return result

    def _print_feedback(self, motor_id: int, fb: MotorFeedback, context: str) -> None:
        print(
            f"[RX M{motor_id}] {context} "
            f"pos={fb.position:+.4f} rad, vel={fb.velocity:+.4f} rad/s, "
            f"torque={fb.torque:+.4f} N·m, temp={fb.temperature:.1f} °C"
        )

    def _log(self, msg: str) -> None:
        if self.control.verbose:
            print(msg)


def bus_from_config(config: AppConfig, motor_ids: list[int]) -> RS06Bus:
    return RS06Bus(
        port=config.serial.port,
        motor_ids=motor_ids,
        baudrate=config.serial.baudrate,
        limits=config.limits,
        control=config.control,
    )
