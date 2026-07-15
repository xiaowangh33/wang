"""RS06 电机 MIT 模式高层控制接口。"""

from __future__ import annotations

import time
from typing import Optional

from config import AppConfig, CurrentSetpoint, MitSetpoint
from protocol import (
    PARAM_MECH_POS,
    PARAM_MECH_VEL,
    RUN_MODE_CURRENT,
    RUN_MODE_MIT,
    MotorFeedback,
    build_broadcast_run_mode_command,
    build_disable_command,
    build_enable_command,
    build_get_device_id_command,
    build_init_command,
    build_mit_command,
    build_read_param_command,
    build_set_iq_ref_command,
    build_set_run_mode_command,
    build_set_zero_command,
    build_start_command,
    decode_device_id_response,
    decode_feedback_from_can_id,
    parse_adapter_buffer,
)
from transport import UsbCanTransport


class RS06Motor:
    """灵足时代 RS06 关节电机 MIT 控制器。"""

    def __init__(self, config: AppConfig):
        self.config = config
        self.transport = UsbCanTransport(
            port=config.serial.port,
            baudrate=config.serial.baudrate,
            timeout=config.serial.timeout,
        )
        self._last_feedback: Optional[MotorFeedback] = None

    @property
    def motor_id(self) -> int:
        return self.config.can.motor_id

    @property
    def host_id(self) -> int:
        return self.config.can.host_id

    @property
    def setpoint(self) -> MitSetpoint:
        return self.config.setpoint

    @property
    def current_setpoint(self) -> CurrentSetpoint:
        return self.config.current

    @property
    def last_feedback(self) -> Optional[MotorFeedback]:
        return self._last_feedback

    def connect(self) -> None:
        self.transport.open()
        self._log(f"已连接串口 {self.config.serial.port} @ {self.config.serial.baudrate}")

    def disconnect(self) -> None:
        self.transport.close()
        self._log("串口已关闭")

    def ping(self) -> bool:
        frame = build_get_device_id_command(self.motor_id)
        self.transport.send(frame, "获取设备ID", self.config.control.verbose)
        raw = self.transport.drain_rx(300)
        if raw and self.config.control.verbose:
            self._log(f"[RX raw] 设备ID查询: {raw.hex()}")
        for can_id, data in parse_adapter_buffer(raw):
            dev = decode_device_id_response(can_id, data)
            if dev:
                mid, uid = dev
                self._log(f"检测到电机 ID={mid}, UID={uid.hex()}")
                return True
        return False

    def ensure_mit_mode(self) -> None:
        frame = build_set_run_mode_command(self.motor_id, RUN_MODE_MIT)
        self.transport.send(frame, "设置 run_mode=0 (MIT)", self.config.control.verbose)
        time.sleep(0.02)
        self._try_read_feedback("模式切换")

    def ensure_current_mode(self) -> None:
        """力控/电流模式 run_mode=3。"""
        frame = build_set_run_mode_command(self.motor_id, RUN_MODE_CURRENT)
        self.transport.send(frame, "设置 run_mode=3 (电流/力控)", self.config.control.verbose)
        time.sleep(0.02)
        self._try_read_feedback("模式切换")

    def enable(self) -> None:
        """上位机顺序：broadcast run_mode@0 -> init -> MIT -> enable(0x04+C4) -> start(0x03)。"""
        self._enable_with_run_mode(RUN_MODE_MIT, "MIT")

    def enable_current(self) -> None:
        """电流/力控模式使能：run_mode=3 -> enable -> start。"""
        self._enable_with_run_mode(RUN_MODE_CURRENT, "电流/力控")

    def _enable_with_run_mode(self, run_mode: int, label: str) -> None:
        self.transport.send(
            build_broadcast_run_mode_command(run_mode),
            f"广播 motor0 run_mode={run_mode}",
            self.config.control.verbose,
        )
        time.sleep(0.02)
        self.transport.send(build_init_command(self.motor_id), "初始化", self.config.control.verbose)
        time.sleep(0.05)
        frame = build_set_run_mode_command(self.motor_id, run_mode)
        self.transport.send(frame, f"设置 run_mode={run_mode} ({label})", self.config.control.verbose)
        time.sleep(0.02)
        self._try_read_feedback("模式切换")
        self.transport.send(build_enable_command(self.motor_id), "使能(0x04+C4)", self.config.control.verbose)
        time.sleep(0.05)
        self.transport.send(build_start_command(self.motor_id), "启动(0x03)", self.config.control.verbose)
        time.sleep(0.02)
        self._try_read_feedback("使能")

    def disable(self, clear_error: bool = False) -> None:
        frame = build_disable_command(self.motor_id, clear_error)
        label = "失能并清错" if clear_error else "失能电机"
        self.transport.send(frame, label, self.config.control.verbose)
        time.sleep(0.02)

    def set_zero(self) -> None:
        frame = build_set_zero_command(self.motor_id)
        self.transport.send(frame, "设置机械零位", self.config.control.verbose)
        time.sleep(0.02)
        self._try_read_feedback("标零")

    def send_mit(self, setpoint: Optional[MitSetpoint] = None) -> Optional[MotorFeedback]:
        sp = setpoint or self.config.setpoint
        frame = build_mit_command(self.motor_id, sp, self.config.limits)
        self.transport.send(frame, "MIT运控", self.config.control.verbose)
        return self._try_read_feedback("MIT控制")

    def send_current(self, iq: Optional[float] = None) -> Optional[MotorFeedback]:
        """力控模式：周期性写 iq_ref (Apeak)。"""
        amps = self.current_setpoint.iq if iq is None else iq
        frame = build_set_iq_ref_command(self.motor_id, amps)
        self.transport.send(
            frame,
            f"iq_ref={amps:.3f}Apeak",
            self.config.control.verbose,
        )
        return self._try_read_feedback("力控")

    def read_param_f32(self, param_index: int) -> Optional[float]:
        import struct

        frame = build_read_param_command(self.motor_id, param_index)
        self.transport.send(frame, f"读取参数 0x{param_index:04X}", self.config.control.verbose)
        parsed = self.transport.read_can_frame(wait_ms=50)
        if not parsed:
            self._log("读取参数超时")
            return None
        _, data = parsed
        if len(data) >= 8:
            value = struct.unpack("<f", data[4:8])[0]
            self._log(f"参数 0x{param_index:04X} = {value}")
            return value
        return None

    def read_state(self) -> Optional[MotorFeedback]:
        pos = self.read_param_f32(PARAM_MECH_POS)
        vel = self.read_param_f32(PARAM_MECH_VEL)
        if pos is None or vel is None:
            return self._last_feedback
        fb = MotorFeedback(position=pos, velocity=vel, torque=0.0, temperature=0.0)
        self._last_feedback = fb
        self._print_feedback(fb, prefix="[状态读取]")
        return fb

    def run_loop(self, duration_s: Optional[float] = None, loop_count: Optional[int] = None) -> None:
        """按设定频率周期性发送控制指令（MIT 或力控）。"""
        if self.config.control.control_mode == "current":
            self.run_current_loop(duration_s=duration_s, loop_count=loop_count)
            return
        self.enable()
        dt = 1.0 / self.config.control.loop_hz
        start = time.time()
        count = 0
        self._log(
            f"开始 MIT 控制循环: {self.config.control.loop_hz:.1f} Hz, "
            f"pos={self.setpoint.position}, vel={self.setpoint.velocity}, "
            f"kp={self.setpoint.kp}, kd={self.setpoint.kd}, tau={self.setpoint.torque}"
        )
        try:
            while True:
                loop_start = time.time()
                self.send_mit()
                count += 1
                if loop_count is not None and count >= loop_count:
                    break
                if duration_s is not None and (time.time() - start) >= duration_s:
                    break
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        finally:
            self.disable()
            self._log(f"控制循环结束，共发送 {count} 帧")

    def run_current_loop(self, duration_s: Optional[float] = None, loop_count: Optional[int] = None) -> None:
        """电流/力控模式循环：重复写 iq_ref。"""
        self.enable_current()
        dt = 1.0 / self.config.control.loop_hz
        start = time.time()
        count = 0
        iq = self.current_setpoint.iq
        self._log(
            f"开始力控循环: {self.config.control.loop_hz:.1f} Hz, "
            f"iq_ref={iq:.3f} Apeak"
        )
        try:
            while True:
                loop_start = time.time()
                self.send_current(iq)
                count += 1
                if loop_count is not None and count >= loop_count:
                    break
                if duration_s is not None and (time.time() - start) >= duration_s:
                    break
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        finally:
            self.send_current(0.0)
            time.sleep(0.05)
            self.disable()
            self._log(f"力控循环结束，共发送 {count} 帧")

    def update_setpoint(self, **kwargs) -> MitSetpoint:
        for key, value in kwargs.items():
            if hasattr(self.setpoint, key):
                setattr(self.setpoint, key, float(value))
            else:
                raise ValueError(f"未知 setpoint 参数: {key}")
        self._log(
            "更新 setpoint: "
            f"pos={self.setpoint.position:.4f}, vel={self.setpoint.velocity:.4f}, "
            f"kp={self.setpoint.kp:.2f}, kd={self.setpoint.kd:.2f}, tau={self.setpoint.torque:.4f}"
        )
        return self.setpoint

    def print_config(self) -> None:
        c = self.config
        print("=" * 60)
        print("RS06 MIT 控制配置")
        print("=" * 60)
        print(f"串口: {c.serial.port} @ {c.serial.baudrate}")
        print(f"电机ID: {c.can.motor_id}, 主机ID: 0x{c.can.host_id:02X}")
        print(
            f"限幅: p_max={c.limits.p_max}, v_max={c.limits.v_max}, "
            f"t_max={c.limits.t_max}, kp_max={c.limits.kp_max}, kd_max={c.limits.kd_max}"
        )
        print(
            f"Setpoint: pos={c.setpoint.position}, vel={c.setpoint.velocity}, "
            f"kp={c.setpoint.kp}, kd={c.setpoint.kd}, tau={c.setpoint.torque}"
        )
        print(
            f"力控 iq_ref: {c.current.iq} Apeak, "
            f"控制模式: {c.control.control_mode}"
        )
        print(f"控制频率: {c.control.loop_hz} Hz")
        print("=" * 60)

    def _try_read_feedback(self, context: str) -> Optional[MotorFeedback]:
        raw = self.transport.drain_rx(300)
        if raw and self.config.control.verbose:
            self._log(f"[RX raw] {context}: {raw.hex()}")
        parsed_list = parse_adapter_buffer(raw)
        if not parsed_list:
            return None
        for can_id, data in parsed_list:
            dev = decode_device_id_response(can_id, data)
            if dev and self.config.control.verbose:
                mid, uid = dev
                self._log(f"[RX] {context}: 设备ID motor={mid} uid={uid.hex()}")
        can_id, data = parsed_list[-1]
        feedback = decode_feedback_from_can_id(can_id, data, self.config.limits)
        if feedback is None:
            if self.config.control.verbose:
                self._log(f"[RX] {context}: can_id=0x{can_id:08X}, data={data.hex()}")
            return None
        self._last_feedback = feedback
        if self.config.control.print_feedback:
            self._print_feedback(feedback, prefix=f"[RX] {context}")
        return feedback

    def _print_feedback(self, feedback: MotorFeedback, prefix: str = "[反馈]") -> None:
        print(
            f"{prefix} pos={feedback.position:+.4f} rad, "
            f"vel={feedback.velocity:+.4f} rad/s, "
            f"torque={feedback.torque:+.4f} N·m, "
            f"temp={feedback.temperature:.1f} °C"
        )

    def _log(self, message: str) -> None:
        if self.config.control.verbose:
            print(message)
