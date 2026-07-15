"""16 电机统一控制/反馈接口（SANPO ET + 400Hz 目标频率）。

用法::

    from robot16_interface import Robot16Interface, MotorCommand

    robot = Robot16Interface(hz=400.0)
    robot.open()
    robot.enable_all_verified()   # 推荐：使能+验证+重试

    robot.set_torque(0, 0.6)      # RS06，单位 N·m（0.2 过小可能不转）
    robot.set_velocity(3, 0.5)    # RS01，单位 rad/s

    robot.start()
    fb = robot.get_feedback(0)
    robot.stop()
    robot.disable_all()
    robot.close()
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from motor_constants import SPECS, torque_to_iq
from motors16_config import MotorSlot, Robot16Config, default_robot16
from protocol import (
    RUN_MODE_CURRENT,
    RUN_MODE_SPEED,
    MotorFeedback,
    build_disable_command,
    build_enable_command,
    build_init_command,
    build_set_current_limit_command,
    build_set_iq_ref_command,
    build_set_run_mode_command,
    build_set_spd_ref_command,
    build_start_command,
    decode_feedback_from_can_id,
    motor_id_from_can_id,
)
from sanpo_bus_map import MCU1_COM, MCU2_COM
from sanpo_et import SanpoEtBus

MOTOR_COUNT = 16

# 使能验证默认参数
DEFAULT_VERIFY_TORQUE_NM = 0.6   # RS06 运动验证力矩（0.2 N·m 通常不足以克服静摩擦）
DEFAULT_VERIFY_VEL_RAD_S = 0.3   # RS01 运动验证速度
DEFAULT_COMMAND_TORQUE_LIMIT_NM = 1.0
DEFAULT_COMMAND_SPEED_LIMIT_RAD_S = 4.0
DEFAULT_WHEEL_TORQUE_LIMIT_NM = 1.0
MOTION_VEL_THRESHOLD = 0.025   # rad/s，判定「在转」
ENABLE_STEP_DELAY_S = 0.045
CAN_BATCH_GAP_S = 0.12
DEFAULT_MAX_ENABLE_RETRIES = 3
FEEDBACK_POLL_MS = 90.0
MOTION_VERIFY_MS = 160.0
FEEDBACK_STALE_S = 0.2           # 运行期反馈超时


@dataclass
class MotorCommand:
    """单电机控制量（接口层单位）。"""

    torque_nm: float = 0.0       # RS06 力矩模式：N·m
    velocity_rad_s: float = 0.0  # RS01 速度模式：rad/s


@dataclass
class MotorFeedbackState:
    """单电机反馈（与协议解码一致）。"""

    index: int
    name: str
    model: str
    position: float = 0.0       # rad
    velocity: float = 0.0       # rad/s
    torque: float = 0.0         # N·m
    temperature: float = 0.0      # °C
    online: bool = False
    enabled: bool = False         # 使能验证通过
    fresh: bool = False           # 本周期是否有新反馈
    fault_bits: int = 0
    timestamp: float = 0.0


def _motor_index(can_index: int, motor_id: int) -> int:
    return can_index * 4 + (motor_id - 1)


def _read_limit_env(name: str, default: float, upper: float) -> float:
    text = os.getenv(name)
    if not text:
        return default
    try:
        value = float(text)
    except ValueError:
        print(f"[Robot16Interface] 忽略无效 {name}={text!r}，使用 {default}")
        return default
    if not math.isfinite(value) or value < 0.0 or value > upper:
        print(f"[Robot16Interface] 忽略越界 {name}={text!r}，使用 {default}")
        return default
    return value


def _clip_abs(value: float, limit: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


@dataclass
class _MotorRuntime:
    slot: MotorSlot
    index: int
    command: MotorCommand = field(default_factory=MotorCommand)
    feedback: MotorFeedbackState = field(default_factory=lambda: MotorFeedbackState(0, ""))
    enabled: bool = False
    recover_fail_count: int = 0


class Robot16Interface:
    """16 电机控制/反馈 API。"""

    def __init__(
        self,
        config: Optional[Robot16Config] = None,
        hz: float = 400.0,
        baudrate: int = 921600,
        mcu1_com: str = MCU1_COM,
        mcu2_com: str = MCU2_COM,
    ):
        self.config = config or default_robot16()
        self.hz = hz
        self.dt = 1.0 / hz
        self._bus = SanpoEtBus(baudrate=baudrate, mcu1_com=mcu1_com, mcu2_com=mcu2_com)
        self.command_torque_limit_nm = _read_limit_env(
            "WHEELDOG_MOTOR_MAX_TORQUE_NM",
            DEFAULT_COMMAND_TORQUE_LIMIT_NM,
            36.0,
        )
        self.command_speed_limit_rad_s = _read_limit_env(
            "WHEELDOG_MOTOR_MAX_SPEED_RAD_S",
            DEFAULT_COMMAND_SPEED_LIMIT_RAD_S,
            50.0,
        )
        self.wheel_torque_limit_nm = _read_limit_env(
            "WHEELDOG_WHEEL_MAX_TORQUE_NM",
            DEFAULT_WHEEL_TORQUE_LIMIT_NM,
            SPECS["rs01"].torque_max_nm,
        )
        print(
            "[Robot16Interface] command safety: "
            f"|tau|<={self.command_torque_limit_nm:.3f} N.m, "
            f"|velocity|<={self.command_speed_limit_rad_s:.3f} rad/s, "
            f"RS01 wheel torque<={self.wheel_torque_limit_nm:.3f} N.m"
        )
        self._motors: list[_MotorRuntime] = []
        for i, slot in enumerate(self.config.motors):
            fb = MotorFeedbackState(index=i, name=slot.name, model=slot.model)
            self._motors.append(_MotorRuntime(slot=slot, index=i, feedback=fb))
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cycle_count = 0
        self._on_cycle: Optional[Callable[[list[MotorFeedbackState]], None]] = None
        self._auto_recover = False
        self._max_recover_attempts = 2

    @property
    def motor_count(self) -> int:
        return MOTOR_COUNT

    def open(self) -> None:
        self._bus.open()

    def close(self) -> None:
        self.stop()
        self._bus.close()

    # --- 控制量下发（仅写入缓存，由 step/start 按 hz 发出） ---

    def set_torque(self, index: int, torque_nm: float) -> None:
        """RS06 力矩给定，单位 N·m。"""
        rt = self._motors[index]
        if rt.slot.model != "rs06":
            raise ValueError(f"电机 {index} ({rt.slot.name}) 非 RS06，请用 set_velocity")
        with self._lock:
            rt.command.torque_nm = _clip_abs(torque_nm, self.command_torque_limit_nm)

    def set_velocity(self, index: int, velocity_rad_s: float) -> None:
        """RS01 速度给定，单位 rad/s。"""
        rt = self._motors[index]
        if rt.slot.run_mode != RUN_MODE_SPEED:
            raise ValueError(f"电机 {index} ({rt.slot.name}) 非速度模式")
        with self._lock:
            rt.command.velocity_rad_s = _clip_abs(
                velocity_rad_s,
                self.command_speed_limit_rad_s,
            )

    def set_command(self, index: int, cmd: MotorCommand) -> None:
        rt = self._motors[index]
        if rt.slot.model == "rs06":
            self.set_torque(index, cmd.torque_nm)
        else:
            self.set_velocity(index, cmd.velocity_rad_s)

    def set_commands(self, commands: list[MotorCommand]) -> None:
        if len(commands) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个命令，收到 {len(commands)}")
        for i, cmd in enumerate(commands):
            self.set_command(i, cmd)

    def set_all_torques(self, torques: list[float]) -> None:
        """按顺序设置 12 个 RS06 力矩；也可传 16 个（RS01 位忽略力矩）。"""
        for i, rt in enumerate(self._motors):
            if rt.slot.model == "rs06" and i < len(torques):
                self.set_torque(i, torques[i])

    # --- 反馈读取 ---

    def get_feedback(self, index: int) -> MotorFeedbackState:
        with self._lock:
            rt = self._motors[index]
            return MotorFeedbackState(
                index=rt.feedback.index,
                name=rt.feedback.name,
                model=rt.feedback.model,
                position=rt.feedback.position,
                velocity=rt.feedback.velocity,
                torque=rt.feedback.torque,
                temperature=rt.feedback.temperature,
                online=rt.feedback.online,
                enabled=rt.enabled,
                fresh=rt.feedback.fresh,
                fault_bits=rt.feedback.fault_bits,
                timestamp=rt.feedback.timestamp,
            )

    def get_all_feedback(self) -> list[MotorFeedbackState]:
        return [self.get_feedback(i) for i in range(MOTOR_COUNT)]

    # --- 使能 / 单步 / 后台循环 ---

    def enable_all(self) -> None:
        """简单使能（无验证）。推荐改用 enable_all_verified()。"""
        for rt in self._motors:
            self._enable_one(rt.slot)
            rt.enabled = True
            rt.feedback.enabled = True
            time.sleep(0.005)

    def enable_all_verified(
        self,
        max_retries: int = DEFAULT_MAX_ENABLE_RETRIES,
        batch_by_can: bool = True,
        motion_verify: bool = False,
        verify_torque_nm: float = DEFAULT_VERIFY_TORQUE_NM,
        verify_vel_rad_s: float = DEFAULT_VERIFY_VEL_RAD_S,
        verbose: bool = True,
    ) -> list[bool]:
        """分批使能 + 反馈验证 + 重试（默认不做运动验证，避免使能时短暂转动）。"""
        results: list[bool] = []
        if batch_by_can:
            for can_idx in range(4):
                indices = [i for i, rt in enumerate(self._motors) if rt.slot.can_channel == can_idx]
                if verbose:
                    print(f"使能 CAN{can_idx + 1} ({len(indices)} 台)...")
                for idx in indices:
                    ok = self.enable_one_verified(
                        idx,
                        max_retries=max_retries,
                        motion_verify=motion_verify,
                        verify_torque_nm=verify_torque_nm,
                        verify_vel_rad_s=verify_vel_rad_s,
                        verbose=verbose,
                    )
                    results.append(ok)
                time.sleep(CAN_BATCH_GAP_S)
        else:
            for idx in range(MOTOR_COUNT):
                ok = self.enable_one_verified(
                    idx,
                    max_retries=max_retries,
                    motion_verify=motion_verify,
                    verify_torque_nm=verify_torque_nm,
                    verify_vel_rad_s=verify_vel_rad_s,
                    verbose=verbose,
                )
                results.append(ok)
        ok_count = sum(results)
        if verbose:
            print(f"使能完成: {ok_count}/{MOTOR_COUNT} 台验证通过")
        return results

    def enable_one_verified(
        self,
        index: int,
        max_retries: int = DEFAULT_MAX_ENABLE_RETRIES,
        motion_verify: bool = False,
        verify_torque_nm: float = DEFAULT_VERIFY_TORQUE_NM,
        verify_vel_rad_s: float = DEFAULT_VERIFY_VEL_RAD_S,
        verbose: bool = True,
    ) -> bool:
        rt = self._motors[index]
        slot = rt.slot
        for attempt in range(1, max_retries + 1):
            self._disable_one(slot, clear_error=True)
            time.sleep(0.05)
            self._enable_one(slot)
            time.sleep(ENABLE_STEP_DELAY_S)

            if not self._verify_feedback(index):
                if verbose:
                    print(f"  [{slot.name}] 尝试 {attempt}/{max_retries}: 无反馈或 fault")
                continue

            if motion_verify and not self._verify_motion(
                index, verify_torque_nm, verify_vel_rad_s
            ):
                if verbose:
                    print(f"  [{slot.name}] 尝试 {attempt}/{max_retries}: 运动验证失败")
                continue

            rt.enabled = True
            rt.recover_fail_count = 0
            rt.feedback.enabled = True
            if verbose:
                tag = "反馈+运动" if motion_verify else "反馈"
                print(f"  [{slot.name}] 使能成功 ({tag})")
            return True

        rt.enabled = False
        rt.feedback.enabled = False
        if verbose:
            print(f"  [{slot.name}] 使能失败（已重试 {max_retries} 次）")
        return False

    def recover_motor(self, index: int, verbose: bool = True) -> bool:
        """单台清错并重新使能验证。"""
        if verbose:
            print(f"恢复电机 index={index} ({self._motors[index].slot.name})...")
        return self.enable_one_verified(index, motion_verify=False, verbose=verbose)

    def set_auto_recover(self, enabled: bool = True, max_attempts: int = 2) -> None:
        """运行中自动对失能/超时电机尝试单台恢复。"""
        self._auto_recover = enabled
        self._max_recover_attempts = max_attempts

    def disable_all(self, clear_error: bool = False) -> None:
        for rt in self._motors:
            self._disable_one(rt.slot, clear_error=clear_error)
            rt.enabled = False
            rt.feedback.enabled = False
        self._bus.flush_tx()
        time.sleep(0.05)

    def _disable_one(self, slot: MotorSlot, clear_error: bool = False) -> None:
        ch = self._bus.tx_channel_for_can_index(slot.can_channel)
        self._bus.write_lingzu(ch, build_disable_command(slot.motor_id, clear_error))

    def _enable_one(self, slot: MotorSlot) -> None:
        """与 motor.py 对齐：逐步使能并留足间隔。"""
        ch = self._bus.tx_channel_for_can_index(slot.can_channel)
        mid = slot.motor_id
        # 同总线 mixed mode 时不 broadcast motor0，仅逐台设 run_mode
        self._bus.write_lingzu(ch, build_init_command(mid))
        self._bus.flush_tx()
        time.sleep(0.05)
        self._bus.write_lingzu(ch, build_set_run_mode_command(mid, slot.run_mode))
        self._bus.flush_tx()
        time.sleep(ENABLE_STEP_DELAY_S)
        self._bus.write_lingzu(ch, build_set_current_limit_command(
            mid,
            self._slot_current_limit_a(slot),
        ))
        self._bus.flush_tx()
        time.sleep(ENABLE_STEP_DELAY_S)
        self._bus.write_lingzu(ch, build_enable_command(mid))
        self._bus.flush_tx()
        time.sleep(ENABLE_STEP_DELAY_S)
        self._bus.write_lingzu(ch, build_start_command(mid))
        self._bus.flush_tx()
        time.sleep(ENABLE_STEP_DELAY_S)

    def _slot_current_limit_a(self, slot: MotorSlot) -> float:
        spec = SPECS[slot.model]
        if slot.model == "rs01":
            torque_limit_nm = min(self.wheel_torque_limit_nm, spec.torque_max_nm)
        else:
            torque_limit_nm = min(self.command_torque_limit_nm, spec.torque_max_nm)
        return abs(torque_to_iq(torque_limit_nm, slot.model))

    def _drain_and_apply(self, wait_ms: float = FEEDBACK_POLL_MS) -> None:
        for com, rx_ch, can_id, data in self._bus.drain_all(wait_ms=wait_ms):
            can_idx = self._bus.can_index_from_rx(com, rx_ch)
            if can_idx is None:
                continue
            mid = motor_id_from_can_id(can_id)
            idx = _motor_index(can_idx, mid)
            if 0 <= idx < MOTOR_COUNT:
                self._try_apply_feedback(idx, can_id, data)

    def _try_apply_feedback(self, index: int, can_id: int, data: bytes) -> bool:
        rt = self._motors[index]
        fb = decode_feedback_from_can_id(can_id, data, rt.slot.limits)
        if fb is None:
            return False
        self._apply_feedback(index, fb)
        return True

    def _verify_feedback(self, index: int) -> bool:
        """使能后发零指令，确认有 type0x02 回传且无 fault。"""
        rt = self._motors[index]
        slot = rt.slot
        ch = self._bus.tx_channel_for_can_index(slot.can_channel)
        mid = slot.motor_id
        if slot.run_mode == RUN_MODE_CURRENT:
            self._bus.write_lingzu(ch, build_set_iq_ref_command(mid, 0.0))
        else:
            self._bus.write_lingzu(ch, build_set_spd_ref_command(mid, 0.0))
        self._bus.flush_tx()
        self._drain_and_apply(wait_ms=FEEDBACK_POLL_MS)
        fb = self._motors[index].feedback
        return fb.online and fb.fault_bits == 0 and (time.time() - fb.timestamp) < 0.5

    def _verify_motion(
        self,
        index: int,
        verify_torque_nm: float,
        verify_vel_rad_s: float,
    ) -> bool:
        """下发验证级控制量，检查速度或位置变化。"""
        rt = self._motors[index]
        slot = rt.slot
        ch = self._bus.tx_channel_for_can_index(slot.can_channel)
        mid = slot.motor_id
        pos0 = rt.feedback.position

        if slot.run_mode == RUN_MODE_CURRENT:
            iq = torque_to_iq(verify_torque_nm, slot.model)
            self._bus.write_lingzu(ch, build_set_iq_ref_command(mid, iq))
        else:
            self._bus.write_lingzu(ch, build_set_spd_ref_command(mid, verify_vel_rad_s))
        self._bus.flush_tx()

        max_vel = 0.0
        deadline = time.time() + MOTION_VERIFY_MS / 1000.0
        while time.time() < deadline:
            self._drain_and_apply(wait_ms=20.0)
            vel = abs(self._motors[index].feedback.velocity)
            max_vel = max(max_vel, vel)
            if max_vel >= MOTION_VEL_THRESHOLD:
                break
            time.sleep(0.02)

        # 恢复零指令
        if slot.run_mode == RUN_MODE_CURRENT:
            self._bus.write_lingzu(ch, build_set_iq_ref_command(mid, 0.0))
        else:
            self._bus.write_lingzu(ch, build_set_spd_ref_command(mid, 0.0))
        self._bus.flush_tx()
        self._drain_and_apply(wait_ms=30.0)

        pos_delta = abs(self._motors[index].feedback.position - pos0)
        return max_vel >= MOTION_VEL_THRESHOLD or pos_delta > 0.02

    def _check_health(self) -> list[int]:
        """返回需要恢复的电机 index 列表。"""
        stale: list[int] = []
        now = time.time()
        for rt in self._motors:
            if not rt.enabled:
                continue
            age = now - rt.feedback.timestamp
            if age > FEEDBACK_STALE_S or rt.feedback.fault_bits != 0:
                stale.append(rt.index)
        return stale

    def step(self) -> list[MotorFeedbackState]:
        """执行一个控制周期：下发 16 路控制量并收集反馈。"""
        t0 = time.time()

        for rt in self._motors:
            rt.feedback.fresh = False

        with self._lock:
            snapshot = [
                (
                    rt.slot,
                    MotorCommand(
                        torque_nm=_clip_abs(
                            rt.command.torque_nm,
                            self.command_torque_limit_nm,
                        ),
                        velocity_rad_s=_clip_abs(
                            rt.command.velocity_rad_s,
                            self.command_speed_limit_rad_s,
                        ),
                    ),
                )
                for rt in self._motors
            ]

        for slot, cmd in snapshot:
            if not self._motors[_motor_index(slot.can_channel, slot.motor_id)].enabled:
                continue
            ch = self._bus.tx_channel_for_can_index(slot.can_channel)
            mid = slot.motor_id
            if slot.run_mode == RUN_MODE_CURRENT:
                torque_nm = _clip_abs(cmd.torque_nm, self.command_torque_limit_nm)
                iq = torque_to_iq(torque_nm, slot.model)
                self._bus.write_lingzu(ch, build_set_iq_ref_command(mid, iq))
            elif slot.run_mode == RUN_MODE_SPEED:
                speed_rad_s = _clip_abs(
                    cmd.velocity_rad_s,
                    self.command_speed_limit_rad_s,
                )
                self._bus.write_lingzu(ch, build_set_spd_ref_command(mid, speed_rad_s))

        self._bus.flush_tx()

        rx_wait = max(1.0, (self.dt - (time.time() - t0)) * 1000.0 * 0.6)
        for com, rx_ch, can_id, data in self._bus.drain_all(wait_ms=rx_wait):
            can_idx = self._bus.can_index_from_rx(com, rx_ch)
            if can_idx is None:
                continue
            mid = motor_id_from_can_id(can_id)
            idx = _motor_index(can_idx, mid)
            if idx < 0 or idx >= MOTOR_COUNT:
                continue
            self._try_apply_feedback(idx, can_id, data)

        if self._auto_recover:
            for idx in self._check_health():
                rt = self._motors[idx]
                if rt.recover_fail_count >= self._max_recover_attempts:
                    continue
                rt.recover_fail_count += 1
                self.recover_motor(idx, verbose=False)

        elapsed = time.time() - t0
        if elapsed < self.dt:
            time.sleep(self.dt - elapsed)

        self._cycle_count += 1
        result = self.get_all_feedback()
        if self._on_cycle:
            self._on_cycle(result)
        return result

    def _apply_feedback(self, index: int, fb: MotorFeedback) -> None:
        rt = self._motors[index]
        rt.feedback.position = fb.position
        rt.feedback.velocity = fb.velocity
        rt.feedback.torque = fb.torque
        rt.feedback.temperature = fb.temperature
        rt.feedback.fault_bits = fb.fault_bits
        rt.feedback.online = True
        rt.feedback.fresh = True
        rt.feedback.timestamp = time.time()

    def start(
        self,
        on_cycle: Optional[Callable[[list[MotorFeedbackState]], None]] = None,
    ) -> None:
        """启动后台控制/反馈线程。"""
        if self._running:
            return
        self._on_cycle = on_cycle
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        while self._running:
            self.step()

    @property
    def cycle_count(self) -> int:
        return self._cycle_count
