"""SANPO 四路 CAN × 每路 3×RS06 + 1×RS01 默认布局。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

from config import MitSetpoint, MotorLimits, SerialConfig
from protocol import RUN_MODE_CURRENT, RUN_MODE_SPEED

MotorModel = Literal["rs06", "rs01"]

DEFAULT_FACTORY_ID = 127

RS06_LIMITS = MotorLimits(
    p_max=12.566370614359172,
    v_max=50.0,
    t_max=36.0,
    kp_max=5000.0,
    kd_max=100.0,
)
RS01_LIMITS = MotorLimits(
    p_max=12.566370614359172,
    v_max=44.0,
    t_max=17.0,
    kp_max=500.0,
    kd_max=5.0,
)

MOTORS_PER_CHANNEL = 4
CHANNEL_COUNT = 4


@dataclass
class MotorSlot:
    name: str
    can_channel: int
    motor_id: int
    model: MotorModel
    run_mode: int = RUN_MODE_CURRENT
    setpoint: MitSetpoint = field(default_factory=MitSetpoint)
    factory_id: int = DEFAULT_FACTORY_ID

    @property
    def limits(self) -> MotorLimits:
        return RS06_LIMITS if self.model == "rs06" else RS01_LIMITS


@dataclass
class Robot16Config:
    serial: SerialConfig = field(default_factory=SerialConfig)
    host_id: int = 0xFD
    loop_hz: float = 400.0
    transport_mode: str = "sanpo"
    motors: list[MotorSlot] = field(default_factory=list)


def default_robot16() -> Robot16Config:
    motors: list[MotorSlot] = []
    channel_names = ["can1", "can2", "can3", "can4"]
    layout = [
        (1, "rs06", "m1", RUN_MODE_CURRENT),
        (2, "rs06", "m2", RUN_MODE_CURRENT),
        (3, "rs06", "m3", RUN_MODE_CURRENT),
        (4, "rs01", "m4", RUN_MODE_SPEED),
    ]
    for ch, ch_name in enumerate(channel_names):
        for mid, model, suffix, run_mode in layout:
            kd = 0.5 if model == "rs06" else 0.3
            motors.append(
                MotorSlot(
                    name=f"{ch_name}_{suffix}",
                    can_channel=ch,
                    motor_id=mid,
                    model=model,
                    run_mode=run_mode,
                    setpoint=MitSetpoint(kp=15.0, kd=kd),
                )
            )
    return Robot16Config(motors=motors)


def target_ids_per_channel() -> list[int]:
    return [1, 2, 3, 4]
