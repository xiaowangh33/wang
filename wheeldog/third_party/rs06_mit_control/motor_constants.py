"""灵足电机力矩↔峰值电流换算系数（官方规格表）。

RS06: Kt = 1.09 N·m/Arms，最大相电流 = 57 Apk
RS01: Kt = 1.22 N·m/Arms，最大相电流 = 23 Apk

RobStride 的 ``iq_ref``/``limit_cur`` 参数使用峰值安培，而规格表的
转矩常数使用 RMS 安培。因此接口层的 N·m 与协议层的 Apeak 之间必须包含
``sqrt(2)`` 换算。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from motors16_config import MotorModel

# 官方 Torque Constant，单位 N·m/Arms
RS06_KT = 1.09
RS01_KT = 1.22

RMS_TO_PEAK = sqrt(2.0)

# 官方峰值力矩 (N·m)，用于控制量限幅
RS06_TORQUE_PEAK = 36.0
RS01_TORQUE_PEAK = 17.0

# iq_ref/limit_cur 的协议单位为峰值安培，与手册最大相电流一致。
RS06_IQ_MAX = 57.0
RS01_IQ_MAX = 23.0


@dataclass(frozen=True)
class MotorElectricalSpec:
    model: MotorModel
    kt_nm_per_a: float  # N·m/Arms
    iq_max_a: float  # Apeak
    torque_max_nm: float


SPECS: dict[MotorModel, MotorElectricalSpec] = {
    "rs06": MotorElectricalSpec("rs06", RS06_KT, RS06_IQ_MAX, RS06_TORQUE_PEAK),
    "rs01": MotorElectricalSpec("rs01", RS01_KT, RS01_IQ_MAX, RS01_TORQUE_PEAK),
}


def torque_to_iq(torque_nm: float, model: MotorModel) -> float:
    """力矩 (N·m) → iq_ref (Apeak)。"""
    spec = SPECS[model]
    t_clamped = max(-spec.torque_max_nm, min(spec.torque_max_nm, torque_nm))
    iq = t_clamped * RMS_TO_PEAK / spec.kt_nm_per_a
    return max(-spec.iq_max_a, min(spec.iq_max_a, iq))


def iq_to_torque(iq_a: float, model: MotorModel) -> float:
    """iq_ref (Apeak) → 力矩 (N·m)，用于调试。"""
    return iq_a / RMS_TO_PEAK * SPECS[model].kt_nm_per_a
