"""RS06 MIT 控制默认参数配置。"""

from dataclasses import dataclass, field


@dataclass
class MotorLimits:
    """RS06 运控模式量化限幅（参考官方协议与同系列电机文档）。"""

    p_max: float = 12.566370614359172  # rad，RobStride 06 运控反馈 ±4π
    v_max: float = 50.0               # rad/s，RobStride 06 运控反馈量程
    t_max: float = 36.0               # N·m，RobStride 06 运控反馈量程
    kp_max: float = 5000.0    # 位置刚度上限
    kd_max: float = 100.0     # 阻尼上限


@dataclass
class SerialConfig:
    """USB 转 CAN 串口配置。"""

    port: str = "COM4"
    baudrate: int = 921600
    timeout: float = 0.05


@dataclass
class CanConfig:
    """CAN 通信 ID 配置。"""

    motor_id: int = 1
    host_id: int = 0xFD


@dataclass
class MitSetpoint:
    """MIT 运控五元组。"""

    position: float = 0.0   # rad
    velocity: float = 0.0   # rad/s
    torque: float = 0.0     # N·m
    kp: float = 20.0
    kd: float = 0.5


@dataclass
class CurrentSetpoint:
    """电流/力控模式给定（iq_ref，单位 Apeak）。"""

    iq: float = 0.0


@dataclass
class ControlConfig:
    """控制循环与打印配置。"""

    loop_hz: float = 50.0
    verbose: bool = True
    print_feedback: bool = True
    ensure_mit_mode: bool = True
    control_mode: str = "mit"  # mit | current


@dataclass
class AppConfig:
    """应用总配置，便于在代码中集中修改。"""

    serial: SerialConfig = field(default_factory=SerialConfig)
    can: CanConfig = field(default_factory=CanConfig)
    limits: MotorLimits = field(default_factory=MotorLimits)
    setpoint: MitSetpoint = field(default_factory=MitSetpoint)
    current: CurrentSetpoint = field(default_factory=CurrentSetpoint)
    control: ControlConfig = field(default_factory=ControlConfig)
