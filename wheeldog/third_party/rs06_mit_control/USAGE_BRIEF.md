# RS06 MIT 控制项目使用简报

## 1. 项目定位

这是一个面向灵足时代 RS06/RS01 电机的 Python 控制工程。它通过串口访问 USB-CAN 或 SANPO 多路 CAN 网关，封装灵足 RobStride 私有 CAN 协议，支持：

- 单 RS06 电机 MIT 五元组控制：`position / velocity / kp / kd / torque`
- 单 RS06 电机电流/力控模式：写 `iq_ref`
- 同一 CAN 总线上的双/多 RS06 电流控制
- SANPO 四路 CAN、共 16 电机的探测、改 ID、模式配置、400 Hz 目标控制和反馈读取

默认工程偏 Windows 现场调试：单电机示例串口是 `COM4`，SANPO 双 MCU 常见串口是 `COM10` 和 `COM12`，波特率默认 `921600`。

## 2. 环境与硬件先决条件

- Python 3.8+
- `pyserial`，安装命令：

```bash
python -m pip install -r requirements.txt
```

- 灵足官方 USB-CAN 模块，或 SANPO 四路 CAN 网关
- 电机已独立供电，CAN_H/CAN_L 接线正确，并按总线需要接终端电阻
- 运行脚本前关闭灵足官方上位机、串口助手等占用 COM 口的软件

当前本机检查结果：项目依赖文件只声明了 `pyserial>=3.5`，但当前 Python 环境没有安装 `serial` 模块，所以 `python main.py --help` 和 `python robot16.py --help` 会在导入串口传输层时报 `ModuleNotFoundError: No module named 'serial'`。先安装依赖再运行控制脚本。

## 3. 单电机 RS06 快速流程

建议先用诊断脚本确认串口和电机 ID，再进入交互控制。

```bash
python tools/diagnose_com.py --list-ports
python tools/diagnose_com.py --port COM4 --motor-id 1
python tools/diagnose_com.py --port COM4 --scan-ids
```

进入交互模式：

```bash
python main.py --port COM4 --motor-id 1
```

交互模式常用命令：

```text
ping                  获取设备 ID
enable                设置 run_mode 并使能
set pos 0.2           设置目标位置 rad
set kp 10             设置较低 Kp
set kd 0.3            设置阻尼
mit                   发送一帧 MIT 指令
loop 5                按当前频率循环 5 秒
read                  读取 mechPos / mechVel
disable               失能
quit                  退出
```

单次 MIT 控制：

```bash
python main.py --mode once --port COM4 --motor-id 1 --pos 0.2 --kp 10 --kd 0.3
```

循环 MIT 控制：

```bash
python main.py --mode loop --port COM4 --motor-id 1 --duration 5 --hz 50 --pos 0.2 --kp 10 --kd 0.3
```

电流/力控模式：

```bash
python main.py --mode force --port COM4 --motor-id 1 --iq 0.5 --duration 3 --hz 50
```

`--iq` 是直接写入驱动参数的峰值电流，单位为 Apeak，不是 Arms。

安全建议：首次调试使用很小位移、低 `kp/kd`、低 `iq`，确认零位、方向、限位和急停手段后再提高控制量。

## 4. 单 CAN 总线多电机

`main.py` 在传入多个 ID 时会走 `RS06Bus`，适合一个 USB-CAN 模块挂多台 RS06 的简单场景。当前多电机入口主要支持 `ping` 和电流/力控循环。

```bash
python main.py --port COM4 --motor-ids 1,127 --mode ping
python main.py --port COM4 --motor-ids 1,127 --mode dual-force --iq 0.5 --duration 3 --hz 30
```

也可以使用专用测试脚本：

```bash
python tools/test_dual_motor.py --port COM4 --motor-ids 1,127 --iq1 0.5 --iq2 -0.5 --duration 3
```

## 5. 16 电机 / SANPO 使用流程

默认 16 电机布局在 `motors16_config.py`：

- CAN1~CAN4 共 4 路
- 每路 ID `1,2,3` 是 RS06，默认运行模式为电流/力控 `run_mode=3`
- 每路 ID `4` 是 RS01，默认运行模式为速度 `run_mode=2`
- RS06 力矩到电流换算使用 `Kt=1.09 N.m/Arms`；驱动的 `iq_ref` 和
  `limit_cur` 使用峰值安培，因此换算为 `iq_ref = torque * sqrt(2) / Kt`

Linux 下 SANPO 双 MCU 通常会显示为 `/dev/ttyACM0` 和 `/dev/ttyACM1`。下面命令里的 `--ports` 顺序必须是 `MCU1(CAN1/2),MCU2(CAN3/4)`；如果不确定，先用只读扫描确认，不要先写 Flash。

推荐现场顺序：

```bash
python tools/check_firmware.py --list-ports
python tools/check_firmware.py --ports /dev/ttyACM0,/dev/ttyACM1
python tools/scan_can_ports.py --ports /dev/ttyACM0,/dev/ttyACM1
python tools/config_motor_modes.py --ports /dev/ttyACM0,/dev/ttyACM1
```

如果使用 `robot16.py` 这套单串口/SANPO 入口：

```bash
python robot16.py list-ports
python robot16.py ping --port COM10 --transport sanpo --scan-factory
python robot16.py control --port COM10 --transport sanpo --ping-only
python robot16.py control --port COM10 --transport sanpo --duration 5
```

批量改 ID 是高风险操作，执行前确认每路 CAN 的电机、供电和 UUID 收集都正确：

```bash
python robot16.py config --port COM10 --transport sanpo --auto-all
```

如果使用双 MCU ET 总线接口，`robot16_interface.py` 会同时打开 `--ports` 指定的两个 MCU 串口，更适合上层程序集成或 400 Hz 目标频率闭环控制演示：

```bash
python tools/demo_robot16_interface.py --ports /dev/ttyACM0,/dev/ttyACM1 --verified-enable --duration 5 --torque 0.6 --velocity 0.3
```

## 6. 程序化调用示例

单电机：

```python
from config import AppConfig, MitSetpoint
from motor import RS06Motor

cfg = AppConfig()
cfg.serial.port = "COM4"
cfg.can.motor_id = 1
cfg.setpoint = MitSetpoint(position=0.2, kp=10.0, kd=0.3)

motor = RS06Motor(cfg)
motor.connect()
try:
    motor.enable()
    motor.send_mit()
finally:
    motor.disable()
    motor.disconnect()
```

16 电机接口：

```python
from robot16_interface import Robot16Interface

robot = Robot16Interface(hz=400.0)
try:
    robot.open()
    robot.enable_all_verified()
    robot.set_torque(0, 0.6)      # RS06, N.m
    robot.set_velocity(3, 0.3)    # RS01, rad/s
    feedback = robot.step()
finally:
    robot.disable_all()
    robot.close()
```

## 7. 核心文件地图

- `main.py`：单电机 CLI、交互模式、单 CAN 多电机简易入口
- `config.py`：串口、CAN ID、MIT setpoint、限幅、控制频率等默认配置
- `protocol.py`：RobStride 通信类型、MIT 编解码、AT/ET 帧解析和参数读写帧构造
- `transport.py`：灵足 USB-CAN 串口读写线程和收发缓存
- `motor.py`：`RS06Motor` 单电机高级 API
- `multi_motor.py`：同一 CAN 总线多 RS06 的 `RS06Bus`
- `motors16_config.py`：16 电机默认拓扑、RS06/RS01 限幅和目标 ID
- `multi_can16.py` / `robot16.py`：SANPO 单串口风格的 16 电机探测、改 ID、MIT 控制
- `sanpo_et.py` / `robot16_interface.py`：双 MCU ET 协议和 16 电机统一控制/反馈 API
- `tools/`：固件检查、串口诊断、CAN 扫描、模式配置、帧验证和演示脚本

## 8. 常见问题排查

- 缺少依赖：出现 `No module named 'serial'`，执行 `python -m pip install -r requirements.txt`
- 串口占用：出现 `PermissionError` 或打开失败，关闭上位机/串口助手后重试
- 无任何回传：检查电机供电、CAN 接线、终端电阻、DIP 开关、串口号和波特率
- 电机 ID 不确定：单电机用 `tools/diagnose_com.py --scan-ids`，SANPO 用 `tools/scan_can_ports.py`
- 16 电机通道混乱：以 `sanpo_bus_map.py` 为准，`COM10` 对应 MCU1 的 CAN1/CAN2，`COM12` 对应 MCU2 的 CAN3/CAN4
- 力矩给定太小不转：RS06 示例里 0.2 N.m 可能不足以克服静摩擦，演示脚本建议从 0.6 N.m 附近验证，但必须先确认机械安全

## 9. 操作红线

- 不要在机械结构未固定、限位未知、人员靠近运动范围时执行循环控制
- 不要在同一 CAN 总线上随意批量改 ID；改 ID 前先记录 UUID 和物理位置
- 不要直接使用高 `kp/kd`、高 `iq` 或大位移做首次测试
- `config.py` 中的 RS06/RS01 限幅来自项目内默认值，正式使用前应与实际电机手册核对
