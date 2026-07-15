# RS06 MIT 运控控制程序

灵足时代 **RS06** 关节电机的 MIT（运控）模式控制工程，通过 **USB 转 CAN** 串口模块通信。

## 功能

- MIT 运控五元组控制：`position / velocity / kp / kd / torque`
- 电机使能、失能、标零、参数读取
- 自动设置 `run_mode = 0`（MIT 模式）
- 命令行参数与交互式命令两种输入方式
- 发送/接收帧十六进制打印，便于调试

## 环境要求

- Python 3.8+
- 灵足官方 USB-CAN 模块（串口帧头 `41 54`，帧尾 `0D 0A`）
- Windows 示例默认串口 `COM4`，波特率 `921600`

## 安装

```bash
cd rs06_mit_control
pip install -r requirements.txt
```

## 快速开始

### 1. 交互模式（推荐调试）

```bash
python main.py --port COM4 --motor-id 1
```

进入后可用命令：

```
rs06> enable          # 切换 MIT 并使能
rs06> set pos 0.5     # 设置目标位置 0.5 rad
rs06> set kp 30       # 设置 Kp
rs06> set kd 0.8      # 设置 Kd
rs06> mit             # 发送一帧 MIT 指令
rs06> loop 5          # 以 50Hz 循环控制 5 秒
rs06> read            # 读取位置/速度
rs06> disable         # 失能
```

### 2. 命令行单次控制

```bash
python main.py --mode once --port COM4 --motor-id 1 --pos 0.5 --kp 20 --kd 0.5
```

### 3. 命令行循环控制

```bash
python main.py --mode loop --duration 10 --hz 50 --pos 1.0 --kp 30 --kd 1.0
```

## 主要参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--port` | COM4 | USB 转 CAN 串口 |
| `--baudrate` | 921600 | 串口波特率 |
| `--motor-id` | 1 | 电机 CAN ID |
| `--host-id` | 0xFD | 主机反馈 ID |
| `--pos` | 0.0 | 目标位置 (rad) |
| `--vel` | 0.0 | 目标速度 (rad/s) |
| `--kp` | 20.0 | 位置刚度 |
| `--kd` | 0.5 | 阻尼 |
| `--tau` | 0.0 | 前馈力矩 (N·m) |
| `--hz` | 50.0 | 控制频率 |

也可在 `config.py` 中直接修改 `AppConfig` 默认值。

## 代码结构

```
rs06_mit_control/
├── main.py         # 入口：CLI + 交互模式
├── config.py       # 参数配置（串口/限幅/setpoint）
├── protocol.py     # CAN 协议与 MIT 编解码
├── transport.py    # USB-CAN 串口收发
├── motor.py        # RS06 高层控制 API
└── requirements.txt
```

## 在代码中调用

```python
from config import AppConfig, MitSetpoint
from motor import RS06Motor

cfg = AppConfig()
cfg.serial.port = "COM4"
cfg.can.motor_id = 1
cfg.setpoint = MitSetpoint(position=0.5, kp=30.0, kd=0.8)

motor = RS06Motor(cfg)
motor.connect()
motor.enable()
motor.send_mit()
motor.run_loop(duration_s=5.0)
motor.disconnect()
```

## 注意事项

1. 首次使用请先小幅度、低 Kp/Kd 测试，确认转向和零位正确。
2. 若电机无响应，检查 USB-CAN 模块 DIP 开关、供电和 CAN 接线/终端电阻。
3. 本程序使用灵足**私有 CAN 协议**下的运控模式（`run_mode=0`），不是 CANopen 模式。
4. RS06 量化限幅默认值见 `config.py` 中 `MotorLimits`，请与实物手册核对后调整。

## 16 电机（SANPO 四路 CAN）

针对 4×CAN、每路 3×RS06 + 1×RS01 的场景：

```bash
# 列出串口（SANPO 板常见 COM10/COM12）
py -3 robot16.py list-ports

# 扫描哪个口有电机回传
py -3 tools/probe_robot16.py

# 探测 16 电机是否在线（默认 sanpo_prefix 通道模式）
py -3 robot16.py ping --port COM10 --scan-factory

# 若只有单路灵足 USB-CAN，改用 --transport lingzu
py -3 robot16.py ping --port COM4 --transport lingzu

# 全接全上电，按 UUID 批量改 ID（每路 127→1,2,3,4）
py -3 robot16.py config --port COM10 --auto-all

# MIT 控制
py -3 robot16.py control --port COM10 --ping-only
py -3 robot16.py control --port COM10 --duration 5
```

新增文件：`motors16_config.py`、`sanpo_transport.py`、`multi_can16.py`、`robot16.py`。


```bash
# 列出串口
py -3 tools/diagnose_com.py --list-ports

# 完整握手 + MIT 测试（默认 COM4）
py -3 tools/diagnose_com.py --port COM4 --motor-id 1

# 扫描 motor_id
py -3 tools/diagnose_com.py --port COM4 --scan-ids

# 核对 AT 帧是否与 gamepad 工程一致
py -3 tools/verify_frames.py
```

若提示 `PermissionError`，请先关闭灵足官方上位机或其他占用 COM 口的程序。

## 协议说明（与 ESP32 实测工程对齐）

- 标准 CAN ID 格式：`TT 00 FD MM`
- **使能**是类型 `0x04` 且数据含 `00 C4`，不是类型 `0x03`
- **启动**才是类型 `0x03`
- USB 转 CAN 串口在 8 字节载荷前会加 `08` 前缀（init 短帧除外）

