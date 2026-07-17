# MCU 构建、烧录与现场调试注意

本文记录 2026-07-09 两块 MCU、四路 CAN、16 个电机 bring-up 的实际流程。

## 目录和目标

MCU 工程目录：

```bash
cd /home/gu/wheel_dog_pc_mcu/robot-v4-firmware-sanpo_spine/firmware/sanpo_spine
```

两块 MCU 使用同一套源码，但编译宏不同：

| MCU | 整机逻辑 CAN | 编译宏 | 输出目录 |
| --- | --- | --- | --- |
| MCU A | CAN1/CAN2, index `0..7` | `WD_CAN_BUS_BASE=0` | `build/Release/bus0_bench0_live1_rawread1/` |
| MCU B | CAN3/CAN4, index `8..15` | `WD_CAN_BUS_BASE=2` | `build/Release/bus2_bench0_live1_rawread1/` |

当前四路直连映射：

| 整机 CAN | MCU 物理 CAN | WDP4 index | 控制模式 |
| --- | --- | --- | --- |
| CAN1 | MCU A CAN1 | `0/1/2` RS06 current, `3` RS01 motion | `iq_ref`, type-1 motion command |
| CAN2 | MCU A CAN2 | `4/5/6` RS06 current, `7` RS01 motion | `iq_ref`, type-1 motion command |
| CAN3 | MCU B CAN1 | `8/9/10` RS06 current, `11` RS01 motion | `iq_ref`, type-1 motion command |
| CAN4 | MCU B CAN2 | `12/13/14` RS06 current, `15` RS01 motion | `iq_ref`, type-1 motion command |

## 编译

```bash
make release-mcu-a
make release-mcu-b
```

生成文件：

```text
build/Release/bus0_bench0_live1_rawread1/sanpo_spine.bin
build/Release/bus2_bench0_live1_rawread1/sanpo_spine.bin
```

`Makefile` 已按 `bus_base/bench/live` 拆分构建目录，避免切换 MCU A/B 配置后误刷旧 bin。

## 烧录

若 ST-LINK 只接一块板，先确认当前接的是 MCU A 还是 MCU B，再烧对应固件。

推荐命令如下，原因是现场 ST-LINK 提示 `NRST is not connected`，普通 `--reset write` 可能无法连接目标。

MCU A：

```bash
LD_LIBRARY_PATH=/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/lib/x86_64-linux-gnu \
/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/bin/st-flash \
  --connect-under-reset --reset write \
  build/Release/bus0_bench0_live1_rawread1/sanpo_spine.bin 0x08000000
```

MCU B：

```bash
LD_LIBRARY_PATH=/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/lib/x86_64-linux-gnu \
/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/bin/st-flash \
  --connect-under-reset --reset write \
  build/Release/bus2_bench0_live1_rawread1/sanpo_spine.bin 0x08000000
```

看到 `Flash written and verified` 才算烧录成功。

烧录后如果 USB CDC 没有 WDP4 feedback，手动 reset 两块 MCU。由于 NRST 没接，烧录工具的 reset 不一定能让应用和 USB CDC 干净重启。

## 串口权限和端口识别

常见端口：

```text
/dev/ttyACM0
/dev/ttyACM1
```

如果权限不足，设备通常是 `root:dialout`。处理方式：

```bash
sudo usermod -aG dialout $USER
```

然后重新登录，或临时用 sudo/提升权限运行测试脚本。Codex 沙箱内访问 `/dev/ttyACM*` 也需要提升权限。

端口可能在 reset 或重新插拔后交换，必须用 WDP4 状态识别：

```bash
cd /home/gu/wheel_dog_pc_mcu
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM0 --duration 2
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM1 --duration 2
```

带 `mcu-base2` 的是 MCU B；不带的是 MCU A。

## 只读安全检查

真实 live 控制前先做 READ_PARAM 只读轮询。

```bash
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM0 --duration 2 --poll-readonly --poll-bus can1 --print-joints
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM0 --duration 2 --poll-readonly --poll-bus can2 --print-joints
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM1 --duration 2 --poll-readonly --poll-bus can1 --print-joints
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM1 --duration 2 --poll-readonly --poll-bus can2 --print-joints
```

## 装配后只读检查工具

实时读取 16 台电机的原始 `0x7019 mech_pos`：

```bash
cd /home/gu/wheel_dog_pc_mcu
/home/gu/anaconda3/bin/python3 wheeldog/tools/read_all_motor_angles.py \
  --ports /dev/ttyACM0,/dev/ttyACM1 \
  --confirm-supported
```

一次性验证四路 CAN 的 ID1~4，并打印逻辑序号和预期关节：

```bash
/home/gu/anaconda3/bin/python3 wheeldog/tools/verify_all_can_joint_mapping.py \
  --ports /dev/ttyACM0,/dev/ttyACM1 \
  --confirm-supported
```

两个程序都会根据 `mcu-base2` 自动识别 MCU A/B，串口顺序不影响
CAN1~CAN4 映射。它们固定为 dry-run，不请求使能，退出时发送急停。

## Live 控制安全规则

真实控制必须显式带三个参数：

```text
--live-control --enable-request --i-understand-live-can
```

脚本侧硬拒绝超过：

```text
RS06 position/torque channels: |tau| <= 36.0 Nm
RS01 motion wheel target:       |dq_des| <= 20.0 rad/s
RS01 motion damping:            Kp=0, Kd=0.4, tau_ff=0
RS01 motion torque ceiling:     limit_torque=17.0 Nm
```

2026-07-14 起，RS01 不再使用 `run_mode=2 + spd_ref` 的内部速度 PI 环。MCU 在
失能状态写 `run_mode=0` 和 `limit_torque=17 Nm`，使能后发送通信类型1：

```text
tau = 0.4 * (dq_des - dq)
```

这与 RobotLab 轮执行器 `stiffness=0, damping=0.4` 对齐。正常 StandUp/RL 的零轮速
仍携带 `Kd=0.4`，属于柔性阻尼；Idle、急停和 disable 前的清零帧使用 `Kd=0`，是真正
零力矩。手册要求运行中不可切换模式，固件因此只在 enable 前设置 `run_mode`。

独立 `wd_mcu_dryrun.py` 和正式部署均把轮虚拟目标限制为 `+/-20 rad/s`；
资格激励模式仍严格限制为单电机 `+/-1 rad/s`。
正式部署已取消轮子的 `8 -> 10 rad/s` 实测速降额/锁停以及 `600 Nm/s`
同向增扭斜率；轮目标受部署 `+/-20 rad/s` 和17 Nm峰值限制。RS01 原生
CAN 速度字段仍按手册使用 `+/-44 rad/s` 编解码，不能随部署限幅一起改动。
腿关节仍保留8 rad/s降额起点与连续三帧10 rad/s的 `measured-overspeed`
锁停，并在MCU当前1 kHz服务tick立即开始本地zero/disable。

live 启动后固定发送零命令：`COMM_OPERATION_STATUS` 的 p/v 必须与
`READ_PARAM mech_pos/mech_vel` 连续三次吻合、固件测得频率 `>=450 Hz`，才会置
`fast-feedback-ready`。正式 bridge 启动自检不再发送逐电机激励；`pex/vex` mask
仅保留为历史/手动动态诊断，不参与 ready。静态检查证明反馈新鲜度和当前姿态下的
通道一致性，不重新证明动态速度缩放或机械方向；方向由吊装后的 `z` 测试确认。

需要独立动态诊断时，仍可手动使用专用资格标志，一次一个电机：

```bash
# RS06 示例；INDEX 按 MCU 的全局逻辑编号填写
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM0 --duration 3 \
  --live-control --enable-request --i-understand-live-can \
  --qualification-excitation --joint-torque 0:0.6 --print-joints

# RS01 示例
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM0 --duration 3 \
  --live-control --enable-request --i-understand-live-can \
  --qualification-excitation --joint-velocity 3:1.0 --print-joints
```

该手动工具硬拒绝同时激励两个电机、RS06 非 `±0.6 Nm` 或 RS01 非 `±1 rad/s`；
RS06 激励仍受绝对 URDF 目标裁剪和 raw 安全墙保护。正式部署不会调用该路径。
工具退出前会自动发送
zero/dry-run cleanup。两块 MCU 在 Linux 下仍是同一物理 USB 下的两个 CDC 设备，
端口号必须通过 `mcu-base2` 识别。

每次 live 后都用普通 dry-run 撤销：

```bash
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM0 --duration 3 --print-joints
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM1 --duration 3 --print-joints
```

## 频率说明

当前实测 WDP4 feedback 的 `actual_control_hz` 为：

```text
MCU A: 1000.0 Hz
MCU B: 1000.0 Hz
```

这表示 MCU 本地控制任务以 1 kHz 运行。

2026-07-10 起，CAN live 调度已改为两路 CAN 并行、每路每毫秒排队两个到期命令，
四电机交错后每电机目标为 `500 Hz`。验证时不要只看 `actual_control_hz=1000`，
还必须同时观察逐电机 `live_command_tx_completed` 和
`operation_status_rx_count` 差分；正式 PC bridge 默认要求 16 路命令 TX 与快速反馈
都达到 `>=450 Hz`、`fast_feedback_valid_mask=0xffff`，且没有超过 5 ms 的调度延迟
才会进入 ready。只有 TX 达标不算闭环达标。

正式 RL 观测按 64 帧短历史匹配双 MCU 相同 `observation_seq`，不再要求两个端口的
“当前最新包”恰好同帧；一次 USB 延迟造成某端领先后可以自动追平。MCU 快反馈有效
边界仍为 `5 ms`，PC 报告样本年龄最大 `10 ms`，同序号两包的 USB 到达偏差最大
`30 ms`，历史帧在 PC 内最多保留 `100 ms`。bridge 只把满足条件的整帧写入共享
内存，不拼接两个不同 setpoint epoch；运行中连续 `500 ms` 没有新的合格整帧才
故障并撤销 live。

operation-status freshness 采用两级处理，不能把它理解成“放宽到300 ms后继续保持
计算”：

- 样本年龄超过 `5 ms`：该电机立即发零命令，旧反馈不会再次用于PD/速度刷新；
- 持续到 `300 ms`：MCU 全机锁存停机；反馈频率资格丢失时也锁停。
  动态快慢交叉误差只诊断，不参与停机；
- PC bridge 不发布短缺口期间的 observation，只接受16电机完整同 epoch 新鲜帧。

新 feedback 诊断 `operation_status_max_gap_ms[]` 记录本次 live 的每电机最大帧间隔，
`live_stop_fast_age_ms[]` 锁存停机瞬间的反馈年龄，`final_joint_torque_cmd_nm[]`
记录所有安全处理后的最终关节力矩，并在停机时保留清零前快照。当前平地可行性验收除平均 `>=450 Hz` 外，还应
归档这两个字段；若最大间隔反复超过5 ms，继续查CAN布线/负载/任务调度，不能用平均
500 Hz掩盖。

`operation_status_max_gap_ms[]` 在八个本地电机首次全部 fast-ready 时清零，从而排除
分阶段 enable 的启动空档。首次资格仍严格要求快慢速度误差 `<=0.20 rad/s`；这
比较的是两条实际反馈通道，不是目标跟踪误差。资格建立后，异步 READ_PARAM/0x02
动态差异继续记录但不撤销 match latch、不停机，因为协议没有共同采样时间戳。
停机日志会进一步标出 stale、rate-lost 或 startup reference-lost；排障时不要只看
总类 `fast-feedback-lost`。

CDC TX feedback payload 必须保存在静态 RAM，不能放回只有1024字节的发送任务栈。
当前 payload/packet 为 `1172/1188 B`，新增每电机低频 VBUS 值与有效位；
双MCU 200 Hz上行约 `475.2 kB/s`。

## 本次调试结论

- 两块 MCU 固件均可构建、烧录、reset 后正常回 WDP4 feedback。
- 四路 CAN、16 个电机映射正确。
- 带动力冷启动已确认 16/16 电机自动发现；`GET_DEVICE_ID` 使用 DLC=2
  的 `01 00`，不再依赖预先存在的电机反馈。
- RobStride 0x02 已按手册使用 RS06 `±4π/±50/±36`、RS01
  `±4π/±44/±17` 的位置/速度/力矩缩放；16 路快慢位置交叉误差实测约
  `0..0.002 rad`。
- 全部电机使用单电机资格激励通过：RS06 `0.6 Nm`，RS01 `1 rad/s`。
- 2026-07-10 曾根据静摩擦把台架版调整为 RS06 `2.0 Nm` / RS01 `4.0 rad/s`；
  2026-07-11 机械绝对标定方向测试使用过 RS06 `8.0 Nm` / RS01 `2.0 rad/s`；
  2026-07-13 根据整机 URDF 和趴地偏载静力估算，将当前正式配置调整为
  RS06 `20.0 Nm` / RS01 `8.0 rad/s`；同日曾将 RS01 速度环 `i_limit` 从等效
  `2.0 Nm` 提升为 `8.0 Nm`（按 `1.22 Nm/A` 换算约 `6.56 A`），作为短时峰值上限，
  不是持续力矩命令。2026-07-14 已由运控模式取代该速度环配置；
  Idle 的显式零增益/零前馈命令保持零力矩。
- 2026-07-14 当前正式配置已将 RS06 腿 effort ceiling 与训练对齐为
  `36.0 Nm`，RS01 `limit_torque` 开放到 `17.0 Nm`；力矩增长斜率层已移除。
- 正式双 MCU bridge 零命令稳定验收：A/B 控制任务均 `1000.0 Hz`，16/16 电机
  `command TX=500.0 Hz`、`operation feedback=500.0 Hz`，`fast=0xffff`，
  `fault=0`，无 CAN 错误或 deadline miss。
- PC 以 200 Hz 发布双 MCU 同 sequence 整帧，RL 仍以 50 Hz 取最新合格观测；
  本次最大样本年龄 `1 ms`，
  到达偏斜 `0 us`（当前门限 `30 ms`），丢失 epoch `0`，bridge 清理退出码 `0`。
- 2026-07-11 已切换机械绝对标定：编码器相位按 `2*pi`，12 个腿关节使用独立 raw
  硬边界，轮位置连续展开；首次整机运行仍须按 `ABSOLUTE_LIMIT_DEPLOY_SEQUENCE.md`
  完成双 MCU 重刷和逐电机方向复核。
