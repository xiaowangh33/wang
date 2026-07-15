# PC+MCU PD 下沉迁移记录

> 当前状态（2026-07-14）：本文为按日期保留的演进记录，后文中的
> `50 Hz PC setpoint`、`20 Nm RS06`、`8 Nm RS01` 和 torque-slew 均是历史阶段，
> 不是当前部署配置。当前契约为：ONNX/训练控制周期
> `20 ms / 50 Hz`，PC→MCU 传输 `5 ms / 200 Hz`，RS06 腿 `36 Nm`，
> RS01 轮 `17 Nm`，且不再施加力矩增长斜率。当前操作说明以
> `REALTIME_CONTROL_NOTES.md` 和 `mcu_build_flash_debug_guide.md` 为准。

## 当前阶段

截至 2026-07-10，本项目已从 dry-run 骨架进入双 MCU、四路 CAN、16 电机
带动力实机验收阶段：PC 以 50 Hz 向两块 MCU 发送 16×5 目标，MCU 用
1 kHz 控制任务保持目标，每个电机的命令 TX 与参与闭环的快速反馈均已
实测达到 `500.0 Hz`。MCU 本地 PD、CAN 调度、反馈资格验证、安全停止与
双 MCU 同 sequence 观测屏障均已接入真实 live 路径。

当前仍然只能视为裸轴台架版：位置零点是 MCU 本次上电首份有效反馈，限位是
startup-relative；机械装配前必须完成绝对零点、方向、机械上下限及单圈绝对编码器
跨边界策略。

新增 MCU 文件：

- `Core/Inc/wd_protocol.h`
- `Core/Src/wd_protocol.c`
- `Core/Inc/wd_safety.h`
- `Core/Src/wd_safety.c`
- `Core/Inc/wd_control.h`
- `Core/Src/wd_control.c`

新增 PC dry-run 工具：

- `wheeldog/tools/wd_mcu_dryrun.py`

接入点：

- `USB_DEVICE/App/usbd_cdc_if.c`
  - 收到 `WDP4` 新协议包时进入 `wd_control_usb_receive`
  - 非新协议数据继续走原 `cdc_recv_fs` 透明转发链路
- `Core/Src/freertos.c`
  - 新增静态分配的高优先级 `wdControlTask`
  - `cdcTXTask` 在新协议激活后发送 dry-run feedback，否则继续旧 `cdc_tx_loop`
  - 如果 `wdControlTask` 未创建成功，`cdcTXTask` 会跑降级 tick，并在 feedback 里置 `control-fallback`

## 频率边界

目标架构：

```text
PC:
  RL/state machine 50 Hz 生成 16x5 setpoint

MCU:
  1000 Hz 控制 tick
  当前阶段 dry-run，只做 setpoint 校验、状态统计、feedback 回包
  后续阶段接入 CAN feedback、PD 计算、RS06 iq_ref、RS01 spd_ref
```

当前 `wdControlTask` 使用静态栈/静态控制块创建，避免 generated heap 紧张时动态线程创建失败。dry-run 阶段用高优先级 `osDelay(1)` tick，2026-07-09 实测 feedback 上报 `hz=1000.0`。

真实电机闭环阶段建议继续收敛为 TIM 或 RTOS semaphore 驱动，让 PD 计算周期不依赖普通任务调度和 USB 发送路径。

当前二进制协议尺寸：

```text
header = 16 bytes
joint command = 20 bytes
PC -> MCU setpoint payload = 352 bytes
joint feedback = 24 bytes
MCU -> PC feedback payload = 672 bytes（含逐电机 CAN TX 诊断）
```

`Core/Src/wd_protocol.c` 里已经加入编译期尺寸断言，防止 PC/MCU 两端结构体布局后续无意错位。

## 限位模式

当前只允许：

```c
WD_LIMIT_STARTUP_RELATIVE
```

含义：

```text
q = sign * (motor_position - startup_motor_position)
limit = startup_zero +/- 120 deg
```

这是调试/bring-up 保护，不是正式机械安全限位。

PC 如果请求：

```c
WD_LIMIT_CALIBRATED_ABSOLUTE
```

MCU 会：

1. 打 `WD_STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED`
2. 回退到 `WD_LIMIT_STARTUP_RELATIVE`
3. 不假装绝对限位已经生效

后续正式限位应改成：

```text
q = sign * (motor_position - calibrated_motor_zero) + joint_zero_offset
limit = absolute_joint_lower / absolute_joint_upper
```

并填入每个关节的绝对零点、机械上下限和方向标定表。

## 力矩和速度处理原则

腿关节：

```text
sanitize setpoint
-> clamp q_des/dq_des/tau_ff
-> tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff
-> position soft limit
-> per-joint effort limit
-> bring-up global torque limit
-> torque slew-rate limit
-> RS06 torque -> iq_ref
-> current limit
```

轮关节：

```text
ignore q_des/kp for wheel
-> clamp dq_des
-> optional acceleration limit
-> RS01 spd_ref
```

MCU 本地关节配置表是最终裁决者。PC 包里带的 actuator mode 如果和 MCU 表不一致，MCU 会强制修正并置 `WD_STATUS_BAD_PACKET`。

2026-07-09 补强 safety 状态出口：

- setpoint 内出现 NaN/Inf、负 `kp/kd`、超出速度/位置/力矩限幅，MCU 会夹紧并置 `WD_STATUS_SETPOINT_CLIPPED`。
- 轮电机速度模式只接受 `dq_des`；`kp/kd/q_des/tau_ff` 会被清零并置 `WD_STATUS_SETPOINT_CLIPPED`。
- dry-run 阶段 PC 即使误发 `WD_CONTROL_FLAG_ENABLE_REQUEST`，MCU 也会清掉该请求并置 `WD_STATUS_ENABLE_BLOCKED_DRY_RUN`。
- 已增加独立 `wd_safety_limit_torque_command()` 和 `wd_safety_limit_velocity_command()`，后续 CAN 写帧必须从这两个出口拿最终命令。
- `WD_STATUS_TORQUE_SLEW_LIMITED` 由转矩 slew-rate 限幅函数置位；当前 dry-run 仍不发送 CAN。
- `WD_STATUS_VELOCITY_GUARD` 表示腿关节实际速度已越过保护速度，且计算力矩仍会继续推同方向，本 tick 已将该力矩清零。
- `WD_STATUS_COMMAND_CLIPPED` 表示最终本地电机命令被力矩/速度/slew 限幅；它和 PC 输入 sanitization 的 `WD_STATUS_SETPOINT_CLIPPED` 分开。
- powered bring-up 阶段默认运行硬限：最终力矩 `<= 1.0 Nm`，最终速度命令 `<= 2.0 rad/s`。

## 烧录和验证流程

当前 workspace 已经具备命令行构建/烧录入口：

- `robot-v4-firmware-sanpo_spine/firmware/sanpo_spine/Makefile`
- `.toolchains/arm-none-eabi/usr/bin/arm-none-eabi-gcc`
- `.toolchains/stlink/usr/bin/st-flash`

构建：

```bash
cd robot-v4-firmware-sanpo_spine/firmware/sanpo_spine
make release
```

产物：

```text
build/Release/sanpo_spine.elf
build/Release/sanpo_spine.bin
build/Release/sanpo_spine.hex
build/Release/sanpo_spine.map
```

当前已验证 `make release` 可以通过，内存占用约：

```text
CCMRAM: 48 KB / 64 KB
RAM:    91648 B / 128 KB
FLASH:  146636 B / 192 KB
```

烧录：

```bash
cd robot-v4-firmware-sanpo_spine/firmware/sanpo_spine
make flash-stlink
```

2026-07-09 实测烧录成功：

```text
Found 1 stlink programmers
version: V2J46S7
chipid: 0x0413
descr: F4xx
write size: 146636 bytes
Flash written and verified
```

烧录日志提示：

```text
NRST is not connected
```

这表示 STLINK 的复位脚没有接上。烧录完成后建议按板上 reset 或重新上电，再做 USB dry-run 验证。

本轮实测：`make flash-stlink` 后如果不物理 reset，`/dev/ttyACM*` 可能能打开但没有 feedback。按 reset 或重插 MCU USB、等 `/dev/ttyACM*` 时间戳更新后再测，可以恢复新固件运行。

等价底层命令：

```bash
LD_LIBRARY_PATH="/home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH" \
  /home/gu/wheel_dog_pc_mcu/.toolchains/stlink/usr/bin/st-flash --reset write build/Release/sanpo_spine.bin 0x08000000
```

非破坏性探测：

```bash
cd robot-v4-firmware-sanpo_spine/firmware/sanpo_spine
make probe-stlink
```

2026-07-09 实测主机已经能枚举到一个 STLINK/V2：

```text
Bus 001 Device 008: ID 0483:3748 STMicroelectronics ST-LINK/V2
```

早期遇到过用户没有 USB 写权限，烧录失败在打开 USB 阶段，尚未写入 flash：

```text
libusb couldn't open USB device /dev/bus/usb/001/008, errno=13
libusb requires write access to USB device nodes
```

当前已经修复过权限；如果重新插拔 STLINK 后再次遇到该错误，可以临时修复：

```bash
sudo chmod a+rw /dev/bus/usb/001/008
```

如果重新插拔 STLINK，`001/008` 这个编号可能变化，需要重新 `lsusb` 确认。

永久修复权限：

```bash
sudo cp /home/gu/wheel_dog_pc_mcu/.toolchains/stlink/lib/udev/rules.d/49-stlink*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

执行后拔插 STLINK，再运行：

```bash
cd robot-v4-firmware-sanpo_spine/firmware/sanpo_spine
make probe-stlink
```

正式烧录前需要确认：

- STLINK 已连接到正确的 SWD 口
- 开发板已供电
- 如果板上两颗 STM32F407 都参与，需要分别确认刷哪一颗

保留原始回退固件：

```text
Release/sanpo_robot_spine_board_firmware-v4-latest.bin
```

烧录后先不要接电机动力，先跑 PC dry-run：

```bash
python3 wheeldog/tools/wd_mcu_dryrun.py --port /dev/ttyACM0 --duration 5
```

期望输出应包含：

```text
status=dry-run|active
hz=1000.0 或接近 1000
last_setpoint 持续增长
timeout 不应出现
```

2026-07-09 静态 `wdControlTask` 版本在已烧录目标 `/dev/ttyACM0` 实测：

```text
feedback seq=1 last_setpoint=2 age=1ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
feedback seq=283 last_setpoint=283 age=18ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
```

同机同时存在 `/dev/ttyACM1`，但本轮 STLINK 只确认烧录到一块目标板；`ttyACM1` 无 feedback，后续如果要刷第二块 MCU，需要单独确认 STLINK/SWD 连接对象。

注意：USB CDC 的 `SET_LINE_CODING` 现在只会对常见 CAN bitrate 触发 CAN 重配置，并且 UART/CAN line-coding 更新失败不会再进入 `Error_Handler()`。PC dry-run 用 `921600` 打开虚拟串口时不应把 MCU 停住。旧透明转发入口仍保留。

2026-07-09 增加第一版只读 CAN feedback 接入：

- WDP4 未激活时，CAN RX 仍走旧透明转发回调。
- WDP4 激活后，CAN RX 中断只读取 CAN 帧并放入小环形队列。
- 1 kHz `wdControlTask` 从队列中解码 RobStride `COMM_OPERATION_STATUS (0x02)` 状态帧。
- 暂定映射：CAN1/CAN2 分别对应 bus index 0/1，即当前已烧录 MCU 只覆盖前 8 个电机；若同固件刷第二块 MCU，需要编译时配置 `WD_CAN_BUS_BASE=2`。
- 每路 CAN 的电机 ID `1/2/3/4` 映射到该 bus 的 4 个 control index，其中 `4` 是 RS01 轮电机，其余为 RS06。
- 第一次收到某个电机反馈时捕获 `startup_motor_position`，反馈给 PC 的 `q` 为 `sign * (raw_position - startup_motor_position)`；当前 sign 仍为全 `+1` 临时表。
- 当前仍不主动发送任何 CAN 请求/控制帧；如果电机不自动上报状态，`online_mask` 会保持 0，下一步再讨论安全的只读轮询/零命令触发策略。
- `wd_mcu_dryrun.py` 增加 `--print-joints`，可打印在线电机的 `q/dq/tau/temp`。

2026-07-09 只读 CAN feedback 版本已烧录到 `/dev/ttyACM0` 对应 MCU，reset 后用默认 `921600` dry-run 复测通过：

```text
feedback seq=1 last_setpoint=2 age=1ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
feedback seq=280 last_setpoint=281 age=0ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
```

当前电机已接线但未接动力电，所以 `online=0x0000` 符合预期。

2026-07-09 safety 状态补强版已烧录并 reset 后复测通过。第一包可能因为 reset 后尚未收到 setpoint 带 `timeout`，后续应恢复：

```text
feedback seq=27 last_setpoint=27 age=17ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
feedback seq=281 last_setpoint=281 age=13ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
```

本版仍未加入任何 `HAL_CAN_AddTxMessage` 调用，WDP4 active 路径不会发送 CAN 控制帧。

2026-07-09 增加 1 kHz 本地命令计算 dry-run：

- offline/fault/timeout/estop 时，内部 `motor_torque_cmd_nm[]` 和 `motor_velocity_cmd_radps[]` 均清零。
- 腿关节复刻 PC `PublishMotorCommands` 的 PD 计算：`kp*(q_des-q)+kd*(dq_des-dq)+tau_ff`，再过软限位、每关节力矩限幅、bring-up 全局力矩限幅、slew-rate、超速同向力矩清零。
- 轮关节只接受速度命令 `dq_des`，过速度限幅后写入内部 `motor_velocity_cmd_radps[]`。
- 这一步仍只计算不发送；内部命令数组尚未接到 CAN 写帧。

本版第一次烧录在擦除完成后的 flash-loader 写入阶段失败；手动 reset 后重试同一 `sanpo_spine.bin` 成功，随后 reset 并 dry-run 复测通过：

```text
feedback seq=27 last_setpoint=27 age=16ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
feedback seq=281 last_setpoint=281 age=11ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
```

无动力电下 `online=0x0000` 仍符合预期；没有出现 `command-clipped`、`velocity-guard`、`torque-slew-limited`。

2026-07-09 powered dry-run 观察：

```text
feedback seq=1 last_setpoint=2 age=3ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
feedback seq=386 last_setpoint=386 age=18ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
```

结论：动力电已接入时，当前电机/总线不会自动给出 MCU 能解码的 `COMM_OPERATION_STATUS (0x02)` feedback。下一步若要拿到 feedback，需要增加受控 CAN TX 触发/轮询；在此之前先烧录 `2.0 rad/s` 速度硬限版。

同一轮尝试烧录 `2.0 rad/s` 速度硬限版时，ST-LINK 设备号已变为 `Bus 001 Device 050`，权限再次不足，烧录未开始、未擦写：

```text
libusb couldn't open USB device /dev/bus/usb/001/050, errno=13
Couldn't find any ST-Link devices
```

随后将 ST-LINK 插回原端口并修复权限，设备变为 `Bus 001 Device 054`；`1.0 Nm / 2.0 rad/s` 限幅版烧录成功，手动 reset 后 powered dry-run 复测通过：

```text
feedback seq=1 last_setpoint=2 age=2ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
feedback seq=385 last_setpoint=385 age=20ms hz=1000.0 status=dry-run|active online=0x0000 fault=0x0000
```

当前实物只接一路四个电机：`ID1/ID2/ID3` 是 RS06 力矩模式，最终力矩硬限 `1.0 Nm`；`ID4` 是 RS01 速度模式，最终速度硬限 `2.0 rad/s`。

下一步加入显式门控的只读轮询：

- 默认 dry-run 仍不发送任何 CAN 帧。
- PC 工具必须加 `--poll-readonly`，setpoint 才会带 `WD_CONTROL_FLAG_READONLY_POLL`。
- `--poll-bus can1|can2` 显式选择当前已烧录 MCU 的 CAN1 或 CAN2；CAN2 通过 `WD_CONTROL_FLAG_READONLY_POLL_CAN2` 表示。
- MCU 收到该标志后只对 ID1~4 慢速轮询 RobStride `READ_PARAM`：`mech_pos (0x7019)` 和 `mech_vel (0x701B)`。
- 如果 PC 停止发送、命令超时、estop、或 setpoint 取消 `READONLY_POLL`，MCU 会停止轮询并 abort 未完成 TX mailbox。
- 该路径不发 `enable`、不发 `start`、不发 `iq_ref`、不发 `spd_ref`，也不改变电机运行模式。
- 读取到 position/velocity 后只更新 `q/dq/online`；`tau/temp` 仍等待后续 `COMM_OPERATION_STATUS (0x02)` 或更完整的反馈策略。

第一次 `--poll-readonly --poll-bus can1` 短测结果：

```text
status=dry-run|active|readonly-poll|can-tx-error online=0x0000
```

结论：CAN1 没有 ACK 或没有可解析回包。随后发送默认 dry-run 清掉轮询标志；下一版加入 timeout 保护和 CAN1/CAN2 显式选择后再继续。

随后 CAN2 只读轮询短测：

```text
status=dry-run|active|readonly-poll online=0x0000
```

CAN2 没有 `can-tx-error`，但仍没有 online。为区分“没有回包”和“回包解析失败”，feedback payload 尾部追加 CAN 诊断计数：`can_rx_frames/can_rx_bad_frames/can_rx_overflows/can_tx_errors`，PC dry-run 输出显示为 `canrx/canbad/cantxerr`。

原始帧诊断显示 READ_PARAM 回包示例：

```text
last_can bus=1 dlc=8 id=0x110001fd data=19700000...
```

即 `comm=0x11`、`motor_id` 在 CAN ID bits 8..15，低 8 位是 host id `0xfd`。MCU READ_PARAM 解析已按该格式修正。

修正 READ_PARAM 的 `motor_id` 解析后，CAN2 只读轮询复测通过：

```text
status=dry-run|active|readonly-poll online=0x00f0 fault=0x0000 canbad=0 cantxerr=0
last_can bus=1 dlc=8 id=0x110003fd data=19700000...
```

这说明当前接上的一路四个电机实际在已烧录 MCU 的 `CAN2` 上。按当前固件映射规则，`CAN1/CAN2` 分别对应 bus index `0/1`，所以这一路 `ID1/ID2/ID3/ID4` 暂时显示为逻辑关节 `4/5/6/7`，不是 `0/1/2/3`。这只是当前测试映射，不代表最终整机索引方案。

本次 CAN2 测试仍是只读 `READ_PARAM`：

- 没有发送 `enable/start/iq_ref/spd_ref`。
- 没有进入电机使能或闭环控制。
- `tau/temp` 仍为 0，因为当前只读轮询只读 `mech_pos/mech_vel`。
- 随后已发送默认 dry-run，撤掉 `READONLY_POLL`，MCU 回到 `status=dry-run|active online=0x0000`。

10 秒 CAN2 只读稳定性测试结果：

```text
status=dry-run|active|readonly-poll online=0x00f0 fault=0x0000 canbad=0 cantxerr=0
joints 4/5/6: q≈0 dq≈0 tau=0 T=0
joints 7: q≈0 dq≈-0.06..0.06 tau=0 T=0
```

测试结束后再次发送默认 dry-run，`READONLY_POLL` 已撤掉，MCU 回到不主动发 CAN 的普通 dry-run 状态：

```text
status=dry-run|active online=0x0000 fault=0x0000
```

2026-07-09 增加台架映射版本：

- Makefile 显式加入 `WD_BENCH_REMAP_CAN2_TO_LOGICAL0 ?= 1`。
- 当前台架版本把已烧录 MCU 的物理 `CAN2 + ID1/ID2/ID3/ID4` 映射为 WDP4 逻辑关节 `0/1/2/3`。
- 对应模式仍是 `0/1/2 = RS06 力矩/PD通道`，`3 = RS01 速度通道`，继续满足前三个力矩硬限 `1.0 Nm`、最后一个轮速硬限 `2.0 rad/s` 的测试边界。
- 物理 `CAN1 + ID1..4` 在该台架映射下会被交换到逻辑关节 `4..7`，避免与 CAN2 台架电机冲突。
- PC dry-run 会打印 `bench-can2-remap` 状态位，表示当前固件启用了这个台架映射。
- `last_can bus=1` 仍表示物理 CAN2 收到回包；`online_mask=0x000f` 才表示逻辑关节 `0..3` 在线。
- 正式整机/生产映射必须把 `WD_BENCH_REMAP_CAN2_TO_LOGICAL0` 设为 `0`，恢复 `CAN1->0..3, CAN2->4..7` 的直接映射。
- 该修改只改变 feedback/setpoint 的逻辑索引归属；不增加 `enable/start/iq_ref/spd_ref`，不改变 dry-run 不使能的安全边界。

台架映射版本已烧录并验证：

```text
status=dry-run|active|bench-can2-remap online=0x0000 canrx=0
status=dry-run|active|readonly-poll|bench-can2-remap online=0x000f fault=0x0000 canbad=0 cantxerr=0
last_can bus=1 dlc=8 id=0x110004fd ...
joints 0/1/2: q≈0 dq≈0 tau=0 T=0
joints 3: q≈0 dq≈-0.07..0.04 tau=0 T=0
```

验证结束后已发送默认 dry-run，撤掉 `READONLY_POLL`：

```text
status=dry-run|active|bench-can2-remap online=0x0000 fault=0x0000
```

2026-07-09 live-capable 台架使能/控制映射留档：

- 默认 Makefile 仍是 `WD_ALLOW_LIVE_CAN_CONTROL ?= 0`，普通构建即使 PC 请求 live 也会置 `live-blocked`，不发 `enable/start/iq_ref/spd_ref`。
- 只有显式构建 `WD_ALLOW_LIVE_CAN_CONTROL=1`，且 PC 同时发送 `--live-control --enable-request --i-understand-live-can`，MCU 才允许进入 live 使能路径。
- live 和 `--poll-readonly` 被 PC 工具互斥；非 live 模式不允许携带非零关节命令。
- PC 工具已增加台架命令完整性检查：`--joint-torque` 只允许逻辑 `0/1/2` 且绝对值不超过 `1.0 Nm`；`--joint-velocity` 只允许逻辑 `3` 且绝对值不超过 `2.0 rad/s`。
- MCU live 路径仍会再次执行 safety sanitize、力矩限幅、速度限幅、力矩 slew-rate 限幅和速度保护；PC 侧检查只是提前拒绝明显错模式/错索引命令。
- live 请求进入后，MCU 先发送只读 `mech_pos/mech_vel` 预检帧刷新逻辑 `0..3` 反馈；四个逻辑电机在线且 fault 为 0 后，才进入 `run_mode -> i_limit -> enable -> start -> zero_command`。
- 任意命令超时、estop、live 请求撤销或反馈过期时，MCU 会请求 stop：先发零命令，再发 disable。

当前台架 live 映射和单位：

| WDP4 逻辑索引 | 物理总线/ID | 电机 | MCU 模式 | 真实写帧 | 单位/换算 | 当前硬限 |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | CAN2 ID1 | RS06 | `run_mode=3` current | `iq_ref (0x7006)` | `iq[A] = tau[Nm] / 1.1` | `|tau| <= 1.0 Nm`, `|iq| <= 0.909 A` |
| 1 | CAN2 ID2 | RS06 | `run_mode=3` current | `iq_ref (0x7006)` | `iq[A] = tau[Nm] / 1.1` | `|tau| <= 1.0 Nm`, `|iq| <= 0.909 A` |
| 2 | CAN2 ID3 | RS06 | `run_mode=3` current | `iq_ref (0x7006)` | `iq[A] = tau[Nm] / 1.1` | `|tau| <= 1.0 Nm`, `|iq| <= 0.909 A` |
| 3 | CAN2 ID4 | RS01 | `run_mode=2` speed | `spd_ref (0x700A)` | `spd_ref[rad/s] = dq_des[rad/s]` | `|dq_des| <= 2.0 rad/s` |

补充：

- `i_limit (0x7018)` 会在 enable 前写入，按 `1.0 Nm` 的运行时力矩硬限换算：RS06 为 `0.909 A`，RS01 为 `0.820 A`。
- `last_can bus=1` 仍代表物理 CAN2；在当前 `bench-can2-remap` 构建中，`online_mask=0x000f` 才代表逻辑 `0..3` 在线。
- 当前限位仍是 startup-relative 调试限位；最终版本必须改为校准绝对零点和绝对位置限位。
- 截至本条记录，真实 `enable/start/iq_ref/spd_ref` 控制帧尚未发送；发送前必须现场确认。

2026-07-09 零命令 live 使能实测：

- 已刷写 `WD_ALLOW_LIVE_CAN_CONTROL=1` 的 live-capable 固件。
- 刷写后普通 dry-run 正常：`hz=1000.0`，`status=dry-run|active|bench-can2-remap`，`canbad=0`，`cantxerr=0`。
- CAN2 只读验证正常：`online=0x000f`，`fault=0x0000`，`canbad=0`，`cantxerr=0`，逻辑 `0/1/2/3` 全在线。
- 用户现场确认后，执行零命令 live：

```text
status=active|bench-can2-remap|live-requested|live-active|live-enable-ready
online=0x000f fault=0x0000 canbad=0 cantxerr=0
```

- 测试期间没有发送非零命令；0/1/2 使用零 `iq_ref`，3 使用零 `spd_ref`。
- live 结束后已发送普通 dry-run 撤销 live，随后又做 CAN2 只读确认，最后再次撤回普通 dry-run。
- 当前收尾状态为普通 dry-run：

```text
status=dry-run|active|bench-can2-remap online=0x0000 fault=0x0000 canbad=0 cantxerr=0
```

必须注意：

- 零命令使能后，只读反馈显示逻辑 `0/1/2` 的 startup-relative `q` 出现约 `-1.317/-0.502/-1.153 rad` 偏移，虽然 `dq≈0`、`fault=0`。
- live 过程中的 operation-status 解码曾出现较大的 `q` 瞬态值；这说明当前 `READ_PARAM mech_pos` 与 `COMM_OPERATION_STATUS` 的位置参考/缩放/零点处理仍需复核。
- 在查清零点、反馈源和 enable/start 后的机械 settling 前，不应继续发送任何非零力矩或速度命令。
- 下一步应先做“只读/零命令/单电机”的反馈一致性排查：确认 `mech_pos` 与 operation-status 的单位和符号、启动零点何时记录、enable/start 是否会造成机械释放或位置跳变。

随后用户现场确认刚才的缓慢移动是实际机械 settling，继续做单电机低幅值验证：

1. 逻辑 `3` / 物理 `CAN2 ID4` / RS01 速度通道：
   - `dq_des=+0.2 rad/s`，q 从约 `0.011` 增加到约 `0.411 rad`，反馈 `dq≈0.08~0.10 rad/s`。
   - `dq_des=-0.2 rad/s`，q 从约 `0.513` 回退到约 `0.122 rad`。
   - 收尾只读确认 q 回到约 `0.005 rad`，`fault=0x0000`，`canbad=0`，`cantxerr=0`。
   - 结论：速度通道映射、符号和单位方向基本正确；低速响应低于命令值但同号、稳定。

2. 逻辑 `0` / 物理 `CAN2 ID1` / RS06 力矩通道：
   - `tau_ff=+0.05 Nm`，反馈 tau 约 `+0.035~+0.066 Nm`。
   - `tau_ff=-0.05 Nm`，反馈 tau 约 `-0.082~-0.022 Nm`。
   - 结论：ID1 的 `tau -> iq_ref` 映射、符号和单位量级正确。

3. 逻辑 `1` / 物理 `CAN2 ID2` / RS06 力矩通道：
   - `tau_ff=+0.05 Nm`，逻辑 1 反馈 tau 约 `+0.037~+0.106 Nm`。
   - 结论：ID2 的力矩通道命中正确。

4. 逻辑 `2` / 物理 `CAN2 ID3` / RS06 力矩通道：
   - `tau_ff=+0.05 Nm`，逻辑 2 反馈 tau 约 `+0.018~+0.059 Nm`。
   - 结论：ID3 的力矩通道命中正确。

最终收尾：

```text
status=dry-run|active|bench-can2-remap online=0x0000 fault=0x0000 canbad=0 cantxerr=0
```

当前台架结论：

- `CAN2 ID1/2/3 -> 逻辑 0/1/2 -> RS06 current/iq_ref` 已实测通过低幅值命令。
- `CAN2 ID4 -> 逻辑 3 -> RS01 speed/spd_ref` 已实测通过正反低速命令。
- 所有非零测试均在 `|tau|=0.05 Nm` 或 `|dq|=0.2 rad/s` 下完成，远低于当前硬限 `1.0 Nm / 2.0 rad/s`。
- 每次 live 后都已撤回普通 dry-run；最后状态为普通 dry-run。
- 后续可以进入更系统的低层框架下沉验证，但放大幅值/频率前仍需先解决 startup-relative 零点和 feedback 源一致性问题，并逐步做单电机、单方向、低幅值扩展。

2026-07-09 追加 `1.0 Nm` 单路力矩可视化验证：

- 用户观察到 `0.05 Nm` 时 ID1/ID2/ID3 的动作不明显，要求用 `1.0 Nm` 看清。测试仍保持当前硬限 `|tau| <= 1.0 Nm`，且每次只给一个逻辑关节非零命令，其他电机为零命令。
- 逻辑 `0` / 物理 `CAN2 ID1`：`tau_ff=+1.0 Nm`，主要响应在 joint 0，反馈 tau 约 `0.25~0.56 Nm`，q/dq 有明显变化。
- 逻辑 `1` / 物理 `CAN2 ID2`：`tau_ff=+1.0 Nm`，主要响应在 joint 1，反馈 tau 约 `0.50~0.71 Nm`，q/dq 有明显变化；出现过一次 `torque-slew-limited|command-clipped`，符合从 0 爬升到 1 Nm 的 slew-rate 限幅。
- 逻辑 `2` / 物理 `CAN2 ID3`：`tau_ff=+1.0 Nm`，主要响应在 joint 2，反馈 tau 约 `0.43~0.55 Nm`，q/dq 有明显变化。
- 三次 live 测试均无故障：`fault=0x0000`，`canbad=0`，`cantxerr=0`。
- 最终 CAN2 只读确认：`online=0x000f`，`fault=0x0000`，三个力矩关节 `dq=0`，最后撤回普通 dry-run：

```text
status=dry-run|active|bench-can2-remap online=0x0000 fault=0x0000 canbad=0 cantxerr=0
```

本轮结论：

- 从反馈看，逻辑 `0/1/2` 分别命中 `CAN2 ID1/ID2/ID3`，不是同一个电机在重复响应。
- 肉眼只看到一个电机更明显，可能来自机械耦合、支撑姿态、各关节负载不同或反馈位置零点在 enable 后发生 settling。
- 不建议直接三路同时 `1.0 Nm`；若要做“整条路一起动”，建议先用 `0.2 Nm` 三路同号同步，再按现场情况逐步放大。

2026-07-09 四路 CAN / 两块 MCU 版本：

- 台架单路 `WD_BENCH_REMAP_CAN2_TO_LOGICAL0=1` 已经改为非默认；四路版本使用直连映射 `WD_BENCH_REMAP_CAN2_TO_LOGICAL0=0`。
- Makefile 构建目录按配置分开，避免 `bus_base`/live/bench 宏切换后误刷旧 bin：
  - `build/Release/bus0_bench0_live1/`
  - `build/Release/bus2_bench0_live1/`
- 新增烧录目标：
  - `make flash-stlink-mcu-a`：第一块 MCU，`WD_CAN_BUS_BASE=0`，物理 CAN1/CAN2 映射到逻辑 bus0/bus1。
  - `make flash-stlink-mcu-b`：第二块 MCU，`WD_CAN_BUS_BASE=2`，物理 CAN1/CAN2 映射到逻辑 bus2/bus3，也就是整机的 CAN3/CAN4。
- live 允许范围从单路 4 电机改为本 MCU 本地 8 电机：
  - MCU A：逻辑 index `0..7`
  - MCU B：逻辑 index `8..15`
- 第二块 MCU 会在反馈状态里显示 `mcu-base2`，便于确认烧的是 `WD_CAN_BUS_BASE=2` 版本。
- PC dry-run 工具的命令校验已扩展到 16 电机：每组四个中 `0/1/2` 相位是力矩通道，`3` 相位是速度通道；仍然硬拒绝超过 `1.0 Nm` 或 `2.0 rad/s` 的命令。

四路逻辑映射：

| 整机逻辑 CAN | MCU | MCU 物理 CAN | WDP4 index | 电机模式 |
| --- | --- | --- | --- | --- |
| CAN1 / bus0 | MCU A (`bus_base=0`) | CAN1 | `0/1/2` RS06, `3` RS01 | `iq_ref`, `spd_ref` |
| CAN2 / bus1 | MCU A (`bus_base=0`) | CAN2 | `4/5/6` RS06, `7` RS01 | `iq_ref`, `spd_ref` |
| CAN3 / bus2 | MCU B (`bus_base=2`) | CAN1 | `8/9/10` RS06, `11` RS01 | `iq_ref`, `spd_ref` |
| CAN4 / bus3 | MCU B (`bus_base=2`) | CAN2 | `12/13/14` RS06, `15` RS01 | `iq_ref`, `spd_ref` |

2026-07-09 四路 live 零命令使能修复和实测：

先前四路 live enable 已经能进入 `live-active`，但 `COMM_OPERATION_STATUS`
里的 p/v 解码尚未完成单位、符号、零点验证，曾把大跳变的 `q` 写入控制反馈，
进而触发 startup-relative 软限位墙，产生接近限幅的纠偏力矩。该路径已经禁止进入控制计算：

- 控制用 `q/dq` 只由 `READ_PARAM mech_pos (0x7019)` 和 `mech_vel (0x701B)` 更新。
- `COMM_OPERATION_STATUS` 暂时只用于 `tau/temp/fault/online/mode_status`，不再覆盖 `q/dq`，也不再设置启动零点。
- `wd_control_feedback_healthy()` 现在要求：
  - 收到过 `mech_pos`，即 startup-relative 零点有效；
  - 收到过 `mech_vel`；
  - 最近一次 `READ_PARAM` 参数反馈未超过 `WD_CAN_FEEDBACK_STALE_MS`；
  - `fault_bits == 0`。
- live enable 阶段和 live-active 阶段都会持续穿插 `READ_PARAM`，避免 8 电机 enable 序列中反馈自然过期。

重新烧录：

- MCU A：`build/Release/bus0_bench0_live1/sanpo_spine.bin`
- MCU B：`build/Release/bus2_bench0_live1/sanpo_spine.bin`
- 两块 MCU 均已用 `st-flash --connect-under-reset --reset write ... 0x08000000` 写入并校验通过。
- 因 ST-LINK 提示 `NRST is not connected`，烧录后手动 reset 两块 MCU。

四路 READ_PARAM 只读复测：

```text
MCU A CAN1: online=0x000f fault=0x0000 canbad=0 cantxerr=0
MCU A CAN2: online=0x00f0 fault=0x0000 canbad=0 cantxerr=0
MCU B CAN1: online=0x0f00 fault=0x0000 canbad=0 cantxerr=0
MCU B CAN2: online=0xf000 fault=0x0000 canbad=0 cantxerr=0
```

真实 live 零命令使能复测：

- MCU A：零 `iq_ref/spd_ref`，进入 `active|live-requested|live-active|live-enable-ready`，
  `online=0x00ff`，`fault=0x0000`，`canbad=0`，`cantxerr=0`。
- MCU B：零 `iq_ref/spd_ref`，进入
  `active|live-requested|live-active|live-enable-ready|mcu-base2`，
  `online=0xff00`，`fault=0x0000`，`canbad=0`，`cantxerr=0`。
- 两次 live 零命令测试中未再出现
  `torque-slew-limited`、`command-clipped` 或 `velocity-guard`。
- 打印的 q 始终来自 READ_PARAM 的 startup-relative 路径，保持在零点附近；operation-status
  仍可显示 `tau/temp/fault`，但不参与 PD/限位计算。
- 每次 live 后都已发送普通 dry-run 撤销 live，使固件走零命令和 disable 停止路径。

当前安全边界仍保持：

- RS06 力矩/电流通道：`|tau| <= 1.0 Nm`，`tau -> iq_ref`。
- RS01 轮速通道：`|dq_des| <= 2.0 rad/s`，`dq_des -> spd_ref`。
- 当前位置限位仍是 startup-relative 调试限位；后续要替换为绝对位置限位。

2026-07-09 两块 MCU / 四路 CAN / 16 电机真实满限命令测试：

测试命令：

- MCU A / index `0..7`：`0/1/2/4/5/6` 下发 `+1.0 Nm`，`3/7` 下发 `+2.0 rad/s`。
- MCU B / index `8..15`：`8/9/10/12/13/14` 下发 `+1.0 Nm`，`11/15` 下发 `+2.0 rad/s`。
- A/B 两块 MCU 同时 live，持续约 5 秒，随后立刻用普通 dry-run 撤销 live。

结果：

```text
MCU A: actual_control_hz=1000.0, online=0x00ff, fault=0x0000, canbad=0
MCU B: actual_control_hz=1000.0, online=0xff00, fault=0x0000, canbad=0
```

- 用户现场确认所有电机均正常动作。
- 满限运动中出现过 `velocity-guard`、`torque-slew-limited`、`command-clipped`，符合当前安全层在达到速度保护阈值和 1 Nm 斜率/幅值限制时介入的预期。
- 测试结束后 A/B 均已撤回普通 dry-run，固件走零命令和 disable 停止路径。
- 后续只读 CAN 检查出现 `can-tx-error` 和 `online=0x0000`，用户确认原因是现场已经断开动力电，电机侧无响应；该现象不作为本次 live 控制异常。

当时的频率结论（已被 2026-07-10 的 500 Hz 调度替代）：

- `actual_control_hz=1000.0` 证明 MCU 本地控制任务以 1 kHz 跑。
- 当前 CAN live 命令发送器仍是每块 MCU 每 1 ms 发送 1 个本地电机命令，8 个本地电机 round-robin，单电机 CAN 写命令刷新约 `125 Hz`。
- 若最终目标是每个电机力矩/速度写帧都达到 `1000 Hz`，后续还需要单独改 CAN 调度和邮箱发送策略。

如果拔掉 PC 或停止发送 setpoint，约 `100 ms` 后应出现：

```text
status=...|timeout
```

## 2026-07-10：正式部署链路与每电机 500 Hz CAN 调度

本轮修复了两个此前互相独立的问题：

1. `rl_deploy` 真机后端原先仍启动旧 `wheeldog_bridge.py -> Robot16Interface`，
   PD 仍在 PC 计算；现在默认启动 `wheeldog_mcu_bridge.py`，以 50 Hz 向两块
   MCU 发送完整 16×5 setpoint，由 MCU 保持并在 1 kHz task 内计算 PD。
2. live CAN 从“每块 MCU 全局每毫秒 1 帧、8 电机轮转”改为“两路 CAN 并行，
   每路每毫秒 2 个到期电机”。每路四个电机交错为 2 ms 周期，目标写帧频率为
   每电机 500 Hz。

新增可验证性：

- bxCAN TX-complete 中断按电机记录 `queued/completed/deferred` 计数；不能再用
  `actual_control_hz=1000` 代替电机刷新率。
- PC bridge 每秒从 `completed` 差分计算 16 个电机的真实完成频率；启动时只有
  所有电机都达到默认 `>=490 Hz` 且没有 2 ms deadline miss，才对 C++ 置 ready；运行中低于阈值会撤回
  live、进入 dry-run/disable 清理。
- 新增纯 C 调度测试与双伪串口端到端测试，覆盖 500 Hz 计数、CRC/分包、双 MCU
  bus-base 冲突、共享内存布局和 16 路频率汇总。
- 1 kHz task 改用 `vTaskDelayUntil`；USB 收到的 352-byte setpoint 先发布到 pending
  buffer，再由控制 task 原子取走，避免 USB 中断与 PD 计算并发造成撕裂。
- C++/Python 共享内存命令与反馈改用奇偶 seqlock；bridge 监控 C++ setpoint
  publisher 和父进程，控制进程卡死或退出时不再无限保持最后命令。

1 Mbps CAN 预算（每路）：四电机×500 Hz = 2000 command frame/s。若每个写帧都
返回一个 8-byte operation-status，经典 CAN 扩展帧按 131 bit 无填充、约 160 bit
最坏填充估算，命令+反馈约占 52.4%~64.0%；加现有 READ_PARAM 后仍有余量。
四电机×1000 Hz 在带逐帧反馈时会超过 1 Mbps，因此 500 Hz 是当前拓扑更合理的
目标。

仍未闭环的关键点：以上证明的是 torque/speed 写帧 500 Hz。控制用 `q/dq` 目前
仍来自低频 READ_PARAM；operation-status 的 p/v 曾出现跳变，尚未重新启用。只有
确认每个 `iq_ref/spd_ref` 写帧都会回 0x02，并完成 RS06/RS01 p/v 的单位、符号、
零点和连续性验证后，才能称为“新鲜反馈参与的 500 Hz PD 闭环”。

## 2026-07-10：500 Hz 新鲜反馈资格确认链路（待上电实测）

上节所列缺口已在软件中改成 fail-safe 的自动资格确认，不再假定每条
`iq_ref/spd_ref` 都会产生 `COMM_OPERATION_STATUS (0x02)`：

1. 每个电机独立累计 `operation_status_rx_count`，并在 200 ms 窗口内测量反馈频率；
   固件阈值默认 `>=490 Hz`，最近反馈超过 `5 ms` 即失效。
2. 0x02 的 p/v 先只进入候选缓冲。低频 `READ_PARAM mech_pos/mech_vel` 到达时，
   分别与最近 10 ms 内的候选值交叉比较；位置误差默认 `<=0.05 rad`、速度误差
   `<=0.20 rad/s`，各连续三次吻合才通过。
3. 上述容差是可改的“资格参数”，不是对 RobStride 协议的猜测。台架首轮应记录
   `fast_position_error_rad[]`、`fast_velocity_error_radps[]` 再决定是否收紧；不得为了
   让错误缩放通过而盲目放宽。
4. 静止零附近的两路错误缩放也可能碰巧吻合，因此还要求每电机在零命令阶段由
   人工转动产生至少 `0.10 rad` 的 READ_PARAM 位置跨度，并观测到至少
   `0.50 rad/s` 的速度。分别用 `fast_position_excited_mask` 和
   `fast_velocity_excited_mask` 报告；没有激励不能通过。激励证据仅在本次 MCU
   上电周期内保留；每次重新上电必须重做。之后再次进入 live 仍会重测 200 ms
   反馈频率并重新取得三次 READ_PARAM 匹配，但不要求重复人工转动。
5. live enable 完成后，500 Hz CAN 调度先固定发送零 `iq_ref/spd_ref`。八个本地电机
   全部通过前，`fast-feedback-unqualified` 保持置位，MCU 不允许非零 PD/轮速命令。
6. 通过后置 `fast-feedback-ready`。PD 计算按每电机 feedback generation 触发：每份
   新 q/dq 只计算一次；1 kHz task 中间的无新反馈 tick 保持上一条命令，不重复计算。
7. 资格通过后，反馈变慢/中断、频率资格丢失或 motor fault 进入运行时安全链。
   无共同采样时间戳的 READ_PARAM/0x02 动态差异只保留为诊断，不撤销已建立资格。
8. PC bridge 再独立对 `live_command_tx_completed` 与
   `operation_status_rx_count` 做 1 秒差分。16 路 TX 和反馈都达到默认 `>=490 Hz`、
   两块 MCU 的 fast-valid mask 合并为 `0xffff` 后，才向 C++ 发布 ready 和非零 setpoint。

无硬件测试现已覆盖：带有效运动跨度且匹配的 500 Hz 通过，静止零信号拒绝，
125 Hz 拒绝，无 0x02 拒绝，首次资格阶段 p/v 不匹配拒绝，反馈超过 5 ms
不再使用旧样本；双伪
CDC/双 MCU 端到端测试汇总 16 路 TX/RX 约 500 Hz。两套 release 固件均完成链接，
但这不是硬件结论。

台架上电后的判定非常明确：

- 若 `operation_status_rx_count` 不增长，说明当前写参数不会返回 0x02，必须另选
  500 Hz 反馈读取/广播方式，不能启用 PD。
- 若计数只有约 125 Hz，资格会失败，必须先解决驱动器反馈触发/轮询方式。
- 若频率约 500 Hz 但 fast-valid bit 不置位，查看逐电机 p/v error，确认 0x02 的
  缩放、零点或字段语义；不能跳过交叉验证。
- 只有 PC 共享内存同时显示 `motor_command_hz[0..15] >= 490`、
  `motor_feedback_hz[0..15] >= 490`、`fast_feedback_valid_mask=0xffff`，才能称为
  每电机 500 Hz 新鲜反馈控制链路通过。

当前仍使用用户确认的台架 startup-relative 限位。固件新增
`WD_STATUS_BENCH_RELATIVE_LIMITS` 常驻提醒；机械装配前必须完成
`迁移说明/MECHANICAL_ASSEMBLY_BLOCKER_ABSOLUTE_LIMITS.md`，否则禁止进入装配测试。

## 2026-07-10：主动资格激励与双 MCU 同时刻观测

现场补充确认：同一物理 USB 在 Linux 枚举为两个 CDC 设备，因此 PC bridge 的双端口
模型正确。裸轴在零命令下无法产生可见运动，至少需要 RS06 `0.6 Nm`、RS01
`1 rad/s` 克服摩擦。相应改动如下：

- 新增 `WD_CONTROL_FLAG_QUALIFICATION_EXCITATION`。显式台架工具和正式 bridge 的
  verified-enable 启动状态机可以设置；正式 bridge 也严格保证全16台中一次只有
  一台非零，完成后立即切下一台或清零。
- 整个 16 电机 setpoint 中最多一个非零资格命令。RS06 只接受纯 `tau_ff` 且
  `|tau|<=0.6 Nm`；RS01 只接受 `|dq_des|<=1 rad/s`。
- RS06 仍受速度保护，并增加 `±0.5 rad` startup-relative 资格行程保护。命令超限、
  混入 PD 参数或同时出现两个非零电机都会保持零输出并报错。
- `wd_mcu_dryrun.py --qualification-excitation` 要求一次只指定一个电机，值必须恰好
  为 `±0.6 Nm` 或 `±1 rad/s`，退出前自动发送 zero/dry-run cleanup。

原 PC bridge 将两个独立 CDC 的最新 feedback 直接拼接，不能保证 RL 的 16 电机观测
属于同一时刻；跨 MCU 最坏可接近一个 20 ms 上报周期。现已改成序号屏障：

1. PC 仍以 50 Hz 向两块 MCU 发送相同 setpoint sequence。
2. MCU 的 1 kHz task 在应用新 sequence 前先排空 CAN RX，然后锁存一个 observation。
3. 每电机携带 operation-status sample age；已通过快速反馈资格时，用
   `q_snapshot = q_latest + dq_latest * age` 将位置外推到 MCU 快照 tick。
4. firmware feedback 新增 `observation_seq/time/max_age/flags` 和 16 个 sample age。
5. PC 只有在 A/B 两包 `observation_seq` 完全相同、最大 sample age 默认 `<=5 ms`、
   两包到达偏差默认 `<=5 ms` 时，才一次性发布 16 电机共享内存快照。
6. 共享内存版本升为 4，新增 observation sequence、最大年龄、到达偏差、逐电机年龄
   和丢失 epoch 计数。不同序号不会产生半新半旧的 RL 观测。
7. ready 后连续 `100 ms` 没有合格的双 MCU 整帧，bridge 报故障并执行
   zero/dry-run cleanup。

这个方案提供的是可测的“同 sequence、年龄和到达偏差有界”，不是声称两个 MCU
具有物理同一时钟。若台架证明 5 ms 仍无法满足策略，需要增加 MCU 共用同步 GPIO、
硬件定时器触发或正式时钟同步协议，不能只靠 USB 到达时间推断绝对同时。

## 2026-07-10：A/B 实机烧录与无动力双 USB 复测

现场拓扑确认：同一 QinHeng USB HUB 下枚举两个 STM32 CDC，当前映射为
`/dev/ttyACM0 = MCU A/bus0`、`/dev/ttyACM1 = MCU B/bus2`。ST-LINK/V2 连接
STM32F407（512 KiB Flash）。

烧录结果：

- MCU A：`bus0_bench0_live1/sanpo_spine.bin`，SHA-256
  `e3a4d5977882e7861fcfe6da8d90b06091236d3837c091fe0e557813979c6f55`。
- MCU B：`bus2_bench0_live1/sanpo_spine.bin`，SHA-256
  `90c8a57c7da08d58f7d92d23ce12b85ac7af4571f1dc4b46878cc89f6fef0679`。
- 两次 ST-LINK 均报告 `Flash written and verified`；随后分别人工 reset。

单 MCU 无动力 dry-run：A/B 均为 `actual_control_hz=1000`，A 正确不带
`mcu-base2`，B 正确带 `mcu-base2`；两者都置 `bench-relative-limits`，新
`observation_seq` 随 setpoint 递增。无动力时 `online=0`、
`observation_max_sample_age_ms=65535` 是预期结果。

真实双 CDC bridge 连续 2.2 秒 dry-run：

```text
epochs=111, observation_seq=2..112
bridge_hz=49.99
dropped_observation_epochs=0
arrival_skew_us: average=2069.5, max=3167
bridge fault=false, exit=0
```

实测同时暴露并修复两个 PC 启动竞态：

1. bridge 在第一条 setpoint 尚未由 MCU 应用时，会把上电遗留的 command-timeout
   立即判故障；现只对身份建立后的最初 250 ms 放宽该单一状态。
2. dry-run ready 后错误启用了 C++ setpoint publisher watchdog，独立诊断约 0.5 秒
   后会误退出；现仅在 `enable` 后启用该 watchdog。

当前动力电关闭，因此尚未执行 CAN online、主动资格激励、operation-status
500 Hz、PD 或带真实电机样本的 observation age 验收。

## 2026-07-10：16 电机带动力 500 Hz 闭环验收

最终烧录二进制：

- MCU A：`f67500065fd9fafa8f9c005b51508f9e83dab5cc270799db37157c72a84927b4`
- MCU B：`becf9f7e59753a2340cdd3845d2c790394596394f7356bf1b99821e204d9c04b`

带动力冷启动首先暴露了两个协议问题，已修复并重刷 A/B：

1. `GET_DEVICE_ID` 应发 DLC=2 的 `01 00`，原实现误发 DLC=8；同时 live
   启动前不得要求“先有反馈才发初始化查询”。现在 MCU 会先发设备查询，
   再进入 `READ_PARAM` 与 live 资格流程。断电重启后 16/16 电机均可自动发现。
2. RobStride 运控 0x02 缩放被旧配置误用。已按手册修正为 RS06
   `p=±4π, v=±50 rad/s, tau=±36 Nm`，RS01
   `p=±4π, v=±44 rad/s, tau=±17 Nm`。修正后 16 电机快慢位置交叉误差
   约 `0..0.002 rad`，不再出现旧版的数十 rad 假误差。

每台电机均用专用资格模式单独激励：RS06 仅 `+0.6 Nm`，RS01 仅
`+1 rad/s`，严禁同时激励多台。实测最高速度约 `1.68 rad/s`，低于固件
`2 rad/s` 保护值和现场 `4 rad/s` 硬性上限。最终快速反馈资格掩码：

```text
MCU A: fast/pex/vex = 0x00ff
MCU B: fast/pex/vex = 0xff00
combined fast feedback = 0xffff
fault = 0x0000, canbad = 0, CAN TX error/deadline miss = 0
```

当时快慢反馈交叉验证曾采用“单次不一致仅记数，连续三次不一致才撤销”；后续
动态实测证明没有共同采样时间戳时仍会误停。当前最终规则见下文第五次整体运行：
首次资格仍须连续三次一致，资格建立后动态差异只记录、不撤销。

使用正式 `wheeldog_mcu_bridge.py` 、双 CDC 同时 live、所有上层命令为零，
稳定运行 4 s 后的验收结果：

```text
MCU control task:       A=1000.0 Hz, B=1000.0 Hz
motor command TX:       16/16 = 500.0 Hz
motor operation status: 16/16 = 500.0 Hz
online/enabled/fresh:   0xffff / 0xffff / 0xffff
fast/fault:             0xffff / 0x0000
PC bridge:              50.0 Hz
observation max age:    1 ms
dual-MCU arrival skew:  0 us in this run, limit 5 ms
dropped observation:    0
bridge cleanup exit:    0
```

因此当前证据已不是“1 kHz 任务在跑”或“只有 CAN 写帧 500 Hz”，而是
16 个电机各自有 500 Hz 已完成命令和 500 Hz 新鲜反馈，MCU 本地 PD 仅由新反馈
generation 触发，并且 PC/RL 只接收同 sequence 的 16 电机整帧。

尚未关闭的硬件阻断项：

- 资格激励的 `±0.5 rad` 是“停止继续向外施加力矩”阈值，不是能吸收惯性的硬
  位置锁；裸轴在撤力后曾滑行至约 `0.75 rad`。
- 当前 `q=raw-startup_raw`，没有向电机持久写零点。这消除了本次上电的任意台架
  姿态偏置，但不能替代机械绝对标零。
- 电机是单圈绝对值编码器。本次运动范围内未经过跨边界，因此尚未验证回绕周期
  和跨边界时的连续化策略。在用户确认实际回绕语义并完成机械装配零点前，
  不猜测周期、不持久改写电机零位。

## 2026-07-10：IMU 与无动力 run.sh 整链路预演

当前 HiPNUC HI91 在断电重连后的实际上电波特率为 `115200`，不是历史日志中的
`921600`。在 921600 下原始字节几乎只有 `00/80`；在 115200 下恢复
`5a a5` 帧头并稳定解析 HI91 约 `139~140 Hz`。静止测量：

```text
rpy 约 [-1.36, 0.14, 0.00] deg
acc 约 [0.23, 0.02, 9.78] m/s^2
mapped acc unit 约 [0.024, 0.002, 1.000]
dot(acc_unit, -projected_gravity) = 0.999
axis map = x,y,z, determinant = +1
```

无动力、禁止电机使能的 `run.sh` 预演已验证：当前目录实机二进制、ONNX
57→16 模型、IMU、双 CDC WDP4 dry-run、200 Hz 高层 tick 和 50 Hz PC→MCU
setpoint 均可同时启动。动力电关闭时 `observation_age=65535` 和显示的电机最后快照
仅是 dry-run 诊断，不是可供 RL 动作使用的新鲜反馈。

同轮安全补强：

- PC 出口额外夹紧 `tau_ff` 到 `±1 Nm`、所有 `dq_des` 到 `±2 rad/s`；MCU
  仍是最终裁决者。
- 运行中 IMU 超过 250 ms 无新帧，或电机整帧反馈超过 150 ms、bridge
  fault、任一电机 offline/fault/unqualified、观测 age/skew 超限，C++ 将锁存
  零命令直到进程重启，不会在数据恢复时自动重新加力。
- 实机启动默认强制 ONNX 模型存在，并校验 `build_local` 必须来自当前源码目录。

## 2026-07-10：首次 z→c 带动力整链路结果

首次正式运行先进入 StandUp，约 18 s 后切换 RL。StandUp 阶段无 bridge
故障；RL 运行约 2 s 后 MCU A 报 `LIVE_CONTROL_BLOCKED`，bridge 立即退出并
清回 dry-run，C++ 运行时锁存同时把16路命令强制为零。旧 bridge 只打印第一个
命中的状态名，因而当时没有保留 `LIVE_SAFETY_STOP`、具体电机掩码或快速反馈
误差，不能从旧日志继续猜测是资格撤销还是电机健康条件丢失。

同一次日志确认进入 RL 时 `HR_HipY=+0.7 rad`，训练/部署默认值为
`-0.5 rad`，偏差约 `1.2 rad`。把该实测观测离线输入本次完全相同的 ONNX
（SHA256 `9c227c64...`）并递推 `last_action`，可以复现轮动作迅速饱和；因此
此时继续 RL 不具备控制意义，尽管 PC/MCU 的 `2 rad/s` 限幅挡住了原始输出。

修补项：

- 真机禁止 Idle 直接进入 RL，必须先经过 StandUp。
- 临时相对模式的零点从“每次 MCU boot 首次反馈”改为“每次新的 live 控制会话、
  电机使能之前重新采集当前位置”。这样 MCU 持续由 USB 供电而动力电/PC 程序
  重启时不会复用旧相对零点；未来 calibrated-absolute 模式不走此路径。
- StandUp→RL 要求12个腿关节距默认目标均不超过 `0.25 rad`，且16个电机
  速度均不超过 `0.5 rad/s`；否则留在 StandUp 并打印最差关节。
- bridge 同时报告全部 critical 状态，而不是只报告第一个；故障前完整打印
  MCU 状态、掩码、CAN 计数、逐电机 qualifier rate/position/velocity error。
- firmware feedback 尾部新增24字节锁存快照，整个 WDP4 feedback packet 从
  `1000` 增为 `1024` 字节。快照区分 `fast-feedback-lost`、
  `command-feedback-unhealthy`、`command-motor-not-enabled`，并记录触发电机、
  时间、当时 fast-valid/fault mask；下一次 live entry 才清零。新增带宽仅
  `2.4 kB/s`（双 MCU、50 Hz），不影响现有 USB 余量。

该 firmware 扩展必须同时刷入 MCU A/B 后，才可进行下一次带动力复现。

第二次整体启动在进入 Idle 前安全退出。两块 MCU 与16路 operation-status 都是
`500 Hz`，快慢反馈误差很小，但 MCU reset 后 `pex/vex=0`；旧正式 bridge 只等待
已经存在的资格，不会主动建立资格，最终 setup timeout。现已补齐 verified-enable
自动状态机：

1. 先以零命令完成双 MCU identity、16电机发现与 enable。
2. 仅当两块 MCU 都 online/enabled、无 fault、控制任务 `>=950 Hz` 后开始激励。
3. 按逻辑电机0..15严格串行；RS06固定 `+0.6 Nm`，RS01固定 `+1 rad/s`。
4. 每台必须同时获得 position/velocity excitation bit，单台默认3 s超时；失败立即
   zero/dry-run cleanup，不继续下一台。
5. 全部完成后清除 qualification flag，再要求 `fast=0xffff`、16路 TX/RX
   `>=490 Hz` 和同 sequence 观测，最后才向 C++ 报 ready。

伪双 CDC 端到端测试已从 `fast/pex/vex=0` 自动推进到 `0xffff`，并验证任何 setpoint
最多只有一台资格电机非零。硬件实测仍必须在用户规定的 `1 Nm / 4 rad/s` 以下归档。

第三次整体运行完成自动资格、Idle、StandUp，并且进入 RL 时12个腿关节全部等于
训练默认站姿。RL 约5 s 后 MCU B 安全停止，新锁存快照给出确定原因：

```text
reason = fast-feedback-lost
first invalid fast bit = motor 15 (snapshot fast=0x7f00)
motor 15 position cross-check error = 25.1295 rad ~= 8*pi
motor fault mask = 0
CAN bad/overflow/TX error = 0/0/0
operation-status rate = 500 Hz
```

电机是单圈绝对编码器，0x02 位置映射区间为 `[-4*pi,+4*pi]`；轮电机经过边界时，
operation-status 与异步 READ_PARAM 可分别落在区间两端。旧代码直接相减，把同一角度
误判为约 `8*pi` 的尺度失配。修复：

- 快慢反馈交叉验证改用周期 `8*pi` 的最短环形位置误差；真实小误差仍按原
  `0.05 rad` 阈值和连续3次规则判断。
- 500 Hz operation-status 位置在 MCU 内按相邻样本最短增量连续展开，再减去本次
  live 会话零点；跨边界时 `q` 连续，不再跳变 `8*pi`。
- host test 新增 `+4*pi -> -4*pi` 边界等价和连续增量用例。机械装配后仍必须实施
  绝对关节零点/限位；环形展开只修复编码连续性，不能替代机械标定。

第四次整体运行再次通过自动资格、Idle、StandUp，进入 RL 时12个腿关节仍准确落在
训练默认站姿；本次未再出现 `8*pi` 位置交叉误差。约16 s 后 MCU B 因10号腿电机
fast feedback 短时失去 freshness 而锁停：

```text
motor = 10
operation-status average = 500.0 Hz
position/velocity excitation = passed/passed
position error = 0.0030 rad
velocity error = 0.2224 rad/s
fault/CAN bad/RX overflow/TX error/deadline miss = 0/0/0/0/0
```

旧逻辑把任一 operation-status 年龄从 `5 ms` 变为 `6 ms` 直接升级为全机停机，无法
区分短 CAN 调度缺口与持续失联。更重要的是，反汇编确认 CDC TX 任务栈只有
`1024 B`，而旧版 `wd_control_usb_tx_poll()` 在栈上分配 `1008 B` feedback payload；
编译后的实际栈帧已经超过任务栈，存在越界破坏内存的确定风险。修复如下：

- feedback payload 移到单生产者静态 RAM；修复后该函数实际栈帧由 `>1052 B`
  降至约 `80 B`（含保存寄存器），消除 CDC 任务栈溢出。
- `5 ms` freshness 仍是控制硬门限：一旦超过，该电机立即发送零命令，绝不保持旧
  样本继续计算；反馈恢复且重新出现新 generation 后才恢复本地控制。
- 调试阶段只有失联持续到 `100 ms` 或反馈频率资格丢失，才升级为全机锁存停机；
  动态快慢交叉误差只诊断。生产机械装配前应根据风险测试重新收紧100 ms。
- 短缺口期间的 observation 不写入共享内存，RL 继续持有上一帧；持续异常由 MCU
  100 ms 锁停与 PC 100 ms 整帧超时兜底，因此不会向策略发布混时刻/陈旧电机帧。
- feedback 追加每电机最大 operation-status 帧间隔、锁停瞬间年龄，以及安全限幅/斜坡/
  位置墙/速度保护后的 `final_joint_torque_cmd_nm[]`。payload/packet 现为
  `1104/1120 B`；双 MCU 50 Hz feedback 为 `112.0 kB/s`。加上双向50 Hz
  setpoint 后总 WDP4 payload 约 `148.8 kB/s`（约 `1.19 Mb/s`，未计 USB framing），
  对12 Mb/s USB FS仍有充分余量，且频率仍是50 Hz而非把500 Hz搬到PC。

第五次整体运行在 CDC 栈修复和100 ms调试门限下稳定进入 RL，运行时间显著超过前一轮，
随后 MCU B 12号腿电机撤销 fast 资格：

```text
stop motor=12, feedback age at stop=0 ms
average operation-status=500.0 Hz
position error=0.0020 rad
velocity error=0.2636 rad/s
fault/CAN bad/overflow/TX error=0
```

因此该次不是100 ms持续失联；停机时反馈完全新鲜。直接原因是初始静态资格用的
`0.20 rad/s` 快慢速度一致门限也被用于动态运行，异步 `READ_PARAM` 和500 Hz运控帧
在电机加速时连续出现 `0.2636 rad/s` 差异，误撤销了已证明的缩放资格。修复为：

- 首次/每次 live 的缩放资格仍要求严格 `0.20 rad/s`，这是两条实际反馈通道的
  一致性检查，不是要求实际速度在短时间内跟上目标速度；
- 资格建立后，动态 READ_PARAM/0x02 位置和速度差继续写入诊断字段并累计不一致，
  但不再清除 match latch、不撤销资格、不触发停机；两条反馈没有共同采样时间戳，
  因此任何固定动态容差都不能作为可靠安全判据；
- 后续锁停原因细分为 `fast-feedback-stale`、`fast-rate-lost`、
  `fast-reference-lost`，同时保留总类 `fast-feedback-lost`；
- 先前约160 ms的 `max_gap` 来自八电机分阶段 enable 期间“已收到一次响应、但尚未
  开始全体周期命令”的启动空档，A/B都存在，不是RL期卡顿。现在首次全体 fast
  ready 时将最大间隔清零，后续字段只统计真正运行期。

2026-07-10 将力矩上限放开到2 Nm、速度配置放开到4 rad/s后的第一次整机测试
暴露了两个确定的安全漏洞：

- StandUp 期间多个腿关节反馈速度持续在 `3..4.3 rad/s`，但旧准入只检查单个
  200 Hz 时刻，最终在偶然低于0.5 rad/s的一帧进入RL。现在必须位姿/速度连续
  `1.0 s` 合格才允许进入。
- MCU原“速度限制”只限目标和同向力矩，实测锁存已出现逻辑电机6
  `-9.91 rad/s`，不等于实际超速保护。新固件用500 Hz operation-status新鲜速度
  检测超速：腿PD通道保守2 rad/s，轮速通道绝对上限4 rad/s；超限立即锁存
  `measured-overspeed`。腿速从1 rad/s起对“继续同向加速”的力矩做线性降额，到2 rad/s
  降为零；反向制动力矩不降额，因此低速仍可使用完整2 Nm克服静摩擦。
- 旧锁存停机分支需要等PC看到下一包FATAL后撤销live，MCU才执行zero/disable。
  新固件在锁存同一个1 kHz tick立即启动本地zero/disable，不再依赖USB往返。
- 该次最终故障为MCU B的13/14/15同时失去快反馈约100 ms，无CAN bad/overflow/
  TX error。现场还需结合“肉眼看到的疯转电机位置”判断是轮速策略饱和、腿PD
  振荡，还是CAN4/供电分支的联动问题，不能仅凭日志猜测。

安全固件后续整体测试中，16/16资格、500 Hz与1 s稳定准入都通过；进入RL约
十余秒后，MCU B锁存逻辑电机9 / `HL_HipY` 的 `measured-overspeed`，证明新实际
速度保护工作。旧日志中的 `velocity_at_stop=-1.802 rad/s` 是USB发包时已回落的快照，
不是触发样本；PC文案已更名为 `velocity_snapshot_after_stop`。

RL的旧2 s切入混合只限制初始阶段，混合结束后50 Hz策略腿目标仍可大幅跳变。
现在在混合/EMA之后永久增加腿目标变化率限制：实机默认 `8 rad/s`，即每个
50 Hz策略帧最大 `0.16 rad`；通过 `RL_LEG_GOAL_SLEW_RADPS` 可在受控调参时覆盖。
日志每秒输出最差腿关节的 `q/q_des/dq/error`与本帧被限速的腿关节数。

## 下一阶段

已完成：

1. WDP4 USB 协议、1 kHz MCU 控制任务、dry-run、基础安全标志和 50 Hz PC setpoint 链路。
2. `8.0 Nm` 轮通道等效峰值力矩硬限（约 `6.56 A` 的 RS01 `i_limit`）和
   `8.0 rad/s` 轮速硬限已经写入 MCU 运行配置；前三个 RS06 按力矩/PD通道处理，
   第四个 RS01 按速度通道处理。
3. 默认 dry-run 不发 CAN 控制帧；显式 `--poll-readonly` 才发 RobStride `READ_PARAM` 只读查询。
4. powered dry-run 已确认 `1000 Hz` 稳定；CAN2 只读轮询已确认当前一路四个电机在线，历史直接映射为逻辑关节 `4..7`，当前台架版本将其重映射为逻辑关节 `0..3`。
5. 双 MCU、四路 CAN、16 电机的命令 TX/反馈闭环已带动力实测为每电机 `500.0 Hz`，并通过同 sequence RL 观测验收。

后续建议按这个顺序继续：

1. 机械装配前完成 16 电机绝对零点、方向、减速比和关节上下限标定，切换到 `WD_LIMIT_CALIBRATED_ABSOLUTE`。
2. 已按 RobStride 0x02 的 `8*pi` 周期实现 MCU 连续化；继续做长时间跨多圈验收并
   归档最大反馈帧间隔，但这仍不能替代机械绝对零点与关节限位标定。
3. 装配后以低力矩单关节复核方向、绝对软限位、超速保护和独立硬件急停，再允许全机 RL 动作。
4. 将本次硬件验收变为可重复的发布前测试：固件 hash、16 路 `>=490 Hz`、观测 age/skew 和清理退出结果必须一起归档。

## 2026-07-11 机械绝对标定接入

本节取代本文早期所有“当前仍为 startup-relative”的状态描述；早期段落只保留为历史
排障记录。

- 新增共享单一标定源 `description/motor_absolute_calibration_table.inc`，MCU C 与 PC
  C++ 同时使用，统一 CAN/ID/关节符号、方向、raw 零点、raw 硬边界和 URDF 上下限。
- 正式 setpoint 只接受 `WD_LIMIT_CALIBRATED_ABSOLUTE`；启动姿态不再参与零点或限位。
- 单圈 raw 机械相位和连续轮展开均按用户确认的 `2*pi` 周期；0x02 字段较宽的数值
  编码区间不再被错误当成物理回绕周期。
- 12 个腿关节增加独立 raw 安全墙：边界前 `5 deg` 连续回推，达到记录边界时强制
  最大向内力矩，绝不因触边直接置零力矩。轮通道不设置位置墙。
- 用户仅给出四个膝关节的 URDF 零点，因此膝 raw 边界暂由零点和
  URDF `[-2.5,+2.5] rad` 外推；吊装 `z` 运动检查后将 FR_Knee/HR_Knee 方向从
  `+1` 修正为 `-1`，FL_Knee/HL_Knee 保持 `+1`。首次扩大行程前必须补做两端复核。
- 用户要求 RS06 腿力矩上限为 `12 Nm`。早期吊装 `z` 全局方向检查曾使用
  `5 Nm`，但按每条腿 `2.5 kg` 和约 `0.2 m` 有效力臂估算，5 Nm 接近仅抵消重力，
  后续按 URDF 静力与动载估算放宽为 `12 Nm`；轮速命令与轮实测超速上限改为
  `4 rad/s`，腿实测超速保护保持 `2 rad/s`，RS01 等效电流/力矩上限保守保持
  `2 Nm`。单电机方向资格仍固定为 `0.6 Nm / 1 rad/s`。
- 双 MCU 固件和 PC 实机目标均已在源码侧构建通过。烧录、reset 和首次逐电机方向
  验证按 `ABSOLUTE_LIMIT_DEPLOY_SEQUENCE.md` 执行。

### 2026-07-13 膝限位与 StandUp 入轨修正

- 实机趴地时四膝稳定在约 `|q|=2.57..2.60 rad`，确认旧 URDF 的 `+/-2.5 rad`
  范围错误；真实膝范围为 `+/-2.65 rad`。训练 URDF、部署 URDF/MuJoCo、共享标定表
  和 PC/MCU 指令裁剪已统一更新。
- 旧 raw 硬墙在越界时会绕过 `50 Nm/s` 力矩斜率并直接注入满幅向内力矩，造成四膝
  同时冲击，随后由 FR_HipX/HR_HipY 等耦合关节先触发实测超速。
- 新 joint/raw 墙保留边界前 `5 deg` 的保护区，但只裁掉继续向外的力矩；规划产生的
  向内恢复力矩不被增强，并继续服从正常力矩限幅、斜率和速度保护。

### 装配后静态启动自检

吊装测试发现每条腿约 `2.5 kg`，原 bridge 的逐电机 `0.6 Nm` 启动激励无法带动
`FL_HipX`：500 Hz 和快慢反馈误差均正常，但 `pex/vex=0` 后超时。按用户要求，正式
启动自检改为全程零命令：只要求绝对模式、16/16 在线、无故障、快反馈 `>=490 Hz`
和快慢位置/速度连续三次匹配。`pex/vex` 保留为诊断但不再阻塞 ready，正式 bridge
也不再生成 qualification-excitation setpoint。该静态检查不重新证明动态速度缩放或
机械方向；两者依赖已完成的台架验证，并由吊装后的单次 `z` 全局方向检查复核。

### 2026-07-13 USB CDC 写超时的 MCU 序号确认

RL 稳定运行时出现过 `/dev/ttyACM1` 连续五次 host write timeout，但同一时刻 MCU
反馈仍新鲜：`feedback_age=4.6 ms`、`setpoint_age=2 ms`。这说明 Linux CDC-ACM
可能在完整帧已到达 MCU 后仍向 pyserial 报告写完成超时。不能简单把重试提高到20次：
`20 * 10 ms` 约为 `200 ms`，已超过 MCU 的 `100 ms` 命令看门狗，并会延迟真正
断链的上层处置。

PC bridge 继续保持每端口最多五次、约 `55 ms` 的有界重试。五次 host 写均超时后，
只在新收到的 MCU feedback 明确满足以下条件时将本次写判为成功：

- `mcu_last_setpoint_seq == pc_packet_seq`；
- MCU setpoint age 不超过 `75 ms`，且仍严格小于 MCU 回报的命令超时；
- 该 feedback 是本轮写开始后新收到的数据。

如果任一条件不成立，仍按 `persistent USB write failure` 锁存并退出；因此该修正只
消除“host completion 假超时”，不放宽真实通信失效保护。周期通信日志新增
`mcu_ack_recoveries_total`，真正失败日志同时打印 `pc_packet_seq` 与 MCU确认序号。

### 2026-07-14 RS01 轮通道改为训练对齐运控模式

- RS01 手册明确给出运控公式
  `tau=Kd*(v_set-v_actual)+Kp*(p_set-p_actual)+tau_ff`。训练端轮执行器为
  `stiffness=0, damping=0.4`，因此四个轮通道改为 `run_mode=0`、`Kp=0`、
  `Kd=0.4`、`tau_ff=0`，不再使用 `run_mode=2 + spd_ref` 内部速度 PI 环。
- 新增独立、可做主机单元测试的 RobStride 通信类型1编码器；目标位置/速度、Kp/Kd
  使用手册规定的大端16位量化，力矩前馈写入扩展 CAN ID 的 bit23..8。
- enable 前对 RS01 写 `limit_torque(0x700B)=8 Nm`；轮目标仍由 PC/MCU 双层限制在
  `+/-8 rad/s`，500 Hz 实测速连续三帧达到 `10 rad/s` 的急停规则不变。
- PC 硬件接口不再清除轮 `Kd`；MCU 只允许轮 `Kd` 在 `[0,5]`，并继续强制轮
  `Kp=q_des=tau_ff=0`。StandUp、SitDown、RL 和 JointDamping 的零轮速使用
  `Kd=0.4` 柔性阻尼；Idle、急停及 disable 前发送 `Kd=0` 真正零力矩。
- 两块正式固件分别以 `WD_CAN_BUS_BASE=0/2` 构建通过，协议/安全主机测试和 PC 实机
  目标均构建通过；尚未在本条记录时烧录或带动力验证。
