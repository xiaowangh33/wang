# 机械绝对限位部署状态

2026-07-11 已根据机械装配后的实测数据关闭原 startup-relative 阻断项。当前正式
WDP4 路径只接受 `WD_LIMIT_CALIBRATED_ABSOLUTE`，旧 wire value `0` 仅为协议兼容保留；
MCU 收到后会标记 bad packet 并强制改为绝对模式，不能回退为启动相对零点。

唯一标定与符号映射源是：

```text
wheeldog/description/motor_absolute_calibration_table.inc
```

该表由 MCU C 和 PC C++ 同时编译使用，包含 logical index、CAN、ID、关节符号、模式、
方向、raw 零点、raw 硬边界及 URDF 上下限。髋关节 raw 端点来自机械硬限位实测；
2026-07-13 确认原 URDF 膝范围有误，真实膝范围是 `[-2.65,+2.65] rad`，四个膝的
URDF、部署模型、指令限位和 raw 边界均已同步。StandUp 方向检查确认左前/左后膝
方向为 `+1`，右前/右后膝必须为 `-1`。

## 固件安全语义

- 单圈绝对编码器机械相位按已确认的 `2*pi` 周期对齐；速度轮也按 `2*pi` 连续展开。
- 12 个位置关节同时受 URDF 目标裁剪和独立 raw 域安全墙保护。
- joint/raw 在边界前 `5 deg` 进入方向保护区：继续向外的力矩立即裁成零，轨迹计划的
  向内恢复力矩保持原值。限位墙不再合成满力矩回推，
  避免从趴地姿态进入 StandUp 时产生冲击。
- 当前 RS06 腿通道最终力矩上限为 `36 Nm`；RS01 轮速命令为
  `8 rad/s`，所有关节从 `8 rad/s` 到 `10 rad/s` 同向加速力矩逐步削减，
  `>=10 rad/s` 连续三个新鲜样本才急停（`>=16 rad/s` 单帧立即急停）；轮驱动等效电流/力矩上限仍
  保守保持 `2 Nm`。
- Idle 的位置关节命令为 `kp=kd=tau_ff=0` 时直接保持零电流，不激活 joint/raw
  限位墙；StandUp/RL 等任意非被动命令仍保留两层限位墙。
- raw 墙介入会置位 `WD_STATUS_RAW_POSITION_LIMIT`。标定表无效会置位
  `WD_STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED` 并阻止 live。

软件只能保证不会主动下发越界方向力矩，无法在外力、惯性、机构变形或传感器故障下
数学保证实际 raw 永不越界。因此首次上电仍须可靠吊装/支撑，并按
`ABSOLUTE_LIMIT_DEPLOY_SEQUENCE.md` 用 `0.6 Nm` 单电机资格激励复核 12 个腿关节方向，
再进入 StandUp/RL。部署腿关节现已与训练 effort limit 对齐为 36 Nm。

右前/右后膝符号修正后，MCU A 和 MCU B 都必须重新烧录并手动 reset；完成
16/16 只读位置检查和吊装 StandUp 复核后，实物上的旧符号才算被替换。
