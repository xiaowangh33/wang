# PC→USB→双 MCU→四 CAN→16 电机通信与安全复核

日期：2026-07-15

## 结论

当前链路没有稳定态通信饱和，但不是“余量无限”：

- USB 若两块 MCU 共用同一个 12 Mbit/s Full-Speed 上行，实时控制的应用层数据约
  `595.2 kB/s`（`4.76 Mbit/s`）；考虑 64 字节 bulk 分包、token、握手和帧间隔后，
  估算占用约 50%～55%。正常态有余量，短时主机调度/CDC completion 抖动可以跨过
  一个 5 ms PC 周期。
- 每条 1 Mbit/s CAN 上有四台电机，命令和 operation-status 各 `4*500=2000`
  帧/s；加上异步 READ_PARAM 交叉检查后，典型/最坏填充估算约 55%～67%。不饱和，
  但 CAN 是整条链中余量最小的一段，不应继续提高 500 Hz 电机刷新率。
- MCU 本地控制为高优先级 1 kHz 静态任务。CAN RX 队列现为 64 槽，按双 CAN 合计
  约 4.2 帧/ms 估算，可吸收约 15 ms 突发；旧 32 槽只有约 7 ms。
- 现有实机 200 Hz 日志中，单端口 USB write 平均约 `0.6～0.8 ms`、正常峰值约
  `4.6 ms`；双端口串行写使 PC 每个 5 ms 周期常用掉约 25%～32%，偶发 setpoint
  gap 为 `8～14 ms`。MCU setpoint age 正常仍为 `0～3 ms`，说明是可恢复抖动，
  不是稳定饱和。

源码中的 `921600` 是 CDC line-coding/主机串口元数据，不是 USB 线速；实际链路仍是
USB Full-Speed bulk，不能用 `921600 baud` 直接判断带宽。

## 带宽核算

| 链路 | 单元大小与频率 | 总应用层流量 | 判断 |
|---|---:|---:|---|
| PC→两 MCU | WDP4 setpoint `368 B * 200 Hz * 2` | `147.2 kB/s` | 余量充足 |
| 两 MCU→PC | full feedback `1188 B * 最高 200 Hz * 2` | `475.2 kB/s` | USB 主负载 |
| USB 合计 | 上述双向相加 | `622.4 kB/s = 4.98 Mbit/s` | FS 总线上中等偏高，未饱和 |
| 每条 CAN | 4 电机命令 + 4 电机反馈，均 500 Hz | `4000` 主帧/s | 约 52%～64% 基础占用 |
| 每条 CAN 慢检查 | 约 100 request/s + 100 reply/s | 约 `200` 帧/s | 合计约 55%～67% |

CAN 扩展 8 字节数据帧不含位填充约 131 bit，最坏位填充约 160 bit。这里未计错误帧和
重发，因此保留的 33%～45% 是故障/仲裁余量，不是可继续增加周期业务的预算。

USB feedback 在没有新 observation 时最低 50 Hz，但当前 live setpoint 为 200 Hz，
每次 MCU 应用新 setpoint 都会置 `observation_tx_pending`，因此稳态上限应按 200 Hz
feedback 核算，不能再沿用旧文档的 50 Hz 数字。

## 已处理的阻塞和误停路径

### PC/USB

- 写完成 timeout、短写和一次 `SerialException/OSError` 不再等价于整机退出。只要
  MCU 最新反馈证明上一 setpoint 的“反馈年龄 + MCU 命令年龄”仍在 250 ms 内，就
  丢弃本次刷新并等待下一帧重同步。
- 250 ms 是容错上限，不会无限持有：它比默认 300 ms MCU 命令看门狗提前 50 ms。
  真正拔线后反馈不再更新，仍会在看门狗前退出并清零/disable。
- 单次 USB read/ioctl 异常仅计数并继续服务另一块 MCU；持续异常由 500 ms MCU
  feedback-loss 门限裁决。
- MCU 对 host timeout 的精确 `last_setpoint_seq` 确认仍保留，避免把“数据已到 MCU、
  Linux 只是在等 completion 时超时”误判为丢包。
- C++/Python 共享命令 seqlock 的最后合格帧保持由 20 ms 放宽到 50 ms；仍远短于
  300 ms MCU 看门狗。PC publisher 真正停止 250 ms 仍会退出。
- 双 MCU observation 改为每端保留 64 帧、在 100 ms 窗口内按相同 sequence
  配对；一次 USB 延迟不再因两个端口“最新包”相差一帧而永久失配。匹配后的 USB
  到达偏差门限由 15 ms 放宽到 30 ms，连续 500 ms 没有新的合格共同帧才锁停。

### MCU/CAN

- bxCAN mailbox 暂时占满时，发现、verified-enable 和异步 READ_PARAM 不再设置
  `CAN_TX_ERROR + LIVE_CONTROL_BLOCKED`；对应步骤不前移，按 5/20 ms 周期重试。
- `CAN_RX_OVERFLOW`、`CAN_TX_ERROR`、`CAN_TX_DEADLINE_MISS` 和单独的
  `LIVE_CONTROL_BLOCKED` 作为通信诊断写入 10 s 周期日志，不再由 PC 单包立即触发
  FATAL；只有连续 5 个不同 MCU feedback 包都出现新增 CAN 错误，或 live blocked
  连续 5 包，才升级为持续通信故障。真正后果还由新鲜度、速率、命令年龄和驱动
  fault 独立检查。
- CAN RX 环形队列由 32 增至 64；这只扩大突发吸收能力，不会让旧反馈参与 PD。
- CAN1/CAN2 开启 AutoBusOff。短时 bus-off 恢复期间，对应电机因反馈超过 5 ms 先
  本地零命令；总线恢复且重新收到新 generation 后才恢复计算。持续 bus-off 仍会
  到达 300 ms 全机锁停。

## 现在的分层安全时序

1. `<=5 ms`：500 Hz 新反馈正常参与本地 PD；旧样本绝不重复计算。
2. `>5 ms` 单电机快反馈陈旧：该通道立即零命令，但不因一次 CAN 调度洞停 16 电机。
3. `<=50 ms`：允许 PC seqlock 抢占和 USB 有界重试/MCU ACK 恢复。
4. `250 ms`：PC 不再容忍未确认 setpoint，提前于 MCU 看门狗退出清理。
5. `300 ms`：MCU 命令超时或单电机快反馈持续失联，进入本地零命令/disable/锁停。
6. `500 ms`：双 MCU coherent observation、PC MCU-feedback 与 C++ 最终一致反馈
   门限兜底。
7. 电机硬件 fault、急停、绝对标定失效、控制任务 fallback、持续三帧实测超速和
   MCU `LIVE_SAFETY_STOP` 仍立即停机，没有放宽。

MCU 的 350 Hz 最低速率是 500 ms 窗口判定；PC 运行期还要求连续 5 个约 1 s 低速率
窗口才退出，因此一两帧丢失不会走“低速率整机停机”。

## “断电”语义

本仓库运行时故障路径没有控制整机电源继电器。bridge FATAL 后执行的是：C++ 发布
零五元组、bridge 连发 dry-run 零命令、MCU 对电机发 zero/disable。因此现场看到的是
16 驱动失能/整机卸力，不是代码直接切断 PC 或动力电。如果现场确实有电源继电器断开，
需继续检查仓库外的急停/电源板逻辑。

## 验证与上线要求

- PC 测试：54 项通过，包括单次 USB 写异常、单次 USB 读异常、上一 setpoint 保持、
  历史同序号恢复、陈旧历史拒绝和连续 5 包通信错误才升级。
- MCU host tests：actuator units、CAN scheduler、fast feedback、safety 和 RobStride
  motion 编码全部通过。
- MCU A/B release 均构建通过；RAM 使用 `96,464 / 131,072 B = 73.6%`，64 槽队列
  仍有约 34.6 kB RAM 余量。
- MCU A/B 已于 2026-07-15 使用项目内 `st-flash` 写入并通过片上校验；PC 实机程序
  已在 observation 历史配对修改后重新构建。应先吊装/轮悬空做至少 30 min soak，
  归档 `setpoint_gap_max`、USB timeout/read-error 计数、CAN overflow/TX/deferred、
  每电机 TX/RX Hz、最大 operation gap、observation age/skew。
- 不建议把 PC/MCU 频率继续上调。若以后确需更高 PC 观测率，应先把 1172 B 全量诊断
  拆成“高频紧凑状态 + 低频诊断”，而不是直接增加 Full-Speed USB 包频率。
