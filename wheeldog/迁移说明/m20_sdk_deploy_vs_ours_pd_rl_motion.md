# 云深处 M20 部署框架 vs 我们框架 — PD/RL/运控差异分析

> 范围：`/home/gu/sdk_deploy/src/M20_sdk_deploy/`（云深处 M20 官方部署框架）。
> 视角：以我们的部署框架（`new_dog_robot/四足机器人/QuardLegAllPrograms/rl/`）的大框架和信息流为准，**只看 M20 与我们不同的部分**：PD 计算、速度控制、RL 解算、传统运控（站起 / 趴下）。
> 不看：硬件驱动（EtherCAT/DDS/CiA-402）、DDS 拓扑、心跳线程、IMU 解析、ROS2 节点等。
>
> 重点参考：
> - 我们已有的迁移计划：`迁移说明/deployment_framework_migration_plan.md`
> - 训练侧 vs M20 的对照：`迁移说明/wheel_vs_quadruped_rl.md`

---

## 0. 一句话区别（最关键）

| 维度 | 我们（Mydog 46kg 四足 → 轮足） | 云深处 M20（轮足） |
|---|---|---|
| **PD 在哪算** | **C++ 上位机算 τ**，再下发扭矩 | **硬件算 PD**，上位机下发 `[kp, q_des, kd, v_des, τ_ff]` 5 列矩阵 |
| **轮控制** | 走位置环（PD）— 我们仍规划走 PD | **轮纯速度环**（policy action 直接 = `v_des`，kp=0） |
| **姿态四元数** | q = body 在世界系 | 训练侧用 **projected_gravity = R⁻¹·g_body**；部署侧用同样的 R⁻¹ |
| **Action 拼接** | 12 维全 joint_pos | **混合 16 维**：12 joint_pos + 4 joint_vel（policy 输出后 4 维直接当 v_des） |
| **Action scale** | leg 0.125/0.25 | leg **0.125/0.25/0.25** + wheel **5.0**（≠训练侧 20.0！） |
| **站起** | 关键帧插值（线性 / 三次样条） | **三次样条从当前位姿插值到反解目标位姿**，1.5 s |
| **趴下** | 关键帧插值 | **三次样条插值到"折腿极低"位姿**（h≈0.03 m），2 s + 1 s 阻尼 |
| **关节阻尼** | kp=0, kd=大数（被动） | kp=0, kd=**腿 4 / 轮 1**（被动） |
| **控制环频率** | EC 2 kHz / PD 1 kHz / RL ~50 Hz | **统一 200 Hz**（timerfd + epoll），RL decimation=4 → 50 Hz |

> ⚠️ 第一条（**PD 在哪算**）是最本质的架构差异，所有"关节命令矩阵"看起来一样，但实现位置完全不同。

---

## 1. PD 计算（核心差异）

### 1.1 我们：上位机算 τ

我们的 `SetJointCommand(matrix)` 实际下发的是 `τ_cmd`（标量扭矩），PD 公式在 `Mydog_integrated_hardware_interface.hpp::ApplyControl()` 里跑：

```cpp
// 伪代码：我们框架
τ[i] = kp[i] * (q_des[i] - q[i]) + kd[i] * (v_des[i] - v[i]) + τ_ff[i];
τ[i] = clip(τ[i], -τ_limit[i], τ_limit[i]);
ec_send_torque(slave_id[i], τ[i]);
```

优点：上位机能改算法（重力补偿、阻抗、零力矩点…）；仿真与硬件代码同源。
缺点：通信抖动直接影响扭矩抖动，1 kHz 调度压力大。

### 1.2 M20：硬件算 PD，上位机只下发"指令包"

`interface/robot/robot_interface.h:99-104` 注释明确：

```cpp
/**
 * Set the joint command in standard form
 *                  torque = kp * (qDes - q) + kd * (vDes - v) + tff
 * @param  input       a dof_num*5 matrix and each column represent
 *          kp, goal_angle_pos, kd, goal_vel, torque_feedforward
 */
virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) = 0;
```

C++ 端只 **打包** `[kp, q_des, kd, v_des, τ_ff]` 矩阵，通过 `ri_ptr_->SetJointCommand(cmd)` 把它丢给 DDS（`dds_interface.hpp` 里的 `drdds/msg/JointsDataCmd`）。**真正的 PD 公式在电机驱动里跑**（CiA-402 CSP 模式，或厂家自定义 protocol）。

实际下发的 5 列矩阵在多个状态里都能看到：

| 状态 | kp | q_des | kd | v_des | τ_ff | 来源 |
|---|---|---|---|---|---|---|
| **IdleState** | 0 | 0 | 0 | 0 | 0 | `idle_state.hpp:148-149`（仅做传感器检查，不动关节） |
| **StandUpState** | swing_leg_kp（200） | 三次样条 | 0（前期）→1（后期） | 三次样条 | 0 | `standup_state.hpp:141-143` |
| **LieDownState** | `[300,300,300,0]` | 三次样条 | `[4.5,4.5,4.5,3]` | 三次样条 → 0（轮） | 0 | `liedown_state.hpp:118-121` |
| **JointDampingState** | 0 | 0 | `[4,4,4,1]` | 0 | 0 | `joint_damping_state.hpp:26-27` |
| **RLControlState** | `[80,80,80,0]` | policy 输出（前 12 维） | `[2,2,2,0.6]` | 0（腿）/ policy 输出（轮） | 0 | `m20_policy_runner.hpp:115-116, 234-235` |

> 上表就是把 `SetJointCommand(MatXf)` 各列直接 dump 出来的结果，没有任何 τ 计算在 C++ 里发生。

**对我们迁移的影响**：
- 如果我们维持"上位机算 τ"路线，迁移后**接口语义变了**——`SetJointCommand` 的第 4 列 `v_des` 实际上是被忽略的（驱动不再做 PD），第 5 列 `τ_ff` 直接当 τ_cmd 用。
- 如果硬件端支持 CSP/Cyclic Synchronous Position 模式（大多数 EtherCAT 伺服支持），可考虑切到 M20 路线：上位机下发 `[kp, q_des, kd, v_des, τ_ff]`，硬件闭环。这样控制环频率能从 1 kHz 降到 200 Hz 不影响性能。
- **推荐**：维持现状（Mydog 上位机算 τ），但**保留 5 列接口**，让 PD 增益可调；将来切到 CSP 只需改 `ApplyControl()` 的填充方式，不改接口。

---

## 2. 速度控制（轮速环）

### 2.1 训练侧约定的"轮是速度环"

`/home/gu/桌面/robot_lab/source/robot_lab/robot_lab/assets/deeprobotics.py` 里 M20 的 `wheel` actuator 是 `DCMotorCfg`，`stiffness=0`、`damping=0.6`、`effort_limit=21.6 Nm`、`velocity_limit=79.3 rad/s`。其物理含义是：

- kp = 0 ⇒ **不做位置恢复**
- kd = 0.6 ⇒ 仅提供弱阻尼收敛
- action = `set_joint_velocity_target(v_des)` ⇒ PhysX/DCMotor 把 `v_des` 作为目标转速驱动

### 2.2 M20 部署侧：把"轮速度"塞进 v_des 列

`m20_policy_runner.hpp:227-236`：

```cpp
for (int i = 0; i < action_dim; ++i) {
    tmp_action_eigen(i) = current_action_eigen(policy2robot_idx[i]);
    tmp_action_eigen(i) *= action_scale_robot[i];   // 腿 0.125/0.25，轮 5
}
tmp_action_eigen += dof_default_eigen_robot;        // 腿加默认位姿；轮默认 0

for (int i = 0; i < 4; ++i) {
    // 腿的 3 个关节 → q_des；轮的 1 个关节 → v_des
    robot_action.goal_joint_pos.segment(i*4, 3) = tmp_action_eigen.segment(i*4, 3);
    robot_action.goal_joint_vel(i*4+3) = tmp_action_eigen(i*4+3);
}
```

注意矩阵的填法：**第 4 列** `goal_joint_vel`（v_des）只在轮关节（第 4 个 = i*4+3）写值，**腿关节 v_des 全填 0**。

硬件端的处理方式要看驱动：
- 若是 CSP 模式（位置闭环）：硬件拿到 `kp=80, q_des=0, kd=0.6, v_des=policy_wheel_speed, τ_ff=0` ⇒ 实际 `τ = 0*(0-q) + 0.6*(v_des-v) + 0` ⇒ 纯速度环 ✓
- 若是 CSV 模式（速度闭环）：硬件直接消费 `v_des`，前面的 kp/q_des/kd 被忽略

### 2.3 与我们框架的对比

| 项 | 我们规划 | M20 部署 | 我们的实现位置 |
|---|---|---|---|
| 轮 kp | 0（旁路 PD） | **0**（写在 matrix 里） | `ApplyControl` 内判 actuator_mode |
| 轮 kd | 0（电气再生） | **0.6**（弱阻尼） | 同上 |
| 轮 v_des 来源 | policy 后 4 维 × 20.0 | policy 后 4 维 × **5.0** | `tmp_action_eigen * action_scale` |
| 腿 v_des | 0 | **0** | 默认填 0 |
| τ_ff | 0 | **0** | 默认填 0 |

> ⚠️ **scale 差异（重要）**：训练 `JointVelocityActionCfg.scale = 20.0`，但 M20 部署端 `action_scale_robot[i] = 5.0`（对 wheel 关节）。这里有 4× 差异，**两种可能**：
> 1. 训练时 `use_default_offset=True` 已经把 action 加了 offset，部署 ×5 才是物理量；
> 2. M20 内部 retrain 过，部署对应新训练；
> 3. 文档未同步。
>
> **我们迁移时**：必须**以 M20 的 5.0 为准**（这是 ONNX 实测接口），写文档标注"训练侧 scale=20.0 是 action 流含义，部署侧 scale=5.0 是物理量含义，两者差 4× 是设计选择"。

### 2.4 速度指令来源（cmd_vel）

`m20_policy_runner.hpp:207` 直接从 `UserCommand` 取：

```cpp
Vec3f command = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);
```

键盘 (`keyboard_interface.hpp`) 和手柄 (`gamepad_interface.hpp`) 的最大速度：

| 通道 | M20 限制（`KeyboardInterface` 成员） |
|---|---|
| `max_forward_` | **0.7 m/s** |
| `max_side_`   | **0.5 m/s** |
| `max_yaw_`    | **0.7 rad/s** |

`UserCommand` 的 cmd_vel 直接拼到 obs 的 `command` 段（3 维），**不再乘任何 scale**。
这一点与我们框架一致（键盘 → `usr_cmd_->vel_*_scale` → 直接进 obs）。

---

## 3. RL 解算（policy runner）

### 3.1 关节顺序 / 索引映射

部署端关节顺序（`robot_order`，16 个）：
```
FL: hipx hipy knee wheel  | FR: hipx hipy knee wheel
HL: hipx hipy knee wheel  | HR: hipx hipy knee wheel
```

策略侧关节顺序（`policy_order`，16 个）：
```
FL: hipx hipy knee  | FR: hipx hipy knee  | HL: hipx hipy knee  | HR: hipx hipy knee
FL_wheel | FR_wheel | HL_wheel | HR_wheel
```

差别：**策略侧把 12 个腿关节排前面、4 个轮关节排后面**；部署侧按腿+轮交错。**两者不一致**，需要 `generate_permutation()` 做映射：

```cpp
// m20_policy_runner.hpp:142-163
robot2policy_idx = generate_permutation(robot_order, policy_order);
policy2robot_idx = generate_permutation(policy_order, robot_order);
```

`robot2policy_idx[i]` 给 obs 用（把关节 i 的 q/v 放到策略输入的第几个位置），`policy2robot_idx[i]` 给 action 用（把策略输出的第 i 个值放回机器人第几个关节）。

> **对我们的影响**：我们框架必须保留**两套索引**——`control_index`（控制矩阵行号，16 维按部署顺序）和 `policy_index`（策略输入/输出向量中的位置，按 leg 前 wheel 后）。
>
> 这正是 `deployment_framework_migration_plan.md §6` 里 `JointConfig.policy_index` 字段要解决的问题。

### 3.2 Observation 构造（57 维）

`m20_policy_runner.hpp:205-222`：

```cpp
Vec3f base_omega = ro.base_omega * omega_scale_;            // omega_scale_ = 0.25
Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;  // R⁻¹·g
Vec3f command = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);

for (int i = 0; i < action_dim; ++i) {
    joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i]);
    joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i]) * dof_vel_scale_;  // dof_vel_scale_ = 0.05
}
joint_pos_rl.segment(12, 4).setZero();  // 关键：轮位强制置 0

joint_pos_rl -= dof_default_eigen_policy;

current_observation_ << base_omega,         // 3
                       projected_gravity,   // 3
                       command,             // 3
                       joint_pos_rl,        // 16 (前 12 是腿相对位姿；后 4 = 0)
                       joint_vel_rl,        // 16
                       last_action_eigen;   // 16
//                                total = 57
```

**关键点**：
1. **关节顺序错位**：用 `robot2policy_idx` 把 16 个关节按 leg→wheel 顺序排好
2. **轮位置强制 0**：`joint_pos_rl.segment(12,4).setZero()` —— 与训练侧 `joint_pos_rel_without_wheel` 一致
3. **轮速度保留**：注意 `joint_vel_rl` 没有 zero out wheel 部分，与训练一致
4. **关节速度 scale = 0.05**：腿和轮都用同一个 0.05（部署侧以 `robot_lab_obs_alignment.hpp` 对齐）
5. **last_action 也是 16 维**：与 action 拼接顺序一致
6. **projected_gravity**：用 `R⁻¹·g_world` 算出来（g_world = [0,0,-1]），不是直接读 IMU 倾角

### 3.3 默认位姿（dof_default_eigen_policy / robot）

```cpp
// 策略侧默认位姿（前 12 个腿关节，单位 rad）
dof_default_eigen_policy << 0.0, -0.6,  1.0,     // FL
                            0.0, -0.6,  1.0,     // FR
                            0.0,  0.6, -1.0,     // HL  ← 后腿膝反向
                            0.0,  0.6, -1.0,     // HR
                            0.0, 0.0, 0.0, 0.0;  // wheels
// 部署侧默认位姿（16 维，按 robot_order）
dof_default_eigen_robot << 0.0, -0.6,  1.0, 0.0,    // FL
                           0.0, -0.6,  1.0, 0.0,    // FR
                           0.0,  0.6, -1.0, 0.0,    // HL
                           0.0,  0.6, -1.0, 0.0;    // HR
```

**注意**：策略侧默认位姿的"HL/HR" 是正向 `+0.6/-1.0`，但部署侧是 `"+0.6, -1.0"`（按"后髋膝取反"的规则）。这里看似矛盾，**实际是策略训练时直接用正向表示（不带后髋膝取反约定），部署侧在 `GetHipYPosByHeight()` / `GetKneePosByHeight()` 里做了取反**——这意味着我们在迁移时**必须严格按 M20 的几何约定**：
- 前腿 FL/FR：hipy 负值 = 大腿前抬；knee 正值 = 小腿后伸
- 后腿 HL/HR：hipy 正值 = 大腿后抬；knee 负值 = 小腿前伸

### 3.4 Action 解算（policy → 关节目标）

```cpp
current_action_eigen = Onnx_infer(current_observation_);  // 16 维 [-1,1]
last_action_eigen    = current_action_eigen;

for (int i = 0; i < action_dim; ++i) {
    tmp_action_eigen(i) = current_action_eigen(policy2robot_idx[i]);  // 反排列
    tmp_action_eigen(i) *= action_scale_robot[i];                       // 物理量
}
tmp_action_eigen += dof_default_eigen_robot;                            // 加默认位姿（轮默认 0）

for (int i = 0; i < 4; ++i) {
    // 腿（前 3 维）：写到 goal_joint_pos
    robot_action.goal_joint_pos.segment(i*4, 3) = tmp_action_eigen.segment(i*4, 3);
    // 轮（第 4 维）：写到 goal_joint_vel（v_des）
    robot_action.goal_joint_vel(i*4+3) = tmp_action_eigen(i*4+3);
}
```

**action_scale_robot**（`m20_policy_runner.hpp:65-68`）：
```cpp
{0.125, 0.25, 0.25, 5,    // FL
 0.125, 0.25, 0.25, 5,    // FR
 0.125, 0.25, 0.25, 5,    // HL
 0.125, 0.25, 0.25, 5}    // HR
```

**对比我们规划（`deployment_framework_migration_plan.md §1`）**：
- 腿：HipX 0.125, HipY/Knee 0.25 ✓ 一致
- 轮：M20 = **5.0**，我们规划 = **20.0**（**需要再核实**）

### 3.5 控制频率（decimation）

`m20_policy_runner.hpp:104`：`SetDecimation(4)`，而主循环是 200 Hz，所以 **RL 实际 50 Hz**。
这与我们规划 `decimation=20`（在 1 kHz 主环下 50 Hz）等价；**频率逻辑不变**。

但要注意：M20 主环是 200 Hz（`set_timer.time_init(5)` = 5 ms），RL 线程里 `PolicyRunner()` 用 `std::this_thread::sleep_for(100us)` 轮询。每 4 个主循环 tick 触发一次 ONNX forward。

### 3.6 双缓冲（多线程安全）

`RLControlState::UpdateRobotObservation()` 把 `RobotBasicState rbs_[2]` 写成 ping-pong 双缓冲：

```cpp
int write_idx = rbs_write_index_.load(std::memory_order_relaxed);
RobotBasicState& buffer = rbs_[write_idx];
// 写 buffer ...
rbs_write_index_.store(1 - write_idx, std::memory_order_release);

// 另一线程读
int getrbsReadIndex() const { return 1 - rbs_write_index_.load(std::memory_order_acquire); }
auto ra = policy_ptr_->getRobotAction(rbs_[getrbsReadIndex()], *(uc_ptr_->GetUserCommand()));
```

**对我们**：我们当前是单线程（ONNX 跑在主循环里），不需要双缓冲；如果未来也切到独立线程，必须引入这个机制。

---

## 4. 传统运控（StandUp / LieDown / JointDamping / Idle）

### 4.1 IdleState（传感器检查站）

```cpp
virtual void Run() {
    GetProprioceptiveData();              // 读 joint/IMU
    joint_normal_flag_ = JointDataNormalCheck();  // NaN、越限、超速
    DisplayProprioceptiveInfo();          // 1 Hz 打印
    MatXf cmd = MatXf::Zero(16, 5);       // 全 0 输出
    ri_ptr_->SetJointCommand(cmd);
}
```

`JointDataNormalCheck()` 检查：
- 关节位置 NaN / 越界（±0.2 rad 容差）
- 关节速度 NaN / 超速（腿按 `joint_vel_limit_`、轮按 `wheel_vel_limit_=100`）
- IMU RPY NaN / 越界 / 加速度 `|a| ∉ [0.1g, 3g]`

**检查失败 → 保持 Idle 状态**（不进入 StandUp）。这与我们的 `InitState` 行为一致。
**但**：M20 的 Idle 输出**完全为 0**（kp=0, kd=0, q_des=0, v_des=0）——这要求硬件在 Idle 阶段不下任何力矩（驱动可能进入 disable 或自由状态），等价于"机械失能"。我们框架的 Idle 行为需要核对。

### 4.2 StandUpState（站起）

```cpp
// 目标位姿（按 robot_order）：FL/FR/HL/HR 每条腿 [hipx, hipy, knee, wheel]
goal_joint_pos_(16) = Vec4f(0, GetHipYPosByHeight(pre_height_), GetKneePosByHeight(pre_height_), 0).replicate(4, 1);
// 后髋膝取反（注意是 HL/HR，不是 FR）
goal_joint_pos_(4)  = 0;                   // FR hipx 不变 (实际是 goal_joint_pos_(4) = -0 = 0)
goal_joint_pos_(12) = 0;                   // HR hipx 不变
goal_joint_pos_(9)  = -goal_joint_pos_(9); // HL hipy 取反
goal_joint_pos_(10) = -goal_joint_pos_(10);// HL knee 取反
goal_joint_pos_(13) = -goal_joint_pos_(13);// HR hipy 取反
goal_joint_pos_(14) = -goal_joint_pos_(14);// HR knee 取反
```

**注意**：`pre_height_ = 0.12 m`，用反解 `GetHipYPosByHeight(h)` / `GetKneePosByHeight(h)`：

```cpp
float GetHipYPosByHeight(float h) {
    float l1 = cp_ptr_->thigh_len_;   // 0.25
    float l2 = cp_ptr_->shank_len_;   // 0.25
    // 几何反解：cos(θ_hipy) = -(l1² + h² - l2²) / (2·h·l1)
    float theta = -acos((l1*l1 + h*h - l2*l2) / (2*h*l1));
    return LimitNumber(theta, fl_joint_lower_(1), fl_joint_upper_(1));
}
float GetKneePosByHeight(float h) {
    // 几何反解：cos(θ_knee) = (l1² + l2² - h²) / (2·l1·l2)
    float theta = M_PI - acos((l1*l1 + l2*l2 - h*h) / (2*l1*l2));
    return LimitNumber(theta, fl_joint_lower_(2), fl_joint_upper_(2));
}
```

`stand_height_ = 0.38 m`，站起分两段：

| 阶段 | 时间 | 轨迹 | 增益 |
|---|---|---|---|
| **阶段 1：拉起到 pre_height** | t∈[0, 1.5 s] | 三次样条 `init_pos → goal(pre_height)`，末端 vel=0 | kp=swing_leg_kp（200），kd=0（前段）→set_wheel_kd=1（后期） |
| **阶段 2：抬升到 stand_height** | t∈[1.5 s, 3.0 s] | 三次样条 `pre_height → stand_height`（用高度反解算关节角） | kp=swing_leg_kp，kd=set_wheel_kd=1 |
| **完成（3.0 s 后）** | — | 留在 stand_height | （已切到 RL 或保 StandUp） |

> **关键点 1**：站起**不需要预定义的关键帧**——直接从当前位姿三次样条插值到反解的目标位姿。这意味着：
> - 若机器人从躺倒状态启动，第一次站起阶段 1 会直接拉平（穿地？不会，因为终点是 `pre_height=0.12` 的"趴下预站"姿态）；
> - 阶段 2 用反解把 base 抬到 `stand_height=0.38`。
>
> **关键点 2**：站起过程中轮是**被动**的（kd=0 阶段 1 前半段，kd=1 后半段），靠轮与地面的接触力决定姿态。这跟我们框架"轮关节整段被动"的设计一致。
>
> **关键点 3**：`stand_duration_ = 1.5`（可在 `control_parameters.h` 改），实际是两段 × 1.5 s = 3 s。

### 4.3 LieDownState（趴下）

```cpp
// 目标位姿：折腿极低（h=0.03 m，几乎贴地）
goal_joint_pos_(16) = Vec4f(0, GetHipYPosByHeight(0.03), GetKneePosByHeight(0.03), 0).replicate(4, 1);
// 后髋膝取反（同 StandUp）
goal_joint_pos_(4) = -goal_joint_pos_(4);
... 

// 增益（写死在源码里，未走 control_parameters）
one_leg_kp << 300, 300, 300, 0;
one_leg_kd << 4.5, 4.5, 4.5, 3;
```

**两段**：

| 阶段 | 时间 | 轨迹 | 增益 |
|---|---|---|---|
| **阶段 1：折腿到 0.03 m** | t∈[0, 2.0 s] | 三次样条 `init_pos → goal(0.03)`，末端 vel=0 | kp=300，kd=[4.5,4.5,4.5,3] |
| **阶段 2：落地阻尼** | t∈[2.0 s, 4.0 s] | 全 0（仅留 kd 阻尼） | kp=0, kd=[4.5,4.5,4.5,3] |
| **完成（4.0 s 后）** | — | 全 0（机械失能） | — |

> **关键点**：
> 1. **增益硬编码**：`kp=300, kd=[4.5,4.5,4.5,3]` 与 StandUp 不同（StandUp 用 `swing_leg_kp_=200, swing_leg_kd_=4`）。
> 2. **轮 kd=3**（比 StandUp 的 1 大）——为了趴下过程中车体不至于滑动；
> 3. **完成后 kp=0, kd=0**（与 Idle 一致）——硬件 disable / 自由状态；
> 4. `liedown_duration_ = 2.0`（控制参数里可调），实际 2 段 × 2 s = 4 s。

### 4.4 JointDampingState（被动阻尼）

```cpp
joint_cmd_.col(2) = kd_;   // kp=0, q_des=0, v_des=0, τ_ff=0
// kd = [swing_leg_kd, swing_leg_kd, swing_leg_kd, 1].replicate(4, 1)
//   = [4, 4, 4, 1,  4, 4, 4, 1,  4, 4, 4, 1,  4, 4, 4, 1]
```

3 秒后**自动转 Idle**（`if (run_time_ - time_record_ < 3.)`）——这是个防卡死机制，避免用户卡在阻尼模式。
`wheel_kd = 1` 比 LieDown 的 3 小，说明阻尼模式主要靠腿把车体撑住，轮只是辅助。

### 4.5 与我们框架的对比表

| 项 | 我们 | M20 |
|---|---|---|
| **Idle 输出** | kp=0 / kd=小 / q=当前位姿 | **全 0**（机械失能） |
| **StandUp 目标** | 关键帧列表（线性插值） | **反解** `GetHipYPosByHeight(0.12)` 然后 cubic spline |
| **StandUp 增益** | kp=站立位姿增益（固定） | kp=200（swing_leg_kp），kd=0（前段）→1（后段） |
| **StandUp 时长** | 视关键帧数 | 3 s（1.5 s × 2 段，硬编码 × 参数） |
| **LieDown 目标** | 关键帧列表 | **反解** `GetHipYPosByHeight(0.03)` 然后 cubic spline |
| **LieDown 增益** | kp=某固定值，kd=小 | **硬编码** kp=300, kd=[4.5,4.5,4.5,3] |
| **LieDown 完成** | 转 Idle | 全 0（机械失能） |
| **JointDamping kd** | 腿=大阻尼，轮=0（旁路） | 腿=4, 轮=1 |
| **JointDamping 超时** | 无 | **3 秒后强制 Idle**（防卡死） |

> **M20 设计哲学**：状态切换对增益的依赖**高度参数化**（一个 `cp_ptr_->swing_leg_kp/kd_` 集中所有站立/趴下增益），不是每个状态硬编码。这是值得我们借鉴的——我们当前每个状态写死增益，迁移/调参很麻烦。

---

## 5. RL 与传统运控的衔接（状态切换）

### 5.1 状态机拓扑

```
[Idle] →z→ [StandUp] →c→ [RLControl]
   ↑          ↓               ↓
   ↑          x               x
   ↑       [LieDown] ←────────┘
   ↑          ↓ (2*stand_duration_ 完)
   ↑       [StandUp]
   ↑
[any] →R→ [JointDamping] →3s→ [Idle]
```

### 5.2 StandUp → RLControl 切换的"姿态就绪"判断

`StandUpState::GetNextStateName()`：

```cpp
if (run_time_ - time_stamp_record_ <= 2. * stand_duration_) {
    return StateName::kStandUp;     // 还没站完
} else {
    if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::RLControlMode)) {
        return StateName::kRLControl;
    }
    if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::LieDown)) {
        return StateName::kLieDown;
    }
}
return StateName::kStandUp;          // 默认留在 StandUp
```

**关键**：只有**站起 3 秒完成后**，按 `c` 才会进 RL；站起过程中按 `c` 是无效的。这避免了"还在站起就 RL"导致策略收到"半站"姿态的 obs 而崩。

> 我们框架的切换条件是 `if (stand_up_finished_ && target_mode == RL)`，与 M20 等价。

### 5.3 RL → LieDown 切换

`RLControlState::GetNextStateName()`：

```cpp
if (uc_ptr_->GetUserCommand()->safe_control_mode != 0) return StateName::kJointDamping;
if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::LieDown)) return StateName::kLieDown;
return StateName::kRLControl;
```

直接根据 `target_mode` 切到 LieDown。**不需要"姿态安全就绪"判断**——RL 直接退出 → LieDown 接住 → 折腿到 0.03 m。这与我们规划一致。

### 5.4 LoseControlJudge（异常退出）

每个 state 都有一个 `LoseControlJudge()`，在主循环 tick 中调用：

```cpp
// StateMachineBase::RunThread
if (set_timer.time_interrupt()) {
    ri_ptr_->RefreshRobotData();
    current_controller_->Run();
    if (current_controller_->LoseControlJudge()) {
        next_state_name_ = StateName::kJointDamping;   // 异常 → 阻尼
    } else {
        next_state_name_ = current_controller_->GetNextStateName();
    }
    // ...
}
```

各 state 的 `LoseControlJudge()`：

| State | LoseControlJudge |
|---|---|
| Idle | false（永不触发） |
| StandUp | `target_mode == JointDamping` |
| LieDown | `target_mode == JointDamping` |
| RLControl | `target_mode == JointDamping \|\| PostureUnsafeCheck()` |
| JointDamping | false |

`PostureUnsafeCheck()` 注释里写了 roll/pitch 阈值但被注释掉了 —— **目前永远返回 false**，意味着姿态异常也不会自动退出 RL。
**对我们的影响**：我们框架如果在 sim2real 阶段检测到翻倒，需要把这段逻辑打开。

---

## 6. 安全监控（SafeController，**独立线程**）

`include/utils/safe_controller.hpp` 起一个独立线程，每 1 ms 检查：

| 检查项 | 触发条件 | 处置 |
|---|---|---|
| `IsImuDataNormal` | RPY/ACC/OMG 有 NaN | `safe_control_mode = 2` → state 切到 JointDamping |
| `IsJointDataNormal` | pos/vel/tau 有 NaN | `safe_control_mode = 3` → 同上 |
| `IsDriverStatusNormal` | `status_word[i] != 1` | `safe_control_mode = 2` |
| `IsMotorTempertureNormal` | `max_temp > 100℃` | `safe_control_mode = 2` |
| `IsBatteryNormal` | `level < 10` | `safe_control_mode = 1` |
| `IsUserCommandNormal` | 心跳超时 > 1 s | `safe_control_mode = 2` |

**注意**：`safe_control_mode` 的写者（SafeController）和读者（state 的 `GetNextStateName`）之间没有任何锁——靠 1 s 间隔的超时来兜底，**不严格正确但能跑**。我们框架如果要复用这个机制需要补锁。

`RobotErrorState` 是 32 位 bitmask，定义在 `union` 里（`safe_controller.hpp:36-52`），含 11 个 error 类别。

---

## 7. 与我们框架的迁移映射（速查）

### 7.1 文件级对照

| M20 文件 | 我们对应 | 备注 |
|---|---|---|
| `state_machine/state_machine_base.h` | `state_machine/state_machine.hpp` | 框架都是 timerfd+epoll 5ms tick + state 切换循环 |
| `state_machine/state_base.h` | 同 | StateBase 抽象 |
| `state_machine/parameters/control_parameters.h` | `parameters/Mydog_control_parameters.*` | 单文件参数化 |
| `state_machine/quadruped_wheel/qw_state_machine.hpp` | 同 | 5 个状态 |
| `state_machine/quadruped_wheel/idle_state.hpp` | `state_init.hpp` | 都做传感器检查，**M20 输出全 0 vs 我们输出被动阻尼** |
| `state_machine/quadruped_wheel/standup_state.hpp` | `state_standup.hpp` | **M20 几何反解 vs 我们关键帧** |
| `state_machine/quadruped_wheel/liedown_state.hpp` | `state_sitdown.hpp` | 同上 |
| `state_machine/quadruped_wheel/joint_damping_state.hpp` | `state_damping.hpp` | M20 腿 kd=4 / 轮 kd=1；3 秒后转 Idle |
| `state_machine/quadruped_wheel/rl_control_state.hpp` | `state_rl_onnx.hpp` | **M20 多线程 ONNX vs 我们单线程** |
| `run_policy/policy_runner_base.hpp` | `run_policy/Mydog_test_policy_runner_onnx.hpp` | ONNX 前向 |
| `run_policy/m20_policy_runner.hpp` | 同上 | obs/action 构造逻辑 |
| `include/utils/safe_controller.hpp` | `safe_controller.hpp` | 异常退出 |
| `include/types/common_types.h` | `types/common_types.h` | VecXf/MatXf 别名 + `RobotBasicState`/`RobotAction` |
| `include/types/custom_types.h` | `types/custom_types.h` | 枚举 |
| `interface/user_command/keyboard_interface.hpp` | `keyboard_interface.hpp` | 键盘 → `UserCommand` |
| `interface/user_command/gamepad_interface.hpp` | `gamepad_interface.hpp` | 手柄 |
| `interface/robot/robot_interface.h` | `interface/robot/robot_interface.h` | **接口同**：5 列 SetJointCommand |

### 7.2 关键数值（迁移时直接抄）

```yaml
# M20 默认几何
thigh_len: 0.25
shank_len: 0.25
hip_len: 0.104
body_len_x: 0.619     # 0.3095 * 2
body_len_y: 0.13      # 0.065 * 2
wheel_link_len: 0.040575

# M20 默认站立
pre_height: 0.12      # 第一次站起的中间高度（防止穿地）
stand_height: 0.38     # 最终站立高度
stand_duration: 1.5    # 阶段时长（实际两段 × 1.5 = 3s）

# M20 默认趴下
liedown_height: 0.03   # 折腿高度
liedown_duration: 2.0  # 阶段时长（实际两段 × 2 = 4s）

# M20 StandUp 增益
swing_leg_kp: 200
swing_leg_kd: 4
standup_wheel_kd: 1    # 阶段 2 给轮的弱阻尼

# M20 LieDown 增益（硬编码，不在 control_parameters）
liedown_kp: 300
liedown_kd_leg: 4.5
liedown_kd_wheel: 3

# M20 JointDamping 增益
damping_kd_leg: 4      # = swing_leg_kd
damping_kd_wheel: 1

# M20 RL 增益
rl_kp_leg: 80
rl_kd_leg: 2
rl_kp_wheel: 0         # 实际不写，由 kp=0 + kd=0.6 实现速度环
rl_kd_wheel: 0.6

# M20 obs/action scale
omega_scale: 0.25
dof_vel_scale: 0.05    # 腿和轮都用 0.05
action_scale_leg_hipx: 0.125
action_scale_leg_hipy: 0.25
action_scale_leg_knee: 0.25
action_scale_wheel: 5.0   # 部署实测值（训练侧文档写 20.0）
```

### 7.3 索引映射（迁移必做）

```yaml
# robot_order: 部署端 16 维关节顺序（按 robot_order）
- FL_hipx, FL_hipy, FL_knee, FL_wheel
- FR_hipx, FR_hipy, FR_knee, FR_wheel
- HL_hipx, HL_hipy, HL_knee, HL_wheel
- HR_hipx, HR_hipy, HR_knee, HR_wheel

# policy_order: 策略输入/输出顺序（按 policy_order）
- FL_hipx, FL_hipy, FL_knee
- FR_hipx, FR_hipy, FR_knee
- HL_hipx, HL_hipy, HL_knee
- HR_hipx, HR_hipy, HR_knee
- FL_wheel, FR_wheel, HL_wheel, HR_wheel

# robot2policy_idx: 把 robot_order[i] 映射到 policy_order 的下标
#  FL_hipx(0) → 0, FL_hipy(1) → 1, FL_knee(2) → 2, FL_wheel(3) → 12
#  FR_hipx(4) → 3, FR_hipy(5) → 4, FR_knee(6) → 5, FR_wheel(7) → 13
#  HL_hipx(8) → 6, HL_hipy(9) → 7, HL_knee(10) → 8, HL_wheel(11) → 14
#  HR_hipx(12) → 9, HR_hipy(13) → 10, HR_knee(14) → 11, HR_wheel(15) → 15
```

> 我们的 `JointConfig.policy_index` 字段就是为这个而存在的，**直接照搬 M20 的 `generate_permutation()` 即可**。

### 7.4 obs/action 维度（57 / 16）

| obs 段 | 维度 | 内容 | scale |
|---|---|---|---|
| base_omega | 3 | `IMU ω × 0.25` | 0.25 |
| projected_gravity | 3 | `R⁻¹·[0,0,-1]` | — |
| command | 3 | `[fwd, side, yaw]` | — |
| joint_pos_rl | 16 | `q - q_default`（轮强制 0） | — |
| joint_vel_rl | 16 | `q̇ × 0.05`（腿和轮） | 0.05 |
| last_action | 16 | 前一时刻策略输出 | — |
| **total** | **57** | | |

| action 段 | 维度 | 物理量 | 缩放 |
|---|---|---|---|
| leg_pos（policy[0:12]） | 12 | `Δq_leg` | × [0.125, 0.25, 0.25] |
| wheel_vel（policy[12:16]） | 4 | `v_wheel` | × 5.0 |
| **total** | **16** | | |

---

## 8. 我们迁移时需做决策的点（与 M20 不同的部分）

| # | 决策点 | M20 选择 | 我们的选择 | 备注 |
|---|---|---|---|---|
| 1 | **PD 在哪算** | 硬件 | 上位机 | 维持现状，保留 5 列接口语义兼容 |
| 2 | **Idle 输出** | 全 0 | 被动阻尼 | 我们框架保留阻尼更安全 |
| 3 | **StandUp 轨迹** | 几何反解 | 关键帧 | 改为几何反解更易适配不同 URDF |
| 4 | **站起 kp/kd** | swing_leg_kp=200, kd=0→1 | 视情况 | 按 M20 重参数化 |
| 5 | **LieDown 增益** | 硬编码 kp=300/kd=4.5 | 视情况 | 抽到 control_parameters 里 |
| 6 | **JointDamping 轮 kd** | 1 | 0（旁路） | 我们规划是旁路更安全 |
| 7 | **JointDamping 超时** | 3 秒 | 无 | 沿用 M20 加超时 |
| 8 | **轮 action scale** | 5.0 | 20.0（训练文档） | **需要验证**：可能 M20 内部 retrain 过 |
| 9 | **关节速度 obs scale** | 0.05（统一） | 腿 0.2 / 轮 0.05 | **纠正**：M20 用统一的 0.05 |
| 10 | **轮位置 obs** | 强制 0 | 强制 0 | 一致 |
| 11 | **轮速度 obs** | 保留 | 保留 | 一致 |
| 12 | **last_action 维度** | 16 | 16 | 一致 |
| 13 | **ONNX 推理线程** | 独立线程 + 双缓冲 | 主循环内同步 | M20 性能更好；我们视 CPU 选 |
| 14 | **LoseControlJudge** | 仅 target_mode | target_mode + PostureUnsafeCheck | 我们建议加上 |
| 15 | **safe_controller 锁** | 无 | 视情况 | 补 std::mutex 保险 |
| 16 | **几何约定（后髋膝取反）** | HL/HR hipy & knee 取反 | 视 URDF | 必须严格按 URDF 对齐 |

---

## 9. 关键引用（行号）

| 内容 | 位置 |
|---|---|
| 5 列矩阵定义 | `interface/robot/robot_interface.h:99-104` |
| 状态机主循环 | `state_machine/state_machine_base.h:117-145` |
| 状态切换映射 | `state_machine/quadruped_wheel/qw_state_machine.hpp:92-119` |
| 站起几何反解 | `state_machine/quadruped_wheel/standup_state.hpp:39-63` |
| 站起两段轨迹 | `state_machine/quadruped_wheel/standup_state.hpp:95-144` |
| 趴下目标位姿 | `state_machine/quadruped_wheel/liedown_state.hpp:73-87` |
| 趴下两段轨迹 | `state_machine/quadruped_wheel/liedown_state.hpp:103-130` |
| RL 观测构造 | `run_policy/m20_policy_runner.hpp:203-222` |
| RL 关节索引映射 | `run_policy/m20_policy_runner.hpp:118-119, 142-163` |
| RL 默认位姿 | `run_policy/m20_policy_runner.hpp:95-103` |
| RL action scale | `run_policy/m20_policy_runner.hpp:65-68` |
| RL 解算（action → 矩阵） | `run_policy/m20_policy_runner.hpp:226-236` |
| 双缓冲观测 | `state_machine/quadruped_wheel/rl_control_state.hpp:40-79` |
| M20 控制参数 | `state_machine/parameters/m20_control_parameters.cpp` |
| SafeController 检查 | `include/utils/safe_controller.hpp:120-241` |
| RobotBasicState / RobotAction | `include/types/common_types.h:46-86` |
| UserCommand | `include/types/common_types.h:89-97` |
| 键盘接口 | `interface/user_command/keyboard_interface.hpp:29-37` |
| 立方样条插值 | `include/utils/basic_function.hpp:81-98` |

---

*文档维护者：Claude（自动生成，2026-06-19）*
*基线：M20 SDK Deploy commit 2025-11-07 / 2026-02-12；我们迁移计划 v2026-06-19*
