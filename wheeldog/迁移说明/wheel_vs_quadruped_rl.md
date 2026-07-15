# 轮足狗（M20）与四足狗（Lite3）RL 训练框架对比说明

> 目的：把 `/home/gu/桌面/robot_lab/source/robot_lab/robot_lab/` 下四足狗（quadruped / Lite3）和轮足狗（wheeled / M20）的 IsaacLab RL 训练框架差异系统化整理出来，便于后续从四足 → 轮足迁移、复现 sim2real、以及调参时快速 reference。
>
> 关键入口：
> - 四足典型配置：`tasks/manager_based/locomotion/velocity/config/quadruped/deeprobotics_lite3/{rough,flat}_env_cfg.py`
> - 轮足典型配置：`tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/{rough,flat}_env_cfg.py`
> - 共享 MDP：`tasks/manager_based/locomotion/velocity/mdp/`
> - 资产定义（驱动/电机/PD）：`assets/deeprobotics.py`

---

## 0. 总览：一句话区别

| 维度 | 四足狗 Lite3 | 轮足狗 M20 |
|---|---|---|
| 关节拓扑 | 12 关节全 revolute（Hip×4, HipY×4, Knee×4） | 12 腿关节 + **4 个轮关节** = 16 关节 |
| 驱动方式 | **DelayedPD**（带指令时延 0~5 步） | **DCMotor**（腿 PD + 轮 速度环，无位置刚度） |
| Action 空间 | 单一 `joint_pos` × 12 | **混合** `joint_pos`（腿 ×12） + `joint_vel`（轮 ×4） |
| 速度来源 | base IMU 推算 | base IMU 推算；轮速度贡献主要在 wheel action 控制 |
| 训练指令 | lin_vel_x ±1.5, lin_vel_y ±0.8, yaw ±1.5 | 默认未收紧（继承 ±1.0），但模型目标是高速 |
| 终止 | `illegal_contact`（机身接触地面即死） | `illegal_contact = None`（摔倒靠 timeout） |

---

## 1. 电机控制方式（最关键差异）

### 1.1 执行器配置（`assets/deeprobotics.py`）

**Lite3（四足）** —— `DelayedPDActuatorWithBiasCfg`：

```python
"Hip": DelayedPDActuatorWithBiasCfg(
    joint_names_expr=[".*_Hip[X,Y]_joint"],
    effort_limit=24.0, velocity_limit=26.2,
    stiffness=30.0, damping=1.0,
    friction=0.0, armature=0.0,
    min_delay=0, max_delay=5,      # ← 关键：模拟通信延迟 0~5 物理步
),
"Knee": DelayedPDActuatorWithBiasCfg(
    joint_names_expr=[".*_Knee_joint"],
    effort_limit=36.0, velocity_limit=17.3,
    stiffness=30.0, damping=1.0,
    min_delay=0, max_delay=5,
),
```

**M20（轮足）** —— `DCMotorCfg`，腿和轮**两套不同参数**：

```python
"joint": DCMotorCfg(                       # 腿：位置 PD
    joint_names_expr=[".*hipx_joint", ".*hipy_joint", ".*knee_joint"],
    effort_limit=76.4, saturation_effort=76.4,
    velocity_limit=22.4,
    stiffness=80.0, damping=2.0,           # 比 lite3 大（更强）
    friction=0.0,
),
"wheel": DCMotorCfg(                       # 轮：纯速度环
    joint_names_expr=[".*_wheel_joint"],
    effort_limit=21.6, saturation_effort=21.6,
    velocity_limit=79.3,                   # 高速比腿快 3.5 倍
    stiffness=0.0,                         # ← 不做位置控制
    damping=0.6,
),
```

### 1.2 核心差异表

| 项 | Lite3 Hip | Lite3 Knee | M20 腿 | M20 轮 |
|---|---|---|---|---|
| 驱动类型 | DelayedPD | DelayedPD | DCMotor (PD) | DCMotor (速度环) |
| stiffness | 30.0 | 30.0 | **80.0** | **0.0** |
| damping | 1.0 | 1.0 | 2.0 | **0.6** |
| effort_limit (Nm) | 24.0 | 36.0 | **76.4** | 21.6 |
| velocity_limit (rad/s) | 26.2 | 17.3 | 22.4 | **79.3** |
| 指令延迟 | 0~5 步 | 0~5 步 | 0 | 0 |
| 控制模式 | 位置 | 位置 | 位置 | **速度** |

**为什么 M20 刚度大 2.7 倍？** 轮足狗整机重量更大（约 28kg vs Lite3 ~10kg），腿要承担更多负载；同时轮速很快，需要腿快速摆动平衡，所以 PD 增益提高 + 不引入延迟。

### 1.3 动作空间（混合 vs 单一）

**Lite3 单一动作流：**

```python
self.actions.joint_pos = mdp.JointPositionActionCfg(
    asset_name="robot", joint_names=self.joint_names, scale=0.25,
    use_default_offset=True, clip=(-100,100)
)
```

策略输出 12 维 → 全 PD 位置指令（rad）。

**M20 混合动作流（`wheeled/deeprobotics_m20/rough_env_cfg.py:21-31`）：**

```python
class DeeproboticsM20ActionsCfg(ActionsCfg):
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[""],
        scale=0.25, use_default_offset=True, clip=None
    )
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot", joint_names=[""],
        scale=20.0, use_default_offset=True, clip=None     # ← 新增
    )
```

最终每个 step 拼成 12+4 = **16 维 action**。

**Action 处理（`joint_actions.py`）：**

```python
def process_actions(self, actions):
    self._processed_actions = self._raw_actions * self._scale + self._offset
    if self.cfg.clip is not None:
        self._processed_actions = torch.clamp(...)

class JointPositionAction:
    def apply_actions(self):
        # q_des = scale * a + default_joint_pos
        self._asset.set_joint_position_target(q_des, joint_ids=...)

class JointVelocityAction:
    def apply_actions(self):
        # qdot_des = scale * a + default_joint_vel (=0)
        self._asset.set_joint_velocity_target(qdot_des, joint_ids=...)
```

**scale 含义**：
- leg `joint_pos` scale 0.25 → 网络输出 a∈[-1,1] ⇒ 关节位置增量 ±0.25 rad ≈ ±14.3°
- wheel `joint_vel` scale 20.0 → 网络输出 a∈[-1,1] ⇒ 轮速 ±20 rad/s
- 腿 HipX 用更小 scale（0.125），防止髋偏角过快打到限位

### 1.4 轮关节：为什么 stiffness=0？

轮本质上是被动+主动混合执行器：
- `stiffness=0` ⇒ 不施加位置恢复力（PD 第一项 = 0）
- `damping=0.6` ⇒ 提供弱阻尼，让 wheel 速度向目标收敛，但不强制锁相
- 命令以 `set_joint_velocity_target` 进入 PhysX，由 DC motor 的力矩-速度特性曲线自动限幅

这就是 sim2real 的关键：**轮控不要做"位置环"**，否则真机轮碰到台阶打滑时直接锁死。

---

## 2. 力矩计算公式（核心物理）

### 2.1 通用 PD 公式（IdealPDActuator, `actuator_pd.py:148`）

```
τ_j,computed = k_p * (q_des - q) + k_d * (q̇_des - q̇) + τ_ff
τ_j,applied = clip(τ_j,computed, -τ_max, τ_max)
```

`τ_ff` 在 RL 框架下为 0；k_p/k_d = cfg.stiffness/damping。

### 2.2 DelayedPD（Lite3）

唯一区别是先过延迟缓冲（`actuator_pd.py:307`）：

```
positions_delay_buffer[lag] ← command[j]
τ_j = (q_delayed - q)*k_p + (q̇_delayed - q̇)*k_d
```

`lag ∈ Uniform(min_delay, max_delay+1)`，每 env reset 时重采样一次，模拟真实通信抖动。

### 2.3 DCMotor（M20 腿 & 轮）

腿同 IdealPD，但 clip 改为**速度-力矩线性曲线**（`actuator_pd.py:201-304`）：

```
v_at_effort_lim = velocity_limit * (1 + effort_limit / saturation_effort)
τ_max(q̇) = clip(τ_stall * (1 - q̇/v_max),  -∞,  τ_con)
τ_min(q̇) = clip(τ_stall * (-1 - q̇/v_max), -τ_con, ∞)
τ_j,applied = clip(τ_j,computed, τ_min(q̇), τ_max(q̇))
```

物理含义：电机转速越高，可用扭矩越小（功率极限的简化）。当 |q̇| > corner velocity 时，扭矩被夹到 `effort_limit`（连续扭矩）。

**M20 腿参数代入**：
- τ_stall = 76.4 Nm
- v_max = 22.4 rad/s
- τ_con = 76.4 Nm（effort_limit）
- v_corner = 22.4 * (1 + 76.4/76.4) = 44.8 rad/s

也就是说：M20 腿在低速段能输出 76.4Nm，超过 22.4 rad/s 时扭矩线性下降到 0，超过 44.8 rad/s 时锁在 ±76.4Nm。

**M20 轮参数代入**：
- τ_stall = 21.6 Nm, v_max = 79.3 rad/s, v_corner ≈ 158.6 rad/s

轮速高 3.5×，但单轮扭矩低 3.5×，总驱动功率近似一致；这是真实轮毂电机的物理近似。

### 2.4 仿真层力矩-关节运动学

RL 不在策略里显式做前馈力矩/重力补偿，但 PhysX 用 PD 闭环把 q 拉到 q_des，所以策略学到的本质是：
- Lite3：直接控制关节角度（末端位置控制）
- M20 腿：同上，但 PD 增益更高、带宽更宽
- M20 轮：直接控制轮速（轮速 = base_v / r_wheel）

**为什么 M20 不需要显式 Jacobian/动力学？**
RL 自动学到；如果上 sim2real 部署真机，需要做：
1. 前馈重力补偿 τ_g = J^T·G（`new_dog_robot/四足机器人/刚体动力学重力补偿`）
2. 摩擦/库仑补偿 τ_f
3. 关节柔性补偿（前馈观测延迟）

---

## 3. 速度计算

### 3.1 观测到的 base 速度（两套一样）

来自 `isaaclab/envs/mdp/observations.py:53-67`：

```python
def base_lin_vel(env, asset_cfg=SceneEntityCfg("robot")):
    return asset.data.root_lin_vel_b      # body frame

def base_ang_vel(env, asset_cfg=SceneEntityCfg("robot")):
    return asset.data.root_ang_vel_b
```

`root_lin_vel_b` 是 PhysX 在 base link 上积分的线速度（body frame，旋转到机身坐标系）。

### 3.2 命令速度（`commands.py`）

继承自 IsaacLab 的 `UniformVelocityCommand`，并被覆写为 `UniformThresholdVelocityCommand`：

```python
def _resample_command(self, env_ids):
    super()._resample_command(env_ids)
    # 把小幅线速度命令归零（避免抖动）
    self.vel_command_b[env_ids, :2] *= (
        torch.norm(self.vel_command_b[env_ids, :2], dim=1) > 0.2
    ).unsqueeze(1)
```

即 |v_cmd| < 0.2 m/s 时清零，相当于训练时强制"低速命令 = 站立"。

**Lite3 命令范围**（`quadruped_lite3/rough_env_cfg.py:152-154`）：
```python
self.commands.base_velocity.ranges.lin_vel_x = (-1.5, 1.5)
self.commands.base_velocity.ranges.lin_vel_y = (-0.8, 0.8)
self.commands.base_velocity.ranges.ang_vel_z = (-1.5, 1.5)
```

**M20 命令范围**（`wheeled_m20/rough_env_cfg.py:229-231`，注释保留默认值）：
```python
# self.commands.base_velocity.ranges.lin_vel_x = (-1.5, 1.5)
# self.commands.base_velocity.ranges.lin_vel_y = (-1.0, 1.0)
# self.commands.base_velocity.ranges.ang_vel_z = (-1.5, 1.5)
```
即 M20 **未收紧**，使用 `velocity_env_cfg.py:120` 的默认 `(-1.0, 1.0)`。这意味着 M20 训练时最大指令速度只有 1.0 m/s，但策略最终要跑到 2~3 m/s——若要扩域，需要手动放开 ranges。

### 3.3 速度跟踪奖励（核心 reward）

`rewards.py:24-50`：

```python
def track_lin_vel_xy_exp(env, std, command_name, asset_cfg):
    err = sum((cmd[:, :2] - root_lin_vel_b[:, :2])^2, dim=1)
    return exp(-err / std^2)

def track_ang_vel_z_exp(env, std, command_name, asset_cfg):
    err = (cmd[:, 2] - root_ang_vel_b[:, 2])^2
    return exp(-err / std^2)
```

即 `r = exp(-||v_cmd - v||²/σ²)`，σ = √0.25 = 0.5。

**权重对比**：

| 奖励项 | Lite3 | M20 |
|---|---|---|
| `track_lin_vel_xy_exp` | **1.2** | **3.0** |
| `track_ang_vel_z_exp` | **0.6** | **1.5** |

M20 速度跟踪权重约是 Lite3 的 **2.5×**，反映轮足狗的训练更强调高速跟踪；同时也意味着策略对 base 速度误差更敏感。

### 3.4 轮编码器/里程计

**Sim 内**：基座速度由 PhysX 直接积分得到（`root_lin_vel_b`），跟轮编码器无关。**真机上**则需要：

```
v_body_x = (ω_fl + ω_fr + ω_hl + ω_hr) / 4 * r_wheel
ω_yaw   = (ω_r - ω_l) * r_wheel / track_width
```

`new_dog_robot/analysis_wrench_allocator.md` 是基于此做的轮力分配讨论。RL 训练时不显式建模，但 sim2real 时必须保留 Odometry 节点把 wheel ω → base_v。

---

## 4. 核心公式汇总

| 公式 | 四足（Lite3） | 轮足（M20） |
|---|---|---|
| 关节执行器 | DelayedPD | DCMotor |
| **τ 计算** | k_p(q_delayed-q)+k_d(q̇_delayed-q̇) | k_p(q-q)+k_d(q̇-q̇)（腿）/ 0+0.6(q̇-q̇)（轮） |
| **τ 饱和** | clip(τ, -τ_lim, τ_lim) | DC 曲线 clip(τ, τ_min(q̇), τ_max(q̇)) |
| 指令延迟 | 0~5 step | 0 |
| Action | 12 维 q_des | 12 维 q_des + 4 维 q̇_des |
| Action scale | leg 0.25 (HipX 0.125) | leg 0.25 (HipX 0.125), **wheel 20.0** |
| q̇_max rad/s | 26.2 (Hip) / 17.3 (Knee) | 22.4 (腿) / **79.3 (轮)** |
| τ_max Nm | 24/36 | 76.4/21.6 |
| 网络观测 base_lin_vel | **None**（不喂策略） | **None**（不喂策略） |
| 速度跟踪奖励权重 | lin 1.2 / ang 0.6 | **lin 3.0 / ang 1.5** |
| 终止条件 | illegal_contact 摔倒即死 | illegal_contact = None，靠 timeout |

---

## 5. 观测与状态区别

### 5.1 policy / critic 观测（`rough_env_cfg.py`）

**两者都不观测 base_lin_vel、height_scan**（已显式置 None）：

```python
self.observations.policy.base_lin_vel = None
self.observations.policy.height_scan = None
```

**M20 的 joint_pos 观测特殊**：用 `joint_pos_rel_without_wheel`（`observations.py:16`）把 wheel joint 通道强制置 0：

```python
def joint_pos_rel_without_wheel(env, asset_cfg, wheel_asset_cfg):
    joint_pos_rel = asset.data.joint_pos[:, asset_cfg.joint_ids] \
                  - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    joint_pos_rel[:, wheel_asset_cfg.joint_ids] = 0     # 关键
    return joint_pos_rel
```

理由：策略只需要知道腿的相对位置偏差；轮是速度环，位置信息冗余且会随时间漂移（轮可任意转动）。

Lite3 没有这个特殊处理，所有 12 关节都观测 `q - q_default`。

### 5.2 关节速度观测 scale

两者都 `joint_vel.scale = 0.05`（量纲 rad/s × 0.05），但 M20 因含 wheel 速度通道，量级差异大（轮速可达 79 rad/s），需要观察 rollout 时是否饱和。

---

## 6. 奖励函数权重对比

### 6.1 核心权重表（rough_env_cfg）

| 奖励项 | 符号 | Lite3 权重 | M20 权重 | 物理含义 |
|---|---|---|---|---|
| **track_lin_vel_xy_exp** | r_v | 1.2 | **3.0** | 线速度跟踪 |
| **track_ang_vel_z_exp** | r_ω | 0.6 | **1.5** | 角速度跟踪 |
| action_rate_l2 | -0.02 | -0.02 | -0.01 | 动作变化率惩罚 |
| lin_vel_z_l2 | -2.0 | -2.0 | -2.0 | 垂直弹跳 |
| ang_vel_xy_l2 | -0.05 | -0.05 | -0.05 | 横滚/俯仰抖动 |
| joint_torques_l2 | -2.5e-5 | -2.5e-5 | -2.5e-5 (leg) | 关节力矩 L2 |
| joint_acc_l2 | -1e-8 | -1e-8 | -2.5e-7 (leg) / -2.5e-9 (wheel) | 关节加速度 |
| joint_power | -2e-5 | -2e-5 | -2e-5 (leg) | 力矩×速度 |
| flat_orientation_l2 | -5.0 | -5.0 | **0** (M20 取消) | 保持水平 |
| base_height_l2 | -10.0 (target 0.35) | -10.0 (target 0.35) | **0** | 保持高度 |
| feet_air_time | 1.0 | 1.0 | **0** | 腾空时间 |
| feet_air_time_variance | -4.0 | -4.0 | **未设**（默认 0） | 摆腿对称 |
| feet_slide | -0.05 | -0.05 | **0** | 脚滑动 |
| feet_height_body | -2.5 (target -0.35) | -2.5 | **0** | 摆腿高度 |
| feet_height | -0.2 | -0.2 | **0** | 脚离地 |
| contact_forces | -1e-2 | -1e-2 | -1.5e-4 (脚轮) | 接触力峰值 |
| undesired_contacts | -0.5 | -0.5 | -1.0 | 非预期接触 |
| joint_deviation_l1 | -0.5 (HipX) | -0.5 | **未设**（默认 0） | 偏离默认构型 |
| stand_still | -0.3 | -0.3 | **-2.0** | 不动惩罚 |
| joint_pos_penalty | - | - | -1.0 | 关节位置偏差 |
| joint_pos_limits | - | - | -5.0 | 关节位置限位 |
| **joint_mirror** | - | - | **-0.05** | 左右镜像对称 |
| **upward** | - | - | **1.0** | 鼓励朝上 |

### 6.2 关键解读

1. **轮足狗完全取消了所有"步态/抬脚/腾空"奖励**（feet_air_time、feet_height、feet_slide 全置 0）。  
   原因：轮不需要周期摆腿，机械结构上轮是连续的。

2. **M20 新增三个轮足专属奖励**：
   - `joint_mirror`（-0.05）：FL≈HR、FR≈HL 镜像，鼓励对称步态
   - `upward`（1.0）：`(1 - gz)²` 让投影重力 z 分量保持接近 1，防止翻倒
   - `joint_pos_penalty`（-1.0）：静止时强制腿回到默认站姿构型

3. **M20 stand_still 权重从 -0.3 → -2.0**：轮足狗在 v_cmd ≈ 0 时必须稳定站立（不漂移），惩罚更强。

4. **wheel_vel_penalty**：weight=0（默认关），逻辑在 `rewards.py:189`：

```python
def wheel_vel_penalty(env, sensor_cfg, command_name, velocity_threshold, command_threshold, asset_cfg):
    in_air = contact_sensor.compute_first_air(env.step_dt)[:, sensor_cfg.body_ids]
    # 当有命令或机身在动 → 惩罚"轮在腾空时还在转"（打滑）
    # 当无命令 → 惩罚"轮任意转动"（不允许溜车）
    reward = where(cmd>v_thr OR body_vel>v_thr, in_air*|q̇|, |q̇|)
    return reward
```

这个 reward 启动后会让策略避免打滑；当前 weight=0，需要 sim2real 调优时打开。

### 6.3 终止条件

- **Lite3**：`illegal_contact` 启用，机身/非脚部位接触地面即 terminate。
- **M20**：`illegal_contact = None`，完全靠 `time_out` 终止（摔倒就躺到 episode end）。这给策略更多试错空间学翻倒恢复。

---

## 7. 重置与域随机化（events）

### 7.1 初始化姿态

**Lite3**：
```python
"joint_pos": {
    ".*HipX_joint": 0.0,
    ".*HipY_joint": -0.8,        # 站立：后腿略弯
    ".*Knee_joint": 1.6,
}
```
初始高度 z=0.35m。

**M20**：
```python
"joint_pos": {
    ".*hipx_joint": 0.0,
    "f[l,r]_hipy_joint": -0.6,   # 前腿略前伸
    "h[l,r]_hipy_joint": 0.6,    # 后腿略后伸
    "f[l,r]_knee_joint": 1.0,
    "h[l,r]_knee_joint": -1.0,   # 跪式后腿（膝反向）
    ".*wheel_joint": 0.0,
}
```
初始高度 z=0.52m（轮足更"高"，因为轮子离地）。

### 7.2 随机化范围

| 参数 | Lite3 | M20 |
|---|---|---|
| pose x/y | ±1.0 | ±0.5 |
| pose roll/pitch | ±0.3 | **±π（全姿态）** |
| pose yaw | ±π | ±π |
| vel x/y/z | ±0.2 | ±0.5 |
| vel roll/pitch/yaw | ±0.05 | ±0.5 |
| 质量/惯量扰动 | 全 body | base/其他 body 分别随机 |
| actuator gain 扰动 | (0.7, 1.3) | (默认) |
| 推力扰动 | `randomize_push_robot = None` | 默认（10~15s 间隔） |
| 外力扰动 | `randomize_apply_external_force_torque = None` | 默认 |

**注意**：M20 的随机化更激进（roll/pitch/yaw 全部 ±π），这是因为轮足狗要从任意姿态恢复，需要覆盖更多初始姿态。

---

## 8. PPO 训练超参（`agents/rsl_rl_ppo_cfg.py`）

两者**完全一致**：

| 超参 | 值 |
|---|---|
| num_steps_per_env | 24 |
| max_iterations | 20000 (rough) / 5000 (flat) |
| init_noise_std | 1.0 |
| actor/critic dims | [512, 256, 128] |
| activation | elu |
| clip_param | 0.2 |
| entropy_coef | 0.01 |
| num_learning_epochs | 5 |
| num_mini_batches | 4 |
| learning_rate | 1.0e-3 (adaptive) |
| gamma | 0.99 |
| lam | 0.95 |
| desired_kl | 0.01 |
| max_grad_norm | 1.0 |

也就是说，从四足 → 轮足迁移**不需要改 PPO**，主要差异在 env cfg。

---

## 9. sim2real 迁移要点（基于本仓库四足 sim2real 经验推论）

### 9.1 必须新增/修改的项

1. **里程计融合**：`new_dog_robot/standup_to_rl_transition_brief.md` 中的 IMU + 轮编码器 EKF 节点需要在真机上：
   - 把轮编码器 `ω_wheel` 积分成 `v_body_x`
   - IMU yaw rate 与轮差速算的 `ω_yaw` 加权融合
   - 提供 `root_lin_vel_b`、`root_ang_vel_b` 喂观测

2. **重力补偿 + 摩擦前馈**：参考 `new_dog_robot/四足机器人/刚体动力学重力补偿` 和 `前馈力矩test`，策略网络输出的是位置增量 Δq，前馈项需要：

   ```
   τ_ff = J^T(q) * (m*g + m*a_des) + τ_friction(q̇)
   ```

   跟 Lite3 的 DelayedPD 配合即可。

3. **关节延迟**：`max_delay=5` 对 M20 关闭，但真机通信/计算延迟可能仍 1~3ms，需要在 delayed_pd_actuator 中重新评估。

4. **轮 encoder bias**：观察 `delayed_pd_actuator.py` 的 `encoder_bias` 字段。M20 真机上轮编码器零位可能偏差，需要在启动时 calibrate（先把所有轮置零，记下初始编码器读数作为 bias）。

5. **动作空间导出**：导出 ONNX 时注意 policy 现在的输出是 **16 维**（12 + 4），不是 12 维；部署端需要把后 4 维解析成 `wheel_velocity_target` 而不是 `joint_position_target`。

### 9.2 训练-真机 gap 监控项

- base 速度跟踪奖励权重高（M20=3.0），需要监控：
  - 真实 base_lin_vel 与仿真差异（轮打滑）
  - 真机 IMU 噪声（仿真里 base_lin_vel 是干净的）
- wheel_vel_penalty 建议在 sim2real 时打开 weight=-0.1 起步，防止打滑累积

---

## 10. 关键文件位置速查

| 内容 | 路径 |
|---|---|
| Lite3 训练配置 | `robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/quadruped/deeprobotics_lite3/rough_env_cfg.py` |
| M20 训练配置 | `robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/rough_env_cfg.py` |
| 资产/电机定义 | `robot_lab/source/robot_lab/robot_lab/assets/deeprobotics.py` |
| 执行器类型 | RobotLab M20 使用 IsaacLab `DCMotorCfg` |
| 共享 MDP（reward/obs/event） | `robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/mdp/` |
| 通用速度 env 配置 | `robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/velocity_env_cfg.py` |
| PPO 配置 | `.../config/{quadruped/wheeled}/deeprobotics_*/agents/rsl_rl_ppo_cfg.py` |
| URDF | `robot_lab/source/robot_lab/data/Robots/deeprobotics/{lite3,m20}_description/urdf/` |
| 真机控制器参考 | `new_dog_robot/四足机器人/QuardLegAllPrograms/rl/` |
| 重力补偿代码 | `new_dog_robot/四足机器人/刚体动力学重力补偿/` |
| 前馈力矩 test | `new_dog_robot/四足机器人/前馈力矩test/` |
| RL 部署说明 | `new_dog_robot/standup_to_rl_transition_brief.md` |

---

## 11. 速查：迁移 Checklist

- [ ] 把 Lite3 `rough_env_cfg.py` 复制到 `deeprobotics_m20/`，改名 `DeeproboticsM20RoughEnvCfg`
- [ ] `joint_names` 改为 `leg_joint_names + wheel_joint_names`
- [ ] `foot_link_name = ".*_wheel"`（不是 `.*_FOOT`）
- [ ] `base_link_name = "base_link"`（小写，M20 URDF 命名约定）
- [ ] 替换 ActionsCfg 为混合 `JointPositionAction + JointVelocityAction`
- [ ] 把 `joint_pos_rel_without_wheel` 注入 `observations.policy/critic.joint_pos.func`
- [ ] 在 `velocity_env_cfg.py` 的 CommandsCfg 里放开 `lin_vel_x` 上限（如 ±3.0）若需高速
- [ ] 关闭所有 feet_* 奖励，开 `joint_mirror`、`upward`、`joint_pos_penalty`
- [ ] 在 assets/deeprobotics.py 加 `DEEPROBOTICS_M20_CFG`（已有，参考即可）
- [ ] PPO 配置沿用，无需改
- [ ] 导出 ONNX 时确认输出维度 = 16（不是 12）
- [ ] sim2real 阶段把 wheel_vel_penalty weight 调到 -0.1~-0.5

---

文档维护者：Claude (Auto-generated 2026-06-19)
