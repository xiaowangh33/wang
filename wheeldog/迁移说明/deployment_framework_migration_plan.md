# 四足 → 轮足 部署框架迁移计划

> 仅做计划与要点说明，**不执行**。范围：上层程序（state machine、policy、observation/action）+ MuJoCo 仿真迁移。实际硬件（EtherCAT + IMU + 电机）暂不动，仅留出虚拟接口、URDF→真机标定表、符号映射表的占位结构。

---

## 0. Context

`/home/gu/new_dog_robot/四足机器人/QuardLegAllPrograms/rl/` 是 46kg 自研四足狗 Mydog 的部署框架。已具备：

- **状态机**：`Idle → StandUp → RL → {JointDamping / SitDown / PACEChirpCollect}`
- **分层频率**：EC 2 kHz / PD 1 kHz（2 分频 ZOH）/ RL ~50 Hz（decimation=20）/ 主循环 500µs
- **仿真/硬件分离**：编译期 `BUILD_SIM` + `USE_MJCPP`/`USE_RAISIM`；运行时统一经 `RobotInterface` 抽象
- **策略推理**：ONNX Runtime，`Mydog_test_policy_runner_onnx` 跑在独立线程
- **标定/映射表**：`Mydog_integrated_hardware_interface.hpp:183-396` 集中 12 个关节的 URDF↔真机映射、encoder 零位、限位、额定扭矩

`/home/gu/桌面/robot_lab/.../deeprobotics_m20/` 的训练侧已先一步迁移到轮足：12 腿 + 4 轮 = 16 关节，混合 action（12 维 `joint_pos` + 4 维 `joint_vel`），`DCMotorCfg`（腿 stiff=80 / 轮 stiff=0）。

**本次目标**：把部署框架（`new_dog_robot/.../rl/`）也迁移到轮足，结构上要兼容自研新轮足（URDF 待定，参考 M20 但不完全等同）。

**用户决策**：
- 目标平台：**自研新轮足**（URDF 占位），不绑死 M20
- 轮关节语义：**纯速度环**（policy 输出 `goal_joint_vel`，不走 PD）
- 不管：重力补偿、摩擦前馈、门控逻辑
- 不管：实际硬件细节；但需留出虚拟接口、URDF→真机标定表、符号映射表的位置

---

## 1. 关键设计决策（已与 Plan agent 校验）

| 决策 | 选型 | 理由 |
|---|---|---|
| `SetJointCommand` 接口 | **保持 5 列不变**，加旁路 actuator-mode 表 | 不破坏现有所有 state / shared memory；速度关节写 `v_des` 直接走 PD 旁路即可（leg 行 PD、wheel 行直通） |
| 硬件抽象层（HAL） | **只声明 header**，sim 实现 stub，hw 留 `// TODO` | 不动现有 `RobotInterface`；将来 HW 端用 HAL 拼装 |
| 关节配置数据 | **手写 `constexpr std::array<JointConfig, N>`**，不做 YAML codegen | 16 关节规模手写最易审查；YAML 运行时加载会引入 yaml-cpp 依赖 |
| DOF count | **`constexpr int kTotalDof` 替换 `#define DOF_COUNT`** | 全扫 `VecXf(12)` 字面量替换 |
| 关节索引空间 | **leg 子空间 [0, N_pos) / wheel 子空间 [N_pos, N_pos+N_wheel)** | leg 内索引 0-11 保持不变，避免破坏 HipY/Knee 索引查找 |
| Wheel obs 是否含位置 | **不包含**（`joint_pos_rel_without_wheel` 把 wheel 位置置零） | 沿用 M20 训练侧 `observations.py:16` 的处理 |
| Wheel obs 是否含速度 | **包含**（`joint_vel` 全维） | 沿用 M20：`joint_vel` 不做零化处理 |
| ONNX action 拆 split | 硬编码 split 点 + 启动时 ONNX 输出维度校验 | retrain 时 split 改变可通过 ONNX metadata 暴露 |
| SafeMode 行为 | per-joint enum：`kLegs`（kp=0, kd=100 阻尼）vs `kWheels`（kp=0, kd=0, v_des=0 主动零速保持） | wheel 走电气再生制动会引发意外 |

---

## 2. 待新增/修改的关键文件

> 文件路径全部相对 `/home/gu/new_dog_robot/四足机器人/QuardLegAllPrograms/rl/`，迁移完成后将整目录拷贝到 `/home/gu/wheeldog/` 下的对应位置。

### 2.1 新增（占位 + 接口）

| 新文件 | 作用 |
|---|---|
| `description/robot_model_config.hpp` | **核心数据表**：`JointConfig[N]` 结构数组 + `kLegDof`/`kWheelDof` 常量 + `ActuatorMode` enum + `SafeModeBehavior` enum。集中存放 URDF↔控制↔硬件三套索引、零位、限位、额定扭矩、默认 PD、action scale 等所有静态信息 |
| `description/wheeled_mudog.urdf`（占位） | 自研新轮足的 URDF。本期暂留 stub（参考 M20 URDF 结构），待真实 URDF 出炉后替换 |
| `description/wheeled_mudog_mujoco.xml`（占位） | MuJoCo 场景。**腿用 position motor，轮用 velocity motor**（`gear=[0,...,1,...,0]`） |
| `interface/robot/hal/motor_hal.hpp` | 单电机 HAL 抽象：`Enable/Disable/SetMode/SetTarget{Pos,Vel,Torque}/GetActual*/GetEncoderCount/GetStatus` |
| `interface/robot/hal/imu_hal.hpp` | IMU HAL 抽象 |
| `interface/robot/hal/ec_bus_hal.hpp` | EtherCAT 总线 HAL 抽象 |
| `interface/robot/hal/sim_motor_hal.cpp` | sim 端 stub 实现：读 MuJoCo `data_->qpos`、写 `data_->ctrl` 或 `qfrc_applied` |

### 2.2 修改（关键路径）

| 文件 | 修改要点 |
|---|---|
| `interface/robot/robot_interface.h` | 在 `RobotInterface` 基类增加 `actuator_modes_` 成员（N 维 enum 数组）、`actuator_safe_modes_` 成员；不修改 `SetJointCommand(matrix)` 签名 |
| `types/common_types.h` | `RobotAction` 结构扩展：增加 `wheel_vel_actions` 字段；`ConvertToMat()` 仍输出 5 列，但 wheel 行的 `goal_joint_pos` 填当前轮位置（占位）、`goal_joint_vel` 填策略输出 |
| `run_policy/Mydog_test_policy_runner_onnx.hpp` | obs/action 维度改 N；leg vs wheel split；`robot_order[]/policy_order[]` 长度改 N；`dof_pos_default_robot[N_pos]`；`action_scale_robot[N]`（leg_pos_scale + wheel_vel_scale 双套）；`kLegDof/kWheelDof` 引用 |
| `utils/robot_lab_obs_alignment.hpp` | `kDimJointPos = N_pos`（腿），`kDimJointVel = N_total`（全），`kDimLastAction = N_total`，`kObsTotal` 重算；新增 `kSplitLegActionDof` 常量（ONNX action 切分点） |
| `interface/robot/simulation/mujoco_interface.hpp` | `ApplyControl()` 内按 `actuator_modes_[i]` 分流：PositionPD 行走原 PD 公式；Velocity 行把 `goal_joint_vel` 直接写到对应 actuator ctrl 索引 |
| `interface/robot/simulation/mujoco_interface.hpp::UpdateJointState()` | 读 N 维关节状态；轮速度从 `data_->qvel` 取 |
| `state_machine/state_machine.hpp` | 把 `#define DOF_COUNT 12` 替换为 `constexpr int kDofCount`（实际值来自 `robot_model_config.hpp`） |
| `state_machine/standup_state.hpp` | 站起轨迹只对 PositionPD 关节生成；wheel 行让 `ApplyControl` 自然忽略 |
| `state_machine/joint_damping_state.hpp` | 按 `actuator_safe_modes_[i]` 分流：腿 = (kp=0, kd=100, q=current)，轮 = (kp=0, kd=0, v_des=0) |
| `state_machine/rl_control_state_onnx.hpp` | obs/action 取数按 leg/wheel 拆分；`UpdateRobotObservation` 重新对齐维度 |
| `state_machine/parameters/Mydog_control_parameters.cpp` | `fl_joint_lower_/fl_joint_upper_/joint_vel_limit_/torque_limit_` 扩到 N_pos 维（仅腿）；新增 wheel 默认参数（v_max 等） |
| `utils/coordinate_transform.hpp` | `kHipYJointIndices[4]` 仍为 leg 内索引（不变）；新增 `kWheelJointIndices[N_wheel]` |
| `share/quardLeg_defines.cc` 的 `init_RobotDataAll()` | 初始化 N 维 `joint_cmd_`、`actuator_modes_`；`kp_base/kd_base` 扩到 N_pos 维 |
| `Sources/robot_state_shm.yaml` | SHM 布局扩到 N：`JointPosition[N]`、`JointVelocity[N]`、`JointTorque[N]`、`JointCmd[N][5]`；新增 `ActuatorMode[N]`（uint8） |

### 2.3 不修改（保留）

- `CMakeLists.txt`：`BUILD_SIM`/`USE_MJCPP`/`USE_RAISIM` 已够用；**不加** `WHEELED_DOG` 选项（避免双形态同 binary 复杂度）
- `Sources/quardLeg.json`、`Sources/quardLeg_mj.json`：进程编排，**不动**
- `main.cpp`、`state_machine.hpp::Run()`：调度逻辑不变
- RoboCore / SOEM / IMU 驱动：本次完全不动

---

## 3. 核心数据流迁移（对照四足现状）

### 3.1 Observation（45 维 → 新维）

```
当前（45）：[base_omega(3), projected_gravity(3), cmd_vel(3),
            joint_pos(12), joint_vel(12), last_action(12)]

新（X 维）：[base_omega(3), projected_gravity(3), cmd_vel(3),
            joint_pos(N_pos),        # 只腿；wheel 位按 joint_pos_rel_without_wheel 置零
            joint_vel(N_total),     # 腿+轮，腿缩放 0.2，轮缩放 0.05
            last_action(N_total)]   # leg_pos_action + wheel_vel_action 拼接
```

**关键**：
- `joint_pos` 取腿子空间，wheel 位写 0（沿用 `joint_pos_rel_without_wheel`）
- `joint_vel` 取全维（**注意**：训练侧 rl_training 的 m20 config 没对 wheel 速度做零化，所以这里保留全维）
- `last_action` 必须与训练时 ONNX 输出一致，导出 ONNX 时需注入 metadata `leg_dof=N_pos`

### 3.2 Action（12 维 → N 维）

策略输出 `[leg_action(N_pos) | wheel_action(N_wheel)]`，长度 `N_total`。

部署端 `GetRobotAction` 拆分：
- leg 部分 → 乘 `action_scale_robot_leg[]`、加 `dof_pos_default_robot[]`、clamp ±1.5 rad → 填到 5 列矩阵的 leg 行的 `goal_joint_pos`；`goal_joint_vel=0`
- wheel 部分 → 乘 `action_scale_robot_wheel[]`（默认 20.0 rad/s） → 填到 wheel 行的 `goal_joint_vel`；`goal_joint_pos` 填当前轮位置（占位）

### 3.3 SetJointCommand → ApplyControl 分流

`MujocoInterface::ApplyControl()` 按行处理：

```cpp
for (int i = 0; i < N_total; ++i) {
    if (actuator_modes_[i] == ActuatorMode::PositionPD) {
        // 原公式
        tau[i] = kp[i]*(q_des[i] - q[i]) + kd[i]*(v_des[i] - v[i]) + tau_ff[i];
        data_->ctrl[actuator_ctrl_idx_[i]] = tau[i];   // position motor
    } else { // Velocity
        // 直接写速度目标
        data_->ctrl[actuator_ctrl_idx_[i]] = v_des[i];   // velocity motor with gear
    }
}
```

> MuJoCo velocity motor 设置示例（写到 MJCF）：
> ```xml
> <motor name="fl_wheel_motor" joint="fl_wheel_joint"
>        gear="0 0 0 0 0 1" ctrllimited="true" ctrlrange="-30 30"/>
> ```

### 3.4 多频率分层（不变）

- EC 2 kHz / PD 1 kHz / RL ~50 Hz：现有架构不变
- 速度环 wheel：与 leg PD 同 1 kHz（ZOH 一次）；驱动内部自己有电流环
- decimation=20：动作触发频率不需改

---

## 4. 仿真迁移要点

### 4.1 URDF / MJCF

- 新增 `description/wheeled_mudog.urdf`（占位）：参考 M20 URDF 结构，但需用户给出实际 URDF
- 新增 `description/wheeled_mudog_mujoco.xml`：
  - 12 个 leg motor：`type="motor"`, `gear=[0,0,0,0,0,0]`（位置，PD 由 C++ 算扭矩后写入 ctrl）
  - 4 个 wheel motor：`gear=[0,0,0,0,0,1]`（速度，ctrl 直接是速度目标）
  - **或**：leg 也用 velocity motor + kp 内置（少写代码但灵活性差）— 暂沿用 C++ 算扭矩方案

### 4.2 MuJoCo 接口

- `ApplyControl()` 分流（见 3.3）
- `UpdateJointState()` 读 N 维（注意 qpos/qvel 索引 7+）
- `UpdateImu()` 不变
- `dt_=0.001`（1 kHz 物理步）不变

### 4.3 URDF→MJCF 转换脚本验证

`utils/convert_urdf_to_mjcf.py` 已在用。**风险点**：wheel joint 在 URDF 里通常是 `continuous` 类型，转换脚本能否正确处理需先用 M20 URDF 验证（即使本期不部署 M20，也建议作为对照测试）。

### 4.4 仿真策略接口一致性测试

迁移完后必须做的最小验收：
1. 用占位 URDF + 占位 MJCF 编译 sim 模式通过
2. 启动 sim、StandUp 成功、键盘进入 RL
3. obs 维度与 ONNX 输入维度一致（启动时校验，失败立即报错）
4. **临时 mock**：可以拿 M20 训练出来的 `policy.onnx` 当对照（即使实际不对应），验证 obs/action 接口管线是否对齐

---

## 5. 硬件抽象层（HAL）— 仅声明，不实现

### 5.1 `motor_hal.hpp`（接口定义）

```cpp
class MotorHAL {
public:
    enum class Mode { Position, Velocity, Torque };
    virtual ~MotorHAL() = default;
    virtual bool Enable() = 0;
    virtual bool Disable() = 0;
    virtual bool SetMode(Mode m) = 0;
    virtual bool SetTargetPos(float rad) = 0;
    virtual bool SetTargetVel(float rad_per_s) = 0;
    virtual bool SetTargetTorque(float nm) = 0;
    virtual float GetActualPos() = 0;
    virtual float GetActualVel() = 0;
    virtual float GetActualTorque() = 0;
    virtual int32_t GetEncoderCount() = 0;
    virtual uint16_t GetStatusWord() = 0;
    virtual bool IsHealthy() = 0;
};
```

### 5.2 `imu_hal.hpp`

```cpp
class ImuHAL {
public:
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual Vec3f GetRpy() = 0;       // rad
    virtual Vec3f GetAcc() = 0;       // m/s^2, body frame
    virtual Vec3f GetOmega() = 0;     // rad/s, body frame
    virtual bool IsDataValid() = 0;
};
```

### 5.3 `ec_bus_hal.hpp`

```cpp
class EcBusHAL {
public:
    virtual bool Init(const char* ifname, int cycle_us) = 0;
    virtual void Shutdown() = 0;
    virtual int CycleOnce() = 0;      // 发送 RxPdo + 接收 TxPdo
    virtual int GetSlaveCount() = 0;
    virtual MotorHAL* GetMotor(int slave_id) = 0;
};
```

### 5.4 sim 端 stub（已实现的部分）

`SimMotorHAL` 在 `sim_motor_hal.cpp` 内：直接读 `mj_data->qpos/qvel`，写 `mj_data->ctrl` 或 `qfrc_applied`。`SimImuHAL` 已存在于 `MujocoInterface::UpdateImu` 中可包一层。

### 5.5 硬件端（TODO，仅占位）

`HardwareMydogInterface` 内部组合 `MotorHAL`（EtherCAT CiA-402 实现）、`ImuHAL`（HI91/HI83 串口解码实现）、`EcBusHAL`（SOEM 实现）。**本期不写，留 `// TODO` 标记**。

---

## 6. URDF ↔ 真机标定表 / 符号映射表

集中在新增的 `description/robot_model_config.hpp`（手写 `constexpr`），结构如下：

```cpp
namespace robot_model {

inline constexpr int kLegDof = 12;            // 占位；最终以自研 URDF 为准
inline constexpr int kWheelDof = 4;
inline constexpr int kTotalDof = kLegDof + kWheelDof;

enum class ActuatorMode : uint8_t { PositionPD, Velocity };
enum class SafeModeBehavior : uint8_t { LegDamping, WheelZeroVel };

struct JointConfig {
    const char* name;                  // "FL_HipX" 等
    int urdf_index;                    // URDF/MJCF 中的索引
    int control_index;                 // SetJointCommand 矩阵行号 = 部署顺序
    int policy_index;                  // ONNX 输入/输出向量中的位置（leg=identity, wheel=offset）
    int hal_index;                     // HAL 数组索引（= EtherCAT slave 或 sim slot）
    ActuatorMode mode;
    SafeModeBehavior safe;
    float sign_to_urdf;                // ±1
    float sign_to_torque;              // ±1
    int32_t encoder_zero_count;        // URDF 零点处编码器读数
    int32_t encoder_min_safety;
    int32_t encoder_max_safety;
    float urdf_lower_rad;
    float urdf_upper_rad;
    float rated_torque_nm;
    float effort_limit_nm;
    float velocity_limit_radps;
    float kp_default;
    float kd_default;
    float action_scale;                // leg=0.125~0.5，wheel=20.0
    float pos_default_rad;             // 站立默认位
};

// 占位数组（用户给真 URDF 时改这里）
inline constexpr std::array<JointConfig, kTotalDof> kJoints = { ... };

}  // namespace robot_model
```

**Mydog 老表 → 新表的迁移对应**：

| Mydog 老位置（`Mydog_integrated_hardware_interface.hpp` 行号） | 新位置（`robot_model_config.hpp` 字段） |
|---|---|
| `JOINT_TO_SLAVE_MAP[12]:183-196` | `kJoints[i].hal_index` |
| `JOINT_ENCODER_TO_URDF_SIGN[12]:314-327` | `kJoints[i].sign_to_urdf` / `sign_to_torque` |
| `CALIB_URDF_RAD[12]:282-295` | 当前全 0；新表保留字段，未来填值 |
| `CALIB_ENCODER_COUNT[12]:296-309` | `kJoints[i].encoder_zero_count` |
| `SAFETY_ENCODER_MIN/MAX:331-358` | `kJoints[i].encoder_min/max_safety` |
| `URDF_JOINT_LOWER/UPPER_RAD:379-390` | `kJoints[i].urdf_lower/upper_rad` |
| `JOINT_URDF_ZERO_OFFSET_RAD:391-396` | 同上合并到 `pos_default_rad` 计算 |
| `RATED_TORQUE_*:255-266` | `kJoints[i].rated_torque_nm` |
| `ENCODER_COUNTS_PER_DEGREE=4369:276` | 全局常量提到 namespace 级 |
| `leg_enabled_[4]:173` | 由 `mode` 隐含（不启用腿 = mode=Velocity 或 disable flag 单独保留） |

迁移时同步把 `share/quardLeg_defines.cc` 内的 `init_RobotDataAll()` 改成读 `robot_model::kJoints`。

---

## 7. 实施阶段（不执行，仅排期）

### Phase 1：数据表与接口（最优先）
- [ ] 写 `description/robot_model_config.hpp`（占位 N=16）
- [ ] 在 `RobotInterface` 基类加 `actuator_modes_[]`、`actuator_safe_modes_[]` 成员
- [ ] 把 `state_machine.hpp` 的 `#define DOF_COUNT 12` 替换为 `constexpr`
- [ ] 全扫 `VecXf(12)`、`array<float, 12>` 字面量替换为 `VecXf(kTotalDof)` 等
- [ ] `share/quardLeg_defines.cc::init_RobotDataAll()` 改用 `kJoints` 初始化

### Phase 2：仿真（独立可验证）
- [ ] 写 `description/wheeled_mudog_mujoco.xml`（占位 16 关节）
- [ ] `MujocoInterface::ApplyControl` 引入 actuator_mode 分流
- [ ] `UpdateJointState` 读 N 维
- [ ] 启动 sim、用 M20 policy.onnx mock 验证 obs/action 接口
- [ ] 验证 `convert_urdf_to_mjcf.py` 处理 wheel continuous joint 正确

### Phase 3：策略层
- [ ] `robot_lab_obs_alignment.hpp` 维度改 N + 加 `kSplitLegActionDof`
- [ ] `Mydog_test_policy_runner_onnx.hpp` obs/action 拆分；ONNX metadata 校验
- [ ] `ConvertToMat()` 输出 5 列（leg/wheel 各自填对应字段）
- [ ] `WarmupObservationOnly` 适配 N 维 `last_action`

### Phase 4：状态机适配
- [ ] `StandUpState`：leg-only 站起轨迹；wheel 留在 0
- [ ] `JointDampingState`：按 `safe_mode` 分流 leg damping / wheel zero-vel
- [ ] `SitDownState`：leg-only 坐下
- [ ] `IdleState` 传感器校验逻辑不变

### Phase 5：HAL 接口声明（不动 HW）
- [ ] `interface/robot/hal/{motor_hal,imu_hal,ec_bus_hal}.hpp`
- [ ] `sim_motor_hal.cpp` stub（替代 `MujocoInterface` 直读 `data_->qpos`）
- [ ] `Mydog_integrated_hardware_interface.hpp` 在 HAL 模式下保留 `// TODO` 占位类

### Phase 6：联调与最小验收
- [ ] sim 模式编译通过
- [ ] StandUp → RL 状态切换正常
- [ ] obs 维度、action split 与 ONNX policy 头部一致（启动时断言）
- [ ] 替换 URDF 后再次验证（用户提供真实 URDF 后）

---

## 8. 难点与风险（提早识别）

| # | 风险 | 缓解 |
|---|---|---|
| 1 | MuJoCo velocity motor 的 `gear` 矩阵写错，轮速度方向反 | 启动时反向跑 1s 自检，断言 `qvel[wheel_idx] * v_des > 0` |
| 2 | obs 中 wheel 速度是否要缩放（M20 用 0.05，leg 用 0.2）— 训练时改过但部署不知道 | obs header 暴露 `kScaleWheelJointVel` 常量，导出 ONNX 时同步 |
| 3 | ONNX action split 点改变后，部署硬编码 split 失效 | `policy_runner_base.hpp::OnEnter` 读 ONNX metadata `split_leg_dof`，与本地 `kSplitLegActionDof` 对比，不一致立即报错 |
| 4 | `StandUpState` 给 wheel 关节也写了 `goal_joint_pos`，被 MuJoCo 误读为位置 | `ApplyControl` 内忽略 velocity-mode 行的 `goal_joint_pos`（推荐） |
| 5 | URDF 的 wheel joint 类型（`continuous` vs `revolute` 无 limit）影响 MuJoCo 模型可加载性 | 用 M20 URDF 先验证 convert 脚本兼容性 |
| 6 | `robot_data_all` SHM 字段增删导致外部进程（web UI）解析失败 | 同步更新 `Sources/robot_state_shm.yaml` 的字段定义；外部进程需要联调 |
| 7 | 新增的 HAL 类被 `state_machine.hpp` 间接 include，编译开销上涨 | 把 HAL 类放独立子目录 `interface/robot/hal/`，按需 include |
| 8 | leg 内 12 关节索引被打乱（如果有人把 wheel 强行塞 leg 索引中） | 强约束：leg 索引 [0,12)，wheel 索引 [12,16)，`robot_order[]` 严格按此排 |
| 9 | 用户给的真实 URDF 与占位 `kJoints` 数组不一致时编译通过但运行错 | 启动时断言 `MJCF nq == urdf_index_max + 7`，并断言每个 joint 名字在 `kJoints` 中能查到 |
| 10 | 自研新轮足与 M20 的状态空间有差异（轮位置约束、电机功率），导致策略直接复用失败 | Phase 1 占位设计时**不绑死 M20**，所有缩放/限位都从 `robot_model_config.hpp` 读 |

---

## 9. 验证方案（本期完成时的最小验收）

1. **编译**：
   - sim 模式（`./build.sh`）通过，无新增 warning
   - hw 模式（`./build.sh hw`）暂不强制（HAL 未实现）；但**编译模板必须通过**（HAL 头存在 + sim stub 实现 + `// TODO` 标记）

2. **仿真运行**：
   - `./run_mj.sh` 启动成功
   - 键盘 `z` 进 StandUp、3 秒后进 idle 待机
   - 键盘 `c` 进 RL，控制台打印 obs 维度（应等于 `kObsTotal`）和 ONNX 输入维度，匹配通过

3. **接口校验**：
   - 启动时断言 `ONNX input dim == kObsTotal`（fail fast）
   - 启动时断言 `ONNX output dim == kTotalDof`（fail fast）
   - 启动时断言 `MJCF nq == kTotalDof + 7`、`nv == kTotalDof + 6`

4. **接口完整性**：
   - `apply_torque_limits` / `undesired_contacts` / `track_lin_vel_xy_exp` 等 reward 暂不跑（仿真阶段不训练），但接口保留

5. **占位接口文档化**：
   - `robot_model_config.hpp` 顶部注释列出"M20 占位值来源 / 自研 URDF 应改哪些字段"
   - HAL 头文件内每个虚函数注释 `// TODO hardware impl` 标注待办

---

## 10. 跨仓库同步说明

迁移完成后：

- `/home/gu/new_dog_robot/四足机器人/QuardLegAllPrograms/rl/` 保留**四足版本**作为参考
- `/home/gu/wheeldog/` 下复制一份 `QuardLegAllPrograms/`，重命名为 `WheeledDogAllPrograms/` 或类似
- 共享文档（迁移说明）放在 `/home/gu/wheeldog/迁移说明/`（已存在）
- 后续若硬件就位，HAL 的 hw 实现放在 `WheeledDogAllPrograms/rl/interface/robot/hardware/` 下，独立 commit
- 与 RobotLab 训练侧的协调：obs/action 维度变更时**两边同步改** `robot_lab_obs_alignment.hpp`（deploy 端） ↔ `robot_lab/.../deeprobotics_m20/rough_env_cfg.py`（train 端）的 joint_names / scales

---

*计划生成时间：2026-06-19*
*范围：仅计划，不执行*
*待执行人：用户*
