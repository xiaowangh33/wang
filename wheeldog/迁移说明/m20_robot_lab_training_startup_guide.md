# M20 训练启动指南（robot_lab 侧）

> 范围：后续 M20 训练统一使用 `/home/gu/桌面/robot_lab`。
> 结论：不要再用 `/home/gu/rl_training` 的 M20 task 作为训练入口。

---

## 0. 为什么统一到 robot_lab

本机现状里有两份 M20：

| 仓库 | M20 task | 状态 |
|---|---|---|
| `/home/gu/桌面/robot_lab` | `RobotLab-Isaac-Velocity-Rough-Deeprobotics-M20-v0` / `RobotLab-Isaac-Velocity-Flat-Deeprobotics-M20-v0` | 推荐使用 |
| `/home/gu/rl_training` | `Rough-Deeprobotics-M20-v0` / `Flat-Deeprobotics-M20-v0` | 不再作为 M20 训练入口 |

关键隐患是：`/home/gu/rl_training/.../deeprobotics_m20/rough_env_cfg.py` 内部实际 import 的是 `robot_lab.tasks...` 和 `robot_lab.assets...`。也就是说，表面上从 `rl_training` 启 M20，运行时却会吃 `robot_lab` 的代码和资产配置。两个包同时安装时，这种交叉引用非常容易让 checkpoint、task ID、asset 参数对不上。

因此后续约定：

1. M20 训练、play、ONNX 导出都从 `robot_lab` 启动。
2. `rl_training` 保留给历史参考或其它 DeepRobotics 训练，不再新增 M20 改动。
3. 部署侧只对齐 RobotLab M20 的 obs/action 定义，见 `utils/robot_lab_obs_alignment.hpp`。

---

## 1. robot_lab M20 关键位置

| 项 | 位置 |
|---|---|
| M20 URDF | `/home/gu/桌面/robot_lab/source/robot_lab/data/Robots/deeprobotics/m20_description/urdf/m20.urdf` |
| M20 meshes | `/home/gu/桌面/robot_lab/source/robot_lab/data/Robots/deeprobotics/m20_description/meshes/` |
| 资产配置 | `/home/gu/桌面/robot_lab/source/robot_lab/robot_lab/assets/deeprobotics.py::DEEPROBOTICS_M20_CFG` |
| rough env | `/home/gu/桌面/robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/rough_env_cfg.py` |
| flat env | `/home/gu/桌面/robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/flat_env_cfg.py` |
| rsl_rl PPO | `/home/gu/桌面/robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/agents/rsl_rl_ppo_cfg.py` |
| cusrl PPO | `/home/gu/桌面/robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/agents/cusrl_ppo_cfg.py` |

---

## 2. 启动训练

推荐用 IsaacLab wrapper 启动，确保 Isaac Sim runtime 被正确加载：

```bash
source /home/gu/anaconda3/etc/profile.d/conda.sh
conda activate env_isaaclab

cd /home/gu/IsaacLab
./isaaclab.sh -p /home/gu/桌面/robot_lab/scripts/reinforcement_learning/rsl_rl/train.py \
    --task RobotLab-Isaac-Velocity-Rough-Deeprobotics-M20-v0 \
    --num_envs 4096 \
    --headless
```

第一次验证建议先跑 flat 小规模：

```bash
cd /home/gu/IsaacLab
./isaaclab.sh -p /home/gu/桌面/robot_lab/scripts/reinforcement_learning/rsl_rl/train.py \
    --task RobotLab-Isaac-Velocity-Flat-Deeprobotics-M20-v0 \
    --num_envs 1024 \
    --headless \
    --max_iterations 100 \
    --video \
    --video_length 100 \
    --video_interval 100
```

常用 task：

| 场景 | task |
|---|---|
| 平地快速验证 | `RobotLab-Isaac-Velocity-Flat-Deeprobotics-M20-v0` |
| rough 正式训练 | `RobotLab-Isaac-Velocity-Rough-Deeprobotics-M20-v0` |

---

## 3. Play 与 ONNX 导出

`robot_lab` 的 rsl_rl `play.py` 会在加载 checkpoint 后自动导出：

- `policy.pt`
- `policy.onnx`

导出目录是 checkpoint 所在目录下的 `exported/`。

```bash
cd /home/gu/IsaacLab
./isaaclab.sh -p /home/gu/桌面/robot_lab/scripts/reinforcement_learning/rsl_rl/play.py \
    --task RobotLab-Isaac-Velocity-Rough-Deeprobotics-M20-v0 \
    --num_envs 4 \
    --checkpoint /path/to/model_20000.pt \
    --video
```

然后把导出的 ONNX 放到部署框架默认路径：

```bash
cp /path/to/exported/policy.onnx /home/gu/wheeldog/policy/policy.onnx
```

部署侧默认从 `policy/policy.onnx` 或 `policy/ppo/policy.onnx` 读取；也可以用环境变量显式指定：

```bash
WHEELDOG_POLICY_PATH=/path/to/policy.onnx /home/gu/wheeldog/run.sh
```

---

## 4. 部署对齐速查

| 项 | RobotLab M20 | 部署侧 |
|---|---|---|
| action 维度 | 16 | 16 |
| obs 维度 | 57 | 57 |
| action 类型 | 12 维腿 `JointPositionAction` + 4 维轮 `JointVelocityAction` | 腿位置目标 + 轮速度目标 |
| leg action scale | hipx=0.125，其余腿关节=0.25 | `robot_model_config.hpp` |
| wheel action scale | 5.0 | `robot_model_config.hpp` |
| base_ang_vel scale | 0.25 | `utils/robot_lab_obs_alignment.hpp` |
| joint_vel scale | 0.05 | `utils/robot_lab_obs_alignment.hpp` |
| wheel position obs | `joint_pos_rel_without_wheel` 置零 | `Mydog_test_policy_runner_onnx.hpp` 置零 |
| policy/control rate | sim dt=0.005, decimation=4, 50 Hz | `robot_model_config.hpp` |

---

## 5. 避免再混用的规则

- M20 文档、脚本、实验名优先写 `robot_lab`，不要写 `rl_training`。
- M20 task ID 必须带 `RobotLab-Isaac-Velocity-` 前缀。
- 如果要修改 M20 env、reward、asset、电机参数，只改 `/home/gu/桌面/robot_lab/source/robot_lab/.../deeprobotics_m20/`。
- 不把 `/home/gu/wheeldog/description/wheeled_mudog.urdf` 覆盖到 `rl_training`；如需替换训练 URDF，也只替换 `robot_lab` 里的 M20 URDF，并先备份。

---

*基线：2026-06-20 本机检查；M20 训练源统一到 robot_lab。*
