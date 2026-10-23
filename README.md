# SO101 All In One

基于 **ROS2 Jazzy** 的 SO-ARM101 桌面机械臂工作区，按研究方向划分：
**robotics**（力矩控制教学）→ **envs**（gymnasium 环境集）→ **rl**（强化学习
训练/推理）→ **il_sim / il_real**（模仿学习，待实现）。文档按方向分页
（Sphinx 构建，参考 stable-baselines3 文档结构）。

```bash
# robotics: 三后端仿真 + MoveIt / draw cube 力矩验收
ros2 launch so101_bringup demo.launch.py sim:=gazebo [moveit:=true]
ros2 launch so101_robotics_torque_app draw_cube.launch.py rviz:=true

# envs: gymnasium 环境集冒烟
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1'); env.reset(); env.step([0.0]*6)"

# rl: 训练 / 推理可视化
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Reach-v1 --steps 200000
ros2 launch so101_rl_infer rl_infer.launch.py env:=reach \
    checkpoint:=outputs/rl/<tag>/latest camera:=true rviz:=true
```

## 方向与功能特性

### robotics — 力矩控制学习栈（教学目的）

对标 [mujoco-robotics-lab lab-2 (UR5e)](https://github.com/ozkannceylan/mujoco-robotics-lab)：

- **模型对齐验证（TDD）**：Pinocchio(URDF) ↔ MuJoCo(MJCF) 逐项对比 + 随机 FK
  数值对比，位置版与力矩版均完全对齐（最大平移误差 0.0025 mm，阈值 1 mm / 0.5°）。
- **三后端仿真统一入口**：`sim:=none`（mock）/ `sim:=gazebo`（Gazebo Harmonic +
  gz_ros2_control）/ `sim:=mujoco`（mujoco_ros2_control）；MoveIt2 集成（OMPL）。
- **力矩控制学习栈**：`libs/so101_robotics` 实现 DLS IK（自适应阻尼 + 随机重启）、
  RNEA/CRBA/ABA、quintic/梯形/min-jerk 轨迹、PD+重力 / computed torque /
  任务空间阻抗控制器族，Pinocchio 与 MuJoCo 双引擎交叉验证；`*_torque` 模型
  ctrl 即关节力矩（±3.35 Nm = STS3215 上限）。
- **draw cube 端到端验收**：12 棱 × 6 航点 → DLS IK（温启动链）→ quintic 轨迹 →
  力矩执行。实测（computed torque，KP/KD=150/15）：**EE 跟踪 RMS 3.06~3.18 mm**；
  position 基线（JTC）9.67 mm。

### envs — gymnasium 环境集（供 RL/IL 训练）

- **8 个注册 id**：reach / lift / place / stack × 无相机（state obs）与带相机
  （fixed 第三人称 + wrist 夹爪跟随，128×128 RGB 双视角成对输出）。
- **纯 MuJoCo 内核**（不经 ros2_control）：位置控制动作空间 Box(-1,1,6)，
  500Hz 仿真 / 10Hz 控制；`gymnasium.utils.env_checker` 全过。
- **配套工具**：单环境验证、脚本专家（pinocchio IK 状态机，reach 100%）、
  mujoco viewer 交互窗口（C 键切视角）、RViz 可视化节点。

### rl — 强化学习训练 / 推理

- **双后端统一封装**：`TrainerSpec` 抽象统一 **sb3**（SAC/PPO/TD3）与
  **skrl**（PPO/SAC），算法注册表 + yaml 超参 + CLI overrides。
- **多环境**：`--n-envs`（sb3 DummyVecEnv / skrl `gym.make_vec`，对齐官方姿势，
  官方文档：DummyVecEnv 通常快于 SubprocVecEnv）；内存红线 ~170MB/env。
- **训练可视化**：tqdm 进度条 + tensorboard（`eval/success_rate` 双后端写入）+
  自动 eval 曲线 `curve.png`。
- **run 管理**：`outputs/rl/<tag>/<时间戳>/` 时间戳分层 + `latest` 指针，
  重复训练不覆盖；checkpoint 带 `meta.json`（后端/env/obs_dim 版本校验）。
- **ROS2 推理**：`so101_rl_infer` 节点加载 checkpoint 驱动 env，
  发布 `/joint_states`、物体 TF、双相机图像 → RViz。
- **lerobot 部署接口**：`SO101DeployPolicy` 协议 + checkpoint 适配器
  （零 lerobot 依赖，真机接线 TODO）。

### il_sim / il_real — 模仿学习（待实现）

仿真数据训练（LeRobotDataset + ACT/Diffusion 候选）与真机遥操作
（leader-follower）管线；前置依赖与草案见对应文档页。

## 环境要求

| 组件 | 版本/说明 |
|---|---|
| ROS2 | Jazzy（`ros-jazzy-*`）+ ros2_control / ros2-controllers |
| MoveIt / Gazebo / MuJoCo(ROS) | MoveIt 2.12 · Gazebo Harmonic + gz_ros2_control · mujoco_ros2_control |
| Pinocchio | `ros-jazzy-pinocchio`（系统 site-packages，venv 需 `--system-site-packages` + `numpy<2`） |
| Python 工具 | uv ≥ 0.12、Python ≥ 3.12、colcon |
| Python 包 | mujoco、gymnasium≥1.0、torch（系统安装，uv 跳过）、stable-baselines3≥2.4、skrl≥1.4、pinocchio，见 `pyproject.toml` |
| GPU（可选） | torch CUDA 用于策略网络训练；MuJoCo 物理在 CPU |

## 快速开始

```bash
source /opt/ros/jazzy/setup.zsh && colcon build --symlink-install && source install/setup.zsh
export MUJOCO_GL=egl        # mujoco headless 渲染必需

uv run pytest               # 220 tests
ros2 launch so101_bringup demo.launch.py sim:=mujoco                 # robotics: 仿真
uv run python scripts/envs_check_env.py --id SO101Reach-v1 --render  # envs: 冒烟
uv run python scripts/rl_train.py --backend sb3 --algo sac --steps 200000  # rl: 训练
```

## 文档

文档按方向划分（Sphinx 构建）：

| 文档 | 内容 |
|---|---|
| [docs/index.md](docs/index.md) | 总览：五方向导航 + 资源表 |
| [docs/robotics/index.md](docs/robotics/index.md) | robotics：定位、构成、页面导航（+ [architecture](docs/robotics/architecture.md) / [usage](docs/robotics/usage.md)） |
| [docs/envs/index.md](docs/envs/index.md) | envs：8 个 id、命令、体系、已知问题 |
| [docs/rl/index.md](docs/rl/index.md) | RL 指南：训练/评估/多环境内存红线/TensorBoard/推理/算法选择 Tips |
| [docs/il_sim/index.md](docs/il_sim/index.md) / [il_real](docs/il_real/index.md) | 模仿学习草案（待实现） |
| [docs/pitfalls.md](docs/pitfalls.md) | 踩坑日志（通用/robotics/envs+rl 分节） |

构建文档站点：`cd docs && make html`（输出 `docs/_build/index.html`）。

## 项目结构

| 路径 | 内容 |
|---|---|
| `libs/so101_robotics/` | robotics 学习库（fk/jacobian/ik/dynamics/trajectory/control/constraints/sim） |
| `libs/so101_envs/` | envs 环境库（自含 MJCF+STL 资产、cameras、randomize、pinocchio IK） |
| `libs/so101_rl/` | rl 训练库（TrainerSpec 双后端、算法注册表、runs 管理、lerobot 部署协议） |
| `src/` | colcon 工作区 7 包：description / sim / bringup / moveit_config / robotics_torque_app / envs_viz / rl_infer |
| `scripts/` | `<项目名>_<功能>.py` 命名：setup_verify_model_alignment、robotics_draw_cube_viewer、envs_{check_env,view_env,run_expert}、rl_{train,eval} |
| `models/` | 上游模型副本（位置版 / 力矩版） |
| `outputs/` | RL 训练产物（gitignore） |
| `plan/` · `reports/` · `todo/` · `learn/` · `.claude/tdd/` | 计划 / 验收报告 / 待办勘察 / 经验复盘 / TDD 证据 |

## 故障排查

详见 **[docs/pitfalls.md](docs/pitfalls.md)**（按方向分节的实战踩坑表）：
启动前检查、gz/MoveIt/ros2_control 常见问题、力矩控制中性稳定、RL 训练静默停止、
多环境 OOM 与 GUI 闪退、gymnasium registry 清空等。

## 开发流程约定

- **所有 git commit 必须先征得维护者同意**（工作流强制）。
- 测试先行（TDD）：证据记入 `.claude/tdd/`；每个方向一份原始计划（`plan/`）。
- 每轮运行 ROS 测试前先 `bash scripts/clean_ros.sh`，防止残留进程污染。
- 脚本命名约定：`<项目名>_<功能>.py`（项目 = plan/ 下的方向名）。

## 相关资料

- 上游模型：[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
- 学习栈参考：[mujoco-robotics-lab lab-2 (UR5e)](https://github.com/ozkannceylan/mujoco-robotics-lab)
- RL 训练参考：[stable-baselines3](https://github.com/DLR-RM/stable-baselines3) · [skrl](https://github.com/skrl-dev/skrl)
- IL 参考：[lerobot](https://github.com/huggingface/lerobot)
- [ros2_control](https://control.ros.org/) 
- [MoveIt2](https://moveit.picknik.ai/) 
- [Gazebo Harmonic](https://gazebosim.org/)
- [MuJoCo](https://mujoco.readthedocs.io/) 
- [Pinocchio](https://gepettowww.laas.fr/software/pinocchio/)

## 许可

与上游模型许可一致（SO-ARM100 系列件为开源硬件）；代码部分 MIT。
