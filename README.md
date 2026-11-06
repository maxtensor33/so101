# SO101 All In One


基于 **ROS2 Jazzy** 的 SO-ARM101 桌面机械臂工作区，按研究方向划分：
**robotics**（力矩控制教学）→ **mtc**（pick-and-place）→ **envs**（gymnasium 环境集）
→ **rl**（强化学习）→ **il_sim / il_real**（模仿学习）→ **hardware**（实机控制）。

## 方向总览

各方向独立成体系（计划/测试/报告/文档页互不干扰），细节见对应 docs 页。

| 方向 | 定位 | 状态 | 文档 |
|---|---|---|---|
| **robotics** | 力矩控制教学：FK/IK/动力学/轨迹/控制 → draw cube 验收（对标 lab-2，三后端仿真入口） | 可用 | [docs/robotics](docs/robotics/index.md) |
| **mtc** | MoveIt Task Constructor pick-and-place（纯 RViz，扁平 stage 链） | 可用（mujoco/gz 后端暂停） | [docs/mtc](docs/mtc/index.md) |
| **envs** | gymnasium 环境集（纯 MuJoCo 内核）：reach/lift/place/stack × cube/can × 双相机 = 16 id | 可用 | [docs/envs](docs/envs/index.md) |
| **rl** | RL 训练/评估/推理（sb3/skrl 双后端，squint 超参，ROS2 推理节点） | 可用 | [docs/rl](docs/rl/index.md) |
| **il_sim** | 模仿学习仿真（lerobot 0.6.1 官方管线：采集/训练/推理） | 可用 | [docs/il_sim](docs/il_sim/index.md) |
| **il_real** | 真机 leader 臂遥操采集（`so101_leader_sim`） | 采集可用 / 部署待接 | [docs/il_real](docs/il_real/index.md) |
| **hardware** | 实机控制（ros2_control 插件、标定转换、数字孪生、命名姿势） | 可用（真机验收待执行） | [docs/hardware](docs/hardware/index.md) |
| **setup** | 环境搭建与 URDF↔MuJoCo 对齐验证（位置版/力矩版双报告） | 可用 | 本文 + [docs/robotics/usage](docs/robotics/usage.md) |

踩坑日志按方向分类：[docs/pitfalls.md](docs/pitfalls.md)（单源自 `workflow/pitfalls/`）。

## 环境要求

| 组件 | 版本/说明 |
|---|---|
| ROS2 | Jazzy（`ros-jazzy-*`）+ ros2_control / ros2-controllers |
| MoveIt / Gazebo / MuJoCo(ROS) | MoveIt 2.12 · Gazebo Harmonic + gz_ros2_control · mujoco_ros2_control |
| Pinocchio | `pin==3.4.0`（uv 直装, cmeel.prefix; envs/mtc 的 IK 用） |
| Python 工具 | uv ≥ 0.12、Python 3.12（`.python-version`）、colcon |
| Python 包 | numpy 2.x、torch 2.11+cu130（uv find-links 直装）、**lerobot[all]==0.6.1**、mujoco≥3.9（override 锁 3.16 线）、gymnasium≥1.0、stable-baselines3≥2.4、skrl≥1.4，见 `pyproject.toml` |
| GPU（可选） | torch CUDA 用于策略网络训练；MuJoCo 物理在 CPU |

## 快速开始

每方向一条入口命令——完整命令、参数表与已知问题见对应 docs 页。

```bash
# 环境（shell 是 zsh，不要 source setup.bash）
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh

# 构建 + 全量测试
colcon build --symlink-install
uv run pytest

# robotics: 启动 demo（三种仿真 + 可选 MoveIt; 实机 real:=true 见 docs/hardware）
ros2 launch so101_bringup demo.launch.py sim:=none|gazebo|mujoco [moveit:=true]

# robotics: draw cube 力矩验收（参数表见 docs/robotics/usage）
ros2 launch so101_robotics_torque_app draw_cube.launch.py

# envs: 环境集冒烟（16 注册 id; 细则见 docs/envs）
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1'); env.reset(); env.step([0.0]*6)"

# rl: 训练（sb3/skrl 双后端, squint 超参/默认 delta; 细则见 docs/rl）
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Reach-v1 --steps 200000

# il_sim: 专家采集（官方 lerobot-record 薄封装; 细则见 docs/il_sim）
uv run python scripts/il_sim_record.py --robot.type=so101_sim --teleop.type=so101_expert \
  --dataset.repo_id=il_sim/reach_expert --dataset.num_episodes=20

# il_real: leader 臂遥操（探针 → 遥操 → 采集; 见 docs/il_real）
uv run python scripts/il_sim_teleop.py --teleop leader --env reach

# mtc: pick-and-place（纯 RViz; 必须显式 sim:=rviz, launch 默认是暂停的 mujoco）
ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=rviz

# 清理残留进程（每轮测试前必做, 防止旧进程向 /robot_description 发布污染）
bash scripts/clean_ros.sh
```

全量命令速查见 `CLAUDE.md`（AI 用）与各 docs 页（完整参考）。
构建文档站点：`cd docs && make html`（输出 `docs/_build/html/index.html`）。

## 项目结构

| 路径 | 内容 |
|---|---|
| `libs/` | 无 ROS 依赖库：`so101_robotics`（力矩学习）、`so101_envs`（环境集，自含资产）、`so101_rl`（训练） |
| `src/` | colcon 工作区 11 包：description / sim / bringup / moveit_config / robotics_torque_app / envs_viz / rl_infer / teleop / il_sim_infer / mtc_pick_place / hardware |
| `scripts/` | CLI，命名约定 `<域>_<功能>.py`（如 `envs_check_env`、`rl_train`、`il_sim_record`） |
| `models/` | 上游模型副本（位置版 / 力矩版） |
| `outputs/` | 运行产物（gitignore）：RL 训练、数据集、checkpoint |
| [workflow/](workflow/README.md) | 开发流程工件（约定见其 README）：`plan/` 计划、`tdd/` TDD 证据、`reports/` 验收报告、`todo/` 暂停待办、`learn/` 复盘、`problems/` 调试交接、`pitfalls/` 按方向坑（单源进 docs）、`reviews/` 评审 |

## 相关资料

- 机械臂：[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) · [mujoco-robotics-lab](https://github.com/ozkannceylan/mujoco-robotics-lab) · [squint](https://github.com/aalmuzairee/squint)
- ROS 集成参考：[lerobot-ros-moveit](https://github.com/Neoxra/lerobot-ros-moveit)（hardware 方向参考）
- 训练/学习库：[stable-baselines3](https://github.com/DLR-RM/stable-baselines3) · [skrl](https://github.com/skrl-dev/skrl) · [lerobot](https://github.com/huggingface/lerobot)
- ROS 生态：[ros2_control](https://control.ros.org/) · [MoveIt2](https://moveit.picknik.ai/) · [Gazebo Harmonic](https://gazebosim.org/) · [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor)
- 仿真/数学：[MuJoCo](https://mujoco.readthedocs.io/) · [Pinocchio](https://gepettowww.laas.fr/software/pinocchio/)
- 相机、手眼标定：[so101-ros-physical-ai](https://github.com/dmlevi/so101-ros-physical-ai)

## 许可

与上游模型许可一致（SO-ARM100 系列件为开源硬件）；代码部分 MIT。
