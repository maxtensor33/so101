# SO-ARM101 ROS2 仿真工作区

基于 **ROS2 Jazzy** 的 SO-ARM101（SO-ARM100 迭代款）桌面机械臂仿真、规划与
机器人学学习工作区。包含两大部分：

1. **仿真与规划**：一套代码打通三个仿真后端（**Gazebo Harmonic** / **MuJoCo** /
   **mock**）与 **MoveIt2 运动规划**，并用 **Pinocchio** 从数学上验证了 URDF 与
   MuJoCo 模型的运动学一致性。
2. **机器人学学习栈**（力矩控制重点）：`libs/so101_robotics` 无 ROS 依赖库提供
   FK / Jacobian / IK / 动力学 / 轨迹生成 / 控制器族，配套力矩控制版模型
   （`*_torque`）、ROS2 力矩控制链路（ros2_control effort 接口）与 draw cube
   端到端验收（对标 mujoco-robotics-lab lab-2）。

```bash
ros2 launch so101_bringup demo.launch.py sim:=gazebo                          # 仿真 + MoveIt
ros2 launch so101_robotics_torque_app draw_cube.launch.py rviz:=true          # draw cube 验收
uv run python scripts/draw_cube_viewer.py                                     # MuJoCo viewer 版
```

## 功能特性

- **模型对齐验证（TDD）**：`scripts/verify_model_alignment.py` 用 Pinocchio 加载
  URDF、MuJoCo Python 加载 MJCF，对关节清单/限位/惯性逐项对比，并以随机关节配置
  做正向运动学数值对比（决定性测试）。位置版与力矩版均 **完全对齐**
  （最大平移误差 0.0025 mm，旋转误差 0.0009°，阈值 1 mm / 0.5°）。
- **三后端仿真统一入口**：`sim:=none`（mock 硬件，无仿真器）、`sim:=gazebo`
  （Gazebo Harmonic + gz_ros2_control）、`sim:=mujoco`（mujoco_ros2_control，
  默认加载含地板/灯光的 `scene.xml` 完整场景）。
- **MoveIt2 集成**：`moveit:=true` 附加 move_group 与带 MotionPlanning 面板的
  RViz；OMPL 规划经 KDL 运动学、轨迹下发到 `arm_controller` /
  `gripper_controller`（FollowJointTrajectory）。
- **力矩控制学习栈**：位置控制默认模型之外提供 `*_torque` 版（motor actuator，
  ctrl 即关节力矩 Nm，±3.35 Nm 对应 STS3215 舵机上限）；`libs/so101_robotics`
  实现 DLS IK（自适应阻尼 + 随机重启）、RNEA/CRBA/ABA、quintic/梯形/min-jerk
  轨迹、PD+重力 / computed torque / 任务空间阻抗控制器族，Pinocchio 与 MuJoCo
  双引擎交叉验证。
- **draw cube 端到端验收**：12 条棱 × 6 航点 → DLS IK（温启动链）→ quintic 关节
  轨迹 → 力矩执行，RViz 同步显示 cube 线框与实际 EE 轨迹。实测（6 cm 立方体，
  computed torque，KP/KD=150/15）：**EE 跟踪 RMS 3.06~3.18 mm**；position 控制
  基线（JTC）为 9.67 mm。详见 `reports/draw_cube_report.md`。
- **机械臂在仿真中稳定站立**：URDF 根为 `world` 链接 + 固定关节，URDF→SDF 转换
  后 base 被钉在世界上（与 MuJoCo MJCF 固定 base 行为一致）。
- **开发体验**：uv 管理 Python 工具链（venv 与系统 ROS 包互通）、pytest 102 项
  全绿（`libs/so101_robotics` 覆盖率 99%）、一键清理脚本、TDD 证据留档。

## 环境要求

| 组件 | 版本/说明 |
|---|---|
| ROS2 | Jazzy（`ros-jazzy-*`） |
| MoveIt | 2.12（`ros-jazzy-moveit` 全家桶） |
| ros2_control | `ros-jazzy-ros2-control` + `ros-jazzy-ros2-controllers` |
| Gazebo | Harmonic（gz-sim8）+ `ros-jazzy-ros-gz-*` + `gz_ros2_control` |
| MuJoCo | `ros-jazzy-mujoco-ros2-control` + `mujoco-vendor` |
| Pinocchio | `ros-jazzy-pinocchio`（系统 site-packages） |
| 工具 | git、colcon、uv ≥ 0.12、Python ≥ 3.12 |
| Python 包 | mujoco、numpy<2、python-xlib（viewer 窗口最大化）、lark（launch 测试用），见 `pyproject.toml` |

## 快速开始

```bash
source /opt/ros/jazzy/setup.zsh && colcon build --symlink-install && source install/setup.zsh

ros2 launch so101_bringup demo.launch.py sim:=mujoco                        # 三后端仿真 (+moveit:=true 规划)
ros2 launch so101_robotics_torque_app draw_cube.launch.py rviz:=true        # draw cube 力矩验收
uv run python scripts/draw_cube_viewer.py                                   # MuJoCo viewer 版
uv run pytest                                                               # 102 tests
```

完整命令与 launch 参数见 **[docs/USAGE.md](docs/USAGE.md)**。

- 控制器：位置版 `arm_controller`/`gripper_controller` 为
  joint_trajectory_controller；力矩版为 forward_command_controller（effort 转发），
  由 `torque_control_node`（100 Hz）计算力矩。
- 模型对齐验证：Pinocchio(URDF) ↔ MuJoCo(MJCF) 逐项对比 + 随机 FK 数值对比，
  位置版与力矩版均完全对齐（FK max 0.0025 mm）。
- MoveIt 规划已验证可用（`/plan_kinematic_path` 返回 SUCCESS）。

## 文档

| 文档 | 内容 |
|---|---|
| [docs/USAGE.md](docs/USAGE.md) | 命令参考：构建/测试/验证、两个 launch 的全部参数、运行时检查 |
| [docs/RUNBOOK.md](docs/RUNBOOK.md) | 故障排查表（实战踩坑）、启动前检查、已知限制 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 架构全景、目录结构、模型体系（xxx_torque）、设计决策与实测基准 |

## 项目结构

完整目录树、模型体系（xxx_torque 命名）与设计决策见
**[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**。要点：

- `libs/so101_robotics` — 无 ROS 依赖的学习库（fk/jacobian/ik/dynamics/trajectory/
  control/constraints/sim），uv wheel 安装，可脱离 ROS 复用（RL/IL 预留）
- `src/` — colcon 工作区 5 包：description / sim（硬件宏 + MJCF）/ bringup /
  moveit_config / robotics_torque_app（力矩学习 app）
- `models/` — 上游模型副本：位置版 `so101_new_calib.*` 与力矩版 `*_torque.*`
- `docs/` — 命令参考、runbook、架构文档
- `reports/`、`plan/`、`.claude/tdd/` — 验收报告、原始计划、TDD 证据

## 故障排查

详见 **[docs/RUNBOOK.md](docs/RUNBOOK.md)**（13 条实战踩坑 + 启动前检查 +
已知限制）。

## 开发流程约定

- **所有 git commit 必须先征得维护者同意**（工作流强制）。
- 测试先行：修改 `scripts/` 下的 Python 逻辑请遵循 tests/ 中已有的
  RED→GREEN 模式；证据记入 `.claude/tdd/`。
- 每轮运行测试前先 `bash scripts/clean_ros.sh`，防止残留进程污染
  `/robot_description`。

## 相关资料

- 上游模型来源：[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)（本地
  路径 `/home/maxtensor/Third/Soft/URDF/SO-ARM100`）
- 学习栈参考：[mujoco-robotics-lab lab-2 (UR5e)](https://github.com/ozkannceylan/mujoco-robotics-lab)（本地路径 `~/Third/Projects/github/mujoco-robotics-lab/lab-2-Ur5e-robotics-lab`）
- [ros2_control 文档](https://control.ros.org/) ·
  [MoveIt2 文档](https://moveit.picknik.ai/) ·
  [Gazebo Harmonic](https://gazebosim.org/) ·
  [MuJoCo](https://mujoco.readthedocs.io/) ·
  [Pinocchio](https://gepettowww.laas.fr/software/pinocchio/)

## 许可

与上游模型许可一致（SO-ARM100 系列件为开源硬件）；代码部分 MIT。
