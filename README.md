# SO-ARM101 ROS2 仿真工作区

基于 **ROS2 Jazzy** 的 SO-ARM101（SO-ARM100 迭代款）桌面机械臂完整仿真与规划工作区。
一套代码打通三个仿真后端（**Gazebo Harmonic** / **MuJoCo** / **mock**）与
**MoveIt2 运动规划**，并用 **Pinocchio** 从数学上验证了 URDF 与 MuJoCo 模型的
运动学一致性。统一入口一条命令启动：

```bash
ros2 launch so101_bringup demo.launch.py sim:=gazebo   # 或 mujoco / none
```

## 功能特性

- **模型对齐验证（TDD）**：`scripts/verify_model_alignment.py` 用 Pinocchio 加载
  URDF、MuJoCo Python 加载 MJCF，对关节清单/限位/惯性逐项对比，并以 100 组随机
  关节配置做正向运动学数值对比（决定性测试）。当前结论：**两模型完全对齐**
  （最大平移误差 0.0025 mm，旋转误差 0.0009°，阈值 1 mm / 0.5°）。
- **三后端仿真统一入口**：`sim:=none`（mock 硬件，无仿真器）、`sim:=gazebo`
  （Gazebo Harmonic + gz_ros2_control）、`sim:=mujoco`（mujoco_ros2_control，
  默认加载含地板/灯光的 `scene.xml` 完整场景）。
- **MoveIt2 集成**：`moveit:=true` 附加 move_group 与带 MotionPlanning 面板的
  RViz；OMPL 规划经 KDL 运动学、轨迹下发到 `arm_controller` /
  `gripper_controller`（FollowJointTrajectory）。
- **机械臂在仿真中稳定站立**：URDF 根为 `world` 链接 + 固定关节，URDF→SDF 转换
  后 base 被钉在世界上（与 MuJoCo MJCF 固定 base 行为一致）。
- **开发体验**：uv 管理 Python 工具链（venv 与系统 ROS 包互通）、pytest 全绿
  （覆盖率 88%）、一键清理脚本、TDD 证据留档。

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

## 快速开始

```bash
# 1. 构建
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh

# 2. 三种模式任选
ros2 launch so101_bringup demo.launch.py sim:=none     # mock 硬件 + RViz
ros2 launch so101_bringup demo.launch.py sim:=gazebo   # Gazebo Harmonic + RViz
ros2 launch so101_bringup demo.launch.py sim:=mujoco   # MuJoCo + RViz

# 3. 带 MoveIt 规划
ros2 launch so101_bringup demo.launch.py sim:=none moveit:=true
```

### launch 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `sim` | `none` | `none` / `gazebo` / `mujoco` |
| `moveit` | `false` | 附加 move_group + MoveIt RViz |
| `rviz` | `true` | 启动基础 RViz（moveit:=true 时由 MoveIt 提供 RViz） |
| `gz_gui` | `true` | gazebo 模式：false 为 headless server |
| `mujoco_model_path` | `scene.xml` | 覆盖 MuJoCo 模型（如只加载机器人：`so101_new_calib.xml`） |

### 控制器

| 控制器 | 关节 | 类型 |
|---|---|---|
| `arm_controller` | shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll | joint_trajectory_controller |
| `gripper_controller` | gripper | joint_trajectory_controller |
| `joint_state_broadcaster` | 全部 | — |

MoveIt 规划已验证可用（`/plan_kinematic_path` 返回 SUCCESS）；轨迹会在 RViz
MoveIt 面板中显示，并可经控制器 action 在仿真中执行。控制器状态检查：

```bash
ros2 control list_controllers    # 三个控制器应均为 active
ros2 topic echo /joint_states    # 关节状态数据流
```

## 模型对齐验证

上游仓库（`SO-ARM100/Simulation/SO101`）同时提供 URDF 与 MuJoCo XML，两套模型
若不一致会导致「规划走 A 路径、仿真走 B 路径」。本项目将其拷贝到 `models/` 并
编写了自动化验证（TDD：先写 9 个失败测试，再实现，覆盖率 88%）：

```bash
uv run pytest                                        # 9 passed
uv run python scripts/verify_model_alignment.py      # 生成 reports/alignment_report.md
```

验证维度：关节清单（名/序/类型）、限位（容差 1e-3 rad）、质量/质心（1e-3）、
**正向运动学**（100 组随机配置，Pinocchio FK vs MuJoCo `mj_forward`，阈值
1 mm / 0.5°）。

## 项目结构

```
so101/
├── CLAUDE.md                    # Claude Code 工作指引（精简）
├── pyproject.toml / uv.lock     # uv 管理 Python 工具链
├── plan/setup.md                # 原始搭建计划（已完成）
├── .claude/tdd/setup.tdd.md     # TDD 证据报告（RED/GREEN/覆盖率/冒烟记录）
├── models/                      # 上游模型副本（对齐验证用）
│   ├── so101_new_calib.urdf
│   ├── so101_new_calib.xml
│   └── assets/*.stl
├── scripts/
│   ├── verify_model_alignment.py  # 对齐验证 CLI
│   └── clean_ros.sh               # 一键清理 demo 残留进程
├── reports/alignment_report.md  # 对齐结果报告
├── tests/                       # pytest（9 tests）
└── src/                         # colcon 工作区（4 个包）
    ├── so101_description/       # URDF（world 根）+ meshes + rviz 配置
    ├── so101_sim/               # ros2_control 三硬件 xacro 宏 + MJCF/scene.xml + controllers.yaml
    ├── so101_bringup/           # demo.launch.py 统一入口
    └── so101_moveit_config/     # SRDF / kinematics / joint_limits / ompl / moveit.rviz
```

### 关键设计

- **URDF 根链接是 `world`**：`base_link` 通过固定 `world_joint` 挂在 world 上。
  这是 URDF→SDF 转换后机械臂能在 Gazebo 中稳定站立的原因（否则整机是自由浮
  动体，重力下倾倒）；MuJoCo 的 MJCF 中 base 本就固定，两侧行为因此一致。
- **ros2_control 三套硬件宏**（`so101_sim/urdf/so101.ros2_control.xacro`）：
  `mock_components/GenericSystem`、`gz_ros2_control/GazeboSimSystem`（配
  `<gazebo>` 系统插件块）、`mujoco_ros2_control/MujocoSystemInterface`（配
  `mujoco_model` 参数）。xacro 按 `sim` 参数三选一。
- **MoveIt 配置经 `MoveItConfigsBuilder`** 生成参数（MoveIt 2.12 参数名为
  `planning_plugins` 复数，手工平铺 yaml 会崩）。
- **`/clock` 桥接**：gazebo 模式下 gz 内嵌 controller_manager 使用 sim time，
  由 `ros_gz_bridge` 发布 `/clock`。

## 故障排查（实战踩坑）

| 症状 | 原因与处理 |
|---|---|
| gz 中机械臂倾倒 | URDF 缺 world 固定关节（本项目已修，勿删 `world_joint`） |
| RViz 报 "Frame base_link does not exist" | /joint_states 未就绪或 TF 缺失；等控制器激活（RViz 已延迟 8s 启动） |
| 反复 "No clock received" | /clock 桥接缺失（demo.launch.py 已含 parameter_bridge） |
| gz 加载了错误的硬件插件（mujoco 字样） | 有其它模式的残留进程在发布旧 /robot_description——先 `bash scripts/clean_ros.sh` |
| `import pinocchio` 段错误 | venv 装了 numpy 2.x——保持 `numpy<2` + `--system-site-packages` |
| move_group 启动即崩 "plugin name is empty" | MoveIt 2.12 参数名是 `planning_plugins`（复数），用 MoveItConfigsBuilder |
| ros2 CLI 没反应 | daemon 假死：`ros2 daemon stop && ros2 daemon start` |
| mujoco headless 崩溃 | `export MUJOCO_GL=egl` |

## 开发流程约定

- **所有 git commit 必须先征得维护者同意**（工作流强制）。
- 测试先行：修改 `scripts/` 下的 Python 逻辑请遵循 tests/ 中已有的
  RED→GREEN 模式；证据记入 `.claude/tdd/`。
- 每轮运行测试前先 `bash scripts/clean_ros.sh`，防止残留进程污染
  `/robot_description`。

## 相关资料

- 上游模型来源：[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)（本地
  路径 `/home/maxtensor/Third/Soft/URDF/SO-ARM100`）
- [ros2_control 文档](https://control.ros.org/) ·
  [MoveIt2 文档](https://moveit.picknik.ai/) ·
  [Gazebo Harmonic](https://gazebosim.org/) ·
  [MuJoCo](https://mujoco.readthedocs.io/) ·
  [Pinocchio](https://gepettowww.laas.fr/software/pinocchio/)

## 许可

与上游模型许可一致（SO-ARM100 系列件为开源硬件）；代码部分 MIT。
