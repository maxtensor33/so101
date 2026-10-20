# Plan: SO-ARM101 ROS2 + MuJoCo/Gazebo + MoveIt2 + Pinocchio 项目搭建

**日期**: 2026-10-20
**状态**: 已完成（2026-10-20，证据见 .claude/tdd/setup.tdd.md）
**复杂度**: Medium（约 4~6 小时）

## 需求重述

1. 使用 git + uv 管理项目。
2. 先用 Pinocchio 验证 `so101_new_calib.urdf`（ROS2 用）与 `so101_new_calib.xml`（MuJoCo 用）在运动学/关节限位/连杆位姿上是否对齐。
3. 创建 launch demo：启动 ROS2 + RViz，仿真后端可选 Gazebo 或 MuJoCo。

## 环境事实（已勘察确认）

| 项 | 状态 |
|---|---|
| ROS2 | Jazzy（`/opt/ros/jazzy`） |
| MoveIt | 2.12.4 已装全 |
| Pinocchio | `ros-jazzy-pinocchio` 已装（Python 绑定在系统 site-packages） |
| MuJoCo ros2_control | `ros-jazzy-mujoco-ros2-control` + `mujoco-vendor` 已装 |
| Gazebo | Gazebo Harmonic（gz-sim8）+ `ros-jazzy-ros-gz-*` + `gz_ros2_control` 已装 |
| 工具 | git 2.43 / uv 0.12.10 / colcon / Python 3.12 |
| 模型资产 | `/home/maxtensor/Third/Soft/URDF/SO-ARM100/Simulation/SO101/`：`so101_new_calib.urdf`、`so101_new_calib.xml`、`scene.xml`、`assets/*.stl` |
| 关节 | URDF 与 XML 关节名一致：`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` |

## 项目结构（目标）

```
so101/
├── .git/  .gitignore
├── pyproject.toml            # uv 管理：验证脚本 + 后续算法依赖
├── uv.lock
├── plan/setup.md             # 本计划
├── scripts/
│   └── verify_model_alignment.py   # Phase 2 对齐验证脚本
├── reports/alignment_report.md     # 验证结果输出（生成物，入库留档）
├── models/                   # 从 SO-ARM100 拷贝并整理
│   ├── so101_new_calib.urdf
│   ├── so101_new_calib.xml   # MuJoCo 模型（mesh 路径需调整）
│   └── assets/*.stl
└── src/                      # colcon 工作区
    ├── so101_description/    # urdf + meshes + rviz 配置
    ├── so101_bringup/        # launch（demo.launch.py, sim:=gazebo|mujoco|none）
    ├── so101_moveit_config/  # MoveIt2 配置（kinematics, controllers, joint_limits）
    └── so101_sim/            # Gazebo/MuJoCo 仿真资源（world, ros2_control xacro）
```

uv 与 ROS 的兼容策略：`uv venv --system-site-packages`，使 uv 虚拟环境能看到系统的 `pinocchio`（ros-jazzy-pinocchio）与 ROS Python 包；`mujoco`（Python）由 uv 安装进 venv，版本与 mujoco-vendor 对齐（3.x）。

## 实施阶段

### Phase 1：仓库初始化
- `git init` + `.gitignore`（`build/ install/ log/ .venv/ __pycache__/ reports/*.png` 等）
- `uv init` → `pyproject.toml`；依赖：`mujoco>=3.2`（pinocchio 走系统包，不入 pyproject）
- 创建 venv：`uv venv --system-site-packages && uv sync`
- **验证**: `uv run python -c "import pinocchio, mujoco; print(pinocchio.__version__, mujoco.__version__)"`

### Phase 2：URDF ↔ MuJoCo XML 对齐验证（核心交付物）
脚本 `scripts/verify_model_alignment.py`，逐项对比：

1. **关节清单**：名称、顺序、类型（hinge↔revolute）、轴（MuJoCo XML 中各关节 axis 均为 `0 0 1`，但 body quat 已把轴旋转到各关节实际方向——需还原到 URDF 坐标系对比）
2. **关节限位**：MuJoCo `range` ↔ URDF `limit lower/upper`（`gripper` 关节 URDF 与 XML 的正负号方向需重点核对）
3. **连杆树**：`base→shoulder→upper_arm→lower_arm→wrist→gripper→moving_jaw`（XML body 名与 URDF link 名存在差异，如 `moving_jaw_so101_v1`，需建立映射表）
4. **正向运动学一致性（决定性测试）**：随机采样 N=100 组关节角（限位内），分别用
   - Pinocchio：`buildModelFromUrdf` → `forwardKinematics` → 末端 `gripper` frame 位姿
   - MuJoCo：`mj_forward` → 对应 body 的 `xpos/xmat`
   对比每组的平移误差与旋转误差（阈值：平移 < 1mm、旋转 < 0.5°）
5. **惯性/质量**：XML `mass/inertial` ↔ URDF `inertial`
6. 输出 `reports/alignment_report.md`：每项 PASS/FAIL 表格 + 最大误差；若 FAIL，列出具体偏差项与建议修正（修 URDF 或修 XML，以 URDF 为准）

### Phase 3：ROS2 包搭建
- `so101_description`：拷贝 URDF + STL（路径改为 `package://so101_description/meshes/...`），加 `rviz/` 显示配置
- `so101_sim`：`ros2_control` xacro 三套硬件接口：
  - `mock`（无仿真）
  - `gz_ros2_control`（Gazebo Harmonic）
  - `mujoco_ros2_control`（引用 `models/so101_new_calib.xml`， mujoco-ros2-control 直接加载 MuJoCo 模型）
  控制器：`arm_controller`（joint_trajectory_controller，6 关节）+ `gripper_controller`
- `so101_moveit_config`：MoveIt Setup Assistant 风格手写配置（kinematics.yaml、joint_limits.yaml、controllers.yaml 指向上述控制器、planning 生成 `move_group`）

### Phase 4：Launch demo
统一入口 `so101_bringup/demo.launch.py`：

```
ros2 launch so101_bringup demo.launch.py sim:=none     # robot_state_publisher + rviz2 (JointStatePublisher 手动拖动)
ros2 launch so101_bringup demo.launch.py sim:=gazebo   # gz sim + spawn + ros2_control + rviz2
ros2 launch so101_bringup demo.launch.py sim:=mujoco   # mujoco_ros2_control + rviz2
ros2 launch so101_bringup demo.launch.py sim:=none moveit:=true   # 附加 move_group + MoveIt RViz 面板
```

- **验证**：三种 sim 模式分别启动，`ros2 topic echo /joint_states` 有数据、RViz 中机器人模型随命令运动；MoveIt 模式下执行一次规划。

## 依赖关系
Phase 1 → Phase 2 → Phase 3 → Phase 4（Phase 2 结果决定 Phase 3 用哪个 URDF/XML；若 XML 需修正，同步回 `models/`）

## 风险

| 风险 | 概率 | 缓解 |
|---|---|---|
| URDF 与 XML 帧表示不同（quat vs rpy）导致 FK 对比误报 | 高 | 对比用数值结果（末端位姿）而非逐参数 diff；先解析再比对 |
| `gripper` 关节方向/限位符号两模型不一致 | 中 | Phase 2 专项核对，以 URDF 为准修正 XML |
| mujoco Python (uv) 与 mujoco-vendor (apt) 版本不匹配 | 低 | XML 只被 `mujoco_ros2_control`（vendor）加载；验证脚本版本独立，无冲突 |
| uv venv 默认隔离导致 `import pinocchio` 失败 | 高 | `--system-site-packages` 创建 venv（已纳入 Phase 1） |
| MuJoCo XML mesh 路径为相对路径，ros2_control 加载时 cwd 不对 | 中 | `models/` 内改用绝对路径由 CMake/launch 注入，或 launch 设置 `MUJOCO` 资源目录 |

## 验收
- [ ] `uv run python scripts/verify_model_alignment.py` 输出对齐报告（全部 PASS 或明确列出修正项）
- [ ] `sim:=none/gazebo/mujoco` 三种模式 launch 均可启动并在 RViz 正常显示
- [ ] git 提交历史覆盖各 Phase，uv.lock 入库

**等待确认**：是否按此计划执行？（yes / modify: ... ）
