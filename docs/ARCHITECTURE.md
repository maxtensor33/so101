# 架构与关键设计

<!-- 手写文档: 与代码演进同步维护; 深入依据见 plan/、reports/、.claude/tdd/ -->

## 全景

```
                     ┌────────────────────────────────────┐
   demo.launch.py ──▶│  so101_bringup (sim:=none/gz/mujoco)│──▶ mock / Gazebo / MuJoCo
                     └────────────────────────────────────┘        │
                     ┌────────────────────────────────────┐        ▼
 draw_cube.launch.py▶│  so101_robotics_torque_app          │   ros2_control (位置 JTC / effort 转发)
                     │  torque_control_node  (100 Hz)     │◀──── /joint_states
                     │  draw_cube_node (IK→traj→执行)     │
                     └─────────┬──────────────────────────┘
                               ▼
                     libs/so101_robotics (无 ROS 依赖)
                     fk · jacobian · ik · dynamics · trajectory
                     · control · constraints · sim
```

- **仿真与规划侧**（`so101_bringup` → `so101_sim`/`so101_description`/
  `so101_moveit_config`）：三后端仿真 + MoveIt2 OMPL 规划。
- **机器人学学习侧**（`so101_robotics_torque_app` + `libs/so101_robotics`）：
  力矩控制全链路，`libs/` 不依赖 ROS，可被后续 RL/IL 项目复用。

## 目录结构

```
so101/
├── CLAUDE.md / README.md        # 工作指引 / 项目概览
├── docs/                        # 命令参考 / runbook / 本文件
├── plan/                        # 原始计划（setup / robotics）
├── .claude/tdd/, .claude/reviews/  # TDD 证据与 review 留档
├── libs/so101_robotics/         # 学习库（hatchling wheel → uv venv）
│   ├── common.py                # 常量、模型加载器（lru_cache 单例）、关节限位
│   ├── fk.py                    # 双引擎 FK + 交叉验证
│   ├── jacobian.py              # 解析/数值 Jacobian、奇异性、manipulability
│   ├── ik.py                    # DLS/伪逆 IK, 自适应阻尼, 随机重启
│   ├── dynamics.py              # rnea/crba/aba, mass_matrix(含 armature)
│   ├── trajectory.py            # quintic/梯形/min-jerk, JointTrajectory
│   ├── control.py               # pd_gravity/computed_torque/task_space_impedance
│   ├── constraints.py           # 力矩饱和 ±3.35 Nm, 关节限位钳位
│   └── sim.py                   # So101Sim 力矩仿真 wrapper（算法层闭环）
├── models/                      # 上游模型副本（对齐验证用）
│   ├── so101_new_calib.urdf/.xml            # 位置控制版
│   ├── so101_torque.urdf / *_torque.xml     # 力矩控制版
│   └── assets/*.stl
├── scripts/                     # verify_model_alignment / draw_cube_viewer / clean_ros
├── reports/                     # 对齐验证与 draw cube 验收报告
├── tests/                       # pytest 102 项（库覆盖率 99%）
└── src/                         # colcon 工作区（5 包）
    ├── so101_description/       # URDF（world 根）+ meshes + rviz
    ├── so101_sim/               # ros2_control 硬件宏（mock/gz/mujoco + effort）+ MJCF
    ├── so101_bringup/           # demo.launch.py
    ├── so101_moveit_config/     # SRDF/kinematics/joint_limits/ompl
    └── so101_robotics_torque_app/  # torque_control_node + draw_cube_node + launch/rviz
```

## 模型体系（xxx_torque 命名）

| 文件 | 作用 |
|---|---|
| `models/so101_new_calib.urdf` / `.xml` | 位置控制版（位置伺服 actuator），三后端仿真用 |
| `models/so101_torque.urdf` / `so101_new_calib_torque.xml` | 力矩版：motor actuator，ctrl 即关节力矩（Nm），±3.35 Nm = STS3215 上限 |
| `src/so101_sim/mujoco/scene_torque.xml` | 力矩版完整场景（地板/灯光），ROS 链路与 viewer 脚本加载 |
| `src/so101_sim/urdf/so101.ros2_control.xacro` | ros2_control 硬件宏 ×4（mock/gz/mujoco + effort 宏） |

对齐验证脚本（`--torque` 开关）保证力矩版与位置版**运动学完全一致**
（FK max 0.0025 mm），仅 actuator 层不同。

## 关键设计决策与依据

- **URDF 根链接是 `world`**：`base_link` 经固定 `world_joint` 挂 world 上。
  URDF→SDF 转换后 base 钉在世界上（否则整机自由浮动倾倒）；MJCF 中 base 本就
  固定，两侧行为一致。
- **ros2_control 硬件宏 ×4**：`mock_components/GenericSystem`、
  `gz_ros2_control/GazeboSimSystem`、`mujoco_ros2_control/MujocoSystemInterface`，
  以及力矩链路的 `so101_joint_interfaces_effort` 宏（6 关节显式 effort command
  + position/velocity state）+ `so101_ros2_control_mujoco_torque` 硬件宏。
- **力矩控制分层**：`torque_control_node`（100 Hz 控制环）订阅
  `/draw_cube/arm_target`（布局 [q_d(5), dq_d(5), ddq_d(5)]），调用
  `libs` 的 computed torque（M·(ddq_d+Kp·e+Kd·ė) + C + g + 被动阻尼补偿）后经
  `forward_command_controller`（`interface_name: effort`，标量）转发 effort 命令。
- **`libs/so101_robotics` 跨环境导入**：hatchling 打包 wheel 安装进 uv venv；
  ROS 节点运行时通过 sys.path 自举（向上找 `pyproject.toml`，插入 `libs/` 与
  `.venv` site-packages）。launch 的 `additional_env` 在 Jazzy 实测不可靠。
- **draw cube 流水线**：12 棱 × 6 航点 → DLS IK（Q_REF 温启动链, max_iters=500,
  n_restarts=10）→ quintic 多段轨迹（段时长按关节距离自适应 `max(seg_dur,
  dist/1.0)`，限关节速度 ~1 rad/s）→ 执行。IK 在回调里阻塞数十秒，执行前经
  one-shot timer 取新鲜 `q_arm` 作为轨迹起点。
- **纯重力补偿是中性稳定**：无位置反馈时保持位形会缓慢漂移，故保持段锁存
  `q_hold` + PD。同理，IK 分支翻转的 1.5+ rad 大段在固定短时长下必然力矩饱和
  ——这是实测 445 mm 失控的根因。
- **RViz 配置源文件优先**：launch 从 install 运行时向上定位 dev 工作区，RViz
  直接加载 `src/` 下 rviz 配置，File→Save 落盘源头；根因是 ament_python
  `data_files` 在 `--symlink-install` 下仍是拷贝（ament_cmake 才有 symlink）。
- **MoveIt 配置经 `MoveItConfigsBuilder`**（MoveIt 2.12 参数名 `planning_plugins`
  复数）。

## 实测基准（reports/draw_cube_report.md）

| 场景 | RMS (mm) | Max (mm) |
|---|---|---|
| ROS 链路 torque（KP/KD=150/15, 100 Hz） | 3.06~3.18 | 6.7~7.8 |
| ROS 链路 torque（KP/KD=50/5） | 13.52 | 95.42 |
| ROS 链路 position 基线（JTC） | 9.67 | 53.66 |
| 库链路 viewer（500 Hz, KP=150/15） | 2.43 | 5.03 |
| 库链路 viewer（KP=300/30） | 1.24 | 2.68 |

ROS 链路同等增益下略差于纯库链路（100 Hz ZOH + 传输延迟），趋势一致。

## 与 lab-2 的对照

对标 [mujoco-robotics-lab lab-2 (UR5e)](https://github.com/ozkannceylan/mujoco-robotics-lab)：
A2/A3/A4/A5 → fk/jacobian/ik/dynamics；B1/B2/B3 → trajectory/control/constraints；
C3 draw cube → draw_cube_node / draw_cube_viewer.py。差异：so101 工作空间小
（cube 6 cm vs UR5e 10 cm）、力矩上限 3.35 Nm、EE 用动爪 frame（掌部对 gripper
关节的 Jacobian 列恒为 0）。
