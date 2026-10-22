# robotics 架构与关键设计

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
  URDF→SDF 转换后 base 钉在世界上；MJCF 中 base 本就固定，两侧行为一致。
- **ros2_control 硬件宏 ×4**：`mock_components/GenericSystem`、
  `gz_ros2_control/GazeboSimSystem`、`mujoco_ros2_control/MujocoSystemInterface`，
  以及力矩链路的 `so101_joint_interfaces_effort` 宏 + `so101_ros2_control_mujoco_torque` 硬件宏。
- **力矩控制分层**：`torque_control_node`（100 Hz）订阅 `/draw_cube/arm_target`
  （[q_d(5), dq_d(5), ddq_d(5)]），调用 `libs` 的 computed torque
  （M·(ddq_d+Kp·e+Kd·ė) + C + g + 被动阻尼补偿）后经
  `forward_command_controller`（`interface_name: effort`，标量）转发。
- **`libs/so101_robotics` 跨环境导入**：hatchling wheel 装进 uv venv；
  ROS 节点 sys.path 自举（向上找 `pyproject.toml`）；launch 的 `additional_env`
  在 Jazzy 实测不可靠。
- **draw cube 流水线**：12 棱 × 6 航点 → DLS IK（Q_REF 温启动链）→ quintic 多段
  轨迹（段时长按关节距离自适应，限速 ~1 rad/s）→ 执行；执行前 one-shot timer
  取新鲜 `q_arm` 作起点。
- **纯重力补偿是中性稳定**：保持段锁存 `q_hold` + PD；IK 分支翻转的 1.5+ rad
  大段在固定短时长下必然力矩饱和（实测 445 mm 失控的根因）。
- **RViz 配置源文件优先**：ament_python `data_files` 在 `--symlink-install` 下仍是拷贝，
  launch 直接加载 `src/` 下 rviz 配置，File→Save 落盘源头。
- **MoveIt 配置经 `MoveItConfigsBuilder`**（MoveIt 2.12 参数名 `planning_plugins` 复数）。

## 实测基准（reports/draw_cube_report.md）

| 场景 | RMS (mm) | Max (mm) |
|---|---|---|
| ROS 链路 torque（KP/KD=150/15, 100 Hz） | 3.06~3.18 | 6.7~7.8 |
| ROS 链路 torque（KP/KD=50/5） | 13.52 | 95.42 |
| ROS 链路 position 基线（JTC） | 9.67 | 53.66 |
| 库链路 viewer（500 Hz, KP=150/15） | 2.43 | 5.03 |
| 库链路 viewer（KP=300/30） | 1.24 | 2.68 |

## 与 lab-2 的对照

A2/A3/A4/A5 → fk/jacobian/ik/dynamics；B1/B2/B3 → trajectory/control/constraints；
C3 draw cube → draw_cube_node / robotics_draw_cube_viewer.py。差异：so101 工作空间小
（cube 6 cm vs UR5e 10 cm）、力矩上限 3.35 Nm、EE 用动爪 frame（掌部对 gripper
关节的 Jacobian 列恒为 0）。
