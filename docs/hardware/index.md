# hardware：实机控制（so101_hardware 插件 + 数字孪生）

**定位**：让 ros2_control 直连 SO-ARM101 实机（Feetech STS3215 串口总线），
含标定转换、命名姿势采集与 MuJoCo 数字孪生镜像。计划与 TDD 证据见
`workflow/plan/setup_hardware.md`、`workflow/tdd/setup_hardware.tdd.md`。

## 构成

```
src/so101_hardware/                     C++ SystemInterface + vendor SCServo SDK
├── src/so101_system.cpp               on_activate 从当前位置起步（无零位要求）
├── config/motor_calibration.yaml      电机 ID/量程（标定转换脚本产物, 唯一真源）
└── urdf/so101.ros2_control_real.xacro plugin + port/calibration_file 参数
src/so101_sim/mujoco_twin.py           数字孪生: 订阅 /joint_states 镜像渲染
scripts/setup_convert_lerobot_calibration.py   LeRobot 标定 JSON → YAML
scripts/setup_capture_named_pose.py            手摆 → 命名姿势 → SRDF + YAML
```

## 命令

```bash
# 标定转换（LeRobot 全行程标定 JSON → motor_calibration.yaml; 方向反了加 --invert <关节>）
uv run python scripts/setup_convert_lerobot_calibration.py [--invert gripper]

# 命名姿势采集（力矩失能手摆 → 命名 → SRDF group_state + named_poses.yaml; 真机）
uv run python scripts/setup_capture_named_pose.py

# 纯实机（无零位要求, 从当前位置起步）
ros2 launch so101_bringup demo.launch.py sim:=none real:=true

# 数字孪生: 实机被控 + MuJoCo 镜像（twin 默认 true）
ros2 launch so101_bringup demo.launch.py sim:=mujoco real:=true [moveit:=true] [twin:=false]

# 可选: port:=/dev/ttyACM0 calibration_file:=<yaml>
# 注意: real:=true + sim:=gazebo 不支持（launch 直接报错退出）
```

## 相机标定（内参 + 手眼）

标定结果用于 sim2real 视野对齐（envs `camera_calib`）和部署侧静态 TF。
完整步骤、`compare_views` 视野对照、坐标系约定与故障排查见
{doc}`camera_calibration`。

```bash
# 内参/手眼标定 → envs 相机定制 → 视野对照（详见 camera_calibration 文档）
uv run python src/so101_camera_calibration/scripts/export_envs_camera_calib.py \
  --handeye outputs/calibration/handeye_eye_in_hand.yaml --intrinsic outputs/calibration/wrist/camera_cal.yaml \
  --handeye outputs/calibration/handeye_eye_to_hand.yaml  --intrinsic outputs/calibration/fixed/camera_cal.yaml
ros2 launch so101_camera_calibration compare_views.launch.py
```

```{toctree}
:hidden:
camera_calibration
```

## 真机验收清单（H7，由用户执行）

低速、避开人员、随时准备断电：

1. 标定转换（`setup_convert_lerobot_calibration.py`）
2. `real:=true sim:=none` — 确认"从当前位置起步"日志、`ros2 control list_controllers` 全 active
3. `sim:=mujoco real:=true` — twin 窗口镜像实机姿态
4. `moveit:=true` — RViz 小步规划到命名姿势，观察实机低速跟随
5. 方向反了 → `--invert <关节>` 转换标定后重启

## 已知限制

- C++ 时序行为（initial-move 锁、read/write 与 JTC 竞争）未经真机验证
- gripper 方向按 SRDF 约定（qpos 大 = 张开）映射，预留 invert 开关
- 深水区坑（xacro bool 转换、park 中断、标定校验等）见
  `workflow/pitfalls/setup_hardware.md`（H1–H11）

## 参考

- 上游参考：[lerobot-ros-moveit](https://github.com/Neoxra/lerobot-ros-moveit)（hardware 方向参考）
- 计划：`workflow/plan/setup_hardware.md`
- 验收报告：`workflow/reports/setup_hardware.md`
- 复盘：`workflow/learn/setup_hardware.md`
- 相关 todo：`workflow/todo/setup_hardware_joint_calibration_todo.md`
