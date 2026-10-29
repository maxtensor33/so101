# 验收报告: setup_hardware 实机控制模块

日期：2026-10-29 · 分支：setup（快进合并 main 至 1477f35）· 计划：`workflow/plan/setup_hardware.md` · TDD 证据：`workflow/tdd/setup_hardware.tdd.md`

## 交付物

| 组件 | 文件 | 说明 |
|---|---|---|
| 实机插件包 | `src/so101_hardware/` | ament_cmake C++ SystemInterface + vendor SCServo SDK（SCSerial/SCS/SMS_STS，来源 Feetech SCServo_Linux，经 Neoxra/lerobot-ros-moveit 引入）；电机 ID/量程由 `config/motor_calibration.yaml` 驱动；`on_activate` 从当前位置起步（无零位要求）；标定必填校验；per-motor invert 可选 |
| 实机宏 | `src/so101_hardware/urdf/so101.ros2_control_real.xacro` | plugin `so101_hardware/SO101SystemHardware` + port/calibration_file 参数 |
| launch 接线 | `demo.launch.py` / `move_group.launch.py` / `so101.urdf.xacro` | `real:=true`（默认 false）；real+gazebo 报错退出；real+mujoco → 标准 controller_manager + twin；sim/real/port/calibration_file 全链透传 |
| 数字孪生 | `src/so101_sim/mujoco_twin.py` | launch_passive + 订阅 /joint_states 写 qpos、qvel 置零；`twin:=false` 可关 |
| 标定转换 | `scripts/setup_convert_lerobot_calibration.py` | LeRobot 标定 JSON → motor_calibration.yaml（丢 homing_offset；`--invert` 支持） |
| 姿势采集 | `scripts/setup_capture_named_pose.py` | 力矩失能手摆 → 命名 → named_poses.yaml + SRDF 文本级幂等插入 group_state |
| 基线修复 | `src/so101_bringup/CMakeLists.txt` | 移除不存在的 config 安装目录（main 遗留干净构建破损） |

## 验证结果

- `colcon build --symlink-install`：**11 packages finished**（基线 10 + so101_hardware）
- `uv run pytest`：**355 passed**（基线 321 + 新增 34，含 xacro 渲染/真实 MJCF/真实标定集成断言）
- mujoco_twin `ros2 run` 冒烟：节点启动、模型加载、/joint_states 订阅正常
- 采集脚本 dry-run 冒烟：SRDF diff 纯 10 行插入，其余字节不变
- xacro 双分支渲染断言：real:=true → 实机插件；real:=false → 原 mock/mujoco/gz 插件不变

## 真机验收（用户执行，本会话刻意未驱动实机）

1. `uv run python scripts/setup_convert_lerobot_calibration.py`（本机标定 R12552801.json 已验证可转换）
2. `ros2 launch so101_bringup demo.launch.py sim:=none real:=true` — 确认日志"从当前位置起步"、`ros2 control list_controllers` 全 active
3. `sim:=mujoco real:=true` — twin 窗口镜像实机姿态
4. `moveit:=true` — RViz 小步规划到命名姿势，观察实机低速跟随
5. 任一关节方向反了 → 转换标定加 `--invert <关节>` 后重启
6. 安全：实机在旁随时准备断电/按急停；首次联调远离夹爪工作空间

## 已知缺口 / 遗留

- C++ 时序行为（initial-move 锁、read/write 与 JTC 竞争）未真机验证（mock 不可替代串口时序）
- gripper 方向按 SRDF 约定（qpos 大 = 张开）映射，未经真机确认，预留 invert 开关
