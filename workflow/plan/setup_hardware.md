# Plan: setup_hardware — SO-ARM101 实机控制模块（hardware 插件 + 数字孪生 + 命名姿势采集）

**日期**: 2026-10-29
**状态**: 已完成（证据: workflow/tdd/setup_hardware.tdd.md + workflow/reports/setup_hardware.md; 真机验收待用户执行, 清单见 workflow/pitfalls/setup-hardware.md H7）
**复杂度**: High
**参考项目**: `/home/maxtensor/Third/Projects/github/so101/lerobot-ros-moveit`（Neoxra/lerobot-ros-moveit，其 `so101_hardware` 为经过真机验证的 C++ 插件）
**分支策略**: 先 `git checkout setup && git merge main`（setup 是 main 的祖先，快进合并，无冲突），之后全部工作在 setup 分支进行

## 需求重述

1. 新增 `src/so101_hardware` 包：ros2_control `SystemInterface` C++ 硬件插件，直连 Feetech STS3215 串口总线，使 ros_control 能控制实机。
2. `demo.launch.py` 新增 `real:=true` 参数（默认 `false`，手动决定）：
   - `real:=false`（默认）：现有三种仿真行为完全不变（回归保证）。
   - `real:=true, sim:=none`：纯实机模式，`controller_manager/ros2_control_node` 加载实机插件。
   - `real:=true, sim:=mujoco`：**数字孪生模式** —— ros_control 控制实机（实机插件），MuJoCo 仅作纯显示（订阅 `/joint_states` 镜像渲染），不参与控制。
   - `real:=true, sim:=gazebo`：不支持组合，launch 直接报错退出。
3. 新增**命名姿势采集脚本**：操作员手动掰动机械臂各关节（力矩失能状态）→ 调好后按键盘确认 → 脚本询问保存的名字 → 保存该姿势，供 MoveIt 使用（SRDF named group states + YAML）。
4. **不要求手动摆零位**：插件 `on_activate` 读取电机当前位置作为控制起点（对齐参考项目行为，也是上轮讨论确认的关键点）。
5. 标定一次性完成（LeRobot 全行程标定流程），转换脚本生成 `motor_calibration.yaml`，C++ 插件与 Python 脚本共用同一份标定与同一套 rad↔raw 线性映射，保证一致性。

## 环境事实（已勘察确认）

| 项 | 事实 |
|---|---|
| 关节名（顺序即 ros2_control 接口顺序） | `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` |
| 本仓库 URDF 限位（rad） | pan ±1.91986 / lift ±1.74533 / elbow ±1.69 / wrist_flex ±1.65806 / wrist_roll [-2.74385, 2.84121] / gripper [-0.174533, 1.74533]；各关节 axis `0 0 1` |
| SRDF gripper 方向约定 | qpos 大 = 张开（`open`=1.0, `closed`=-0.02） |
| 参考项目核心文件 | `so101_hardware/src/so101_system.cpp`（SystemInterface，内置 SCServo SDK，115200 baud，SyncWritePosEx）；`on_activate` 读当前位置起步 + optional initial-move 锁 read 1000 周期；rad↔raw 为 range→URDF 限位全行程线性映射，**刻意丢弃 homing_offset/drive_mode** |
| 参考项目标定 | LeRobot calibrate 全行程标定生成 JSON → 外部脚本转 YAML（`id/drive_mode/range_min/range_max` per motor） |
| 参考项目缺姿势工具 | SRDF named states（home/close/open）为手写，**没有**采集脚本 —— 本计划新增此能力 |
| 本仓库 hardware 插件先例 | **无**（src/ 下无 hardware_interface 依赖；插件导出写法可类比 `src/so101_mtc_pick_place` 的 `pluginlib_export_plugin_description_file`） |
| 现有三硬件宏 | `src/so101_sim/urdf/so101.ros2_control.xacro`：mock/gazebo/mujoco 三宏 + `so101_joint_interfaces`（position cmd + position/velocity state） |
| demo.launch.py 现状 | `OpaqueFunction(_setup)` 按 `sim` 字符串分支；xacro 只传 `sim:=`；controller spawner 为单一 `TimerAction(5.0)`；`moveit:=true` 时 include `so101_moveit_config/launch/move_group.launch.py` |
| move_group.launch.py 必改点 | 行 40 硬编码 xacro `sim:=none`，real/sim 参数需透传 |
| controllers.yaml | `update_rate: 100`、`use_sim_time: false`（与实机一致，无需改）；`arm_controller`(JTC, 5 关节) + `gripper_controller`(JTC, gripper) + `joint_state_broadcaster` |
| Python 依赖 | `lerobot[all]==0.6.1` 已在 pyproject（含 feetech 总线 SDK，采集脚本直接复用，无需新装包） |
| setup 分支 | main 的祖先，落后 14 提交，尖端 `c55dacf`；`git merge main` 为 fast-forward |
| 测试惯例 | `tests/conftest.py` 集中路径常量；包契约测试风格见 `tests/test_mtc_package.py`（ElementTree/字符串断言）；`uv run pytest` 当前 9 tests |

## 关键设计决策

- **D1 插件路线**：C++ `SystemInterface` + 将参考项目的 SCServo SDK（`SCSerial/SCS/SMS_STS`）源码 vendor 进包。不用 lerobot Python 总线做 ros_control —— Jazzy 无 Python hardware_interface 先例，且参考项目已真机验证。保留 SDK 上游版权头并注明来源。
- **D2 单位映射**：rad↔raw 采用**全行程线性映射**（标定 range_min/range_max → 本仓库 URDF 限位），丢弃 LeRobot 的 homing_offset 与 drive_mode（对齐参考项目的成熟决策）。限位表硬编码于插件中并与本仓库 URDF 一致（注意：与参考项目 URDF 的 elbow/wrist_roll/gripper 限位不同，不能照抄其数值），函数内注释 TODO 后续可改为解析 URDF。
- **D3 无零位要求**：`on_activate` 逐电机读当前位置写入 `hw_positions_` 与 `hw_commands_`，从当前姿态起步。保留可选 `initial_positions_file` 参数（含 initial-move 锁 read 机制，防 JTC 与真实反馈打架），但默认不提供。
- **D4 电机 ID**：从标定 YAML 按 joint 名读取（比参考项目的硬编码更进一步），标定文件即唯一真源。
- **D5 采集脚本后端**：Python 直接用 lerobot 的 `FeetechMotorsBus` 原始读写（Torque_Enable=0 允许手动掰动），**共用 `motor_calibration.yaml` 与同一套线性映射公式**（Python 侧镜像实现），保证脚本读出的 rad 与插件/MoveIt 看到的一致。输出两份产物：`src/so101_moveit_config/config/named_poses.yaml`（机器可读真源）+ **幂等更新** `config/so101.srdf` 的 `<group_state>`（供 MoveIt 直接使用）。
- **D6 数字孪生**：新增 `src/so101_sim/so101_sim/mujoco_twin.py`——`mujoco.viewer.launch_passive` 加载 `src/so101_sim/mujoco/scene.xml`，订阅 `/joint_states` 写 `data.qpos`。real+mujoco 时启动的是**标准** `controller_manager/ros2_control_node`（非 mujoco_ros2_control），twin 节点纯渲染不控制，无时序冲突。
- **D7 MoveIt 联动**：`move_group.launch.py` 将 `sim`/`real` 透传给 xacro；实机模式下 MoveIt 规划即作用于实机（controllers.yaml 的 JTC 名称不变，`moveit_controllers.yaml` 无需改）。

## 项目结构（目标）

```
src/so101_hardware/                    # 新增（ament_cmake, MIT, maintainer maxtensor@163.com）
├── package.xml                        # depends: hardware_interface, pluginlib, rclcpp, rclcpp_lifecycle
├── CMakeLists.txt                     # C++17; 编译 SDK + 插件; pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)
├── so101_hardware.xml                 # 插件描述: so101_hardware/SO101SystemHardware
├── include/so101_hardware/so101_system.hpp
├── src/so101_system.cpp               # on_init/on_configure/on_activate/read/write/on_deactivate
├── scservo_sdk/                       # vendor 自参考项目（SCSerial/SCS/SMS_STS）
├── config/motor_calibration.yaml      # 标定转换脚本产物（模板 + 使用说明注释）
└── urdf/so101.ros2_control_real.xacro # so101_ros2_control_real 宏（port/calibration_file 参数）

src/so101_sim/
├── urdf/so101.ros2_control.xacro      # UPDATE: include real 宏（或直接在此文件加宏）
└── so101_sim/mujoco_twin.py           # 新增: 数字孪生显示节点（launch 可执行）

src/so101_bringup/urdf/so101.urdf.xacro        # UPDATE: real arg + <xacro:if> 分支
src/so101_bringup/launch/demo.launch.py        # UPDATE: real 参数 + 分支 + twin 节点 + gazebo 组合报错
src/so101_moveit_config/launch/move_group.launch.py  # UPDATE: sim/real 透传
src/so101_moveit_config/config/named_poses.yaml        # 新增: 采集脚本产物
src/so101_moveit_config/config/so101.srdf              # UPDATE: 采集脚本幂等追加 <group_state>

scripts/setup_convert_lerobot_calibration.py  # 新增: LeRobot 标定 JSON → motor_calibration.yaml
scripts/setup_capture_named_pose.py           # 新增: 手动掰臂 → 命名 → 保存姿势

tests/test_setup_hardware.py                  # 新增: TDD 契约/单元测试
tests/conftest.py                             # UPDATE: 新增路径常量
CLAUDE.md                                     # UPDATE: 命令区新增条目
workflow/tdd/setup_hardware.tdd.md            # 新增: TDD 证据
workflow/reports/setup_hardware.md            # 新增: 验收报告
workflow/pitfalls/setup-hardware.md           # 新增: 踩坑记录（按 pitfalls 索引惯例）
```

## 实施阶段

### Phase 0：分支准备
- `git checkout setup && git merge main`（快进到 `1477f35`）；`bash scripts/clean_ros.sh`；基线 `colcon build --symlink-install` + `uv run pytest`（9 tests 全绿）。
- **验证**: 基线构建与测试通过。

### Phase 1：so101_hardware 包骨架 + SDK vendor（TDD）
- 先写 `tests/test_setup_hardware.py` 契约测试（package.xml 依赖/插件 xml 类名与基类型/CMakeLists 导出/SDK 文件存在），看其失败。
- 建 `src/so101_hardware` 骨架：package.xml、CMakeLists、so101_hardware.xml、hpp、vendor SCServo SDK 源码。
- **验证**: `uv run pytest`（新增测试转绿）+ `colcon build --packages-select so101_hardware`。

### Phase 2：so101_system.cpp 实现
- 移植参考项目 `so101_system.cpp` 并按 D2/D3/D4 调整：限位表换成本仓库 URDF 数值、电机 ID 从标定 YAML 读、on_activate 读当前位置起步、保留 initial-move 锁机制、SyncWritePosEx 一次写 6 关节（speed 2400 / initial-move 1000，acc 50）。
- **验证**: 编译通过 + 插件可被 `ros2 control list_hardware_components` 发现（mock 冒烟中验证描述解析）。

### Phase 3：标定转换脚本（TDD）
- `scripts/setup_convert_lerobot_calibration.py`：读 LeRobot 标定 JSON（`~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<name>.json`），提取每电机 `id/range_min/range_max`，输出 `motor_calibration.yaml`（drive_mode 保留字段但标注未使用）。
- 单测：内置最小样例 JSON fixture → 断言 YAML 输出结构。
- **验证**: `uv run pytest` + 真实标定文件存在时的转换冒烟（真机步骤，用户执行）。

### Phase 4：URDF / launch 接线（TDD）
- xacro：`so101_ros2_control_real` 宏（plugin `so101_hardware/SO101SystemHardware`，params: `port` 默认 `/dev/ttyACM0`、`calibration_file`）；`so101.urdf.xacro` 加 `real` arg —— `real==true` 时用 real 宏替代对应 sim 宏的 plugin（`sim` 仍决定环境语义）。
- `demo.launch.py`：加 `real` 参数（default false）；`real=true and sim=gazebo` → 报错退出；`real=true and sim=mujoco` → `controller_manager/ros2_control_node` + twin 节点（不启动 mujoco_ros2_control）；`real=true and sim=none` → 标准控制节点 + 实机插件。
- `move_group.launch.py`：`sim`/`real` 透传 xacro（去掉硬编码 `sim:=none`）。
- 测试：xacro 双分支渲染断言（real 真假 → plugin 行）、launch 文件契约断言。
- **验证**: `uv run pytest` + 回归 `ros2 launch so101_bringup demo.launch.py sim:=mujoco moveit:=true`（real 缺省行为不变）。

### Phase 5：mujoco_twin 数字孪生节点
- `src/so101_sim/so101_sim/mujoco_twin.py`：launch_passive 加载 scene.xml，订阅 `/joint_states`（消息驱动 + 兜底 timer），joint 名 → qpos 索引映射，qvel 置零防漂移。
- launch：real+mujoco 分支中作为 Node 启动（可加 `twin:=false` 开关）。
- **验证**: mock 模式手动发布 `/joint_states` 冒烟；真机验收留待用户。

### Phase 6：命名姿势采集脚本（TDD）
- `scripts/setup_capture_named_pose.py`：FeetechMotorsBus 原始读写（力矩失能 → 循环打印当前关节 rad → 回车确认 → 输入名字 → 写 `named_poses.yaml` + 幂等更新 SRDF `<group_state>`，同名的覆盖并先备份）。
- SRDF 更新逻辑抽为纯函数做单测（fixture SRDF → 断言插入/覆盖幂等）。
- **验证**: `uv run pytest`；真机采集为用户执行步骤（文档写清流程）。

### Phase 7：文档与工件
- CLAUDE.md 命令区：标定转换、采集姿势、`real:=true` demo 命令；pitfalls 记录（串口权限 dialout、gripper 方向、initial-move 锁原因）；TDD 证据与验收报告。
- **验证**: `uv run pytest` 全绿 + `colcon build` 全量通过。

## 依赖关系

Phase 0 → 1 → 2 → 4 → 5 → 7（线性主线）；Phase 3 仅依赖 1（标定 YAML 格式定义），Phase 6 仅依赖 3，二者可与主线并行穿插。

## 风险

| 风险 | 概率 | 缓解 |
|---|---|---|
| gripper/关节方向与电机安装镜像不一致（官方 SO-101 gripper 反向，参考项目未做镜像） | 中 | 首次真机联调用低速小步长验证每关节方向；必要时在转换脚本增加 per-motor `invert` 字段（映射函数支持取反 range） |
| 插件 read 与 JTC 反馈打架（参考项目踩坑） | 低 | 仅在使用 initial_positions_file 时才触发 initial-move 锁；默认从当前位置起步无此问题；pitfalls 记录 |
| 本仓库 URDF 限位与参考项目不同导致映射错位 | 低 | D2 明确使用本仓库数值；Phase 4 xacro 渲染断言把关 |
| SCServo SDK 许可证/来源合规 | 低 | 保留上游版权头，models 式来源说明写入 package.xml/README 注释 |
| 串口权限（/dev/ttyACM0 需 dialout 组） | 低 | 文档注明 `sudo usermod -aG dialout $USER` |
| real+mujoco 双进程时序（twin 渲染频率） | 低 | twin 纯订阅渲染不回写；qvel 置零 |
| setup 分支 merge 后推送 origin/setup | 低 | 快进合并，正常 push 即可 |
| 真机验收无法在本会话完成（无实机时） | 中 | mock 全流程替代验证（插件加载、launch 接线、twin 冒烟），真机步骤写成 checklist 由用户执行，报告如实标注未验证项 |

## 验收

- [ ] `colcon build --symlink-install` 全量通过（含新包 so101_hardware）
- [ ] `uv run pytest` 全绿（原 9 tests + 新增 setup_hardware 测试）
- [ ] `real` 缺省时三种仿真行为与 main 完全一致（回归验证 sim:=none/mujoco 各一次）
- [ ] `real:=true sim:=mujoco` 启动：实机插件被 controller_manager 加载、MuJoCo twin 窗口镜像 /joint_states、MoveIt 规划下发到实机（真机项由用户执行）
- [ ] 命名姿势脚本完成一次采集（mock 或真机），SRDF 出现对应 `<group_state>` 且重复采集同名为覆盖
- [ ] 文档（CLAUDE.md / pitfalls / tdd / reports）齐备，SDK 来源合规说明完成
