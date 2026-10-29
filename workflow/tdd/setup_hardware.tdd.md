# TDD 证据报告: 实机控制模块（plan/setup_hardware.md）

日期：2026-10-29 · 分支：setup（快进合并 main 至 1477f35）· 运行器：`uv run pytest` + `colcon build`

## 源计划与旅程

`workflow/plan/setup_hardware.md`（用户已确认，AskUserQuestion 决策：main 快进合并进 setup 后在 setup 开发）。journeys：

- 作为操作员，我要 ros_control 直接控制实机（Feetech STS3215），不需要手动摆零位。
- 作为操作员，我要 `real:=true sim:=mujoco` 数字孪生：实机被控的同时 MuJoCo 镜像显示。
- 作为操作员，我要手动掰臂采集命名姿势并用于 MoveIt（SRDF group_state + named_poses.yaml）。
- 作为调试者，我要标定/映射/C++ 插件/Python 脚本四方一致（同一标定 YAML + 同一线性映射公式）。

## 任务 → 测试 → RED/GREEN

| 任务 | 测试 | RED | GREEN |
|---|---|---|---|
| P0 分支准备 | 基线全量 | — | 321 passed + 10 包（顺带修 main 遗留 bringup config 安装破损 H3） |
| P1 包骨架+SDK vendor | TestPackageLayout 5 项 | 5 failed（包不存在） | 5 passed + `colcon build --packages-select so101_hardware` |
| P2 so101_system.cpp 适配 | TestSystemCpp 7 项 | 7 failed（硬编码 ID/参考项目限位） | 12 passed + 编译通过（限位换本仓库 URDF、ID 由标定 YAML 驱动、invert 字段） |
| P3 标定转换脚本 | TestConvertCalibration 4 项 | 4 failed（ModuleNotFoundError） | 16 passed + 真实标定 R12552801.json 冒烟生成 motor_calibration.yaml |
| P4 URDF/launch 接线 | TestLaunchWiring 6 项 + TestXacroRender 3 项 | 7 failed | 24 passed（踩 H1 xacro bool 坑，最小复现定位后改 `${real}` 布尔判断） |
| P5 mujoco_twin | TestMujocoTwin 3 项 | ModuleNotFoundError + launch 断言 | 28 passed + `ros2 run` 冒烟（节点启动、订阅正常；踩 H4/H5） |
| P6 姿势采集脚本 | TestCapturePose 6 项 | 5 failed + 注释丢失回归 | 34 passed + dry-run 冒烟（SRDF diff 纯 10 行插入，踩 H2 改文本级更新） |
| P7 文档工件 | — | — | 全量 355 passed + 11 包构建 |

## 测试规格

| # | 保证 | 测试/命令 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | 包结构/插件注册/CMake 导出/SDK vendor | test_setup_hardware.py::TestPackageLayout | 结构 | PASS |
| 2 | 标定必填、无硬编码 ID、本仓库限位、joint 名索引、initial-move 锁、SyncWrite | TestSystemCpp | 结构 | PASS |
| 3 | 标定转换丢 homing_offset、invert 旗标、真实文件 6 关节 | TestConvertCalibration | 单元 | PASS |
| 4 | real:=true 渲染出实机插件、real 缺省三分支回归 | TestXacroRender（subprocess xacro，无 ROS 时 skip） | 集成 | PASS |
| 5 | real+gazebo 组合报错、move_group 透传、twin 接线 | TestLaunchWiring | 结构 | PASS |
| 6 | twin 关节映射与 qpos 写入/qvel 置零（真实 scene.xml） | TestMujocoTwin | 集成 | PASS |
| 7 | raw→rad 线性映射含 invert/clamp、SRDF 幂等/覆盖/注释保留 | TestCapturePose | 单元 | PASS |
| 8 | 全量回归（原 321 + 新 34） | `uv run pytest` | 回归 | PASS (355) |

## 覆盖与已知缺口

- C++ 插件逻辑验证 = colcon 编译 + 源码契约测试（无 gtest，沿仓库既有约定）；真机时序（read/write 与 JTC 打架、initial-move 锁）**未真机验证**
- 真机验收 = 用户执行，checklist 见 `workflow/pitfalls/setup-hardware.md` H7；本会话刻意未驱动实机（只读采集未执行）
- xacro 渲染测试依赖 ROS 环境，无 xacro 时自动 skip（hermetic 性让位于真实覆盖）

## Merge evidence

commit 待用户批准（仓库纪律）。建议两段：
1. `fix: so101_bringup 安装不存在的 config 目录（干净构建破损）`
2. `feat: so101_hardware 实机插件 + real:=true 数字孪生 + 命名姿势采集（plan/setup_hardware.md）`
