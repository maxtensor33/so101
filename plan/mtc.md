# Plan: SO-ARM101 MoveIt Task Constructor (MTC) Pick and Place

> 参考: `~/Third/Projects/github/UR3_ROS2_PICK_AND_PLACE/ur_mtc_pick_place_demo`
> 验收: `mtc_pick_and_place` — RViz 显示 MTC 规划，MuJoCo **与** Gazebo 双后端仿真执行

## 需求重述

在现有 so101 工作区新增基于 MoveIt Task Constructor 的 pick and place 项目：

1. **MTC 任务**：对标参考项目 `mtc_node.cpp` 的 stage 组织
   （CurrentState → open gripper → move to pick → pick 容器
   [generate grasp pose → ComputeIK → approach → CloseGripper → attach]
   → move to place → place 容器 [place pose → retreat → open → detach]）。
2. **双端可见 + 双后端**：不仅 RViz（MotionPlanning + MTC solution 显示）能看到
   MTC 规划，执行也要落到真实仿真。**MuJoCo 与 Gazebo 都实现**，launch 传参
   `sim:=mujoco|gazebo` 决定执行后端（规划与 MTC 节点完全复用，仅执行侧不同）。
3. **验收产物**：一条命令
   `ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=mujoco|gazebo`
   完成「RViz 规划显示 + 仿真中 cube 被夹起并搬到 place 位」，输出执行报告。

## 环境事实（已勘察确认）

| 事实 | 来源 |
|---|---|
| **MTC 依赖已安装**: ros-jazzy-moveit-task-constructor-{core,msgs,visualization,capabilities} 0.1.8 | `apt list --installed` |
| SRDF 已有 `arm` group（chain → gripper_frame_link）、`gripper` group（单 joint）、`end_effector` | `src/so101_moveit_config/config/so101.srdf` |
| MoveIt 控制器: arm/gripper 均 FollowJointTrajectory，与 mujoco 后端 JTC 同名 | `config/moveit_controllers.yaml` |
| `demo.launch.py sim:=mujoco moveit:=true` 已可用（include move_group.launch.py） | `src/so101_bringup/launch/demo.launch.py:179` |
| 参考项目 mtc_node.cpp 为标准 MTC pick+place（含 degenerate trajectory 过滤、grasp IK fallback、retry），执行经 move_group `/execute_task_solution` | `ur_mtc_pick_place_demo/src/mtc_node.cpp`（1662 行） |
| **参考项目已回退 main 干净版**，上述引用行号（217-228 / 1113 / 1335-1349）已逐一复核存在 | git status + sed 复核 2026-10-21 |
| MuJoCo gripper 是**单动爪** hinge（-0.17~1.75 rad），固定爪不动，无 equality 耦合 | `so101_new_calib.xml:114` |
| `scene.xml` 当前**无任何可抓取物体**；gz 模式用内置 `empty.sdf`（无自定义 world） | 已勘察 |
| so101 工作空间小（draw cube 实测仅 6 cm cube 可达） | `reports/draw_cube_report.md` |

## 模式与参考

| 类别 | 来源 | 模式 |
|---|---|---|
| MTC stage 组织 | `ur_mtc_pick_place_demo/src/mtc_node.cpp:1113-1450` | CurrentState→MoveTo→Connect→SerialContainer(pick/place) |
| CMake 链接 MTC | 同包 `CMakeLists.txt` | find_package(moveit_task_constructor_core) |
| 抓取位姿生成 | 同上 `:1335-1349` | GeneratePose + ComputeIK(setIKFrame=gripper_frame) + 多候选 fallback |
| 包结构 | `src/so101_moveit_config` | ament_cmake + MoveItConfigsBuilder |
| launch 复用 | `src/so101_bringup/launch/demo.launch.py` | include move_group.launch.py，OpaqueFunction 分支 |
| 可达性探测 | `scripts/draw_cube_viewer.py::solve_path` | 探测脚本先验证再写死配置 |
| TDD | `tests/` | pytest RED→GREEN；C++ 侧以编译级测试 |

## 实施阶段

### Phase 0：spike + 可达性探测（消解 HIGH 风险）
- 编写探测脚本 `scripts/probe_pick_poses.py`：用 libs/so101_robotics 的 IK
  探测「桌面高度 cube 抓取位姿（预抓取/抓取/提拉）+ place 位姿」在关节限位内
  全可达的 cube 摆位（复用 draw cube 探测方法论）。
- 物理夹持 spike：在 MuJoCo 中手动闭合 gripper 压住 cube，验证摩擦夹持能否
  提拉离面；不行则调整 cube 尺寸/摩擦/夹持目标开度，记录结论。
- 验证 MTC 头文件/库可链接（最小 CMakeLists 试编译一个 task）。
- 验证: `uv run python scripts/probe_pick_poses.py` 全 PASS 输出可达摆位。

### Phase 1：pick 场景（MuJoCo + Gazebo）+ 夹持配置
- **MuJoCo**: 新增 `src/so101_sim/mujoco/scene_pick.xml`：scene.xml + 可抓取
  cube（freejoint、合理摩擦/尺寸，位置取 Phase 0 探测值）+ place 标记。
- **Gazebo**: 新增 `src/so101_bringup/worlds/pick_place.sdf`（table + cube +
  place 标记），gz 模式 gz_args 指向自定义 world 替代 empty.sdf。
- 夹持物理参数调优（cube 质量/摩擦/gripper forcerange 内的伺服 kp），
  **两个后端分别 spike**（物理引擎不同，行为可能有差异）。
- 测试: `tests/test_scene_pick.py`（两个场景文件加载、cube 存在、对齐不破坏）。
- 验证: spike 在两个后端分别复现「夹起-平移-放下」物理成功（mujoco 脚本直测；
  gz 经 ros_gz_bridge 读 cube 位姿验证）。

### Phase 2：`so101_mtc_pick_place` C++ 包（MTC 任务节点）
- ament_cmake 包：package.xml / CMakeLists.txt（find_package
  moveit_task_constructor_core 等）。
- `src/mtc_pick_place_node.cpp`：精简版 MTC pick+place（cube 固定已知位姿，
  不做感知分割——超范围）：
  - pick 容器: CurrentState → MoveTo(open) → Connect →
    GenerateGraspPose+ComputeIK → approach(MoveRelative) → CloseGripper →
    attach → lift
  - place 容器: Connect → GeneratePlacePose+ComputeIK → place → open →
    retreat → detach
  - 参数化: cube 尺寸/位姿、place 位姿、arm/gripper group 名。
- planning scene: 节点启动时发布 cube CollisionObject（与 mujoco 场景位姿
  一致，常量参数）+ table 平面。
- 测试: 编译通过 + 最小 task 单测（gtest，构建 task 对象成功）。
- 验证: `colcon build --packages-select so101_mtc_pick_place`。

### Phase 3：launch + RViz MTC 可视化
- `launch/mtc_pick_place.launch.py`：**`sim:=mujoco|gazebo` 参数**（默认 mujoco）：
  - mujoco: ros2_control mujoco + `scene_pick.xml` + JTC + move_group + RSP + RViz
  - gazebo: ros_gz_sim 加载 `pick_place.sdf` world（含 cube/table）+
    gz_ros2_control + /clock bridge + move_group + RSP + RViz
  - 两侧共用 mtc_pick_place_node 与 move_group（执行后端无关，控制器同名）。
  实现方式: OpaqueFunction 按 sim 分支直接组装（镜像 demo.launch.py 模式）。
- RViz 配置 `rviz/mtc_pick_place.rviz`：MotionPlanning + MTC Introspection
  面板（solution 阶段逐一显示）。
- 验证: 两个 sim 参数下分别启动成功，RViz 中 MTC solution 可逐 stage 回放。

### Phase 4：mtc_pick_and_place 端到端验收
- 冒烟循环: RViz 显示规划 → execute_task_solution → 仿真中 JTC 执行 →
  cube 实际被搬到位。**mujoco 与 gazebo 各验收一轮**。
- 判定: 订阅/读取 cube 位姿对比 pick 与 place 区域（mujoco: 状态接口/脚本；
  gz: ros_gz_bridge 桥接 pose 或 gz topic）。
- 产出 `reports/mtc_pick_place_report.md`（两后端分别记录）+
  `.claude/tdd/mtc.tdd.md`。
- 验收标准: MTC 规划成功（RViz 可见 solutions）；**两个后端**中 cube 从 pick
  位被夹起、经空中平移、放入 place 位（位移 > 8 cm），全程无翻倒/掉落；
  规划-执行闭环无人干预一条命令完成。

## 文件变更清单

| 文件 | 动作 | 说明 |
|---|---|---|
| `plan/mtc.md` | CREATE | 本计划 |
| `scripts/probe_pick_poses.py` | CREATE | Phase 0 可达性探测 |
| `src/so101_sim/mujoco/scene_pick.xml` | CREATE | MuJoCo pick 场景（cube + place 标记） |
| `src/so101_bringup/worlds/pick_place.sdf` | CREATE | Gazebo pick world（table + cube + place 标记） |
| `tests/test_scene_pick.py` | CREATE | 双场景测试 |
| `src/so101_mtc_pick_place/**` | CREATE | C++ MTC 包（CMakeLists/package.xml/src/launch/rviz） |
| `reports/mtc_pick_place_report.md` | CREATE | 验收报告 |
| `CLAUDE.md` / `docs/` | UPDATE | 收尾时补命令与架构条目 |

## Risks

| 风险 | 等级 | 缓解 |
|---|---|---|
| 单动爪夹持不稳（单侧挤压，cube 可能滑脱/翻倒），**mujoco 与 gz 物理引擎行为可能不同** | **HIGH** | Phase 0/1 双后端物理 spike 前置消解：调 cube 尺寸/摩擦/闭合速度；兜底方案 = 抓取瞬间激活 equality weld 约束（mujoco）/ gazebo 固定 joint（仿真级"磁吸"，保留视觉真实性） |
| so101 工作空间小，pick+place 双点不可同时可达 | HIGH→已探测法消解 | Phase 0 探测脚本先验证再定摆位（draw cube 已验证此方法论有效） |
| MTC C++ 编译/链接问题（版本 API 差异 0.1.8 vs 参考项目） | MEDIUM | 参考 CMakeLists 起步；Phase 0 最小编译 spike |
| MTC solution 执行到 JTC 的时序问题（degenerate/zero-motion 轨迹被 JTC 拒绝） | MEDIUM | 参考项目已有 re-stamp/filter 处理逻辑可移植（mtc_node.cpp:217-228） |
| move_group 与仿真 /clock（use_sim_time）不一致导致 TF/规划超时 | MEDIUM | 两个 launch 均已含 /clock 桥接；mtc 节点统一 use_sim_time |
| 双后端范围翻倍（场景×2、验收×2） | MEDIUM | MTC 节点/规划层完全复用（后端无关），仅执行侧场景与 launch 分支差异 |
| 范围蔓延（感知/分割/ArUco） | LOW | 明确不做：cube 位姿为常量已知，不接感知 |

## Estimated Complexity: LARGE

- Phase 0: 2~3h（探测脚本 + 物理 spike + 编译 spike）
- Phase 1: 3~4h（双后端场景 + 夹持调参）
- Phase 2: 4~6h（C++ MTC 节点，精简自参考项目）
- Phase 3: 3~4h（双后端 launch + RViz MTC 面板）
- Phase 4: 3~5h（双后端端到端验收循环 + 报告）

## Acceptance

- [ ] `scripts/probe_pick_poses.py` 探测输出全可达摆位（Phase 0 产物）
- [ ] `colcon build` 6 包全过（新增 so101_mtc_pick_place）
- [ ] `ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=mujoco`
      与 `sim:=gazebo` 均一条命令完成：RViz 可见 MTC solutions（Introspection
      逐 stage），仿真中 cube 被 pick→place 实际搬运成功（双后端分别验收）
- [ ] `reports/mtc_pick_place_report.md`（双后端分别记录）+
      `.claude/tdd/mtc.tdd.md` 产出
- [ ] commit 前征得用户同意（工作流约定）
