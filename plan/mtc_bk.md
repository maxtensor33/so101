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


# Plan: MTC Pick and Place 夹持质量修复（R 系列）

> 承接: `plan/mtc.md`（已打通 mujoco 端到端）、`problems/mtc_deepseek-v4-1-flash-economy-261017.md`（遗留问题清单）
> 用户实测三问题: ① cube 太大 ② cube 贴着一边夹爪提起 ③ cube 与夹爪穿模

## 需求重述

当前 MuJoCo 端到端已打通（位移 0.19m > 0.08m 阈值），但夹持质量不可接受：

1. **cube 太大**（4cm）：单动爪扫掠半径 ~2.5cm，4cm cube 相对爪偏大，需要缩小, 先测试2cm。
2. **cube 贴一边夹爪提起**：kinematic attach 的 relpose 是"cube 贴着动爪侧"——
   根因是 EE 原点在 cube 侧方 3cm（横向偏移），cube 从未处于固定指面与动爪指面
   之间的对称位置，attach 后看起来像"粘在爪边上"。
3. **cube 与夹爪穿模**：attach 后 cube 与爪 mesh 重叠（economy 记录释放瞬间穿透
   ~2cm，接触力把 cube 弹开 5~7cm，落点精度差）。

三问题同源：**缺一次夹持几何校准**——cube 应置于固定指面与动爪指面之间的中点
（留 1~2mm 间隙），EE 横向偏移与闭合角据此重算。

## 环境事实（已勘察确认）

| 事实 | 来源 |
|---|---|
| 动爪指在 EE frame −y 方向 2.5cm 处（geom pos=[-0.001,-0.025,0.019]），绕 frame z 扫掠 | `so101_new_calib.xml` moving_jaw geom + Phase 0 诊断 |
| 固定指 geom 在掌部 frame（z=-0.023/-0.028 两块），**从未实测其世界位姿** | 同上（gripper body geom） |
| 当前 attach relpose = 规划初始 cube 位姿（被单动爪推走前的），贴动爪侧 | economy 会话 §5 |
| 当前 cube 4cm / mass 0.05 / friction 1.5 | `scene_pick.xml` |
| probe 的 IK 只查关节限位、**不做自碰撞检查**（曾给出穿肩 3mm 的解） | economy 会话 根因 A |
| release 时穿透 2cm 弹开 → 落点偏 5~7cm | economy 会话 §3.2 |
| 场景三处需同步：scene_pick.xml / pick_place.sdf / 节点 cube_size 参数 | 已勘察 |

## 实施阶段

### Phase R1：夹持几何校准（TDD spike，纯 Python）
- 诊断脚本：抓取位形下输出固定指面与动爪指面的**世界系位姿**（geom 数据 +
  forward kinematics），求两指面的中轴线与法向。
- 计算"理想夹持位姿"：cube 中心 = 两指面中点沿法向偏移（留 1~2mm 间隙），
  EE 横向偏移（standoff）与闭合角据此反解。
- 输出：`standoff`、`close_angle`、`ideal_rel_pose`（cube 相对 EE/动爪的目标位姿）
  ——写成可直接粘贴进参数/常量的形式。
- 测试: `tests/test_grasp_geometry.py`——理想位姿与两指面等距、cube box 与两指
  mesh AABB 无穿透、随 yaw 旋转不变性。
- 验证: `uv run pytest tests/test_grasp_geometry.py` 全绿。

### Phase R2：cube 缩小（0.04 → 0.02，用户指定 2cm）+ 摆位重算
- 三处同步: `scene_pick.xml`（size=0.01 / pos.z=0.01）、`pick_place.sdf`、mtc 节点
  `cube_size` 参数默认值 0.02（用户指定 2cm）。
- `probe_pick_poses.py` **加 mujoco 自碰撞校验**（IK 解逐个过 `mj_forward` +
  碰撞查询，自碰撞即弃——economy 教训：原 grasp 解穿肩 3mm）。
- 重跑探测拿新摆位（cube 缩小后 EE 高度 0.02→0.01，位形会变）与无自碰撞关节解，
  更新节点默认 `grasp_joints` / `lift_joints` / `place_joints`。
- 测试: `tests/test_scene_pick.py` 断言更新（cube size/pos），probe 测试补自碰撞
  断言。
- 验证: `uv run pytest` 全绿 + 探测脚本输出可行组合。

### Phase R3：attach relpose 改为理想夹持位姿
- `grasp_attach_plugin`：`ideal_rel_pose` 参数（R1 输出）——attach 时不再取
  "推挤后的实际位姿"，直接用理想位姿（居中、无穿模）。
- 释放微调：release 时沿远离爪方向平移 gap（消除 2cm 穿透弹开），或延迟到爪
  完全张开 + retreat 后（二选一，R4 实测定）。
- gz 侧: `pick_place.sdf` DetachableJoint 挂接点对齐同一几何（child_link 与
  attach 位姿）。
- TDD: 复用 Phase 1 的 attach 语义 spike（携带跟随误差、释放落物断言），
  新增"夹持居中"断言。
- 验证: spike 输出 cube 中心与两指面中点偏差 < 5mm、无穿透。

### Phase R4：双后端验收 + 报告
- **MuJoCo**: 冒烟 + cube 位姿数据确认三问题消除（不贴边/不穿模/落点距 place
  目标 < 3cm），位移仍 > 8cm。
- **Gazebo**: 补上遗留验收（economy 会话未验证）——sim:=gazebo 全流程，
  DetachableJoint attach/detach 生效。
- 更新 `reports/mtc_pick_place_report.md`（若不存在则新建）与
  `.claude/tdd/mtc.tdd.md`。
- 验收标准: ① cube 尺寸 ≤ 3cm ② 夹起后 cube 位于两指之间（目测/数据）③ 全程
  无穿模帧 ④ 落点距 place 目标 < 3cm ⑤ 双后端 SUCCESS。

## 文件变更清单

| 文件 | 动作 | 说明 |
|---|---|---|
| `plan/mtc_fix.md` | CREATE | 本计划 |
| `tests/test_grasp_geometry.py` | CREATE | R1 夹持几何测试 |
| `scripts/probe_grasp_geometry.py` | CREATE | R1 诊断/校准脚本 |
| `scripts/probe_pick_poses.py` | UPDATE | 加自碰撞校验 |
| `src/so101_sim/mujoco/scene_pick.xml` | UPDATE | cube 3cm |
| `src/so101_bringup/worlds/pick_place.sdf` | UPDATE | cube 3cm |
| `src/so101_mtc_pick_place/src/mtc_pick_place_node.cpp` | UPDATE | cube_size 默认值 + 新关节解 |
| `src/so101_mtc_pick_place/src/grasp_attach_plugin.cpp` | UPDATE | ideal_rel_pose |
| `tests/test_scene_pick.py` | UPDATE | 断言同步 |
| `reports/mtc_pick_place_report.md` | CREATE/UPDATE | R4 验收 |
| `.claude/tdd/mtc.tdd.md` | CREATE/UPDATE | TDD 证据 |

## Risks

| 风险 | 等级 | 缓解 |
|---|---|---|
| 3cm cube 在 5-dof 可行族内不可达（EE 高度降到 0.015 更低） | MEDIUM | Phase R2 探测先验证；不行则试 2.5cm 或抬高 cube（垫块）——2cm 更小, 达标概率更高 |
| 固定指面几何复杂（mesh），理想夹持点计算误差 | MEDIUM | 用 geom AABB 近似 + mujoco 穿透查询实证迭代 |
| 释放瞬间弹开问题与接触求解器相关，微调不一定根除 | MEDIUM | release 时序调整（先张爪后 retreat）兜底 |
| gz DetachableJoint 的 child_link 名需运行时确认 | LOW | gz 侧 spawn 后实际 link 名核对（economy 已记录方法） |

## Estimated Complexity: MEDIUM

- Phase R1: 1~2h（几何校准 + 测试）
- Phase R2: 1~2h（三处同步 + 探测重跑）
- Phase R3: 1~2h（plugin 参数化 + spike 验证）
- Phase R4: 1~2h（双后端验收 + 报告）

## Acceptance

- [ ] cube = 2cm，三处（mujoco/gz/planning scene）一致
- [ ] 夹起时 cube 居于两指之间（与两指面距离差 < 5mm），无穿模
- [ ] 释放后落点距 place 目标 < 3cm（当前 5~7cm）
- [ ] MuJoCo 与 Gazebo 双后端端到端 SUCCESS
- [ ] `uv run pytest` 全绿；报告与 TDD 证据更新
- [ ] commit 前征得用户同意
