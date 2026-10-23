# Plan: MTC 纯 RViz 专注开发（mujoco/gz 全部暂停）

> 承接: `plan/mtc_bk.md`（旧双后端计划归档）、`.claude/tdd/mtc_fix.tdd.md`
> （mujoco 端成果与遗留缺口）
> 用户决策: **mujoco 与 gazebo 都不用**——整个 pick-and-place 在 RViz 的 MTC
> 下完成（planning scene 虚拟场景 + MTC 规划 + RViz 内回放/执行）；
> 文档/命名以 main 分支为最终标准。

## 需求重述

1. **目标**：so101 基于 MoveIt Task Constructor 的 pick and place，**全程 RViz**：
   cube/table/place 区域全部是 planning scene CollisionObject（无任何物理仿真），
   MTC 规划 solution 在 RViz MotionPlanning + MTC Introspection 面板逐 stage 回放。
2. **mujoco + gazebo 暂停**：`scene_pick.xml`、`pick_place.sdf`、
   `grasp_attach_plugin` 等仿真产物不删除，记入 main 风格
   `todo/mtc_sim_backends_todo.md` 待办（含遗留缺口与恢复条件）。
3. **架构回归正统 MTC**：现有 `mtc_pick_place_node.cpp` 是"预求解关节解扁平链"
   （为绕过 mujoco 夹持物理的 hack，见 `.claude/tdd/mtc_fix.tdd.md` §陈旧测试同步）。
   纯 RViz 模式**没有夹持物理问题**，直接用标准 MTC stage 链
   （参考 `~/Third/Projects/github/UR3_ROS2_PICK_AND_PLACE/ur_mtc_pick_place_demo`
   的 `mtc_node.cpp`：CurrentState → open → Connect → pick 容器
   [GenerateGraspPose+ComputeIK+approach+CloseGripper+attach+lift] → place 容器），
   这样 MTC Introspection 才有完整的逐 stage 树可展示。
4. **文档/命名对齐 main**：mtc 分支基于 `30c4842`，缺 main 的 docs Sphinx 重构
   与 il_sim 合并；docs 旧三件套、单文件 `todo/todo.md` 与 main 结构冲突，以 main 为准；
   脚本命名 `<域>_<功能>.py`（`probe_pick_poses.py` 等 3 个加 `mtc_` 前缀）。

## 环境事实（已勘察确认）

| 事实 | 来源 |
|---|---|
| mtc 分支 = `301e006`（基于 30c4842），落后 main 2 commit（fcefe40 docs 重构 + ef0769b il_sim） | `git log main...mtc` |
| MTC 依赖已装: ros-jazzy-moveit-task-constructor-{core,msgs,visualization,capabilities} 0.1.8 | `plan/mtc_bk.md` 勘察 |
| SRDF: `arm` group（chain→gripper_frame_link）、`gripper` group（单 joint）、`end_effector`；gripper 方向已修正 open=1.0/closed=-0.02 | `.claude/tdd/mtc_fix.tdd.md` |
| 现节点为预求解扁平链（非 stage 链）；标准 stage 链参考项目 1662 行可精简移植 | 同上 + mtc_bk.md |
| 可达摆位已探测过（12/28 可行组合，含自碰撞过滤），夹持几何已校准（standoff 27.9mm、close_angle=-0.02）——**这些纯运动学结论对 RViz 模式直接可用** | `.claude/tdd/mtc_fix.tdd.md` R1/R2 |
| `demo.launch.py sim:=none moveit:=true`（mock 硬件 + move_group + RViz）main 已可用 | main CLAUDE.md:21 |
| planning scene 已有 cube CollisionObject 发布逻辑（常量位姿）可复用 | `mtc_pick_place_node.cpp` |
| 工作树已删（未提交）：`plan/mtc.md`、`plan/mtc_fix.md`、`todo/todo.md`（内容并入 `plan/mtc_bk.md`） | `git status` |
| main 的 todo 惯例：`todo/<域>_<主题>_todo.md`；main 无 `problems/` 目录 | main 树 |

## 实施阶段

### Phase 0：对齐 main（分支整备）
- `git merge main` 进 mtc 分支（mtc 侧未改 main 侧文件，预期无冲突；
  docs 旧三件套按 main 侧删除为准）。
- 提交既有删除：`plan/mtc.md`、`plan/mtc_fix.md`、`todo/todo.md`。
- `problems/` 三份会话记录原样保留不动（main 无此目录但不冲突；若用户后续
  要求归档再移 `learn/`）。
- 命名对齐（`git mv` + 引用同步）：`probe_pick_poses.py` → `mtc_probe_pick_poses.py`、
  `probe_grasp_geometry.py` → `mtc_probe_grasp_geometry.py`、
  `attach_semantics.py` → `mtc_attach_semantics.py`（attach_semantics 仅 mujoco
  spike 用，也可随仿真一起归 todo 不 rename——实施时定，倾向保留+rename）。
- 双后端暂停记 todo：`todo/mtc_sim_backends_todo.md` = scene_pick.xml /
  pick_place.sdf / grasp_attach_plugin / attach_semantics 现状 + 遗留缺口
  （指尖穿地板 23mm、凸包穿透 -18mm、gz 未验收、"RViz 与仿真差异"原始问题）+
  恢复条件。
- 验证: `uv run pytest` 全绿（merge 后）+ `colcon build`（mtc 包编译）。

### Phase 1：标准 MTC stage 链节点（RViz 模式核心）
- 重写/新增 `src/so101_mtc_pick_place/src/mtc_rviz_node.cpp`（或改造现有节点
  加 `--mode rviz`——实施时按侵入度定，倾向**新节点文件**保留旧扁平链对照）：
  - stage 链（精简自参考项目）：CurrentState → MoveTo("open") → Connect →
    SerialContainer(pick): GenerateGraspPose（cube 位姿参数化）+ ComputeIK
    （setIKFrame=gripper_frame，含 R1 校准的 standoff/grasp frame 偏移）→
    MoveRelative(approach) → MoveTo("close") → ModifyPlanningScene(attachObj) →
    MoveRelative(lift)；SerialContainer(place): Connect → GeneratePlacePose +
    ComputeIK → MoveRelative(place) → MoveTo("open") → ModifyPlanningScene(detach) →
    MoveRelative(retreat)
  - planning scene 发布：table + cube（摆位用 R2 已探测的可行值）+ place 目标区
  - 参数: cube size 0.02 / 位姿 / place 位姿（沿用 R1/R2 校准值）
- TDD: `tests/test_mtc_package.py` 扩展（rviz 节点存在性、stage 链关键断言、
  CMake 链接 MTC）；C++ 侧以编译级测试。
- 验证: `colcon build --packages-select so101_mtc_pick_place` + 节点
  `--ros-args` 干跑参数解析。

### Phase 2：RViz 端到端 launch + 验收
- launch: `launch/mtc_rviz.launch.py`（或扩展现 launch 加 `sim:=rviz` 缺省分支）：
  robot_state_publisher + mock 控制器（复用 moveit_controllers 或
  demo.launch.py sim:=none 的 controllers）+ move_group + mtc_rviz_node + RViz
  （MotionPlanning + MTC Introspection 面板）。
- MTC Introspection 面板订阅 solution 话题逐 stage 树状展示/回放。
- 执行: `execute_task_solution` → mock 控制器走轨迹（RViz 内动画呈现 pick→place 全程）。
- TDD/headless 验证: launch 起后 `rostopic` 探针断言（solution 话题有消息、
  `/monitored_planning_scene` 含 cube/table collision objects、无 ERROR 级日志）；
  GUI 画面由用户实测。
- 验收: 一条命令完成——RViz 中 MTC solution 全 stage SUCCESS 树 + 轨迹动画
  完整演示 cube 被抓起（attach 后随末端移动）→ 放入 place 区。

### Phase 3：收尾
- `reports/mtc_rviz_report.md`（RViz 模式验收 + 与 mujoco 端成果关系说明）；
  `.claude/tdd/mtc_rviz.tdd.md`；CLAUDE.md 增补命令条目；docs/ Sphinx 增 mtc 页
  （对齐 main 结构）。
- `uv run pytest` 全量 + `colcon build` 全绿。
- commit 前征得用户同意。

## 文件变更清单

| 文件 | 动作 | 说明 |
|---|---|---|
| `plan/mtc_rviz.md` | CREATE | 本计划 |
| `todo/mtc_sim_backends_todo.md` | CREATE | mujoco/gz 暂停待办（产物清单+遗留缺口+恢复条件） |
| `plan/mtc.md`、`plan/mtc_fix.md`、`todo/todo.md` | DELETE（提交既有删除） | 内容已在 plan/mtc_bk.md |
| `scripts/mtc_probe_pick_poses.py` 等 | RENAME | 命名对齐 main（引用同步） |
| `src/so101_mtc_pick_place/src/mtc_rviz_node.cpp` | CREATE | 标准 MTC stage 链（RViz 模式） |
| `src/so101_mtc_pick_place/launch/*.launch.py` | UPDATE | RViz-only 入口 |
| `src/so101_mtc_pick_place/rviz/*.rviz` | UPDATE | MotionPlanning + Introspection 布局 |
| `tests/test_mtc_package.py` | UPDATE | rviz 节点/stage 链断言 |
| `reports/mtc_rviz_report.md`、`.claude/tdd/mtc_rviz.tdd.md` | CREATE | 收尾证据 |
| `CLAUDE.md`、`docs/**` | UPDATE | 对齐 main 结构增补 |

## 不删除（暂停保留，见 todo）

`src/so101_sim/mujoco/scene_pick.xml`、`src/so101_bringup/worlds/pick_place.sdf`、
`src/so101_mtc_pick_place/src/{mtc_pick_place_node,grasp_attach_plugin}.cpp`、
`scripts/mtc_smoke.sh`、`scripts/attach_semantics.py`、`reports/mtc_pick_place_report.md`。

## Risks

| 风险 | 等级 | 缓解 |
|---|---|---|
| MTC 0.1.8 API 与参考项目版本差异（stage 参数名/attach 语义） | MEDIUM | 参考 CMakeLists + Phase 1 编译 spike 前置；参考项目已验证同 Jazzy 环境 |
| 现节点改造成本（扁平链 vs stage 链并存增加维护面） | MEDIUM | 新节点文件并行，旧节点随仿真一起归 todo 不删 |
| CloseGripper/attach 在无物理仿真下为纯规划语义（Introspection 正常但"夹住"只是场景标记） | LOW | RViz 模式预期行为，文档说明 |
| merge main 后 uv 环境变化（lerobot/torch） | LOW | merge 后 `uv sync` + 全量 pytest |
| so101 工作空间小，GenerateGraspPose 的 IK 解不足 | MEDIUM | R1/R2 已探测可行摆位作为常量兜底；GeneratePose 失败时回退给定关节解的 MoveTo |

## Estimated Complexity: MEDIUM

- Phase 0: 1h（merge + 命名 + todo 化）
- Phase 1: 3~5h（标准 stage 链节点 + 编译/参数 spike）
- Phase 2: 2~3h（launch + Introspection + 端到端验收循环）
- Phase 3: 1h（收尾）

## Acceptance

- [ ] mtc 分支包含 main 全部内容（docs Sphinx/todo 目录/learn），旧三件套删除
- [ ] 脚本命名对齐 main（`mtc_` 前缀 rename，引用同步）
- [ ] `todo/mtc_sim_backends_todo.md` 记录 mujoco/gz 暂停与遗留缺口
- [ ] 一条命令 RViz-only：MTC solution 全 stage SUCCESS（Introspection 树可见），
      轨迹动画完整演示 pick→place（cube attach 后随末端移动、place 后释放）
- [ ] `uv run pytest` 全量 + `colcon build` 全绿
- [ ] `reports/mtc_rviz_report.md` + `.claude/tdd/mtc_rviz.tdd.md` 产出
- [ ] commit 前征得用户同意
