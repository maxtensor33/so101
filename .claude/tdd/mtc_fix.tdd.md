# TDD 证据报告: MTC Pick and Place 夹持质量修复 (R 系列)

> 计划: `plan/mtc_fix.md`（2026-10-21 执行）
> 上游: `plan/mtc.md`（端到端打通）、`problems/mtc_deepseek-v4-1-flash-economy-261017.md`（遗留问题）
> 用户实测三问题: ① cube 太大 ② 贴一边夹爪提起 ③ cube 与夹爪穿模

## 1. 源计划与旅程

journeys（来自 plan/mtc_fix.md 验收标准）:

- 作为操作者，我要 cube 缩小到 2cm 且三处（mujoco/gz/planning scene）一致，
  以便单动爪能夹住它。
- 作为操作者，我要夹起时 cube 居于两指之间（与两指面等距、无穿模），
  以便视觉上像真实夹持。
- 作为操作者，我要释放后落点距 place 目标 < 3cm，以便任务可复现。
- 作为操作者，我要 MuJoCo 与 Gazebo 双后端端到端 SUCCESS。

## 2. 任务报告（计划任务 → 测试 → 证据）

### R1: 夹持几何校准

- 执行摘要: 修复 `probe_grasp_geometry.py` 两处测量 bug（servo mesh 误判为
  固定指；jaw/follower 顶点 frame 混用），实测闭合轴 = gripper frame x、
  指刃判定 |y|<13mm、带中点切片测量；输出高精度 `ideal_rel_pose`。
- 验证命令: `uv run pytest tests/test_grasp_geometry.py`
- RED 证据: 新增 `TestMeasuredGeometry` 4 用例，3 失败——
  `span=-129.8mm（负值）/ fixed<moving / close_angle 退化为扫描上限 1.70`
- GREEN 证据: `11 passed`（7 原有用例 + 4 实测几何不变量）
- 校准输出（cube 2cm）: `close_angle=-0.02`、span 22.4mm、standoff 27.9mm、
  `ideal_rel_pose=[-0.019618,-0.048269,0.0188, 0.707071,-0.707071,-0.007071,0.007071]`

### R2: cube 缩小 2cm + 摆位重算 + probe 自碰撞

- 执行摘要: 三处同步（`scene_pick.xml` size 0.01/z 0.01、`pick_place.sdf`
  box 0.02/pose z 0.01、节点 `cube_size` 0.02/`cube_z` 0.01）；
  `probe_pick_poses.py` 的 `grasp_pose` 改由 R1 `T_rel` 构造
  （`T_ee = T_cube ∘ inv(T_rel)`，EE 带 standoff 偏移，修复"EE 正上方 cube"
  的贴边根因）；新增 `has_self_collision`（mujoco 全接触检查）并接入
  `probe_pair`；重跑探测 12/28 可行，原摆位仍可行，节点默认关节解全部更新。
- 验证命令: `uv run pytest tests/test_probe_pick_poses.py tests/test_scene_pick.py tests/test_mtc_package.py`
- RED 证据: 10 失败（cube 仍 4cm×2、probe 无 T_rel 构造、无自碰撞过滤×3、
  节点默认值）
- GREEN 证据: 全套件 `154 passed`；自碰撞过滤器对已知坏解
  `[0.7612,-0.1760,1.4566,0.2903,-0.7610]`（穿肩 3mm）判 True、
  对 economy 好解判 False

### R3: attach 理想夹持位姿

- 执行摘要: `grasp_attach_plugin` 新增 `ideal_rel_pose` 参数 → attach 后
  刚体跟随（`wp = jaw_pos + R_jaw·rel_p`，`wq = q_jaw ⊗ rel_q`）；
  **发现并纠正方向约定错误**: SRDF `closed=1.5` 实为物理全开（R1 实测
  qpos 大 = 张开），爪从未物理闭合——SRDF 改 open=1.0/closed=-0.02，
  插件阈值反转（attach ref<0.1、release ref>0.6）；yaml 同步。
- 验证命令: `uv run pytest tests/test_attach_semantics.py` + `colcon build --packages-select so101_mtc_pick_place`
- RED 证据: 5 errors（spike 模块缺失）；spike 首跑穿透 -18mm →
  定位为 MuJoCo mesh 碰撞凸包（含掌部连接臂）向指间隙鼓出的超近似，
  渲染网格以面间距度量（gap 22.5mm ≥ cube 20mm）
- GREEN 证据: `5 passed`——attach/release 触发、居中偏差 0.1mm（<5mm）、
  视觉网格间隙 >0、释放落地 z≈9.7mm、落点距 place 11.8mm（<30mm）

### 陈旧测试同步（非计划内，恢复全绿所需）

- `tests/test_mtc_package.py::test_mtc_node_exists` 断言 `ComputeIK/Connect/
  GeneratePose`，但节点已按 plan/mtc.md 决策改为预求解关节解扁平链——
  同步断言至现行架构（含"不再使用"反向断言）。

## 3. 测试规格

| # | 保证 | 测试 | 类型 | 结果 |
|---|------|------|------|------|
| 1 | 实测 span 为正且 ≤80mm | `test_grasp_geometry.py::TestMeasuredGeometry` | 单元 | PASS |
| 2 | close_angle 是真实解非兜底 | 同上 | 单元 | PASS |
| 3 | grasp_pose 构造自洽（EE∘T_rel 回到 cube 中心） | `test_probe_pick_poses.py::TestIdealGraspFromRelPose` | 单元 | PASS |
| 4 | EE 有 standoff 水平偏移（~28mm） | 同上 | 单元 | PASS |
| 5 | 穿肩坏解被判自碰撞 / economy 好解通过 | `TestSelfCollisionFilter` | 单元 | PASS |
| 6 | probe_pair 丢弃自碰撞解 | 同上 | 单元 | PASS |
| 7 | cube 2cm 三处一致 | `test_scene_pick.py` + `test_mtc_package.py` | 集成 | PASS |
| 8 | attach 居中 <3mm（实测 0.1mm） | `test_attach_semantics.py` | 仿真集成 | PASS |
| 9 | 视觉网格无穿透（间隙 >0） | 同上 | 仿真集成 | PASS |
| 10 | 释放落地 z≈10mm | 同上 | 仿真集成 | PASS |
| 11 | 落点距 place <3cm（实测 11.8mm） | 同上 | 仿真集成 | PASS |
| 12 | SRDF 开合方向与物理一致 | `test_mtc_package.py::test_srdf_gripper_direction` | 集成 | PASS |

## 4. 覆盖与已知缺口

- `uv run pytest` 全套件: **154 passed**（本任务新增 ~20 用例）。
- 已知缺口（有意）:
  1. **指尖穿地板 ~23mm**: SO-101 指刃比带中点低 45mm，贴地 cube 的 top-grasp
     必然过深（经济版 4cm 已存在，非本计划引入；kinematic attach 不受影响）。
     如需根除，用计划中的"垫块"方案抬高 cube。
  2. **凸包接触穿透 -18mm**: mujoco mesh 碰撞用凸包，超近似；以视觉网格
     面间距（22.5mm ≥ 20mm）为准。
  3. gz 端到端见 `reports/mtc_pick_place_report.md`（R4）。
- 合并证据: 本报告即 RED/GREEN 汇总（commit 待用户批准）。

## 5. 深层根因（比计划多修的部分）

执行中发现的三个"计划外但同源"的缺陷，均已修复并有测试锁定:

1. **probe grasp_pose 把 EE 放 cube 正上方**——与爪几何（standoff 27.9mm）
   矛盾，cube 永远不在两指面之间（"贴边"根因之一）。
2. **SRDF gripper 方向反了**（closed=1.5 实为全开）——"close gripper"实际
   张开爪，物理夹持从未发生，全靠 kinematic attach 掩盖。
3. **插件阈值方向随之反了**（ref>1.2 attach / ref<0.3 release）——已随 SRDF
   一并反转。
