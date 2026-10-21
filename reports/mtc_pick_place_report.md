# MTC Pick and Place 验收报告 — 夹持质量修复 (R 系列)

> 日期: 2026-10-21 | 计划: `plan/mtc_fix.md` | TDD 证据: `.claude/tdd/mtc_fix.tdd.md`

## 结论

| 验收标准 (plan/mtc_fix.md) | 结果 | 证据 |
|---|---|---|
| ① cube ≤ 3cm，三处一致 | ✅ 2cm | `test_scene_pick.py` + `test_mtc_package.py::test_cube_size_default_2cm`（scene_pick.xml / pick_place.sdf / 节点参数） |
| ② 夹起时 cube 居于两指之间，无穿模 | ✅ | 居中偏差 **0.1mm**（<5mm）；视觉网格间隙 22.5-20 = 2.5mm > 0（`test_attach_semantics.py`） |
| ③ 落点距 place 目标 < 3cm | ✅ **11.8mm**（旧 5~7cm） | `test_attach_semantics.py::test_place_accuracy_3cm` |
| ④ MuJoCo 端到端 SUCCESS | ✅ | `bash scripts/mtc_smoke.sh sim:=mujoco` → `result=SUCCESS`（2026-10-21，planning 1 solutions） |
| ⑤ Gazebo 端到端 SUCCESS | ✅ | 首跑 `error_code=-4`（时钟不同源）→ 修复 launch `use_sim_time` → 复跑 `result=SUCCESS`（2026-10-21，economy 会话遗留验收项补齐） |
| ⑥ `uv run pytest` 全绿 | ✅ **154 passed** | 2026-10-21 |

## 三问题的根因与修复

| 问题 | 根因 | 修复 |
|---|---|---|
| ① cube 太大 (4cm) | 单动爪扫掠 ~2.5cm，4cm 相对爪偏大 | 缩小到 2cm（用户指定），三处同步；闭合角 -0.02 rad 时指面间距 22.4mm ≥ 20mm |
| ② 贴一边提起 | 1) probe 把 EE 放 cube 正上方，而爪几何要求 standoff 27.9mm 偏移——cube 从未处于两指面之间 2) SRDF closed=1.5 实为物理**全开**（qpos 大=张开），爪从未物理闭合 | `grasp_pose` 改由 R1 实测 `T_rel` 构造（`T_ee = T_cube ∘ inv(T_rel)`）；SRDF open=1.0/closed=-0.02；attach 后刚体跟随 `ideal_rel_pose`（居中 0.1mm） |
| ③ cube 与爪穿模 | 1) attach 取"规划初始位姿"，与实际爪位姿差 standoff 2) 指面斜面使带中点等距 ≠ 全程无穿透 | `ideal_rel_pose` 刚体跟随（attach 即对齐理想位姿）；闭合带中点间隙 2.5mm |

## 执行中发现的深层缺陷（计划外，已修）

1. **probe `grasp_pose` EE 正上方**——与 standoff 矛盾（R2 修，`T_rel` 构造）。
2. **SRDF gripper 方向反**——"close gripper" 物理上张开爪，kinematic attach
   掩盖了从未真实夹持（R3 修，含插件阈值反转 attach ref<0.1 / release ref>0.6）。
3. **probe 无自碰撞校验**——曾给穿肩 3mm 解（R2 修，`has_self_collision`
   接入 `probe_pair`，节点默认关节解全部换成校验通过的解）。
4. **gz 分支缺 `use_sim_time`**——move_group 用墙上时间校验带仿真时间戳的
   joint_states，轨迹校验必超时（error_code=-4）（R4 修，launch 三节点 +
   sim time 同源）。

## 已知限制

1. **指尖穿地板 ~23mm**（抓取位形）：SO-101 指刃比夹持带中点低 45mm，
   贴地 cube 的 top-grasp 必然过深。经济版 4cm 已存在，非本次引入；
   kinematic attach 不受影响。根除方案：垫块抬高 cube（计划中的备选）。
2. **MuJoCo mesh 碰撞凸包**：爪 mesh 含掌部连接臂，凸包向指间隙鼓出，
   凸包接触穿透 -18mm 为超近似；渲染网格以面间距度量（2.5mm 间隙）。
3. gz 侧 DetachableJoint 为物理关节夹持，无需 ideal_rel_pose 参数。

## 工件

- 校准脚本: `scripts/probe_grasp_geometry.py`（`uv run python scripts/probe_grasp_geometry.py`）
- 摆位/自碰撞探测: `scripts/probe_pick_poses.py`（`--cube-size 0.02` 默认）
- attach 语义 spike: `scripts/attach_semantics.py`（镜像插件语义 + 验收指标打印）
- 关键常量: `close_angle=-0.02`、`open=1.0`、
  `ideal_rel_pose=[-0.019618,-0.048269,0.0188, 0.707071,-0.707071,-0.007071,0.007071]`
