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
