# Todo: envs 抓取物理调优（lift/place/stack 穿模与保持失败）

**状态**: 待办 · **创建**: 2026-10-29 · **来源**: il_sim_leader 遥操实测 + `scripts/il_sim_grasp_diagnose.py` 诊断

## 2026-10-30 突破: 根因定位 + squint 碰撞网格迁移（已验证）

**根因确认（运行时运动学实测）**: v1 指尖 pad box 的"手工换算顶点"位置全错——
固定指 pad 偏到手掌后方（rest 姿态世界系 z=0.236 vs 指尖 0.094–0.200），距
gripperframe site 0.10–0.14m。方块传送到夹持中心 + 渐进闭爪到底 = **零接触**
（is_grasping=False），指尖唯一碰撞体不在手指上 → 视觉 mesh（contype=0）直接
穿过方块 = 用户遥操"穿模"的真因。todo 下文"物理侧已可抓（传送实验 0.2mm）"的
旧结论是软接触深穿透假象：当年闭爪穿透 2-3cm 靠摩擦"夹住"，硬接触后伪影暴露。

**修复（照抄 squint/ManiSkill 成功配方）**: 对比 squint `envs/robot/so101.urdf`
（注释 "Decomposed collision for proper gripper contact"），迁移其三块碰撞网格
到 `libs/so101_envs/assets/`（已去 mtllib 行），替换两个错位 pad box：
- 固定指: `Fixed_part_1.obj` + `Fixed_part_2.obj`（凸分解），
  origin 与 URDF 同源 `pos="0 -0.000218214 0.000949706" quat="0 1 0 0"`
- 动爪: `moving_jaw_so101_v1_convex.obj`，origin `pos="0 0 0.0189"`
- 指尖碰撞网格同步硬接触 `solref="0.005 1" solimp="0.95 0.99 0.0001"`

**验证结果**:
- 闭爪卡在方块上（qpos 停 1.551 未达全闭合限位），平衡穿透 **0.2mm**
- 此前全量唯一失败的 `test_lift_grasp_teleported_item` 转绿，envs 测试全绿
- **遗留（原有问题，与本次无关）**: 专家 IK 对准 1/3（爪闭在方块旁零接触，见下文
  "剩余问题"）；遥操需用户实测复验

**测量陷阱（防复发）**: `m.mesh_vert` 是全局扁平数组，必须用 `mesh_vertadr[id]`/
`mesh_vertnum[id]` 切片再变换，直接 `[mesh_id]` 索引得到错数据（本轮踩坑，与
v1 pad box 手工换算出错同源）。几何结论一律以运行时 `d.geom_xpos/xmat` 变换 +
渲染 PNG 目视为准，不做手工坐标换算。

**测试方法陷阱**: 把方块传送到 site 本身就是几何错误——site 是爪尖全闭合收敛点，
固定指尖永远在 site 处，方块中心放那里必然与爪重叠（表现为与夹爪力参数无关的
初始穿透）；悬空方块还会被闭爪弧线切向推走（无桌面摩擦抵抗）。真实判定只能走
"桌上方块 + 上方合围"流程（专家/遥操）。

## 诊断数据（2026-10-29, SO101Lift-v1）

- 穿透深度：双爪闭合到底时爪-块接触穿透 **2.0-2.9cm**（方块边长 4.4cm）→ 目视"直接穿过"
- 穿透仍抬不起：item z 0.0276 → 0.0219（不升反降），success=False
- 专家 IK 对准成功率：3 episode 中 1/3（另 2 次爪闭在方块旁, 零接触）
- 静态碰撞本身正常：深度重叠实验立即产生接触（item↔gripper/-0.0119）、无排除对、contype/conaffinity 正确
- 历史：envs_report 已记录"lift/place/stack 抓取保持失败, reach 100%"——本诊断量化了它

## 2026-10-29 修复进展（本轮已完成）

| 项 | 修复 | 效果 |
|---|---|---|
| 视觉穿模 21mm | item geom 硬接触 `solref="0.005 1" solimp="0.95 0.99 0.0001"`（四个 scene）+ `<option cone="elliptic" impratio="10" iterations="200">` | 穿透 21mm → **0.2-0.3mm** ✅ |
| 生成半径超工作空间 | `SO101LiftEnv.GRASPABLE_RADIUS=0.26` 钳制（半径 >0.28 时 wrist_flex 触限 + 固定指拖桌） | 生成半径 ≤0.26 ✅ |
| 夹持力 | 爪面摩擦 2.0→3.5；夹爪 forcerange ±1.2→±2.0（kp 保持 400 防抖） | 接触力余量 ✅ |
| IK 种子 | grasp_item 种子池加 neutral（部分随机种子 DLS 局部极小, 误差 245mm 级） | 预解改善 ✅ |
| 到位机制 | 停机坪接近（防关节空间扫掠推飞方块 5cm）+ 自适应爬行（warm-start IK 局部滞点, 每轮只爬 1-2cm） | 部分改善 ⚠️ |

**关键结论: 物理侧已可抓** — MuJoCo 接触实验证实方块置于爪间时双爪稳定夹持
（硬接触下穿透 0.2mm）。**用户遥操的"穿模"反馈产生于硬接触修复之前, 需重测。**

## 物理侧验收完成 (2026-10-29)

`test_lift_grasp_teleported_item` 修正后通过: 方块钉在夹持中心 (gripperframe site) +
渐进闭合 → is_grasped 触发 (双爪 ≥0.5N)。**硬接触物理下抓取链路完整可用。**
夹持中心实测 (gripper 系): [-0.0068, 0.0024, -0.1006] ≈ gripperframe site。

## TCP 帧与指尖几何法证 (2026-10-29 深入排查, 未完全收敛)

**已确证的事实**:
1. MuJoCo mesh 碰撞用凸包, 且 `m.mesh_vert` 是**重心重定位后**的坐标 (含 `mesh_pos/quat`
   旋转平移) — 手工换算极易出错, 本轮多次测量互相矛盾皆源于此
2. 凸包幽灵面是"未接触先动/滑落"的根因 → 已用指尖 pad box 替换两指 mesh 碰撞
   (视觉 mesh 保留), 并加 `<contact><exclude>` 排除张爪自碰撞 (真实硬件不可能相碰)
3. `gripperframe` site 与 URDF `gripper_frame_link` 同源 (-0.0981); 用"两指网格最近点对"
   定位夹持点的方法不可靠 — 最近点对被铰接处 (两 mesh 机械相连, 间距≈0) 主导,
   据此"修正" site 反而放到了掌部 (渲染证实), **已回退原始 site**
4. 渲染观察: 原始 site 的指尖归属仍待确认 (标记球被网格遮挡, 需更好的可视化手段)

**当前状态**: `test_envs_tasks` 5/6 通过; 唯一失败 = 传送抓取测试 (pad 位置与 site
的匹配未收敛)。物理修复 (硬接触/pad/排除/半径/摩擦) 均已就位且无回归。

## 下一轮建议方案 (避免本轮的手工换算陷阱)

1. **以运行时接触为准**: 在 env 里逐帧扫描 `data.contact`, 让夹爪从不同位姿闭合,
   用实际接触点反推 pad 应在位置 (而非网格顶点换算)
2. 或直接对动爪/固定指用**多 box 分解** (MuJoCo 无凸分解, 可手放 2-3 个 box 覆盖指面),
   每个 box 位置用"传送方块到该处→闭合→看接触"闭环校准
3. site 归属: 渲染标记球改用大尺寸 (2cm) + 侧视角, 或临时把 site 挂到 worldbody 对比
4. 专家重整定: site/pad 定案后, SEED_POOL/停机坪/爬行参数需按新 TCP 重调

## 剩余问题（下一轮）

- 专家脚本水平对准残差: tool-down 可达带极薄（wrist_flex 限位 + 指底-桌间隙 4mm 三重约束）,
  1/3 episode 差 1-2cm 未到位 → 方向: 相机反馈伺服 / 考虑 item geom 加 mocap 辅助 / 提高 via 点密度
- 指底-桌面间隙仅 ~4mm, tool-down 抓取高度窗口窄

## 根因假设（按优先级, 已部分验证）

1. **夹爪位置伺服目标直接打到全闭合**：目标在方块内部 → 持续深穿透。改法：抓取时闭合目标只到"贴住方块宽度"（按方块尺寸算 qpos 目标），而非 -0.175 全闭合
2. **夹爪执行器太软**：kp=400 / forcerange ±1.2Nm——保持 25g 方块的摩擦力余量不足。改法：提高 forcerange 或 kp（需防抖）
3. **接触求解软度**：默认 solref 下深压穿模。改法：调 solref/solimp 或提高 solver iterations
4. 专家对准成功率低（1/3）：IK 预解种子池/容差问题（envs_run_expert grasp_item）

## 验收标准

- [ ] 闭合时目视穿透 < 5mm
- [ ] 脚本专家 lift 连续 5 episode 成功率 ≥ 80%
- [ ] leader 遥操 lift 可实际完成抓取-抬升演示

## 2026-10-31 更新：lift 已按 squint 迁移，保持/穿模收敛

`workflow/reports/lift_squint_migration.md`：物体改为 squint 尺寸（half 0.011–0.014,
密度 200 → ~3g）、指尖摩擦回到 2.0、item 接触 `condim=4`（扭转摩擦）。真实夹持中心
是**动态两指尖中点**（不是固定 `gripperframe` site）：25mm cube 对应夹爪目标约
0~0.2 rad，全闭合会把方块挤进指尖。

穿模根因与修复：
- jaw 的 218 块凸分解被 MuJoCo 合成一个大凸包 → 只保留夹持面 `convex_37/38`
  （`moving_jaw_pad.obj`）。
- 3g 物体接触刚度过软 → item / table / floor / 手指碰撞体统一
  `solref="0.001 1" solimp="0.99 0.999 0.0001"`；实测穿透 10–15mm → <1mm。
- 回归：`test_grasp_holds_after_release`（30/30 保持 + 穿透 <0.5mm）；用户策略 10
  episodes 手指穿透 ≤0.41mm，success 9/10（旧策略，需用新物理重训）。

专家 IK 对准问题（本文件上文）仍待重做。

相关: workflow/pitfalls/envs.md（"抓取行为不符合直觉"）、workflow/reports/envs_report.md
