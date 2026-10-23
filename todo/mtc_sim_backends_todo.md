# TODO: MTC mujoco/gz 双后端仿真（暂停, plan/mtc_rviz.md）

决策：RViz 表现与仿真结果差别大 → mujoco/gz 双后端**暂停**，专注纯 RViz 的
标准 MTC stage 链（`mtc_rviz_node`）。已有产物全部保留，恢复时从此清单接续。

## 保留产物（未删除）

- `src/so101_sim/mujoco/scene_pick.xml`（cube 2cm 摆位=已探测可行值）
- `src/so101_bringup/worlds/pick_place.sdf`（gz world，**gz 端从未端到端验收**）
- `src/so101_mtc_pick_place/src/mtc_pick_place_node.cpp`（预求解关节解扁平链，
  mujoco 端端到端已通：位移 0.19m）
- `src/so101_mtc_pick_place/src/grasp_attach_plugin.cpp`（kinematic attach，
  ideal_rel_pose 已校准：居中 0.1mm/落点 11.8mm；SRDF open=1.0/closed=-0.02）
- `scripts/mtc_smoke.sh`、`scripts/mtc_attach_semantics.py`（mujoco spike）
- `reports/mtc_pick_place_report.md`（mujoco 端验收记录）

## 恢复前必读的遗留缺口（.claude/tdd/mtc_fix.tdd.md §4）

1. 指尖穿地板 ~23mm（SO-101 指刃比带中点低 45mm，贴地 top-grasp 必然过深；
   兜底=垫块抬高 cube）
2. mujoco mesh 凸包碰撞超近似（接触穿透 -18mm；以视觉网格面间距 22.5mm 为准）
3. gz 端 DetachableJoint 端到端未验收
4. 原始问题：RViz 显示与 mujoco/gz 执行结果差别大（未做过系统对比——若恢复，
   先按 plan/mtc_rviz.md 已删稿 Phase 1 的基线对比方法做现象确认）

## 恢复条件/入口

- 触发：需要真实物理验证抓取（而非规划演示）时
- 入口：`ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=mujoco`
  （gz 侧接续 sim:=gazebo）；关联 RViz 模式 = `mtc_rviz_node`（标准 stage 链）

## 附加：标准 MTC stage 链（plan/mtc_rviz.md 一次尝试, 已暂停）

纯 RViz 模式曾尝试标准 stage 链（Connect+SerialContainer+GeneratePose+ComputeIK，
参考 UR demo），会话内修复 5 个问题后仍卡 planning：
1. SerialContainer 在 task_.add(std::move) 后解引用旧指针 → segfault（add 必须在 insert 之后）
2. GeneratePose 缺 setMonitoredStage(current state)
3. ComputeIK/MoveRelative 属性传播链断（'group' declared but undefined）→ 全显式 setter
4. ComputeIK setIKFrame 不能用 moving_jaw（KDL tip 只有 gripper_frame_link），
   目标位姿需换算到 gripper_frame_link 系（T_mjaw_goal ∘ inv(T_gframe→mjaw)）
5. 移除 table CollisionObject（大平面盒包住机器人 → 初始即碰撞 0 解）

仍卡：plan(5) 200s+ 不返回（与 problems/ 三会话记录的"place 侧 Connect 从未触发"
同类调度深水区）。**结论与 mtc_pick_place_node.cpp 头注释一致：此环境下扁平链是
正确架构**。重启条件：MTC 升级或换臂（6-dof）后重试；本期 RViz 模式复用扁平链。
