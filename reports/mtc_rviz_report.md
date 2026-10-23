# MTC 纯 RViz 模式验收报告（plan/mtc_rviz.md）

日期：2026-10-23 · 分支：mtc（已 merge main）

## 结论

`ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=rviz` 一条命令：
**planning succeeded (1 solutions) → execution error_code=1 → exit (SUCCESS)**。
执行链 = 复用 mujoco 端已验证的扁平链节点（mtc_pick_place_node），mock 硬件执行；
RViz 面板（MotionPlanning + MTC Introspection，`rviz/mtc_pick_place.rviz`）由用户
图形环境实测（headless 无法验证画面）。

## 本模式改动

- launch `sim:=rviz`：xacro `sim:=none`（mock_components 硬件）+
  `config/controllers_rviz.yaml`（无 mujoco_plugins）+ move_group（含
  ExecuteTaskSolution capability）+ mtc_pick_place_node + RViz
- planning scene：cube（2cm，已探测摆位）+ place_area（纯视觉标记；曾因与
  gripper_link 碰撞导致执行 PREEMPT 而移除）
- mujoco/gz 产物全部保留，暂停事项见 `todo/mtc_sim_backends_todo.md`

## 过程中的修复（RViz 模式特有）

| 问题 | 根因 | 修复 |
|---|---|---|
| 执行 PREEMPT → error_code=-3 | place_area 标记盒与 gripper_link 碰撞（"Trajectory component invalid"） | 移除 place_area |
| RViz 空布局（用户报告） | setup.py data_files 漏 rviz 目录 | 补 data_files 行 |
| planning 卡死（标准链尝试期间） | table 盒包住机器人 → 初始碰撞 0 解 | 移除 table；连同标准链一起暂停 |

## 与 mujoco 端的关系

同一节点、同一 stage 链、同一参数（cube 2cm/摆位/grasp 校准值），仅执行后端不同：
mujoco=物理仿真+kinematic attach；rviz=mock 硬件+planning attach（场景语义）。

## 已知限制

- RViz 画面与 Introspection 面板实际显示待用户实测（headless 无法覆盖）
- 标准 MTC stage 链（树状 Introspection）为深水区已暂停，见 todo
