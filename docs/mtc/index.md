# MTC：MoveIt Task Constructor pick and place

**定位**：so101 用 MoveIt Task Constructor 完成 pick-and-place。当前**纯 RViz 模式**
（mock 硬件，无物理仿真）：cube/table 全部是 planning scene 虚拟场景，MTC 规划
solution 在 RViz 里逐 stage 回放。mujoco/gz 双后端**暂停**（产物保留，见
`workflow/todo/mtc_sim_backends_todo.md`）。

## 架构

```
so101_mtc_pick_place (ament_cmake)
├── mtc_pick_place_node   扁平 stage 链（预求解关节解 + ModifyPlanningScene attach）
│   └── planning scene: cube(2cm) CollisionObject, 执行经 move_group ExecuteTaskSolution
├── grasp_attach_plugin   mujoco kinematic attach（仿真后端用, RViz 模式不加载）
├── launch/mtc_pick_place.launch.py   sim:=rviz|mujoco|gazebo（后两者暂停）
└── rviz/mtc_pick_place.rviz          MotionPlanning + Motion Planning Tasks 面板/Display
```

**为什么是扁平链而不是标准 stage 链**（Connect + SerialContainer + GeneratePose +
ComputeIK）：MTC 0.1.8 + 5-dof 臂上容器调度是深水区（place 侧 Connect 从未触发，
三次会话实测，见 `workflow/problems/`），扁平链绕开接口方向/调度问题且 Introspection
仍可发布 solution 动画。详细尝试记录见 todo。

## 命令

```bash
# 纯 RViz 模式（当前主线）：mock 硬件 + 规划 + 执行动画
ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=rviz
# 节点任务完成后保持存活（keep_running），RViz 面板可反复回放; Ctrl+C 结束

# 关闭自动执行（只规划, RViz 面板手动回放）
ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=rviz execute:=false

# 可达性探测（校准/摆位变更后重跑; pinocchio IK + 自碰撞校验）
uv run python scripts/mtc_probe_pick_poses.py
```

## 关键校准值（R1/R2 实测, 已写进节点默认参数）

| 量 | 值 |
|---|---|
| cube | 2cm, pick=(0.117,-0.077,0.01), place=(0.125,0.063) |
| 夹持几何 | ideal_rel_pose 居中偏差 0.1mm, standoff 27.9mm, close_angle=-0.02 |
| SRDF gripper 方向 | open=1.0 / closed=-0.02（0.1.8 实测方向反转过一次, 勿改回） |

## 已知限制

- **MoveIt KDL IK tip 只有 `gripper_frame_link`**——`moving_jaw` 不能作
  setIKFrame（`Cannot compute IK ... pose reference frame`）。目标位姿需换算：
  `T_gframe_goal = T_mjaw_goal ∘ inv(T_gframe→mjaw)`。
- 标准 MTC stage 链（树状 Introspection）暂停，见
  `workflow/todo/mtc_sim_backends_todo.md` 附加节（含 5 个已修 bug 与剩余卡点）。
-RViz 模式的"夹住"是 planning scene attach 语义（无物理）；cube 动画由
  Introspection 的 solution 轨迹驱动。
- C++ 改动后 `colcon build --packages-select so101_mtc_pick_place --cmake-clean-first`
  （symlink-install 增量判断对本包不可靠）。
- 执行期 planning scene 若混入额外 CollisionObject（如视觉标记盒），
  会因 "Trajectory component invalid" 中止执行（PREEMPT）——共享场景写入者
  只限 mtc 节点自身。

## 恢复仿真后端（暂停项）

见 `workflow/todo/mtc_sim_backends_todo.md`：scene_pick.xml / pick_place.sdf /
grasp_attach_plugin 均保留；遗留缺口 = 指尖穿地板 23mm、mesh 凸包穿透、
gz 端未验收。
