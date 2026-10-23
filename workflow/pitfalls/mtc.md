# mtc（纯 RViz 模式）

| 症状 | 根因与处置 |
|---|---|
| RViz Motion Planning Tasks 面板空树 | ① 面板类名应为 `moveit_task_constructor/Motion Planning Tasks`（旧名 `..._visualization/TaskConstructorPanel` 已废弃）；② .rviz 必须同时有 **TaskDisplay**（数据订阅者）与 Panel（UI 壳）；③ mtc 节点须 `keep_running`（introspection 的 transient_local 数据随进程销毁）。判据：`ros2 topic info /description` 看订阅数 |
| ComputeIK 报 `Cannot compute IK ... pose reference frame 'moving_jaw...'` | KDL IK tip 只有 `gripper_frame_link`；moving_jaw 目标位姿须换算（`T_gf = T_mjaw ∘ inv(T_gframe→mjaw)`） |
| MTC 容器/Connect 长期 0 解或不返回 | MTC 0.1.8 + 5-dof 的调度深水区（三次会话实证，见 workflow/problems/）——**用扁平链**，勿重启标准 stage 链 |
| 执行 PREEMPT（error_code=-3），日志 "Trajectory component invalid" | planning scene 混入额外 CollisionObject（视觉标记盒等）与机器人碰撞——共享场景写入者只限 mtc 节点 |
| 改 .cpp 后行为没变化 | symlink-install 增量编译对本包不可靠——`colcon build --packages-select so101_mtc_pick_place --cmake-clean-first` |
| stage init 报 `declared, but undefined` | MTC 属性传播链断裂——改全显式 setter（setGroup/setEndEffector/setIKFrame），别依赖 exposeTo 链 |
| SerialContainer 段错误 | `task_.add(std::move(container))` 后旧指针已空——insert 完全部子 stage 再 add |
| plan 长时间不返回 | planning scene 大平面盒（table/地面）包住机器人 → 初始即碰撞 0 解；去掉大盒 |

