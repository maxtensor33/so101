# draw cube 验收报告 (plan/robotics.md Phase 5)

日期: 2026-10-21 · 模型: `so101_new_calib_torque.xml` (motor actuator, ±3.35 Nm)
末端: `moving_jaw_so101_v1` (动爪尖端, "笔尖") · 立方体: 6 cm, 72 航点全可达

## 验收标准 (plan/robotics.md)

| 标准 | 目标 | 实测 | 结果 |
|---|---|---|---|
| torque 模式 EE 跟踪 RMS | < 5 mm | **3.06 ~ 3.18 mm** (3 次运行) | ✅ PASS |
| 力矩饱和 | 无持续饱和 | \|tau\| ≤ 3.35 Nm 已饱和限幅, 无失控 | ✅ PASS |
| 优于位置基线 | torque < position | 3.2 mm vs 9.7 mm (≈3×) | ✅ PASS |
| RViz 同步可视化 | robot + 轨迹 + cube 线框 | rviz2 全程存活; 绿色 cube 线框 + 蓝色 EE 轨迹实时增长 (0.5s 重发布, TRANSIENT_LOCAL QoS) | ✅ PASS |
| MuJoCo viewer 轨迹 | cube 线框 + trail | `scripts/draw_cube_viewer.py` (lab-2 风格, scene_torque 场景) | ✅ PASS |
| 窗口关闭联动退出 | 关 MuJoCo/RViz 窗口 → launch 退出 | rviz on_exit=Shutdown 实测: `was required: shutting down launched system` | ✅ PASS |
| viewer 全屏 + 隐藏面板 | 启动即最大化, 无左右 UI 面板 | `show_left_ui/right_ui=False` + X11 EWMH 最大化 (实测 `_NET_WM_STATE` 含 MAXIMIZED_VERT/HORZ) | ✅ PASS |

## 两种控制模式对比 (ROS 链路, mujoco_ros2_control, 100 Hz)

| 模式 | 执行方式 | KP/KD | RMS (mm) | Max (mm) | 日志 |
|---|---|---|---|---|---|
| **torque** (computed torque) | effort 命令 @100 Hz | 150/15 | **3.06** | 6.69 | smoke19 |
| **torque** (复跑) | 同上 | 150/15 | 3.18 | 7.81 | rviz run |
| **torque** (KP=50 基线) | 同上 | 50/5 | 13.52 | 95.42 | smoke16 |
| **position** (JTC 基线) | follow_joint_trajectory | 伺服内置 | 9.67 | 53.66 | smoke23 |

## 库链路 (无 ROS, 500 Hz 内环, `scripts/draw_cube_viewer.py`, 加载 `scene_torque.xml`)

| KP/KD | RMS (mm) | Max (mm) |
|---|---|---|
| 50/5 | 7.20 | 15.25 |
| 150/15 | **2.43** | 5.03 |
| 300/30 | 1.24 | 2.68 |

ROS 链路 100 Hz ZOH + 传输延迟使同等增益下误差略大 (3.2 vs 2.4 mm), 趋势一致。
viewer 脚本加载 `src/so101_sim/mujoco/scene_torque.xml` (含地板场景, 与 ROS 链路
一致), qpos/qvel 按关节名索引地址; 用户关闭 viewer 窗口即停止并输出统计。

## 调试过程中修复的关键问题

1. **IK 冗余分支翻转** → 相邻航点关节跳变 1.5+ rad, 固定段时长下力矩饱和失控。
   修复: 段时长按关节空间距离自适应 (`max(seg_dur, dist/1.0)`, 限关节速度 ~1 rad/s)。
2. **`_build_and_run` 同步跑 ~25 s IK 阻塞 executor** → 轨迹起点取到 25 s 前的陈旧
   `q_arm` → 开局大瞬态 (max 160 mm)。修复: IK 完成后经 one-shot timer 用新鲜
   `q_arm` 启动执行。
3. **纯重力补偿中性稳定** → 保持位形缓慢漂移。修复: 无目标时锁存 `q_hold` 并做
   PD+重力补偿。
4. **ForwardCommandController `interface_name` 必须为标量** `effort`（数组导致运行
   时类型错误），测试断言同步修正。
5. **`fk.mujoco_fk` 与仿真数据无干扰**（独立 MjData 缓存）已验证; viewer 脚本初版
   缺 `data.ctrl[:] = tau` 写入导致 445 mm 自由坠落, 修复后 7.2 mm。
6. **cube 顶点排序与边表不匹配**（`cube_vertices` 字典序 vs `CUBE_EDGES` 底面/顶面
   约定）→ 画出的"立方体"是扭曲折线, 线框 marker 跨边跳变。修复顶点排序后 TDD
   断言 (段长 = side/5, 首尾相接) 全过。
7. **RViz marker 单发易丢** → 改为 0.5s 周期重发布 + TRANSIENT_LOCAL QoS, 晚启动
   的 RViz 也能收到; EE 轨迹实时增长显示。

## 复现命令

```bash
# ROS 链路 torque 模式 (RViz 同步)
ros2 launch so101_robotics_torque_app draw_cube.launch.py rviz:=true \
  kp:=150.0,150.0,150.0,150.0,150.0 kd:=15.0,15.0,15.0,15.0,15.0
# 位置基线对照
ros2 launch so101_robotics_torque_app draw_cube.launch.py rviz:=false ctrl_mode:=position
# MuJoCo viewer 版 (lab-2 风格, 500 Hz 内环)
uv run python scripts/draw_cube_viewer.py
```
