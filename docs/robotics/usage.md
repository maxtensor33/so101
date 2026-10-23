# robotics 命令参考

## 构建 / 测试 / 验证

| 命令 | 说明 |
|---|---|
| `colcon build --symlink-install` | 构建全部 colcon 包（7 个） |
| `uv run pytest` | 全量测试（220 passed） |
| `uv run python scripts/setup_verify_model_alignment.py` | 位置版 URDF↔MJCF 对齐 → `reports/alignment_report.md` |
| `uv run python scripts/setup_verify_model_alignment.py --torque` | 力矩版对齐 → `reports/alignment_report_torque.md` |
| `bash scripts/clean_ros.sh` | 清理 demo 残留进程（每轮测试前必做） |

## 仿真 + MoveIt（so101_bringup）

```bash
ros2 launch so101_bringup demo.launch.py sim:=none|gazebo|mujoco [moveit:=true] [rviz:=false] [gz_gui:=false]
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `sim` | `none` | `none`（mock）/ `gazebo`（Harmonic）/ `mujoco` |
| `moveit` | `false` | 附加 move_group + MoveIt RViz |
| `rviz` | `true` | 基础 RViz |
| `gz_gui` | `true` | gazebo 模式 GUI |

## draw cube 验收

```bash
ros2 launch so101_robotics_torque_app draw_cube.launch.py rviz:=true \
  kp:=150.0,150.0,150.0,150.0,150.0 kd:=15.0,15.0,15.0,15.0,15.0
ros2 launch so101_robotics_torque_app draw_cube.launch.py rviz:=false ctrl_mode:=position
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `ctrl_mode` | `torque` | `torque`（effort 命令）/ `position`（JTC 基线） |
| `controller` | `computed_torque` | `pd_g` / `computed_torque`（反馈线性化） |
| `rviz` | `true` | RViz 同步可视化（配置优先加载 src 源文件） |
| `kp` / `kd` | 50.0 / 5.0 | 逗号分隔 5 关节增益；实测 150/15 时 RMS 3.06~3.18 mm |
| `edge_duration` | `1.5` | 每条棱基准时长 s（按关节距离自适应拉伸） |

- 关闭 MuJoCo 或 RViz 窗口时整个 launch 自动退出（`on_exit=Shutdown`）。
- RViz 里 File→Save 直接写回 `src/so101_robotics_torque_app/rviz/draw_cube.rviz`。

## MuJoCo viewer 版（无 ROS）

```bash
uv run python scripts/robotics_draw_cube_viewer.py [--side 0.06] [--edge-duration 1.5] [--kp 150] [--kd 15]
```

- 加载 `src/so101_sim/mujoco/scene_torque.xml`，500 Hz 内环 computed torque。
- viewer.user_scn 覆盖层：绿色 cube 线框 + 红色 EE 轨迹 trail + 黄色 EE 标记。
- 启动即最大化窗口并隐藏左右 UI 面板；关闭窗口即停止并输出 RMS/Max 统计。

## 运行时检查

```bash
ros2 control list_controllers    # 控制器应均为 active
ros2 topic echo /joint_states    # 关节状态数据流
ros2 topic echo /draw_cube/arm_target   # torque 模式期望轨迹 (15 值)
```
