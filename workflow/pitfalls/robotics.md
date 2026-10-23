# robotics（力矩控制 / draw cube）

| 症状 | 原因与处理 |
|---|---|
| ForwardCommandController 报 interface 类型错误 | `interface_name` 必须是标量 `effort`，YAML 数组会被拒 |
| 力矩保持时机械臂缓慢下垂 | 纯重力补偿是中性稳定——`torque_control_node` 锁存 `q_hold` 做 PD+重力补偿 |
| t=0 时 MuJoCo 报一次 "NaN in CTRL at ACTUATOR 0" | mujoco_ros2_control 启动瞬态，首帧命令后消失 |
| 轨迹执行中力矩饱和失控 | IK 冗余分支翻转导致相邻航点关节跳变（1.5+ rad 大段）——段时长按关节距离自适应（已实现） |
| EE 跟踪精度参考 | computed torque（KP/KD=150/15）实测 RMS 3.06~3.18 mm；position 基线（JTC）9.67 mm。详见 `workflow/reports/draw_cube_report.md` |
