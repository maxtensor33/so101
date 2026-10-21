# Runbook：故障排查与运行手册

<!-- 内容来自 setup 与 robotics 两轮 TDD 的实战记录; 新踩的坑请追加到表末 -->

## 启动前检查

1. `bash scripts/clean_ros.sh` —— 清理残留进程（旧进程向 `/robot_description`
   发布会污染新会话，症状见排查表第 1 条）。
2. `source /opt/ros/jazzy/setup.zsh && source install/setup.zsh`（shell 是 zsh，
   setup.bash 会报错）。
3. MuJoCo 相关命令需 `export MUJOCO_GL=egl`（headless 渲染）。

## 故障排查表

| 症状 | 原因与处理 |
|---|---|
| gz 加载了错误的硬件插件（mujoco 字样）或 RViz 报 "Frame base_link does not exist" | 有其它模式的残留进程发布旧 `/robot_description`——`bash scripts/clean_ros.sh` 后重来 |
| gz 中机械臂倾倒 | URDF 缺 world 固定关节（本项目已修，勿删 `world_joint`） |
| 反复 "No clock received" | /clock 桥接缺失（demo.launch.py 已含 parameter_bridge） |
| `import pinocchio` 段错误 | venv 装了 numpy 2.x——保持 `numpy<2` + `--system-site-packages` |
| move_group 启动即崩 "plugin name is empty" | MoveIt 2.12 参数名是 `planning_plugins`（复数），用 MoveItConfigsBuilder |
| ros2 CLI 没反应 | daemon 假死：`ros2 daemon stop && ros2 daemon start` |
| mujoco headless 崩溃 | `export MUJOCO_GL=egl` |
| ForwardCommandController 报 interface 类型错误 | `interface_name` 必须是标量 `effort`，YAML 数组会被拒 |
| 力矩保持时机械臂缓慢下垂 | 纯重力补偿是中性稳定（无位置反馈）——`torque_control_node` 无目标时锁存 `q_hold` 做 PD+重力补偿，不要改回纯 g-comp |
| t=0 时 MuJoCo 报一次 "NaN in CTRL at ACTUATOR 0" | mujoco_ros2_control 上游接口初始值是 quiet_NaN 的启动瞬态，首帧命令后消失，无需处理 |
| 改了 rviz/launch 配置 build 后失效 | ament_python 的 data_files 是拷贝而非 symlink；draw_cube 的 RViz 配置已改为源文件优先加载，其余包改配置需重新 build 或改 src 后同步 |
| 轨迹执行中力矩饱和失控 | IK 冗余分支翻转导致相邻航点关节跳变——段时长必须按关节距离自适应（已实现于 draw_cube_node / draw_cube_viewer） |
| ROS 节点报 `No module named 'so101_robotics'` | uv+colcon 混合环境：节点靠 sys.path 自举（向上找 pyproject.toml 后加 `libs/` 与 `.venv`）；launch 的 `additional_env` 在 Jazzy 实测不可靠，不要依赖 |
| ament_python 包改了 launch/urdf/rviz 后行为没变 | 这类包的 data_files 是拷贝非 symlink，改动后必须 `colcon build`；或参照 draw_cube 的做法在代码里直接解析 src 源文件 |

## 验证链路是否健康

```bash
ros2 control list_controllers      # 三个控制器 active
ros2 topic hz /joint_states        # ~100 Hz
```

- 力矩链路端到端：`ros2 launch so101_robotics_torque_app draw_cube.launch.py
  rviz:=false kp:=150.0,... kd:=15.0,...`，日志出现
  `DONE samples=... rms=...` 且 rms < 5 mm 为达标。
- 模型一致性回归：`uv run pytest`（含对齐测试）+ 两个对齐验证脚本。

## 已知限制

- 力矩仿真为理想执行器模型（无舵机动力学/温升/堵转保护），实测数据不能直接
  外推到真机。
- ament_python 包（`so101_robotics_torque_app`）的静态资源是 build 拷贝——
  `--symlink-install` 只对 ament_cmake 的 install() 生效。
- mujoco_ros2_control 启动首帧可能有一条 NaN ctrl 警告（上游初始值行为），
  一次性、无后续影响。
