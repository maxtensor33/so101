# 通用 / 环境

| 症状 | 原因与处理 |
|---|---|
| gz 加载了错误的硬件插件（mujoco 字样）或 RViz 报 "Frame base_link does not exist" | 残留进程发布旧 `/robot_description`——`bash scripts/clean_ros.sh` 后重来 |
| **诊断命令被莫名杀掉（exit 144）** | `clean_ros.sh` 的 `pkill -f "rviz2"` 等按完整命令行匹配——诊断命令行里含同名字面量会被自杀；用变量拼接（`K=rviz; pgrep -cx ${K}2`）或 clean 与诊断分开执行 |
| 反复 "No clock received" | /clock 桥接缺失（demo.launch.py 已含 parameter_bridge；mtc/其他自组 launch 须自行加） |
| move_group 启动即崩 "plugin name is empty" | MoveIt 2.12 参数名是 `planning_plugins`（复数），用 MoveItConfigsBuilder |
| ros2 CLI 没反应 | daemon 假死：`ros2 daemon stop && ros2 daemon start` |
| mujoco headless 崩溃/黑屏 | `export MUJOCO_GL=egl` |
| ROS 节点报 `No module named 'so101_robotics'` / 'pinocchio' | uv+colcon 混合环境 sys.path 自举（向上找 pyproject.toml），**须包含 `.venv` site-packages + `cmeel.prefix/.../site-packages`**（pinocchio 的 .pth 不随手动 sys.path 生效）；launch 的 `additional_env` 在 Jazzy 实测不可靠 |
| ament_python 包改了 launch/urdf/rviz 后行为没变 | data_files 是拷贝非 symlink，改动后必须 `colcon build` |
| ament_cmake 包改了 .cpp 后行为没变化 | symlink-install 增量编译不可靠——`colcon build --packages-select <pkg> --cmake-clean-first` |
| 全量 pytest 报 8 项 rclpy ERROR | 全量测试须在 ROS 环境下跑（`source /opt/ros/jazzy/setup.zsh && source install/setup.zsh` 后 `uv run pytest`） |
| 裸 uv venv 跑 pytest 缺 ROS 模块 | 同上；pytest 已在 pyproject addopts 禁用 ROS 的 launch_testing 插件（勿删） |
| console_scripts 装到 bin/，`ros2 run` 找不到 | colcon + setuptools>=80 行为——ament_python 包 data_files 装 `lib/<pkg>/` wrapper（envs_viz/teleop/il_sim_infer 均此模式） |
| zsh 报错 | 环境加载用 `source /opt/ros/jazzy/setup.zsh`（setup.bash 会报错） |
