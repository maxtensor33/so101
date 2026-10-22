# CLAUDE.md

SO-ARM101 机械臂 ROS2 Jazzy 工作区（uv + colcon 混合管理）。

## 常用命令

```bash
# 环境（shell 是 zsh，不要 source setup.bash）
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh

# 构建
colcon build --symlink-install

# Python 测试（对齐验证, 9 tests）
uv run pytest

# URDF↔MuJoCo 对齐验证 CLI（输出 reports/alignment_report.md）
uv run python scripts/verify_model_alignment.py

# 启动 demo（三种仿真 + 可选 MoveIt）
ros2 launch so101_bringup demo.launch.py sim:=none|gazebo|mujoco [moveit:=true] [rviz:=false] [gz_gui:=false]

# draw cube 验收（torque 默认; position 基线; kp/kd 逗号列表各 5 关节）
ros2 launch so101_robotics_torque_app draw_cube.launch.py [ctrl_mode:=torque|position] [rviz:=true] [kp:=150.0,...] [kd:=15.0,...]

# envs: gymnasium 环境集（reach/lift/place/stack, mujoco 内核, 位置控制; plan/envs.md）
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1'); env.reset(); env.step([0.0]*6)"

# envs: 单环境快速验证（--render 存 /tmp 双视角 png）
uv run python scripts/check_env.py --id SO101Lift-v1 [--steps 30] [--render]

# envs: mujoco viewer 交互窗口（C 键循环 free/fixed/wrist 视角; 需图形环境）
uv run python scripts/view_env.py --id SO101Reach-v1 [--cam fixed] [--random]

# envs: gymnasium 标准 human 渲染（render_mode 透传, 同样弹 mujoco viewer 窗口）
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1', render_mode='human'); env.reset(seed=0); [env.step([0.0]*6) for _ in range(100)]; env.close()"

# envs: 脚本专家验收（reach 100%; lift/place/stack 抓取保持为已知问题）
uv run python scripts/run_expert.py --env all --episodes 5

# envs: RViz 可视化（env:=reach|lift|place|stack, camera:=true 开双相机图像）
ros2 launch so101_envs_viz env_viz.launch.py [env:=reach] [camera:=false] [rviz:=true]  

# MuJoCo viewer 版 draw cube（无 ROS, lab-2 风格, 500Hz 内环）
uv run python scripts/draw_cube_viewer.py

# 清理残留进程（每轮测试前必做, 防止旧进程向 /robot_description 发布污染）
bash scripts/clean_ros.sh
```

## 结构速览

- `models/` — 上游模型副本（对齐验证脚本用）；`src/so101_sim/mujoco/` 是仿真实际加载的 MJCF（默认 scene.xml 含地板）
- `src/so101_description` — URDF（根为 world link）+ meshes + rviz 配置
- `src/so101_sim` — ros2_control 三硬件宏（mock/gz/mujoco）+ controllers.yaml
- `src/so101_bringup` — demo.launch.py（OpaqueFunction 按 sim 分支）
- `src/so101_moveit_config` — MoveItConfigsBuilder + SRDF/kinematics/joint_limits/ompl
- `libs/so101_robotics` — 无 ROS 依赖的学习库：fk/jacobian/ik/dynamics/trajectory/control/constraints/sim（uv wheel 安装；ROS 节点靠 sys.path 自举导入）
- `libs/so101_envs` — 无 ROS 依赖的 gymnasium 环境库（plan/envs.md）：纯 MuJoCo 内核 + 位置控制动作空间 + fixed/wrist 双相机 + pinocchio IK；`assets/` 自含场景 MJCF 与 STL（与 src/so101_sim/mujoco 独立）
- `src/so101_envs_viz` — envs 的 RViz 可视化包：viz_node 发布 /joint_states、物体 TF、/envs/{fixed,wrist}/image
- `src/so101_robotics_torque_app` — 力矩学习 app：torque_control_node + draw_cube_node + draw_cube.launch.py
- `tests/` + `scripts/verify_model_alignment.py` — URDF↔XML 对齐验证（TDD）
- `scripts/draw_cube_viewer.py` — MuJoCo viewer 版 draw cube（lab-2 c3 风格 user_scn 覆盖层）
- `.claude/tdd/setup.tdd.md`, `.claude/tdd/robotics.tdd.md` — TDD 证据报告；`plan/setup.md`, `plan/robotics.md` — 原始计划
- `reports/` — 对齐验证与 draw cube 验收报告

## 关键约定与坑（详见 skill: ros2-jazzy-sim-integration-pitfalls）

- **所有 git commit 必须先征得用户同意，不得擅自提交**
- 系统 pinocchio 按 NumPy 1.x 编译：venv 必须 `--system-site-packages` + `numpy<2`
- pytest 已在 pyproject addopts 禁用 ROS 的 launch_testing 插件，勿删
- URDF 根是 `world`（fixed joint 固定 base_link），SRDF 因此没有 virtual_joint
- MoveIt 2.12 参数名 `planning_plugins`（复数）；配置经 MoveItConfigsBuilder 生成
- gz 内嵌 controller_manager 需要 `/clock` bridge（demo.launch.py 已加）
- zsh 环境：`source /opt/ros/jazzy/setup.zsh`（setup.bash 会报错）
- mujoco headless 需 `export MUJOCO_GL=egl`
- 力矩链路：ForwardCommandController 的 `interface_name` 是标量 `effort`；纯重力补偿中性稳定会漂移，保持需锁存位形 + PD；IK 冗余分支翻转会产生 1.5+ rad 大段，轨迹段时长须按关节距离自适应（`reports/draw_cube_report.md`）
- envs（plan/envs.md）：env MJCF 是单动爪夹爪，爪口沿 gripper 系 x_g 上下开合；gripper actuator 已软化（kp=400, ±1.2Nm）；相机版 obs 为 Dict{state, rgb{fixed,wrist}}，EGL 渲染有 ±1 位级噪声（env_checker 需容差）；colcon + setuptools>=80 会把 console_scripts 装到 bin/，ros2 run 需 data_files 装 lib/<pkg>/ wrapper；lift/place/stack 抓取保持失败为已知问题（reports/envs_report.md）
