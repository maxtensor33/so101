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
uv run python scripts/setup_verify_model_alignment.py

# 启动 demo（三种仿真 + 可选 MoveIt）
ros2 launch so101_bringup demo.launch.py sim:=none|gazebo|mujoco [moveit:=true] [rviz:=false] [gz_gui:=false]

# draw cube 验收（torque 默认; position 基线; kp/kd 逗号列表各 5 关节）
ros2 launch so101_robotics_torque_app draw_cube.launch.py [ctrl_mode:=torque|position] [rviz:=true] [kp:=150.0,...] [kd:=15.0,...]

# envs: gymnasium 环境集（reach/lift/place/stack, mujoco 内核, 位置控制; plan/envs.md）
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1'); env.reset(); env.step([0.0]*6)"

# envs: 单环境快速验证（--render 存 /tmp 双视角 png）
uv run python scripts/envs_check_env.py --id SO101Lift-v1 [--steps 30] [--render]

# rl: 训练/评估（sb3/skrl 双后端; torch 走系统安装, uv 不重复装）
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Reach-v1 --steps 200000
uv run python scripts/rl_eval.py --ckpt outputs/rl/sb3-sac-reach --episodes 10

# rl: ROS2 推理可视化（加载 checkpoint 驱动 env, RViz 订阅）
ros2 launch so101_rl_infer rl_infer.launch.py env:=reach checkpoint:=outputs/rl/sb3-sac-reach [camera:=true] [rviz:=true]

# il_sim: 采集（官方 lerobot-record 薄封装; expert/keyboard; episode 自动重置; 默认弹 MuJoCo 窗口, 关闭用 --robot.render=false）
uv run python scripts/il_sim_record.py --robot.type=so101_sim --teleop.type=so101_expert \
  --dataset.repo_id=il_sim/reach_expert --dataset.root=outputs/datasets/il_sim/il_sim/reach_expert \
  --dataset.fps=10 --dataset.episode_time_s=8 --dataset.reset_time_s=0 --dataset.num_episodes=20 \
  --dataset.video=true --dataset.push_to_hub=false --dataset.single_task="reach the goal above the item"

# il_sim: 遥操 launch 验收（teleop 类型/是否录制/RViz; record 默认 false 不落盘）
ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=false [env:=reach] [rviz:=true] [mujoco:=false]

# il_sim: 训练（小网络冒烟; 全量迁移见 docs/il_sim/index.md; VQ-BeT 需单相机数据集）
uv run python scripts/il_sim_train.py --policy act --repo-id il_sim/reach_expert --steps 10

# il_sim: 推理（lerobot policy 驱动 env, MuJoCo 与 RViz 同步）
ros2 launch so101_il_sim_infer il_sim_infer.launch.py env:=reach checkpoint:=outputs/il_sim/<run>/checkpoints/<step>/pretrained_model [rviz:=true] [mujoco:=false]

# envs: mujoco viewer 交互窗口（C 键循环 free/fixed/wrist 视角; 需图形环境）
uv run python scripts/envs_view_env.py --id SO101Reach-v1 [--cam fixed] [--random]

# envs: gymnasium 标准 human 渲染（render_mode 透传, 同样弹 mujoco viewer 窗口）
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1', render_mode='human'); env.reset(seed=0); [env.step([0.0]*6) for _ in range(100)]; env.close()"

# envs: 脚本专家验收（reach 100%; lift/place/stack 抓取保持为已知问题）
uv run python scripts/envs_run_expert.py --env all --episodes 5

# envs: RViz 可视化（env:=reach|lift|place|stack, camera:=true 开双相机图像）
ros2 launch so101_envs_viz env_viz.launch.py [env:=reach] [camera:=false] [rviz:=true]  

# MuJoCo viewer 版 draw cube（无 ROS, lab-2 风格, 500Hz 内环）
uv run python scripts/robotics_robotics_draw_cube_viewer.py

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
- `libs/so101_rl` — RL 训练库（plan/rl.md）：TrainerSpec 抽象统一 sb3/skrl 双后端；checkpoint 目录约定 model.{zip|pt}+meta.json(+vecnormalize.pkl)（torch 已随 il_sim 改为 uv 直装，见 pyproject find-links/清华源）
- `src/so101_rl_infer` — RL 推理节点：加载 checkpoint 驱动 env，发布 /joint_states、/rl/{fixed,wrist}/image
- `src/so101_teleop` — lerobot Robot/Teleoperator 协议适配层（plan/il_sim.md）：SimSO101Robot（so101_envs 相机版后端）+ expert/keyboard teleop + ros_viz；`scripts/il_sim_record.py`/`il_sim_teleop.py` 采集/遥操 CLI
- `src/so101_il_sim_infer` — IL 推理包（镜像 rl_infer）：checkpoint 加载（policy/pre/post 三元组）+ env 驱动 + ROS 话题
- `src/so101_robotics_torque_app` — 力矩学习 app：torque_control_node + draw_cube_node + draw_cube.launch.py
- `tests/` + `scripts/setup_verify_model_alignment.py` — URDF↔XML 对齐验证（TDD）
- `scripts/robotics_robotics_draw_cube_viewer.py` — MuJoCo viewer 版 draw cube（lab-2 c3 风格 user_scn 覆盖层）
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
- rl（plan/rl.md）：sb3 函数式 callback 返回值 = continue_training（返回 None 会静默停止训练）；sb3 load 的 cloudpickle 会清空 gymnasium registry（自定义注册 env 必须快照恢复）；torch 走系统 site-packages，uv 用 override-dependencies marker hack 跳过
- envs（plan/envs.md）：env MJCF 是单动爪夹爪，爪口沿 gripper 系 x_g 上下开合；gripper actuator 已软化（kp=400, ±1.2Nm）；相机版 obs 为 Dict{state, rgb{fixed,wrist}}，EGL 渲染有 ±1 位级噪声（env_checker 需容差）；colcon + setuptools>=80 会把 console_scripts 装到 bin/，ros2 run 需 data_files 装 lib/<pkg>/ wrapper；lift/place/stack 抓取保持失败为已知问题（reports/envs_report.md）
- il_sim（plan/il_sim.md）：lerobot 0.6.1 已 uv 直装（numpy 2.x 共存，勿回退 numpy<2）；features 键名 `{motor}.pos: float`→observation.state/action、相机键→observation.images.<key>；`cameras` 是 lerobot RobotConfig 保留名（dict[str, CameraConfig]），相机子集用 `camera_keys`（VQ-BeT 只支持 1 路图像）；重复 repo_id 录制=删旧重录（`cli_config.remove_existing_dataset`，官方 record 遇已存在目录 FileExistsError）；episode 自动重置走 env_backend 握手（teleop 请求 → send_action 帧边界执行，边界帧 action 是对旧目标计算的）；ros_viz 线程不渲染（EGL 上下文线程绑定，主线程渲后经 env_backend 缓存转发）；colcon 树下包 `__init__` 须自举 repo libs/ + .venv site-packages + cmeel.prefix（pinocchio .pth 不随手动 sys.path 生效）；policy 推理设备以预处理器 to_device 步骤为准（checkpoint 存于 GPU 时 to cuda，节点侧勿再 to("cpu")）；SmolVLA=TODO（todo/il_sim_smolvla_todo.md）；gym-aloha（lerobot[all] 带入）会把 mujoco 压到 <3.9 破坏 envs 抓取物理，pyproject 已 override-dependencies 锁 mujoco>=3.9，勿删；全量 pytest 须在 ROS 环境下跑（裸 uv venv 缺 rclpy 会 ERROR 8 项）；MuJoCo viewer 窗口由 SimSO101Robot config `render` 开关（launch 参数 `mujoco:=true` 默认开，无显示环境置 false）；viewer 默认隐藏左右工具栏+最大化（`SO101*Env(viewer_ui=, viewer_maximize=)`，maximize 走 X11 EWMH 即 `so101_envs.utils.maximize_window_by_name`，Wayland 无效静默跳过）；viewer_no_input（不抢键盘焦点）KWin 下无效=TODO（todo/il_sim_viewer_no_input_todo.md，workaround=焦点留终端）；keyboard 录制自动重置：←/→/n/r 与成功到达均触发 env.reset（env_backend reset 计数 → teleop 保持目标吸附新 qpos，防机械臂弹回旧位姿）
