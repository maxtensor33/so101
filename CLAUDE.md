# CLAUDE.md

SO-ARM101 机械臂 ROS2 Jazzy 工作区（uv + colcon 混合管理）。

## 常用命令

```bash
# 环境（shell 是 zsh，不要 source setup.bash）
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh

# 构建
colcon build --symlink-install

# Python 测试（全量回归）
uv run pytest

# URDF↔MuJoCo 对齐验证 CLI（位置版输出 alignment_report.md; --torque 输出 alignment_report_torque.md, 默认分文件）
uv run python scripts/setup_verify_model_alignment.py

# 启动 demo（三种仿真 + 可选 MoveIt）
ros2 launch so101_bringup demo.launch.py sim:=none|gazebo|mujoco [moveit:=true] [rviz:=false] [gz_gui:=false]

# draw cube 验收（torque 默认; position 基线; kp/kd 逗号列表各 5 关节）
ros2 launch so101_robotics_torque_app draw_cube.launch.py [ctrl_mode:=torque|position] [rviz:=true] [kp:=150.0,...] [kd:=15.0,...]

# envs: gymnasium 环境集（reach/lift/place/stack × cube/can 共 8 任务, mujoco 内核, 位置控制; workflow/plan/envs.md）
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1'); env.reset(); env.step([0.0]*6)"

# envs: 单环境快速验证（--render 存 /tmp 双视角 png）
uv run python scripts/envs_check_env.py --id SO101Lift-v1 [--steps 30] [--render]

# rl: 训练/评估（sb3/skrl 双后端; torch 走系统安装, uv 不重复装）
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Reach-v1 --steps 200000
uv run python scripts/rl_eval.py --ckpt outputs/rl/sb3-sac-reach-v1/latest --episodes 10
# rl: lift（严格对齐 squint: delta 控制 / 2.2-2.8cm 密度200 物体 / squint SAC 超参; 默认即此配置）
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Lift-v1 \
  --steps 1500000 --eval-freq 20000 --n-envs 4 --out outputs/rl/lift-squint
# 对照: --action-mode position / --no-domain-randomization / --config none

# rl: ROS2 推理可视化（加载 checkpoint 驱动 env, RViz 订阅; mujoco:=true 开同一 env 的 MuJoCo 窗口;
#      默认 Enter 重置 episode, reset_on_enter:=false 关闭）
ros2 launch so101_rl_infer rl_infer.launch.py env:=reach checkpoint:=outputs/rl/sb3-sac-reach-v1/latest [camera:=true] [rviz:=true] [mujoco:=true] [reset_on_enter:=true]

# il_sim: 采集（官方 lerobot-record 薄封装; expert/keyboard; episode 自动重置; 默认弹 MuJoCo 窗口, 关闭用 --robot.render=false）
uv run python scripts/il_sim_record.py --robot.type=so101_sim --teleop.type=so101_expert \
  --dataset.repo_id=il_sim/reach_expert --dataset.root=outputs/datasets/il_sim/il_sim/reach_expert \
  --dataset.fps=10 --dataset.episode_time_s=8 --dataset.reset_time_s=0 --dataset.num_episodes=20 \
  --dataset.video=true --dataset.push_to_hub=false --dataset.single_task="reach the goal above the item"

# il_sim: 遥操 launch 验收（teleop 类型/是否录制/RViz; record 默认 false 不落盘）
ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=false [env:=reach] [rviz:=true] [mujoco:=false]

# il_sim: leader 臂遥操/采集（真机 leader=ACM0; 先跑探针校方向, 反向关节加 --invert;
# 采集用 il_sim_record.py 走 draccus 透传 --teleop.type=so101_leader_sim --teleop.port=... --teleop.id=...）
uv run python scripts/il_sim_leader_probe.py [--invert <关节>]
uv run python scripts/il_sim_teleop.py --teleop leader [--record] [--env reach] [--invert <关节>]

# il_sim: RL 专家自动采集（8 环境通用: reach/lift/place/stack × cube/can; env 由 checkpoint
#   meta 推导, --env liftcan 可覆盖; 默认只保存成功 episode, 沿用 checkpoint 的 domain_randomization,
#   每 episode 50 步后手工 reset; VQ-BeT 单相机加 --camera-keys fixed; 数据可直接训练）
uv run python scripts/il_sim_collect_rl.py --checkpoint outputs/rl/lift-squint \
  --repo-id il_sim/lift_rl_expert --episodes 50 [--env liftcan] [--keep-failures] [--no-mujoco]

# il_sim: 双相机 dataset -> 单相机（VQ-BeT 对照; 默认保留 fixed 视角, 新 repo/目录名后缀 _single）
uv run python scripts/il_sim_dataset_single_camera.py \
  --dataset-root outputs/datasets/il_sim/il_sim/lift_rl_expert [--keep fixed] [--no-video] [--dry-run]

# il_sim: 训练（小网络冒烟; 全量迁移见 docs/il_sim/index.md; VQ-BeT 需单相机数据集）
uv run python scripts/il_sim_train.py --policy act --repo-id il_sim/reach_expert --steps 10

# il_sim: 推理（lerobot policy 驱动 env, MuJoCo 与 RViz 同步）
ros2 launch so101_il_sim_infer il_sim_infer.launch.py env:=reach checkpoint:=outputs/il_sim/<run>/checkpoints/<step>/pretrained_model [rviz:=true] [mujoco:=false] [reset_on_enter:=true]

# il_sim_develop: 实机 IL 部署开发（lerobot policy 驱动真机; real:=true 底座内置; estop 发布 Empty 到
#   /il_sim_develop/estop 即停发; Enter=homing 回 rest; confirm_step:=true 逐单步; 相机内置 cv2 节点 top->fixed/wrist;
#   端口默认 follower by-id（leader=ACM0 陷阱见 pitfalls）; 设备映射改 config/cameras.yaml;
#   RViz 布局沿用 envs 双相机版, 开关 gui:=false（rviz 为兼容别名）;
#   真机 checklist 见 workflow/reports/il_sim_develop.md）
ros2 launch so101_il_sim_develop il_sim_develop.launch.py checkpoint:=outputs/il_sim/<run>/checkpoints/<step>/pretrained_model [env:=reach] [camera_keys:=fixed,wrist] [rate:=10.0] [confirm_step:=false] [gui:=true]

# camera_calibration: 相机内参/手眼标定（迁移自 so101-ros-physical-ai; 中文详解 docs/hardware/camera_calibration.md）
uv run python src/so101_camera_calibration/scripts/gen_charuco_intrinsic.py   # A4 板生成（handeye 同理; 打印要求见 docs）
uv run python src/so101_camera_calibration/scripts/export_envs_camera_calib.py \
  --handeye outputs/calibration/handeye_eye_in_hand.yaml --intrinsic outputs/calibration/wrist/camera_cal.yaml \
  --handeye outputs/calibration/handeye_eye_to_hand.yaml  --intrinsic outputs/calibration/fixed/camera_cal.yaml
ros2 run so101_camera_calibration camera_intrinsic_calibration_node --ros-args \
  -p image_topic:=/real/fixed/image_raw -p output_dir:=outputs/calibration/fixed   # Viser UI :8080
ros2 launch so101_camera_calibration camera_calibration.launch.py camera:=eye_in_hand \
  intrinsic_yaml:=outputs/calibration/wrist/camera_cal.yaml [mode:=auto] [port:=<follower by-id>]
# envs 相机定制（sim 渲染=真机视野; 标定默认开启, 默认 outputs/calibration/envs_camera_calib.yaml,
#   缺文件回退未标定; gym.make(..., camera_calib=False) 关闭, 传路径用自定义标定））

# mtc: MoveIt Task Constructor pick-and-place（纯 RViz 模式; mujoco/gz 后端暂停见 workflow/todo/）
ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=rviz [execute:=false] [rviz:=true]
uv run python scripts/mtc_probe_pick_poses.py   # 可达性探测（摆位/校准变更后）

# hardware: 实机控制（workflow/plan/setup_hardware.md; 真机验收清单见 workflow/pitfalls/setup_hardware.md H7）
uv run python scripts/setup_convert_lerobot_calibration.py [--invert gripper]  # LeRobot 标定 JSON → motor_calibration.yaml
uv run python scripts/setup_capture_named_pose.py   # 力矩失能手摆 → 命名姿势 → SRDF group_state + named_poses.yaml（真机）
ros2 launch so101_bringup demo.launch.py sim:=none real:=true    # 纯实机（无零位要求, 从当前位置起步）
ros2 launch so101_bringup demo.launch.py sim:=mujoco real:=true [moveit:=true] [twin:=false]  # 数字孪生: 实机被控 + MuJoCo 镜像
# 可选 port:=/dev/ttyACM0 calibration_file:=<yaml>; real:=true+sim:=gazebo 不支持

# envs: mujoco viewer 交互窗口（C 键循环 free/fixed/wrist 视角; 需图形环境）
uv run python scripts/envs_view_env.py --id SO101Reach-v1 [--cam fixed] [--random]

# envs: gymnasium 标准 human 渲染（render_mode 透传, 同样弹 mujoco viewer 窗口）
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1', render_mode='human'); env.reset(seed=0); [env.step([0.0]*6) for _ in range(100)]; env.close()"

# envs: 脚本专家（仅 reach/can 在 CLI; squint 迁移后 reach 100%, 其余专家已移出）
uv run python scripts/envs_run_expert.py --env all --episodes 5

# envs: RViz 可视化（env:=reach|lift|place|stack, camera:=true 开双相机图像）
ros2 launch so101_envs_viz env_viz.launch.py [env:=reach] [camera:=false] [rviz:=true]  

# MuJoCo viewer 版 draw cube（无 ROS, lab-2 风格, 500Hz 内环）
uv run python scripts/robotics_draw_cube_viewer.py

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
- `libs/so101_envs` — 无 ROS 依赖的 gymnasium 环境库（workflow/plan/envs.md）：纯 MuJoCo 内核 + 位置控制动作空间 + fixed/wrist 双相机 + pinocchio IK；`assets/` 自含场景 MJCF 与 STL（与 src/so101_sim/mujoco 独立）
- `src/so101_envs_viz` — envs 的 RViz 可视化包：viz_node 发布 /joint_states、物体 TF、/envs/{fixed,wrist}/image
- `libs/so101_rl` — RL 训练库（workflow/plan/rl.md）：TrainerSpec 抽象统一 sb3/skrl 双后端；checkpoint 目录约定 model.{zip|pt}+meta.json(+vecnormalize.pkl)（torch 已随 il_sim 改为 uv 直装，见 pyproject find-links/清华源）
- `src/so101_rl_infer` — RL 推理节点：加载 checkpoint 驱动 env，发布 /joint_states、/rl/{fixed,wrist}/image
- `src/so101_teleop` — lerobot Robot/Teleoperator 协议适配层（workflow/plan/il_sim.md）：SimSO101Robot（so101_envs 相机版后端）+ expert/keyboard teleop + ros_viz；`scripts/il_sim_record.py`/`il_sim_teleop.py` 采集/遥操 CLI
- `src/so101_il_sim_infer` — IL 推理包（镜像 rl_infer）：checkpoint 加载（policy/pre/post 三元组）+ env 驱动 + ROS 话题
- `src/so101_il_sim_develop` — il_sim 实机部署开发包（workflow/plan/il_sim_develop.md）：il_sim_develop_node（obs 桥 + estop + homing + confirm_step）+ 内置 cv2 相机节点 + launch；复用 il_sim_infer.core；`il_real` 命名空间留给后续正式实机包
- `src/so101_mtc_pick_place` — MTC pick-and-place（workflow/plan/mtc_rviz.md）：扁平 stage 链节点（mtc_pick_place_node）+ kinematic attach 插件（仿真后端用）+ sim:=rviz|mujoco|gazebo launch（后两者暂停）
- `src/so101_hardware` — 实机 ros2_control 插件（workflow/plan/setup_hardware.md）：C++ SystemInterface + vendor SCServo SDK；标定 YAML 驱动电机 ID/量程，rad↔raw 全行程线性映射，无零位要求；`scripts/setup_convert_lerobot_calibration.py` 生成标定、`scripts/setup_capture_named_pose.py` 采集命名姿势
- `src/so101_robotics_torque_app` — 力矩学习 app：torque_control_node + draw_cube_node + draw_cube.launch.py
- `tests/` + `scripts/setup_verify_model_alignment.py` — URDF↔XML 对齐验证（TDD）
- `scripts/robotics_draw_cube_viewer.py` — MuJoCo viewer 版 draw cube（lab-2 c3 风格 user_scn 覆盖层）
- `workflow/` — 开发流程工件（非运行时, 约定见 workflow/README.md）：`plan/` 各方向原始计划、`tdd/` TDD 证据（setup/robotics/envs/rl/il_sim/il_sim_leader/mtc/setup_hardware 变体）、`reports/` 验收报告（全方向, 含 squint 迁移）、`todo/` 暂停待办、`learn/` 经验复盘、`problems/` 跨会话调试交接（结案带横幅）、`pitfalls/` 按方向坑（单源进 docs 站）、`reviews/` 评审记录
- `workflow/reports/` — 各方向验收报告；对齐验证分位置版（`alignment_report.md`）与力矩版（`alignment_report_torque.md`, `--torque` 默认分文件防覆盖）

## 关键约定与坑

- **所有 git commit 必须先征得用户同意，不得擅自提交**
- 其余约定与坑**按方向分类**放在 `workflow/pitfalls/`（索引见其 README）：
  general（环境/colcon/pytest）/ setup（URDF·对齐）/ setup_hardware（实机）/ robotics / mtc / envs / rl / il_sim；
  ros2 集成类另见 skill: ros2-jazzy-sim-integration-pitfalls
- 方向原始计划在 `workflow/plan/`，TDD 证据在 `workflow/tdd/`，暂停待办在 `workflow/todo/`
