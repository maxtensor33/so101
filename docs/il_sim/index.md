# il_sim：模仿学习（仿真数据采集 / 训练 / 推理）

**定位**：用 `so101_envs` 相机版环境采集 lerobot 格式（LeRobotDataset v3）数据，
经 lerobot 官方管线训练 ACT / Diffusion Policy / VQ-BeT，再由 ROS2 推理节点驱动
env 实现 MuJoCo 与 RViz 同步可视化。**不使用 ROS2 采集/训练**——采集复用官方
`lerobot-record`，il_real 阶段换真机 Robot 后端即可复用全部采集命令。

## 架构

```
so101_teleop (lerobot 协议适配层, src/so101_teleop)
├── SimSO101Robot(Robot)      ──▶ 官方 record(cfg) ──▶ LeRobotDataset v3
│   └─ env 后端: SO101*Camera-v1          outputs/datasets/il_sim/<repo_id>/
├── ScriptExpertTeleop (so101_expert)
├── So101KeyboardTeleop (so101_keyboard)
├── GamepadTeleopPlaceholder (so101_gamepad, TODO)
└── leader = lerobot 自带 so101_leader (il_real 直接用)
ros_viz (采集/遥操内置启用): /joint_states + /il/{fixed,wrist}/image + 物体 TF ──▶ RViz
推理: src/so101_il_sim_infer (ROS2 节点, 镜像 rl_infer)
```

- `SimSO101Robot`：features 键 `{motor}.pos: float` → 派生 `observation.state`/`action`
  (shape 6，归一化 [-1,1])；相机键 → `observation.images.<key>`。
  `send_action` 内含 `env.step`；专家 SETTLE 结束 → `env_backend` reset 握手 →
  **帧边界对齐的 episode 自动重置**（官方 record 无 reset 概念，仿真由适配层补齐）。
- `ScriptExpertTeleop`：pinocchio IK 状态机 WAIT→GO→SETTLE（复用 envs 脚本专家逻辑），
  经 `env_backend` 共享 env 闭环（Teleoperator 协议无 obs 入参）。
- `So101KeyboardTeleop`：`1-6` 选关节，`w/s` 增/减，`c` 手动请求场景重置。
  **keyboard 录制全自动重置**：`→`/`n`（提前完成）、`←`/`r`（重录）、成功到达目标
  （`auto_reset_on_success`）三者都会在帧边界自动 `env.reset`，无需人工复位；
  reset 后保持目标自动吸附新 qpos（否则位置控制会把机械臂弹回旧目标位姿）。
- 环境进程自举：colcon install 树下由包 `__init__` 注入 repo `libs/`、uv venv
  site-packages 与 cmeel.prefix（pinocchio 的 .pth 不随手动 sys.path 生效）。

## 命令

### 采集（官方 CLI 薄封装；默认弹 MuJoCo 窗口，无显示环境加 `--robot.render=false`；
重复 repo_id = 删旧重录，不覆盖保留旧数据）

```bash
# 脚本专家采集 reach（episode 自动重置；ros_viz 需 ROS 环境才有输出）
uv run python scripts/il_sim_record.py \
  --robot.type=so101_sim --robot.env_id=SO101ReachCamera-v1 \
  --teleop.type=so101_expert \
  --dataset.repo_id=il_sim/reach_expert \
  --dataset.root=outputs/datasets/il_sim/il_sim/reach_expert \
  --dataset.fps=10 --dataset.episode_time_s=8 --dataset.reset_time_s=0 \
  --dataset.num_episodes=20 --dataset.video=true --dataset.push_to_hub=false \
  --dataset.single_task="reach the goal above the item"

# VQ-BeT 用单相机数据集（它只支持 1 路图像输入）
#   加 --robot.camera_keys='["fixed"]'

# 遥操主进程（record 默认 false 不落盘; 两种模式均内置 ros_viz）
uv run python scripts/il_sim_teleop.py --teleop keyboard --env reach
uv run python scripts/il_sim_teleop.py --teleop expert --record --repo-id il_sim/reach_expert --episodes 10
```

### launch 验收（ROS2）

```bash
# 遥操/采集：自动启动环境进程(内含 env+ros_viz) + RViz；record 默认 false
ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=false [env:=reach] [rviz:=true] [mujoco:=false]
ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=true repo_id:=il_sim/reach_expert episodes:=10
```

### 训练（不使用 ROS2）

```bash
# 冒烟（小网络模板, steps=10 级别）
uv run python scripts/il_sim_train.py --policy act      --repo-id il_sim/reach_expert --steps 10
uv run python scripts/il_sim_train.py --policy diffusion --repo-id il_sim/reach_expert --steps 10
# VQ-BeT 需单相机数据集
uv run python scripts/il_sim_train.py --policy vqbet    --repo-id il_sim/single_cam    --steps 10
# 产物: outputs/il_sim/<policy>_<name>/checkpoints/<step>/pretrained_model/
```

**迁移全量训练（别的机器）**：官方 `lerobot-train` 默认超参 + 同一数据集，等价命令：

```bash
lerobot-train \
  --dataset.repo_id=il_sim/reach_expert \
  --dataset.root=outputs/datasets/il_sim/il_sim/reach_expert \
  --policy.type=act --steps=100000 --batch-size=8 \
  --output_dir=outputs/il_sim/act_full
# SmolVLA = TODO：需 smolvla_base 预训练权重（HF hub ~1GB+）+ 显存评估，见 todo/
```

### 推理（ROS2, MuJoCo 与 RViz 同步）

```bash
ros2 launch so101_il_sim_infer il_sim_infer.launch.py env:=reach \
  checkpoint:=outputs/il_sim/<policy>_<name>/checkpoints/<step>/pretrained_model \
  [camera:=true] [rviz:=true] [rate:=10.0] [mujoco:=false]
```

## 已知限制与坑

- **episode 边界**：官方 record 按时间切 episode；专家一次 reach 约 6s（10Hz），episode_time_s
  取 8s 使一次 attempt ≈ 一个 dataset episode。reset 发生在帧边界（send_action 末尾），
  边界帧的 action 是对旧目标计算后世界重置——单帧不连续，ACT chunk 训练可容忍。
- **VQ-BeT 单相机**：lerobot 0.6.1 `VQBeTConfig.validate_features` 强制 1 路图像；
  用 `--robot.camera_keys='["fixed"]'` 录单相机数据集。
- **policy 推理设备**：checkpoint 保存于 GPU 时预处理器 to_device=cuda，
  `load_policy` 已把权重对齐到该设备（无 GPU 回退 CPU）。
- **MuJoCo 窗口**：`SimSO101RobotConfig.render=True`（launch `mujoco:=true` 默认）→ env 以 human 模式
  创建，viewer 惰性弹出、step 自动 sync；无显示环境（服务器采集）用 `mujoco:=false`。
  viewer 默认**隐藏左右工具栏 + 最大化**（`viewer_ui=False, viewer_maximize=True`；
  工具栏 Tab/Shift+Tab 可切，maximize 走 X11 EWMH，Wayland 静默跳过）。
  键盘焦点抑制（`viewer_no_input`）当前无效（TODO 见 `todo/il_sim_viewer_no_input_todo.md`），
  workaround：焦点保持在终端——viewer 字母键为 C++ 层渲染开关，仅鼠标交互即可。
- **推理帧率**：diffusion 小网络 ~5-6Hz（GPU），ACT 更快；`rate` 参数可调。
- **SmolVLA**：TODO（`todo/il_sim_smolvla_todo.md`），当前 `small_policy_config("smolvla")`
  显式 NotImplementedError。
