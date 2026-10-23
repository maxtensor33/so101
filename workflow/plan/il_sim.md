# Plan: SO101 il_sim —— 模仿学习（仿真数据采集/训练/推理）

**日期**: 2026-10-23
**状态**: 待确认
**复杂度**: Large（约 12~16 小时）
**前置**: envs 与 rl 已合并 main（相机版环境 SO101ReachCamera-v1、rl_infer 模式可镜像）

## 需求重述

1. 用 **envs 相机版环境**采集 il 数据（`SO101ReachCamera-v1`），采集时 **RViz 显示
   相机图片**；数据集符合 **lerobot 格式**（LeRobotDataset），输出
   `outputs/datasets/il_sim/`。lerobot 实现细节对照本地源码（`~/Third/Soft/lerobot`）。
2. 采集支持 **四种 teleop 方式**，以 `so101_teleop` 抽象实现（il_real 阶段换真机
   Robot 后端即可复用全部采集代码）：脚本专家（**reach 即可**）、键盘操作、
   游戏手柄（**TODO**）、leader 主臂（**TODO**）。
3. il 训练**不使用 ROS2**，纯 lerobot：**必须跑通 ACT / Diffusion Policy / VQ-BeT**
   三个策略；**SmolVLA 降级为 TODO 先不实现**（依赖 `smolvla_base` 预训练权重
   HF hub 下载与显存评估，待办清单见 todo/）。
   训练只跑通流程（可缩减网络），**长训由用户迁移到别的机器执行**。
4. il 推理使用 **ROS2**，支持 **MuJoCo 与 RViz 同步可视化**。
5. 在主目录切换 **git 分支 `il_sim`** 开发（主工作区 venv 已含 lerobot 0.6.1，
   numpy<2 共存实测可用）；commit 由用户手动执行。

## 环境事实（已勘察确认）

| 项 | 状态 |
|---|---|
| lerobot 源码 | `~/Third/Soft/lerobot`（dataset v3 格式；policies/{act,diffusion,vqbet,smolvla} 齐备） |
| record 管线 | `lerobot_record.py::record()` 基于 Robot 协议（get_observation/send_action）+ Teleoperator 协议（get_action）→ 自动写 LeRobotDataset v3（支持视频编码） |
| **官方采集 CLI** | `lerobot-record`（console_script → main()）：config 经 **draccus ChoiceRegistry** 注册（`@RobotConfig.register_subclass("...")`），`--robot.type=xxx --teleop.type=yyy` 直接解析；`register_third_party_plugins()` 支持第三方包注册 |
| Teleoperator 现成 | lerobot 自带 keyboard/gamepad 实现 |
| SmolVLA | **TODO 先不实现**：`smolvla_base` finetune 需 hub 下载权重 + 显存评估（用户要求降级） |
| 训练入口 | `lerobot_train.py::train(TrainPipelineConfig)` |
| numpy | 用户实测 numpy<2 可用 lerobot——Phase 0 冒烟锁定实际版本与 API；若冲突回退独立 venv（备选不阻塞） |
| envs 相机版 | obs `Dict{state(25), rgb{fixed,wrist}}`；action Box(-1,1,6) |
| 脚本专家 | `envs_run_expert.py` 的 IK 状态机逻辑可复用 |
| 推理镜像 | `src/so101_rl_infer` 先例可镜像 |

## 架构设计

**核心思路**：`so101_teleop` 实现 lerobot 的 `Robot`/`Teleoperator` 协议并用
draccus 注册（`@RobotConfig.register_subclass("so101_sim")` 等）——采集**直接使用
官方 `lerobot-record` CLI**（`--robot.type=so101_sim --teleop.type=so101_expert`），
不自写数据集写入、不重造 record 循环。il_real 阶段把 env 后端换成真机 Robot
即可复用全部采集命令。`scripts/il_sim_record.py` 仅做薄封装：import 触发注册 +
挂载 ROS viz 线程 + 透传官方 CLI。

```
so101_teleop (lerobot 协议适配层)          lerobot 官方管线
├── SimSO101Robot(Robot)      ──────────▶ record(cfg) ──▶ LeRobotDataset v3
│   └─ env 后端: SO101ReachCamera-v1          │  outputs/datasets/il_sim/<repo_id>/
├── ScriptExpertTeleop(Teleoperator)          ▼
├── KeyboardTeleop(Teleoperator)       il_sim_train.py (TrainPipelineConfig)
├── GamepadTeleop(TODO)                ▶ ACT / Diffusion / VQ-BeT / SmolVLA
└── LeaderTeleop(TODO)                        │ checkpoint
ros_viz(采集内置启用): joint_states + 相机图像 ──▶ RViz
```

## 目录结构（目标）

```
so101/
├── src/so101_teleop/                 # ★ ROS2 包（ament_python）: lerobot 协议适配层
│   ├── package.xml / setup.py / wrapper/   # 镜像 envs_viz/rl_infer 模式
│   │   └── launch/teleop.launch.py    # 遥操验收 launch（teleop 类型/是否录制/RViz）
│   ├── so101_teleop/
│   │   ├── robot.py                  # SimSO101Robot(Robot): env 后端
│   │   ├── configs.py                # SimSO101RobotConfig(RobotConfig) 等
│   │   ├── ros_viz.py                # 采集时 RViz 同步显示（内置启用，非可选）
│   │   └── teleoperators/
│   │       ├── expert.py             # 脚本专家（IK 状态机, reach）
│   │       ├── keyboard.py           # 键盘关节控制
│   │       ├── gamepad.py            # TODO（lerobot 自带 gamepad 可低成本接入）
│   │       └── leader.py             # TODO（真机 so101_leader）
├── scripts/                          # <项目名>_<功能>.py 约定（il_sim 与 il_real 区分）
│   ├── il_sim_record.py              # 采集 CLI（注册 + ros_viz + 透传官方 lerobot-record）
│   └── il_sim_train.py               # 训练 CLI（TrainPipelineConfig → train()）
├── src/so101_il_sim_infer/           # ★ ROS2 推理包（镜像 rl_infer；il_sim 专属）
│   ├── so101_il_sim_infer/il_sim_infer_node.py  # 加载 lerobot policy → env 驱动 → ROS
│   ├── launch/il_sim_infer.launch.py · rviz/ · wrapper/
├── tests/
│   ├── test_il_sim_smoke.py          # lerobot 环境冒烟
│   ├── test_il_sim_teleop.py         # Robot/Teleop 协议契约
│   ├── test_il_sim_record.py         # 采集冒烟 → LeRobotDataset 结构校验
│   └── test_il_sim_train.py          # 3 策略冒烟 + checkpoint 可加载
└── outputs/datasets/il_sim/          # LeRobotDataset 输出（gitignore; il_real 用 il_real/）
```

**依赖**：本 venv 不新增（lerobot 用用户已装环境，Phase 0 冒烟锁定版本；冲突回退独立 venv 备选）。

## 实施阶段

### Phase 0：分支切换 + lerobot 环境冒烟（TDD）
- **Action**: `git checkout il_sim`（主目录开发，venv 已含 lerobot 0.6.1）；
  冒烟锁定环境：`import lerobot`、版本、
  `LeRobotDataset` API、`record`/`train` 可导入、四个 policy 包可导入；记录 numpy
  实际版本共存结论（用户实测 numpy<2 可用）
- **测试先行**: `tests/test_il_sim_smoke.py`——上述检查 + `SO101ReachCamera-v1` 可用性
- **Validate**: `uv run pytest tests/test_il_sim_smoke.py`

### Phase 1：SimSO101Robot（Robot 协议实现）
- **Action**: `src/so101_teleop/`（ament_python 包骨架 + package.xml/setup.py）+
  `so101_teleop/robot.py` + `configs.py`：实现 Robot ABC 全部方法；
  obs/features 键名对齐 lerobot 约定（`observation.state`、`observation.images.*`、
  `action`）；图像 uint8 HWC；`send_action` 一次调用 = 一个 10Hz 控制步；
  calibrate/is_connected 等 stub（仿真无需）
- **测试先行**: `test_il_teleop.py::test_robot_features`（键/形状）+
  `test_robot_obs_action_roundtrip`（10 步循环，图像 shape/dtype，state 连续变化）
- **Validate**: `uv run pytest tests/test_il_sim_teleop.py`

### Phase 2：Teleop 三实现 + record 接通
- **Action**:
  - `teleoperators/expert.py`：IK 状态机 Teleoperator（复用 `envs_run_expert`
    逻辑 + `so101_envs.ik`；reach 目标点 → 动作序列）
  - `teleoperators/keyboard.py`：关节轮询选择 + 增量控制（自写 Teleoperator）
  - `gamepad.py`/`leader.py`：TODO 占位类（注明 lerobot 自带 gamepad 可低成本接入）
  - draccus 注册：config 类装饰 `@RobotConfig.register_subclass("so101_sim")` /
    `@TeleoperatorConfig.register_subclass("so101_expert"/"so101_keyboard")`
  - `scripts/il_sim_record.py` 薄封装：import 触发注册 + 挂载 ros_viz 线程 +
    透传官方 `lerobot-record` CLI（`--robot.type=so101_sim
    --teleop.type=so101_expert --dataset.repo_id=... --dataset.root=outputs/datasets/il_sim`）
- **测试先行**: `test_il_sim_record.py::test_record_smoke`（小段 record → LeRobotDataset
  目录结构：meta/info.json、episodes、features 键、图像帧数）
- **Validate**: `uv run pytest tests/test_il_sim_record.py` + 手动采集一条 episode

### Phase 3：遥操 launch 验收 + RViz 同步显示 + 采集验收
- **Action**:
  - `ros_viz.py`（**内置启用**的线程 ROS 节点：/joint_states、
    /il/{fixed,wrist}/image、物体 TF——采集/遥操时 RViz 显示相机图片是硬需求）
  - `scripts/il_sim_teleop.py`：遥操主进程（参数 `--teleop expert|keyboard`、
    `--record`（默认 false：不录制数据）、`--env reach`；record=false 走
    lerobot 官方 `teleoperate()`，true 走 `il_sim_record` 的 record 管线；
    ros_viz 两种模式均内置启动）
  - `src/so101_teleop/launch/teleop.launch.py`：launch 验收入口——参数
    `teleop`（遥操类型）、`record`（默认 false）、`env`、`rviz`（true）；
    **必须启动环境进程（内含 env + ros_viz）与 RViz**
- **Validate**: launch 验收三连——
  ① `ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=false`
  （纯遥操，RViz 显示双相机图像，无数据落盘）
  ② `... teleop:=keyboard record:=false`（键盘驱动）
  ③ `... teleop:=expert record:=true`（录制 → outputs/datasets/il_sim 落盘）+
  `uv run pytest`（全套）

### Phase 4：il_sim_train.py（3 策略跑通）
- **Action**: `scripts/il_sim_train.py`：组 `TrainPipelineConfig`（dataset 指向
  `outputs/datasets/il_sim/<repo_id>`，policy 三选一 + **小网络模板**：
  ACT（dim_feedforward/隐层缩减）、Diffusion（降 vis/obs 维、n_obs_steps）、
  VQ-BeT（小 backbone））；`--steps 10` 冒烟 → checkpoint 可加载
- **测试先行**: `test_il_sim_train.py::test_act_smoke`（steps=10 → 输出目录 +
  checkpoint 文件）+ `test_policy_registry`（3 策略 config 均可构造）
- **验收**: 三策略各 steps=10 跑通（**SmolVLA = TODO 不实现**）
- **文档**: 迁移训练指引（用户在别的机器上换全量步数的命令）
- **Validate**: `uv run pytest tests/test_il_sim_train.py`

### Phase 5：il_infer ROS2 推理包
- **Action**: `src/so101_il_sim_infer/`（镜像 rl_infer）：加载 lerobot policy
  （`policy.predict_action`），自持 env（相机版），10Hz tick → ROS 话题
  （/joint_states、物体 TF、/il/{fixed,wrist}/image）；launch/rviz/wrapper 镜像
- **测试先行**: checkpoint 加载器单测（格式 + predict 形状）；headless 话题验证
- **Validate**: `ros2 launch so101_il_sim_infer il_sim_infer.launch.py env:=reach
  checkpoint:=<训练产物> camera:=true rviz:=true`（MuJoCo 与 RViz 同步可视化）
- **调参点**: policy 推理 chunk 语义（ACT chunk_size=50 → n_action_steps 执行节拍）

### Phase 6：文档 + 报告 + TDD 证据
- **Action**: `docs/il_sim/index.md` 更新（落地版：命令/数据格式/训练迁移指引）；
  `reports/il_sim_report.md`（采集验收数据/四策略冒烟结论/SmolVLA 可行性结论）；
  CLAUDE.md 增补；`.claude/tdd/il_sim.tdd.md`
- **Validate**: `uv run pytest` 全套 + colcon build

## Pattern Grounding（镜像现有模式）

| 类别 | 来源 | 模式 |
|---|---|---|
| 协议适配 | lerobot `robots/robot.py`、`teleoperators/teleoperator.py` | Robot/Teleoperator ABC 全方法实现 |
| ROS 包 | `src/so101_envs_viz/`、`src/so101_rl_infer/` | ament_python + data_files lib wrapper（setuptools≥80 规避）+ sys.path 自举 |
| record 复用 | `lerobot_record.py::record()` | 官方管线（不自写数据集写入） |
| 推理包 | `src/so101_rl_infer/` | checkpoint 加载器 + env 驱动 + ROS 话题 + wrapper |
| 脚本专家 | `scripts/envs_run_expert.py` | IK 状态机逻辑抽用 |
| CLI | `scripts/rl_train.py` | argparse + 输出目录约定 |
| run 管理 | `libs/so101_rl/runs.py` | 数据集输出不覆盖（repo_id 约定即可） |

## 依赖与兼容

- lerobot：**用户已装环境**（numpy<2 实测可用，Phase 0 冒烟锁定版本与 API；
  冲突回退独立 venv 备选）
- 本 venv 不新增依赖；so101_teleop 包的 package.xml 声明 rclpy/sensor_msgs/tf2_ros
  （ros_viz 需要），lerobot 由用户环境提供
- HF hub 访问：SmolVLA 需下载预训练权重（~1GB+），离线机器标记 TODO

## Risks

| 风险 | 可能性 | 缓解 |
|---|---|---|
| 用户 lerobot 版本与源码 HEAD API 不一致 | 中 | Phase 0 冒烟锁定实际可用 API，代码只依赖冒烟验证过的接口 |
| record 管线对 sim robot 的隐藏假设（calibrate/事件键） | 中 | Robot 方法 stub 最小实现；record 冒烟尽早暴露（Phase 2） |
| env step 频率与 record 循环节拍不匹配 | 中 | SimSO101Robot 内部对齐 10Hz（send_action 阻塞到下一控制节拍） |
| ~~SmolVLA~~ | — | 已降级 TODO 先不实现（hub 下载/显存评估为前置待办） |
| lerobot 训练对数据集 stats/图像格式的隐藏要求 | 中 | 用官方 record 写入（格式保证）+ Phase 4 冒烟尽早暴露 |
| obs 键名漂移 | 低 | features 键名集中在 so101_teleop 常量，双端引用 |
| il_sim/il_real 命名混淆 | 低 | 脚本/包/测试/数据集目录统一 il_sim 前缀；so101_teleop 为跨方向共享层 |

## Estimated Complexity: LARGE

- Phase 0: 1h；Phase 1: 2~3h；Phase 2: 3~4h（record 接通是关键路径）；
  Phase 3: 2h；Phase 4: 2~3h；Phase 5: 2~3h；Phase 6: 1h

## Acceptance

- [ ] `il_sim` 分支：SimSO101Robot + 三类 teleop（expert/keyboard/占位）通过协议契约测试
- [ ] `il_sim_record.py` 采集 reach：专家采集 ≥5 episodes，success ≥90%，LeRobotDataset
      结构校验通过（meta/features/图像帧）；采集时 RViz 显示双相机图像
- [ ] `teleop.launch.py` 验收：遥操类型可选（expert/keyboard）、record 默认
      false（不落盘）、launch 自动启动环境进程与 RViz；录制模式数据落
      outputs/datasets/il_sim
- [ ] `il_sim_train.py` 三策略（ACT/DP/VQ-BeT）`--steps 10` 冒烟跑通并产出
      可加载 checkpoint（SmolVLA = TODO，见 todo/）
- [ ] `so101_il_sim_infer` ROS2 推理：MuJoCo 与 RViz 同步可视化（策略驱动 reach）
- [ ] 迁移训练指引文档（用户在别的机器执行全量训练的命令）
- [ ] `uv run pytest` 全套 + colcon build 全绿；commit 由用户手动执行
- [ ] `reports/il_sim_report.md` + `.claude/tdd/il_sim.tdd.md` 证据留档
