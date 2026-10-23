# TDD 证据报告: il_sim（plan/il_sim.md）

日期：2026-10-23 · 分支：il_sim · 测试运行器：`uv run pytest`（runner 探测：uv + pyproject
`[tool.pytest.ini_options]`；无 npm/pnpm——项目为 Python/ROS，Step 0 的 PM 探测脚本不适用）

## Source plan

`plan/il_sim.md`（untrusted 输入已sanitize：无破坏性操作/凭据/指令覆盖语句；验证命令
映射为 `uv run pytest ...`、`colcon build`、`ros2 launch ...` 白名单动作）。

## User journeys（来自 plan 需求重述）

1. 采集者用相机版 env 采集 lerobot 格式数据，采集时 RViz 显示相机图片。
2. 采集支持 expert/keyboard 遥操（gamepad/leader 为 TODO 占位，leader 直接用 lerobot 自带）。
3. 训练不用 ROS2，ACT/DP/VQ-BeT 三策略必须跑通（SmolVLA 降级 TODO）。
4. 推理用 ROS2，MuJoCo 与 RViz 同步可视化。

## Task → Test → RED/GREEN 映射

| # | Plan 任务 | 测试 | RED 证据 | GREEN 证据 |
|---|---|---|---|---|
| P0 | lerobot 环境冒烟 | test_il_sim_smoke.py (9) | 环境验证类测试，无生产代码 → 首跑即绿（不适用 RED gate） | 9 passed 7.5s |
| P1 | SimSO101Robot + 包骨架 | test_il_sim_teleop.py (8) | ModuleNotFoundError: so101_teleop（7 failed/err）；中途锚定后 `get_choice_names`→`get_known_choices`、`lerobot.constants`→`lerobot.utils.constants`、`IdentityProcessor`→`IdentityProcessorStep` 三次修正测试导入即绿 | 8 passed |
| P2 | Teleop 三实现 + record | test_il_sim_record.py (10) | ModuleNotFoundError（teleoperators 缺失）+ RecordConfig 导入路径修正（configs.parser→scripts.lerobot_record） | 10 passed 19s（含真实 record() 落盘） |
| P3 | cli_config + teleoperate 冒烟 | test_il_sim_teleop_run.py (4) | ModuleNotFoundError: cli_config | 4 passed |
| P4 | 三策略训练 config + ACT 冒烟 | test_il_sim_train.py (4) | ModuleNotFoundError: train_config；随后 ACTConfig 参数修正（crop_* 不存在于 0.6.1）、policy.push_to_hub=False | 4 passed 31s（ACT steps=10 真训练） |
| P5 | checkpoint 加载器 + predict | test_il_sim_infer.py (2) | ModuleNotFoundError: so101_il_sim_infer | 2 passed 30s（自含 record→train→load→predict） |
| P5b | ROS 推理节点 | 无 pytest（rclpy 不在 uv venv） | — | launch 无头验收：话题流 + 无崩溃（见下） |
| P5c | ros_viz / teleop launch | 无 pytest（同上） | — | 验收①②③ + rviz 进程验证 |

## 测试规格表

| # | What is guaranteed | Test | Type | Result | Evidence |
|---|---|---|---|---|---|
| 1 | lerobot 0.6.1 核心 API 与 4 policy 可导入 | test_il_sim_smoke.py | unit/smoke | PASS | `uv run pytest tests/test_il_sim_smoke.py` |
| 2 | SO101ReachCamera-v1 reset/step/图像 uint8 HWC | test_so101_reach_camera_env | integration | PASS | 同上 |
| 3 | so101_sim 注册进 draccus ChoiceRegistry | test_sim_config_registered | unit | PASS | `uv run pytest tests/test_il_sim_teleop.py` |
| 4 | features 键名派生 observation.state/action/images（官方推导管线） | test_derived_dataset_features | contract | PASS | 同上 |
| 5 | obs/action 往返：图像 uint8 HWC、state [-1,1] | test_robot_observation_roundtrip | contract | PASS | 同上 |
| 6 | 单相机 config 裁剪 features（VQ-BeT 前置） | test_single_camera_config | contract | PASS | 同上 |
| 7 | 专家闭环 ≤80 帧到达 reach 目标 | test_expert_drives_tcp_to_goal | integration | PASS | `uv run pytest tests/test_il_sim_record.py` |
| 8 | reset 握手：SETTLE 请求 → send_action 帧边界消费 | test_reset_handshake | integration | PASS | 同上 |
| 9 | 键盘 1-6/w/s 增量钳位 + c 请求重置 | test_keyboard_* | unit | PASS | 同上 |
| 10 | 官方 record() 冒烟 → LeRobotDataset 结构 | test_record_smoke | integration | PASS | 同上（19s，真实落盘） |
| 11 | record/teleop config 构造约定（root/fps/push_to_hub） | test_il_sim_teleop_run.py | unit | PASS | `uv run pytest tests/test_il_sim_teleop_run.py` |
| 12 | teleoperate 模式 1.5s 无异常 | test_teleoperate_smoke | integration | PASS | 同上 |
| 13 | 三策略小网络 config 可构造；SmolVLA=TODO | test_il_sim_train.py | unit | PASS | `uv run pytest tests/test_il_sim_train.py` |
| 14 | ACT steps=10 → checkpoint 可 from_pretrained | test_act_train_smoke | integration | PASS | 同上（31s，真训练） |
| 15 | checkpoint 加载 + select_action (6,) 值域 | test_il_sim_infer.py | integration | PASS | `uv run pytest tests/test_il_sim_infer.py` |
| 16 | launch ①expert 纯遥操话题流 | 无头验收 | E2E(手动) | PASS | /joint_states 11Hz, /il/fixed/image 10Hz, 0 错误 |
| 17 | launch ②keyboard 纯遥操 | 无头验收 | E2E(手动) | PASS | /joint_states 10Hz, 图像帧可达 |
| 18 | launch ③expert 录制落盘 | 无头验收 | E2E(手动) | PASS | outputs/datasets/il_sim/il_sim/teleop_launch_smoke 1ep/80帧 |
| 19 | rviz:=true 进程拉起 | 无头验收 | E2E(手动) | PASS | rviz2 进程存活 |
| 20 | IL 推理 launch 话题流 | 无头验收 | E2E(手动) | PASS | joint ~4.7Hz / image ~5.8Hz（diffusion），无崩溃 |

## 覆盖与已知缺口

- `uv run pytest`（ROS 环境下）：**257 passed / 0 failed**；il_sim 新增 37 项
  （smoke 9 + teleop 8 + record 10 + teleop_run 4 + train 4 + infer 2）
- **依赖坑（已修）**：gym-aloha（lerobot[all] 带入）压 mujoco<3.9 → envs 抓取物理测试失败 +
  mj_fullM 签名变化；pyproject override-dependencies 锁 mujoco>=3.9（3.16.0）后全绿
- **有意盲区**：ros_viz 与 il_sim_infer_node 的 rclpy 路径（uv venv 无 ROS；launch 无头验收替代）。
  训练冒烟用小网络 + video=False（ffmpeg 编码路径未覆盖，采集 CLI 手动验收用的 video=true
  语法见 docs；VQ-BeT 单相机数据集覆盖）。
- **待用户复核**：RViz 画面（双相机图像显示）与全量采集成功率（plan 验收：专家采集 ≥5
  episodes 成功率 ≥90% —— 冒烟数据集已多次确认专家可达 goal，全量 20 episodes 采集由
  用户在图形环境跑 `scripts/il_sim_record.py` 验收）全量训练在别的机器。

## Merge evidence

commit 由用户手动执行（CLAUDE.md 约定，plan Acceptance 同）。建议提交切分（对应
RED/GREEN checkpoint 语义）：
1. `feat: il_sim Phase 0-1 冒烟 + SimSO101Robot 契约`（tests/test_il_sim_{smoke,teleop}.py + src/so101_teleop 骨架）
2. `feat: il_sim Phase 2-3 teleop/record/launch`（teleoperators + cli_config + ros_viz + launch）
3. `feat: il_sim Phase 4-6 训练/推理/文档`（train_config + il_sim_infer + docs/reports/tdd）
4. `chore: pyproject lerobot 0.6.1 uv 直装`（pyproject/uv.lock/.python-version）
