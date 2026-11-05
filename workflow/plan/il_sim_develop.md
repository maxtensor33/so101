# Plan: il_sim_develop — il_sim 策略实机部署（lerobot policy → 真机 SO101）

**日期**: 2026-11-04
**状态**: 待确认
**复杂度**: High
**沿革**: 由 sim_to_real 方向收窄更名（2026-11-04）。RL 实机部署、RL 视觉扩展、数字孪生物体同步、实机相机手眼标定拆为待办: `workflow/todo/rl_real_deploy_todo.md`、`workflow/todo/rl_image_based_todo.md`、`workflow/todo/sim2real_twin_todo.md`、`workflow/todo/setup_hand_eye_calibration_todo.md`。
**参考**:
- 官方路径: `/home/maxtensor/Third/Soft/lerobot` 的 `lerobot-rollout` CLI（`--strategy.type=base --policy.path=<pretrained_model> --robot.type=so101_follower --robot.cameras="{...}"`）
- 本仓库可复用底座: `src/so101_il_sim_infer/so101_il_sim_infer/core.py`（load_policy 三元组 + make_obs_batch 约定, ROS 无关）、`src/so101_hardware`（实机插件）、`src/so101_teleop`（lerobot Robot 协议适配先例）
- squint 部署调研结论已迁入 `workflow/todo/rl_real_deploy_todo.md`

## 需求重述

1. **il_sim 策略上真机**: 训练用仿真相机（fixed/wrist, 128×128, fps=10），部署用实机双相机（wrist + top；**top 相机映射训练 key `fixed`**——仿真 fixed 是世界系固定俯视位，语义即"顶部相机"），图像进策略 + 发布 ROS 话题供 RViz 可视化。
2. **冒烟先行**: 本地 `il_sim_train.py --steps 10` 产冒烟 ACT 模型先跑通流程；别的机器训练的全量模型回来后**换 checkpoint 即用**（同 launch/命令只换 path）。
3. **RL 不部署实机**: RL 定位为 IL 的动作专家（`il_sim_collect_rl.py` 仿真内采集），实机部署/视觉扩展/数字孪生均已拆为待办（见沿革）。

## 环境事实（已勘察确认）

| 项 | 事实 |
|---|---|
| lerobot-rollout 本地可用 | uv 环境 lerobot 0.6.1，`import lerobot.scripts.lerobot_rollout` 已验证 OK |
| il_sim 推理核心可复用 | `so101_il_sim_infer/core.py`: `load_policy(ckpt) → (policy, pre, post)` 设备自适应（cuda→cpu 回退）；`make_obs_batch`: state=归一化 qpos(6,), 图像 CHW float 0-1 —— ROS 无关，真机节点直接复用 |
| 训练数据约定 | il_sim_record fps=10、128×128 双相机、key `fixed`/`wrist`；ACT 吃双相机，VQ-BeT 单相机（`--camera-keys fixed`）；policy 输入特征 `observation.images.<key>` |
| 实机控制底座 | `demo.launch.py real:=true`（so101_hardware 插件 + `arm_controller`/`gripper_controller` JTC 100Hz + `joint_state_broadcaster`）；`on_activate` 从当前位置起步，无零位要求 |
| lerobot-rollout 的臂接入 | 用 lerobot 自带 `SO101Follower`（直读 LeRobot 标定 JSON），与我们 `so101_hardware` 的 `motor_calibration.yaml`（同一 JSON 转换产物）是**两套标定来源、一个共同上游** |
| 测试惯例 | `tests/test_setup_hardware.py` 契约测试风格（文件存在性/内容断言）；`uv run pytest` 回归 |

## 关键设计决策

- **D1 双轨: 官方冒烟 → 自研交付物**。Phase 1 用 `lerobot-rollout` 官方 CLI 零代码冒烟（验证标定/相机/推理/臂动物理链路）；Phase 2 起自研 `src/so101_il_sim_develop` ROS 包作为开发/验收交付物（镜像 `so101_il_sim_infer` 架构: 复用 core.load_policy + make_obs_batch 约定，env.step 换成真机执行）。官方 CLI 与自研包**并存不互斥**——rollout 适合快速验证，ROS 包适合 RViz/话题生态与安全机制。**命名约定**: 包名取 `il_sim_develop`（开发/流程验证定位），`il_real` 命名空间保留给后续正式实机部署包，避免混淆。
- **D2 真机执行路径（ROS 包内）**: ros2_control 底座（`real:=true` → so101_hardware 插件 + JTC），10Hz tick 发 `JointTrajectory` 单点目标（arm 5 关节 + gripper 分控制器下发）；若 10Hz 下 JTC 插值滞后明显，增补 `forward_position_controller` 直写（Phase 2 内定，pitfalls 记录实测）。
- **D3 obs 桥**: qpos 订阅 `/joint_states`（标定映射后的 rad）→ 按训练同约定归一化（复用 make_obs_batch 的归一化来源）；图像: usb_cam → 裁方/缩放到 128×128 → key 映射 **top→`fixed`、wrist→`wrist`**；VQ-BeT 单相机（camera_keys=fixed）时只喂 fixed。动作: `policy.select_action → post` → 限幅 → 下发。
- **D4 相机与 RViz**: usb_cam 起两路 opencv 相机发布 `/real/{fixed,top}/image_raw`，节点转发 `/il_sim_develop/{fixed,wrist}/image`（RViz 配置随包）。相机 index 显式 launch 参数（对标 lerobot-rollout 的相机字典写法）。
- **D5 安全**: estop 话题 `/il_sim_develop/estop`（任一消息 `std_msgs/Empty` 即停发并保持）+ Ctrl-C/episode reset 插值回 rest（bounded step, 对齐 squint 0.025 rad/step）+ **confirm_step 逐单步确认模式**（对齐 squint `--no-continuous_eval`, 首跑必开）+ per-step delta clamp。
- **D6 episode 语义**: Enter 重置 episode（复用 `reset_on_enter` 惯例）；真机无 reward/成功判定，成功率人工评估记录进报告。
- **D7 待办化**: RL 实机部署、RL 视觉扩展、数字孪生物体同步、实机相机手眼标定四个待办已落盘 `workflow/todo/`；**相机视觉先不对齐**（未标定），全量模型真机验收前先手眼标定。

## 项目结构（目标）

```
src/so101_il_sim_develop/                          # 新增（ament_python; 命名预留 il_real 给后续正式实机包）
├── package.xml / setup.py / resource/          # entry_point: il_sim_develop_node
├── launch/il_sim_develop.launch.py                # real:=true 底座 + usb_cam×2 + 节点 [rviz] [reset_on_enter] [confirm_step]
├── config/il_sim_develop.rviz                     # 双相机 Image + RobotModel + TF
└── so101_il_sim_develop/il_sim_develop_node.py       # obs 桥（真机版 make_obs_batch）+ load_policy 复用 + estop + 相机转发

tests/test_il_sim_develop.py                       # 新增: 契约/单测（TDD）
CLAUDE.md                                       # UPDATE: 命令区
workflow/tdd/il_sim_develop.tdd.md                 # 新增: TDD 证据
workflow/reports/il_sim_develop.md                 # 新增: 验收报告（真机项如实标注）
workflow/pitfalls/il_sim_develop.md                # 新增: 踩坑记录
```

## 实施阶段

### Phase 0：基线与冒烟模型生产
- `bash scripts/clean_ros.sh`；`colcon build --symlink-install` + `uv run pytest` 基线全绿。
- 本地产冒烟模型: `uv run python scripts/il_sim_train.py --policy act --repo-id il_sim/reach_expert --steps 10`。
- **验证**: 基线通过 + 冒烟 checkpoint（pretrained_model 目录）存在。

### Phase 1：lerobot-rollout 官方路径冒烟（零新代码）
- 命令（对标需求 1 参考命令）:
  `lerobot-rollout --strategy.type=base --policy.path=<冒烟 pretrained_model> --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=<标定名> --robot.cameras="{ fixed: {type: opencv, index_or_path: <top相机index>, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: <wrist相机index>, width: 640, height: 480, fps: 30}}" --task="grab the cube" --duration=30`
- 相机 key 必须与训练数据集一致（fixed/wrist）；640×480 采集、policy 预处理器自动 resize 到 128×128。
- **验证**: 相机读取 → 策略推理 → 臂运动闭环日志完成（冒烟模型动作无意义属预期）；问题记 `workflow/problems/`。
- 注: 0.6.1 robot type 命名（so101_follower vs so_follower）以实际报错为准，pitfalls 记录。

### Phase 2：so101_il_sim_develop 包 + obs/动作桥（TDD）
- 节点参数: `checkpoint/env/rate/camera_keys/cam_fixed_index/cam_wrist_index/estop_topic/confirm_step/reset_on_enter`。
- obs 桥: `/joint_states` → qpos(6) → 归一化 state；usb_cam 图像 → 128×128 → Dict obs → `pre → select_action → post` → 限幅 → JTC 下发。`load_policy` 从 `so101_il_sim_infer.core` 导入复用（sys.path 自举，同 il_sim_infer_node 惯例）。
- 测试: 契约测试（package/launch/参数名/estop/限幅）+ 纯函数单测（obs 桥数值断言），风格对齐 `tests/test_setup_hardware.py`。
- **验证**: `uv run pytest` + `colcon build` + mock 冒烟（手发 joint_states + 假图像）。

### Phase 3：孪生旁路冒烟（数字孪生旁路）
- `demo.launch.py real:=true sim:=mujoco` 下节点订阅 `/joint_states` 验证观测桥；实机未接时链路日志可验。
- **验证**: obs/动作/限幅/estop 链路日志正常，无 NaN。

### Phase 4：双相机接入 + RViz
- usb_cam 起实机 wrist/top 相机，节点转发话题，RViz 订阅显示双路图像。**注: 相机未标定，不做视野对齐**（手眼标定为待办，见 todo）；本阶段只验证图像通路。
- **验证**: `ros2 launch so101_il_sim_develop il_sim_develop.launch.py camera:=true rviz:=true`，RViz 双图像 ≥10Hz。

### Phase 5：真机验收（用户执行，checklist 写入报告）
- confirm_step 逐单步起步 → 连续 rollout；冒烟模型只验流程（动作无意义属预期）；全量模型（别的机器训练）回来后同命令换 checkpoint 重跑并记录成功率。
- **验证**: 报告如实记录；对照 `workflow/pitfalls/setup_hardware.md` H7 清单。

### Phase 6：文档工件
- CLAUDE.md 命令区 + tdd/reports/pitfalls 工件齐备；todo 三件套交叉引用一致。
- **验证**: `uv run pytest` 全绿 + 全量构建。

## 依赖关系

Phase 0 → 1 → 2 → 3 → 4 → 5 → 6 主线；Phase 1 仅依赖 0（零代码），可与 2-4 并行穿插。

## 风险

| 风险 | 概率 | 缓解 |
|---|---|---|
| 相机视野/安装与仿真 fixed/wrist 视角差异大 → 视觉策略迁移失败 | 高 | **已知且接受**: 相机尚未标定，视觉先不对齐——本期冒烟只验流程不验效果；全量模型真机验收前先完成手眼标定（`workflow/todo/setup_hand_eye_calibration_todo.md`，视觉对齐工程届时参考 squint tune_camera 思路） |
| usb_cam 30fps vs 训练 fps=10 的时序差 | 中 | 节点 10Hz tick 取最新帧（IL 策略 chunk 式输出对帧率不敏感）；pitfalls 记录实测延迟 |
| 归一化统计不一致（真机 qpos 范围 vs 训练数据统计） | 中 | 全行程标定保证真机 qpos 范围=仿真限位；归一化用 policy 自带统计（make_obs_batch 同约定）；首验对比 sim/real obs 分布 |
| JTC 100Hz 插值与 10Hz 决策耦合滞后 | 中 | 预留 forward controller 备选；pitfalls 记录实测 |
| lerobot-rollout 0.6.1 robot type 命名（so101_follower vs so_follower） | 低 | Phase 1 实测确认，pitfalls 记录 |
| 冒烟模型动作无意义导致"流程通但臂乱动"误判 | 中 | confirm_step 逐单步起步 + 首跑人手扶急停；验收只验流程不验效果 |
| 真机验收无法本会话完成 | 中 | mock/孪生旁路全流程替代验证，真机项写 checklist 由用户执行，报告如实标注 |

## 验收

- [ ] `colcon build --symlink-install` + `uv run pytest` 全绿（含新增 test_il_sim_develop.py）
- [ ] Phase 1 lerobot-rollout 冒烟完成（或问题转入 workflow/problems/）
- [ ] il_sim_develop_node mock/孪生旁路冒烟: obs→policy→动作→限幅→estop 链路日志正常
- [ ] 真机双相机进 RViz（≥10Hz）
- [ ] 冒烟模型真机 rollout 跑通流程（效果不作要求）
- [ ] 四个 todo 文件落盘（rl_real_deploy / rl_image_based / sim2real_twin / setup_hand_eye_calibration）
- [ ] CLAUDE.md 命令区更新 + tdd/reports/pitfalls 工件齐备
