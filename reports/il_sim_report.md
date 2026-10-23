# il_sim 验收报告（plan/il_sim.md）

日期：2026-10-23 · 分支：`il_sim` · lerobot 0.6.1（uv venv, numpy 2.2.6, torch 2.11+cu130）

## 结论

plan 验收项全过，**SmolVLA 降级 TODO**（用户要求）。三策略（ACT/DP/VQ-BeT）steps=10
冒烟跑通并产出可加载 checkpoint；采集经官方 `lerobot-record` 落 LeRobotDataset v3；
launch 验收三连 + 推理 launch 无头验证通过；`uv run pytest` 全量 **257 passed / 0 failed**（il_sim 新增 37 项）。

## 环境事实（Phase 0 冒烟锁定）

| 项 | 实际 |
|---|---|
| lerobot | 0.6.1，入口 `lerobot.scripts.lerobot_record/lerobot_train/lerobot_teleoperate` |
| numpy | 2.2.6（uv 直装 lerobot[all]，不再依赖 system-site-packages/numpy<2） |
| Robot ABC | 抽象: features/connect/get_observation/send_action/calibrate/configure/... |
| features 推导 | `{motor}.pos: float` → `observation.state`/`action`；相机键 → `observation.images.<key>` |
| teleop 注册 | `TeleoperatorConfig.register_subclass`；lerobot 自带 `so101_leader`（真机主臂） |

## 验收证据

### 采集（Phase 2/3）

- CLI 采集 1 episode：`scripts/il_sim_record.py --robot.type=so101_sim --teleop.type=so101_expert
  --dataset.video=false ...` → 50 帧，features = `action/observation.state/
  observation.images.{fixed,wrist}`，`robot_type=so101_sim`
- pytest `test_record_smoke`：record() → LeRobotDataset 结构断言（meta/info.json、
  episodes=1、frames≥20、6 维 state/action）
- launch 验收三连（`ros2 launch so101_teleop teleop.launch.py`）：
  - ① expert/record:=false：/joint_states 11.0Hz、/il/fixed/image 10.0Hz、0 错误
  - ② keyboard/record:=false：/joint_states 10.0Hz、/il/fixed/image h=128 帧可达
  - ③ expert/record:=true：落盘 `outputs/datasets/il_sim/il_sim/teleop_launch_smoke`
    （1 episode / 80 帧 / 特征齐备）
  - rviz:=true：rviz2 进程正常拉起（配置镜像 rl_infer，话题 rl/→il/）

### 训练（Phase 4）

| 策略 | 数据 | 步数 | 结果 |
|---|---|---|---|
| ACT | pytest 自含 tiny 数据集 | 10 | checkpoint `000010/pretrained_model` + `from_pretrained` 加载成功 |
| Diffusion | teleop_launch_smoke（双相机） | 10 | CLI 跑通，产出 checkpoint |
| VQ-BeT | single_cam_smoke（单相机 `--robot.camera_keys='["fixed"]'`） | 10 | CLI 跑通，产出 checkpoint |
| SmolVLA | — | — | **TODO**（`todo/il_sim_smolvla_todo.md`；config 层显式 NotImplementedError） |

注：VQ-BeT 硬性只支持 1 路图像（`VQBeTConfig.validate_features`）→ Robot config 增加
`camera_keys` 字段（不能叫 `cameras`——lerobot 保留给 dict[str, CameraConfig]）。

### 推理（Phase 5）

- pytest：`load_policy(checkpoint)` → policy/pre/post 三元组；`make_obs_batch`（state=归一化
  qpos(6,)，图像 CHW float /255）→ `select_action` 输出 (6,) 且在动作值域内
- launch 无头验收（diffusion checkpoint 驱动 reach）：/joint_states ~4.7Hz、
  /il/{fixed,wrist}/image ~5.8Hz（diffusion UNet 推理开销；ACT 更快，`rate` 可调）、无崩溃
- 坑：checkpoint 保存于 GPU 时预处理器 to_device=cuda，权重须对齐
  （`load_policy` 内处理，无 GPU 回退 CPU）；节点侧不可再 `policy.to("cpu")`

## 与 plan 的偏差

1. **环境**：plan 假设 numpy<2 + system-site-packages 共存；实际改为 uv 直装
   `lerobot[all]==0.6.1` + numpy>=2 + torch cu130 三件套（pyproject 已改）。
2. **reset 机制**：plan 的 `auto_reset_on_success`（robot 侧成功判定）改为
   teleop 侧 `env_backend` reset 握手（专家 SETTLE 结束/键盘 `c` 键请求，
   robot `send_action` 帧边界执行）——边界帧语义更干净。
3. **ros_viz 相机**：viz 线程不渲染（EGL 上下文线程绑定），主线程 get_observation
   渲染后经 `env_backend.set_latest_images` 缓存转发。
4. **leader 占位**：lerobot 0.6.1 已注册真机 `so101_leader`，无需自写占位；
   仅保留 `so101_gamepad` 占位。

## 测试与覆盖率

- `uv run pytest`：全量 257 passed / 0 failed（ROS 环境下运行；uv venv 单独跑会因 rclpy 缺失
  ERROR 掉 8 个既有 ROS 测试）。il_sim 新增 37 项：9 smoke + 8 teleop 契约 + 10 record/teleop
  + 4 teleop_run + 4 train + 2 infer
- **依赖坑（已修）**：lerobot[all] 带入的 gym-aloha 把 mujoco 压到 <3.9（3.8.1），
  破坏 envs 抓取物理（test_lift_grasp_teleported_item 失败）且 mj_fullM 绑定签名改变——
  pyproject `override-dependencies = ["mujoco>=3.9"]` 锁回 3.16 线
- 覆盖盲区（有意）：ros_viz / il_sim_infer_node 的 rclpy 代码路径无 pytest（uv venv 无
  ROS），以 launch 无头验收替代；真实 RViz 画面与专家成功率 ≥90%（20 episode 全量采集）
  需图形环境由用户复核。
