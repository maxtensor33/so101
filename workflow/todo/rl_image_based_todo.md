# Todo: rl_image_based — RL 视觉（rgb+state）训练扩展

> 状态: 待办（2026-11-04 由 sim_to_real 方向拆出）。触发条件: RL 需要真机部署且 state-only + 孪生路线效果不达标时启动；RL 仅作 IL 动作专家（`il_sim_collect_rl.py` 仿真内采集）时**无需**此项。

## 背景

squint 原版是 image-based（rgb+state 混合观测），真机部署靠 rgb 通道兜底（详见 [[rl_real_deploy_todo]] 调研节）。本地复刻是 state-only（obs 52 维），真机部署必须有 item_pose 真值来源。rgb+state 版可与 squint 原路线完全对齐，部署时对物体位姿真值的依赖大幅降低。

## 现状（2026-11-04 勘察）

**envs 层已复刻**:
- 8 个 `-Camera` 变体注册（`libs/so101_envs/__init__.py:33-48`，4 任务 × cube/can）
- `camera=True` 时 obs = `Dict{"state": (52,), "rgb": {"fixed": 128×128×3, "wrist": 128×128×3}}`（`base_env.py:100-104`；`RENDER_SIZE=128` 与 squint `render_size=128` 一致）
- MuJoCo EGL 渲染 + fixed 相机位姿 DR 噪声（`_randomize_fixed_cam`）
- **相机喂法待决策**: squint 策略只吃 wrist 单相机，envs 给 fixed+wrist 双相机——训练时需选定（建议先对齐 squint 只用 wrist）

**训练层零支持**（用 `-Camera` id 训练会直接崩）:

| 缺口 | 现状证据 |
|---|---|
| sb3 策略类硬编码 `MlpPolicy` | `libs/so101_rl/backends/sb3_backend.py:109` — Dict obs 需 `MultiInputPolicy` + 自定义 CNN features_extractor |
| skrl 后端无视觉 encoder | `skrl_backend.py` grep 无 camera/Dict/encoder |
| meta.json 无 obs 类型记录 | 只有标量 `obs_dim`，推理侧无法重建观测约定 |
| `rl_train.py` 无 camera 开关 | `scripts/rl_train.py:59-62` env_kwargs 只传 action_mode/domain_randomization（env_id 透传可绕，但死在后端） |
| 部署层无 rgb 预处理 | `libs/so101_rl/lerobot_deploy.py` 假定 state ndarray；squint 部署侧有裁方+resize+16×16 下采样（`train_squint.py:488-499`） |
| squint 双流网络无对应物 | CNNEncoder(16×16) + rgb_proj/state_proj 拼接（`train_squint.py:264-275`） |

## 实施要点（届时展开为 plan）

1. `so101_rl` 视觉扩展: sb3 `MultiInputPolicy` + CNN features extractor（或照抄 squint encoder 结构）/ skrl encoder；图像下采样策略（squint: 渲染 128 → 训练下采样 16×16）。
2. meta.json 记 obs 类型（`state` / `rgb+state`）与相机 key 喂法。
3. 全链路 rgb: rl_train / rl_eval / rl_infer_node / lerobot_deploy 的 obs 构造与预处理。
4. 真机侧需配套视觉对齐工程（squint: 黑幕背景/相机对齐工具/同色物体），参考其 `deploy_utils/tune_camera.py`。
