# RL 指南（so101_envs × sb3/skrl）

<!-- 参考 stable-baselines3 guide 结构（quick_start/rl_tips/vec_envs/tensorboard/save_format）
     编写; 命令表带 AUTO-GENERATED 标记, 改 CLI 后请同步 -->

本文覆盖 SO101 RL 的完整动线：训练 → 评估 → 推理可视化 → 部署接口。
环境集见 `libs/so101_envs`（8 个 gymnasium id），训练库见 `libs/so101_rl`。

## Quick Start

```bash
# 1. 训练（sb3 SAC, 单环境, 进度条 + tensorboard 自动开启）
uv run python scripts/rl_train.py --backend sb3 --algo sac \
    --env SO101Reach-v1 --steps 200000

# 2. 看曲线 / tensorboard
uv run python -c "import webbrowser; ..."   # 或直接打开:
tensorboard --logdir outputs/rl/sb3-sac-reach-v1/latest/tb

# 3. 评估（--ckpt 支持 <tag>/latest 指针, 自动解析最新 run）
uv run python scripts/rl_eval.py --ckpt outputs/rl/sb3-sac-reach-v1/latest --episodes 10

# 4. 推理可视化（RViz）
ros2 launch so101_rl_infer rl_infer.launch.py env:=reach \
    checkpoint:=outputs/rl/sb3-sac-reach-v1/latest camera:=true rviz:=true
```

:::{note}
训练默认**不会覆盖**历史：每次 run 落在 `outputs/rl/<tag>/<时间戳>/`，
`<tag>/latest` 指针始终指向最新（`rl_eval`/`rl_infer` 都解析它）。
训练产物已 gitignore。
:::

## 训练

| 参数 | 默认 | 说明 |
|---|---|---|
| `--backend` | （必填） | `sb3` \| `skrl` |
| `--algo` | （必填） | `sac` \| `ppo` \| `td3`（skrl: `ppo`/`sac`） |
| `--env` | `SO101Reach-v1` | 任意注册 id（训练用 state 版） |
| `--steps` | `200000` | 总环境步数（多环境时按 env 计数） |
| `--n-envs` | `1` | 并行环境数（见下节） |
| `--eval-freq` | `10000` | eval success 评估间隔（同步写 TB） |
| `--out` | 自动 | 显式指定则原样使用；默认 `outputs/rl/<tag>/<时间戳>/` |
| `--normalize` | 关 | 启用 VecNormalize（状态随 checkpoint 存/载，**推理必须同载**） |
| `--no-tb` | — | 关闭 tensorboard |
| `--overrides` | — | 超参覆盖，如 `--overrides lr=0.001 batch_size=128` |

算法默认超参在 `libs/so101_rl/configs/<backend>_<algo>.yaml`，
优先级：代码内置 < yaml < `--overrides`。

## 多环境

```bash
uv run python scripts/rl_train.py --backend sb3 --algo ppo --n-envs 16 --steps 400000
```

- sb3 用 `DummyVecEnv`（官方文档：**通常快于 SubprocVecEnv**——mujoco 类轻量
  step 环境，进程 IPC 开销高于仿真本身）；skrl 用 `gym.make_vec(..., "sync")`。
- PPO 的 `n_steps` 会按 env 数自动缩减，保持每次更新的样本预算恒定。
- **内存红线：~170MB/env**（每 env 独立加载 MjModel 含 13 个 mesh）。
  15GB 机器桌面环境（GUI 占 4-5GB）建议 `--n-envs ≤ 32`；超过会触发 OOM
  killer，**症状是 wezterm/vscode 集体闪退而非训练报错**。
- 大规模并行（数百 envs / 视觉训练）需要 GPU 物理管线，路线勘察见
  `workflow/todo/rl_gpu_parallel_todo.md`（Warp/mjlab/ManiSkill3）。

## 评估与可视化

| 产物 | 位置 | 用法 |
|---|---|---|
| eval 曲线 | `<run>/curve.png` | 每次训练自动生成（success rate vs steps） |
| tensorboard | `<run>/tb/` | `tensorboard --logdir outputs/rl/<tag>/latest/tb`；含 `eval/success_rate` |
| checkpoint | `<run>/model.{zip\|pt}` + `meta.json`（backend/algo/env_id/obs_dim 校验） | `rl_eval` / `rl_infer` 自动识别后端 |

评估独立于训练：`rl_eval --ckpt <tag>/latest --episodes 10` 逐 episode 打印
success 并汇总。

## Tensorboard 指标

| 指标 | 含义 |
|---|---|
| `eval/success_rate` | 每 `--eval-freq` 步在独立 eval env 上跑 5 episodes 的成功率 |
| `rollout/ep_rew_mean` 等 | 后端原生训练指标（sb3/skrl 各自命名） |

## 推理（ROS2 + RViz）

```bash
ros2 launch so101_rl_infer rl_infer.launch.py env:=reach \
    checkpoint:=outputs/rl/<tag>/latest camera:=true rviz:=true
```

- 节点自持 env 实例，10Hz tick：obs → policy → env.step → 发布
  `/joint_states`、物体 TF、`/rl/{fixed,wrist}/image`（Reliable QoS）。
- `checkpoint` 支持 `<tag>/latest` 指针；`meta.json` 校验 env_id/obs_dim 防版本漂移。
- MuJoCo 窗口对照：`uv run python scripts/envs_view_env.py --id SO101Reach-v1`。

## Tips（算法选择与调试）

- **reach 类任务先 SAC**（采样效率高），PPO 步数预算 ×2.5 起步。
- 训练"没动静"先查三件事：进度条是否在动（sb3 函数式 callback 返回值）、
  TB 是否有事件（skrl 的 `experiment.directory` 必须在 agent 构造前设置）、
  内存是否打爆（多环境 OOM 杀 GUI 不报错）。
- lift/place/stack 环境可训练，但抓取保持有已知物理问题（见
  `workflow/reports/envs_report.md`），成功率预期低——reach 验收后再扩展。
- 长训前先跑 `--steps 2000 --n-envs 4` 冒烟，确认动线全通再上大预算。
- 部署真机：`libs/so101_rl/lerobot_deploy.py` 定义了协议与适配器
  （零 lerobot 依赖），真机接线 TODO 见类内文档。

## 部署接口（lerobot，零依赖）

`libs/so101_rl/lerobot_deploy.py` 定义 sim2real 协议层：

- `SO101DeployPolicy`：`reset()` / `act(obs)` 抽象协议——真机执行循环只依赖此接口。
- `GymPolicyAdapter`：把双后端 checkpoint 适配为协议（`from_checkpoint` 与 rl_infer 同入口）。
- `LerobotSO101Arm`：真机接线 TODO 骨架（不 import lerobot；硬件就绪后按类内
  6 步 TODO 实现：独立环境装 lerobot → SO101Follower → REST 位形 →
  `denormalize_action` 逐关节位置目标 → 真机 obs 构造 → 安全限幅）。

obs/action 归一化约定：action `[-1,1]` ↔ 真机 `lo + (a+1)/2·(hi-lo)`
（与 MJCF ctrlrange 一致，lerobot 舵机位置控制与仿真 `<position>` actuator 同构）。
