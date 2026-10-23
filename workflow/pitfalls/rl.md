# rl（强化学习训练 / 推理）

| 症状 | 原因与处理 |
|---|---|
| RL 训练"没动静"（进度条不动、无报错） | ① sb3 函数式 callback 返回值必须 True（**None = 静默停止训练**）；② PPO 多环境 n_steps×n_envs 预算过大网络零更新（已自动缩减）；③ TB 无事件是 skrl 的 experiment.directory 未在 agent 构造前设置 |
| `model.load(...)` 后 `gym.make` 全部 NameNotFound | sb3/skrl load 反序列化清空 gymnasium registry——load_policy 内 import so101_envs + registry 快照恢复（已实现）；自定义脚本需自行 import |
| tensorboard 打开无数据 | skrl `experiment.directory` 必须在 agent 构造前设置（构造时固化）；`write_interval="auto"` 依赖 init 的 trainer_cfg.timesteps |
| rl_eval 报 `SO101Reach doesn't exist` | 同 registry 条 |
| 训练产物覆盖 | 已由时间戳分层 + latest 指针解决（`libs/so101_rl/runs.py`）；显式 `--out` 仍由用户自管 |
| torch/pinocchio 环境 | 已改 uv 直装（torch 2.11+cu130 find-links、pin 3.4.0）——旧的"torch 走系统 site-packages / numpy<2 / system-site-packages"说法**已过时** |
| lerobot 与 envs 的 mujoco 版本冲突 | gym-aloha（lerobot[all] 带入）把 mujoco 压到 <3.9 破坏 envs 抓取物理——pyproject 已 override-dependencies 锁 mujoco>=3.9，**勿删** |

完整深度复盘（含"重来一次"改进清单）：`workflow/learn/rl.md`。
