# SO101 RL 报告（plan/rl.md Phase 7）

**日期**: 2026-10-22
**分支**: `rl`（commit 由用户手动执行）

## 交付物

| 组件 | 路径 |
|---|---|
| 训练库 | `libs/so101_rl/`（TrainerSpec 抽象 + sb3/skrl 双后端 + 5 算法注册表 + normalize 约定） |
| CLI | `scripts/rl_train.py`、`scripts/rl_eval.py`（<项目名>_<功能>.py 命名约定） |
| ROS2 推理 | `src/so101_rl_infer/`（rl_infer_node + launch + rviz，headless 话题验证通过） |
| lerobot 接口 | `libs/so101_rl/lerobot_deploy.py`（协议 + GymPolicyAdapter + TODO 骨架，零 lerobot 依赖） |
| gz TODO | `todo/rl_sim2sim_gz_todo.md` |
| 依赖 | sb3 2.9.0 + skrl 2.1.1（uv 安装）；torch 2.11.0+cu130 系统透传（override-dependencies 跳过安装） |
| 多环境 | `--n-envs`：sb3=SubprocVecEnv（真并行进程）+ skrl=SyncVectorEnv（子进程内含 env 注册）；GPU 物理化（Warp/mjlab/MJX/ManiSkill3）勘察结论见 todo/rl_gpu_parallel_todo.md |
| 训练可视化 | 每次训练自动出 `<ckpt>/curve.png`（eval success 曲线，libs/so101_rl/plotting.py） |

## 测试

`uv run pytest` 全套 **211 passed**（rl 新增 ~13 例：smoke 3 + 契约 2 + 注册表 smoke 5 + 身份 1 + normalize 2 + deploy 1）

## 训练记录（SO101Reach-v1）

| run | 后端/算法 | 步数 | eval success | 备注 |
|---|---|---|---|---|
| sb3-sac-reach | sb3/SAC | 2e5 | 0%（20 次评估） | 未收敛；策略 min-dist 5-11mm 显示学习发生，到达后振荡、静止条件未满足 |

> **训练交接**：流程搭建验收后训练由用户执行（2026-10-22 约定）。
> 加长训练命令：`uv run python scripts/rl_train.py --backend sb3 --algo sac --steps 400000
> --eval-freq 20000`；若仍不收敛的备选：reward shaping wrapper（到达后静止奖励，envs 库
> 不动）或放宽 `is_static` 阈值。skrl 侧：`--backend skrl --algo ppo --steps 500000`。

### 首训诊断（2e5, 0%）

- 载入 checkpoint 实测：`tcp_to_goal_dist` min 5-11mm（阈值 3cm），说明策略显著趋近目标
- 未成功的差距：到达后动作振荡漂移 + `is_static`（qvel<0.15rad/s）不同时满足
- 处置：训练交接给用户（步数加长 / reward shaping wrapper 两个备选方向已记录）

## 关键坑（详见 CLAUDE.md）

1. sb3 函数式 callback 返回值 = continue_training，返回 None 会**静默停止训练**
   （首训 eval history 空 + num_timesteps 冻结的根因，已修）
2. sb3 load 的 cloudpickle 数据会**清空 gymnasium registry**——自定义注册 env 必须
   load 前快照恢复（已修，`sb3_backend.load_policy`）
3. skrl 2.1 API：instantiator network 格式 `{"layers": [...], "activations": [...]}`、
   输入表达式 `concatenate(OBSERVATIONS, ACTIONS)`、`PPO_CFG`/`SAC_CFG` dataclass、
   `cfg.experiment.{write,checkpoint}_interval` 置 0 关自动保存
4. meta.json env_id 曾丢 "-v1"（sb3 从内部 env 读 spec），改为 make_model 时记录

## TDD 证据与复盘

- TDD 证据：`.claude/tdd/rl.tdd.md`
- 经验复盘（收获/坑/重来改进，glm-5.3-flash 撰写）：`learn/rl.md`
