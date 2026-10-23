# RL GPU 并行路线勘察（TODO, 2026-10-22 讨论结论）

背景：用户问询 GPU 多环境（num_envs）方案。结论：**当前用 CPU 多环境已实现**（sb3
SubprocVecEnv / skrl SyncVectorEnv，`--n-envs`），GPU 物理化记为待办，触发条件见下。

## 选项全景（2026-10 时点）

| 路线 | 结论 |
|---|---|
| **MuJoCo Warp**（mujoco 3.3+ 内置 warp 后端） | **首选勘察对象**：API 与 CPU mujoco 同构，可直接吃我们的 MJCF；环境 Python 逻辑保留，只向量化物理 step；warp↔torch 零拷贝。风险：mesh 凸包夹爪 / frictionloss / armature 舵机参数在 warp 求解器的数值保真度需勘察 |
| mjlab | warp 之上的 manager-based MDP 框架（Isaac Lab 风格）——选它意味着 MDP 移植，比直接用 warp 多一层；gym wrapper 可挂现有 TrainerSpec |
| MJX（JAX） | **排除**：要求整个环境+训练栈 JAX 化（第三套栈，与 sb3/skrl(torch) 冲突）；mesh 接触是其性能弱区 |
| ManiSkill 3 | 现成替代：warp GPU 后端 + **SO100 官方资产**（squint 即此路线，任务源码可参考）。若做视觉 RL 训练值得评估（相机/DR 白送），代价是放弃自研 so101_envs |
| Isaac Lab / Genesis / Newton | 排除：脱离 mujoco 主线，模型/参数全套迁移 |

## 触发条件（满足其一再立项）

1. CPU 多环境实测吞吐（steps/s × n_envs）仍无法满足训练时长需求
2. 启动视觉版（rgb obs）RL 训练——渲染成为大头，GPU batch 渲染才有意义
3. 需要数千环境的大规模并行（大规模 PPO / 超参数搜索）

## 勘察清单（立项时第一步）

- [ ] mujoco-warp 对 `libs/so101_envs/assets/so101_env.xml` 的加载与逐步对照（CPU vs warp 的
      接触力/轨迹差异，阈值 1e-3 量级）
- [ ] warp 对 position actuator + frictionloss + armature 的支持确认
- [ ] numpy<2 约束与 warp 栈共存验证（可能需独立 venv）
- [ ] 训练库 `TrainerSpec` 增 `warp_backend`（agent.act 批量化 + eval 单 env 抽样）
