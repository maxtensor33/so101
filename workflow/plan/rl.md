# Plan: SO101 RL 训练与推理（sb3/skrl 双后端 + ROS2 推理 + lerobot 部署接口）

**日期**: 2026-10-22
**状态**: 待确认
**复杂度**: Large（约 10~14 小时）
**前置**: plan/envs.md 已完成并合并 main（`libs/so101_envs` 8 个 gymnasium id，reach 专家 100%）

## 需求重述

1. 在本项目新增 so101 **RL 项目**，技术栈 ros2 + mujoco + rviz + lerobot。
2. **训练不使用 ROS2**：纯 gymnasium（`libs/so101_envs`），支持 **sb3 与 skrl 两种包**
   及多种 RL 算法（SAC / PPO / TD3）。
3. **推理使用 ROS2**：新 ROS2 节点加载训练好的 checkpoint 驱动 env，支持 **MuJoCo
   与 RViz 可视化**（joint_states/TF/相机图像）。
4. **lerobot = sim2real 部署接口**（已确认）：本项目内只留适配层接口与文档，
   不安装 lerobot 依赖，真机验收待硬件就绪。
5. **sim2sim 的 gz 验证：仅 TODO 文档占位**，不实现（已确认）。
6. **训练范围（已确认）**：reach 任务先行贯通 sb3 + skrl 双后端各 1 个算法
   （sb3-SAC + skrl-PPO），验证封装层后再扩展算法矩阵与其余任务。
7. **scripts 命名约定**：`<项目名>_<功能>.py`（项目 = plan/ 下的项目名，如
   `rl_train.py`、`robotics_draw_cube_viewer.py`），以项目作区分；Phase 0 一并回改存量。
8. 创建 git 分支 **`rl`** 开发；commit 由用户手动执行（沿 envs 约定）。

## 环境事实（已勘察确认）

| 项 | 状态 |
|---|---|
| 环境库 | `libs/so101_envs`：state obs 25/29/33/35 维 + 相机版 Dict obs；动作 Box(-1,1,6)；10Hz 控制/500Hz 仿真；state env 1429 steps/s（训练可行性已验证） |
| reach 可解性 | 脚本专家 100% success（5/5），奖励含 1−tanh(5d) 密集项，RL 可学 |
| lift/place/stack | 抓取保持已知问题（reports/envs_report.md），RL 可训练但成功率预期低——rl 先以 reach 验收 |
| venv 约束 | `--system-site-packages` + `numpy<2`；torch 2.x 兼容 numpy 1.26 ✓；sb3≥2.4 支持 gymnasium 1.x ✓；skrl≥1.4 支持 gymnasium ✓ |
| ROS 包模式 | `src/so101_envs_viz` 先例：ament_python + setup.py data_files 装 lib/<pkg> wrapper（setuptools≥80）+ OpaqueFunction launch + rviz 配置 |
| GPU | 机器有 NVIDIA GPU（MUJOCO_GL=egl 渲染正常）；torch 用 cu12x wheel |
| 注册 id | SO101Reach-v1 等 8 个（无相机/Camera）；训练用 state 版（相机版训练留扩展） |

## 架构决策（已与用户确认）

| 决策 | 结论 |
|---|---|
| lerobot 角色 | sim2real 部署接口层（接口 + 文档，不装依赖，不验收真机） |
| 训练范围 | reach 先行：sb3-SAC + skrl-PPO 贯通 → 再扩 sb3-PPO/TD3 + skrl-SAC |
| 推理架构 | 独立 `src/so101_rl_infer` 包（rl_infer_node 自持 env + checkpoint），与 viz_node 职责分离 |

## 目录结构（目标）

```
so101/
├── libs/so101_rl/                    # ★ 纯 Python 训练库（无 ROS 依赖，uv wheel）
│   ├── __init__.py
│   ├── backends/
│   │   ├── base.py                   # TrainerSpec 抽象: make_env/make_model/train/save/load/predict
│   │   ├── sb3_backend.py            # SB3: DummyVecEnv+Monitor, .zip checkpoint, VecNormalize 状态随存
│   │   └── skrl_backend.py           # skrl: gymnasium env 直入 trainer, torch .pt checkpoint
│   ├── algos.py                      # 算法注册表: {"sb3:sac": ..., "skrl:ppo": ...} + 默认超参
│   ├── normalize.py                  # VecNormalize 开关约定（默认关; 开则状态必须随 checkpoint 存/载）
│   ├── lerobot_deploy.py             # sim2real 接口层: SO101Policy 协议 + lerobot 接线 TODO 文档
│   └── configs/                      # 每算法 yaml 超参（可被 CLI 覆盖）
│       ├── sb3_sac.yaml / sb3_ppo.yaml / sb3_td3.yaml
│       └── skrl_ppo.yaml / skrl_sac.yaml
├── scripts/                          # 命名约定: <项目名>_<功能>.py
│   ├── rl_train.py                   # CLI: --backend sb3|skrl --algo sac|ppo|td3 --env SO101Reach-v1
│   │                                 #       --steps 2e5 --out outputs/rl/<tag>/
│   └── rl_eval.py                    # 加载 checkpoint 跑 N episode 出 success rate
├── src/so101_rl_infer/               # ★ ROS2 ament_python 推理包
│   ├── so101_rl_infer/rl_infer_node.py
│   │   # 参数: env / checkpoint / camera / rate(默认10Hz)
│   │   # 每 tick: obs → policy.predict → env.step → 发布 /joint_states、物体 TF、
│   │   #       /rl/{fixed,wrist}/image（camera=true 时）; QoS Reliable（RViz 兼容）
│   ├── launch/rl_infer.launch.py     # OpaqueFunction: rsp + rl_infer + rviz
│   ├── rviz/rl_infer.rviz
│   └── wrapper/rl_infer_node         # setuptools>=80 lib wrapper（镜像 envs_viz）
├── todo/rl_sim2sim_gz_todo.md        # gz 验证 runbook 占位（不实现; 待办归 todo/ 目录）
├── tests/
│   ├── test_rl_backends.py           # 双后端契约测试: make/train(短)/save/load/predict 形状与一致性
│   ├── test_rl_normalize.py          # VecNormalize 状态存/载往返一致性
│   └── test_rl_smoke.py              # 依赖版本/导入冒烟（torch cuda 可用性打印）
└── reports/rl_report.md              # 训练曲线/success rate/推理演示记录
```

**依赖变更**：`pyproject.toml` 增 `stable-baselines3>=2.4`、`skrl>=1.4`、
`torch>=2.3`（cu12x wheel）、`pyyaml`（已有）；hatch wheel packages 增 `libs/so101_rl`。
**不装 lerobot**（接口层零依赖）。

## 实施阶段

### Phase 0：分支 + 依赖 + 骨架（TDD 冒烟）
- **Action**: `git checkout -b rl`；`uv add torch stable-baselines3 skrl`（torch 用
  pytorch cu12x index，注意 uv 配置 `[tool.uv.sources]`）；创建 libs/so101_rl 骨架 +
  backends 抽象；pyproject wheel packages 更新
- **测试先行**: `test_rl_smoke.py`——import torch/sb3/skrl、打印 cuda 可用性、
  `so101_envs` 与 sb3 的 gymnasium 版本兼容检查（sb3 用我们的 env 跑 1 step）
- **存量脚本改名**（git mv 保留历史 + 引用同步: CLAUDE.md / docs/ / reports/ / plan/ /
  脚本内自述）：

  | 现名 | 新名 | 归属项目 |
  |---|---|---|
  | `draw_cube_viewer.py` | `robotics_draw_cube_viewer.py` | robotics |
  | `verify_model_alignment.py` | `setup_verify_model_alignment.py` | setup |
  | `check_env.py` | `envs_check_env.py` | envs |
  | `view_env.py` | `envs_view_env.py` | envs |
  | `run_expert.py` | `envs_run_expert.py` | envs |
  | `clean_ros.sh` | 保持（跨项目通用工具） | — |

- **Validate**: `uv run pytest tests/test_rl_smoke.py` + `grep -rn "旧脚本名" 排查引用清零`

### Phase 1：SB3 后端 + reach 训练贯通（TDD）
- **Action**:
  - `backends/base.py` TrainerSpec 抽象（make_env(seed)/make_model/train/save/load/predict）
  - `backends/sb3_backend.py`：SAC 实现（MlpPolicy，默认超参 yaml），Monitor +
    EvalCallback（每 N steps 跑 eval env 出 success rate）+ CheckpointCallback
  - `normalize.py`：VecNormalize 可选开关；开则 `vecnormalize.pkl` 随 checkpoint 存/载
    （**推理侧必须同载**——checkpoint 目录约定：`model.zip` + `meta.json` + `vecnormalize.pkl`）
  - `scripts/rl_train.py` / `scripts/rl_eval.py`
- **测试先行**: `test_rl_backends.py::test_sb3_contract`（make→train 200 steps→save→
  load→predict 形状 (6,) 且在动作空间内）
- **验收**: `uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Reach-v1
  --steps 2e5` → eval success rate ≥80%（目标 90%+，SAC 对 reach 类任务强）
- **Validate**: `uv run pytest tests/test_rl_backends.py tests/test_rl_normalize.py`

### Phase 2：skrl 后端 + 贯通（TDD）
- **Action**: `backends/skrl_backend.py`：PPO 实现（gymnasium env 直入 skrl trainer，
  device 自动 cuda/cpu，checkpoint torch .pt + normalize 状态同目录约定）；
  algos.py 注册表统一 `--backend/--algo` 分发
- **测试先行**: `test_rl_backends.py::test_skrl_contract`（同 SB3 契约，断言双后端接口形状一致）
- **验收**: `--backend skrl --algo ppo --steps 5e5`（PPO 采样效率低于 SAC，预算加大）
  → eval success rate ≥80%；两条训练曲线入 reports/rl_report.md
- **Validate**: 同 Phase 1 命令 + `scripts/rl_eval.py` 双后端各跑 10 episodes

### Phase 3：算法矩阵扩展 + 超参配置
- **Action**: 补 sb3-PPO / sb3-TD3 / skrl-SAC（全部走注册表，每算法一个 yaml）；
  超参 yaml 字段可被 CLI `--overrides key=val` 覆盖
- **测试先行**: 注册表契约测试（每个注册项 make_model 构造 + 1 步 train 不炸）
- **验收**: 5 个 (backend, algo) 组合各 200 steps smoke 通过 + 记录到报告
  （长训只跑 reach 的 sb3-SAC 与 skrl-PPO，其余组合命令留文档）
- **Validate**: `uv run pytest tests/test_rl_backends.py`

### Phase 4：ROS2 推理包（rl_infer）
- **Action**:
  - `rl_infer_node.py`：加载 checkpoint 目录（按 backend 自动识别 .zip/.pt + normalize
    状态），自持 env（不经 human 渲染），10Hz tick：predict→step→发布；
    `/joint_states`、物体 TF（复用 viz_node 的 TASK_BODIES 表）、`/rl/{fixed,wrist}/image`
    （Reliable QoS）；启动日志打 checkpoint 路径/backend/algo/最近 eval success
  - launch（OpaqueFunction: rsp + rl_infer + rviz）+ rviz 配置（镜像 envs_viz）
  - wrapper + package.xml + setup.py data_files（镜像 envs_viz 的 setuptools 规避）
- **测试先行**: 节点逻辑单测（checkpoint 加载器：双后端格式识别 + predict 形状）；
  ROS 侧 headless 话题验证（/joint_states 变化符合策略输出）
- **Validate**: `bash scripts/clean_ros.sh && ros2 launch so101_rl_infer rl_infer.launch.py
  env:=reach checkpoint:=outputs/rl/<tag> camera:=true rviz:=true`
  （RViz 中机械臂执行训练策略完成 reach；MuJoCo viewer 可同开 `scripts/envs_view_env.py` 对照）
- **调参点**: 推理 rate 与 env 控制频率一致（10Hz）；obs 顺序与训练一致（同一 env 类保证）

### Phase 5：lerobot 部署接口层（零依赖）
- **Action**: `libs/so101_rl/lerobot_deploy.py`：
  - `SO101DeployPolicy` 协议（`reset()` / `act(obs) -> np.ndarray(6)`）
  - `GymPolicyAdapter`：把双后端 checkpoint 包成该协议（推理节点即用此类）
  - `LerobotSO101Arm` TODO 骨架：注释文档写明接线点（lerobot Robot 接口 /
    so101_follower 配置 / obs 对齐约定：真机 qpos + 目标位形），**不 import lerobot**
  - 文档：obs/action 归一化约定（[-1,1] 动作 ↔ 真机关节限位映射）
- **测试先行**: `test_rl_backends.py::test_deploy_protocol`（GymPolicyAdapter 满足协议、
  act 输出形状/范围）
- **Validate**: pytest + 文档评审（真机验收待硬件）

### Phase 6：sim2sim gz 验证 TODO（仅文档）
- **Action**: `docs/rl_sim2sim_gz_todo.md`：gz 验证 runbook 占位——待办清单
  （MJCF→URDF/SDF 差异、gz_ros2_control 链路、策略对 obs 的敏感性评估），
  标记 TODO 不实现
- **Validate**: 文档评审

### Phase 7：文档 + 报告 + TDD 证据
- **Action**: `reports/rl_report.md`（训练曲线、eval success rate、推理演示记录、
  超参与耗时基准）；CLAUDE.md 增补 rl 常用命令与约定；TDD 证据 `.claude/tdd/rl.tdd.md`
- **Validate**: `uv run pytest`（全套）+ `colcon build --symlink-install` 通过

## Pattern Grounding（镜像现有模式）

| 类别 | 来源 | 模式 |
|---|---|---|
| 训练库 | `libs/so101_envs/` | 纯 Python 无 ROS 库 + uv wheel + hatch packages 注册 |
| ROS 包 | `src/so101_envs_viz/` | ament_python + data_files lib wrapper + OpaqueFunction launch + Reliable QoS |
| 环境使用 | `scripts/check_env.py` | `import so101_envs` 注册副作用 + gym.make |
| checkpoint 约定 | （新增） | 目录制：`model.{zip\|pt}` + `meta.json`(backend/algo/env/obs_dim) + `vecnormalize.pkl` |
| 测试 | `tests/` + pytest | 契约测试 + 冒烟；短训练 smoke 不超 10s |
| 脚本命名 | `scripts/`（本计划约定） | `<项目名>_<功能>.py`：rl_train / envs_check_env / robotics_draw_cube_viewer |

## 依赖与兼容

- 新增 pip：`torch>=2.3`（cu12x）、`stable-baselines3>=2.4`、`skrl>=1.4`
- venv 约束不变：`--system-site-packages` + `numpy<2`（torch 2.x 兼容）
- sb3 2.4+ 才支持 gymnasium 1.x——smoke 测试锁定版本组合
- 推理节点零新增 ROS 依赖（rclpy/sensor_msgs/tf2_ros 同 envs_viz）
- 不装 lerobot；gz 相关全部 TODO

## Risks

| 风险 | 可能性 | 缓解 |
|---|---|---|
| torch 安装体积/索引配置（uv + cu12x） | 中 | `[tool.uv.sources]` 指 pytorch-cu12 index；失败回退 CPU wheel（训练慢但可行） |
| sb3/skrl 与 gymnasium 1.x 兼容性差异 | 中 | Phase 0 smoke 锁版本；sb3>=2.4、skrl>=1.4 下限写死 |
| VecNormalize 状态漏载导致推理策略失效 | 高 | normalize.py 目录约定 + 契约测试强制往返一致；默认关闭 normalize |
| PPO 在 reach 上收敛慢（相对 SAC） | 中 | 步数预算 5e5 + EvalCallback；验收线 80%，失败则记录曲线并调超参 |
| 训练曲线与评估的 seed 方差 | 中 | EvalCallback 固定 eval seed；报告记录多 seed 均值（时间允许） |
| rl_infer 与训练 env 的 obs 不一致（版本漂移） | 低 | checkpoint meta.json 存 env_id + obs_dim，加载时校验 |
| 双后端 checkpoint 格式不统一 | 低 | TrainerSpec 抽象 + meta.json 分发；契约测试锁定接口形状 |
| gz 验证缺失（TODO） | — | 已确认为有意延期，文档记录路径 |

## Estimated Complexity: LARGE

- Phase 0: 1h；Phase 1: 3~4h（含 SAC 训练等待）；Phase 2: 3h；Phase 3: 2h；
  Phase 4: 2~3h；Phase 5: 1h；Phase 6: 0.5h；Phase 7: 1h

## Acceptance

- [ ] `rl` 分支上 sb3-SAC 与 skrl-PPO 在 reach 训练收敛，eval success ≥80%
- [ ] 双后端契约测试通过（make/train/save/load/predict 接口形状一致）
- [ ] VecNormalize 状态存/载往返一致（若启用）
- [ ] `ros2 launch so101_rl_infer ...` RViz 中加载 checkpoint 执行 reach 演示
- [ ] lerobot 部署接口协议测试通过（零 lerobot 依赖）
- [ ] gz 验证 TODO 文档就位（不实现）
- [ ] scripts 命名符合 `<项目名>_<功能>.py` 约定，旧引用（CLAUDE.md/docs/reports/plan）已同步
- [ ] `uv run pytest` 全套 + `colcon build` 全绿；commit 由用户手动执行
- [ ] `reports/rl_report.md` + `.claude/tdd/rl.tdd.md` 记录证据
