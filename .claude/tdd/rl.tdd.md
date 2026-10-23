# TDD 证据报告: SO101 RL（plan/rl.md）

**来源计划**: `plan/rl.md`（用户经 `/ecc:tdd-workflow plan/rl.md` 确认执行）
**日期**: 2026-10-22
**分支**: `rl`（checkpoint commit 按用户约定不自动执行，由用户手动 commit）

## 用户旅程（源自计划）

1. 作为 RL 研究者，我想要 sb3/skrl 双后端统一接口的训练库，以便同一套代码切换
   算法与框架训练 so101 任务。
2. 作为部署工程师，我想要 ROS2 推理节点加载 checkpoint 并在 RViz 中验证策略行为。
3. 作为迁移工程师，我想要零依赖的 lerobot 部署协议层，以便真机就绪后直接接线。

## 任务 → 测试 → 证据映射

| # | 计划任务 | 测试目标 | RED 证据 | GREEN 证据 |
|---|---|---|---|---|
| 1 | Phase 0: 依赖+骨架 | `test_rl_smoke.py` 3 例 | —（基础设施验证，随依赖就绪转绿） | 3 passed |
| 2 | Phase 1: SB3 契约 | `test_sb3_contract` | ModuleNotFoundError sb3_backend | passed |
| 3 | Phase 1: 注册表 | `test_registry_knows_planned_algos` + `test_registry_rejects_unknown` | AssertionError 空 | passed |
| 4 | Phase 2: skrl 契约 | `test_skrl_contract` | ModuleNotFoundError skrl_backend | passed |
| 5 | Phase 3: 5 组合 smoke | `test_registry_smoke` ×5 | skrl:sac 假绿（PPO 冒名）→ `test_backend_algo_identity` RED → 实现 SAC 分支 | 10 passed |
| 6 | Phase 1: normalize 往返 | `test_vecnormalize_roundtrip` + `test_load_norm_missing_file` | —（契约锁定） | 2 passed |
| 7 | Phase 5: 部署协议 | `test_deploy_protocol` | ModuleNotFoundError lerobot_deploy | passed |
| 8 | Phase 4: 推理节点 | headless 话题验证（/joint_states 策略驱动值 + /rl/fixed/image 128px） | —（集成） | 通过 |
| 9 | Phase 1: reach 训练 | sb3-SAC 2e5 | — | 0%（未收敛，诊断+4e5 重训中，见 rl_report.md） |

## 测试规格（人类可读保证）

| # | 保证内容 | 测试 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | torch/sb3/skrl 版本下限 + cuda 可用 | `tests/test_rl_smoke.py` | 冒烟 | PASS |
| 2 | sb3 env_checker 通过 SO101Reach-v1（gymnasium 1.x 兼容） | `test_sb3_checker_accepts_our_env` | 兼容 | PASS |
| 3 | skrl wrapper 可包我们的 env（torch tensor 动作约定） | `test_skrl_wraps_our_env` | 兼容 | PASS |
| 4 | SB3 契约: make/train/save/load/predict 全链路形状与目录约定 | `test_sb3_contract` | 契约 | PASS |
| 5 | skrl 契约: 同上双后端一致 | `test_skrl_contract` | 契约 | PASS |
| 6 | 5 个 (backend,algo) 组合注册表 smoke | `test_registry_smoke` | 契约 | PASS |
| 7 | skrl:sac 是真 SAC（非 PPO 冒名）| `test_backend_algo_identity` | 单元 | PASS |
| 8 | VecNormalize 存/载往返一致 + 缺文件回退 | `tests/test_rl_normalize.py` | 单元 | PASS |
| 9 | GymPolicyAdapter 满足部署协议（零 lerobot 依赖） | `test_deploy_protocol` | 协议 | PASS |
| 10 | rl_infer 节点发布策略驱动的 /joint_states 与相机图像 | headless ROS 验证 | 集成 | PASS |
| 11 | 多环境（n_envs=2）双后端契约（sb3 SubprocVecEnv / skrl SyncVectorEnv） | `test_multi_env_contract` | 契约 | PASS |
| 12 | 训练可视化: eval 曲线 png 生成 | `test_eval_curve_png` | 单元 | PASS |

## 覆盖率与已知缺口

- 全套 `uv run pytest` → **214 passed**
- 已知缺口（有意）：**训练收敛由用户执行**（流程搭建完成为 rl 交付边界，2026-10-22 约定；
  SAC 2e5 首训未收敛的诊断与备选方向见 reports/rl_report.md）；相机版 obs 训练未做
  （计划允许，state 版先行）；gz 验证为 TODO 文档
- 校验命令（实际执行）：
  - `uv run pytest` → 211 passed
  - `uv run python scripts/rl_train.py --backend sb3 --algo sac --steps 200000` → checkpoint + eval history（0%）
  - `colcon build --symlink-install --packages-select so101_rl_infer` → 通过
  - `ros2 launch so101_rl_infer rl_infer.launch.py ... camera:=true` → 话题验证通过

## Merge 证据

- commit 由用户手动执行；本报告与 `reports/rl_report.md` 保存 RED→GREEN 映射。
- 输出损坏事件：`lerobot_deploy.py` 首次 Write 出现乱码碎片且落盘损坏，经 py_compile
  与测试发现，删除后分短块重写，最终 211 passed 验证。
