# Plan: SO101 Image-based DAgger（state RL expert → vision student 蒸馏 + 推理/实机部署）

**日期**: 2026-11-07
**状态**: 已实施（证据: workflow/tdd/rl_dagger.tdd.md；执行方式见 D11——主会话分组派发子代理，组间验收）
**复杂度**: Large（约 12~16 小时）
**前置**: plan/rl.md 已完成（state-based RL expert checkpoint 可用，`libs/so101_rl` TrainerSpec 双后端 + `scripts/rl_train.py/rl_eval.py` 贯通）；plan/envs.md 已完成（`Camera-v1` 相机版 env 8 个）
**参考实现**: `/home/maxtensor/Master/EAI/project/github/s2v-dagger`（AAAI 2025, arXiv:2412.13662 官方实现，CleanRL 风格，Adroit 域）

## 需求重述

1. 已有一个 **state-based RL expert**（现有 rl checkpoints，sb3/skrl 任一后端），现需要一个 **image-based 的 student 策略**，方法参考 s2v-dagger：**两阶段 DAgger**——student 用图像执行 rollout，expert 用 state 事后批量标注，BC 损失蒸馏，数据环形混合。
2. 新增训练/评估脚本：`scripts/rl_dagger_train.py`、`scripts/rl_dagger_eval.py`（沿 rl 项目 `<项目名>_<功能>.py` 命名约定）。
3. 新增两个 ROS2 包：
   - `src/so101_rl_dagger_infer` —— 仿真推理节点（镜像 `src/so101_rl_infer`：加载 student checkpoint 自持相机版 env，发布 joint_states/TF/相机图像）
   - `src/so101_rl_dagger_develop` —— 实机部署开发包（镜像 `src/so101_il_sim_develop`：obs 桥 + estop + homing + confirm_step + 内置 cv2 相机）
4. 训练不经 ROS；ROS 只做推理/可视化/实机部署（沿项目横向约定）。
5. 开发流程：TDD（tests/ 证据 → workflow/tdd/），先出本计划确认后再动手。

## 适配性评估（s2v-dagger → 本项目）

**结论：可以应用**，且本项目结构天然匹配：

| s2v-dagger 需求 | 本项目现状 | 差距 |
|---|---|---|
| expert 输入 state 一维向量，输出 action∈[-1,1] | 现有 rl checkpoint 正是此契约（`load_policy` 返回 `obs->action(6)∈[-1,1]`，自动处理 vecnormalize） | 无 |
| student 观测 Dict{rgb, state}，同一结构下 expert 走 state 通道、student 走 rgb 通道 | `Camera-v1` env 的 obs 恰为 `Dict{"state": ..., "rgb": {"fixed", "wrist"}}`（uint8 HWC） | 无 |
| expert 在线批量标注（student 执行、expert 事后给 label） | expert 是黑盒可调用 `obs->action`，与后端无关 | 无 |
| 自写 BC 训练循环 + DAggerBuffer（torch） | `libs/so101_rl` 目前只有 sb3/skrl，无 image 通路、无自写循环 | **需新增** dagger 模块 |
| 并行 env 采集 | skrl 后端已有 `gym.make_vec(vectorization_mode="sync")` 惯例 | 无 |
| eval（成功率统计） | env info["success"] + `SuccessEvalCallback` 先例 | 无 |
| 部署/推理代码 | s2v-dagger 没有 → 本项目自有模板（rl_infer / il_sim_develop） | 按本项目模板新建 |

关键借鉴点（来自参考实现探查）：
- **bc_loss 阈值早停** + 每 collect 固定步数（参考实现控制训练时长的核心 trick）
- **buffer 按 collect 轮组织**、环形覆盖、均匀采样（新旧数据混合，经典 DAgger agg 模式）
- expert 动态加载按 checkpoint 目录 + meta.json 重建（本项目 `resolve_ckpt_dir→read_meta→get_trainer→load_policy` 链更稳，直接沿用）

## 环境事实（已勘察确认）

| 项 | 状态 |
|---|---|
| expert 契约 | `libs/so101_rl/backends/base.py:45-65`：`load_policy -> 可调用 obs->action(6)∈[-1,1]`；meta.json 固定字段含 env_kwargs（action_mode/domain_randomization） |
| expert checkpoint | `outputs/rl/<tag>/<时间戳>/`：model.{zip\|pt} + meta.json (+vecnormalize.pkl)；`resolve_ckpt_dir` 解 latest |
| 相机版 env | `SO101{Task}{Object}Camera-v1` 8 个，obs Dict{state, rgb{fixed,wrist}}，图像 uint8 0-255 HWC；`RENDER_SIZE=(128,128)` 默认，可 `render_size=` 覆写；相机标定默认开（calib 只覆写 pos/quat/fovy，与分辨率解耦） |
| obs state 维度 | reach 47 / lift 52 / place 57 / stack 57（含任务段），expert 用 |
| RL 库 | `libs/so101_rl` 无任何 image/Dict obs 支持；VecNormalize 只适用 flat obs |
| 并行 env 惯例 | sb3 后端 DummyVecEnv（OOM 教训）；skrl 后端 `gym.make_vec(..., vectorization_mode="sync")` |
| infer 包模板 | `src/so101_rl_infer`：`_find_repo_root()` sys.path 自举 + declare_parameter + 10Hz tick + pynput/stdin Enter 重置；话题 `joint_states`/物体 TF/`rl/{cam}/image` |
| develop 包模板 | `src/so101_il_sim_develop`：`real_infer.py`（normalize_qpos/denormalize_action/clamp_target/plan_to_rest/EstopLatch）+ 节点状态机 run\|homing\|confirm_wait + `camera_node.py`（cv2 相机→`/real/{key}/image_raw`）+ `config/cameras.yaml` |
| 包安装惯例 | ament_python setup.py data_files 装 `lib/<pkg>` wrapper（setuptools≥80 console_scripts 落 bin/ 而 ros2 run 需要 wrapper） |
| 测试约定 | `tests/test_<项目>_<主题>.py`；TDD 证据 workflow/tdd/；`bash scripts/test.sh` 统一入口 |
| torch | uv 直装（pyproject find-links/清华源），cu12x wheel；MUJOCO_GL=egl 可用 |

## 架构决策（随计划确认生效）

| # | 决策 | 结论 |
|---|---|---|
| D1 | 方法 | 照搬 s2v-dagger 两阶段：**不重训 expert**，直接加载现有 rl checkpoint 当 teacher；student = torch CNN + BC，DAgger 循环 student 执行 / expert 批量标注 |
| D2 | student 观测 | **只用 rgb，单相机 fixed 起步**（`--camera-keys fixed` 可扩 wrist/双相机）；state 通道仅训练时供 expert 标注 → 实机部署纯视觉，无需 qvel/tcp 观测 |
| D3 | 代码位置 | `libs/so101_rl` 新增 `dagger.py`（student 网络 `StudentPolicy` + `DAggerBuffer` + `train_dagger` 循环 + `load_student_policy`）；**不走 TrainerSpec sb3/skrl**（BC 不属这两后端），meta.json `backend: "dagger"`，checkpoint = `model.pt + meta.json` |
| D4 | 采集 env | `gym.make_vec(..., vectorization_mode="sync")`（skrl 惯例，单进程稳）；`--n-envs` 默认 4；`--render-size` 默认 480x640（**2026-11-07 用户确认与 env 原生 RENDER_SIZE 一致**，覆盖原 128x128 方案；相机标定保持默认开启；buffer_size 默认随之下调至 10000 ≈ 8.8GB 单相机）；**2026-11-07（下午）用户因机器性能回退 128x128**，buffer_size 恢复 100000（≈4.7GB 单相机） |
| D5 | expert 加载 | 训练脚本 `--expert-ckpt` 走现有 `resolve_ckpt_dir→read_meta→get_trainer→load_policy` 链（vecnormalize/sb3 registry 快照自动处理）；meta env 与 `--env` 不一致时以 `--env` 为准并 warn |
| D6 | 超参组织 | `libs/so101_rl/configs/dagger.yaml` 默认超参（lr=3e-4, minibatch=256, collect_size, bc_loss_th 等）+ CLI `--overrides key=val`（沿 algos.py 优先级惯例简化版） |
| D7 | eval | 独立脚本 `scripts/rl_dagger_eval.py --ckpt --episodes`（镜像 rl_eval.py），驱动相机版 env 统计 info["success"] 成功率 |
| D8 | infer 包 | `src/so101_rl_dagger_infer` 镜像 `so101_rl_infer`（自持 Camera env + student policy），节点 `rl_dagger_infer_node.py`，话题前缀 `rl_dagger/` |
| D9 | develop 包 | `src/so101_rl_dagger_develop` 镜像 `il_sim_develop`：`real_infer.py` 核心（图像 resize/crop 到 student 输入分辨率 + 动作 clamp/homing/EstopLatch）+ `dagger_camera_node.py`（cv2 相机）+ `config/cameras.yaml`；core 复用 `so101_rl_dagger_infer` 的 load 函数 |
| D10 | 分支 | 在已有 **`rl` 分支**上开发（先 merge main，2026-11-07 已 fast-forward 到 aef6fca）；commit 由用户手动执行（沿 rl 约定） |
| D11 | 执行方式 | 主会话按组派发子代理顺序实施：A(Phase1+2 算法本体) → B(Phase3 脚本) → C(Phase4 infer 包) → D(Phase5 develop 包) → E(Phase6 收尾，主会话)；每组之间主会话验收；TDD 契约内嵌任务书（RED 门禁 + 禁 commit + test.sh 汇总回流），不走 tdd-workflow skill 本体 |

## 目录结构（目标）

```
so101/
├── libs/so101_rl/
│   ├── dagger.py                      # ★ StudentPolicy(CNN encoder + MLP head, 确定性 tanh 输出)
│   │                                  #   DAggerBuffer(按 collect 轮组织, 环形覆盖, uint8 存 rgb)
│   │                                  #   train_dagger(expert, vec_env, ...) 含 bc_loss 早停
│   │                                  #   load_student_policy(ckpt_dir) 读 meta 重建网络
│   └── configs/
│       └── dagger.yaml                # 默认超参
├── scripts/
│   ├── rl_dagger_train.py             # ★ --expert-ckpt --env --camera-keys --steps --n-envs
│   │                                  #   --render-size --seed --out --overrides ...
│   └── rl_dagger_eval.py              # ★ --ckpt --episodes（成功率统计）
├── src/so101_rl_dagger_infer/         # ★ ament_python，镜像 so101_rl_infer
│   ├── setup.py / package.xml
│   ├── so101_rl_dagger_infer/
│   │   ├── dagger_infer_node.py       # 自持 Camera env + student policy, 10Hz tick
│   │   └── core.py                    # ROS 无关: load_student + make_obs_batch(Dict obs)
│   └── launch/rl_dagger_infer.launch.py
├── src/so101_rl_dagger_develop/       # ★ ament_python，镜像 il_sim_develop
│   ├── setup.py / package.xml
│   ├── so101_rl_dagger_develop/
│   │   ├── dagger_develop_node.py     # 状态机 run|homing|confirm_wait + estop
│   │   ├── real_infer.py              # ROS 无关: 图像预处理/动作 clamp/homing/EstopLatch
│   │   └── dagger_camera_node.py      # cv2 相机 → /real/{key}/image_raw
│   ├── config/cameras.yaml (+cameras.local.yaml)
│   └── launch/rl_dagger_develop.launch.py
└── tests/
    ├── test_rl_dagger.py              # 网络/缓冲区/标注契约 + 微型 smoke 训练
    ├── test_rl_dagger_scripts.py      # CLI/落盘格式/短训 smoke
    ├── test_rl_dagger_infer.py        # core 无 ROS + launch 结构断言
    └── test_rl_dagger_develop.py      # real_infer 核心 + 节点/launch 结构断言
```

## 实施阶段

### Phase 1：student 网络 + DAggerBuffer（TDD）
- **Action**: `dagger.py` 实现 `StudentPolicy`（PlainConv 风格 encoder：5 组 Conv3x3+MaxPool → Linear 特征 → 确定性 tanh head；输入 N 相机 × HxWx3）与 `DAggerBuffer`（按 collect 轮 append/环形覆盖、均匀采样、rgb uint8 CPU 存储）
- **Mirror**: 结构参考 s2v `PlainConv/Agent`，代码风格沿 `libs/so101_rl` 现有模块
- **Validate**: `tests/test_rl_dagger.py` —— 输出 shape=(6,)∈[-1,1]、buffer 覆盖/采样契约、双相机输入可配
- **Validate cmd**: `bash scripts/test.sh tests/test_rl_dagger.py`

### Phase 2：DAgger 训练循环 + expert 标注链路（TDD）
- **Action**: `train_dagger()`——每轮 student 确定性 rollout collect_size 步（不用 reward）→ 观测展平 → expert 批量标注 `expert(obs["state"])` → 入 buffer → BC mse 更新 ×N（bc_loss < 阈值早停转下一轮）；`load_student_policy` + meta 落盘（`backend: "dagger"`，env_kwargs 随存）
- **Mirror**: expert 加载链 `resolve_ckpt_dir→read_meta→get_trainer→load_policy`（sb3_backend.py:130-166）
- **Validate**: 假 expert（固定映射脚本）+ 微型 reach Camera env 短训，loss 下降 + checkpoint 可回读推理
- **Validate cmd**: `bash scripts/test.sh tests/test_rl_dagger.py`

### Phase 3：训练/评估脚本（TDD）
- **Action**: `scripts/rl_dagger_train.py`（argparse 风格镜像 rl_train.py，含 `parse_known_args` 忽略 `--ros-args`、结尾 `os._exit(0)` 惯例）+ `scripts/rl_dagger_eval.py`
- **Validate**: 短训 smoke 落盘 model.pt+meta.json → rl_dagger_eval 可加载并统计成功率
- **Validate cmd**: `bash scripts/test.sh tests/test_rl_dagger_scripts.py`

### Phase 4：`so101_rl_dagger_infer` 包（TDD）
- **Action**: core.py（load_student + Dict obs → 网络输入 batch）+ 节点（自持 Camera env、`--env --checkpoint --rate --mujoco` 参数、joint_states/TF/`rl_dagger/{cam}/image` 话题、Enter 重置）+ launch + setup.py lib wrapper
- **Mirror**: `src/so101_rl_infer/so101_rl_infer/rl_infer_node.py` 全套
- **Validate**: core 无 ROS 导入测试 + launch ET 结构断言
- **Validate cmd**: `bash scripts/test.sh tests/test_rl_dagger_infer.py`

### Phase 5：`so101_rl_dagger_develop` 包（TDD）
- **Action**: real_infer.py（图像 resize/crop 到 student 输入分辨率、clamp_target、plan_to_rest、EstopLatch）+ 节点状态机（run|homing|confirm_wait，动作发 `/arm_controller` + `/gripper_controller` JointTrajectory）+ cv2 相机节点 + cameras.yaml + launch（include bringup real:=true）
- **Mirror**: `src/so101_il_sim_develop`（core 复用方式：sys.path 插入 src/so101_rl_dagger_infer）
- **Validate**: real_infer 无 ROS 测试 + 节点/launch/cameras.yaml 结构断言
- **Validate cmd**: `bash scripts/test.sh tests/test_rl_dagger_develop.py`

### Phase 6：收尾
- **Action**: `bash scripts/test.sh` 全量 + docs/commands.md 新增 rl_dagger 节 + workflow/tdd/rl_dagger.tdd.md 证据
- **Validate cmd**: `bash scripts/test.sh`（汇总行回流）

## Pattern Grounding（镜像现有模式）

| Category | Source | Pattern |
|---|---|---|
| 训练脚本 CLI | `scripts/rl_train.py:26-43` | argparse + sys.path 自举 + env_kwargs 落盘 + os._exit(0) |
| expert 加载 | `libs/so101_rl/backends/sb3_backend.py:130-166` | registry 快照 + load_policy 黑盒可调用 |
| checkpoint 元数据 | `libs/so101_rl/backends/base.py:18-38` | write_meta 固定字段 + env_kwargs 随存 |
| 并行 env | `libs/so101_rl/backends/skrl_backend.py:79-87` | gym.make_vec sync 向量化 |
| infer 包 | `src/so101_rl_infer/so101_rl_infer/rl_infer_node.py` | sys.path 自举 + declare_parameter + 10Hz tick + Enter 重置 + lib wrapper 安装 |
| develop 包 | `src/so101_il_sim_develop/` | real_infer 核心 + 状态机 + estop + cv2 相机 + cameras.yaml |
| 测试 | `tests/test_rl_backends.py` | MUJOCO_GL=egl 置顶 + 契约复用函数 + 短训 smoke + try/finally close |
| DAgger 算法本体 | s2v_dagger_adroit.py:470-517, 282-316（参考仓库 scripts 目录，非本 repo 文件） | student 执行/expert 标注/bc_loss 早停/按轮 buffer |

## 依赖与兼容

- 不新增第三方依赖（torch/gymnasium/mujoco/cv2 均已在环境）；不改 `libs/so101_rl` 现有 TrainerSpec/sb3/skrl 代码（dagger.py 为新增模块）
- 现有 rl checkpoints 直接复用，零迁移成本
- `so101_rl_dagger_develop` 复用 `so101_rl_dagger_infer.core`（沿 il_sim_develop→il_sim_infer 复用惯例）；`il_real` 命名空间仍留给正式实机包

## Risks

| Risk | 等级 | 缓解 |
|---|---|---|
| render_size=128x128 与相机标定文件兼容性未验证 | 中 | **已消除**：apply_camera_calib 只覆写 cam_pos/cam_quat/cam_fovy，与分辨率解耦，已确认（128x128 为 1:1 纵横比，水平视野比 4:3 窄，但几何一致）；标定默认开启 |
| 图像渲染慢：sync 向量化 + 多 env 相机渲染，吞吐低于参考实现 16 env | 中 | 默认 n-envs=4 + render_size 128x128；训练时长不设硬指标，以 bc_loss 收敛为准 |
| buffer 内存：双相机 640x480 uint8 会爆内存 | 低 | 默认单相机 + 128x128（约 6.3MB/千步），容量 CLI 可调 |
| expert（如 vecnormalize）与 dagger 训练进程内多 env 并存是否正常 | 低 | load_policy 已有测试覆盖；Phase 2 smoke 即验证 |
| sim2real 图像域差（student 纯视觉更敏感） | 中 | 训练默认开启 env 已有 domain_randomization；实机验收沿 il_sim_develop 分阶段（confirm_step）流程 |
| 学生成功率上限受 expert 上限约束 | 信息 | DAgger 仅在 expert 覆盖的状态分布内修正；预期 reach 先验收，lift/stack 沿 rl.md 的已知限制 |

## Estimated Complexity: LARGE
- Phase 1-2（算法本体 + 测试）：4~6h
- Phase 3（脚本）：1~2h
- Phase 4-5（两个 ROS 包）：4~6h
- Phase 6（收尾）：1~2h

## Acceptance

- [x] `tests/test_rl_dagger*.py` 全绿（53 例），`bash scripts/test.sh` 全量 547 passed, 1 skipped
- [x] `scripts/rl_dagger_train.py` 端到端验证产出合法 checkpoint（model.pt + meta.json，backend=dagger；微型 sb3 expert bc_loss 0.083→0.040）
- [x] `scripts/rl_dagger_eval.py` 加载 student 并输出成功率统计
- [ ] `ros2 launch so101_rl_dagger_infer rl_dagger_infer.launch.py` 仿真推理可见（结构测试已过；真实 launch 目检待用户执行）
- [x] `so101_rl_dagger_develop` 结构与 il_sim_develop 对齐（estop/homing/confirm_step/cv2 相机），实机验收沿既有分阶段流程
- [x] workflow/tdd/rl_dagger.tdd.md 证据落盘；docs/commands.md 更新
