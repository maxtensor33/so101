# RL 项目复盘：收获与坑（plan/rl.md 实施）

> 作者：**glm-5.3-flash**（Claude Code 会话内运行）· 2026-10-22
> 背景：从零搭建 so101 RL 训练/推理流程（sb3/skrl 双后端 + ROS2 推理 + 多环境），
> 全程 TDD（220 passed），经历三轮用户实测反馈（终端闪退 ×2、进度条缺失）。

## 一、最大的收获

### 1. 对照官方源码是最高效的修复路径
多环境方案反复踩坑（spawn OOM、SyncVectorEnv batched spaces、进度条不动），
最后靠用户指路 `RL_Lib`（sb3/skrl 官方仓库）对照官方示例一次性收敛：
- sb3 官方文档明说 **DummyVecEnv 通常快于 SubprocVecEnv**——mujoco 类轻量 step
  环境，进程 IPC 开销高于仿真本身，我们自作聪明上的多进程方案全是负优化；
- skrl 官方示例用 `gym.make_vec(env_id, num_envs=N, vectorization_mode="sync")`，
  且 wrapper 在 vectorized 时**自动把 observation_space 暴露为 single space**——
  我们手动 `single_*_space` 的修复方向对，但绕了远路。
**教训：先跑官方最小示例，再写自己的封装。**

### 2. TrainerSpec 抽象的回报
双后端统一接口（make_env/make_model/train/save/load_policy）+ meta.json
（backend/algo/env_id/obs_dim）+ 契约测试，使得：
- 换算法/后端只动注册表一行；
- rl_eval 与 rl_infer 完全不感知后端；
- 契约测试在 skrl:sac "PPO 冒名"假绿时抓住了算法身份错误。
**跨框架封装必须配契约测试，否则"能跑"不等于"跑对了"。**

### 3. 用户实测反馈暴露的都是真问题
三轮用户反馈（闪退 ×2、进度条、输出目录）全部指向我在无 GUI headless 环境
里看不到的盲区：内存红线、终端依赖、真实使用动线。

## 二、踩过的坑（按主题）

### sb3（stable-baselines3 2.9）
| 坑 | 现象 | 规则 |
|---|---|---|
| **函数式 callback 返回值 = continue_training** | 返回 None → 训练第一步后静默停止（num_timesteps 冻结，eval history 空），无任何报错 | 函数式 callback 每条路径必须显式 `return True` |
| **load 的 cloudpickle 清空 gymnasium registry** | 自定义注册的 env 在 `SAC.load` 后 `gym.make` 全部 NameNotFound | load 前快照 `dict(registry)`，finally 里 clear+update 恢复 |
| **从内部 env 读 spec.id 丢版本号** | meta.json 里 env_id 变成 `SO101Reach`（无 -v1），推理侧加载崩 | env_id 由 make_env/make_model 显式记录，不从 model 内部 env 反查 |
| PPO `n_steps` 语义 | n_steps 是**每环境**步数；64 envs × 2048 = 131k 样本才更新一次，小步数训练网络零更新 | 多环境时按 env 数缩减 n_steps，保持每次更新样本预算恒定 |

### skrl（2.1）
| 坑 | 说明 |
|---|---|
| `experiment.directory` 在 `agent.__init__` 时固化 | 构造后再改无效（TB 写到默认 ./runs） |
| `init(trainer_cfg=...)` 只收 dataclass | 传 dict 会 `dataclasses.asdict` 报错 |
| `write_interval="auto"` 依赖 trainer_cfg.timesteps | 空则算出 0 → 不建 TB writer |
| instantiator network 格式 | `{"layers": [...], "activations": [...]}`（不是 sequence/units）；多输入用 `"concatenate(OBSERVATIONS, ACTIONS)"` 表达式；输入名用 `OBSERVATIONS`（不是 STATES） |
| `PPO.act` 恒采样 | 确定性推理取 `outputs["mean_actions"]`（SAC 则直接取 actions） |
| `PPO_CFG/SAC_CFG` dataclass | 无 PPO_DEFAULT_CONFIG 字典；自动 checkpoint 用 `cfg.experiment.checkpoint_interval = 0` 关 |
| RandomMemory 即通用 memory | PPO `replacement=False` / SAC `replacement=True`（replay） |
| wrapper vectorized 语义 | `wrapped.observation_space` 自动返回 single space；obs 仍以 batch 形状进出 |

### 多环境与系统级
| 坑 | 现象 | 结论 |
|---|---|---|
| **spawn 64 进程 OOM** | 15GB 机器：spawn 无 COW 内存共享，64 × 全量 mujoco 环境 ≈ 爆内存，**OOM killer 连坐杀掉 wezterm/vscode**（内存大户 GUI 进程） | 用户终端"集体闪退"= 先查 OOM（`free -g`、dmesg），别只盯代码 |
| DummyVecEnv 每 env ~170MB | MjModel 独立加载（13 个 mesh STL），64 envs 单进程也要 ~11GB | 内存估算先行：`--n-envs ≤ (可用内存 × 0.6) / 170MB` |
| headless 测不出的问题 | GUI 会话内存基线（~4-5GB）比 headless 高，用户环境必崩而测试环境幸存 | 发布给用户的功能要按用户环境的内存基线验证 |

### 工程杂项
- **长文本写入易损坏**：本会话多次出现生成内容夹带乱码碎片并落盘损坏（scene_lift.xml、
  lerobot_deploy.py 等），靠 `py_compile`/XML 解析/测试及时检出，改为短块写入+逐块验证。
- colcon + setuptools≥80：console_scripts 装到 `bin/`，ros2 run 需要 data_files 装
  `lib/<pkg>/` wrapper（见 src/so101_envs_viz、so101_rl_infer 的 setup.py）。
- EGL 渲染有 ±1 位级噪声：涉及渲染的确定性断言要留容差。
- uv 管理下 torch 走系统安装：`[tool.uv] override-dependencies = ["torch; sys_platform == 'never'"]`
  marker hack，避免 2.5GB 重复下载（venv `--system-site-packages` 透传）。

## 三、如果重来一次

1. **第一件事：跑通两个官方示例（sb3 zoo + skrl examples/gymnasium）**，
   记录官方的多环境/进度条/TB 用法，再开始封装。多环境的 spawn/fork/DummyVecEnv
   折腾 + OOM 事故本可完全避免（官方文档一句话的事）。
2. **内存预算前置**：设计多环境时就按"每 env 内存 × n"估算并写进 CLI（默认保守 +
   超限警告），而不是等用户桌面被 OOM 杀掉两次。
3. **PPO 超参语义先吃透**：n_steps 是 per-env 的，多环境场景的默认 yaml 应该在
   Phase 1 就设计成"按样本预算"而非照抄单环境值。
4. **可视化第一天就接**：进度条/TB 在 Phase 1 就该是验收项（用户训练时才发现没有，
   返工横跨双后端）；eval success curve.png + tb 一起做，成本几乎不变。
5. **输出目录/gitignore/latest 指针这类"使用动线"设计前置**：用户第二次跑训练就
   会覆盖第一次——这类问题在写 save 时就该想到，而不是用户问。
6. **先测系统依赖再规划安装**：系统已有 torch 2.11+cu130，我配了 cu12x 索引让 uv
   下载 2.5GB 才被叫停。动手前 `pip list`/`python -c "import ..."` 一遍。
7. **抓取保持问题更早暴露给用户决策**：lift/place/stack 专家 0% 的根因（mesh 凸包 +
   硬位置伺服）在 Phase 2 就显现，应在进入 RL 前就给"修物理参数 vs RL 容忍"的分叉
   决策，而不是最后写进已知问题。
8. **每阶段末跑一次用户视角冒烟**：headless 测试全绿 ≠ 用户可用；用户动线
   （训练→看进度→评估→推理→RViz）应在每个 Phase 末走一遍真实命令。
