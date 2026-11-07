# TDD 证据报告: SO101 Image-based DAgger（plan/rl_dagger.md）

**来源计划**: `plan/rl_dagger.md`（用户确认后执行）
**日期**: 2026-11-07
**分支**: `rl`（先 merge main fast-forward 到 aef6fca；commit 由用户手动执行）
**执行方式**: 主会话按组分派子代理顺序实施（A 算法本体 → B 脚本 → C infer 包 → D develop 包 → E 收尾），每组之间主会话独立验收（重跑 test.sh 汇总行 + git status 核对改动面）

## 用户旅程（源自计划）

1. 作为 RL 研究者，我想要 image-based 的 student 策略（DAgger 蒸馏已有 state RL expert），
   以便 student 只靠相机即可执行任务（真机无需 qvel/tcp 观测）。
2. 作为训练工程师，我想要 `rl_dagger_train/eval` 脚本，用现有 rl checkpoint 当 teacher
   一条命令完成 DAgger 训练与成功率评估。
3. 作为部署工程师，我想要仿真推理节点（RViz/MuJoCo 可视化）与实机部署开发包
   （estop/homing/confirm_step/cv2 相机），沿既有分阶段验收流程上真机。

## 任务 → 测试 → 证据映射

执行中途一次规格变更（用户确认）：图像尺寸 128x128+关标定 → **480x640（env 原生）+ 标定默认开启**（覆盖计划 D4），各组已对齐；buffer_size 相应下调（见风险闭环 R1/R2）。2026-11-07（下午）二次变更：因机器性能全链路回退 128x128（标定默认开启不变），buffer_size 恢复 100000。

| # | 计划任务 | 实现 | RED 证据 | GREEN 证据 |
|---|---|---|---|---|
| 1 | Phase 1: StudentPolicy + DAggerBuffer | `libs/so101_rl/dagger.py`（组 A） | collection error（模块不存在） | 11 passed |
| 2 | Phase 2: train_dagger + expert 标注 + meta 落盘 | 同上（组 A，含假 expert 微型 env smoke） | 同上 RED 覆盖 | 同上 |
| 3 | Phase 3: rl_dagger_train.py / rl_dagger_eval.py | 组 B | collection error（脚本不存在） | 9 passed（含真实子进程端到端：微型 sb3 expert→train bc_loss 0.083→0.040→eval 统计） |
| 4 | Phase 4: so101_rl_dagger_infer 包 | 组 C（镜像 rl_infer 全套） | 9 failed | 9 passed |
| 5 | Phase 5: so101_rl_dagger_develop 包 | 组 D（镜像 il_sim_develop） | collection error（包不存在） | 24 passed |
| 6 | Phase 6: 收尾 | 主会话：docs/commands.md rl_dagger 节 + 本证据 + 计划引用修复 | test_docs_links 2 failed（s2v 外部脚本误命中 + 本文件未产出） | 全量收尾汇总见下 |

## 测试规格（人类可读保证）

| # | 保证内容 | 测试 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | StudentPolicy 输出 (6,)∈[-1,1]；单/双相机可配；save/load roundtrip | tests/test_rl_dagger.py | unit | PASS |
| 2 | DAggerBuffer 环形覆盖最旧轮；均匀采样形状；uint8 存/float32 取 | tests/test_rl_dagger.py | unit | PASS |
| 3 | train_dagger 假 expert 微型 env 短训：loss 有限、ckpt 合法、load 可推理 | tests/test_rl_dagger.py | integration | PASS |
| 4 | 训练/评估脚本 CLI：meta env 推导相机版 id、--overrides 生效、端到端 train→eval | tests/test_rl_dagger_scripts.py | integration | PASS |
| 5 | infer 包 core 无 ROS import；make_obs_batch 单/双相机顺序敏感；结构断言（setup.py lib wrapper/launch） | tests/test_rl_dagger_infer.py | unit+structure | PASS |
| 6 | develop 包 real_infer（BGR→RGB+裁方+resize/denorm/clamp/homing/EstopLatch）+ 结构断言 + 微型 student 端到端 | tests/test_rl_dagger_develop.py | unit+structure | PASS |

## 关键实现记录

- **expert 复用零迁移**：teacher 走既有 `resolve_ckpt_dir→read_meta→get_trainer→load_policy` 链（sb3/skrl 均可），DAgger 循环把 expert 当黑盒可调用（单条 state → action(6)∈[-1,1]）。
- **student 纯视觉**（计划 D2）：state 通道仅训练时供 expert 标注；实机部署只需相机帧 → `make_obs_batch({"rgb": frames}, ...)`。
- **checkpoint 契约**：`model.pt + meta.json`（backend="dagger"，env_kwargs 含 camera/render_size/camera_calib/action_mode/domain_randomization；hparams 含 camera_keys/in_hw）。
- **坑（组 B 发现，组 C 测试固化）**：env 的 `camera_calib` kwarg 语义 `None`=默认开启 / `False`=关闭，**传 True 会 pathlib 崩**——重建 env 一律 `gym.make(env_id, **meta["env_kwargs"])` 原样透传。
- **DAggerBuffer 采样**：按轮过渡数加权的单段均匀采样（轮大小相当时与"均匀轮+轮内随机"等价），docstring 注明。
- **多相机**：每图过同一 encoder，特征按 camera_keys 顺序拼接后过共享 head（encoder 参数不随相机数增长）。

## 风险闭环（对照计划 Risks）

| 计划 Risk | 结局 |
|---|---|
| R1 render_size 与相机标定兼容 | 已消除：apply_camera_calib 只覆写 cam_pos/quat/fovy，与分辨率解耦，标定默认开启；全链路现统一 128x128 = env 原生 RENDER_SIZE（1:1 纵横比，水平视野比 4:3 窄，但几何一致） |
| buffer 内存 | 已消除：buffer_size 默认 100000（128x128 单相机 uint8 ≈0.047MB/过渡 → ≈4.7GB RAM），CLI --overrides 可调 |
| 图像渲染慢 | 部分缓解：sync 向量化 n-envs=4；训练时长以 bc_loss 收敛为准，未设硬指标 |
| expert(vecnormalize) 与多 env 并存 | 组 B 端到端微型 sb3 expert 验证通过 |
| sim2real 图像域差 | 未验收（真机沿 il_sim_develop 分阶段流程，属后续方向） |
| student 上限受 expert 约束 | 信息项，reach 先验收（计划既定） |

## 收尾后变更（2026-11-07 同日）

1. **图像尺寸回退 480x640 → 128x128**（机器性能约束：640x480 软渲染 ~0.7s/帧扛不住）。
   全链路单源对齐 `base_env.RENDER_SIZE=(128,128)`：dagger.py DEFAULT_HP/StudentPolicy 缺省
   in_hw、dagger.yaml render_size/buffer_size（回 100000 ≈ 4.7GB）、脚本 --render-size 默认、
   tests、docs/commands.md 均已同步；pitfalls/envs.md 与 pitfalls/rl.md 新增两条记录
   （单源变更清单 + 标定分辨率解耦结论）。上文风险闭环中 R1/R2 的"480x640/10000"表述为
   当时的中间态，以本条为准。83 例回归（rl_dagger 4 文件 + envs_spaces + il_sim_record）全绿。
2. **train_dagger 新增 tqdm 轮级进度条**（`progress` 超参默认开，CLI `--no-progress` 关闭；
   postfix 实时显示 steps/bc_loss/updates，日志经 bar.write 防打断），测试 53→54 例。

## 收尾验证

- 文档修复前全量：`2 failed, 545 passed, 1 skipped`（两个 failed 均为本文档产出前的元问题：计划引用外部参考脚本误命中 SCRIPT_REF + 本证据文件未产出）
- 文档修复后全量收尾：`547 passed, 1 skipped`（199.87s，含 4 个 rl_dagger 测试文件 53 例）

## 已知遗留

- develop 包 launch 无 RViz 配置/节点（模板 gui/rviz 参数省略，需定制布局时后续补 .rviz 文件）
- estop 语义取模板行为（锁存后完全冻结、不自动 homing）；如需"estop→归位"为单行改动，待用户定夺
- 全量测试收尾结果见下方「收尾验证」；真实训练验收报告按 reports 惯例待产出（完成 reach 实训后补）
