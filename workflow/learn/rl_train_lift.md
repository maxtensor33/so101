# lift RL 训练与 squint 迁移复盘：收获与坑

> 作者：**deepseek-v4-1-flash-economy-261017[1m]**（Claude Code 会话内运行）· 2026-10-31
> 背景：`SO101Lift-v1` RL 长期无法收敛。用户要求对照参考项目
> `/home/maxtensor/Master/EAI/project/github/squint`（ManiSkill/SAPIEN）核查是否
> 如实迁移。本会话完成了 lift（含 can）严格迁移、穿模修复、剩余 5 个任务
> （reach/place/stack + can）迁移、TCP 对齐、推理侧 MuJoCo/Enter 重置，以及
> 训练速度差距的量化分析。
> 产物：`workflow/reports/lift_squint_migration.md`、
> `workflow/reports/envs_squint_migration.md`、新增/修改测试若干。

## 一、先给结论

原实现**不是如实迁移**，且失败不是单一原因。六个关键偏差叠加：

1. **动作模式**：squint 用 `pd_joint_target_delta_pos`（每步增量 ±0.1 rad、夹爪 ±0.2），
   我们用全行程绝对位置目标；
2. **物体物理**：squint cube half 0.011–0.014 / 密度 200（约 3g），我们是 4.4cm / 25g；
3. **奖励/终止**：squint 奖励 `/3` 归一化、success 不终止，我们额外 +2 且 success 即终止；
4. **success 判据**：squint 用控制器 target qpos 距 rest <0.2，我们用实际 qpos <0.5；
5. **观测**：缺 `is_grasped`、`dist_to_rest`、item 尺寸/摩擦/密度、clean_qpos；
6. **RL 配置没接线**：`sb3_sac_squint.yaml` 存在但 CLI 没有 `--config`，
   delta 动作测试引用的 `DELTA_MAX/action_mode` 根本未实现。

迁移后进一步定位到**物理侧两个隐蔽问题**：

7. `moving_jaw_so101_v1_convex.obj` 是 218 个凸分解块，MuJoCo 把它们合成一个
   大凸包，填平指缝 → 方块陷进碰撞体 6–13mm；
8. 3g 小物体在默认 `solimp` 下接触刚度过软，方块被压进桌面/指尖 10–15mm（视觉穿模）。

最终 lift 物理可行：指尖穿透 <0.41mm、保持 30/30、success 100%（构造性）；reach 专家
100%（5/5）；全套 pytest exit 0。

## 二、最大的收获

### 1. “迁移”必须逐项对照源码，不能只看任务骨架
最初对照只看了 rest 位形、spawn、success 三段式、50 步，误以为“已镜像”。
真正影响可学性的是：**控制模式、奖励尺度、success 判据、观测特权信息、物体尺寸/
质量、DR、训练超参**。对照表应覆盖“状态/动作/奖励/终止/观测/复位/物理参数/训练配置”
九类，而不是任务名字。

### 2. 物理引擎差异必须用运行时测量，不能照抄网格坐标
- squint 的 `finger1_tip/finger2_tip` 在 URDF 里只是两个固定 frame；我们的 MJCF
  没有，但可以用 body 变换 + 局部偏移复算（数值与 pinocchio 结果一致）。
- squint 的 `moving_jaw_so101_v1_convex.obj` 在 SAPIEN 里是 218 个凸块；MuJoCo 一个
  mesh 只能一个凸包。**同一资产在不同引擎的碰撞语义完全不同**。
- 所有“网格位置/夹持中心”结论必须用运行时 `d.contact`、`geom_xpos/xmat`、
  渲染 PNG 验证；手工坐标换算在本项目中已多次出错（见 `envs_grasp_tuning_todo.md`）。

### 3. 小质量物体的接触刚度与 condim 是抓取成败关键
- `condim=4`（扭转摩擦）把保持从 11/30 提到 30/30（SAPIEN patch 接触的 MuJoCo 等效）。
- `solref="0.001 1" solimp="0.99 0.999 0.0001"` 把方块压进桌面 7.7mm、压进指尖
  10–15mm 的穿透压到 <1mm；只调 item 不够，**table/finger geoms 的 solimp 也要一起调**。
- 把步长改成 1ms+100 substeps 反而失稳（10–14mm、3/6 成功）；2ms×50 保持。

### 4. TCP 是任务语义，不只是观测字段
- squint TCP = **动态两指尖中点**；我们最初用 `gripperframe` site（闭爪收敛点）。
  25mm cube 在夹爪目标 0–0.2 rad 时两者差 ~24mm，按 site 放置会滑落。
- TCP 随夹爪角变化：reach 如果“开爪到位再闭爪”，闭爪会使中点移动，success 丢失；
  reach 专家必须**全程闭爪**到达；IK 需要把 `q_gripper` 作为已知参数，reach 用
  `position_only=True`（不锁朝向）否则远目标不可达。
- “兼容旧 checkpoint”是产品决策，不是技术必需。用户明确不兼容后，才能把 TCP/语义
  彻底对齐 squint。

### 5. 训练速度差距要用数据拆解，不能归因于“GPU”
实测（squint README/results CSV + 用户 TB）：

| 指标 | squint LiftCube | 本项目用户 run |
|---|---|---|
| 首次 80% | 200k samples / 173s | — |
| 首次 100% | 300k / 229s | 800k / ~25min |
| 吞吐 | ~1,300 samples/s | ~540 samples/s |
| 配置 | 1024 GPU envs, rgb+state | state, DummyVecEnv（串行） |

到 100% 的墙钟差 ≈7× = 样本效率 2.7× × 吞吐 2.4×。原因分解：

- **GPU**：SAPIEN/PhysX 批量 1024 env + GPU 128×128 渲染 + torch GPU 回放/更新；
  我们的 `DummyVecEnv` 是串行，`n_envs` 只增加多样性，不增加墙钟。
- **仿真步数**：squint 每控制步 10 substep（100Hz sim）；我们 50 substep（500Hz），
  物理计算 5×。
- **算法/架构**：squint 是分布式 C51 SAC（101 atoms、2 Q、LayerNorm 512/256、
  CNN+state），UTD≈0.25；我们用 SB3 标准 SAC + MLP，若 `n_envs=1` 则 UTD=1（4×更新）。
- **数据多样性**：1024 并行 env 的每个 batch 覆盖更广的状态分布。
- **物理版本**：用户 run 时还是软接触+site TCP（深穿透），任务更难；新物理下重训
  应缩短收敛样本。
- **每步 Python 开销**：obs/reward 里重复 `is_grasping` 接触扫描（逐 contact 循环）。

### 6. “headless 测试全绿”不等于用户可用
本会话两次被用户视角发现盲区：
- 用户说“能抓起来但穿模”——headless 只测了接触检测/成功，没测**视觉穿透/几何一致性**；
- `mj_setConst` 清零 data 的 bug 下测试仍全绿，因为测试大多不检查 reset 后 qpos
  是否等于 rest（只检查形状/成功构造）。后来用“reset 后 qpos 与 REST 差值”才暴露。

## 三、踩过的坑（按代价排序）

### 1. `mj_setConst` 会把整个 data 重置为默认态（最高代价）
- 现象：lift reset 后机器人 qpos 全 0（直臂），测试却通过；TCP 位于 389mm 而不是
  229mm。训练/评估实际在错误初始位形下跑。
- 根因：运行期改 `geom_size/body_mass` 后调 `mujoco.mj_setConst(m,d)` 重算 rbound；
  该函数同时把 qpos/qvel/ctrl/time 清零。
- 解法：`_recompute_constants()` 先保存 qpos/qvel/ctrl/time/mocap，再 setConst，再恢复。
- 规则：**任何运行期改模型常量的操作，都要保存/恢复 data，并加一条 reset 后
  qpos≈REST 的断言**。

### 2. 218 块凸分解被 MuJoCo 合成一个大凸包
- 现象：夹爪视觉上穿进方块；`contact.dist` 达 -8~-13mm。
- 根因：`moving_jaw_so101_v1_convex.obj` 的 `o convex_0..217` 被当一个 mesh 凸包化，
  把指缝填平。
- 解法：只保留夹持面 `convex_37/38` → `moving_jaw_pad.obj`；同时用接触参数把剩余
  穿透压到 <1mm。
- 规则：多凸块 OBJ 在 MuJoCo 中必须**逐块拆 mesh geom**；不能指望 OBJ 的 group 语义。

### 3. 软接触 + 小质量 → 深穿透（视觉穿模）
- 扫参：item/table/finger 全部 `solref=0.001 1, solimp=0.99 0.999 0.0001`；
  item `condim=4`。
- `dt=1ms/100 substep` 失稳；保持 2ms×50。
- 规则：穿透验收用 `contact.dist`（物体-手指/桌面），不要只看 success。

### 4. 夹持中心/TCP 用错
- 旧 `gripperframe` site 不是方块中心；25mm cube 的正确中心是**动态指尖中点**
  （夹爪目标 0–0.2 rad）。
- 全闭合目标（-0.174）会过度闭合把方块挤进指尖；固定夹爪目标也不稳定。
- 规则：抓取/保持测试用“动态指尖中点 + 适中夹爪目标”，不要用全闭合。

### 5. 观测维度改动引发连锁测试
reach obs 25→47、lift 29→52、place/stack 57。
受影响：`test_envs_spaces`、`test_rl_smoke`、`test_rl_backends`、`test_rl_normalize`、
`test_il_sim_record`（reach success 现在要求闭爪）。
- 规则：obs 语义变更时，grep 所有 `25/46/obs_dim/zeros(` 硬编码；用
  `env.observation_space.shape` 代替魔法数。

### 6. reach success 要求闭爪，连带 IL/遥操自动重置语义
- squint reach success = 到达 & 静止 & **夹爪闭合**。
- 旧 reach 专家/IL 专家/leader 遥操开爪到达 → success 永假 → auto_reset 不触发。
- 解法：专家全程闭爪；leader 用户需到达后闭爪才自动重置（行为变化，已告知用户）。
- 规则：success 语义变更必须检查所有消费方（RL eval、IL auto reset、脚本专家）。

### 7. DummyVecEnv 串行 vs 多环境
- 文档里“DummyVecEnv 通常快于 SubprocVecEnv”是**单环境**结论；多环境要真墙钟提升
  必须 Subproc/GPU，否则 `n_envs` 只增加多样性。
- 64 Subproc 曾 OOM 连坐杀 GUI；建议 8–16。
- UTD：squint 每 1024 env 步做 256 更新（0.25）；SB3 `n_envs=1` 时 1.0，
  `n_envs=4` 时 0.25；`gradient_steps` 要按 n_envs 调。

### 8. 资产/接触改动的隐藏回归
- `so101_env.xml` 的指尖摩擦/接触参数是全局的，会同时影响 reach/place/stack；
- place/stack 的 bin/item 尺寸是运行时改的，`_recompute_constants` 必须保存/恢复；
- `moving_jaw_pad.obj` 是新增未跟踪资产，提交时必须带上。

### 9. 脚本专家的期望值
- 迁移后 reach 100%（闭爪 + position-only IK），lift/place/stack 旧专家按 4.4cm/
  软接触调参已失效；用户明确“脚本专家保留 reach”，其余从 CLI 移除。
- 规则：专家是 sanity 工具，不是迁移验收标准；验收用构造性测试 + RL。

## 四、如果重来一次

1. **先写“squint ↔ 本项目”九类语义对照表**（动作/奖励/终止/观测/复位/物理/
   success/episode/训练配置），逐项打勾再动代码；不要在任务骨架上自我说服。
2. **先做物理验收测试**：`夹持中心闭合 → 释放 → 保持 + contact.dist 穿透 <0.5mm`，
   再谈 RL。否则策略学会了也会穿模。
3. **reset 后第一断言**：qpos≈REST、目标 ctrl≈qpos、item 在桌面上。
4. **运行期改模型后立刻 `mj_setConst` 保存/恢复 data**，并加回归测试。
5. **TCP 早定为动态指尖中点**，IK 同步接受夹爪角；reach 用 position-only。
6. **checkpoint 元数据带 env_kwargs**（action_mode/DR），推理侧按 meta 重建 env；
   否则 delta/position、DR 开关不一致，策略行为漂移。
7. **训练前先 benchmark**：`n_envs`、substeps、uTD、接触扫描的 steps/s 占比；
   不要用“1.5M 步”当唯一预算。
8. **用户视角冒烟**：训练→进度/TB→评估→`mujoco:=true` 推理→Enter 重置，
   每个阶段末跑一遍真实命令。
9. **不要为不存在的兼容性妥协语义**：用户明确会重训后，直接对齐 squint。
10. **文档与实际同步**：envs_report 的 4.4cm/75/100 步、reach 100%、专家 0% 都被
    迁移取代；报告/CLAUDE.md 必须同步更新。

## 五、可复用验收命令

```bash
# 全套测试
uv run pytest

# lift squint 语义 + 物理保持/穿透
uv run pytest tests/test_envs_lift_squint.py tests/test_envs_delta_action.py

# 剩余环境语义
uv run pytest tests/test_envs_squint_remaining.py

# 脚本专家（仅 reach/can）
uv run python scripts/envs_run_expert.py --env all --episodes 5

# lift 训练（squint 配为默认）
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Lift-v1 \
  --steps 1500000 --eval-freq 20000 --n-envs 4 --out outputs/rl/lift-squint

# 推理可视化 + Enter 重置
ros2 launch so101_rl_infer rl_infer.launch.py env:=lift \
  checkpoint:=outputs/rl/lift-squint camera:=true rviz:=true mujoco:=true
```

## 六、遗留与下一步

- **旧 lift checkpoint 已不适用**（TCP/物理/obs 语义变更），需重训；
- 视觉策略（rgb+state/CNN）与夹爪刚度/阻尼 DR 未迁移；
- 脚本专家 lift/place/stack 已暂停（仅保留 reach）；
- GPU 加速路线见 `workflow/todo/rl_gpu_parallel_todo.md` 与本文件第二节第 5 点：
  优先 CPU Subproc + 降 substep + UTD 对齐；视觉/大规模并行再上 MuJoCo Warp。

相关：`workflow/reports/lift_squint_migration.md`、
`workflow/reports/envs_squint_migration.md`、`workflow/learn/rl.md`、
`workflow/todo/rl_gpu_parallel_todo.md`。
