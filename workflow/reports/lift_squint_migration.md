# lift 严格对齐 squint 迁移报告

**日期**: 2026-10-31
**分支**: `rl`
**参考**: `/home/maxtensor/Master/EAI/project/github/squint`（ManiSkill/SAPIEN）

## 背景

RL 在 `SO101Lift-v1` 上无法学习。对照 squint 后发现原实现并非如实迁移：
绝对位置控制、4.4cm/25g 大方块、奖励未归一化且 success 终止、观测缺
`is_grasped`/`dist_to_rest`、squint 超参配置未接线。本报告记录逐项对齐结果。

## 逐项对齐

| 项 | squint | 迁移后实现 |
|---|---|---|
| 控制 | `pd_joint_target_delta_pos`（±0.1 rad, 夹爪 ±0.2, use_target） | `base_env.py` 新增 `action_mode="delta"`：目标积分 + ctrlrange 限幅；RL 默认 delta |
| 物体 cube | half 0.011–0.014, 密度 200, 摩擦 0.1–0.5 | `lift.py` 每 episode 采样 size/mass/inertia；DR 关取中点（0.0125/3.125g/0.3） |
| 物体 can | r 0.014–0.019, half-h 0.025–0.035, 密度 200 | 同 cube；DR 关中点 r=0.0165 / hh=0.03 / 10.26g |
| 出生域 | `[0.3,0] ± 0.1`，无裁剪 | 同；删除 `GRASPABLE_RADIUS=0.26` 钳制 |
| 初始位形噪声 | `rest + randn*0.02` | `rng.normal(0, 0.02)` |
| 观测噪声 | DR 开时 qpos 噪声 5° | `robot_qpos_noise_std=np.deg2rad(5)`，DR 开时加在 state obs 的 qpos 段 |
| 观测 | qpos/qvel/tcp/item_pose/is_grasped/dist_to_rest/target + item 尺寸/摩擦/密度/clean_qpos | lift state 52 维：基础 19 + 任务 33（含 target_qpos、dist_to_rest、is_grasped、尺寸/摩擦/密度、clean_qpos） |
| success | `lifted & grasped & ‖target_qpos[:-1]-rest[:-1]‖ < 0.2` | 同；lifted 阈值 `half_z + 1e-3` |
| TCP | 两指尖中点（`finger1_tip`/`finger2_tip`） | **动态两指尖中点**（base_env.tcp_pos，随夹爪角变化）+ gripper_link 朝向 |
| 奖励 | `[1-tanh(5d)+g+exp(-2*drest)*g-3*touch-(¬lifted)] / 3` | 同式、同 /3 归一化 |
| 终止 | `ignore_terminations=True`（success 不终止） | `SO101LiftEnv.TERMINATE_ON_SUCCESS=False` |
| 抓握判定 | 双指力 ≥0.5N + 力方向与开合轴 ≤110° | 同语义；开合轴由接触法向推导（URDF/MJCF 帧约定不同，不能照搬 body y 轴） |
| 接触惩罚 | 仅双指接触桌面，阈值 1e-2 N | `is_touching(..., min_force=1e-2, bodies=双指)` |
| 指尖摩擦 | gripper 材质 2.0 | `so101_env.xml` 指尖碰撞 3.5 → 2.0 |
| 接触模型 | SAPIEN patch_radius 软接触 patch | item `condim=4`（+扭转摩擦）作为 MuJoCo 等效；扫参见下 |
| SAC 超参 | gamma .9 / tau .01 / batch 512 / buffer 1M / starts 5k | `configs/sb3_sac_squint.yaml`；`rl_train.py --config squint` 已接线（默认） |
| checkpoint | — | `meta.json` 新增 `env_kwargs`（action_mode / DR）；`rl_eval`、`rl_infer` 按 meta 重建 env |

## 变更文件

- `libs/so101_envs/base_env.py`：delta 动作、DR 开关、qpos 观测噪声、target_qpos、终止 hook
- `libs/so101_envs/lift.py`：按 squint 重写（尺寸/密度/观测/奖励/success）
- `libs/so101_envs/utils.py`：双指方向感知 `is_grasping`、`is_touching(bodies=...)`
- `libs/so101_envs/randomize.py`：spawn `[0.3,0]`、摩擦 `(0.1,0.5)`、观测噪声
- `libs/so101_envs/assets/scene_lift.xml`、`scene_lift_can.xml`、`so101_env.xml`
- `libs/so101_rl/`：超参变体加载、env_kwargs 随 checkpoint 存/载
- `scripts/rl_train.py` / `rl_eval.py`、`src/so101_rl_infer/.../rl_infer_node.py`
- `tests/test_envs_lift_squint.py`（新增 7 例）、`test_envs_delta_action.py`（已转绿）

## 验证

- `uv run pytest`：全套通过（exit 0；含新增 lift-squint 8 例与 delta 6 例）
- `tests/test_rl_backends.py::test_deploy_denormalize_delta_mode`：delta 部署积分语义
- 10k 步冒烟：`rl_train --env SO101Lift-v1 --steps 10000 --n-envs 4`
  - 正常训练/保存/绘制曲线；`meta.json` 记录 `action_mode=delta, domain_randomization=true`
  - eval success 0%（10k 步不足以学习 lift，符合预期；1.5M 步命令见下）
- 抓取保持实测（SO101Lift-v1, DR off）：夹持中心闭环 → 释放 → 保持 30 步
  item z 稳定，`is_grasped=True`、`success=True`（`tests/test_envs_lift_squint.py::
  test_grasp_holds_after_release` 锁定）。
- 接触模型扫参（同序列，统计保持步数/30）：
  - `condim=3` + 指尖摩擦 2.0：11/30（滑落）
  - `condim=4`：30/30；`condim=6`：30/30
  - 结论：item `condim=4`（扭转摩擦）替代 SAPIEN patch_radius 软接触的保持能力。

## 2026-10-31 追加：视觉穿模修复

用户反馈 RL 成功后夹爪**穿过方块**。三个根因与修复：

1. **凸分解被 MuJoCo 合成一个大凸包**：`moving_jaw_so101_v1_convex.obj` 含 218 个
   `o convex_i` 凸块；MuJoCo 把整个文件合成单一凸包，填平指缝，方块陷进碰撞体
   6–13mm。修复：只保留夹持面附近的 `convex_37/38` → `moving_jaw_pad.obj`。
2. **3g 小物体接触刚度过软**：MuJoCo 接触刚度随约化质量下降；桌面/手指 geom 仍是
   默认 `solimp=(0.5,0.95,0.001)`。实测方块被压进桌面 7.7mm、压进指尖 10–15mm。
   修复：item / table_top / floor / 全部手指碰撞 geom 统一
   `solref="0.001 1" solimp="0.99 0.999 0.0001"`。
   - 不用 `dt=1ms + 100 substeps`：实测反而失稳（10–14mm、成功率 3/6）。
3. **真实夹持中心是动态指尖中点**：`gripperframe` site 是闭爪收敛点；方块中心应在
   **当前夹爪角下的两指尖中点**（25mm cube 对应夹爪目标约 0~0.2 rad）。全闭合目标
   （-0.174）会过度闭合，把方块挤进指尖。

**验证（用户训练好的 `outputs/rl/lift-squint`，10 episodes；该次验证在 TCP 改为
指尖中点之前，旧 checkpoint 现已不适用，仅保留为接触物理证据）**：
- 手指-方块最大穿透 **0.41mm**（此前 10–15mm）
- 含掉落/地面在内的最大穿透 1.15mm（ep8 方块掉到地面，非夹持穿模）
- success 9/10（旧策略在新接触参数下偶发掉落；用新物理重训应恢复）
- 回归测试 `test_grasp_holds_after_release` 已改为“动态指尖中点夹持 → 释放”，
  断言 30/30 保持 + 穿透 <0.5mm。

## 复现训练

```bash
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Lift-v1 \
  --steps 1500000 --eval-freq 20000 --n-envs 4 --out outputs/rl/lift-squint
uv run python scripts/rl_eval.py --ckpt outputs/rl/lift-squint --episodes 10
ros2 launch so101_rl_infer rl_infer.launch.py env:=lift \
  checkpoint:=outputs/rl/lift-squint
```

`rl_infer` 从 `meta.json` 的 `env_kwargs` 重建 env，动作语义与训练一致。

## 仍未迁移（明确偏差）

1. **视觉策略**：squint 训练 `rgb+state`（手腕相机 + CNN + 颜色抖动）。本项目 RL 仍为
   state-only（MlpPolicy）。视觉 RL 需 CNN 编码器与大 batch GPU 渲染（本机渲染 153 steps/s），
   留作后续方向，不影响 state 版 lift 学习。
2. **夹爪刚度/阻尼 DR**：squint 随机化 gripper stiffness 500–2000 / damping 50–200；
   本项目执行器是 MuJoCo position actuator（kp/kv 语义不同），未迁移该维度。
3. **并行度**：squint 1024 GPU env；本项目 CPU `DummyVecEnv`（见
   `workflow/todo/rl_gpu_parallel_todo.md`）。
4. **place/stack/reach 的物体尺寸**：仍为旧值（4.4cm cube）；本次范围仅 lift（+can）。
   后续如同样失败，按同一方法迁移。
5. **TCP 定义（已对齐）**：obs/reward/TCP 已改为动态两指尖中点（base_env），与
   squint 一致。25mm cube 在夹爪目标 0~0.2 rad 时中点=方块中心。此前的
   `outputs/rl/lift-squint` checkpoint 按旧 site TCP 语义训练，已不适用，需重训。
6. **mj_setConst 注意**：运行期改物体尺寸/质量必须调 `mj_setConst` 重算 rbound；
   该调用会把 data 重置为默认态，`_apply_geometry` 已保存/恢复 qpos/ctrl/time。
