# reach / place / stack 严格对齐 squint 迁移报告

**日期**: 2026-10-31
**分支**: `rl`
**参考**: `/home/maxtensor/Master/EAI/project/github/squint`（ManiSkill/SAPIEN）
**前置**: lift 迁移见 `workflow/reports/lift_squint_migration.md`

## 迁移范围

| 项 | squint | 迁移后 |
|---|---|---|
| reach 物体 | cube half [0.011,0.014] / can r [0.014,0.019] half-h [0.025,0.035]，密度 200，摩擦 [0.1,0.5] | 同；DR 关取中点（3.125g / 10.26g / 0.3） |
| reach goal / success | item 正上方 `half+0.02`；TCP ≤0.02 & 静止 & **夹爪闭合** | 同 |
| reach reward | `[(1-tanh(5d)) + closeness + reached -3*touch_table -1*touch_item] / 4`，success 覆盖为 4 | 同 |
| place 物体 | 同 reach 尺寸/密度/摩擦 | 同 |
| place bin | half x [0.035,0.045] / y [0.045,0.055] / z [0.012,0.018]，壁厚 0.005，随机 yaw | 同；reset 逐 episode 改写 5 块 geom |
| place 出生 | item/bin 均在 `[0.3,0]±0.1` 且不重叠 | 同（拒绝采样） |
| place success | item 中心在 bin **本地** xy 内 & 未接触 item & 机器人静止 & 未接触 bin | 同 |
| place reward | squint 原式 /9 | 同 |
| stack itemA | cube half [0.011,0.014]，密度 200 | 同 |
| stack itemB | cube half [0.0125,0.0175] 或 can r/hh 同 reach | 同 |
| stack 出生 | 两物体 `[0.3,0]±0.1` 不重叠、随机 yaw | 同 |
| stack success | xy≤0.02 & \|z-(hA+hB)\|≤0.01 & A 静止(线速度≤0.02) & 未抓 A & 机器人静止 | 同 |
| stack reward | squint 原式 /9 | 同 |
| episode 长度 | 8 个任务均 50 | 同（原 place=75 / stack=100 已改） |
| 终止 | `ignore_terminations=True` | 四个任务 `TERMINATE_ON_SUCCESS=False` |
| 接触物理 | SAPIEN patch_radius | item `condim=4` + `solref=0.001 1 solimp=0.99 0.999 0.0001`（table/floor/手指同）；穿模 10–15mm → <1mm |
| 观测 | item/bin 位姿、相对位置、is_grasped、尺寸/摩擦/密度、clean_qpos | reach 47 维 / place 57 维 / stack 57 维 |

## 实现要点

- 公共几何采样/质量惯性/`mj_setConst` 包装进 `base_env.py`
  （`sample_box_geometry` / `sample_cylinder_geometry` / `apply_free_geometry` /
  `_recompute_constants`，后者保存/恢复 data，避免 setConst 清零 qpos）。
- 新增 `utils.is_touching_any`（bin 多 geom 接触判定）。
- `randomize.py`：`item_density=200`、`target_goal_thresh=0.02`（squint）。
- scene XML：reach/place/stack（cube+can）默认尺寸/密度/摩擦/接触参数全部更新。

## 验证

- `uv run pytest`：**exit 0**（新增 `tests/test_envs_squint_remaining.py` 10 例）
- 构造性 success：
  - reach：闭合夹爪 + goal 移到 TCP → success；张开 → 到达但失败
  - place：item 放到 bin 中心 → `in_bin=True`、`success=True`
  - stack：A 放到 B 顶（动态半高）→ `on_top=True`、`success=True`
- DR 范围：5 个 seed 内尺寸/摩擦/密度均落在 squint range，item/bin 不重叠
- RL 流水线 smoke：reach/place/stack 各 5000 步（delta + DR + squint 超参）正常训练/保存/评估
- 物体/接触：stiff 接触 + condim=4 下 lift 已实测指尖穿透 ≤0.41mm

## 偏差与已知问题

1. **TCP 已对齐**：所有 env 的 TCP 改为动态两指尖中点（base_env），IK 同步改为
   指尖中点并接受夹爪角（`SO101IK.solve(..., q_gripper=...)`）。旧 checkpoint
   按 site TCP 训练，需重训。
2. **视觉策略未迁移**：squint 默认 `rgb+state` + CNN；本项目 RL 仍是 state-only。
3. **夹爪刚度/阻尼 DR 未迁移**：MuJoCo position actuator 语义与 SAPIEN PD 不同。
4. **脚本专家**：按用户要求只保留 reach（`POLICIES` 仅 reach/reachcan）。
   - reach 已恢复 **100%**（5/5）：全程闭爪到达（闭爪会移动指尖中点 TCP），
     IK 用 `position_only=True`（reach 不约束朝向，否则远目标不可达）。
   - lift/place/stack 专家函数保留但不在 CLI 中（squint 迁移后失效，暂不使用）。
   - 专家仅作 sanity 工具，迁移验收以构造性测试 + RL 为准。

## 复现训练

```bash
# reach（squint 超参默认生效）
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Reach-v1 \
  --steps 1500000 --eval-freq 20000 --n-envs 4 --out outputs/rl/reach-squint
# place
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Place-v1 \
  --steps 1500000 --eval-freq 20000 --n-envs 4 --out outputs/rl/place-squint
# stack
uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Stack-v1 \
  --steps 2000000 --eval-freq 20000 --n-envs 4 --out outputs/rl/stack-squint
```

推理：
```bash
ros2 launch so101_rl_infer rl_infer.launch.py env:=place \
  checkpoint:=outputs/rl/place-squint camera:=true rviz:=true mujoco:=true
```
