# 验收报告: il_sim_leader（真实 leader 臂遥操 il_sim）

日期：2026-10-29 · 分支：il_sim · 计划：`workflow/plan/il_sim_leader.md` · TDD 证据：`workflow/tdd/il_sim_leader.tdd.md`

## 交付物

| 组件 | 文件 | 说明 |
|---|---|---|
| 包装类 | `src/so101_teleop/so101_teleop/teleoperators/leader.py` | 注册名 `so101_leader_sim`；内部组合 lerobot `SO101Leader`（串口/标定/重试复用官方），输出换算为管线统一 6 维 [-1,1]；度走 follower 标定链（homing+deg*2048/180→range 线性映射，D2）；gripper 0-100→[-1,1]；per-joint `invert` |
| CLI 接线 | `cli_config.py` + `scripts/il_sim_teleop.py` | `make_teleop_config` leader 分支（`--leader-port/--leader-id/--invert`）；reset 策略同 keyboard（D4）；`il_sim_record.py` 走 draccus 透传零改动 |
| 探针 | `scripts/il_sim_leader_probe.py` | 只读打印 deg/0-100 → rad → [-1,1]，供方向/零位/夹爪校验 |

## 验证结果

- `colcon build`：11 packages finished
- `uv run pytest`：**377 passed**（基线 359 + 新增 18）
- 探针脚本语法/入口冒烟通过

## 探针实测（Phase 3, 2026-10-29 用户执行）

| 关节 | 操作员动作物理方向 | norm 变化 | 结论 |
|---|---|---|---|
| shoulder_pan | 右 | 增大 | 同向 ✓ |
| shoulder_lift | 前 | 增大 | 同向 ✓ |
| elbow_flex | 前 | 增大 | 同向 ✓ |
| wrist_flex | 前 | 增大 | 同向 ✓ |
| wrist_roll | 右 | 增大 | 同向 ✓ |
| gripper | 开 | 增大 | 同向 ✓（env 约定 qpos 大=张开） |

**结论：无关节需要 `--invert`**，遥操/采集不带该参数。home 偏差未逐项记录，
以遥操实操体验为准（若起始姿态明显别扭再回评 D2 备选映射）。

## 真机实测问题与修复（2026-10-29, Phase 4 第一轮）

| 现象 | 根因 | 修复 |
|---|---|---|
| 只有 gripper 能动, 其余关节钉死 | D2 初版误用 lerobot 0.5.x 度公式（homing_offset+deg*2048/180）; 0.6.1 实际是量程中点公式（deg=(raw-mid)*360/4095, homing 不参与）→ raw 恒低于 range_min, norm 钳在 -1 | `deg_to_norm` 改 0.6.1 公式; 测试加 `test_homing_offset_ignored` + 真实量纲 sanity |
| 仿真和实机不符 | 同上（norm 钳死） | 同上 |
| RViz 没弹 | 直跑脚本只起话题线程不起 RViz | 用 launch: `ros2 launch so101_teleop teleop.launch.py teleop:=leader` |
| 夹得住保持不住 / 视觉穿模 | MuJoCo 接触实验证实: 双爪碰撞正常（瞬间闭合 29/40 帧双侧接触, 穿透 6-8mm=软化执行器正常形变量）; 真正问题是 10Hz 手驱噪声破坏摩擦保持 | D7 动作平滑: norm 每步限幅 0.15/夹爪 0.25（可关）, env 重置后重吸附 |

探针方向表（Phase 3）: pan 右/lift 前/elbow 前/wrist_flex 前/wrist_roll 右/gripper 开 → 全部同向增大, **无需 invert**;
gripper 开=增大与 env 约定（qpos 大=张开）吻合。

## 真机验收（用户执行）

1. 探针：`uv run python scripts/il_sim_leader_probe.py` — 记录 leader home 时各关节 norm 值（home 偏差）、
   逐关节掰动确认方向（反向 → `--invert <关节>` 重跑确认）、夹爪开合方向
2. 纯遥操：`uv run python scripts/il_sim_teleop.py --teleop leader [--invert ...] --env reach`
   （MuJoCo 窗口 + RViz，leader 带动仿真臂）
3. 采集：`uv run python scripts/il_sim_teleop.py --teleop leader --record --env reach \
     --repo-id il_sim/reach_leader --episodes 10`
   或 draccus 透传：`uv run python scripts/il_sim_record.py --robot.type=so101_sim \
     --teleop.type=so101_leader_sim --teleop.port=/dev/ttyACM0 --teleop.id=L07252801 \
     --dataset.repo_id=il_sim/reach_leader --dataset.root=outputs/datasets/il_sim/il_sim/reach_leader \
     --dataset.num_episodes=10`
4. 回读：`LeRobotDataset("outputs/datasets/il_sim/il_sim/reach_leader")` features 校验
5. 可选：`il_sim_train.py --repo-id il_sim/reach_leader` 冒烟

## 已知缺口 / 遗留

- home 偏差（D2 caveat）待探针实测；若影响操作可切备选映射（leader 自身 range 直映射），决策点在 report 回填
- 手动 reset 手势（如张开夹爪保持触发）本期不做
- commit 待用户批准
