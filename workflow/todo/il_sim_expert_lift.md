# Todo: il_sim lift 脚本专家（teleop 移植版）

**状态**: 已移除，待重做 · **创建**: 2026-10-31 · **来源**:
`workflow/problems/il_sim_lift_expert_glm-5-3-flash-261030.md`（2026-10-30 交接）

## 结论（2026-10-31）

按要求移除 `src/so101_teleop/so101_teleop/teleoperators/expert.py` 中的 lift 抓取管线
（第三版逐动作生成器 `_lift_pipeline` + `TOOL_DOWN_R/SEED_POOL/GRASPABLE_RADIUS` 等
常量与分派逻辑），`ScriptExpertTeleop` 恢复为 **reach-only** 状态机，与 HEAD 零 diff。
`tests/test_il_sim_record.py` / `test_il_sim_smoke.py` 24 passed（reach 链路正常）。

移除原因：lift 绝对成功率仅 ~20–40%（本进程 A/B 4/20），不满足"连续 5 episode ≥80%"
的验收线；且 teleop 移植版长期存在 0% 的严重行为偏差。

## 已定位根因（重做时必须避免）

原 teleop 转录版 `servo_step_action` 把夹爪通道写死 `0.0`，而 envs
`scripts/envs_run_expert.py::_servo_step` 读当前夹爪角 `normalize_qpos(qpos)[5]`。
`0.0` = gripper ctrlrange `[-0.17453, 1.74533]` 的中点 ≈ 0.785 rad（**半开**）：

- 闭爪阶段：每个伺服帧把爪子弹回半开，闭合被反复打断 → 抓取率 1–5/12；
- 抬升阶段：抓住后整段半开 → 抓住必掉。

修正后同进程同 seed 逐帧动作序列与 uv 版 **bit 级一致**
（seed 100/101/105/106：`first_diff=None, maxdiff=0.000e+00`），证明是唯一转录偏差；
其余偏差（缺 `_wait(10)`、`walk_to_q` 判定时机、终点阈值、`_arm_step` 丢失）也已核对。

## 重做推荐路径

1. **共享实现，不再转录**：把 `scripts/envs_run_expert.py` 的
   `grasp_item / servo_close / lift_servo` 抽到无 ROS 库（如
   `libs/so101_envs/expert/lift.py`），uv 与 teleop 共用同一份动作生成逻辑；
   uv 侧 `env.step(a)` 与 teleop 侧 `yield a` 只是执行器差异。
2. **夹爪通道禁止写常量**：伺服帧保持当前 `normalize_qpos(qpos)[5]`；
   仅明确"张/闭"意图的帧才写 `+1.0 / -1.0`。
3. **验收带样本量**：跨进程 IK 浮点非确定性，小样本（<12）不可靠；
   调参必须大样本或同进程 A/B。
4. 实现后加测试：与 uv `run_lift` 同 seed 动作序列 bit 级比对（本会话的
   `/tmp/test_lift_equiv.py` 思路可固化进 `tests/`）。

## 未解决的成功率问题（envs 层，与 teleop 移植无关）

- **失败模式 A**：全开动爪闭合弧线沿切向把方块推走，闭爪后零接触
  （`is_grasped` 恒 False）。
- **失败模式 B**：`is_grasped` 刚过 0.5N 阈值的边缘抓取，抬升或保持 0.8s 内掉落
  （teleop 抬升后 8 帧收尾保持可复现：同轨迹下 uv 判成功、teleop 判失败）。
- 候选实验（详见 problem 文档"未验证的候选方案"）：
  物体相对帧手腕 ±90°（把动爪扫掠面从水平切向改为竖直径向）、
  抬升段爪压常值扫描、相机反馈伺服。
- 关联：`workflow/todo/envs_grasp_tuning_todo.md`（抓取物理/保持调优）。

## 验收线

- teleop 链路 `env:=lift` 连续 5 episode ≥80%（与 uv 链路成功率同量级且动作等价）。
- reach 回归不得低于历史 100%（2~3/5 的旧回归疑似 dffb7fd 求解器参数，需独立 bisect）。

## 复现命令

```bash
# uv 链路（当前仍可用，成功率 ~20-40%）
uv run python scripts/envs_run_expert.py --env SO101Lift-v1 --episodes 10
uv run python scripts/envs_run_expert.py --env SO101Lift-v1 --view

# ROS2 链路（移除 lift 专家后 env:=lift 会退化为零动作保持，不再演示抓取）
ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=false env:=lift rviz:=true mujoco:=true

# 回归
uv run pytest tests/test_il_sim_record.py tests/test_il_sim_smoke.py -q
```
