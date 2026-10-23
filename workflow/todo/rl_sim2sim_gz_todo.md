# RL sim2sim: gz 验证（TODO, plan/rl.md Phase 6 — 有意延期，不实现）

目标：训练好的策略（MuJoCo 内核）在 Gazebo 仿真中复现 reach 行为，作为 sim2real 前的中间验证。

## 待办清单

1. **模型链路**：MJCF（`libs/so101_envs/assets/`）↔ URDF（`src/so101_description/urdf/so101.urdf`）
   的残余差异清单化（夹爪摩擦、actuator kp/kv、接触参数），评估对策略行为的影响
2. **控制链路**：策略输出 [-1,1] 位置目标 → gz_ros2_control 的 position command interface；
   验证 gz 舵机模型（`so101_sim` 的 gz variant）与 mujoco `<position>` actuator 的等价性
3. **观测重建**：gz 侧按同序构造 obs（qpos/joint_states、qvel 差分、TCP 由 TF/FK、
   goal/item 位姿由 gz topic），与训练 obs 分布对比（均值/方差表）
4. **step 频率**：gz 实时因子 vs mujoco 500Hz 内环——策略 10Hz 控制在 gz 侧的等价节拍
5. **验收标准**：gz 中 reach success（与 mujoco 同一判定语义）≥ 训练 eval 的 70%
6. **风险预案**：若 gz 中失效，逐项做 obs 分布对齐 + 域随机化补强（颜色/摩擦/相机噪声）

## 现状

- 不阻塞 rl 交付：训练/推理均在 mujoco 内核闭环
- 入口已就绪：`scripts/rl_eval.py`、`src/so101_rl_infer`（策略执行与可视化与 gz 无关）
