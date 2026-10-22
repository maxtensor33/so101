# il_sim：模仿学习（仿真数据）——待实现

**定位**：用仿真数据（脚本专家 / RL 策略 rollout / 遥操作合成）训练模仿学习策略，
输出与 rl 同构的 checkpoint 供 rl_infer 推理可视化。

## 计划要点（草案）

- 数据格式：lerobot `LeRobotDataset` v2.x（为 il_real 遥操作数据复用做准备）
- 数据来源：
  - 脚本专家（`scripts/envs_run_expert.py`，pinocchio IK 状态机）批量 rollout
  - 训练好的 RL 策略（DAgger 式混合）
  - envs 相机版双视角图像 + state
- 环境：`SO101Lift-v1` / `SO101Place-v1` 优先（**前置：抓取保持物理问题需先解决**，
  见 `reports/envs_report.md`）
- 训练框架候选：lerobot 官方 il 管线（ACT / Diffusion Policy）

## 前置依赖

- [ ] 抓取物理调参收敛（lift/place 成功率达阈值）
- [ ] lerobot 数据格式适配层（rl 方向已有部署协议雏形 `libs/so101_rl/lerobot_deploy.py`）
- [ ] 相机版训练管线验证（rgb obs）

相关待办：`todo/rl_gpu_parallel_todo.md`（视觉训练的 GPU 路线评估）。
