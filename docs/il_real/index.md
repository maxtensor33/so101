# il_real：模仿学习（真机遥操作）——待实现

**定位**：真机遥操作采集数据（lerobot SO101 leader-follower）→ 训练 → 真机部署。

## 计划要点（草案）

- 硬件：lerobot SO101 leader（主臂遥操作）+ follower（从臂执行），Sox/lerobot 遥操作栈
- 数据：`LeRobotDataset`（图像 + 关节，帧率与控制频率对齐）
- 训练：ACT / Diffusion Policy（lerobot 管线）
- 部署：`libs/so101_rl/lerobot_deploy.py` 的 `SO101DeployPolicy` 协议
  （rl 方向已定义，真机接线 TODO 见类内文档）

## 前置依赖

- [ ] 真机硬件就绪（leader + follower 舵机套件）
- [ ] il_sim 管线先通（仿真验证训练/评估/部署闭环）
- [ ] sim2real 差异评估（obs 对齐、动作限幅、安全限位）
