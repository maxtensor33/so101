# il_real：模仿学习（真机遥操作）

**定位**：真机 leader 臂遥操采集数据（lerobot SO101 leader-follower）→ 训练 →
真机部署。仿真侧管线复用 il_sim（`src/so101_teleop` 协议适配层）。

## 已可用：leader 臂遥操 / 采集（2026-10-29 落地）

真实 leader 臂（主臂）驱动 **仿真** follower（MuJoCo env）——第三条 teleop 通道
（除脚本专家 / 键盘外），可直接录制 LeRobotDataset：

```bash
# 1. 探针：方向/零位/夹爪校验（只读打印; 反向关节加 --invert <关节> 重跑）
uv run python scripts/il_sim_leader_probe.py

# 2. 纯遥操（MuJoCo 窗口 + RViz; 实测 6 关节全部同向, 无需 invert）
uv run python scripts/il_sim_teleop.py --teleop leader --env reach

# 3. 采集
uv run python scripts/il_sim_teleop.py --teleop leader --record --env reach \
  --repo-id il_sim/reach_leader --episodes 10
# 或 draccus 透传（il_sim_record 官方管线, 注册名 so101_leader_sim）
uv run python scripts/il_sim_record.py --robot.type=so101_sim \
  --teleop.type=so101_leader_sim --teleop.port=/dev/ttyACM0 --teleop.id=L07252801 \
  --dataset.repo_id=il_sim/reach_leader --dataset.num_episodes=10
```

- 注册名 `so101_leader_sim`（包装 lerobot `SO101Leader`，度/0-100 → 统一 [-1,1]）
- 动作平滑（每步限幅 0.15 / 夹爪 0.25）防 10Hz 手驱噪声破坏抓取保持
- 训练沿用 il_sim：`scripts/il_sim_train.py --repo-id ...`

## 真机部署（前置依赖）

- [x] leader 遥操采集链路（上节）
- [x] 真机硬件控制插件（见 {doc}`/hardware/index`：标定转换、`real:=true`、数字孪生）
- [ ] follower 真机执行 policy（sim2real：obs 对齐、动作限幅、安全限位）
- [ ] 部署接口接线：`libs/so101_rl/lerobot_deploy.py` 的 `SO101DeployPolicy` 协议
  （rl 方向已定义，真机接线 TODO 见类内文档）
- [ ] 训练：ACT / Diffusion Policy（lerobot 管线，同 il_sim）

## 参考

- 计划：`workflow/plan/il_sim_leader.md`
- 验收报告：`workflow/reports/il_sim_leader_report.md`（含探针方向表与真机实测修复）
- TDD 证据：`workflow/tdd/il_sim_leader.tdd.md`
