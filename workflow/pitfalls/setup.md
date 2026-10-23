# setup / 模型对齐

| 症状 | 原因与处理 |
|---|---|
| gz 中机械臂倾倒 | URDF 缺 world 固定关节（本项目已修，勿删 `world_joint`）；URDF 根是 `world`（fixed joint 固定 base_link），SRDF 因此没有 virtual_joint |
| 对齐验证失败/漂移 | 位置版与力矩版（`*_torque`）分别验证：`uv run python scripts/setup_verify_model_alignment.py [--torque]`，报告在 `workflow/reports/alignment_report*.md` |
| 力矩版 ctrl 语义 | `*_torque` MJCF 的 ctrl 即关节力矩（±3.35 Nm = STS3215 上限），位置版勿混用 |
| MoveIt 配置 | 经 MoveItConfigsBuilder 生成；SRDF 无 virtual_joint（见上） |
