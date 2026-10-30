# TDD 证据报告: il_sim_leader（plan/il_sim_leader.md）

日期：2026-10-29 · 分支：il_sim（快进 main 至 14ba7d7）· 运行器：`uv run pytest`

## 源计划与旅程

`workflow/plan/il_sim_leader.md`（用户确认）。journeys：
- 作为采集员，我要用真实 leader 臂驱动 il_sim 仿真从臂（reach 等）做遥操演示。
- 作为采集员，我要用 leader 臂录制演示数据集（官方 record 管线，≥10 episodes 回读校验）。
- 作为调试者，我要一个只读探针校验 leader→仿真 的方向/零位/夹爪映射。

## 任务 → 测试 → RED/GREEN

| 任务 | 测试 | RED | GREEN |
|---|---|---|---|
| P0 基线 | 全量 | — | 359 passed + 11 包 |
| P1 leader.py | test_il_sim_teleop_leader.py 14 项 | 收集 ERROR（模块不存在）+ 5 failed（stub 缺 connect） | 14 passed；既有 il_sim 25 passed 无回归 |
| P2 CLI 接线 | TestCliWiring 4 项 | —（随 P1 一起 RED） | 18 passed；全量 377 passed |
| P3 探针 | 语法/入口冒烟 | — | `--help` 正常（真机校验由用户执行） |
| P4 第一轮真机反馈 | MuJoCo 接触实验（脚本） | — | 证实静态碰撞正常 → D7 平滑修复; 全量回归见下 | — | 见 report |

## 测试规格

| # | 保证 | 测试/命令 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | 注册名 so101_leader_sim 进 draccus ChoiceRegistry | TestRegistration | 契约 | PASS |
| 2 | 度→[-1,1] 换算数学（中点/四分位/clamp/homing_offset 参与换算） | TestConversion | 单元 | PASS |
| 3 | gripper 0-100→[-1,1] 与 invert | TestConversion | 单元 | PASS |
| 4 | 动作契约：6 键 {m}.pos、[-1,1]、值正确、per-joint invert | TestActionContract（stub inner leader） | 契约 | PASS |
| 5 | 未连接 get_action 报 RuntimeError；is_connected/disconnect 委托 | TestActionContract | 契约 | PASS |
| 6 | 工厂 leader 分支 / 未知类型仍 ValueError | TestCliWiring | 契约 | PASS |
| 7 | leader 采集 reset 策略 = keyboard（auto_reset + 5s，D4） | TestCliWiring | 契约 | PASS |
| 8 | 全量回归（359 基线 + 18 新增） | `uv run pytest` | 回归 | PASS (377) |
| 9 | D7 动作平滑: 首帧吸附/每步限幅 0.15(关节)·0.25(夹爪)/reset 重吸附/可关闭 | TestSmoothing | 单元 | PASS |
| 10 | D2 公式修正: 0.6.1 量程中点制, homing_offset 不参与; 真实量纲 sanity | TestConversion（修正后） | 单元 | PASS |

## 覆盖与已知缺口

- 真机 leader 串口路径未在本会话验证（stub 替代）——探针/遥操/采集为用户执行步骤
- Phase 4 数据集回读校验待真机采集后补录
- 语义转换的 D2 caveat（home≠量程中点偏差）待探针实测记录

## Merge evidence

commit 待用户批准（仓库纪律）。建议两段：
1. `feat: il_sim leader 臂遥操（so101_leader_sim 语义转换 + CLI 接线 + 探针）`
2. `docs(workflow): il_sim_leader plan/tdd/report 工件`
