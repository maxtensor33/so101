# TDD 证据报告: MTC 纯 RViz 模式（plan/mtc_rviz.md）

日期：2026-10-23 · 分支：mtc · 运行器：`uv run pytest` + `colcon build`

## 源计划与旅程

`plan/mtc_rviz.md`（用户已确认）。journeys：
- 作为操作者，我要一条命令在 RViz 里完成 MTC pick-and-place（无物理仿真）。
- 作为操作者，我要文档/命名与 main 一致，双仿真后端暂停可恢复。
- （计划修订）标准 stage 链在 MTC 0.1.8 + 5-dof 上为三会话确认深水区，
  经用户决策复用扁平链——plans are data, 此修订已同步 plan 文档注释与 todo。

## 任务 → 测试 → RED/GREEN

| 任务 | 测试 | RED | GREEN |
|---|---|---|---|
| P0 对齐 main | merge + git mv + 引用同步 | mtc 测试 6 ERROR（FileNotFoundError: 旧脚本名） | mtc 子集 52 passed；全量 319 passed；colcon 10 包 |
| P0 命名对齐 | 既有脚本加载测试 | 同上 | 同上（tests/test_grasp_geometry.py 等引用已同步） |
| P0 双后端暂停 | `todo/mtc_sim_backends_todo.md` 存在性（人工） | — | 文件含产物清单/遗留缺口/恢复条件 |
| P1 标准 stage 链（后取消） | TestRvizNode 4 项 | 4 failed（文件缺失/无 Connect 等） | 实现后 4 passed，但**端到端 plan 不返回** → 经用户决策放弃，测试替换为 TestRvizMode |
| P2 RViz 模式复用扁平链 | TestRvizMode 2 项 | test_rviz_controllers_config 2 次断言失败（注释词误报） | `test_mtc_package.py` 15 passed |
| P2 端到端 | headless 冒烟 | error_code=-3（place_area 碰撞 PREEMPT） | **error_code=1, exit (SUCCESS)** |

## 测试规格

| # | 保证 | 测试/命令 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | rviz 控制器配置无 mujoco 插件、控制器名齐 | test_mtc_package.py::TestRvizMode | 结构 | PASS |
| 2 | launch 含 sim:=rviz 入口且复用扁平链节点 | TestRvizMode::test_launch_rviz_mode_uses_flat_node | 结构 | PASS |
| 3 | 旧 mtc 测试全绿（rename 后引用同步） | `uv run pytest tests/test_mtc_*.py tests/test_scene_pick.py tests/test_attach_semantics.py` | 回归 | PASS (52) |
| 4 | RViz 模式端到端规划+执行成功 | headless 冒烟 `/tmp/mtc_rviz_final.log`: error_code=1 | E2E | PASS |
| 5 | 全量回归 | `uv run pytest`（merge 后） | 回归 | PASS (319，标准链删除后再跑见下) |

## 覆盖与已知缺口

- C++ 侧验证 = colcon build + 端到端冒烟（无 gtest，沿既有约定）
- RViz 画面/Introspection 面板显示待用户图形环境实测
- 标准 stage 链的 5 个已修 bug 与剩余卡点记录于 todo/mtc_sim_backends_todo.md

## Merge evidence

commit 待用户批准。建议单 commit：
`feat: mtc 纯 RViz 模式（sim:=rviz mock 执行）+ 对齐 main + 双仿真暂停`
