# TDD 证据报告: SO-ARM101 项目搭建

**日期**: 2026-10-20
**源计划**: [plan/setup.md](../../plan/setup.md)
**TDD 循环**: RED `479e301` → GREEN `7f148f2` → refactor（合入 GREEN 提交）→ Phase 3/4 `ac9369b`

## 用户旅程（源自计划）

1. 作为机器人开发者，我要验证 URDF 与 MuJoCo XML 运动学对齐，以便 MoveIt/Pinocchio 规划与仿真行为一致。
2. 作为开发者，我要一条 launch 命令启动 RViz（sim:=none/gazebo/mujoco 可选）。
3. 作为开发者，我要 demo 中可选启用 MoveIt 规划。

## 任务报告

### Phase 1: git + uv 仓库初始化
- **执行摘要**: uv venv（--system-site-packages）+ pyproject；发现系统 pinocchio 以 NumPy 1.x 编译，固定 `numpy<2` 修复段错误。
- **验证**: `uv run python -c "import pinocchio, mujoco"` → pinocchio 4.1.0 / mujoco 3.16.0 / numpy 1.26.4
- **保证**: 验证脚本运行环境可复现（uv.lock 入库）。

### Phase 2: URDF ↔ MuJoCo XML 对齐验证（核心 TDD 循环）
- **执行摘要**: 先写 9 个 pytest（tests/test_alignment.py），RED（ModuleNotFoundError），再实现 scripts/verify_model_alignment.py 转 GREEN，重构移除死代码。
- **验证命令**: `uv run pytest`（9 passed, cov 88%）；CLI 生成 reports/alignment_report.md
- **RED 证据**:
  ```
  E   ModuleNotFoundError: No module named 'verify_model_alignment'
  ```
- **GREEN 证据**:
  ```
  9 passed in 0.36s
  scripts/verify_model_alignment.py     160     19    88%
  ```
- **CLI 结果**:
  ```
  [PASS] joint_list: 6 joints aligned
  [PASS] joint_limits: max |d| = 0.000004 rad
  [PASS] inertials: max |dm| = 0.000000 kg
  [PASS] forward_kinematics: n_samples=100 max_pos=0.0025 mm, max_rot=0.0009 deg
  ```
- **保证**: URDF 与 MuJoCo XML 在关节清单/限位/惯性/随机 100 组配置 FK 上对齐（平移 <1mm，旋转 <0.5° 阈值内）。

### Phase 3/4: ROS2 四包 + 统一 launch（运行时冒烟证据）
- **执行摘要**: 四个 ament_cmake 包；demo.launch.py 以 OpaqueFunction 按 sim 三选一；修复 ParameterValue 包裹、MoveIt 2.12 `planning_plugins`（复数）参数名、URDF/SRDF robot name 统一、gz 插件类名 `gz_ros2_control::GazeboSimROS2ControlPlugin`、GZ_SIM_RESOURCE_PATH。
- **保证**: 三种仿真后端 + MoveIt 规划可用，一条命令启动。

## 测试规范

| # | 保证内容 | 测试文件/命令 | 类型 | 结果 | 证据 |
|---|---|---|---|---|---|
| 1 | 关节名/顺序两模型一致 | `tests/test_alignment.py::TestJointList` | unit | PASS | `uv run pytest` |
| 2 | 限位偏差 ≤1e-3 rad | `::TestJointLimits` | unit | PASS | 同上 |
| 3 | 质量/质心一致 | `::TestInertials` | unit | PASS | 同上 |
| 4 | 100 组随机配置 FK 对齐（决定性） | `::TestForwardKinematics` | integration | PASS | 同上 |
| 5 | 报告生成含 PASS/FAIL 表格 | `::TestReport` | unit | PASS | 同上 |
| 6 | sim:=none 三控制器 active + /joint_states | `/tmp/smoke_none.sh` | runtime smoke | PASS | list_controllers 3 active |
| 7 | sim:=mujoco 物理仿真数据 | `/tmp/smoke_mujoco.sh` | runtime smoke | PASS | /joint_states 含重力微偏移 |
| 8 | sim:=gazebo spawn + 控制器 | `/tmp/smoke_gz.sh` | runtime smoke | PASS | Entity creation successful + 3 active |
| 9 | MoveIt 规划成功 | `/tmp/plan_and_check.sh` | E2E | PASS | `PLAN_RESULT error_code=1 points=5` |

## 覆盖率与已知缺口

- **覆盖率**: `verify_model_alignment.py` 88%（≥80% 达标）。未覆盖：argparse CLI main() 与异常路径。
- **已知缺口**（有意留白，后续迭代补）:
  - RViz GUI 显示为人工目视项，headless 环境无自动化断言
  - Gazebo/MuJoCo 模式下的轨迹执行（action 下发）未测，仅 none 模式规划
  - `gz_gui:=true` 桌面 GUI 模式未自动化验证

## 合并证据

TDD 检查点提交均在 main 分支 HEAD 可达：
`ed91096` (init) → `479e301` (RED) → `7f148f2` (GREEN) → `ac9369b` (Phase 3/4)。
无 squash；RED/GREEN/refactor 证据保留于各提交信息中。
