# Local Review: plan/robotics.md 全部未提交变更

**Reviewed**: 2026-10-21
**Scope**: 30 个变更文件（8 tracked + 22 untracked），+~2700 行
**Decision**: **APPROVE with comments**（0 CRITICAL / 0 HIGH / 6 MEDIUM / 7 LOW）

## Summary

robotics 学习栈（torque 模型 + libs/so101_robotics + ROS app + viewer 脚本）整体质量良好：
双引擎交叉验证测试、自适应阻尼 IK、错误处理与限位钳位到位，98 tests 全绿。
无安全问题。发现的问题集中在:残留死代码、调试遗留硬编码、仓库卫生与 docstring 过时。

## Findings

### CRITICAL
None

### HIGH
None

### MEDIUM

| # | 位置 | 问题 | 建议 |
|---|---|---|---|
| M1 | `draw_cube.launch.py:31-37,124,136` | `additional_env`/`node_env` 实测不生效（真正生效的是节点内 sys.path 自举），且注释误导 | 删除 node_env 与 additional_env，注释改指 sys.path 自举 |
| M2 | `draw_cube_node.py:284` | `_finish` 写死 `/tmp/draw_cube_errors.csv`（调试遗留），无开关 | 参数化 `csv_out` 参数，默认空禁用 |
| M3 | `torque_control_node.py:36-37,33` | 死常量 KP/KD（已被参数化 self.KP 取代）、unused import `dynamics`、docstring 仍写"纯重力补偿保持"（实际是锁存 q_hold + PD） | 清理 |
| M4 | 仓库卫生 | `.coverage` 被 git 跟踪（本地产物）；`MUJOCO_LOG.TXT` 未 ignore（MuJoCo 警告日志） | `git rm --cached .coverage` + 两者加入 .gitignore |
| M5 | `draw_cube_node.py:235` | `_torque_tick` 用 `hasattr(self, "_hold_started")`；`_finish` 后 0.02s timer 永续空转（靠 None guard 兜底） | 改为实例属性初始化；_finish 时 destroy timer |
| M6 | `draw_cube_node.py:250` | `client.wait_for_server()` 无超时——控制器未起时 executor 永久阻塞 | `wait_for_server(timeout_sec=10.0)` + 失败日志退出 |

### LOW

| # | 位置 | 问题 |
|---|---|---|
| L1 | `dynamics.py:1` | docstring 路径笔误 "plan/robotics力矩.md" |
| L2 | `trajectory.py:73` | `w1 is self.waypoints[-1]` 身份比较脆弱，建议按索引判断 |
| L3 | `control.py:34` / `dynamics.py:64` | `[:6]` 硬编码自由度，模型变更时静默出错 |
| L4 | `sim.py:43,51` | q/dq 属性硬编码 `[:6]`，与 `_qpos_adrs` 按名索引机制不一致 |
| L5 | `ik.py:89-98` | `result[4]` 魔法索引，建议命名元组 |
| L6 | `draw_cube_viewer.py` | `run()` ~110 行（>50 行基准），主线脚本可接受，可拆 sim 循环 |
| L7 | `common.py:60` | `get_pin_data` lru_cache 共享 Data 单例——单线程正确，但 FK/Jacobian/IK 交错复用同一 Data，线程不安全（加注释声明单线程约定即可） |

### 附注（上游行为，不需修改）
- `MUJOCO_LOG.TXT` 显示 mujoco_ros2_control 在 t=0 发出一次 "NaN in CTRL at ACTUATOR 0"——
  上游 `InterfaceData.command_` 初始为 quiet_NaN，控制器 claim 后首帧 write 前未被覆盖。
  实测为一次性瞬态，之后命令正常，无需处理。

## Validation Results

| Check | Result |
|---|---|
| pytest（98 tests） | **Pass** |
| colcon build（5 包） | **Pass** |
| 对齐验证 `--torque`（50 样本） | **Pass**（FK max 0.0025mm） |
| 对齐验证（位置版，50 样本） | **Pass** |

## Files Reviewed

Modified: CLAUDE.md, plan/robotics.md, pyproject.toml, scripts/verify_model_alignment.py,
src/so101_sim/urdf/so101.ros2_control.xacro, tests/conftest.py, uv.lock, .coverage
Added: libs/so101_robotics/(9), src/so101_robotics_torque_app/(全部), scripts/draw_cube_viewer.py,
tests/(8), models/*_torque*(2), src/so101_sim/(3), reports/(2), .claude/tdd/robotics.tdd.md
Junk: MUJOCO_LOG.TXT（建议删除+ignore）
