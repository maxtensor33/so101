# TDD 证据报告: plan/robotics.md — so101 机器人学学习栈

日期: 2026-10-20 ~ 2026-10-21 · 计划: `plan/robotics.md` · 工作流: `/ecc:tdd-workflow`

## Source plan

`plan/robotics.md`（6 Phase，参考 lab-2-Ur5e-robotics-lab）。用户 journey:
1. 作为学习者，我要 FK/Jacobian/IK/动力学/轨迹/控制六模块 + 双引擎交叉验证，
   以便在 so101 上学习机器人学核心算法（torque 版为学习重点）。
2. 作为学习者，我要一条命令在 MuJoCo 力矩仿真中让 so101 画出 6 cm 立方体并在
   RViz 同步可视化，以便端到端验收整条流水线。
3. 作为学习者，我要位置控制基线对照，以便量化 computed torque 的优势。

## Task report

| Phase | 交付 | 验证命令 | 结果 |
|---|---|---|---|
| 0 | `*_torque` MJCF/URDF + effort 接口宏 + `controllers_torque.yaml` + `--torque` 对齐验证 | `uv run pytest tests/test_alignment_torque.py` | 16 tests PASS；对齐报告 `reports/alignment_report_torque.md` 全 PASS（FK 0.0025mm） |
| 1 | `libs/so101_robotics`: fk/jacobian | `uv run pytest tests/test_robotics_fk.py tests/test_robotics_jacobian.py` | PASS；pinocchio↔MuJoCo FK 交叉验证 max 位置误差 < 0.03mm |
| 2 | ik（DLS/自适应阻尼/随机重启）/dynamics（rnea/crba/aba） | `uv run pytest tests/test_robotics_ik.py tests/test_robotics_dynamics.py` | PASS；M(q) 双引擎一致 < 1e-9 |
| 3 | trajectory（quintic/梯形/min-jerk）/control（pd_g/computed torque/阻抗）/constraints/sim | `uv run pytest tests/test_robotics_trajectory.py tests/test_robotics_control.py` | PASS |
| 4 | `so101_robotics_torque_app`（xacro/launch/rviz/torque_control_node/draw_cube_node） | `colcon build` + 冒烟 | PASS；effort 接口实测可用（HIGH 风险消解） |
| 5 | draw cube 验收 + `reports/draw_cube_report.md` + `scripts/draw_cube_viewer.py` | 冒烟 3 次 + position 基线 | torque RMS 3.06~3.18mm（<5mm 达标），position 基线 9.67mm |

## Test specification

| # | 保证 | 测试 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | torque 模型全部 actuator 为纯力矩（motor, ±3.35 Nm, 无位置伺服） | `tests/test_alignment_torque.py::TestTorqueActuators` | unit | PASS |
| 2 | torque 版运动学与位置版 100% 一致 | `::TestTorqueKinematicAlignment` | integration | PASS |
| 3 | effort 接口宏存在且不含 position 命令接口 | `::TestEffortInterfacesXacro` | unit | PASS |
| 4 | controllers yaml 用标量 `interface_name: effort` | `::TestControllersTorqueYaml` | unit | PASS |
| 5 | FK 双引擎一致（随机 100 配态交叉验证） | `tests/test_robotics_fk.py` | unit | PASS |
| 6 | Jacobian 与数值差分一致、奇异性度量正确 | `tests/test_robotics_jacobian.py` | unit | PASS |
| 7 | DLS IK 收敛可达目标、限位钳位、随机重启 | `tests/test_robotics_ik.py` | unit | PASS |
| 8 | rnea/crba/aba 与 MuJoCo 一致（含 armature） | `tests/test_robotics_dynamics.py` | unit | PASS |
| 9 | quintic 边界条件、梯形退化、JointTrajectory 越界抛错 | `tests/test_robotics_trajectory.py` | unit | PASS |
| 10 | 三类控制器输出结构/重力项正确、力矩饱和 | `tests/test_robotics_control.py` | unit | PASS |
| 11 | cube 几何（8 顶点/12 边/度数 3）、航点在边上、默认配置与 ROS 节点一致 | `tests/test_draw_cube_viewer.py` | unit | PASS |
| 12 | 端到端: torque draw cube RMS < 5mm、position 基线对照、RViz 同步 | ROS 冒烟（smoke19/22/23 + rviz run） | E2E | PASS |

RED 证据（抽样）: viewer 测试先于脚本创建，`pytest tests/test_draw_cube_viewer.py`
报 ERROR（ModuleNotFoundError / spec load failure）→ 实现后 12 PASS。

## Coverage

`uv run pytest` → **91 passed**（原对齐 25 + robotics 库 54 + viewer 12）。
库覆盖率 ~99%（此前全量统计）。未覆盖: ROS 节点运行时（launch 文件为拷贝非
symlink，需要 colcon 环境 + 显示服务器，属 E2E 冒烟范畴，见 #12）。

## Merge evidence

Phase 0 完成时用户选择"暂不提交"；**全部工作尚未 commit，待用户批准**。
修复历史与证据保留在本报告与 `reports/draw_cube_report.md`。
