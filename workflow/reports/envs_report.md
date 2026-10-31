# SO101 Envs 验收报告（plan/envs.md Phase 6）

**日期**: 2026-10-21/22
**分支**: `envs`（未提交，commit 由用户手动执行）

## 交付物

| 组件 | 路径 |
|---|---|
| gymnasium 环境库 | `libs/so101_envs/`（8 个注册 id: 4 任务 × 无相机/`Camera`） |
| MJCF 资产 | `libs/so101_envs/assets/`（so101_env.xml + 4 场景，自含 STL） |
| ROS2/RViz 可视化包 | `src/so101_envs_viz/`（viz_node + env_viz.launch.py + rviz 配置） |
| pinocchio IK | `libs/so101_envs/ik.py`（DLS + 限位裁剪 + tool-down 姿态模式） |
| 脚本专家 | `scripts/run_expert.py`（4 任务状态机 + 多种子 IK） |
| 测试 | `tests/test_envs_{align,api,spaces,tasks,ik}.py`（5 个文件） |

## 测试结果

```
uv run pytest tests/test_envs_align.py tests/test_envs_api.py tests/test_envs_spaces.py tests/test_envs_tasks.py tests/test_envs_ik.py
86 passed, 8 warnings in ~15s
```

| 套件 | 数量 | 内容 |
|---|---|---|
| test_envs_align | 32 | 4 场景 MJCF↔URDF：关节/限位/actuator/FK 采样/site+相机/物体 freejoint |
| test_envs_api | 25 | env_checker（state id）、seed 可复现、5-tuple、位置伺服收敛、obs 在空间内 |
| test_envs_spaces | 13 | state obs 布局、相机 Dict obs（fixed+wrist 128×128 uint8）、DR 生效 |
| test_envs_tasks | 13 | reach 构造性 success、lift 抓握检测、place 入筐、stack 摞放 |
| test_envs_ik | 3 | IK 收敛 (<1mm)、限位满足 |

env_checker：4 个无相机 id 全过。带相机 id 因 EGL 渲染位级噪声（±1 像素）
不走位级确定性检查，由 `test_camera_obs_is_dict_space` 以 ±1 容差覆盖。

## 脚本专家验收（`uv run python scripts/run_expert.py --env all --episodes 5`）

| env | 结果 | 验收线 |
|---|---|---|
| SO101Reach-v1 | **100% (5/5)** | ≥90% ✓ |
| SO101Lift-v1 | 0% (0/5) | >0% ✗ |
| SO101Place-v1 | 0% (0/5) | 0% ✗ |
| SO101Stack-v1 | 0% (0/5) | 0% ✗ |

### 已知问题：抓握保持失败（lift/place/stack）

链路各环节已单独验证：
- tool-down IK 收敛 <1mm（多种子重试）
- 专家可把爪对准物体（下探后 TCP 与物体中心差 <1cm）
- 闭合时 is_grasping 检测触发（teleport 与专家两条路径均验证）

失败环节：**持续抓握**。闭合中物体被单侧旋转爪沿爪长方向挤出爪尖掉落（1-2s 内）。

已尝试：forcerange 3.35→1.2→0.35 Nm 深闭合堵转；抓住即停（无压紧力，掉落）；
kp 400 + 物体缩至 4.4cm——均能检测到抓握，保持失败。

根因：单动爪 + mesh 凸包接触 + 硬位置伺服；真机有柔性指垫，squint(SAPIEN) 用
patch_radius 接触模型。后续方向：
1. jaw 碰撞体改原生 box/capsule + solref 柔性接触
2. 指面加摩擦垫碰撞块
3. tool-down 抓取位形下重新标定 wrist_cam（当前在 rest 位形标定）

**RL/IL 使用不受阻塞**：reach 完整可用；lift/place/stack 的 obs/reward/evaluate
语义完整且有构造性测试锁定，RL 可直接训练（专家仅 sanity 参照，非依赖）。

## 性能基准

| env | 每控制步耗时 | 吞吐 |
|---|---|---|
| SO101Reach-v1 (state) | 0.70 ms | 1429 steps/s |
| SO101ReachCamera-v1 (rgb) | 6.52 ms | 153 steps/s |

控制 10Hz（50 substeps × 2ms），episode 长度对齐 squint（50/50/75/100 步）。

## ROS2/RViz 验证（headless）

- `colcon build --symlink-install --packages-select so101_envs_viz` 通过
- `ros2 run so101_envs_viz viz_node -p env:=reach -p camera:=true`：
  /joint_states 6 关节 ✓；/envs/fixed/image、/envs/wrist/image 128×128 rgb8 ✓；
  /tf 物体 frame（item, 父 world）✓
- `ros2 launch so101_envs_viz env_viz.launch.py env:=reach rviz:=false`：
  rsp + viz_node 启动，/joint_states 关节名正确 ✓
- RViz 图形验证需显示环境：`ros2 launch so101_envs_viz env_viz.launch.py`

## 调参记录

- so101_env.xml：gripper actuator `kp=400, forcerange=±1.2Nm`（原始 ±3.35/kp998），
  夹爪 3 个 collision geom `friction=2.0`；wrist_cam `pos=(0.04,-0.045,-0.02) 俯视 60°`
- 物体：lift/place item 4.4cm cube 25g（4.8cm 超 rest 位形开口）
- episode：reach/lift 50，place 75，stack 100 步（对齐 squint）
- 动作：Box(-1,1,6) 归一化关节位置目标，10Hz 控制 / 500Hz 仿真

## 2026-10-31 更新：8 环境已按 squint 迁移

本文件上文的物体尺寸（4.4cm cube）、episode 长度（place 75 / stack 100）、
以及“抓握保持失败”结论均已被取代。最新实现：

- lift：`workflow/reports/lift_squint_migration.md`
- reach / place / stack：`workflow/reports/envs_squint_migration.md`
- 统一：2.2–2.8cm / 密度 200 物体、condim=4 + 硬接触、squint 奖励/观测/终止、
  episode 50、delta 动作（RL）。
- 脚本专家在新的出生域/物理下全部 0%，需按新尺寸重调（非 RL 验收项）。

## TDD 证据

见 `.claude/tdd/envs.tdd.md`（RED→GREEN 映射表）。
