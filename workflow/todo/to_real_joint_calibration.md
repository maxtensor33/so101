# Todo: to real 的关节校准（逐关节真机限位标定）

**状态**: 待办 · **创建**: 2026-10-29 · **来源**: setup_hardware 实机联调（旧标定误用 + lift 限位手动放宽）

## 背景

- 实机映射（`so101_system.cpp` / `setup_capture_named_pose.py`）采用全行程线性映射，
  前提是"标定量程 = URDF/MJCF 限位行程"。2026-10-29 误用过不完整标定
  （`so101_follower/R12552801.json`，4 月旧档）导致角度整体拉伸，已由
  `setup_convert_lerobot_calibration.py` 的行程体检兜底（欠扫告警）。
- `shoulder_lift` 限位目前是**手动估计 ±1.91986（±110°）**（对齐 menagerie
  trs_so_arm100 的物理行程建模），原 onshape 导出值 ±1.74533（±100°）。
  同步改动位置（恢复或修正时全链 6 处一起改）：
  1. `models/so101_new_calib.xml`（joint range + actuator ctrlrange）
  2. `models/so101_new_calib.urdf`
  3. `src/so101_sim/mujoco/so101_new_calib.xml`（joint range + ctrlrange）
  4. `src/so101_description/urdf/so101.urdf`
  5. `src/so101_hardware/src/so101_system.cpp`（so101JointLimits）
  6. `scripts/setup_capture_named_pose.py`（JOINT_LIMITS）+
     `scripts/setup_convert_lerobot_calibration.py`（JOINT_TRAVEL_DEG）
- 其余 5 个关节的软限位（尤其 elbow/wrist_roll/gripper 与参考项目有微差）同样
  **未对真机逐关节验证**。

## 待办步骤

1. [ ] 力矩失能逐关节测量物理极限（`setup_capture_named_pose.py --monitor` + 手动掰到两端，记录 raw 值）
2. [ ] 把每关节 raw 端点经标定换算成 URDF rad，与现有限位对比（重点 lift/elbow/wrist_roll/gripper）
3. [ ] 修正上表 6 处限位（如与 ±1.91986 估计不符），同步更新 `workflow/pitfalls/setup-hardware.md`
4. [ ] 方向校验：`--monitor` + `ros2 run so101_sim mujoco_twin.py` 逐关节确认同向；
       反向关节在标定转换加 `--invert <关节>`
5. [ ] MoveIt 端到端验收（pitfalls/setup-hardware.md H7 清单），在
       `workflow/reports/setup_hardware.md` 追加真机验收节

## 验收标准

- [ ] 每关节 raw 端点 ↔ URDF 限位端点对应关系有实测记录（写入本文件或 pitfalls）
- [ ] 标定跨度体检（convert 脚本）全关节无告警
- [ ] MoveIt 命名姿势真机执行与 RViz 预期一致
