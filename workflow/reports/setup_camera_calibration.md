# 验收报告: setup_camera_calibration（plan/setup_camera_calibration.md）

日期：2026-11-05（初版）· 2026-11-06 真机标定 + 方向 bug 修复 · 状态: 真机内参/手眼已完成, sim 相机方向已修正

## 已完成（证据见 workflow/tdd/setup_camera_calibration.tdd.md）

- [x] Phase 0: viser 1.1.1（唯一新增依赖, 纯 Python）
- [x] Phase 1: ChArUco 生成器（TDD×8）→ **A4 交付物已产出**:
  - `outputs/calibration/charuco_intrinsic_A4.{png,pdf}`（8×6, 25mm 方格）
  - `outputs/calibration/charuco_handeye_A4.{png,pdf}`（4×5, 15mm 方格, 贴夹爪）
  - 打印要求: 实际尺寸/100%、亚光纸、卡尺校验（25.0mm/15.0mm ±0.2mm）、压平/紧固
- [x] Phase 2: 内参节点迁移（弃 cv_bridge/手动解码/output_dir 参数/自举）; 合成闭环 K 恢复 ±5%; 冒烟 28 角点 + viser HTTP 200
- [x] Phase 3: 手眼双模式（auto 24 姿势 JTC 驱动 / manual 遥操兜底）; PARK/HORAUD 双解 + 一致性; 合成闭环 <5mm; optical→link 换算对齐参考
- [x] Phase 4: envs `camera_calib` 参数（None 回归不变）+ 导出桥接脚本; 覆写后渲染可见差异
- [x] Phase 6: docs/hardware/camera_calibration.md 中文详解 + README 引用 + todo 回链 + CLAUDE.md

## 真机 checklist（2026-11-06）

- [x] 打印两块板并卡尺校验（要求见 docs/hardware/camera_calibration.md 第 3 节）
- [x] 内参标定: fixed + wrist 各一次 → `outputs/calibration/{fixed,wrist}/camera_cal.yaml`
- [x] 手眼标定: eye_in_hand + eye_to_hand 各 20 组样本, PARK/HORAUD 一致
- [x] `export_envs_camera_calib.py` 汇总 → envs `camera_calib`
- [ ] 静态 TF 片段接入 il_sim_develop, RViz 图像-TF 投影对照（待真机 TF 验收）
- [x] 结果数值回填本报告

## 真机标定结果（2026-11-06）

| 项 | 结果 |
|---|---|
| wrist 内参 | fx=683.10, fy=684.12, cx=327.29, cy=255.93（640×480） |
| fixed 内参 | fx=440.66, fy=441.15, cx=316.54, cy=243.63（640×480） |
| eye_in_hand (wrist) | 20 样本, 重投影 0.12–0.41px, PARK/HORAUD 一致 ✓ |
|  相机在 gripper_frame_link | t=(-0.0303, 0.0723, -0.0871) m, rpy=(-27.24, -3.01, -168.81)° |
| eye_to_hand (fixed) | 20 样本, 重投影 0.09–0.28px, PARK/HORAUD 一致 ✓ |
|  相机在 world | t=(0.3057, 0.1459, 0.7088) m, rpy=(-174.27, -4.02, 88.84)° |
| 板刚性（正确解 std） | wrist 9.3mm / fixed 16.7mm（错误输入变体 80–600mm） |

## 2026-11-06 修复: sim 相机左右/上下颠倒

**现象**: `compare_views.launch.py` 真机 fixed 在左前、sim 在右后; wrist 真机朝东北向下、sim 朝西南向上。

**根因与修复**（详见 `workflow/pitfalls/setup_camera_calibration.md`）:
1. `handeye_core.solve_handeye` 对 cv2 输出多取了一次逆 → 不再取逆（cv2 `cam2gripper/cam2base` 本身就是相机在父系位姿）。
2. `export_envs_camera_calib.py` 的 `R_mj = CONV @ R_opt` 乘反 → 改为 `R_opt @ CONV`。
3. wrist 手眼父系 `gripper_frame_link` ≠ MJCF `wrist_cam` 父 body `gripper`(=`gripper_link`) → 导出时左乘固定关节变换（平移 -0.0079/-0.000218/-0.098127；绕 y 180°）。

**验证**:
- 现有 20 组真机样本重求解 + 重导出，`outputs/calibration/envs_camera_calib.yaml` 已更新。
- fixed 相机从 sim 右后移到左前: (0.306, 0.146, 0.709) m; wrist 光轴改为朝下（随夹爪）。
- 渲染对照: `outputs/calibration/compare_sim_calibrated.png`（左上/右上=默认; 左下/右下=标定后）。
- `uv run pytest tests/test_camera_calibration.py` 33 passed; 全量 `uv run pytest` 退出码 0。
- 新增回归: 手眼双向合成闭环（平移+旋转）、变体选择、四元数右乘、TCP 常量 vs URDF、wrist 父帧组合。

**待用户真机确认**: 重新 `ros2 launch so101_camera_calibration compare_views.launch.py`，
在相同 leader 位姿下 sim 与真机视野应逐路一致（fixed 左前俯视、wrist 随臂朝下）。
