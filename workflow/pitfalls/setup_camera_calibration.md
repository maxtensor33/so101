# 坑: setup_camera_calibration（相机标定）

> 占位 stub（计划阶段创建）。实施中踩坑逐条追加。

## 2026-11-05 实施记录

- **OpenCV 4.14 移除 `aruco.calibrateCameraCharuco`**: 新 API 用 `cv2.calibrateCamera` + 棋盘角点物坐标（`board.getChessboardCorners()[ids]`）——双 API 兼容写在 `intrinsics.calibrate_charuco`。
- **`board.generateImage` 返回单通道**: 不要再 cvtColor BGR2GRAY（Bad number of channels）。
- **节点迁移 docstring 吞代码事故**: 用字符串 marker 插入自举代码时把 `"""` 边界吞掉, 自举代码落进 docstring 从未执行, 症状恰似"自举失效"。教训: 对大文件做结构化修改后必须 `ast.parse` + 逐行 head 检查, 且用 `ros2 run` 同路径复现导入问题。
- **uv run / ros2 run 双环境**: 包代码在 uv 侧测试（.venv 完整栈）, ros2 run 走系统 python3 + 自举——两处都要冒烟（同 il_sim_develop 教训）。
- **cv2.calibrateHandEye 约定**: eye-in-hand 输入 (gripper2base, target2cam) 输出 cam2gripper; eye-to-hand 输入 (base2gripper, target2cam) 输出 cam2base。**输出就是相机在父系的位姿（T_gc / T_bc）, 不要再取逆**——详见下节事故。
- **MuJoCo 相机模型**: mjModel.cam_pos/cam_quat 是 body 系相对位姿（fixed_cam 挂 world 即全局; wrist_cam 挂夹爪 body 覆写一次即随动）; 不支持畸变/主点偏移, 内参只对齐 fovy = 2·atan(H/2fy)。

## 2026-11-06 真机标定事故: sim 相机位置/朝向整体反了

**症状**: `compare_views.launch.py` 中真机 fixed 相机在左前, sim 却在右后; wrist 真机朝东北向下, sim 朝西南向上。

**三个独立错误叠加**（任何一个都会导致方向错）:

1. **手眼结果被多取了一次逆**（`handeye_core.solve_handeye`）:
   cv2 返回的 `cam2gripper/cam2base` 已经就是相机在父系的位姿。
   再取逆后 `optical_frame.t` 变成"父系原点在相机系"的坐标, 相机自然跑到反方向。
   验收数据: 修复前 fixed t=(-0.20,-0.23,0.72)（右后）; 修复后 t=(0.306,0.146,0.709)（左前, 与真机一致）。
2. **optical→MuJoCo 相机旋转矩阵乘反了**（`export_envs_camera_calib.py`）:
   光轴系到 MuJoCo 相机系是 `R_mj = R_opt @ diag(1,-1,-1)`（右乘）。
   写成 `diag(1,-1,-1) @ R_opt` 会得到旋转 180° 的朝向; rpy=0 时两者相同, 所以原测试没抓到,
   必须用非对称旋转做回归（新增 `test_export_quat_uses_right_multiplication`）。
3. **wrist 手眼父系 ≠ MuJoCo 相机父 body**:
   手眼结果相对 `gripper_frame_link`(TCP), 而 MJCF `wrist_cam` 挂在 body `gripper`(= URDF `gripper_link`)。
   二者差一个固定关节 `xyz=(-0.0079,-0.000218121,-0.0981274), rpy=(0,π,0)`;
   导出时必须左乘 `T_gripper_link←gripper_frame_link`, 否则 wrist 相机偏移 ~10cm 且绕 y 反 180°。

**顺带修正**:
- `solve_variants` 不再用弱位置先验自动选解; 权威解固定用 TF/solvePnP 文档约定, 变体仅作诊断。
  诊断排序增加"板刚性"（板在样本间应是刚体, 正确解反推的板位姿 std 最小, 实测 9-17mm vs 错误变体 80-600mm）。
- `consistent` 字段改为真正的 PARK/HORAUD 平移差 <1cm, 不再误用物理先验结果。
- eye-in-hand 的静态 TF 父帧改为 `gripper_frame_link`（原来是 world, 只在 fixed 上成立）。
