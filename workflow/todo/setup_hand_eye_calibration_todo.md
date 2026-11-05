# Todo: setup_hand_eye_calibration — 实机相机手眼标定

> 状态: 待办（2026-11-04 由 il_sim_develop 方向拆出）。触发条件: il_sim 全量模型真机验收前（或任何需要"图像 ↔ 机械臂基坐标系"映射的工作）需要先完成相机标定。

## 需求

实机两路相机（wrist 腕部 + top 顶部）尚未标定——当前安装位姿与仿真 fixed/wrist 相机视角未对齐，视觉策略真机迁移在此之前预期效果差（il_sim_develop 冒烟只验流程不验效果）。需要手眼标定建立相机与机械臂的坐标关系。

## 标定对象

| 相机 | 类型 | 标定内容 |
|---|---|---|
| wrist | 腕部固定（eye-in-hand） | 相机 ↔ 末端法兰外参（随臂运动，需多姿态采集） |
| top | 世界固定（eye-to-hand） | 相机 ↔ 机械臂基座外参（静态一次标定 + 内参） |

## 可选方案（届时决策）

1. **MoveIt Hand-Eye Calibration**（`moveit_calibration_gui`，ROS2 可用性需勘察）——标准 handeye 标定流程，棋盘格/ArUco 目标。
2. **OpenCV calibrateHandEye** 自写脚本——仿照本仓库脚本风格（`scripts/setup_*.py`），采集 N 组姿态 → Tsai/Park 求解。
3. **lerobot 生态工具**——先查 lerobot 是否有现成相机标定流程（`lerobot_find_cameras` 只做设备发现，非标定）。

## 产出（预期）

- 相机内参 + 外参 YAML（放 `src/so101_il_sim_develop/config/` 或独立标定配置）。
- 腕部相机 TF（`wrist_cam_link`）与 top 相机 TF（`top_cam_link`）接入 URDF/静态 TF 发布，RViz 中图像可投影。
- （延伸）真机物体位姿估计能力——为 [[sim2real_twin_todo]] 与 [[rl_real_deploy_todo]] 的 item_pose 感知来源铺路。

## 关联

- [[rl_real_deploy_todo]]（squint 的视觉对齐工程是同一问题的 squint 解法: tune_camera 手工对齐 + 黑幕背景）
- [[sim2real_twin_todo]]（标定后可自动求物体位姿）
- `workflow/plan/il_sim_develop.md`（其相机视野风险缓解项指向本待办）
