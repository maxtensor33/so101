# Todo: sim2real_twin — 数字孪生物体同步

> 状态: 待办（2026-11-04 由 sim_to_real 方向拆出）。主要消费者是 [[rl_real_deploy_todo]]（state-only RL 真机部署需要 item_pose 真值）；il_sim 实机部署是视觉策略，**不需要**此项。

## 需求

实际人先摆放物体，然后仿真同步物体与机器人的位姿信息——仿真成为真机场景的"数字孪生"，为真机策略提供观测真值。

## 现状

- `src/so101_sim/so101_sim/mujoco_twin.py` 已有: 订阅 `/joint_states` 渲染**机器人**（`real:=true sim:=mujoco [twin:=false]` 已接线），**不含物体**。
- RL 任务段 obs（如 lift 的 `item_pose(7)`）在真机上无感知来源，孪生同步是根治方案（另一路是 rgb+state 重训，见 [[rl_image_based_todo]]）。

## 设计草案

1. **物体位姿进仿真**: twin 扩展——订阅物体位姿话题（或交互式指定/标定板量取），写 MuJoCo 物体 body 的 qpos/free joint。
2. **发布观测真值**: twin 发布 `/twin/item_pose`（geometry_msgs/Pose），rl_real_node 缺省参数时订阅之（接口已在设计草案预留）。
3. **人工标定流程**: 首期工具——手摆物体后量取 xyz+朝向填参数；进阶——AprilTag/棋盘格标定板 + 固定 top 相机自动求位姿。
4. **一致性保证**: 仿真物体尺寸/材质与真机实物对齐（与 squint 的 3D 打印同款思路一致）。

## 依赖

- 仅在 RL 实机部署启动时有消费者；纯 IL（视觉策略）路线可无限期搁置。
