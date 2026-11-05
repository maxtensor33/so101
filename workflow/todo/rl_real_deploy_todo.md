# Todo: rl_real_deploy — RL 实机部署（暂缓）

> 状态: 待办（2026-11-04 由 sim_to_real 方向拆出）。当前定位: RL 仅作 IL 的动作专家（`il_sim_collect_rl.py` 仿真内采集数据），**不部署实机**。触发条件: 需要 RL 直接上真机时启动。

## squint 真机部署调研结论（2026-11-04，保留备用）

squint 走 **ManiSkill `Sim2RealEnv` 换 agent 路线**（`deploy.py` + `deploy_utils/manipulator.py`，已解包 mani_skill 3.0.1 源码验证）:

- **机制**: 保留 sim env 实例，仿真 agent 换成 `LeRobotRealAgent`（lerobot `Robot` → ManiSkill `BaseRealAgent` 适配）。每 tick 真机 rgb **替换**仿真 rgb、真机 qpos **覆写**仿真机器人 qpos，obs 仍由仿真框架计算；**不步进仿真物理**；物体不回灌。
- **观测本质**: image-based（rgb+state 混合）。部署时 state 里的特权特征（`item_pose`/`tcp_to_item`/`is_item_grasped`）指向仿真幽灵物体——`lift.py _initialize_episode` 物体出生位随机化不受 DR 开关控制、deploy 每集 reset 仿真 env，与真机脱节；Sim2RealEnv 文档明示特权特征需用户自建感知管线覆盖 `_get_obs_extra`，squint 未做。**物体定位唯一有效来源是 rgb 通道**；真机上有效的 state 只有本体系（noisy_qpos/控制器目标/dist_rest/tcp 经 sim FK）。
- **视觉对齐工程**（sim2real 成功的真正重心）: 仿真 segmentation 抠物体 + 实拍黑幕替换背景（`black_overlay.png`）、`tune_camera.py` 滑块对齐真机/仿真相机视野、3D 打印同色物体（`deploy_utils/blender_stls`）、真机图像裁方 resize 到 128×128。
- **控制细节**: delta 模式 + `action_scale=0.15` + clip；控制频率建议 ≤15Hz；qpos `sync_read` 5–6ms/次（我们 C++ 插件 100Hz read 无此瓶颈）；**gripper 角度域重映射**（sim -10~120° ↔ 舵机 -62.5~64.62°）+ SO100 elbow +6.8° 标定偏置修正（lerobot 标定读数与仿真角度存在偏置是实测坑）；reset 以 0.025 rad/step @30Hz 插值回 rest；键盘 s/q + Ctrl-C 安全退出；验收路径: 先 reach → `--no-continuous_eval` 逐单步 → 光照要求。

## 本地设计草案（届时展开为 plan）

- **新包 `src/so101_rl_real`**: `rl_real_node` 镜像 `so101_rl_infer/rl_infer_node.py`（checkpoint 加载走 `so101_rl` 同一入口），不持 env——真机执行器替换 env.step。
- **观测桥（state-only）**: qpos 订阅 `/joint_states`，qvel 差分/dt，tcp+quat 用 `so101_robotics.fk`，任务段逐项对齐 `lift.py _task_obs()`；item_pose 来自配置参数或订阅 `/twin/item_pose`（[[sim2real_twin_todo]]）。
- **动作桥 + 安全**: `denormalize_action`（`libs/so101_rl/lerobot_deploy.py` 已有）+ 绝对限幅 + 步进限幅（DELTA_MAX）+ `action_scale` 参数（squint 真机默认 0.15）；10Hz 向 JTC 发单点目标（备选 forward controller）；estop 订阅 + Ctrl-C/reset 插值回 rest。
- **item_pose 真值来源二选一**: 数字孪生物体同步（根治），或重训 rgb+state 版走 squint 原路线（[[rl_image_based_todo]]）。
- **标定偏置坑**: 真机首验逐关节对照仿真角度（重点 gripper 符号），pitfalls 记录。
