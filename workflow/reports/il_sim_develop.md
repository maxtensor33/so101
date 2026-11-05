# 验收报告: il_sim_develop（plan/il_sim_develop.md）

日期：2026-11-04 · 状态: 仿真/mock 部分完成, 真机项待用户执行

## 已完成（本会话, 证据见 workflow/tdd/il_sim_develop.tdd.md）

- [x] Phase 0: 基线构建/回归全绿；冒烟 ACT checkpoint 验证可加载
- [x] Phase 2: `src/so101_il_sim_develop` 包 TDD（RED→GREEN, 22 契约/单元测试；全量 459 passed）
- [x] Phase 3（mock 旁路）: 伪 joint_states + 伪双相机驱动真实节点加载冒烟 ACT → 3s 发出 28 条 JTC 轨迹；estop 排空后 0 条（SMOKE PASS）
- [x] Phase 1 预研: lerobot 0.6.1 `--robot.type so101_follower` 确认可用
- [x] Phase 6 部分: tdd/reports/pitfalls/CLAUDE.md 工件

## 真机 checklist（用户执行）

- [ ] **Phase 1 lerobot-rollout 冒烟**（零新代码, 用 lerobot 自带 SO101Follower）:
  ```bash
  uv run lerobot-rollout \
    --strategy.type=base \
    --policy.path=outputs/il_sim/act_reach_expert/checkpoints/000010/pretrained_model \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 --robot.id=my_follower_arm \
    --robot.cameras="{ fixed: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: /dev/video1, width: 640, height: 480, fps: 30}}" \
    --task="grab the cube" --duration=30
  ```
  注意: 相机 key 必须与训练数据集一致（fixed/wrist）；冒烟模型动作无意义属预期，只验"相机读取→推理→臂动"闭环；首跑人手扶急停。
- [ ] **Phase 3（孪生旁路, 可选 GUI）**: `bash scripts/clean_ros.sh && ros2 launch so101_il_sim_develop il_sim_develop.launch.py checkpoint:=... confirm_step:=true rviz:=true`（real:=true 由 launch 内置；无相机时节点会持续 warn 缺帧属预期）
- [ ] **Phase 4 双相机 RViz**: `ros2 launch so101_il_sim_develop il_sim_develop.launch.py checkpoint:=... rviz:=true`，RViz 两个 Image 面板 ≥10Hz
- [ ] **Phase 5 真机 rollout**: confirm_step:=true 起步逐单步 → 连续；全量模型（他机训练）回来后换 checkpoint 重跑
- 安全: 首跑人手扶急停、`ros2 topic pub --once /il_sim_develop/estop std_msgs/msg/Empty` 实测停发、gripper 力矩确认
- 相机未标定（视觉先不对齐）: 全量模型验收前先做 `workflow/todo/setup_hand_eye_calibration_todo.md`
