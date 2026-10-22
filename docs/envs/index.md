# envs：gymnasium 环境集（mujoco 内核）

**定位**：供后续 RL/IL 训练使用的标准 gymnasium 环境。纯 MuJoCo 内核
（不经 ros2_control），位置控制动作空间，与 `libs/so101_robotics`（力矩教学）解耦。

## 环境 id（8 个）

| 任务 | 无相机（state obs） | 带相机（rgb，fixed+wrist 双相机成对） |
|---|---|---|
| reach | `SO101Reach-v1` | `SO101ReachCamera-v1` |
| lift | `SO101Lift-v1` | `SO101LiftCamera-v1` |
| place | `SO101Place-v1` | `SO101PlaceCamera-v1` |
| stack | `SO101Stack-v1` | `SO101StackCamera-v1` |

- 无相机 obs：扁平 state 向量（25/29/33/35 维，qpos/qvel/TCP/任务专属）。
- 带相机 obs：`Dict{"state", "rgb": {"fixed"(第三人称), "wrist"(跟随夹爪)}}`，
  128×128 uint8。
- 10Hz 控制 / 500Hz 仿真；动作 Box(-1,1,6) 归一化关节位置目标。

## 命令

```bash
# 冒烟
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1'); env.reset(); env.step([0.0]*6)"
# 单环境验证（--render 存 /tmp 双视角 png）
uv run python scripts/envs_check_env.py --id SO101Lift-v1 [--steps 30] [--render]
# 脚本专家验收（reach 100%; lift/place/stack 抓取保持为已知问题）
uv run python scripts/envs_run_expert.py --env all --episodes 5
# mujoco viewer 交互窗口（C 键循环 free/fixed/wrist）
uv run python scripts/envs_view_env.py --id SO101Reach-v1 [--cam fixed] [--random]
# gymnasium 标准 human 渲染
uv run python -c "import gymnasium as gym, so101_envs; env = gym.make('SO101Reach-v1', render_mode='human'); env.reset(seed=0); [env.step([0.0]*6) for _ in range(100)]; env.close()"
# RViz 可视化
ros2 launch so101_envs_viz env_viz.launch.py [env:=reach] [camera:=false] [rviz:=true]
```

## 体系

```
libs/so101_envs (assets 自含 MJCF+STL, pinocchio IK)
  ├─ base_env.py + reach/lift/place/stack + cameras + randomize
  └─ ▲ scripts/envs_check_env 等
src/so101_envs_viz ── joint_states/TF/相机图像 ──▶ RViz
```

## 已知问题

- lift/place/stack 抓取保持失败（mesh 凸包 + 硬位置伺服），诊断与调参记录见
  `reports/envs_report.md`——**RL 可训练但成功率预期低**。
