# CLAUDE.md

SO-ARM101 机械臂 ROS2 Jazzy 工作区（uv + colcon 混合管理）。

## 常用命令

```bash
# 环境（shell 是 zsh，不要 source setup.bash）
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh

# 构建
colcon build --symlink-install

# Python 测试（对齐验证, 9 tests）
uv run pytest

# URDF↔MuJoCo 对齐验证 CLI（输出 reports/alignment_report.md）
uv run python scripts/verify_model_alignment.py

# 启动 demo（三种仿真 + 可选 MoveIt）
ros2 launch so101_bringup demo.launch.py sim:=none|gazebo|mujoco [moveit:=true] [rviz:=false] [gz_gui:=false]

# 清理残留进程（每轮测试前必做, 防止旧进程向 /robot_description 发布污染）
bash scripts/clean_ros.sh
```

## 结构速览

- `models/` — 上游模型副本（对齐验证脚本用）；`src/so101_sim/mujoco/` 是仿真实际加载的 MJCF（默认 scene.xml 含地板）
- `src/so101_description` — URDF（根为 world link）+ meshes + rviz 配置
- `src/so101_sim` — ros2_control 三硬件宏（mock/gz/mujoco）+ controllers.yaml
- `src/so101_bringup` — demo.launch.py（OpaqueFunction 按 sim 分支）
- `src/so101_moveit_config` — MoveItConfigsBuilder + SRDF/kinematics/joint_limits/ompl
- `tests/` + `scripts/verify_model_alignment.py` — URDF↔XML 对齐验证（TDD）
- `.claude/tdd/setup.tdd.md` — TDD 证据报告；`plan/setup.md` — 原始计划

## 关键约定与坑（详见 skill: ros2-jazzy-sim-integration-pitfalls）

- **所有 git commit 必须先征得用户同意，不得擅自提交**
- 系统 pinocchio 按 NumPy 1.x 编译：venv 必须 `--system-site-packages` + `numpy<2`
- pytest 已在 pyproject addopts 禁用 ROS 的 launch_testing 插件，勿删
- URDF 根是 `world`（fixed joint 固定 base_link），SRDF 因此没有 virtual_joint
- MoveIt 2.12 参数名 `planning_plugins`（复数）；配置经 MoveItConfigsBuilder 生成
- gz 内嵌 controller_manager 需要 `/clock` bridge（demo.launch.py 已加）
- zsh 环境：`source /opt/ros/jazzy/setup.zsh`（setup.bash 会报错）
- mujoco headless 需 `export MUJOCO_GL=egl`
