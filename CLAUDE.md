# CLAUDE.md

SO-ARM101 机械臂 ROS2 Jazzy 工作区（uv + colcon 混合管理）。

完整命令示例、参数注释与 launch 开关见 **docs/commands.md**（按方向分节）；
方向文档 `docs/<方向>/`；原始计划 `workflow/plan/`。

## 核心命令速查

```bash
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh   # shell 是 zsh, 别 source setup.bash
colcon build --symlink-install
bash scripts/test.sh                                  # pytest 统一入口（无参数=全量, 只回流汇总行）
bash scripts/clean_ros.sh                             # 每轮测试前清残留 ROS 进程（必做）
uv run python scripts/setup_verify_model_alignment.py # URDF↔MuJoCo 对齐验证 CLI
ros2 launch so101_bringup demo.launch.py sim:=none|gazebo|mujoco [moveit:=true]
```

## 结构速览

- `models/` — 上游模型副本（对齐验证用）；`src/so101_sim/mujoco/` 是仿真实际加载的 MJCF（含地板）
- `src/so101_description` — URDF（根为 world link）+ meshes + rviz 配置
- `src/so101_sim` — ros2_control 三硬件宏（mock/gz/mujoco）+ controllers.yaml
- `src/so101_bringup` — demo.launch.py（OpaqueFunction 按 sim 分支）
- `src/so101_moveit_config` — MoveIt 配置（SRDF/kinematics/joint_limits/ompl）
- `libs/so101_robotics` — 无 ROS 学习库：fk/jacobian/ik/dynamics/trajectory/control/constraints/sim（uv wheel 安装；ROS 节点 sys.path 自举）
- `libs/so101_envs` — 无 ROS gymnasium 环境库：MuJoCo 内核 + 位置控制 + fixed/wrist 双相机 + pinocchio IK；相机标定默认开启（camera_calib.py，见 docs/commands.md envs 节）
- `libs/so101_rl` — RL 库：TrainerSpec 统一 sb3/skrl；checkpoint = model.{zip|pt}+meta.json(+vecnormalize.pkl)；torch 走 uv 直装（pyproject find-links/清华源）
- `src/so101_envs_viz` — envs RViz 可视化：/joint_states、物体 TF、/envs/{fixed,wrist}/image
- `src/so101_rl_infer` — RL 推理节点：checkpoint 驱动 env + ROS 话题
- `src/so101_teleop` — lerobot Robot/Teleoperator 适配层：SimSO101Robot（so101_envs 后端）+ expert/keyboard/leader teleop + ros_viz（CLI 经 scripts/il_sim_*.py）
- `src/so101_il_sim_infer` — IL 推理包：checkpoint 加载（policy/pre/post）+ env 驱动 + ROS 话题
- `src/so101_il_sim_develop` — 实机 IL 部署开发：obs 桥 + estop + homing + confirm_step + 内置 cv2 相机（复用 il_sim_infer.core；`il_real` 命名空间留给正式实机包）
- `src/so101_mtc_pick_place` — MTC pick-and-place：stage 链节点 + kinematic attach 插件（sim:=rviz, mujoco/gz 暂停见 workflow/todo/）
- `src/so101_hardware` — 实机 ros2_control 插件：标定 YAML 驱动电机 ID/量程，rad↔raw 全行程线性映射，无零位要求
- `src/so101_robotics_torque_app` — 力矩 app：torque_control_node + draw_cube_node
- `tests/` — 全项目测试（TDD）
- `workflow/` — 流程工件（约定见 workflow/README.md）：`plan/` 各方向计划、`tdd/` TDD 证据、`reports/` 验收报告、`todo/` 暂停待办、`learn/` 复盘、`problems/` 跨会话调试交接、`pitfalls/` 按方向坑（单源进 docs 站）、`reviews/` 评审
- `workflow/reports/` — 对齐验证报告分位置版（alignment_report.md）与力矩版（alignment_report_torque.md）

## Token 节约纪律

- 测试一律走 `bash scripts/test.sh [pytest 参数]`（无参数=全量；全文在 /tmp/so101_test/pytest.log，只回流汇总行）；改动期只跑受影响的测试文件，全量仅收尾跑一次；**禁止反复 tail/grep 试探输出格式后重跑**
- 预期输出 >50 行的命令（构建/测试/launch）重定向到 /tmp/so101_test/ 并只回流 grep 汇总行
- >200 行的文件先 grep 定位再分段 read（命中点前后 ~40 行），禁止整读
- repo 级搜索（跨包调用点/注册处）交给 explore 子代理，主会话只收结论
- 编辑用 edit 替换时 oldString 从最近一次 read 精确复制，不凭记忆重写

## 关键约定与坑

- **所有 git commit 必须先征得用户同意，不得擅自提交**
- 其余约定与坑**按方向分类**放在 `workflow/pitfalls/`（索引见其 README）：
  general（环境/colcon/pytest）/ setup（URDF·对齐）/ setup_hardware（实机）/ robotics / mtc / envs / rl / il_sim；
  ros2 集成类另见 skill: ros2-jazzy-sim-integration-pitfalls
- 方向原始计划在 `workflow/plan/`，TDD 证据在 `workflow/tdd/`，暂停待办在 `workflow/todo/`
