# Plan: SO101 Gymnasium 环境集（mujoco 内核 + ros2/rviz 可视化）— reach/lift/place/stack，供 RL/IL 使用

**日期**: 2026-10-21
**状态**: 待确认
**复杂度**: Large（约 12~16 小时）
**参考**: `~/Third/Projects/github/so101/squint`（ManiSkill/SAPIEN 实现，任务定义/奖励/相机思路来源）

## 需求重述

1. 参考 squint 项目，在本工作区新增 **so101 envs** 项目：gymnasium 兼容环境集，
   **供后续 RL 与 IL 训练使用**。
2. 使用 **MuJoCo** 物理内核，实现 **4 个环境**：`reach` / `lift` / `place` / `stack`
   （对齐 squint 任务集合；物体默认 cube，物体类型参数化留作扩展）。
3. 提供 **无 camera 与 camera 两个版本**：camera 版本支持 **固定视角（fixed，
   第三人称）** 与 **夹爪视角（wrist，跟随 gripper）** 两种。
4. **纯 MuJoCo 内核**（已确认）：env 内部直接 `mujoco.step()`，不经 ros2_control。
   ROS2/RViz 仅承担**可视化**（独立 viz 节点），后续部署桥接口另立。
5. 动作空间为**关节位置目标**（已确认）：Box(6)，5 臂关节 + 1 夹爪，最终以
   **位置控制**（MJCF `<position>` actuator）执行。
6. **不使用 `libs/so101_robotics`**（仅供 torque 学习）。需要 IK 时（脚本专家策略、
   未来任务空间接口）用 **pinocchio**（系统 site-packages 已装，venv 已配
   `--system-site-packages` + `numpy<2`）。
7. 创建 git 分支 **`envs`** 开发。

## 环境事实（已勘察确认）

| 项 | 状态 |
|---|---|
| 现有 MJCF | `src/so101_sim/mujoco/so101_new_calib.xml` 全部为 `<position class="sts3215">` actuator，forcerange=±3.35 Nm（舵机上限）——env 可直接复用位置 actuator 思路 |
| 上游副本 | `models/so101_new_calib.xml` + `scripts/verify_model_alignment.py`（9 tests）已建立 URDF↔XML 对齐验证模式 |
| Pinocchio | `ros-jazzy-pinocchio`（系统 site-packages，venv 已配好） |
| squint 结构 | `envs/base_random_env.py`（ThirdCamera/WristCamera 两相机基类 + DR config）、`envs/robot/so101.py`（TCP=双指 tip 中点、is_grasping 按接触力+方向角）、4 任务 env（evaluate/dense_reward 各自实现） |
| squint 相机 | third: look_at [0.6,0.3,0.3]→[0.3,0,0.05] 每步加噪；wrist: 挂 gripper_link，基座偏移 (-0.005,0.050,-0.059)、欧拉 (-90°,91°,-35.31°)、FOV 71° |
| squint 控制 | `pd_joint_target_delta_pos`（Δ±0.1 rad），sim_freq=100/control_freq=10；rest pose keyframe `start` |
| squint 任务注册 | `SO101ReachCube-v1`(50步) / `SO101LiftCube-v1`(50步) / Place / Stack，`spawn_box_pos=[0.3,0]` 半径 0.1 |
| pytest | pyproject 已禁 ROS launch 插件；testpaths=`tests/` |
| MUJOCO_GL | headless 渲染需 `export MUJOCO_GL=egl`（现有约定） |

## 架构决策（已与用户确认）

| 决策 | 结论 | 理由 |
|---|---|---|
| ROS2 角色 | 纯 MuJoCo 内核 + 独立 ROS2/RViz 可视化节点 | RL 训练需几十~几百 step/s，ros2_control 在环（10-50ms/步）不可行 |
| 动作空间 | 关节位置目标 Box(6)，归一化到 [-1,1]（按关节限位） | 与 lerobot/SO-101 真机遥操作格式一致，适合 IL；无 IK 依赖 |
| 四个环境 | reach / lift / place / stack | 对齐 squint；物体类型（cube/can）留参数扩展 |

## 环境命名与注册（8 个 id）

相机版本是**整体开关**：带相机的环境**同时含固定视角与夹爪视角两个相机**
（用户要求：相机要么不带，要么两个都有），id 以 `Camera` 标志区分：

| 任务 | 无相机（state obs） | 带相机（rgb，fixed + wrist 双相机） |
|---|---|---|
| reach | `SO101Reach-v1` | `SO101ReachCamera-v1` |
| lift | `SO101Lift-v1` | `SO101LiftCamera-v1` |
| place | `SO101Place-v1` | `SO101PlaceCamera-v1` |
| stack | `SO101Stack-v1` | `SO101StackCamera-v1` |

- 无相机 id：obs 为扁平 state 向量（`Box`）
- `*Camera-v1`：obs 为 `Dict{"state": vector, "rgb": {"fixed": HxWx3, "wrist": HxWx3}}`，
  fixed = 世界系第三人称，wrist = 跟随 gripper
- 内部类只有一个（如 `SO101ReachEnv(camera=False)`），注册示例：

  ```python
  gymnasium.register(id="SO101ReachCamera-v1",
                     entry_point="so101_envs.reach:SO101ReachEnv",
                     kwargs={"camera": True})
  ```

## 目录结构（目标）

```
so101/
├── libs/so101_envs/                  # ★ 纯 Python gymnasium 库（无 ROS 依赖，uv wheel 安装）
│   ├── __init__.py                   # gymnasium.register 8 个 id：4 任务 × 无相机/Camera 变体
│   │                                 #   （见下方「环境命名与注册」表）
│   ├── assets/
│   │   ├── so101_env.xml             # 机器人（从 so101_new_calib.xml 派生 + wrist 相机 site）
│   │   └── scene_reach.xml / scene_lift.xml / scene_place.xml / scene_stack.xml
│   │                                 #   （include 机器人 + 桌面 + 任务物体，mujoco 坐标系）
│   ├── base_env.py                   # SO101BaseEnv(gymnasium.Env)：mujoco 内核、500Hz 内环、
│   │                                 #   位置 PD、obs/action space、reset/step/render/close
│   ├── cameras.py                    # 相机组（成对启用）：fixed（世界系固定 look_at）+
│   │                                 #   wrist（挂 gripper_link site）；Renderer 离屏渲染（egl）
│   ├── randomize.py                  # RandomizationConfig dataclass：出生域/初始qpos噪声/
│   │                                 #   物体尺寸摩擦/颜色光照（可开关）
│   ├── ik.py                         # pinocchio 数值 IK（脚本专家/未来 EE 接口用，env 核心不用）
│   ├── utils.py                      # TCP（双指 tip 中点）、grasp 判定（接触力+方向角）、
│   │                                 #   quat 工具、MJCF/URDF 常量
│   └── reach.py / lift.py / place.py / stack.py   # 各任务：场景加载/episode 重置/
│                                                  #   evaluate()/compute_dense_reward()
├── src/so101_envs_viz/               # ★ ROS2 ament_python 可视化包
│   ├── so101_envs_viz/
│   │   └── viz_node.py               # 跑 env → 发布 /joint_states、物体/相机 TF、相机图像
│   ├── launch/env_viz.launch.py      # env:=reach|lift|place|stack camera:=false|true rviz:=true
│   ├── rviz/env_viz.rviz             # 复用 so101_description 机器人 + Image 显示
│   └── config/…                      # 可选：固定视角参数等
├── tests/
│   ├── test_envs_api.py              # gymnasium.utils.env_checker.check_env（8 个注册 id 全查）
│   ├── test_envs_spaces.py           # obs/action space 形状（state 与 rgb）
│   ├── test_envs_tasks.py            # 脚本策略验收：IK reach 达标、lift 抓起、place 入筐、stack 成功
│   └── test_envs_align.py            # env MJCF ↔ URDF 关节名/限位/origin 对齐（镜像 verify_model_alignment）
└── reports/envs_report.md            # 验收报告（scripts/viz + 数据）
```

**依赖变更**：`pyproject.toml` 增 `gymnasium>=1.0`；hatch wheel packages 增
`libs/so101_envs`。ROS 侧 `src/so101_envs_viz` 由 colcon 管理（同其他 src 包）。

## 实施阶段

### Phase 0：分支 + MJCF 资产 + 对齐测试（TDD）
- **Action**: `git checkout -b envs`；编写 `libs/so101_envs/assets/`：
  - `so101_env.xml`：从 `src/so101_sim/mujoco/so101_new_calib.xml` 派生——去掉 ros2_control
    相关 name 依赖（保留即可）、加 `gripper_site`/`finger_tip sites`（TCP 与接触检测用）、
    `wrist_cam` site（挂 gripper_link，squint 同款偏移/朝向）
  - 4 个 scene XML：桌面（桌子边缘贴机器人基座，squint table_pose 同款几何）+ 任务物体
    （reach: cube+goal site；lift: cube；place: cube+bin；stack: cubeA+cubeB）+ 地板 + 灯光
  - 位置 actuator 沿用 sts3215 class（forcerange ±3.35），kp/kd 初值取现有 XML，后续调参
- **测试先行**: `test_envs_align.py`（关节名顺序、限位、TCP site 存在性）
- **Mirror**: `models/` 上游副本 + `verify_model_alignment.py` 模式
- **Validate**: `uv run pytest tests/test_envs_align.py`

### Phase 1：base_env + reach（gymnasium 骨架贯通）
- **Action**: `base_env.py`：
  - `gymnasium.Env`，`action_space=Box(-1,1,(6,))`（归一化关节位置目标，按 qlimits 反归一化；
    预留 `action_mode: absolute|delta`，默认 absolute）
  - 每控制步 = 20 个 mujoco 子步（sim 500Hz / control 25Hz；位置 actuator 即内环 PD）
  - obs（state 模式）：`qpos(6) qvel(6) tcp_pose(7) tcp_vel(3) + 任务专属项`（拼接为 Box 向量，
    同时提供 `obs_dict` 结构化版本）
  - `render_mode="rgb_array"` 支持；`reset(seed=…)` 严格播种（DR 可复现）
  - `reach.py`：继承 base，实现出生随机化、goal site、evaluate（TCP 距 goal≤2cm & 静止）、
    dense reward（1−tanh(5d) 镜像 squint）
- **测试先行**: `test_envs_api.py`（env_checker）+ `test_envs_spaces.py`（reach 部分）
- **Mirror**: squint `reach.py` 奖励/成功条件；workspace pytest 约定
- **Validate**: `uv run pytest tests/test_envs_api.py tests/test_envs_spaces.py`

### Phase 2：lift / place / stack
- **Action**: 三个任务 env：
  - `utils.py`：`is_grasping`（双指-物体接触力范数 ≥ 阈值 + 力方向与指面法向夹角判定，
    squint 同款）、`is_static`（qvel 阈值）
  - `lift`：grasped + 物体抬高 + 返回 rest pose 附近 → success；奖励 = reach + grasp + 回位
  - `place`：cube + bin（box 几何），success = 物体 xy 在 bin 内 & 未被抓 & 静止
  - `stack`：cubeA 红放 cubeB 蓝上，success = A 静置于 B 上 & 未被抓 & 双静止
- **测试先行**: `test_envs_tasks.py` 先写构造性用例（直接 set 物体/机械臂状态验证 evaluate 与 reward 符号）
- **Mirror**: squint `lift.py`/`place.py`/`stack.py`
- **Validate**: `uv run pytest tests/test_envs_tasks.py`

### Phase 3：camera 版本 + 域随机化
- **Action**:
  - `cameras.py`：`camera: bool` 开关（默认 False），开启后**成对启用** fixed + wrist 两相机
    - fixed：世界系固定 look_at 相机（squint third 视角，128×128 可配），支持每步位姿噪声
    - wrist：读取 `wrist_cam` site（mujoco cam 天然跟随 body，挂 site 即可）
    - obs 变为 dict：`{"state": vector, "rgb": {"fixed": …, "wrist": …}}`（HxWx3 uint8）；
      space 用 `gymnasium.spaces.Dict`
  - `__init__.py` 注册 8 个 id（4 任务 × 无相机/`Camera`，见命名表）；
    无相机 id 在 Phase 1/2 先行注册，`*Camera` id 在本 Phase 补齐
  - `randomize.py`：RandomizationConfig（初始 qpos 噪声、出生域、物体尺寸/摩擦/颜色、光照），
    默认关（除出生域与 qpos 噪声），RL 后续按需开
- **测试先行**: `test_envs_spaces.py` 扩展带相机 id 的 obs 形状 + 渲染尺寸
- **Mirror**: squint `base_random_env.py` 的 ThirdCamera/WristCamera 参数
- **Validate**: `uv run pytest tests/test_envs_spaces.py`（headless: `MUJOCO_GL=egl`）

### Phase 4：pinocchio IK + 脚本专家验收（4 env 端到端）
- **Action**: `ik.py`：pinocchio 数值 IK（DLS/伪逆，关节限位裁剪）——仅服务于脚本专家
  与未来任务空间接口；`scripts/run_expert.py`：对每个 env 跑脚本策略（reach 用 IK 到 goal，
  lift/place/stack 用 IK + 开合夹爪状态机），记录 success rate
- **测试先行**: `test_envs_tasks.py` 增 IK 冒烟用例（给定可达位姿，IK 收敛且 FK 复核）
- **Mirror**: `libs/so101_robotics/ik.py` 的算法思路（只参考算法，不 import）
- **Validate**: `uv run python scripts/run_expert.py --env all --episodes 10`
  （目标：reach ≥ 90%，lift/place/stack > 0% 且稳定复现）
- **调参点**: 位置 actuator kp/kd、夹爪摩擦（gripper links friction≈2.0，squint 同款）、
  接触 condim——写入 `reports/envs_report.md`

### Phase 5：ROS2/RViz 可视化包
- **Action**: `src/so101_envs_viz`（ament_python）：
  - `viz_node.py`：运行 env（指定 env/camera），发布 `/joint_states`、物体与相机 TF
    （`tf2_ros`）、`sensor_msgs/Image`（fixed/wrist 相机）；时钟用 wall time
  - `env_viz.launch.py`：OpaqueFunction 模式，参数 `env` `camera` `rviz`；
    启动 robot_state_publisher（so101_description URDF，注意 URDF 根是 world link 约定）+ viz_node + RViz
- **Mirror**: `so101_bringup/demo.launch.py`（OpaqueFunction 分支模式）、`scripts/clean_ros.sh` 测试前清残留
- **Validate**: `bash scripts/clean_ros.sh && ros2 launch so101_envs_viz env_viz.launch.py env:=reach camera:=fixed`
  （RViz 中机械臂与相机图像同步显示）

### Phase 6：文档 + 验收报告
- **Action**: `reports/envs_report.md`（env_checker 结果、脚本专家成功率、调参记录、
  每 env step 耗时基准）；CLAUDE.md 增补 envs 常用命令与约定；README/docs 增补
- **Validate**: `uv run pytest`（全套）+ `colcon build --symlink-install` 通过

## Pattern Grounding（镜像现有模式）

| 类别 | 来源 | 模式 |
|---|---|---|
| 库结构 | `libs/so101_robotics/` | 纯 Python 无 ROS 库 + uv wheel + sys.path 自举（viz 节点同法导入 so101_envs） |
| 模型对齐 | `scripts/verify_model_alignment.py` | URDF↔MJCF 关节/限位对齐测试（Phase 0 TDD） |
| 任务定义 | `squint/envs/reach.py` 等 | evaluate()/compute_dense_reward() 分离；spawn_box 出生随机化 |
| 相机 | `squint/envs/base_random_env.py` | third/wrist 基座位姿与噪声参数直接沿用 |
| grasp 判定 | `squint/envs/robot/so101.py` | 双指接触力 + 方向角；TCP=双指 tip 中点 |
| Launch | `so101_bringup/demo.launch.py` | OpaqueFunction 按参数分支 |
| 测试 | `tests/` + pytest | addopts 已禁 ROS 插件；TDD 先红后绿 |

## 依赖与兼容

- 新增 pip：`gymnasium>=1.0`（其余 mujoco/numpy/pinocchio 均已就绪）
- venv 约束不变：`--system-site-packages` + `numpy<2`（pinocchio 依赖）
- headless 渲染：`MUJOCO_GL=egl`（训练脚本与测试内自动 setdefault）
- 不触碰 `libs/so101_robotics`、`so101_robotics_torque_app`、torque 模型链路
- main 分支不受影响：所有改动在 `envs` 分支，完成后按用户节奏合并

## Risks

| 风险 | 可能性 | 缓解 |
|---|---|---|
| 位置 actuator kp/kd 不当导致抖动/抓取失败 | 高 | Phase 4 集中调参；kp 初值沿用现有 XML；force limit ±3.35 保守 |
| mujoco 抓取稳定性差（接触参数敏感） | 中 | 夹爪 friction=2.0 + condim=3 起步；grasp 阈值放宽（squint min_force=0.5 宽松版）；必要时加接触参数调整 |
| wrist 相机视角标定与真机不一致 | 中 | site 位姿直接抄 squint 实测值；viz 节点里可视化复核；参数外置可调 |
| IK 冗余分支翻转/大段跳动（项目已知坑） | 中 | IK 仅用于脚本专家；Q 温启动 + 关节限位裁剪 + 步长限制 |
| RViz TF 与 mujoco 坐标系不一致 | 中 | viz 节点统一从 env 读取位姿；基座放置几何对齐 squint table_pose |
| 4 env obs 维度/空间定义漂移导致 RL 侧返工 | 低 | env_checker + spaces 测试锁定；`__init__.py` 集中注册与文档化 |
| colcon 与 uv 混装（so101_envs 装两份） | 低 | viz 节点仅依赖 uv 安装的 so101_envs；package.xml 不声明 python 依赖冲突 |

## Estimated Complexity: LARGE

- Phase 0: 2~3h；Phase 1: 2~3h；Phase 2: 3~4h；Phase 3: 2h；Phase 4: 2~3h；Phase 5: 2h；Phase 6: 1h

## Acceptance

- [ ] `envs` 分支上 8 个注册 id 全部通过 `gymnasium.utils.env_checker.check_env`
- [ ] `*Camera-v1` 同时输出 fixed + wrist 双视角 RGB（128×128），无相机 id 为纯 state obs
- [ ] 脚本专家（pinocchio IK）在 reach 达成 ≥90% success，lift/place/stack 稳定成功
- [ ] `uv run pytest` 全绿（新增 ~4 个测试文件）
- [ ] `ros2 launch so101_envs_viz env_viz.launch.py` RViz 同步可视化
- [ ] 未 import `libs/so101_robotics`（grep 验证）；未提交未授权 commit
- [ ] `reports/envs_report.md` 记录调参与基准数据
