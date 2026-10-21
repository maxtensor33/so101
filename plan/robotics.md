# Plan: SO-ARM101 机器人学学习栈（ros2 + mujoco + rviz）— FK/Jacobian/IK/动力学/轨迹/控制 → draw cube

**日期**: 2026-10-20
**状态**: 待确认
**复杂度**: Large（约 12~18 小时）
**参考**: `~/Third/Projects/github/mujoco-robotics-lab/lab-2-Ur5e-robotics-lab`（A/B/C 模块化教学结构）

## 需求重述

1. 参考 lab-2-Ur5e 的模块化结构，在本项目新增 so101 的机器人学学习栈：
   **FK → Jacobian → IK → 动力学 → 轨迹生成 → 控制**，最终以 **draw cube**
   （末端画立方体）作为验收 demo。
2. 仿真后端只用 **MuJoCo**；机器人运动须能在 **RViz 中同步可视化**。
3. 模型文件新增**力矩控制版本**：`xxx_torque` 命名（`so101_new_calib_torque.xml`、
   URDF 同理）；原位置控制版本保留为默认。
4. **重点实现 torque 版本**（学习目的）——力矩控制是本计划的核心学习内容。
5. `draw_cube.launch.py` 增加 **`ctrl_mode:=position|torque`** 参数：位置控制与
   力矩控制都可完成 draw cube 验收（位置模式复用现有 JTC 链路作基线对照，
   力矩模式走新链路，两者可对比学习）。

## 环境事实（已勘察确认）

| 项 | 状态 |
|---|---|
| 现有链路 | `demo.launch.py sim:=mujoco` 已通：mujoco_ros2_control + JTC（position 接口）+ rsp→RViz TF |
| MJCF actuator | 当前 `so101_new_calib.xml` 全部为 `<position class="sts3215">`，forcerange=±3.35 Nm（舵机上限） |
| ros2_control 接口 | `so101.ros2_control.xacro` 当前只声明 `command_interface: position` |
| Pinocchio | `ros-jazzy-pinocchio`（系统 site-packages，venv `--system-site-packages` + numpy<2 已配好） |
| MuJoCo Python | uv venv 内 3.x（对齐验证脚本已在用） |
| 对齐验证 | `scripts/verify_model_alignment.py` 9/9 PASS，FK max_pos=0.0025mm |
| 参考 lab 结构 | `src/a1..a5`（基础）+ `b1..b3`（轨迹/控制/约束）+ `c1..c3`（集成/ROS 桥/draw cube）+ `ur5e_common.py`/`mujoco_sim.py` 共享层 |
| lab draw cube 做法 | DLS IK + quintic 关节轨迹 + MuJoCo 位置舵机加重力补偿与速度前馈；**cube 是轨迹目标（画线），不是抓取实体** |

## 目录结构（目标）

```
so101/
├── libs/                            # ★ 非 ROS 纯 Python/C++ 库目录（uv 管理）
│   │                                #   后续 rl/、il/ 等不需要 ros2 的库与它并列
│   └── so101_robotics/
│       ├── __init__.py
│       ├── common.py                # 常量/关节名/限位/模型加载器（对标 ur5e_common.py）
│       ├── fk.py                    # A2: Pinocchio FK + MuJoCo 交叉验证
│       ├── jacobian.py              # A3: 解析/数值 Jacobian、奇异性、manipulability
│       ├── ik.py                    # A4: pseudo-inverse + DLS 数值 IK
│       ├── dynamics.py              # A5: RNEA/CRBA/ABA vs MuJoCo qfrc_bias
│       ├── trajectory.py            # B1: cubic/quintic/trapezoidal/minimum-jerk
│       ├── control.py               # B2: PD+g、computed torque、任务空间阻抗、OSC（核心）
│       ├── constraints.py           # B3: 力矩饱和(±3.35Nm)/关节限位/速度缩放
│       └── sim.py                   # MuJoCo 仿真 wrapper（对标 mujoco_sim.py，纯算法层用）
├── tests/                           # 现有 tests/ 内新增 test_robotics_*.py（pytest）
│   ├── test_robotics_fk.py
│   ├── test_robotics_jacobian.py
│   ├── test_robotics_ik.py
│   ├── test_robotics_dynamics.py
│   ├── test_robotics_trajectory.py
│   └── test_robotics_control.py
├── src/
│   ├── so101_sim/
│   │   ├── mujoco/
│   │   │   ├── so101_new_calib_torque.xml   # ★ 力矩版 MJCF（motor actuator）
│   │   │   └── scene_torque.xml             # 力矩版场景（地板+灯光，加载 torque 模型）
│   │   ├── config/
│   │   │   └── controllers_torque.yaml      # ★ 力矩链路控制器（forward_command_controller）
│   │   └── urdf/
│   │       └── so101.ros2_control.xacro     # UPDATE: 新增 effort 接口宏 so101_joint_interfaces_effort
│   ├── so101_description/
│   │   └── urdf/so101_torque.urdf           # ★ 力矩版 URDF（描述层副本，含 effort 限位注释）
│   └── so101_robotics_torque_app/           # ★ 新 ROS2 包（对接 so101_robotics 库）
│       ├── package.xml / CMakeLists.txt
│       ├── launch/draw_cube.launch.py       # ★ 验收入口, ctrl_mode:=position|torque
│       ├── rviz/draw_cube.rviz              # ★ RViz 配置统一放包内 rviz/ 子目录
│       └── so101_robotics_torque_app/
│           ├── torque_control_node.py       # 力矩控制器节点（τ = g(q) + PD + 前馈）
│           └── draw_cube_node.py            # C3: cube 航点 → IK → quintic → 下发
└── models/                                  # 对齐验证脚本用的副本
    ├── so101_new_calib_torque.xml
    └── so101_torque.urdf
```

说明：
- `libs/so101_robotics/` 放顶层 **libs/** 目录（不放 `src/`），后续 rl、il 等
  不需要 ros2 的 python/cpp 库同样放 `libs/` 下并列；ROS 应用包统一在 `src/` 下。
  算法库不依赖 rclpy，可独立 `uv run pytest` 测试。
- 根 `pyproject.toml` 增加包配置使 `so101_robotics` 可被 uv venv 导入
  （如 hatch 的 `packages = ["libs/so101_robotics"]` 或 setuptools `package-dir`），
  ROS 节点经同一 venv `import so101_robotics`，不走 colcon Python 打包。
- 位置控制版本不新增文件（demo.launch.py 已覆盖）；仅在 xacro 内**并存**两套
  接口宏，`ctrl_mode` 参数选择。

## 实施阶段

### Phase 0：力矩模型文件（xxx_torque）+ 对齐验证
1. `so101_new_calib_torque.xml`：复制位置版 MJCF，actuator 由
   `<position class="sts3215" .../>` 改为 `<motor class="sts3215" .../>`
   （gear=1，ctrl 单位 = Nm，保留 `forcerange="±3.35"`；`kv` 项删除）。
   `models/` 放验证副本。
2. `scene_torque.xml`：含地板/灯光的完整场景，引用 torque 模型。
3. `so101_torque.urdf`：描述层副本。URDF 本身无 actuator 概念，差异点为
   effort 限位注释 + 独立文件命名（用户要求 xxx_torque 命名）。
4. `so101.ros2_control.xacro` 新增 `so101_joint_interfaces_effort` 宏
   （`command_interface: effort` + position/velocity state），新增
   `so101_ros2_control_mujoco_torque` 宏（`mujoco_model_path` 指向
   `scene_torque.xml`）。
5. `controllers_torque.yaml`：`forward_command_controller/ForwardCommandController`
   （effort 接口）×2（arm 5 关节 + gripper）+ joint_state_broadcaster。
6. 扩展 `verify_model_alignment.py`（或加 `--torque` 开关）验证 torque MJCF 与
   torque URDF 的关节/限位/FK 对齐（运动学层应 100% 一致，仅 actuator 不同）。

**验证**: `uv run pytest`（新增 torque 对齐测试）；`uv run python scripts/verify_model_alignment.py --torque` 全 PASS。

### Phase 1：`libs/so101_robotics` 库骨架 + FK + Jacobian（对标 A2/A3）
1. 根 `pyproject.toml` 增加包配置（hatch `packages = ["libs/so101_robotics"]` 或
   setuptools 等价写法），确认 `uv run python -c "import so101_robotics"` 可导入。
2. `common.py`：关节名列表、home 位姿、限位表（从 `models/so101_torque.urdf` 读取）、
   Pinocchio/MuJoCo 双模型加载器（单例缓存，复用对齐脚本的映射表经验）。
3. `fk.py`：`pinocchio_fk(q)` / `mujoco_fk(q)` / 交叉验证函数（阈值 1mm/0.5°）。
4. `jacobian.py`：Pinocchio `computeFrameJacobian`（几何 Jacobian）、数值 Jacobian
   （有限差分对比）、奇异性指标（σ_min）、manipulability w² = det(JJᵀ)。
   注意末端 frame 选择：用 `gripper` frame（与对齐脚本一致）。

**验证**: `uv run pytest tests/test_robotics_fk.py tests/test_robotics_jacobian.py`。

### Phase 2：IK + 动力学（对标 A4/A5，力矩学习重点之一）
1. `ik.py`：pseudo-inverse IK（`q += J⁺e`）+ 自适应阻尼 DLS（`Jᵀ(JJᵀ+λ²I)⁻¹`，
   λ 随 σ_min 自适应）；随机可达位姿基准测试（waypoint 误差阈值 1mm 内——
   so101 工作空间小，先采样再收敛）。
2. `dynamics.py`：Pinocchio `rnea`/`crba`/`aba` 与 MuJoCo `mj_inverse`/
   `qfrc_bias`/`mj_forward` 交叉验证（M/C/g 对比阈值 1e-3 相对误差）；
   输出重力向量 g(q) 供控制层使用。

**验证**: pytest 对应文件全过。

### Phase 3：轨迹生成 + 控制器族（对标 B1/B2，torque 学习核心）
1. `trajectory.py`：多段 cubic/quintic（零速边界）、梯形速度、minimum-jerk；
   关节空间 + 笛卡尔空间（笛卡尔轨迹经 IK 转关节）。
2. `control.py`（按教学梯度逐个实现，全部输出力矩 τ）：
   - **PD+g**（重力补偿 + PD）：`τ = g(q) + Kp(q_d−q) + Kd(q̇_d−q̇)`
   - **computed torque**（反馈线性化）：`τ = M(q)(q̈_d + Kp e + Kd ė) + C(q,q̇)q̇ + g(q)`
   - **任务空间阻抗**：`F = M_d ẍ_d + Kp(x_d−x) + Kd(ẋ_d−ẋ)` → `τ = JᵀF + g(q)`
   - **OSC**（操作空间控制，可选进阶）
   - 每个控制器接 `constraints.py` 的力矩饱和（±3.35 Nm）+ 关节限位软约束。
3. `sim.py`：纯算法层 MuJoCo wrapper（`step`/`set_ctrl`/`get_state`），
   使 A/B 模块可脱离 ROS 独立仿真验证——这是**学习闭环**：控制器→仿真→误差曲线。

**验证**: pytest 控制器收敛测试（PD+g 阶跃响应收敛、computed torque 跟踪
quintic 轨迹 RMS < 阈值）；matplotlib 出对比图（生成物放 `reports/`）。

### Phase 4：ROS2 集成（so101_robotics_torque_app）
1. 新包 `src/so101_robotics_torque_app`（ament_python），依赖 `so101_robotics`
   （根 pyproject 已配置 `libs/` 包路径，节点经同一 uv venv `import so101_robotics`，
   不走 colcon Python 打包——项目已是 uv+colcon 混合模式）。
2. `torque_control_node.py`：
   - 订阅轨迹目标（`/torque_command` 或 follow_joint_trajectory 风格话题）
   - 100 Hz 控制环：读 `/joint_states` → 算 `τ`（PD+g / computed torque 可切）
   - 发布到 `forward_command_controller` 的 `commands` 话题（effort 接口）
3. `draw_cube.launch.py`（OpaqueFunction，镜像 demo.launch.py 结构），新增
   **`ctrl_mode:=position|torque`** 参数（默认 `torque`）：
   - `ctrl_mode:=torque`：mujoco_ros2_control 加载 `scene_torque.xml` +
     torque xacro（effort 宏）+ `controllers_torque.yaml`（forward_command_controller）
     + torque_control_node + draw_cube_node
   - `ctrl_mode:=position`：加载现有 `scene.xml` + position 宏 +
     现有 `controllers.yaml`（JTC），draw_cube_node 直接向 arm_controller 的
     follow_joint_trajectory 下发轨迹（复用 demo.launch.py 已验证链路作基线）
   - 两种模式共用：rsp（TF→RViz）+ RViz（rviz/draw_cube.rviz）+ draw_cube_node
4. **RViz 同步可视化**：机器人 TF 已由 rsp→/joint_states 链路解决（现有机制）；
   cube 目标与末端轨迹用 **Marker**（LINE_LIST 立方体棱 + TRAJECTORY 实际末端
   轨迹线）由 draw_cube_node 发布——cube 是画线目标，无需 mujoco 实体同步。

**验证**: `colcon build` 通过；`ros2 launch so101_robotics_torque_app draw_cube.launch.py`
冒烟：3 控制器 active、/joint_states 有数据、RViz 显示机器人 + 立方体线框。

### Phase 5：draw cube 验收（对标 C3）
1. `draw_cube_node.py` 流水线：10cm 立方体 12 条边 → 每边 N 航点 → DLS IK
   （Phase 2）→ quintic 关节轨迹（Phase 3）→ 按执行端下发：
   - `ctrl_mode:=torque`（默认）：轨迹点发 torque_control_node 执行，控制器可
     `controller:=pd_g|computed_torque` 切换对比（学习核心）
   - `ctrl_mode:=position`：轨迹点经 JTC `follow_joint_trajectory` action 下发
     （基线对照，复用现有链路）
2. 节点内实时统计末端 RMS/max 误差（FK 还原 vs 期望航点），结束后打印 +
   写 `reports/draw_cube_report.md`；RViz Marker 显示期望线框 vs 实际轨迹。
3. 验收标准（初始值，实机标定后可调）：
   - 12 条边全部完成，中断率 0
   - 末端 RMS 误差 < 5mm（so101 为 hobby 舵机臂，非 UR5e 工业级；力矩上限
     3.35Nm + armature 较大，先给宽松阈值，跑通后收紧）
   - 执行过程力矩全部处于 ±3.35 Nm 饱和内（无 clipping 告警）

**验证**: 完整跑一遍 draw_cube.launch.py，reports/ 出报告，肉眼确认 RViz 中
轨迹贴着立方体线框。

## Pattern Grounding（镜像现有模式）

| 类别 | 来源 | 模式 |
|---|---|---|
| 命名 | `plan/setup.md`、参考 lab `src/a2_*.py` | 模块小写蛇形 + 阶段编号注释；模型文件 `xxx_torque` 后缀 |
| 计划风格 | `plan/setup.md` | 需求重述 → 环境事实 → 结构 → Phase（含验证）→ 风险 |
| ROS 节点/launch | `src/so101_bringup/launch/demo.launch.py:29` | OpaqueFunction + TimerAction 延迟 spawner/rviz |
| 测试 | `tests/test_alignment.py` | pytest + conftest（venv `--system-site-packages`）；参考 lab 用 unittest，本项目**改用 pytest** 对齐现状 |
| FK 交叉验证 | `scripts/verify_model_alignment.py` | Pinocchio vs MuJoCo 数值对比（非参数 diff）|
| 错误处理 | 对齐脚本 FAIL→列表输出 | 验证器收集全部偏差再统一报告 |

## 依赖与兼容

- `so101_robotics` 依赖：`numpy<2`（已有）、`pinocchio`（系统包）、`mujoco`（venv）；
  `scipy`（quintic/min-jerk 可手写，不强需）；matplotlib（报告图，加入 pyproject）。
- ROS 侧新增依赖：`forward_command_controller`（ros2_controllers 自带）。
- 无新系统包需要安装（除非 Phase 4 发现 mujoco_ros2_control 不支持 effort，见风险）。

## Risks

| 风险 | 等级 | 缓解 |
|---|---|---|
| **mujoco_ros2_control 是否支持 effort command interface 未验证** | ~~HIGH~~ → **已消解（2026-10-20 静态 spike）**：`/opt/ros/jazzy/include/mujoco_ros2_control/data.hpp:97` 存在 `effort_interface` + `is_effort_control_enabled`，`.so` 符号含 `joint_command_to_actuator_command`；line 427 "effort not supported" 仅指 JointState override API。运行时联调（Phase 4）仍需实测确认 ctrl 映射正确 |
| so101 舵机力矩小（3.35Nm）+ armature/damping 大，阻抗/OSC 易不稳定 | MEDIUM | 学习梯度从 PD+g → computed torque 起步（模型基座补偿后对参数误差鲁棒）；OSC 标记可选 |
| mujoco 更新率 100Hz 对力矩控制偏低（舵机动力学瞬态） | MEDIUM | controllers_torque.yaml 调 update_rate 至 200~500 实测；仍不足则调小 mjcf timestep |
| 数值 IK 在 so101 小工作空间内陷入局部解/越限 | LOW | 先采样可达位姿再求解；限位钳位 + 多起点重试 |
| Pinocchio URDF 与 MJCF 的 gripper 关节方向差异（历史坑，已在对齐脚本处理） | LOW | 直接复用 `verify_model_alignment.py` 的映射与验证逻辑 |
| RViz 无 mujoco 实体 cube 同步通道 | LOW | 设计上 cube 为画线目标，Marker 展示即可；若后续要实体抓取再议 |

## Estimated Complexity: LARGE

- Phase 0: 2~3h（模型转换 + 验证扩展 + effort spike）
- Phase 1: 2h（fk/jacobian）
- Phase 2: 2h（ik/dynamics）
- Phase 3: 3~4h（trajectory/4 类控制器/constraints/sim wrapper + 收敛测试）
- Phase 4: 3h（ROS 包 + launch + RViz 配置）
- Phase 5: 2h（draw cube 节点 + 调参 + 报告）

## Acceptance

- [ ] `models/` 与 `src/` 下存在 `*_torque` 模型文件，`--torque` 对齐验证全 PASS
- [ ] `uv run pytest` 全过（原 9 + 新增 robotics 测试）
- [ ] `colcon build` 4+1 包全过
- [ ] `ros2 launch so101_robotics_torque_app draw_cube.launch.py` 一条命令完成：
      mujoco 仿真 + 力矩控制器 + RViz 同步显示机器人 + 立方体线框 + 轨迹
- [ ] `ctrl_mode:=position` 同一脚本可跑通（JTC 基线），两种模式误差可对比
- [ ] `reports/draw_cube_report.md`：RMS < 5mm、无力矩饱和告警
- [ ] 力矩模式下控制器可切换（pd_g / computed_torque）且行为可对比（学习价值）
- [ ] 所有 git commit 经用户批准
