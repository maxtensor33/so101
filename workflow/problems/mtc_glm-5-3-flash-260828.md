# MTC Pick and Place 调试问题交接 (glm-5.3 会话)

日期: 2026-10-21 · 计划: `plan/mtc.md` · 分支: main · 测试: 127 passed

## 任务背景

按 `plan/mtc.md` 实施 SO-ARM101 的 MTC pick and place（RViz 规划 + mujoco/gz 双后端执行）。
Phase 0~3 已完成，Phase 4 端到端验收卡在 MTC 规划链的最后一个 stage。

## 当前状态（截至交接）

### 已打通 ✅

| 组件 | 说明 |
|---|---|
| 双后端场景 | `src/so101_sim/mujoco/scene_pick.xml`（位置版 + cube）；`src/so101_bringup/worlds/pick_place.sdf`（gz world） |
| 可行摆位（TDD 探测） | `scripts/probe_pick_poses.py`，**"EE frame x 竖直"姿态族**（5-dof 臂唯一可行族，"tool z 朝下"0/8 全灭） |
| 摆位数据 | PICK_CUBE=[0.117,-0.077,0.02] PLACE_CUBE=[0.125,0.063,0.02]（yaw=-33°方位），关节解见下 |
| C++ 包 | `src/so101_mtc_pick_place`（MTC 节点 + mujoco 抓取 plugin，编译通过） |
| 抓取语义 | **kinematic attach**（Python 验证 100%：携带跟随误差 0.0mm、释放正常落物）。物理夹持与 weld equality 都实测不可靠（见下） |
| launch/rviz | `mtc_pick_place.launch.py`（sim:=mujoco\|gazebo）、`rviz/mtc_pick_place.rviz`（MTC 面板） |
| cube 进 planning scene | `known objects: cube` ✓ |
| stage 链 | current state→open gripper→grasp state→close→approach→attach→lift→place 容器全链**各自 1 solution** |

### 唯一卡点 ❌

```
stage 'move to pick':  0 solutions   ← Connect（OMPL，home→抓取位形关节规划）
stage 'move to place': 0 solutions   ← Connect（OMPL，抓取后→放置位形）
```

**独立 OMPL 探针（/plan_kinematic_path 直发 home→grasp 关节约束）在同一个 move_group 上是
SUCCESS（error_code=1, 7 轨迹点）**——OMPL 本身能规划这条路径，但 MTC 的 Connect 拿不到解。

### 根因分析（我的最终判断，未验证完）

**FixedState 的手工 scene 的 ACM 不含 cube 放行**。链路推理：

1. `grasp state`/`place state` 用 `mtc::stages::FixedState`，scene 是手工构造的
   `make_shared<PlanningScene>(robot_model)` —— **它的 ACM 只有 SRDF 默认**（相邻 link disable），
   没有"爪 link ↔ cube 允许"。
2. 探测的抓取位形爪与 cube **有意接触/重叠**（kinematic attach 方案下这是任务语义）。
3. Connect 的 goal 是 pick 容器的 start（approach backward 从 grasp state 推出的 pre-grasp），
   OMPL 的 goal state 碰撞检查用 **该 scene 的 ACM** → 爪-cube 接触被拒 → goal 无效 → 0 解。
4. 佐证：独立探针走 move_group 的 ACM（我在 move_group 层用 `/apply_planning_scene` 放行了
   爪↔cube）→ SUCCESS；MTC Connect 走 FixedState scene 的 ACM → FAIL。
5. 30s timeout 也失败 → 不是时间问题；approach/lift/attach 都 1 solution → 不是位形问题。

**下一步（被中断的修复，未构建）**：在两处 FixedState 的 scene 构造后加：

```cpp
auto & acm = scene->getAllowedCollisionMatrixNonConst();
for (const auto & link : task_.getRobotModel()
     ->getJointModelGroup(gripper_group_)->getLinkModelNamesWithCollisionGeometry()) {
  acm.setEntry(link, "cube", true);
}
acm.setEntry("moving_jaw_so101_v1_link", "cube", true);
```

（两处：`"grasp state"` 与 `"place state"` 的 `stage->setState(scene);` 之前。）
**注意**：`stage->setIgnoreCollisions(true)` 只豁免该 stage 自身，不改变 scene 的 ACM，
所以对 Connect/approach 的检查无效——必须改 scene 的 ACM 本身。

## 已踩过的坑（重要，避免重蹈）

1. **colcon `--symlink-install` 增量编译不生效**（C++）：改了 .cpp 后 `colcon build` 显示
   finished 但二进制没重编（增量判断失效）。**用 `colcon build --packages-select <pkg>
   --cmake-clean-first` 或不加 --symlink-install**。曾因此浪费多轮"修了没变化"。
2. **mujoco_model 是 xacro hardware 参数**：mujoco_ros2_control 从 URDF 的 ros2_control
   tag 读 `mujoco_model`（不是节点参数）。切换场景必须经 xacro 传
   `mujoco_model_path:=<绝对路径>`（`so101.urdf.xacro` 已有该 arg）。
3. **新建 MJCF 后要重建 so101_sim**（install 是拷贝清单，scene_pick.xml 曾因没重建
   而不在 install 里）。
4. **ACM 是整体替换**：`/apply_planning_scene` 带 `allowed_collision_matrix` 会**覆盖**
   整个 ACM——必须先 `/get_planning_scene` 取完整矩阵再追加，否则 SRDF 的
   disable_collisions 全被冲掉，所有规划因"相邻 link 自碰撞"失败（报错形如
   `5 contacts: base_link-shoulder_link...`）。
5. **MTC 0.1.8 API 差异**：
   - 命名空间 `moveit::task_constructor`，**没有 `mtc` 别名**（需 `namespace mtc = ...`）
   - `Stage::traverseRecursively` 回调签名 `bool(const Stage&, unsigned int)`
   - `Connect::setTimeout()`（不是 timeout()）
   - capability 类名是 **`move_group/ExecuteTaskSolutionCapability`**（不是
     `moveit_task_constructor_capabilities/...`）
   - move_group 的 `capabilities` 参数是 **string**（非 string_array）
   - `GeneratePose::setPose` 设 "pose" 属性；ComputeIK 的 `target_pose` 需显式
     `properties().set("target_pose", <PoseStamped>)`（单层容器直通时）
   - `exposeTo` 方向：`task.properties().exposeTo(container->properties(), ...)`（PARENT→容器）
   - `FixedState::setState` 接受 `PlanningScenePtr`（非 RobotState），且**没有**
     `setMonitoredStage`
   - **KDL IK tip 只有 gripper_frame_link**：moving_jaw 是 revolute 关节后的 link，
     不能作 setIKFrame（报 `Cannot compute IK ... pose reference frame 'moving_jaw...'`）
6. **Jazzy move_group 参数**：`capabilities` 传 string_array 会 abort（InvalidParameterType）。
7. **planning scene 同步**：psi.applyCollisionObjects 后 MTC 的 scene monitor 需要时间同步
   （加了轮询等待 known objects 非空）。
8. **probe/ROS 工具类**：JSB 的 /joint_states 是 sensor QoS（best_effort），订阅要
   `qos_profile_sensor_data`；srv 字段名 `GetMotionPlan.Request.motion_plan_request`、
   `Response.motion_plan_response.trajectory`。

## 备选方案（如果 ACM 修复仍不通）

1. **绕过 Connect**：把 `move to pick`/`move to place` 的 Connect 换成
   `MoveTo`（joint interpolation 到探测的 pre-grasp/pre-place 关节解）——
   关节解全部已知（见下），不依赖 OMPL。
2. **gz 后端优先验收**：gz 侧 DetachableJoint 是官方组件，若 mujoco 侧 MTC-执行联调
   持续受阻，可先验收 gz（MTC 规划层与后端无关）。
3. **降低验收标准**：plan 成功 + RViz Introspection 显示 + execute 其中一段。

## 探测关节解（probe_pick_poses.py 输出，已写入节点默认参数）

```
pre-grasp: [0.7616, -0.9181, 1.3588, 1.1301, -0.7605]
grasp:     [0.7612, -0.1760, 1.4566, 0.2903, -0.7610]   ← 节点参数 grasp_joints
lift:      [0.7612, -0.7163, 1.4398, 0.8474, -0.7610]
pre-place: [-0.8919, -0.7670, 0.8978, 1.4370, -2.4157]
place:     [-0.8918, -0.5661, 1.1766, 0.9603, -2.4140]  ← 节点参数 place_joints
```

## 复现与调试命令

```bash
# 环境
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
export MUJOCO_GL=egl
bash scripts/clean_ros.sh

# 构建（C++ 改动后务必 clean-first 或不用 symlink-install）
colcon build --packages-select so101_mtc_pick_place --cmake-clean-first

# 端到端冒烟（日志落 /tmp/mtc_smoke1.log）
timeout 150 ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py \
  sim:=mujoco rviz:=false > /tmp/mtc_smoke1.log 2>&1

# stage 级诊断在 mtc_pick_place_node 的 "no solution found" WARN 行
grep -E "stage '" /tmp/mtc_smoke1.log

# 独立 OMPL 探针（对照实验, 需先起 launch）
/tmp/ompl_probe.sh   # python: /tmp/ompl_probe.py

# 关键源文件
src/so101_mtc_pick_place/src/mtc_pick_place_node.cpp      # MTC stage 链
src/so101_mtc_pick_place/src/grasp_attach_plugin.cpp      # kinematic attach
src/so101_mtc_pick_place/config/controllers_pick.yaml     # plugin 参数
src/so101_mtc_pick_place/launch/mtc_pick_place.launch.py  # 双后端组装
scripts/probe_pick_poses.py                               # 可达性探测
```

## 测试状态

`uv run pytest` → 127 passed（含 scene/probe/包结构测试）。
C++ 侧验证 = colcon build + 端到端冒烟（无 gtest）。
