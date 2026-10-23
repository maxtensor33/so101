# MTC Pick and Place 调试问题交接 (deepseek-v4-flash 会话)

日期: 2026-10-21 · 计划: `plan/mtc.md` · 分支: main · 测试: 127 passed(未新增)
承接: `problems/mtc_glm-5-3-flash-260828.md`(上一会话交接, 原始卡点 = move to pick/place 的 Connect 0 解)

> ⚠️ **本会话未修复成功, 且当前代码处于"不可用"状态**（`task.init()` 抛异常失败）。
> 本文件记录精确的根因线索与已排除项, 供下一会话接手, 别从交接文本(glm5_3)的
> 根因判断继续——它已被本会话实测推翻。

---

## 一、实测推翻的根因(别再走弯路)

交接 glm5_3 判断「FixedState 手工 scene 的 ACM 不含 cube 放行 → Connect goal 碰撞被拒」,
**已由探针实测排除**:

1. 用 `/get_planning_scene`(见 `/tmp/acm_check.py`)读 move_group 场景:`cube` 存在,
   ACM `cube allowed-with = [gripper_link, moving_jaw_so101_v1_link]`,`moving_jaw <-> cube = True`。
   → **move_group 的 ACM 爪↔cube 放行早已生效**。
2. 把 ACM 进一步扩张到 **cube ↔ 全部 link 全放行**(spawnCollisionObjects 已改), 探针里
   `grasp` 立即从 FAIL→SUCCESS。但 `move to pick` 在 MTC 里仍是 0 解 → 证明 MTC 失败
   不是 ACM。
3. tolerance 假设: PipelinePlanner 默认 `goal_joint_tolerance=1e-4 rad`, 加大到 `0.05`
   并 `num_planning_attempts=10u` 后**仍 0 解** → 排除 tolerance。

## 二、决定性探针数据(move_group `/plan_kinematic_path` 直发关节约束)

`scripts/probe_pick_poses.py` 的关节解在 **move_group**(`/tmp/ompl_probe2.py` / `/tmp/ompl_probe3.py`)
从 home 实况 start 的 OMPL 可达性:

| 目标(关节解) | cube 仅爪放行 | cube 全放行 | 说明 |
|---|---|---|---|
| `grasp`     | FAIL(99999,7点) | **SUCCESS** | 接触位形, 冲突项=其他臂末end(如 wrist)碰 cube → 全放行即可过 |
| `pre-grasp` | FAIL(99999,0点) | **FAIL(99999,0点)** | ⚠️ 0 路径点 = **无解, 非碰撞** |
| `lift`      | SUCCESS | SUCCESS | 高位形全程可达 |
| `pre-place`/`place` | SUCCESS | SUCCESS | 全程可达 |

**核心异常**: `pre-grasp` 关节解 `[0.7616,-0.9181,1.3588,1.1301,-0.7605]` 即使 cube
全放行, OMPL 从 home 也**找不到任何路径**(0 点)。而 `grasp`(同处接触位)可达。
→ move to pick 原始 Connect 以 pre-grasp 为目标, 必然 0 解。这解释了 glm5_3 的卡点本身,
但**没解释 MTC 为何连 grasp 也 bridge 不过去**(见下一条)。

## 三、真正的 MTC 内部问题(= 未被解释的深水区)

- **手工 `FixedState` 的 to-scene 无法被上游 `Connect` 桥接**。日志 transition:
  `- 1 → 0 ← 0 - / move to pick` —— 上游 current/open-gripper 各 1 解, 但 Connect
  不管用 interpolation 还是 OMPL 都通向手工 FixedState 失败, **与位形可达性无关**。
  现象在把 move to pick 目标改为 **grasp(探针证明可达)** 后依旧出现。
  → 强烈指向:MTC 的 Connect 无法把「实时 move_group 传播的 from-scene」桥接到
    「`make_shared<PlanningScene>(robot_model)` 手工构造的 to-scene」(joint 集/维度/
    状态不一致)。
- 容器内 `close gripper`/`attach`/`lift`/place 各 stage **原本都能各自出 1 解**(在有
  approach 的完整版里 pick 容器=1 solution), 所以容器内部没问题; 问题只悬在外部
  bridge 段(move to pick / move to place 连接处)。

## 四、本会话已做的改动与最后状态(当前 = task.init() 失败)

`src/so101_mtc_pick_place/src/mtc_pick_place_node.cpp` 已修改, **当前会 `task.init()` 抛
InitStageException 退出**, 不能冒烟:

1. **cube ACM 全放行**(spawnCollisionObjects 的 ACM 循环改为 cube↔所有 link true),
   并新增私有方法 `allowGripperCubeCollision(scene)` 对 FixedState 场景同样放行。
2. **删除 approach stage**(不再有水平/竖直进给;**pick 容器 start 直接 = grasp 位形**)。
3. **两个 Connect 换成 MoveTo** 直达已知关节解:
   - `move to pick` → MoveTo(interpolation) → `grasp_joints_` map
   - `move to place` → MoveTo(interpolation) → `place_joints_` map
   - 已从这两个 MoveTo 上**去掉 `configureInitFrom(PARENT)`**(否则 parent 的 `"goal"`
     属性 PoseStamped 会覆盖本 stage 的 joint-map `"goal"`, 报类型冲突)。
4. `sampling_planner`(PipelinePlanner)仍保留但已不被 move to pick/place 使用(供未来恢复);
   设了 `goal_joint_tolerance=0.05`、`num_planning_attempts=10u`、`publish_planning_requests=true`。
5. `task.init()` 的 catch 已改成 `RCLCPP_ERROR_STREAM(... << e)`, 但日志仍只打印首行
   `Error initializing stages:` 后无 stage 列表(详情被截断, 未查得)。

**当前最后的冒烟现象**(`bash scripts/mtc_smoke.sh sim:=mujoco`):
```
task.init() failed: Error initializing stages:
mtc_pick_place_node exit (FAILED)
```
具体是哪个 stage、缺哪个属性, **未查明**(异常被截断)。下一步应先拿到完整 init 异常
(必要时把 MoveTo 的 joint-map goal 换成 `moveit_msgs::msg::RobotState` 或命名位形再试)。

## 五、新增工具(自终止冒烟, 解决"timeout 死等")

`scripts/mtc_smoke.sh`(本会话新增, 已验证可用): 后台起 launch 落日志, 轮询
`mtc_pick_place_node exit (SUCCESS|FAILED)` 关键词, **一出结果就 kill 整个进程组并立即
返回**, 不依赖 `timeout` 死等、不残留进程。
```bash
bash scripts/mtc_smoke.sh sim:=mujoco rviz:=false --log /tmp/mtc_smoke.log
```
之前 `timeout 150 ros2 launch ...` 会因 launch 在节点退出后仍挂着 mujoco 而死等超时,
被用户手动关闭才返回——非常废时间。**用脚本即可秒级出结果。**

探测/诊断脚本(在 `/tmp`, 未入库, 可参考重建):
- `/tmp/acm_check.py` — 读 move_group 场景的 cube 位置与 ACM 状态。
- `/tmp/ompl_probe2.py` / `/tmp/ompl_probe3.py` — 对交接关节解逐个发 `/plan_kinematic_path`
  测 OMPL 可达性(结论见第二节表格)。

## 六、其余已稳定结论 / 待查项

- mujoco 抓取语义 = kinematic attach(grasp_attach_plugin), 与 Msetsolve 无关; 物理夹持与
  weld 均不可靠(glm5_3 实测)。
- build 后**必须 `--cmake-clean-first`**, 否则增量判断失效不重编。
- ⚠️ 待查:**/get_planning_scene 返回的 cube 位置是 `(0,0,0)`**, 而非应设的
  `(0.117,-0.077,0.02)`(spawnCollisionObjects 的参数)。这可能是 move_group 场景里 cube
  摆位相关的一个隐藏 bug, 与规划失败是否有关未定论。
- 若 MoveTo/init 修好后 MoveTo 直线插值路径仍碰撞(长程 place 段), 备选: 给 move to place
  用可达的中间高位形分段 MoveTo, 或用回 OMPL(但需先解释为何 MTC Connect 桥接手工
  FixedState 失败)。

## 七、测试状态

- `uv run pytest` → 127 passed(本会话未新增 C++ 侧测试;C++ 侧 = colcon build + 冒烟,
  当前冒烟 FAILED 在 task.init)。
- 无 commit(工作流约定: commit 前须征得用户同意)。