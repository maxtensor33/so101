# TDD 证据报告: SO101 Envs（plan/envs.md）

**来源计划**: `plan/envs.md`（用户经 `/ecc:tdd-workflow plan/envs.md` 确认执行）
**日期**: 2026-10-21 ~ 10-22
**分支**: `envs`（TDD checkpoint commit 按用户指示未自动执行，由用户手动 commit）

## 用户旅程（源自计划）

1. 作为 RL 研究者，我想要 gymnasium 兼容的 SO101 mujoco 环境（reach/lift/place/stack），
   以便直接接入 RL 训练管线。
2. 作为 IL 研究者，我想要带相机（固定视角 + 夹爪视角成对）的环境变体，
   以便采集视觉观测数据。
3. 作为开发者，我想要 MJCF 与 URDF 的对齐保证，以便仿真与 ROS2/RViz/真机一致。
4. 作为学习者，我想要脚本专家与 IK，以便 sanity check 任务可解性。

## 任务 → 测试 → 证据映射

| # | 计划任务 | 测试目标 | RED 证据 | GREEN 证据 |
|---|---|---|---|---|
| 1 | Phase 0: MJCF 资产 + 对齐 | `test_envs_align.py` 32 例 | 32 failed（FileNotFoundError，资产不存在） | 32 passed |
| 2 | Phase 1: base_env + reach | `test_envs_api.py` + `test_envs_spaces.py` | 7 failed（NameNotFound 未注册） | 7 passed + 1 xfail（Phase 3 移除） |
| 3 | Phase 1: reach 任务语义 | `test_envs_tasks.py::test_reach_teleported_goal_success` | failed（settle 时序） | passed |
| 4 | Phase 2: lift/place/stack | `test_envs_tasks.py` 3 例 | 3 failed（未注册） | 4 passed |
| 5 | Phase 3: 相机版（成对双相机） | `test_envs_spaces.py::test_camera_obs_is_dict_space` ×4 | RED（未注册 Camera id） | 46 passed（api+spaces） |
| 6 | Phase 3: DR 生效 | `test_envs_spaces.py::test_domain_randomization_applied` | （随套件 RED） | passed |
| 7 | Phase 4: pinocchio IK | `test_envs_ik.py` 3 例 | 3 error（ModuleNotFound） | 3 passed |
| 8 | Phase 4: 专家端到端 | `scripts/run_expert.py` | —（验收脚本） | reach 5/5；lift/place/stack 0%（见下） |
| 9 | Phase 5: ROS2 可视化 | headless 话题验证 | bin/ 布局 ros2 run 失败 | /joint_states、双相机图像、物体 TF 全部发布 |
| 10 | Phase 6: 报告+文档 | 全套 pytest + colcon build | — | 86 passed；colcon build 通过 |

## 测试规格（人类可读保证）

| # | 保证内容 | 测试 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | 4 场景 MJCF 六关节/限位/actuator 与 URDF 对齐；FK 50 采样 <1mm/0.5° | `tests/test_envs_align.py` | 对齐 | PASS |
| 2 | 8 个注册 id 存在；4 state id 通过 gymnasium env_checker | `tests/test_envs_api.py` | API | PASS |
| 3 | 位置伺服收敛：恒定指令 → 关节 ±0.05 rad 到位 | `test_position_servo_converges` | 行为 | PASS |
| 4 | reset(seed) 可复现（state 严格，rgb ±1 EGL 噪声） | `test_reset_seed_reproducible` | API | PASS |
| 5 | 相机 id obs = Dict{state, rgb{fixed,wrist}} 128×128 uint8 | `test_camera_obs_is_dict_space` | 空间 | PASS |
| 6 | DR 开启后物体摩擦/颜色随机化生效 | `test_domain_randomization_applied` | 单元 | PASS |
| 7 | reach: goal 移至 TCP 处 → is_reached & success | `test_reach_teleported_goal_success` | 构造 | PASS |
| 8 | lift: 物体入爪闭合 → is_grasped 触发 | `test_lift_grasp_teleported_item` | 构造 | PASS |
| 9 | place: 物体入 bin → in_bin & success | `test_place_item_in_bin_success` | 构造 | PASS |
| 10 | stack: A 摞 B 上 → on_top & success | `test_stack_A_on_B_success` | 构造 | PASS |
| 11 | IK: 可达目标收敛 <1-2mm、解在限位内 | `tests/test_envs_ik.py` | 单元 | PASS |
| 12 | 专家 reach ≥90% | `scripts/run_expert.py` | E2E | PASS (100%) |
| 13 | 专家 lift/place/stack 稳定成功 | `scripts/run_expert.py` | E2E | **FAIL (0%)，已知问题** |

## 覆盖率与已知缺口

- 覆盖率命令未单独跑（pytest-cov 已配置）；86 例全绿，无 skip。
- 已知缺口（有意）：`randomize.py` 尺寸随机化未实现（需处理 geom_size 派生量
  rbound，留扩展）；lift/place/stack 专家端到端抓取保持失败（根因与后续方向见
  `reports/envs_report.md`），不影响环境 obs/reward/evaluate 语义。
- 校验命令（实际执行）：
  - `uv run pytest tests/test_envs_align.py tests/test_envs_api.py tests/test_envs_spaces.py tests/test_envs_tasks.py tests/test_envs_ik.py` → 86 passed
  - `uv run python scripts/run_expert.py --env all --episodes 5` → reach 100%，其余 0%
  - `colcon build --symlink-install --packages-select so101_envs_viz` → 通过
  - `ros2 run so101_envs_viz viz_node -p camera:=true` → 话题验证通过

## Merge 证据

- commit 由用户手动执行；本报告与 `reports/envs_report.md` 保存 RED→GREEN 映射。
- 输出损坏事件记录：开发过程中两次长文件写入出现乱码碎片（scene_lift.xml 首版、
  一次 Write 调用），均经完整性检查发现并重写；最终盘上文件均经 py_compile/
  XML 解析/pytest 验证。
