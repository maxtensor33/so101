# TDD 证据报告: il_sim_develop（plan/il_sim_develop.md）

日期：2026-11-04 · 分支：main · 测试运行器：`uv run pytest`（Python/ROS 项目，ECC PM 探测脚本不适用，Step 0 以 pyproject `[tool.pytest.ini_options]` 解析）

## 来源计划

`workflow/plan/il_sim_develop.md`（本会话生成，用户 `/ecc:tdd-workflow` 触发执行）。

## 用户旅程（Step 1, 取自计划需求重述）

1. 作为开发者，我要 `il_sim_develop.launch.py` 在真机底座上驱动 il_sim policy，以便验证 sim2real 部署流程。
2. 作为操作员，我要 estop 话题立即停发动作目标，以便紧急情况保护机械臂。
3. 作为操作员，我要 confirm_step 逐单步确认 / Enter 触发 homing 回 rest，以便首跑安全。
4. 作为开发者，我要 obs 桥把 /joint_states + 相机帧按训练同约定构造成 policy 输入，以便零改动复用 checkpoint。
5. 作为操作员，我要退出/重置时插值回 rest 而非跳变，以便避免机械臂猛动。

## 任务报告（Step 8.3, 实际执行摘要）

| 任务 | 结果 |
|---|---|
| Phase 0 基线 | `clean_ros.sh` ✓；`colcon build --symlink-install` 11 包 ✓；`uv run pytest` 暴露 1 个 RED（`test_docs_links` 引用校验：计划引用的 tdd/reports/pitfalls 文件不存在）→ 建 3 个占位 stub 转绿。冒烟模型复用已有 `outputs/il_sim/act_reach_expert/checkpoints/000010/pretrained_model`（ACT, input: state(6)+fixed/wrist 3×128×128, output: action(6)），`load_policy` 验证可加载 |
| Phase 2 RED | `tests/test_il_sim_develop.py` 22+3 测试先行；首跑 `ModuleNotFoundError: so101_il_sim_develop`（编译期 RED，因实现缺失） |
| Phase 2 GREEN | 实现包（real_infer 纯函数 / il_sim_develop_node / camera_node / launch / rviz）后 `uv run pytest tests/test_il_sim_develop.py` **22 passed**；全量回归 **459 passed**（基线 437 = 436 过 + 1 修） |
| Phase 3 mock 冒烟 | 单进程集成：伪 /joint_states + 伪双相机帧驱动真实节点（加载冒烟 ACT）→ RUN 3s 发出 **28 条** /arm_controller/joint_trajectory；estop 后继续 spin 排空 0.3s → **0 条**（SMOKE PASS）。发现并修复 2 个缺陷：action 带 batch 维 (1,6) 需 reshape(-1)；estop 判定须排除已发布未投递的 in-flight 消息 |

## 测试保证表（Step 8.4）

| # | 保证 | 测试 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | 包结构/入口/launch/rviz/相机映射契约 | `tests/test_il_sim_develop.py::TestPackageLayout` | 契约 | PASS |
| 2 | normalize_qpos 两端点→±1、中点→0 | `::TestNormalizeQpos` | 单元 | PASS |
| 3 | obs batch key/布局/CHW/255 归一化/相机子集/缺帧报错 | `::TestBuildObsBatch` | 单元 | PASS |
| 4 | 动作单步增量限幅 + 绝对限界 | `::TestClampTarget` | 单元 | PASS |
| 5 | 回 rest 插值每步 ≤0.025 rad 且收敛 | `::TestPlanToRest` | 单元 | PASS |
| 6 | 640×480→中心裁方→128×128（squint 同款） | `::TestResizeToPolicy` | 单元 | PASS |
| 7 | estop 锁存 trigger/hold/clear | `::TestEstopLatch` | 单元 | PASS |
| 8 | 真实节点端到端: obs→policy→JTC 下发 28 条/3s | `/tmp/smoke_il_sim_develop.py`（Phase 3） | 集成 | PASS |
| 9 | estop 后（排空宽限 0.3s）零下发 | 同上 | 集成 | PASS |

## 覆盖率与已知缺口（Step 8.5）

- 未跑 pytest-cov 覆盖率（仓库无此配置项约定）；核心纯函数 `real_infer.py` 全函数被测，节点 I/O 回调由 Phase 3 集成冒烟覆盖。
- 已知缺口: 真机相关路径（cv2 采集/串口/JTC 实控/视觉对齐）无法本会话验证——Phase 1/4/5 checklist 留用户；节点 destroy_node 的 homing 循环仅语法验证。
- 计划偏差（如实记录）: usb_cam 未安装（apt 需 sudo）→ 改为包内内置 cv2 相机节点 `il_sim_camera_node`（对标 lerobot OpenCVCamera 语义），契约测试同步更新。

## 合并证据（Step 8.6）

checkpoint commits 待用户批准（项目约定: commit 须征得同意）。建议: `test: add il_sim_develop contract/unit tests (RED)` / `feat: so101_il_sim_develop package + mock smoke GREEN`。
