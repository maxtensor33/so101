# SO-ARM101 工作区文档

ROS2 Jazzy + MuJoCo + uv/colcon 混合管理的 SO-ARM101 机械臂项目。
文档按方向划分，各方向独立成页（参考 stable-baselines3 文档结构）。

## 方向总览

| 方向 | 定位 | 入口 |
|---|---|---|
| **robotics** | 力矩控制学习（教学目的）：FK/IK/动力学/轨迹/控制 → draw cube | {doc}`robotics/index` |
| **mtc** | MoveIt Task Constructor pick-and-place（纯 RViz 模式） | {doc}`mtc/index` |
| **envs** | gymnasium 环境集（mujoco 内核）：reach/lift/place/stack × cube/can × 双相机 | {doc}`envs/index` |
| **rl** | RL 训练/评估/推理（sb3/skrl 双后端 + ROS2 推理） | {doc}`rl/index` |
| **il_sim** | 模仿学习（仿真采集/训练/推理，lerobot 管线） | {doc}`il_sim/index` |
| **il_real** | 模仿学习（真机遥操作；leader 臂链路已可用） | {doc}`il_real/index` |
| **hardware** | 实机控制：so101_hardware 插件、标定、数字孪生、真机验收 | {doc}`hardware/index` |
| **setup** | 环境搭建与 URDF↔MuJoCo 对齐验证（命令见 README 与 robotics usage） | — |

## 快速命令

```bash
# 构建 / 测试
colcon build --symlink-install
uv run pytest

# 环境前置（zsh）
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
export MUJOCO_GL=egl
```

各方向的完整命令见对应方向页；通用约定见项目根 `CLAUDE.md`。

## 资源

| 位置 | 内容 |
|---|---|
| `workflow/plan/` | 各方向原始计划（setup / robotics / envs / rl / mtc_rviz / il_sim / il_sim_leader / setup_hardware；`mtc_bk` 归档） |
| `workflow/tdd/` | TDD 证据（各方向 `[.变体].tdd.md`） |
| `workflow/reports/` | 验收报告（对齐 / draw cube / envs / rl / mtc / il_sim / setup_hardware / squint 迁移等） |
| `workflow/todo/` | 暂停待办（`<域>_<主题>_todo.md`） |
| `workflow/learn/` | 经验复盘（坑清单与改进） |
| `workflow/problems/` | 跨会话调试交接（接手新方向必读） |
| `outputs/` | 运行产物（gitignore） |

```{toctree}
:maxdepth: 1
:caption: 方向

robotics/index
mtc/index
envs/index
rl/index
il_sim/index
il_real/index
hardware/index
```

```{toctree}
:maxdepth: 1
:caption: 附注

pitfalls
```
