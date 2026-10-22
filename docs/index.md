# SO-ARM101 工作区文档

ROS2 Jazzy + MuJoCo + uv/colcon 混合管理的 SO-ARM101 机械臂项目。
文档按方向划分，各方向独立成页（参考 stable-baselines3 文档结构）。

## 方向总览

| 方向 | 定位 | 入口 |
|---|---|---|
| **robotics** | 力矩控制学习（教学目的）：FK/IK/动力学/轨迹/控制 → draw cube | {doc}`robotics/index` |
| **envs** | gymnasium 环境集（mujoco 内核）：reach/lift/place/stack × 双相机 | {doc}`envs/index` |
| **rl** | RL 训练/评估/推理（sb3/skrl 双后端 + ROS2 推理） | {doc}`rl/index` |
| **il_sim** | 模仿学习（仿真数据）——待实现 | {doc}`il_sim/index` |
| **il_real** | 模仿学习（真机遥操作）——待实现 | {doc}`il_real/index` |

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
| `plan/` | 各方向原始计划（setup/robotics/envs/rl） |
| `reports/` | 验收报告（对齐/draw cube/envs/rl） |
| `todo/` | 待办勘察（gz 验证、GPU 并行路线、文档图片清单） |
| `learn/` | 经验复盘（坑清单与改进） |
| `outputs/` | RL 训练产物（gitignore） |

```{toctree}
:maxdepth: 1
:caption: 方向

robotics/index
envs/index
rl/index
il_sim/index
il_real/index
```

```{toctree}
:maxdepth: 1
:caption: 附注

pitfalls
```
