"""SO101 RL 训练库（plan/rl.md）。

纯 Python 无 ROS 依赖：sb3/skrl 双后端统一 TrainerSpec 抽象。
checkpoint 目录约定: model.{zip|pt} + meta.json (+ vecnormalize.pkl 若启用 normalize)。
"""

from __future__ import annotations
