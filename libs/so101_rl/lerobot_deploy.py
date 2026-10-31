"""lerobot sim2real 部署接口层（workflow/plan/rl.md Phase 5, 零 lerobot 依赖）。

定义部署策略协议 + 把训练 checkpoint 适配为该协议。
真机接线待硬件就绪（见 LerobotSO101Arm 类内 TODO）。

约定（与训练一致）:
- obs: 环境 state 向量（真机按同序构造: qpos 读舵机 rad, qvel 差分, tcp 由 FK）
- action: [-1,1] (6,)
  - position 模式: target_rad = lo + (a+1)/2 * (hi-lo)
  - delta 模式（squint 默认）: target_rad = current_target + a * DELTA_MAX（需外部保留目标状态）
"""

from __future__ import annotations

import abc
from pathlib import Path

import numpy as np

# MJCF actuator ctrlrange（与 so101_env.xml 一致）
JOINT_LOWER = np.array([-1.91986, -1.91986, -1.69, -1.65806, -2.74385, -0.17453])
JOINT_UPPER = np.array([1.91986, 1.91986, 1.69, 1.65806, 2.84121, 1.74533])
# squint pd_joint_target_delta_pos 每步增量上限
DELTA_MAX = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.2])


def denormalize_action(action, current_target=None, action_mode="position"):
    """[-1,1] 动作 -> 真机关节目标 (rad); delta 模式需传上一拍 target。"""
    a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
    if action_mode == "delta":
        if current_target is None:
            raise ValueError("delta 模式需要 current_target（上一拍目标 qpos）")
        return np.clip(np.asarray(current_target, dtype=float) + a * DELTA_MAX,
                       JOINT_LOWER, JOINT_UPPER)
    return JOINT_LOWER + (a + 1.0) / 2.0 * (JOINT_UPPER - JOINT_LOWER)


class SO101DeployPolicy(abc.ABC):
    """部署策略协议：真机执行循环只依赖此接口。"""

    @abc.abstractmethod
    def reset(self) -> None:
        """episode 开始时调用（清内部状态）。"""

    @abc.abstractmethod
    def act(self, obs):
        """观测 -> [-1,1] 归一化动作 (6,)。"""


class GymPolicyAdapter(SO101DeployPolicy):
    """把 so101_rl checkpoint（sb3/skrl 双后端）适配为部署协议。"""

    def __init__(self, policy, meta: dict):
        self._policy = policy
        self.meta = meta
        self.action_mode = meta.get("env_kwargs", {}).get("action_mode", "position")

    @classmethod
    def from_checkpoint(cls, ckpt_dir):
        """按 meta.json 识别后端并加载（与 rl_infer_node 同一入口）。"""
        from so101_rl.algos import get_trainer
        from so101_rl.backends.base import read_meta

        ckpt_dir = Path(ckpt_dir)
        meta = read_meta(ckpt_dir)
        trainer = get_trainer(meta["backend"], meta["algo"])
        return cls(trainer.load_policy(ckpt_dir), meta)

    def reset(self) -> None:
        return None  # 无状态 MLP 无需重置; 预留递归策略/滤波状态位

    def act(self, obs):
        return np.asarray(self._policy(obs))


class LerobotSO101Arm(SO101DeployPolicy):
    """lerobot 真机执行 TODO 骨架（不 import lerobot，硬件就绪后接线）。

    TODO(lerobot): 硬件就绪后的接线步骤
    1. 安装 lerobot（独立环境，避免污染本 venv 的 numpy<2 约束）
    2. SO101Follower(port=/dev/ttyACM*) connect()
    3. reset(): 发送 REST 位形（so101_envs.base_env.REST_QPOS 对齐）
    4. act(obs): denormalize_action(a, current_target, adapter.action_mode) -> 逐关节目标 rad
       delta 模式必须保留上一拍 target 并逐步积分（与训练 action_mode 一致）
    5. 真机 obs 构造: qpos 读舵机(calibration 后 rad), qvel 差分/dt, tcp 用 pinocchio FK
    6. 安全: 限幅/急停/与仿真限位一致
    """

    def __init__(self, policy, port: str = ""):
        raise NotImplementedError("lerobot 真机部署待硬件就绪（workflow/plan/rl.md Phase 5 TODO）")

    def reset(self) -> None:  # pragma: no cover
        raise NotImplementedError

    def act(self, obs):  # pragma: no cover
        raise NotImplementedError
