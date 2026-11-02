"""RLExpertTeleop: 用训练好的 RL checkpoint 作为 il_sim 自动采集专家。

闭环方式与脚本专家一致：Teleoperator.get_action() 无 obs 入参，经 env_backend
读取共享 env（robot.get_observation 缓存的同一帧 state）。

动作语义（关键）:
- RL checkpoint 的 meta.json 记录训练时的 env_kwargs（如 action_mode=delta）。
- 采集 env 恒为 position 模式、数据集 action 恒为 [-1,1] 绝对归一化关节目标
  （与 il_sim 推理端 env.step 语义一致）。
- 若 checkpoint 训练于 delta 模式，此处按训练语义积分:
      target_rad = clip(prev_target + clip(a) * DELTA_MAX, ctrlrange)
      prev_target = env.target_qpos（position 模式的 _ctrl_target，逐帧等于上一拍目标）
  再归一化回 [-1,1] 交给 SimSO101Robot.send_action —— 闭环行为与 RL 推理节点一致。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from so101_envs.base_env import DELTA_MAX

from ..constants import JOINT_NAMES
from ..env_backend import get_active_env, latest_state


@TeleoperatorConfig.register_subclass("so101_rl_expert")
@dataclass
class RLExpertTeleopConfig(TeleoperatorConfig):
    # RL checkpoint 目录（也接受 outputs/rl/<tag>/latest 指针）
    checkpoint: str = ""


class RLExpertTeleop(Teleoperator):
    config_class = RLExpertTeleopConfig
    name = "so101_rl_expert"

    def __init__(self, config: RLExpertTeleopConfig):
        super().__init__(config)
        self.config = config
        self._adapter = None
        self._action_mode = "position"

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in JOINT_NAMES}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._adapter is not None

    def is_calibrated(self) -> bool:
        return True

    def connect(self) -> None:
        if not self.config.checkpoint:
            raise ValueError("so101_rl_expert requires --teleop.checkpoint=<RL checkpoint dir>")

        from so101_rl.lerobot_deploy import GymPolicyAdapter
        from so101_rl.runs import resolve_ckpt_dir

        self._adapter = GymPolicyAdapter.from_checkpoint(
            resolve_ckpt_dir(Path(self.config.checkpoint)))
        self._action_mode = self._adapter.action_mode

    def disconnect(self) -> None:
        self._adapter = None

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass

    def obs_state(self) -> np.ndarray:
        """RL policy 观测：优先复用 robot.get_observation 缓存的同一帧 state。"""
        state = latest_state()
        if state is not None:
            return np.asarray(state)
        obs = get_active_env().unwrapped._get_obs()
        return np.asarray(obs["state"] if isinstance(obs, dict) else obs)

    def get_action(self) -> dict[str, float]:
        if self._adapter is None:
            raise RuntimeError("so101_rl_expert is not connected")
        env = get_active_env()
        u = env.unwrapped
        if getattr(u, "action_mode", "position") != "position":
            raise RuntimeError(
                "so101_rl_expert 要求采集 env 为 position 模式（数据集 action = 绝对关节目标）；"
                f"当前 action_mode={getattr(u, 'action_mode', '?')}"
            )

        raw = np.clip(
            np.asarray(self._adapter.act(self.obs_state()), dtype=np.float64).reshape(-1),
            -1.0, 1.0)
        lo, hi = u._ctrlrange[:, 0], u._ctrlrange[:, 1]
        if getattr(self._adapter, "action_mode", self._action_mode) == "delta":
            target = np.clip(u.target_qpos + raw * DELTA_MAX, lo, hi)
        else:
            target = u.denormalize_action(raw)
        a_norm = np.clip(u.normalize_qpos(target), -1.0, 1.0)
        return {f"{m}.pos": float(v) for m, v in zip(JOINT_NAMES, a_norm)}
