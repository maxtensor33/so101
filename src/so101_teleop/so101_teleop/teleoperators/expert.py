"""ScriptExpertTeleop: IK state-machine Teleoperator (workflow/plan/il_sim.md Phase 2).

Closed-loop via shared env (env_backend). Phases: WAIT -> GO -> SETTLE,
then request_reset() so the robot resets the env at the next frame boundary
and the state machine restarts for the next episode.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from so101_envs.ik import SO101IK

from ..constants import JOINT_NAMES
from ..env_backend import get_active_env, request_reset

SETTLE_STEPS = 8
WAIT_STEPS = 10
GO_STEPS_MAX = 40
TOL = 0.006
MAX_DQ = 0.12


@TeleoperatorConfig.register_subclass("so101_expert")
@dataclass
class ScriptExpertTeleopConfig(TeleoperatorConfig):
    urdf_path: str | None = None  # None = search repo models/ upward
    max_dq: float = MAX_DQ


def _find_urdf() -> Path:
    for parent in Path(__file__).resolve().parents:
        cand = parent / "models" / "so101_new_calib.urdf"
        if cand.is_file():
            return cand
    raise FileNotFoundError("models/so101_new_calib.urdf not found")


class ScriptExpertTeleop(Teleoperator):
    config_class = ScriptExpertTeleopConfig
    name = "so101_expert"

    def __init__(self, config: ScriptExpertTeleopConfig):
        super().__init__(config)
        self.config = config
        self._ik = None
        self._phase = "WAIT"
        self._step = 0
        # squint reach success 要求闭爪；全程闭爪（开爪到位再闭爪会移动指尖中点 TCP）
        self._grip_hold = -1.0

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in JOINT_NAMES}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._ik is not None

    def is_calibrated(self) -> bool:
        return True

    def connect(self) -> None:
        urdf = Path(self.config.urdf_path) if self.config.urdf_path else _find_urdf()
        self._ik = SO101IK(urdf)
        self._phase = "WAIT"
        self._step = 0

    def disconnect(self) -> None:
        self._ik = None

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass

    # ---------- IK state machine ----------

    @property
    def phase(self) -> str:
        return self._phase

    def get_action(self) -> dict[str, float]:
        env = get_active_env()
        u = env.unwrapped
        q_arm = u.data.qpos[u._qpos_adr][:5]
        a = u.normalize_qpos(u.data.qpos[u._qpos_adr])
        a[5] = self._grip_hold

        # 非 reach 环境（如 lift）: 零动作保持。lift 专家已移除,
        # 见 workflow/todo/il_sim_expert_lift.md, 不在此处做抓取。
        if not hasattr(u, "_goal"):
            return {f"{m}.pos": float(v) for m, v in zip(JOINT_NAMES, a)}

        if self._phase == "WAIT":
            self._step += 1
            if self._step >= WAIT_STEPS:
                self._phase = "GO"
                self._step = 0
        elif self._phase == "GO":
            goal = u._goal
            if np.linalg.norm(u.tcp_pos - goal) < TOL or self._step >= GO_STEPS_MAX:
                self._phase = "SETTLE"
                self._step = 0
                self._grip_hold = -1.0  # squint reach success 要求夹爪闭合
            else:
                q_ref, _ = self._ik.solve(
                    q_arm, goal, n_iter=5,
                    q_gripper=u.data.qpos[u._qpos_adr[5]],
                    position_only=True)  # reach 不约束朝向
                dq = np.clip(q_ref - q_arm, -self.config.max_dq, self.config.max_dq)
                lo = u._ctrlrange[:5, 0]
                hi = u._ctrlrange[:5, 1]
                a_arm = 2.0 * (q_arm + dq - lo) / (hi - lo) - 1.0
                a[:5] = a_arm
            self._step += 1
        elif self._phase == "SETTLE":
            self._step += 1
            if self._step >= SETTLE_STEPS:
                request_reset()
                self._phase = "WAIT"
                self._step = 0
        return {f"{m}.pos": float(v) for m, v in zip(JOINT_NAMES, a)}
