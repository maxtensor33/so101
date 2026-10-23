"""SimSO101Robot: so101_envs camera env -> lerobot Robot protocol (plan/il_sim.md Phase 1).

Conventions (lerobot 0.6.1, locked by tests/test_il_sim_smoke.py):
- feature keys `{motor}.pos: float` -> derived observation.state / action (shape (6,))
- camera keys fixed/wrist -> (H,W,3) -> derived observation.images.<key>
- record_loop per frame: get_observation -> teleop.get_action -> send_action (env.step inside)
"""

from __future__ import annotations

from functools import cached_property

import gymnasium as gym
import numpy as np

import so101_envs  # noqa: F401  gym 注册副作用
from lerobot.robots.robot import Robot

from .configs import SimSO101RobotConfig
from .constants import CAMERA_KEYS, JOINT_NAMES


class _SimCamera:
    """Placeholder camera descriptor (record() only uses len(robot.cameras))."""

    def __init__(self, name: str, shape: tuple[int, int, int]):
        self.name = name
        self.shape = shape
        self.height, self.width, self.channels = shape
        self.is_connected = True


class SimSO101Robot(Robot):
    config_class = SimSO101RobotConfig
    name = "so101_sim"

    def __init__(self, config: SimSO101RobotConfig):
        super().__init__(config)
        self.config = config
        self._env = None
        self._cameras = {name: _SimCamera(name, (128, 128, 3)) for name in config.camera_keys}

    # ---------- lerobot protocol ----------

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        ft: dict[str, type | tuple] = {f"{m}.pos": float for m in JOINT_NAMES}
        for cam in self._cameras.values():
            ft[cam.name] = cam.shape
        return ft

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in JOINT_NAMES}

    @property
    def is_connected(self) -> bool:
        return self._env is not None

    def connect(self, calibrate: bool = True) -> None:
        """Create + reset the gym env. Simulation needs no calibration."""
        if self.is_connected:
            raise RuntimeError(f"{self} is already connected")
        render_mode = "human" if self.config.render else None
        self._env = gym.make(
            self.config.env_id,
            render_mode=render_mode,
            viewer_ui=self.config.viewer_ui,
            viewer_maximize=self.config.viewer_maximize,
            viewer_no_input=self.config.viewer_no_input,
        )
        self._env.reset(seed=self.config.seed)
        from .env_backend import set_active_env

        set_active_env(self._env)
        for cam in self._cameras.values():
            cam.is_connected = True

    def disconnect(self) -> None:
        if self._env is not None:
            from .env_backend import clear_active_env

            clear_active_env()
            self._env.close()
            self._env = None

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        pass

    @property
    def is_calibrated(self) -> bool:
        return True

    def get_observation(self) -> dict[str, object]:
        env_obs = self._env.unwrapped._get_obs()
        qpos = np.asarray(env_obs["state"][:6])
        qpos_norm = self._env.unwrapped.normalize_qpos(qpos)
        obs: dict[str, object] = {f"{m}.pos": float(v) for m, v in zip(JOINT_NAMES, qpos_norm)}
        for cam in self._cameras:
            obs[cam] = np.asarray(env_obs["rgb"][cam])
        from .env_backend import set_latest_images

        set_latest_images({cam: obs[cam] for cam in self._cameras})
        return obs

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        target = np.array([action[f"{m}.pos"] for m in JOINT_NAMES], dtype=np.float64)
        target = np.clip(target, -1.0, 1.0)
        self._env.step(target)
        from .env_backend import take_reset_request

        success = self._env.unwrapped.evaluate().get("success", False)
        if take_reset_request() or (self.config.auto_reset_on_success and success):
            self._env.reset(seed=self.config.seed)
            from .env_backend import bump_reset_count

            bump_reset_count()
        return {f"{m}.pos": float(v) for m, v in zip(JOINT_NAMES, target)}

    @property
    def cameras(self) -> dict:
        return self._cameras
