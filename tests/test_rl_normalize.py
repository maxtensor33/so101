"""VecNormalize 状态存/载往返一致性（plan/rl.md normalize.py 契约）。"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
from stable_baselines3.common.vec_env import VecNormalize


def test_vecnormalize_roundtrip(tmp_path):
    from so101_rl.normalize import load_norm, maybe_wrap, save_norm

    env = maybe_wrap(None, normalize=False)
    assert env is None

    import gymnasium as gym
    import so101_envs  # noqa: F401

    venv = maybe_wrap(gym.make("SO101Reach-v1"), normalize=True)
    assert isinstance(venv, VecNormalize)
    try:
        venv.reset()
        # 产生一些统计量
        for _ in range(5):
            venv.step(np.zeros(6, dtype=np.float32))
        assert save_norm(venv, tmp_path) is True
        obs = venv.normalize_obs(np.zeros(25, dtype=np.float32))

        # 载入到新 venv
        venv2 = VecNormalize(__import__("stable_baselines3").common.vec_env.DummyVecEnv(
            [lambda: gym.make("SO101Reach-v1")]), norm_obs=True, norm_reward=False)
        loaded, ok = load_norm(tmp_path, venv2)
        assert ok is True
        obs2 = loaded.normalize_obs(np.zeros(25, dtype=np.float32))
        assert np.allclose(obs, obs2), f"{obs[:4]} vs {obs2[:4]}"
    finally:
        venv.close()


def test_load_norm_missing_file(tmp_path):
    import gymnasium as gym
    import so101_envs  # noqa: F401
    from stable_baselines3.common.vec_env import DummyVecEnv

    from so101_rl.normalize import load_norm

    venv = DummyVecEnv([lambda: gym.make("SO101Reach-v1")])
    out, ok = load_norm(tmp_path, venv)
    assert ok is False and out is venv
