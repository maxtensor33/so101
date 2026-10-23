"""rl 依赖与环境兼容冒烟（workflow/plan/rl.md Phase 0, TDD）。

- torch/sb3/skrl 可导入且版本满足下限
- cuda 可用性打印（CI 无 GPU 不强制）
- sb3 env_checker 通过我们的 SO101Reach-v1（兼容性关键锁定）
- skrl wrapper 包我们的 env 跑 1 step
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")


def test_imports_and_versions():
    import stable_baselines3 as sb3
    import skrl
    import torch

    assert tuple(int(x) for x in sb3.__version__.split(".")[:2]) >= (2, 4), sb3.__version__
    assert tuple(int(x) for x in skrl.__version__.split(".")[:1]) >= (1,), skrl.__version__
    assert tuple(int(x) for x in torch.__version__.split(".")[:1]) >= (2,), torch.__version__
    print(f"cuda available: {torch.cuda.is_available()} "
          f"(device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'cpu'})")


def test_sb3_checker_accepts_our_env():
    """sb3 env_checker 通过 + 1 step（gymnasium 1.x 兼容锁定）。"""
    import gymnasium as gym
    import so101_envs  # noqa: F401  注册副作用
    from stable_baselines3.common.env_checker import check_env

    env = gym.make("SO101Reach-v1")
    try:
        check_env(env)
        obs, _ = env.reset(seed=0)
        assert obs.shape == (25,)
        obs, r, term, trunc, info = env.step(env.action_space.sample())
        assert np_shape_ok(obs, (25,))
    finally:
        env.close()


def test_skrl_wraps_our_env():
    """skrl 环境 wrapper 包我们的 env 跑 1 step。"""
    import gymnasium as gym
    import so101_envs  # noqa: F401  注册副作用
    import torch
    from skrl.envs.wrappers.torch import wrap_env

    env = wrap_env(gym.make("SO101Reach-v1"))
    obs = env.reset()
    # skrl torch wrapper 要求 torch tensor 动作（不自动转换 numpy）
    action = torch.as_tensor(env.action_space.sample(), dtype=torch.float32)
    out = env.step(action)
    env.close()


def np_shape_ok(arr, shape):
    import numpy as np

    return np.asarray(arr).shape == shape
