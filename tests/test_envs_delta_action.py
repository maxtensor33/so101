"""delta 动作模式测试（squint 对齐: 每步 ctrl 增量 ±DELTA_MAX, 目标积分 + 限幅）。

对应改动: base_env.step 的 delta 分支; position 默认行为不变（兼容专家/遥操链路）。
"""

from __future__ import annotations

import numpy as np
import pytest

import so101_envs  # noqa: F401  注册副作用
from gymnasium import make

from so101_envs.base_env import DELTA_MAX, REST_QPOS


@pytest.fixture
def env():
    env = make("SO101Reach-v1", action_mode="delta")
    env.reset(seed=0)
    yield env
    env.close()


def _ctrl(env):
    return env.unwrapped.data.ctrl.copy()


def test_action_space_unchanged(env):
    """两种模式动作空间一致: Box(-1,1,(6,)), 推理/采集链路零感知。"""
    assert env.action_space.shape == (6,)
    assert np.all(env.action_space.low == -1) and np.all(env.action_space.high == 1)


def test_delta_integration_bounded(env):
    """持续 +1 动作: ctrl 每步至多挪 DELTA_MAX, 最终钳在 ctrlrange 上限。"""
    lo = env.unwrapped._ctrlrange[:, 0]
    hi = env.unwrapped._ctrlrange[:, 1]
    for _ in range(200):
        prev = _ctrl(env)
        env.step(np.ones(6, dtype=np.float32))
        cur = _ctrl(env)
        assert np.all(cur - prev <= DELTA_MAX + 1e-9)
        assert np.all(cur <= hi + 1e-9)
    assert np.allclose(cur, hi)


def test_delta_symmetric_negative(env):
    """持续 -1 动作: 对称地积到 ctrlrange 下限。"""
    lo = env.unwrapped._ctrlrange[:, 0]
    for _ in range(200):
        env.step(-np.ones(6, dtype=np.float32))
    assert np.allclose(_ctrl(env), lo)


def test_delta_reset_starts_from_q0(env):
    """reset 后 ctrl = 初始 qpos（rest + 噪声）, 与 position 模式语义一致。"""
    obs, _ = env.reset(seed=42)
    q0 = env.unwrapped.data.qpos[env.unwrapped._qpos_adr]
    assert np.allclose(_ctrl(env), q0)
    # squint: rest + randn*0.02（std=0.02, 非 uniform ±0.02）
    assert np.all(np.abs(q0 - REST_QPOS) <= 5 * 0.02 + 1e-9)


def test_position_mode_default():
    """默认（不传参）仍为 position 全行程映射——专家/遥操/采集链路不受影响。"""
    env = make("SO101Reach-v1")
    env.reset(seed=0)
    assert env.unwrapped.action_mode == "position"
    action = np.array([1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    env.step(action)
    ctrl = env.unwrapped.data.ctrl
    hi = env.unwrapped._ctrlrange[:, 1]
    lo = env.unwrapped._ctrlrange[:, 0]
    assert np.isclose(ctrl[0], hi[0]) and np.isclose(ctrl[1], lo[1])
    env.close()


def test_invalid_mode_rejected():
    from gymnasium import spaces

    from so101_envs.reach import SO101ReachEnv

    with pytest.raises(AssertionError, match="action_mode"):
        SO101ReachEnv(action_mode="bogus")
