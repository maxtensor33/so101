"""obs/action 空间测试（plan/envs.md Phase 1/3, TDD）。

- 无相机 id: 扁平 state 向量（Box），形状与语义布局锁定
- 带相机 id（Phase 3 补）: Dict{"state", "rgb": {"fixed","wrist"}}，128x128 uint8
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import so101_envs  # noqa: F401  注册副作用
import gymnasium as gym
import mujoco
import numpy as np
import pytest

STATE_IDS = [
    "SO101Reach-v1", "SO101Lift-v1", "SO101Place-v1", "SO101Stack-v1",
]
CAMERA_IDS = [
    "SO101ReachCamera-v1", "SO101LiftCamera-v1", "SO101PlaceCamera-v1", "SO101StackCamera-v1",
]

# reach state obs 布局: [qpos(6) qvel(6) tcp_pos(3) tcp_quat(4) | goal_pos(3) tcp_to_goal(3)]
REACH_STATE_DIM = 25
SLICE_QPOS = slice(0, 6)
SLICE_QVEL = slice(6, 12)
SLICE_TCP_POS = slice(12, 15)
SLICE_TCP_QUAT = slice(15, 19)


def test_state_obs_shape():
    """无相机 reach obs 为 25 维 float64 Box，切片布局锁定。"""
    env = gym.make("SO101Reach-v1")
    try:
        obs, _ = env.reset(seed=0)
        assert obs.shape == (REACH_STATE_DIM,)
        assert obs.dtype == np.float64
        assert np.all(np.isfinite(obs))
        assert obs[SLICE_QPOS].shape == (6,)
        assert obs[SLICE_QVEL].shape == (6,)
        assert obs[SLICE_TCP_POS].shape == (3,)
        assert obs[SLICE_TCP_QUAT].shape == (4,)
    finally:
        env.close()


def test_action_space():
    """action space 为 Box(-1,1,(6,))。"""
    env = gym.make("SO101Reach-v1")
    try:
        assert env.action_space.shape == (6,)
        assert np.all(env.action_space.low == -1.0)
        assert np.all(env.action_space.high == 1.0)
        assert env.action_space.sample().shape == (6,)
    finally:
        env.close()


@pytest.mark.parametrize("env_id", CAMERA_IDS)
def test_camera_obs_is_dict_space(env_id):
    """带相机 id: obs 为 Dict{state, rgb{fixed, wrist}}，128x128 uint8。"""
    env = gym.make(env_id)
    try:
        obs, _ = env.reset(seed=0)
        assert isinstance(obs, dict)
        assert set(obs.keys()) == {"state", "rgb"}
        assert set(obs["rgb"].keys()) == {"fixed", "wrist"}
        for img in obs["rgb"].values():
            assert img.shape == (128, 128, 3)
            assert img.dtype == np.uint8
        assert env.observation_space.contains(obs)
        obs, _, _, _, _ = env.step(env.action_space.sample())
        assert env.observation_space.contains(obs)
    finally:
        env.close()


@pytest.mark.parametrize("env_id", STATE_IDS)
def test_state_ids_are_flat(env_id):
    """无相机 id: obs 为扁平向量。"""
    env = gym.make(env_id)
    try:
        obs, _ = env.reset(seed=0)
        assert isinstance(obs, np.ndarray)
    finally:
        env.close()


def test_domain_randomization_applied():
    """DR 开启: 物体摩擦与颜色在 reset 后被随机化（安全可变字段）。"""
    from so101_envs.randomize import RandomizationConfig
    from so101_envs.reach import SO101ReachEnv

    cfg = RandomizationConfig(randomize_item_color=True, item_friction_range=(0.5, 1.5))
    env = SO101ReachEnv(cfg=cfg)
    try:
        env.reset(seed=42)
        gid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "item_geom")
        f1 = float(env.model.geom_friction[gid][0])
        c1 = env.model.geom_rgba[gid][:3].copy()
        env.reset(seed=43)
        f2 = float(env.model.geom_friction[gid][0])
        assert f1 != f2 or not np.allclose(c1, env.model.geom_rgba[gid][:3])
        assert 0.5 <= f2 <= 1.5
    finally:
        env.close()


def test_item_color_dr_reproducible_across_resets():
    """M1: 同一 seed 多次 reset, 物体颜色应一致（不得增量累积漂移）。"""
    from so101_envs.randomize import RandomizationConfig
    from so101_envs.reach import SO101ReachEnv

    cfg = RandomizationConfig(randomize_item_color=True)
    env = SO101ReachEnv(cfg=cfg)
    try:
        gid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "item_geom")
        env.reset(seed=1)
        c1 = env.model.geom_rgba[gid][:3].copy()
        env.reset(seed=1)
        c2 = env.model.geom_rgba[gid][:3].copy()
        env.reset(seed=1)
        c3 = env.model.geom_rgba[gid][:3].copy()
        assert np.allclose(c1, c2) and np.allclose(c2, c3), f"{c1} {c2} {c3}"
    finally:
        env.close()


def test_fixed_cam_noise_reproducible_across_resets():
    """M2: 同一 seed 多次 reset, fixed 相机位姿应一致（不得累积漂移）。"""
    from so101_envs.randomize import RandomizationConfig
    from so101_envs.reach import SO101ReachEnv

    cfg = RandomizationConfig(fixed_camera_pos_noise=0.02)
    env = SO101ReachEnv(cfg=cfg)
    try:
        cid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed_cam")
        env.reset(seed=7)
        p1 = env.model.cam_pos[cid].copy()
        env.reset(seed=7)
        p2 = env.model.cam_pos[cid].copy()
        env.reset(seed=7)
        p3 = env.model.cam_pos[cid].copy()
        assert np.allclose(p1, p2) and np.allclose(p2, p3), f"{p1} {p2} {p3}"
    finally:
        env.close()
