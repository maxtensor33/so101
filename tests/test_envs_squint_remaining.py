"""reach/place/stack 严格对齐 squint 的语义测试（2026-10-31）。

锁定项: 物体尺寸/密度/摩擦、出生域、success 判据、reward 归一化、
bin 随机尺寸/yaw、stack 动态半高。
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym
import mujoco
import numpy as np
import pytest

import so101_envs  # noqa: F401  注册副作用


def _geom_size(u, name):
    gid = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_GEOM, name)
    return u.model.geom_size[gid].copy()


def _geom_mass(u, name):
    gid = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_GEOM, name)
    return float(u.model.body_mass[int(u.model.geom_bodyid[gid])])


def _geom_friction(u, name):
    gid = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_GEOM, name)
    return float(u.model.geom_friction[gid][0])


def _hold(u):
    return u.normalize_qpos(u.data.qpos[u._qpos_adr])


# ---------------- reach ----------------

def test_reach_defaults_and_closed_gripper_success():
    """squint reach: cube half=0.0125/3.125g/0.3; goal=2h+0.02; 夹爪闭合才 success。"""
    env = gym.make("SO101Reach-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        assert np.allclose(_geom_size(u, "item_geom"), 0.0125)
        assert _geom_mass(u, "item_geom") == pytest.approx(0.003125)
        assert _geom_friction(u, "item_geom") == pytest.approx(0.3)
        assert u._goal[2] == pytest.approx(2 * u._item_half_z + 0.02)

        # 夹爪闭合 + goal 移到 TCP -> success
        a = _hold(u)
        a[5] = -1.0
        for _ in range(10):
            *_, info = env.step(a)
        bid = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, "goal_site")
        u.data.mocap_pos[u.model.body_mocapid[bid]] = u.tcp_pos
        u._goal = u.tcp_pos.copy()
        *_, info = env.step(a)
        assert info["is_gripper_closed"] is True
        assert info["success"] is True

        # 夹爪张开 -> 不满足闭爪条件（张开也会移动指尖中点 TCP）
        a[5] = 1.0
        for _ in range(10):
            *_, info = env.step(a)
        assert info["is_gripper_closed"] is False
        assert info["success"] is False
    finally:
        env.close()


@pytest.mark.parametrize("env_id", ["SO101Reach-v1", "SO101ReachCan-v1"])
def test_reach_dr_ranges(env_id):
    env = gym.make(env_id, domain_randomization=True)
    try:
        for seed in range(4):
            env.reset(seed=seed)
            u = env.unwrapped
            assert 0.1 <= _geom_friction(u, "item_geom") <= 0.5
            if env_id == "SO101Reach-v1":
                h = float(_geom_size(u, "item_geom")[0])
                assert 0.011 <= h <= 0.014
                assert _geom_mass(u, "item_geom") == pytest.approx(200 * (2 * h) ** 3)
            else:
                r, hh = _geom_size(u, "item_geom")[:2]
                assert 0.014 <= r <= 0.019 and 0.025 <= hh <= 0.035
                assert _geom_mass(u, "item_geom") == pytest.approx(
                    200 * np.pi * r ** 2 * 2 * hh)
    finally:
        env.close()


# ---------------- place ----------------

def test_place_defaults_and_bin_success():
    """squint place: 物体默认 half=0.0125; bin 默认 0.04/0.05/0.015; bin 本地坐标 success。"""
    env = gym.make("SO101Place-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        assert np.allclose(_geom_size(u, "item_geom"), 0.0125)
        assert _geom_mass(u, "item_geom") == pytest.approx(0.003125)
        assert np.allclose(u._bin_dims, [0.04, 0.05, 0.015])
        for _ in range(10):
            env.step(_hold(u))
        bp = u.free_body_pos("bin")
        u.set_free_body_pose("item", (bp[0], bp[1], u._bin_thickness + u._item_half_z),
                             yaw=0.0)
        mujoco.mj_forward(u.model, u.data)
        info = u.evaluate()
        assert info["in_bin"] is True and info["success"] is True
    finally:
        env.close()


def test_place_dr_ranges_and_no_overlap():
    env = gym.make("SO101Place-v1", domain_randomization=True)
    try:
        for seed in range(5):
            env.reset(seed=seed)
            u = env.unwrapped
            h = float(_geom_size(u, "item_geom")[0])
            assert 0.011 <= h <= 0.014
            bx, by, bz = u._bin_dims
            assert 0.035 <= bx <= 0.045 and 0.045 <= by <= 0.055
            assert 0.012 <= bz <= 0.018
            d = np.linalg.norm(u.free_body_pos("item")[:2] - u.free_body_pos("bin")[:2])
            assert d > h  # 至少不重叠
    finally:
        env.close()


def test_place_success_reward_is_normalized_one():
    env = gym.make("SO101Place-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        info = dict(u.evaluate(), success=True, item_lifted=True)
        assert u.compute_dense_reward(np.zeros(6), info) == pytest.approx(1.0)
    finally:
        env.close()


# ---------------- stack ----------------

def test_stack_defaults_and_dynamic_half_success():
    """squint stack: A half=0.0125, B cube half=0.015; success 用半高之和。"""
    env = gym.make("SO101Stack-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        assert np.allclose(_geom_size(u, "itemA_geom"), 0.0125)
        assert np.allclose(_geom_size(u, "itemB_geom"), 0.015)
        assert u._item_a_half_z == pytest.approx(0.0125)
        assert u._item_b_half_z == pytest.approx(0.015)
        for _ in range(10):
            env.step(_hold(u))
        b = u.free_body_pos("itemB")
        a = b + np.array([0, 0, u._item_a_half_z + u._item_b_half_z + 0.0005])
        u.set_free_body_pose("itemA", a, yaw=0.0)
        u.data.qvel[:] = 0
        mujoco.mj_forward(u.model, u.data)
        info = u.evaluate()
        assert info["on_top"] is True and info["success"] is True
    finally:
        env.close()


@pytest.mark.parametrize("env_id", ["SO101Stack-v1", "SO101StackCan-v1"])
def test_stack_dr_ranges(env_id):
    env = gym.make(env_id, domain_randomization=True)
    try:
        for seed in range(4):
            env.reset(seed=seed)
            u = env.unwrapped
            hA = float(_geom_size(u, "itemA_geom")[0])
            assert 0.011 <= hA <= 0.014
            if env_id == "SO101Stack-v1":
                hB = float(_geom_size(u, "itemB_geom")[0])
                assert 0.0125 <= hB <= 0.0175
            else:
                r, hh = _geom_size(u, "itemB_geom")[:2]
                assert 0.014 <= r <= 0.019 and 0.025 <= hh <= 0.035
    finally:
        env.close()


def test_stack_success_reward_is_normalized_one():
    env = gym.make("SO101Stack-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        info = dict(u.evaluate(), success=True, is_itemA_lifted=True)
        assert u.compute_dense_reward(np.zeros(6), info) == pytest.approx(1.0)
    finally:
        env.close()
