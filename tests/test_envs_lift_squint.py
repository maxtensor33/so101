"""lift 严格对齐 squint 的语义测试（workflow/plan/envs.md, 2026-10-31）。

锁定项:
- 物体尺寸/密度/摩擦（squint LiftRandomizationConfig）
- obs 布局（item_pose/target_qpos/dist_to_rest/is_grasped/item 尺寸/摩擦/密度）
- success 判据（target_qpos 距 rest < 0.2, 非实际 qpos）
- dense reward 公式与 /3 归一化
- success 不终止 episode（squint ManiSkillVectorEnv ignore_terminations）
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym
import mujoco
import numpy as np
import pytest

import so101_envs  # noqa: F401  注册副作用
from so101_envs.base_env import REST_QPOS
from so101_envs.lift import ITEM_DENSITY, REST_THRESH


def _item_geom(u):
    return mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_GEOM, "item_geom")


def test_default_geometry_matches_squint_midpoint():
    """DR 关: cube half=0.0125（squint range 中点）, 密度 200 -> 3.125g, 摩擦 0.3。"""
    env = gym.make("SO101Lift-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        gid = _item_geom(u)
        assert np.allclose(u.model.geom_size[gid], 0.0125)
        assert u.model.body_mass[u.model.geom_bodyid[gid]] == pytest.approx(0.003125)
        assert u.model.geom_friction[gid][0] == pytest.approx(0.3)
    finally:
        env.close()


@pytest.mark.parametrize("env_id", ["SO101Lift-v1", "SO101LiftCan-v1"])
def test_dr_geometry_within_squint_ranges(env_id):
    """DR 开: 尺寸/摩擦逐 episode 落在 squint range, 质量与密度 200 自洽。"""
    env = gym.make(env_id, domain_randomization=True)
    try:
        for seed in range(5):
            env.reset(seed=seed)
            u = env.unwrapped
            gid = _item_geom(u)
            bid = int(u.model.geom_bodyid[gid])
            mass = float(u.model.body_mass[bid])
            friction = float(u.model.geom_friction[gid][0])
            assert 0.1 <= friction <= 0.5
            if env_id == "SO101Lift-v1":
                h = float(u.model.geom_size[gid][0])
                assert 0.011 <= h <= 0.014
                assert mass == pytest.approx(ITEM_DENSITY * (2 * h) ** 3)
            else:
                r, hh = u.model.geom_size[gid][:2]
                assert 0.014 <= r <= 0.019 and 0.025 <= hh <= 0.035
                assert mass == pytest.approx(ITEM_DENSITY * np.pi * r ** 2 * 2 * hh)
    finally:
        env.close()


def test_lift_obs_layout_and_dim():
    """state 52 维 = 基础 19 + 任务 33（target/dist_rest/grasped/尺寸/摩擦/密度/clean_qpos）。"""
    env = gym.make("SO101Lift-v1")
    try:
        obs, _ = env.reset(seed=3)
        u = env.unwrapped
        assert obs.shape == (52,) and obs.dtype == np.float64
        target = obs[29:35]
        dist_rest = obs[35:40]
        grasped = obs[40]
        dims = obs[41:44]
        friction = obs[44]
        density = obs[45]
        clean_qpos = obs[46:52]
        assert np.allclose(target, u.target_qpos)
        assert np.allclose(dist_rest, target[:5] - REST_QPOS[:5])
        assert grasped == 0.0
        assert np.allclose(dims, 0.0125)
        assert friction == pytest.approx(0.3)
        assert density == ITEM_DENSITY
        assert np.allclose(clean_qpos, u.data.qpos[u._qpos_adr])
        assert env.observation_space.contains(obs)
    finally:
        env.close()


def test_success_uses_target_qpos_not_actual_qpos():
    """squint 用控制器 target_qpos 判 rest, 实际 qpos 偏离不影响该分量。"""
    env = gym.make("SO101Lift-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        u.data.qpos[u._qpos_adr[:5]] = REST_QPOS[:5] + 1.0  # 实际关节远离 rest
        mujoco.mj_forward(u.model, u.data)
        info = u.evaluate()
        # target 仍在 reset 的近 rest 处
        assert info["distance_to_rest_qpos"] < REST_THRESH
    finally:
        env.close()


def test_dense_reward_matches_squint_normalized_formula():
    """reward = [1-tanh(5d) + g + exp(-2*drest)*g - 3*touch - (not lifted)] / 3。"""
    env = gym.make("SO101Lift-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        info = u.evaluate()
        expected = (
            1.0 - np.tanh(5.0 * info["tcp_to_item_dist"])
            + float(info["is_item_grasped"])
            + np.exp(-2.0 * info["distance_to_rest_qpos"]) * float(info["is_item_grasped"])
            - 3.0 * float(info["robot_touching_table"])
            - 1.0 * float(not info["item_lifted"])
        ) / 3.0
        assert u.compute_dense_reward(np.zeros(6), info) == pytest.approx(expected)
    finally:
        env.close()


def _tip_midpoint(u):
    """squint TCP: 固定指尖与动爪指尖中点（URDF finger1_tip/finger2_tip）。"""
    g = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    j = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
    f1 = u.data.xpos[g] + u.data.xmat[g].reshape(3, 3) @ np.array([-0.002, 0, -0.092])
    f2 = u.data.xpos[j] + u.data.xmat[j].reshape(3, 3) @ np.array([-0.01, -0.077, 0.02])
    return (f1 + f2) / 2


def _max_item_finger_penetration(u):
    """物体-手指接触的最大穿透深度 (mm)。"""
    item = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, "item")
    fingers = {mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, name)
               for name in ("gripper", "moving_jaw_so101_v1")}
    worst = 0.0
    for i in range(u.data.ncon):
        c = u.data.contact[i]
        bodies = {int(u.model.geom_bodyid[int(c.geom1)]),
                  int(u.model.geom_bodyid[int(c.geom2)])}
        if item in bodies and bodies <= (fingers | {item}):
            worst = min(worst, float(c.dist))
    return -worst * 1000.0


def test_grasp_holds_after_release():
    """物理保持回归: 指尖中点夹持 → 释放 → 30 步不下滑且穿透 < 0.5mm。

    夹爪目标给到 0 rad（不压到全闭合），方块中心放在**动态两指尖中点**（squint TCP）;
    过大闭合目标（>=0.3）或按 gripperframe site 放置都会挤出/滑落。
    """
    env = gym.make("SO101Lift-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        for _ in range(10):  # settle 同时保持 rest 位形（零动作会驱动到限位中点）
            env.step(u.normalize_qpos(u.data.qpos[u._qpos_adr]))
        a = u.normalize_qpos(u.data.qpos[u._qpos_adr])
        a[5] = u.normalize_qpos(REST_QPOS)[5]  # squint rest 60°
        for _ in range(15):
            env.step(a)
        target = u.data.qpos[u._qpos_adr].copy()
        target[5] = 0.0
        for _ in range(15):  # 收爪到 0 rad，同时把方块钉在动态指尖中点
            u.set_free_body_pose("item", _tip_midpoint(u), yaw=0.0)
            mujoco.mj_forward(u.model, u.data)
            env.step(u.normalize_qpos(target))
        # 释放：保持当前位形/夹爪目标，物体自由
        z0 = u.free_body_pos("item")[2]
        held = 0
        info = {}
        for _ in range(30):
            *_, info = env.step(u.normalize_qpos(target))
            held += int(info["is_item_grasped"])
        z1 = u.free_body_pos("item")[2]
        assert held == 30, f"保持丢失: {held}/30, z {z0 * 1000:.1f}->{z1 * 1000:.1f}mm"
        assert abs(z1 - z0) < 0.005, f"物体下滑 {abs(z1 - z0) * 1000:.1f}mm"
        assert info["success"] is True
        pen = _max_item_finger_penetration(u)
        assert pen < 0.5, f"穿模 {pen:.2f}mm"
    finally:
        env.close()


def test_success_does_not_terminate_episode():
    """squint ignore_terminations: lift 不因 success 提前终止（50 步 TimeLimit 截断）。"""
    from so101_envs.lift import SO101LiftEnv

    assert SO101LiftEnv.TERMINATE_ON_SUCCESS is False
    env = gym.make("SO101Lift-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        info = dict(u.evaluate(), success=True)
        u.evaluate = lambda: info  # 构造 success=True, 验证 step 不返回 terminated
        _, _, terminated, truncated, _ = env.step(np.zeros(6))
        assert terminated is False and truncated is False
    finally:
        env.close()
