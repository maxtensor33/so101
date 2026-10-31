"""任务级行为测试（workflow/plan/envs.md Phase 1/2/4, TDD）。

构造性用例：直接操控世界状态验证 evaluate() 与 reward 语义。
Phase 4 增补 pinocchio IK 脚本专家端到端用例。
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import so101_envs  # noqa: F401  注册副作用
import gymnasium as gym
import mujoco
import numpy as np
import pytest


def _move_goal_to_tcp(env):
    """reach: 把 goal site 移到当前 TCP 处。"""
    bid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "goal_site")
    m = env.model.body_mocapid[bid]
    env.data.mocap_pos[m] = env.tcp_pos


def test_reach_teleported_goal_success():
    """goal 移到 TCP 处 + 夹爪闭合 -> is_reached 且 success（squint 语义）。"""
    env = gym.make("SO101Reach-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        hold = u.normalize_qpos(u.data.qpos[u._qpos_adr])
        hold[5] = -1.0  # squint: success 要求夹爪闭合
        for _ in range(10):  # settle 到静止并闭合夹爪
            *_, info = env.step(hold)
        assert info["is_static"] and info["is_gripper_closed"]
        _move_goal_to_tcp(u)
        u._goal = u.tcp_pos.copy()  # 同步 obs 内的 goal 缓存
        obs, reward, terminated, truncated, info = env.step(hold)
        assert info["is_reached"] is True
        assert info["success"] is True
        assert terminated is False  # squint: success 不终止 episode
    finally:
        env.close()


# ---------------- Phase 2: lift / place / stack ----------------

def _settle(env, n=10):
    info = {}
    for _ in range(n):
        *_, info = env.step(np.zeros(6))
    return info


def _hold_current_pose(u):
    """动作 = 当前 qpos（保持不动），夹爪全开。"""
    a = u.normalize_qpos(u.data.qpos[u._qpos_adr])
    a[5] = u.normalize_qpos(np.array([0, 0, 0, 0, 0, u._ctrlrange[5, 1]]))[5]
    return a


def _close_gripper(u, other):
    a = other.copy()
    a[5] = u.normalize_qpos(np.array([0, 0, 0, 0, 0, u._ctrlrange[5, 0]]))[5]
    return a


def test_lift_grasp_teleported_item():
    """物体放到爪间 + 闭合 -> 闭合过程中 is_grasped 至少触发一次。

    注: mujoco mesh 凸包夹持对闭合深度敏感（过闭合会把物体沿爪挤出），
    "持续抓握"由 Phase 4 脚本专家以合适闭合深度端到端验证。
    """
    env = gym.make("SO101Lift-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        _settle(env)
        # 夹爪全开
        act = _hold_current_pose(u)
        act[5] = 1.0
        for _ in range(15):
            env.step(act)
        # 物体放到夹持中心（硬接触后几何须精确, 旧偏移 [0.02,0,-0.07] 靠软接触压穿过关）
        g = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        Rg = u.data.xmat[g].reshape(3, 3)
        # 夹持中心 (硬接触下须精确置于爪间, 实测见 todo/envs_grasp_tuning_todo.md)
        target = u.data.xpos[g] + Rg @ np.array([-0.0068, 0.0024, -0.1006])
        u.set_free_body_pose("item", target, yaw=0.0)
        mujoco.mj_forward(u.model, u.data)
        # 渐进闭合
        # 硬接触下每帧把方块钉在夹持中心 (gripperframe site = 闭合夹持点):
        # reset 姿态爪朝上, 不钉则方块在闭爪完成前自由坠落脱离夹持区
        # (旧软接触靠 21mm 压穿兜住坠落方块, 修复穿模后该捷径不存在)
        sid = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        seen = False
        for _ in range(40):
            a = u.normalize_qpos(u.data.qpos[u._qpos_adr])
            a[5] = max(a[5] - 0.06, -1.0)
            u.set_free_body_pose("item", u.data.site_xpos[sid].copy(), yaw=0.0)
            mujoco.mj_forward(u.model, u.data)
            *_, info = env.step(a)
            seen = seen or info["is_grasped"]
        assert seen, "闭合过程中从未检测到 is_grasped"
    finally:
        env.close()


def test_place_item_in_bin_success():
    """物体放进 bin -> in_bin & success（未抓 & 静止）。"""
    env = gym.make("SO101Place-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        info = _settle(env)
        bin_pos = u.free_body_pos("bin")
        u.set_free_body_pose("item", bin_pos + np.array([0, 0, 0.02]), yaw=0.3)
        mujoco.mj_forward(u.model, u.data)
        info = _settle(env, 8)
        assert info["in_bin"] is True
        assert info["success"] is True
    finally:
        env.close()


def test_stack_A_on_B_success():
    """A 放到 B 正上方 -> success。"""
    env = gym.make("SO101Stack-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        _settle(env)
        b = u.free_body_pos("itemB")
        u.set_free_body_pose("itemA", b + np.array([0, 0, 0.022 + 0.028 + 0.002]), yaw=0.5)
        mujoco.mj_forward(u.model, u.data)
        info = _settle(env, 10)
        assert info["on_top"] is True
        assert info["success"] is True
    finally:
        env.close()


def test_reach_goal_mocap_synced():
    """reset 后 goal_site mocap 位置与逻辑 goal（判定点）一致。"""
    env = gym.make("SO101Reach-v1")
    try:
        env.reset(seed=3)
        u = env.unwrapped
        bid = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, "goal_site")
        m = u.model.body_mocapid[bid]
        assert m >= 0
        mocap = u.data.mocap_pos[m]
        assert np.allclose(mocap, u._goal, atol=1e-9), f"mocap={mocap}, goal={u._goal}"
    finally:
        env.close()


def test_stack_success_requires_itemA_static():
    """M3: itemA 在 B 上但自身仍在运动 -> success 必须为 False。"""
    env = gym.make("SO101Stack-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        _settle(env)
        b = u.free_body_pos("itemB")
        a_pos = b + np.array([0, 0, u._item_a_half_z + u._item_b_half_z + 0.001])
        u.set_free_body_pose("itemA", a_pos, yaw=0.0)
        mujoco.mj_forward(u.model, u.data)
        # 给 itemA 一个横向速度（未静止）
        jadr = u.model.body_jntadr[mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, "itemA")]
        vadr = u.model.jnt_dofadr[jadr]
        u.data.qvel[vadr] = 0.3
        mujoco.mj_forward(u.model, u.data)
        info = u.evaluate()
        assert info["on_top"] is True
        assert info["item_static"] is False
        assert info["success"] is False
    finally:
        env.close()


# ---------- can 变体（squint 8 环境对齐: item_type 换物体, 任务逻辑复用） ----------

CAN_IDS = [
    "SO101ReachCan-v1",
    "SO101LiftCan-v1",
    "SO101PlaceCan-v1",
    "SO101StackCan-v1",
]


@pytest.mark.parametrize("env_id", CAN_IDS)
def test_can_variant_smoke(env_id):
    """can 变体冒烟: reset/step 闭环 + info 键 + 圆柱罐出生在桌面上方。"""
    env = gym.make(env_id)
    try:
        env.reset(seed=0)
        u = env.unwrapped
        info = _settle(env, 4)
        # 物体中心 z = 各自半高（cube half / can half-h; squint 尺寸）
        for body, geom in (("item", "item_geom"), ("itemA", "itemA_geom"),
                           ("itemB", "itemB_geom")):
            bid = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_BODY, body)
            if bid < 0 or u.model.body_jntadr[bid] < 0:
                continue  # 场景无此物体或非自由体
            gid = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_GEOM, geom)
            half = float(u.model.geom_size[gid][2])  # box h / cylinder half-length
            p = u.free_body_pos(body)
            assert p[2] > half - 0.01, f"{body} 未立在桌面: z={p[2]:.4f}"
        assert "success" in info
    finally:
        env.close()


def test_lift_can_item_half_used_in_evaluate():
    """lift-can 的 lifted 判定用罐半高 0.03（而非 cube 0.022）。"""
    env = gym.make("SO101LiftCan-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        u.set_free_body_pose("item", (0.3, 0.0, 0.032), yaw=0.0)
        mujoco.mj_forward(u.model, u.data)
        info = u.evaluate()
        # squint: lifted = z >= half_z + 1e-3 -> 0.032 > 0.03 + 1e-3
        assert info["item_lifted"] is True
    finally:
        env.close()


def test_stack_can_on_top_semantics():
    """stack-can 的 on_top 判定: cube A 中心距罐 B 顶 = HALF_A + 罐半高。"""
    env = gym.make("SO101StackCan-v1")
    try:
        env.reset(seed=0)
        u = env.unwrapped
        _settle(env)
        b = u.free_body_pos("itemB")
        a_pos = b + np.array([0, 0, u._item_a_half_z + u._item_b_half_z + 0.001])
        u.set_free_body_pose("itemA", a_pos, yaw=0.0)
        mujoco.mj_forward(u.model, u.data)
        info = u.evaluate()
        assert info["on_top"] is True
    finally:
        env.close()
