"""接触检测与四元数工具（squint 同款思路, mujoco 实现）。"""

from __future__ import annotations

import mujoco
import numpy as np

# 两"指"：gripper（固定爪）+ moving_jaw（活动爪）
ROBOT_BODIES = frozenset(
    ["base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"]
)

_MIN_FORCE = 0.5  # N


def mat2quat(R):
    """3x3 旋转矩阵 -> wxyz 四元数。"""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.reshape(-1))
    return q


def _pair_force(model, data, b1_id, b2_id):
    """两 body 间接触法向力之和 (N)。参数为 body id。"""
    total = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = int(c.geom1)
        g2 = int(c.geom2)
        ids = {model.geom_bodyid[g1], model.geom_bodyid[g2]}
        if ids == {b1_id, b2_id} and b1_id != b2_id:
            f = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, f)
            total += float(np.linalg.norm(f[:3]))
    return total


def _body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def is_grasping(model, data, item_body, min_force=_MIN_FORCE):
    """双"指"对物体均有 >=min_force 接触力 -> 抓住。"""
    item = _body_id(model, item_body)
    fixed = _pair_force(model, data, _body_id(model, "gripper"), item)
    moving = _pair_force(model, data, _body_id(model, "moving_jaw_so101_v1"), item)
    return bool(fixed >= min_force and moving >= min_force)


def is_touching(model, data, geom_name, min_force=0.2):
    """任一机器人 geom 与指定 geom 有 >=min_force 接触力。"""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if gid < 0:
        return False
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = int(c.geom1)
        g2 = int(c.geom2)
        if gid not in (g1, g2):
            continue
        other = g2 if g1 == gid else g1
        if model.geom_bodyid[other] in ROBOT_BODIES:
            f = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, f)
            if np.linalg.norm(f[:3]) >= min_force:
                return True
    return False


def is_static(data, qvel_addrs, threshold=0.15):
    """机器人关节速度均低于阈值 (rad/s)。"""
    v = data.qvel[qvel_addrs]
    return bool(np.max(np.abs(v)) <= threshold)


def is_body_static(model, data, body, threshold=0.15):
    """自由物体（freejoint body）速度均低于阈值 (m/s 与 rad/s)。"""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    vadr = model.jnt_dofadr[model.body_jntadr[bid]]
    v = data.qvel[vadr:vadr + 6]
    return bool(np.max(np.abs(v)) <= threshold)
