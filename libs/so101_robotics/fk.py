"""FK 模块 (plan/robotics.md Phase 1, 对标 lab-2 a2_forward_kinematics.py)。

Pinocchio 与 MuJoCo 双引擎 FK + 随机配置交叉验证。
两模型已在 Phase 0 验证为运动学一致。
"""

import mujoco
import numpy as np
import pinocchio as pin

from . import common


def pinocchio_fk(q, frame=None):
    model = common.get_pinocchio_model()
    data = common.get_pin_data()
    q = np.asarray(q, dtype=float)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    if frame is None:
        frame = common.EE_FRAME
    fid = model.getFrameId(frame, pin.BODY)
    return data.oMf[fid].copy()


def mujoco_fk(q, body=None):
    model = common.get_mujoco_model()
    data = common.get_mj_data()
    q = np.asarray(q, dtype=float)
    for k, name in enumerate(common.JOINT_NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = q[k]
    mujoco.mj_forward(model, data)
    if body is None:
        body = common.EE_BODY
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    rot = data.xmat[bid].reshape(3, 3).copy()
    pos = data.xpos[bid].copy()
    return pin.SE3(rot, pos)


def cross_check(n_samples=100, seed=0):
    """随机配置下双引擎 FK 交叉验证。返回 (max_pos_err, max_rot_err)。"""
    limits = common.get_joint_limits()
    rng = np.random.default_rng(seed)
    max_pos = 0.0
    max_rot = 0.0
    for _ in range(n_samples):
        q = rng.uniform(limits[:, 0], limits[:, 1])
        for link, body in common.LINK_BODY_MAP.items():
            p = pinocchio_fk(q, frame=link)
            m = mujoco_fk(q, body=body)
            e_pos = float(np.linalg.norm(p.translation - m.translation))
            e_rot = float(np.linalg.norm(pin.log3(p.rotation @ m.rotation.T)))
            max_pos = max(max_pos, e_pos)
            max_rot = max(max_rot, e_rot)
    return max_pos, max_rot
