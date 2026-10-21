"""Constraints 模块 (plan/robotics.md Phase 3, 对标 lab-2 b3_constraints.py)。

力矩饱和 (STS3215 ±3.35 Nm) 与关节限位约束。
"""

import numpy as np
import mujoco

from . import common

TORQUE_LIMIT = 3.35  # Nm, MJCF actuator forcerange


def saturate_torque(tau, limit=None):
    """力矩饱和钳位。"""
    limit = TORQUE_LIMIT if limit is None else limit
    return np.clip(np.asarray(tau, dtype=float), -limit, limit)


def clamp_joint_target(q_des):
    """期望关节角钳位到关节限位内。"""
    limits = common.get_joint_limits()
    return np.clip(np.asarray(q_des, dtype=float), limits[:, 0], limits[:, 1])