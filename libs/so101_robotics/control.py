"""control 模块 (plan/robotics.md Phase 3, 对标 lab-2 b2_control_hierarchy.py)。

力矩控制器族 (学习梯度): PD+重力 → computed torque → 任务空间阻抗。
全部输出 6 维关节力矩, 上层用 constraints.saturate_torque 饱和。
"""

import numpy as np
import pinocchio as pin

from . import common, dynamics, jacobian


def pd_gravity(q, dq, q_des, dq_des, kp, kd):
    """PD + 重力补偿: tau = g(q) + Kp(q_des-q) + Kd(dq_des-dq)。"""
    q = np.asarray(q, dtype=float)
    e = np.asarray(q_des, dtype=float) - q
    de = np.asarray(dq_des, dtype=float) - np.asarray(dq, dtype=float)
    kp = np.asarray(kp, dtype=float)
    kd = np.asarray(kd, dtype=float)
    return dynamics.gravity(q) + kp * e + kd * de


def computed_torque(q, dq, q_des, dq_des, ddq_des, kp, kd, compensate_passive=True):
    """反馈线性化: tau = M(q)(ddq_des + Kp e + Kd de) + C+g + 被动阻尼补偿。"""
    q = np.asarray(q, dtype=float)
    dq = np.asarray(dq, dtype=float)
    e = np.asarray(q_des, dtype=float) - q
    de = np.asarray(dq_des, dtype=float) - dq
    a_des = np.asarray(ddq_des, dtype=float) + np.asarray(kp, dtype=float) * e \
        + np.asarray(kd, dtype=float) * de
    tau = dynamics.mass_matrix(q) @ a_des + dynamics.nle(q, dq)
    if compensate_passive:
        # MJCF 关节 damping (0.6 Nms/rad) 是仿真真实存在的被动项
        tau = tau + common.get_mujoco_model().dof_damping[:6] * dq
    return tau


def task_space_impedance(q, dq, x_des, dx_des=None, kp_lin=60.0, kd_lin=14.0):
    """任务空间阻抗: F = Kp(x_des-x) + Kd(dx_des-dx), tau = J^T F + g(q)。"""
    q = np.asarray(q, dtype=float)
    dq = np.asarray(dq, dtype=float)
    x = fk_ee_pos(q)
    J = jacobian.pinocchio_jacobian(q)
    dx = J[:3] @ dq
    if dx_des is None:
        dx_des = np.zeros(3)
    F = kp_lin * (np.asarray(x_des, dtype=float) - x) \
        + kd_lin * (np.asarray(dx_des, dtype=float) - dx)
    return J[:3].T @ F + dynamics.gravity(q)


def fk_ee_pos(q):
    import mujoco

    from . import fk

    return np.asarray(fk.mujoco_fk(q).translation, dtype=float)
