"""Dynamics 模块 (plan/robotics力矩.md Phase 2, 对标 lab-2 a5_dynamics.py)。

Pinocchio RNEA/CRBA/ABA 与 MuJoCo qfrc_bias/mj_fullM 交叉验证过。
"""

from __future__ import annotations

import mujoco
import numpy as np
import pinocchio as pin

from . import common


def gravity(q):
    """重力矩 g(q) ∈ R^6。"""
    model = common.get_pinocchio_model()
    data = common.get_pin_data()
    q = np.asarray(q, dtype=float)
    return pin.rnea(model, data, q, np.zeros(6), np.zeros(6)).copy()


def mass_matrix(q, include_armature=True):
    """关节空间质量矩阵 M(q) ∈ R^{6x6}。"""
    model = common.get_pinocchio_model()
    data = common.get_pin_data()
    q = np.asarray(q, dtype=float)
    M = np.triu(pin.crba(model, data, q))
    M = M + M.T - np.diag(M.diagonal())
    if include_armature:
        M = M + np.diag(mujoco_armature())
    return M


def nle(q, dq):
    """非线性效应 (C(q,q̇)q̇ + g(q))。"""
    model = common.get_pinocchio_model()
    data = common.get_pin_data()
    q = np.asarray(q, dtype=float)
    dq = np.asarray(dq, dtype=float)
    return pin.rnea(model, data, q, dq, np.zeros(6)).copy()


def rnea(q, dq, ddq):
    """逆动力学: 产生 (q,dq,ddq) 所需力矩。"""
    model = common.get_pinocchio_model()
    data = common.get_pin_data()
    q = np.asarray(q, dtype=float)
    dq = np.asarray(dq, dtype=float)
    ddq = np.asarray(ddq, dtype=float)
    return pin.rnea(model, data, q, dq, ddq).copy()


def aba(q, dq, tau):
    """正动力学: 力矩 → 加速度。"""
    model = common.get_pinocchio_model()
    data = common.get_pin_data()
    tau = np.asarray(tau, dtype=float)
    return pin.aba(model, data, q, dq, tau).copy()


def mujoco_armature() -> np.ndarray:
    """6 个关节的 armature (来自 MJCF sts3215 默认类, 控制建模用)。"""
    return common.get_mujoco_model().dof_armature[:6].copy()
