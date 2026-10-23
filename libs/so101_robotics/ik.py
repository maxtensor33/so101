"""IK 模块 (workflow/plan/robotics.md Phase 2, 对标 lab-2 a4_inverse_kinematics.py)。

DLS (阻尼最小二乘) 与伪逆两种数值 IK:
  - target 为 pin.SE3: 6D (位置+姿态), rot_weight 可调 (位置优先用小权重)
  - target 为 (3,) 数组: 仅位置
  - 自适应阻尼: σ_min 越小 λ 越大; 联合限位钳位与随机重启
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from . import common, fk, jacobian

LAMBDA0 = 1e-3


@dataclass
class IKResult:
    q: np.ndarray
    pos_err: float
    rot_err: float
    iters: int
    converged: bool


def adaptive_damping(q) -> float:
    """σ_min 越小阻尼越大: λ = λ0 + k·max(0, σ_thresh - σ_min)。"""
    sigma_min = jacobian.singularity_index(q)
    sigma_thresh = 1e-2
    k = 10.0
    return float(LAMBDA0 + k * max(0.0, sigma_thresh - sigma_min))


def _error_and_jacobian(target, q, rot_weight):
    """误差向量与加权 Jacobian。仅位置目标时返回 3 维误差与 3 行 J。"""
    se3 = fk.pinocchio_fk(q)
    J = jacobian.pinocchio_jacobian(q)
    if isinstance(target, pin.SE3):
        e_pos = target.translation - se3.translation
        e_rot = pin.log3(target.rotation @ se3.rotation.T)
        err = np.concatenate([e_pos, rot_weight * e_rot])
        J = np.vstack([J[:3], rot_weight * J[3:]])
        pos_err = float(np.linalg.norm(e_pos))
        rot_err = float(np.linalg.norm(e_rot))
    else:
        target = np.asarray(target, dtype=float)
        e_pos = target - se3.translation
        err = e_pos
        J = J[:3]
        pos_err = float(np.linalg.norm(e_pos))
        rot_err = 0.0
    return err, J, pos_err, rot_err


def _attempt(target, q_init, lam_override, max_iters, tol_pos, tol_rot, adaptive, rot_weight):
    q = np.asarray(q_init, dtype=float).copy()
    limits = common.get_joint_limits()
    lo, hi = limits[:, 0], limits[:, 1]
    q = np.clip(q, lo, hi)
    converged = False
    iters = 0
    pos_err = float("inf")
    rot_err = float("inf")
    for it in range(max_iters):
        iters = it + 1
        err, J, pos_err, rot_err = _error_and_jacobian(target, q, rot_weight)
        if pos_err < tol_pos and (not isinstance(target, pin.SE3) or rot_err < tol_rot):
            converged = True
            break
        lam = lam_override
        if lam is None:
            lam = adaptive_damping(q)
        m = err.size
        A = J @ J.T + lam * lam * np.eye(m)
        dq = J.T @ np.linalg.solve(A, err)
        dq = np.clip(dq, -0.5, 0.5)
        q = np.clip(q + dq, lo, hi)
    return q, pos_err, rot_err, iters, converged


def _with_restarts(target, q_init, attempt_fn, n_restarts, seed):
    if q_init is None:
        q_init = common.Q_HOME
    result = attempt_fn(q_init)
    if result[4] or n_restarts <= 0:
        return IKResult(*result)
    limits = common.get_joint_limits()
    rng = np.random.default_rng(seed)
    for _ in range(n_restarts):
        q0 = rng.uniform(limits[:, 0], limits[:, 1])
        result = attempt_fn(q0)
        if result[4]:
            return IKResult(*result)
    return IKResult(*result)


def ik_dls(target, q_init=None, max_iters=200, tol_pos=1e-3, tol_rot=1e-2,
           adaptive=True, rot_weight=1.0, n_restarts=3, seed=0):
    """阻尼最小二乘 IK (随机重启)。target: pin.SE3 (6D) 或 (3,) (仅位置)。"""
    return _with_restarts(
        target, q_init,
        lambda q0: _attempt(target, q0, None, max_iters, tol_pos, tol_rot, adaptive, rot_weight),
        n_restarts, seed,
    )


def ik_pinv(target, q_init=None, max_iters=200, tol_pos=1e-3, tol_rot=1e-2,
            rot_weight=1.0, n_restarts=3, seed=0):
    """伪逆 IK (λ=0 的 DLS 特例)。"""
    return _with_restarts(
        target, q_init,
        lambda q0: _attempt(target, q0, 0.0, max_iters, tol_pos, tol_rot, False, rot_weight),
        n_restarts, seed,
    )