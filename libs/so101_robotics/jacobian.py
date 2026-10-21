"""Jacobian 模块 (对标 lab-2 a3_jacobian.py)。

解析 (Pinocchio) 与数值 (有限差分) Jacobian、奇异性指标、manipulability。
参考系: LOCAL_WORLD_ALIGNED (世界系, 线速度作用于 ee 原点)。
"""

import numpy as np
import pinocchio as pin

from . import common, fk


def pinocchio_jacobian(q, frame=None):
    """世界对齐 (LOCAL_WORLD_ALIGNED) 几何 Jacobian, 6x6。"""
    model = common.get_pinocchio_model()
    data = common.get_pin_data()
    q = np.asarray(q, dtype=float)
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    if frame is None:
        frame = common.EE_FRAME
    fid = model.getFrameId(frame, pin.BODY)
    return pin.getFrameJacobian(
        model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    ).copy()


def numerical_jacobian(q, eps=1e-6):
    """有限差分 Jacobian (6x6), 用于校验解析解。"""
    q = np.asarray(q, dtype=float)
    J = np.zeros((6, 6))
    se3_0 = fk.pinocchio_fk(q)
    for k in range(6):
        e = np.zeros(6)
        e[k] = eps
        se3_1 = fk.pinocchio_fk(q + e)
        J[:3, k] = (se3_1.translation - se3_0.translation) / eps
        J[3:, k] = pin.log3(se3_1.rotation @ se3_0.rotation.T) / eps
    return J


def singular_values(q):
    J = pinocchio_jacobian(q)
    return np.linalg.svd(J, compute_uv=False)


def singularity_index(q):
    return float(singular_values(q).min())


def manipulability(q):
    J = pinocchio_jacobian(q)
    return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
