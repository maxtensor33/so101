"""Jacobian 模块 TDD 测试 (plan/robotics.md Phase 1, 对标 lab-2 A3)。"""

import numpy as np
import pinocchio as pin
import pytest

from so101_robotics import common, fk, jacobian

N_V = 6  # 6 个 1-DOF 关节


def _random_q(seed):
    rng = np.random.default_rng(seed)
    limits = common.get_joint_limits()
    return rng.uniform(limits[:, 0], limits[:, 1])


class TestAnalyticJacobian:
    def test_shape(self):
        J = jacobian.pinocchio_jacobian(common.Q_HOME)
        assert J.shape == (6, N_V)

    def test_world_frame_linear_part(self):
        # LOCAL_WORLD_ALIGNED: 线速度部分是世界系下 ee 点速度
        J = jacobian.pinocchio_jacobian(common.Q_HOME)
        # 与 pinocchio 原生接口直接对比 (frame Jacobian, LOCAL_WORLD_ALIGNED)
        model = common.get_pinocchio_model()
        data = common.get_pin_data()
        fid = model.getFrameId(common.EE_FRAME)
        pin.computeJointJacobians(model, data, common.Q_HOME)
        pin.updateFramePlacements(model, data)
        J_ref = pin.getFrameJacobian(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        assert np.allclose(J, J_ref, atol=1e-12)


class TestNumericalJacobian:
    @pytest.mark.parametrize("seed", [0, 1, 2])
    def test_matches_analytic(self, seed):
        q = _random_q(seed)
        J_a = jacobian.pinocchio_jacobian(q)
        J_n = jacobian.numerical_jacobian(q)
        assert np.max(np.abs(J_a - J_n)) < 1e-4, f"seed={seed}"

    def test_non_singular_somewhere(self):
        # 工作空间中应存在良好姿态 (manipulability > 0)
        rng = np.random.default_rng(0)
        limits = common.get_joint_limits()
        best = max(
            jacobian.manipulability(rng.uniform(limits[:, 0], limits[:, 1]))
            for _ in range(20)
        )
        assert best > 1e-3, f"20 个随机位形最大 manipulability={best:.2e}"


class TestJacobianConsistency:
    """J dq 应预测 FK 的微小位移 (决定性一致性测试)。"""

    @pytest.mark.parametrize("seed", [3, 4, 5])
    def test_first_order_displacement(self, seed):
        q = _random_q(seed)
        rng = np.random.default_rng(seed)
        dq = rng.uniform(-1e-4, 1e-4, size=N_V)
        J = jacobian.pinocchio_jacobian(q)

        se3_0 = fk.pinocchio_fk(q)
        se3_1 = fk.pinocchio_fk(q + dq)
        J_lin = J[:3] @ dq

        pos_err = np.linalg.norm(se3_1.translation - se3_0.translation - J_lin)
        assert pos_err < 1e-5, f"seed={seed} pos_err={pos_err:.2e}"

        # 旋转一阶: log3(R0^T R1) ≈ J_ang dq (世界系)
        J_ang = J[3:] @ dq
        rot_pred = pin.log3(se3_0.rotation.T @ se3_1.rotation)
        # LOCAL_WORLD_ALIGNED 角速度 -> body 系需左乘 R^T
        body_pred = se3_0.rotation.T @ J_ang
        assert np.linalg.norm(rot_pred - body_pred * (1 + 0)) < 2e-4 or np.linalg.norm(
            rot_pred - body_pred
        ) < 2e-4, f"seed={seed}"


class TestSingularityMetrics:
    def test_singular_values_sorted(self):
        q = _random_q(7)
        sv = jacobian.singular_values(q)
        assert sv.shape == (N_V,)
        assert (np.diff(sv) <= 1e-12).all()
        assert (sv >= 0).all()

    def test_sigma_min_nonneg(self):
        q = _random_q(8)
        assert jacobian.singularity_index(q) >= 0.0

    def test_manipulability_equals_sigma_prod(self):
        q = _random_q(9)
        sv = jacobian.singular_values(q)
        w = jacobian.manipulability(q)
        assert np.isclose(w, np.prod(sv), rtol=1e-6)
