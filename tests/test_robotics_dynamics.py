"""Dynamics 模块 TDD 测试 (plan/robotics.md Phase 2, 对标 lab-2 A5)。"""

import numpy as np
import pinocchio as pin
import pytest

from so101_robotics import common, dynamics

import mujoco

ARM_SLICE = slice(0, 5)


def _random_state(seed):
    rng = np.random.default_rng(seed)
    limits = common.get_joint_limits()
    q = rng.uniform(limits[:, 0], limits[:, 1])
    dq = rng.uniform(-1.0, 1.1, size=6)
    dq = dq - 0.05  # 偏置防边界
    return q, dq


def _set_mujoco_qpos(mj_model, mj_data, q):
    for k, name in enumerate(common.JOINT_NAMES):
        jid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        mj_data.qpos[mj_model.jnt_qposadr[jid]] = q[k]


class TestGravity:
    def test_gravity_matches_mujoco_bias_at_zero_velocity(self):
        q, _ = _random_state(0)
        g_pin = dynamics.gravity(q)
        mj_model = common.get_mujoco_model()
        mj_data = mujoco.MjData(mj_model)
        _set_mujoco_qpos(mj_model, mj_data, q)
        mujoco.mj_forward(mj_model, mj_data)
        g_mj = mj_data.qfrc_bias[:6]
        assert np.allclose(g_pin, g_mj, atol=1e-6), np.max(np.abs(g_pin - g_mj))

    def test_gravity_within_servo_limit(self):
        """重力矩全工作空间 < 1.5 Nm — 重力补偿在舵机能力内 (学习关键结论)。"""
        rng = np.random.default_rng(1)
        limits = common.get_joint_limits()
        for _ in range(50):
            q = rng.uniform(limits[:, 0], limits[:, 1])
            assert np.max(np.abs(dynamics.gravity(q))) < 1.5


class TestMassMatrix:
    def test_mass_matrix_matches_mujoco_fullM(self):
        q, _ = _random_state(2)
        M_pin = dynamics.mass_matrix(q)
        mj_model = common.get_mujoco_model()
        mj_data = mujoco.MjData(mj_model)
        _set_mujoco_qpos(mj_model, mj_data, q)
        mujoco.mj_forward(mj_model, mj_data)
        M_mj = np.zeros((6, 6))
        # mujoco 3.16 绑定: mj_fullM(m, d, dst)（3.8.x 换成 (m, dst, M) 签名, pyproject
        # 已 override 锁 3.16 线, 勿降）
        mujoco.mj_fullM(mj_model, mj_data, M_mj)
        # mujoco M 含 armature; mass_matrix(include_armature=True) 与之直接对齐
        d = M_pin - M_mj
        assert np.allclose(d, 0.0, atol=1e-7), np.max(np.abs(d))


class TestMassMatrixSpd:
    def test_symmetric_positive_definite(self):
        q, _ = _random_state(3)
        M = dynamics.mass_matrix(q, include_armature=False)
        assert np.allclose(M, M.T, atol=1e-12)
        assert (np.linalg.eigvalsh(M) > 0).all()


class TestCoriolis:
    def test_coriolis_vs_mujoco_bias_difference(self):
        q, dq = _random_state(4)
        cq_pin = dynamics.rnea(q, dq, np.zeros(6)) - dynamics.gravity(q)
        mj_model = common.get_mujoco_model()
        mj_data = mujoco.MjData(mj_model)
        _set_mujoco_qpos(mj_model, mj_data, q)
        mj_data.qvel[:] = dq
        mujoco.mj_forward(mj_model, mj_data)
        mj_data2 = mujoco.MjData(mj_model)
        _set_mujoco_qpos(mj_model, mj_data2, q)
        mujoco.mj_forward(mj_model, mj_data2)
        cq_mj = mj_data.qfrc_bias[:6] - mj_data2.qfrc_bias[:6]
        assert np.allclose(cq_pin, cq_mj, atol=1e-6), np.max(np.abs(cq_pin - cq_mj))


class TestRneaAba:
    def test_aba_inverse_consistency(self):
        q, dq = _random_state(5)
        tau = np.array([0.1, -0.2, 0.15, 0.05, 0.02, 0.01])
        ddq = dynamics.aba(q, dq, tau)
        tau_rec = dynamics.rnea(q, dq, ddq)
        assert np.allclose(tau, tau_rec, atol=1e-9)
