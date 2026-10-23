"""Control / constraints / sim 模块 TDD 测试 (workflow/plan/robotics.md Phase 3, 对标 lab-2 B2/B3)。"""

import numpy as np
import pytest

from so101_robotics import common, constraints, control, sim
from so101_robotics import trajectory as traj

KP = np.full(6, 50.0)
KD = np.full(6, 4.0)


class TestConstraints:
    def test_torque_limit_from_model(self):
        assert constraints.TORQUE_LIMIT == pytest.approx(3.35)

    def test_saturate_torque_clips(self):
        tau = np.array([5.0, -5.0, 0.1, 0.0, -0.1, 3.0])
        out = constraints.saturate_torque(tau)
        assert np.max(np.abs(out)) <= constraints.TORQUE_LIMIT + 1e-12
        assert out[2] == 0.1  # 未饱和项不变

    def test_saturate_torque_shape(self):
        out = constraints.saturate_torque(np.zeros(6))
        assert out.shape == (6,)


class TestSimWrapper:
    def test_step_advances_time_and_state(self):
        s = sim.So101Sim()
        s.reset(common.Q_HOME)
        q0 = s.q.copy()
        t0 = s.time
        s.step(np.zeros(6), n_substeps=5)
        assert s.time > t0
        assert not np.allclose(s.q, q0)  # 重力使臂运动

    def test_ee_pos_shape(self):
        s = sim.So101Sim()
        s.reset(common.Q_HOME)
        assert s.ee_pos().shape == (3,)


class TestPdGravity:
    def test_step_response_converges(self):
        s = sim.So101Sim()
        s.reset(common.Q_HOME)
        q_des = np.array([0.0, 0.3, -0.6, 0.3, 0.0, 0.3])
        for _ in range(300):  # 3s @ 100Hz
            tau = control.pd_gravity(s.q, s.dq, q_des, np.zeros(6), KP, KD)
            s.step(constraints.saturate_torque(tau), n_substeps=5)
        assert np.max(np.abs(s.q - q_des)) < 0.05, np.max(np.abs(s.q - q_des))


class TestComputedTorque:
    def test_tracks_quintic_trajectory(self):
        s = sim.So101Sim()
        s.reset(common.Q_HOME)
        q1 = np.array([0.3, -0.4, 0.5, -0.3, 0.0, 0.3])
        T = 2.0
        errs = []
        for i in range(200):  # 2s @ 100Hz
            q_d, dq_d, ddq_d = traj.quintic(common.Q_HOME, q1, T, i * 0.01)
            tau = control.computed_torque(s.q, s.dq, q_d, dq_d, ddq_d, KP, KD)
            s.step(constraints.saturate_torque(tau), n_substeps=5)
            errs.append(float(np.linalg.norm(s.q - q_d)))
        rms = float(np.sqrt(np.mean(np.square(errs))))
        assert rms < 0.15, f"RMS={rms:.4f} rad"


class TestTaskSpaceImpedance:
    def test_pulls_ee_to_target(self):
        from so101_robotics import fk

        q_ref = np.array([0.0, 0.8, -1.5, 0.7, 0.0, 0.3])
        s = sim.So101Sim()
        s.reset(q_ref)
        x_des = fk.pinocchio_fk(q_ref).translation + np.array([-0.02, 0.0, 0.02])
        for _ in range(300):  # 3s
            tau = control.task_space_impedance(s.q, s.dq, x_des)
            s.step(constraints.saturate_torque(tau), n_substeps=5)
        err = float(np.linalg.norm(s.ee_pos() - x_des))
        assert err < 0.02, f"ee 误差 {err * 1000:.1f} mm"
