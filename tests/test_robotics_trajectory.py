"""Trajectory 模块 TDD 测试 (workflow/plan/robotics.md Phase 3, 对标 lab-2 B1)。"""

import numpy as np
import pytest

from so101_robotics import trajectory as traj

Q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
Q1 = np.array([0.5, -0.4, 0.3, -0.2, 0.1, 0.4])
T = 2.0


class TestQuintic:
    def test_endpoints_and_boundary_conditions(self):
        q, dq, ddq = traj.quintic(Q0, Q1, T, 0.0)
        assert np.allclose(q, Q0, atol=1e-12)
        assert np.allclose(dq, 0.0, atol=1e-12)
        q, dq, ddq = traj.quintic(Q0, Q1, T, T)
        assert np.allclose(q, Q1, atol=1e-12)
        assert np.allclose(dq, 0.0, atol=1e-12)
        assert np.allclose(ddq, 0.0, atol=1e-12)

    def test_midpoint_value(self):
        q, dq, _ = traj.quintic(Q0, Q1, T, T / 2)
        assert np.allclose(q, (Q0 + Q1) / 2, atol=1e-9)

    def test_peak_velocity(self):
        # quintic 峰值速度 1.875·Δ/T
        _, dq, _ = traj.quintic(Q0, Q1, T, T / 2)
        peak = 1.875 * (Q1 - Q0) / T
        assert np.allclose(dq, peak, atol=1e-9)


class TestMinJerk:
    def test_endpoints(self):
        q, dq, _ = traj.min_jerk(Q0, Q1, T, 0.0)
        assert np.allclose(q, Q0, atol=1e-12)
        q, dq, _ = traj.min_jerk(Q0, Q1, T, T)
        assert np.allclose(q, Q1, atol=1e-12)


class TestTrapezoidal:
    def test_displacement_over_full_time(self):
        """以 v_max/a_max 生成梯形速度轨迹, 全程位移应等于 Δq。"""
        v_max = 0.5
        a_max = 1.0
        dq_total = Q1 - Q0
        T_total = traj.trapezoidal_duration(dq_total, v_max, a_max)
        assert T_total > 0
        dt = 1e-3
        q = Q0.copy()
        t = 0.0
        while t < T_total:
            q += traj.trapezoidal_vel(dq_total, v_max, a_max, t) * dt
            t += dt
        assert np.allclose(q, Q1, atol=2e-3)

    def test_respects_velocity_limit(self):
        dq_total = Q1 - Q0
        v_max = 0.5
        a_max = 1.0
        for t in np.arange(0, 3.0, 0.01):
            v = traj.trapezoidal_vel(dq_total, v_max, a_max, t)
            assert (np.abs(v) <= v_max + 1e-9).all()


class TestJointTrajectory:
    def test_multi_segment_continuity(self):
        """两段 quintic 拼接: 边界处位置连续、速度为零。"""
        Q2 = np.array([0.2, 0.1, -0.1, 0.3, 0.0, 0.2])
        jt = traj.JointTrajectory([Q0, Q1, Q2], [1.5, 1.5])
        assert jt.duration == 3.0
        q_a, dq_a, _ = jt.sample(1.5)
        q_b, dq_b, _ = jt.sample(1.5 + 1e-9)
        assert np.allclose(q_a, Q1, atol=1e-9)
        assert np.allclose(q_b, Q1, atol=1e-9)
        assert np.allclose(dq_a, 0.0, atol=1e-9)

    def test_sample_out_of_range(self):
        jt = traj.JointTrajectory([Q0, Q1], [2.0])
        with pytest.raises(ValueError):
            jt.sample(-0.1)
        with pytest.raises(ValueError):
            jt.sample(2.1)


class TestCartesianWaypoints:
    def test_cube_edges_to_joint_traj(self):
        """笛卡尔航点经 IK 温启动转 JointTrajectory (draw cube 流水线用)。"""
        from so101_robotics import common, fk, ik

        # 收臂参考位形 (home 是伸直边界位形, 其邻域不可达); 偏移经实测可达
        q_ref = np.array([0.0, 0.8, -1.5, 0.7, 0.0, 0.3])
        base = fk.pinocchio_fk(q_ref).translation
        pts = [
            base + np.array([0.0, 0.04, 0.0]),
            base + np.array([-0.04, 0.0, 0.0]),
            base + np.array([-0.02, 0.0, 0.03]),
        ]
        q0 = q_ref
        qs = []
        for p in pts:
            r = ik.ik_dls(p, q_init=q0)
            assert r.converged, f"航点不可达: {p}"
            qs.append(r.q)
            q0 = r.q  # 温启动
        jt = traj.JointTrajectory([qs[0], qs[1], qs[2]], [1.0, 1.0])
        q, dq, _ = jt.sample(1.0)
        assert np.allclose(q, qs[1], atol=1e-6)