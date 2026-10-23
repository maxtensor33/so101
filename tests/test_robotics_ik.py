"""IK 模块 TDD 测试 (workflow/plan/robotics.md Phase 2, 对标 lab-2 A4)。"""

import numpy as np
import pinocchio as pin
import pytest

from so101_robotics import common, fk, ik

TOL_POS = 1e-3  # 1mm


def _random_target(seed, n=1):
    """以随机配置的 FK 生成可达目标 (自洽目标)。"""
    rng = np.random.default_rng(seed)
    limits = common.get_joint_limits()
    targets = []
    for _ in range(n):
        q = rng.uniform(limits[:, 0], limits[:, 1])
        targets.append(fk.pinocchio_fk(q))
    return targets


class TestDlsIk:
    def test_converges_to_reachable_target(self):
        target = _random_target(0)[0]
        result = ik.ik_dls(target, q_init=common.Q_HOME)
        assert result.converged, f"pos_err={result.pos_err:.4f}"
        assert result.pos_err < TOL_POS

    def test_solution_reproduces_target_fk(self):
        """决定性测试: IK 解经 FK 还原应得到目标位姿。"""
        target = _random_target(1)[0]
        result = ik.ik_dls(target, q_init=common.Q_HOME, max_iters=200)
        achieved = fk.pinocchio_fk(result.q)
        assert np.linalg.norm(achieved.translation - target.translation) < TOL_POS
        assert np.linalg.norm(pin.log3(achieved.rotation @ target.rotation.T)) < 1e-2

    @pytest.mark.parametrize("seed", [2, 3, 4])
    def test_batch_waypoint_accuracy(self, seed):
        """20 个随机可达航点 (仅位置) 全部收敛 — draw cube 的使用模式。"""
        targets = _random_target(seed, n=20)
        positions = [t.translation for t in targets]
        results = [ik.ik_dls(p, q_init=common.Q_HOME, max_iters=300) for p in positions]
        acc = sum(r.converged for r in results) / len(results)
        assert acc >= 0.9, f"waypoint 成功率 {acc:.0%}"

    def test_pinv_variant_converges(self):
        target = _random_target(5)[0]
        result = ik.ik_pinv(target.translation, q_init=common.Q_HOME)
        assert result.converged

    def test_unreachable_target_reports_failure(self):
        # 远超工作空间的目标
        target = pin.SE3(np.eye(3), np.array([2.0, 0.0, 2.0]))
        result = ik.ik_dls(target, q_init=common.Q_HOME, max_iters=200)
        assert not result.converged

    def test_joint_limits_respected(self):
        targets = _random_target(6, n=10)
        limits = common.get_joint_limits()
        for t in targets:
            r = ik.ik_dls(t, q_init=common.Q_HOME, max_iters=200)
            assert (r.q >= limits[:, 0] - 1e-9).all()
            assert (r.q <= limits[:, 1] + 1e-9).all()

    def test_damping_activates_near_singularity(self):
        """σ_min 小时阻尼应大于基准 λ0。"""
        q_near = np.zeros(6)  # home 为奇异位形 (伸直)
        assert ik.adaptive_damping(q_near) > ik.LAMBDA0
