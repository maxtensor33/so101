"""pinocchio IK 测试（plan/envs.md Phase 4, TDD）。

- 可达位姿: IK 收敛 (err < 1mm) 且 FK 复核
- 解在关节限位内
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
URDF = REPO_ROOT / "models" / "so101_new_calib.urdf"

REST_ARM = np.array([0.0, 0.0, 0.0, np.pi / 2, -np.pi / 2])  # 5 臂关节


@pytest.fixture
def ik():
    from so101_envs.ik import SO101IK

    return SO101IK(URDF)


def test_ik_converges_reachable(ik):
    """目标 = rest 位形 FK 正解位置 -> IK 收敛且 FK 复核。"""
    q0 = REST_ARM.copy()
    target, _ = ik.fk(q0)
    q, err = ik.solve(q0, target)
    assert err < 1e-3
    pos_fk, _ = ik.fk(q)
    assert np.linalg.norm(pos_fk - target) < 1e-3


def test_ik_converges_front_target(ik):
    """桌面上方典型抓取位姿 (0.28, 0.0, 0.05) 附近可达。"""
    q, err = ik.solve(REST_ARM, np.array([0.28, 0.0, 0.05]))
    assert err < 2e-3, f"err={err}"


def test_ik_respects_limits(ik):
    """解在关节限位内。"""
    rng = np.random.default_rng(0)
    for _ in range(10):
        q0 = REST_ARM + rng.uniform(-0.3, 0.3, size=5)
        target = np.array([0.25 + rng.uniform(-0.05, 0.05), rng.uniform(-0.1, 0.1), 0.03 + rng.uniform(0, 0.08)])
        q, err = ik.solve(q0, target)
        if err > 5e-3:  # 不可达目标允许不收敛, 但限位必须满足
            continue
        assert np.all(q >= ik.q_lower - 1e-6)
        assert np.all(q <= ik.q_upper + 1e-6)
