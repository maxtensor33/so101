"""FK 模块 TDD 测试 (plan/robotics.md Phase 1, 对标 lab-2 A2)。

对齐标准与 setup_verify_model_alignment 一致: 平移 < 1mm, 旋转 < 0.5°。
"""

import numpy as np
import pinocchio as pin
import pytest

from so101_robotics import common, fk

ALL_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class TestCommon:
    def test_joint_names(self):
        assert common.JOINT_NAMES == ALL_JOINTS
        assert common.ARM_JOINT_NAMES == ALL_JOINTS[:5]

    def test_joint_limits_shape(self):
        limits = common.get_joint_limits()
        assert limits.shape == (6, 2)
        lo, hi = limits[:, 0], limits[:, 1]
        assert (lo < hi).all()
        # 0 (home) 在所有关节限位内
        assert ((lo <= 0.0) & (0.0 <= hi)).all()

    def test_models_cached(self):
        m1 = common.get_pinocchio_model()
        m2 = common.get_pinocchio_model()
        assert m1 is m2
        w1 = common.get_mujoco_model()
        w2 = common.get_mujoco_model()
        assert w1 is w2


class TestPinocchioFk:
    def test_returns_se3(self):
        se3 = fk.pinocchio_fk(common.Q_HOME)
        assert isinstance(se3, pin.SE3)
        assert se3.translation.shape == (3,)
        assert se3.rotation.shape == (3, 3)
        assert np.isclose(np.linalg.det(se3.rotation), 1.0)

    def test_base_frame_at_home(self):
        # base_link 在世界原点 (URDF 根为 world, fixed joint); 注意传 link 名而非 body 名
        se3 = fk.pinocchio_fk(common.Q_HOME, frame="base_link")
        assert np.allclose(se3.translation, 0.0, atol=1e-9)

    def test_ee_reachable_height(self):
        # SO-ARM101 臂展 ~25cm, 末端高度应在合理范围内
        t = fk.pinocchio_fk(common.Q_HOME).translation
        assert 0.0 < t[2] < 0.5


class TestMujocoFk:
    def test_returns_se3(self):
        se3 = fk.mujoco_fk(common.Q_HOME)
        assert isinstance(se3, pin.SE3)

    def test_matches_pinocchio_at_home(self):
        p, m = fk.pinocchio_fk(common.Q_HOME), fk.mujoco_fk(common.Q_HOME)
        assert np.linalg.norm(p.translation - m.translation) < 1e-3
        assert np.linalg.norm(pin.log3(p.rotation @ m.rotation.T)) < np.deg2rad(0.5)


class TestCrossValidation:
    def test_random_configs_within_tolerance(self):
        max_pos, max_rot = fk.cross_check(n_samples=100, seed=0)
        assert max_pos < 1e-3, f"max_pos={max_pos * 1000:.4f} mm"
        assert max_rot < np.deg2rad(0.5), f"max_rot={np.rad2deg(max_rot):.4f} deg"

    def test_all_link_bodies_agree(self):
        # 每个 link↔body 映射对都应在同一随机配置下一致
        rng = np.random.default_rng(42)
        limits = common.get_joint_limits()
        q = rng.uniform(limits[:, 0], limits[:, 1])
        for link, body in common.LINK_BODY_MAP.items():
            p = fk.pinocchio_fk(q, frame=link)
            m = fk.mujoco_fk(q, body=body)
            assert np.linalg.norm(p.translation - m.translation) < 1e-3, link
            assert np.linalg.norm(pin.log3(p.rotation @ m.rotation.T)) < np.deg2rad(0.5), link
