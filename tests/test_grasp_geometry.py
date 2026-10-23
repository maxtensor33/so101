"""夹持几何校准的 TDD 测试 (plan/mtc_fix.md Phase R1)。

理想夹持位姿: cube 中心位于固定指面与动爪指面的中点 (留 gap 间隙),
与两指面等距、无穿透, 且相对几何随 yaw 旋转不变。
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mtc_probe_grasp_geometry.py"


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("probe_grasp_geometry", SCRIPT)
    m = importlib.util.module_from_spec(spec)
    sys.modules["probe_grasp_geometry"] = m
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def calibrated(mod):
    """calibrate() 结果 (2cm cube), 供实测几何断言共用。"""
    return mod.calibrate(cube_size=0.02, yaw_deg=-33.0)


class TestFingerMidpoint:
    def test_midpoint_is_equidistant(self, mod):
        """中点到两指面距离相等。"""
        fixed = np.array([0.0, 0.02, 0.0])
        moving = np.array([0.0, -0.05, 0.0])
        mid = mod.finger_midpoint(fixed, moving)
        assert np.linalg.norm(mid - fixed) == pytest.approx(
            np.linalg.norm(mid - moving))

    def test_midpoint_between(self, mod):
        mid = mod.finger_midpoint(np.zeros(3), np.abs(np.array([0.0, 0.1, 0.0])))
        assert mid == pytest.approx(np.array([0.0, 0.05, 0.0]))


class TestIdealCubePose:
    def test_cube_centered_between_fingers(self, mod):
        """理想位姿: cube 中心在两指面中点, 沿夹持轴无偏。"""
        fixed_y, moving_y = 0.02, -0.05
        axis = np.array([0.0, 1.0, 0.0])
        base = np.zeros(3)
        pose = mod.ideal_cube_pose(base, axis, fixed_y, moving_y,
                                   cube_half=0.015, gap=0.001)
        d_fixed = abs(abs(float(np.dot(pose - base, axis)) - fixed_y) - 0.015)
        d_moving = abs(abs(float(np.dot(pose - base, axis)) - moving_y) - 0.015)
        assert d_fixed == pytest.approx(d_moving, abs=1e-9)

    def test_no_penetration(self, mod):
        """cube 与指面之间保留 gap 间隙 (不穿透)。
        指面间距 = cube + 2*gap 的自洽布置: fixed=+0.016, moving=-0.016,
        cube_half=0.015, gap=0.001。"""
        axis = np.array([0.0, 1.0, 0.0])
        pose = mod.ideal_cube_pose(np.zeros(3), axis, 0.016, -0.016,
                                   cube_half=0.015, gap=0.001)
        lo = pose[1] - 0.015
        hi = pose[1] + 0.015
        assert 0.016 - hi == pytest.approx(0.001, abs=1e-9)
        assert lo - (-0.016) == pytest.approx(0.001, abs=1e-9)

    def test_yaw_invariance(self, mod):
        """绕法线旋转 (yaw), 沿夹持轴的相对坐标不变。"""
        axis = np.array([0.0, 1.0, 0.0])
        pose0 = mod.ideal_cube_pose(np.zeros(3), axis, 0.02, -0.05, 0.015, 0.001)
        c, s = np.cos(np.pi / 2), np.sin(np.pi / 2)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        axis2 = Rz @ axis
        pose1 = mod.ideal_cube_pose(np.zeros(3), axis2, 0.02, -0.05, 0.015, 0.001)
        assert float(np.dot(pose1, axis2)) == pytest.approx(
            float(np.dot(pose0, axis)))


class TestMeasuredGeometry:
    """实测几何不变量 (校准输出必须物理自洽)。

    回归背景: 首版 _finger_face_positions 把 servo mesh 误当固定指,
    轴向内面选择反号, 输出 span=-129.8mm (负值)、close_angle 退化为扫描上限。
    """

    @pytest.fixture
    def out(self, calibrated):
        return calibrated

    def test_span_is_positive_and_plausible(self, out):
        """两指面间距必须为正, 且在 SO-101 爪开口物理范围 (<=80mm) 内。"""
        span = abs(out["fixed_surface_y"] - out["moving_surface_y"])
        assert 0.0 < span <= 0.08

    def test_fixed_face_beyond_moving_face(self, out):
        """按函数约定 fixed > moving (fixed 指面在轴正侧)。"""
        assert out["fixed_surface_y"] > out["moving_surface_y"]

    def test_close_angle_genuine_not_fallback(self, mod, out):
        """close_angle 必须是真实解: 该闭合角下 span >= cube+2*gap,
        而不是扫描耗尽后的上限兜底值。"""
        target = 0.02 + 2 * 0.001
        assert out["span_closed"] >= target
        assert out["close_angle"] <= float(mod.CLOSE_SCAN[-1])

    def test_reported_span_matches_surface_difference(self, out):
        """span_closed 与两指面坐标差一致 (无符号翻转)。"""
        diff = out["fixed_surface_y"] - out["moving_surface_y"]
        assert diff == pytest.approx(out["span_closed"], abs=1e-9)


class TestCalibrate:
    def test_calibrate_returns_all_outputs(self, mod):
        """calibrate() 返回 standoff/close_angle/ideal_rel_pose 结构完整的 dict。"""
        out = mod.calibrate(cube_size=0.02, yaw_deg=-33.0)
        for key in ("standoff", "close_angle", "ideal_rel_pose",
                    "fixed_surface_y", "moving_surface_y"):
            assert key in out
        assert out["standoff"] > 0
        # close_angle 是 gripper 关节范围内的真实闭合角 (可为负: 关节下限 -0.175)
        assert -0.175 <= out["close_angle"] <= 1.75
        assert len(out["ideal_rel_pose"]) == 7  # [x,y,z,qw,qx,qy,qz]

    def test_cube_fits_between_fingers(self, mod):
        """cube 尺寸必须小于两指面间距 - 2*gap (否则永远夹不住)。"""
        out = mod.calibrate(cube_size=0.02, yaw_deg=-33.0)
        span = abs(out["fixed_surface_y"] - out["moving_surface_y"])
        assert 0.02 < span - 2 * 0.001
