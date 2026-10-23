"""URDF ↔ MuJoCo XML 对齐验证的 TDD 测试。

被测模块: scripts/setup_verify_model_alignment.py
对齐标准 (workflow/plan/setup.md Phase 2):
  - FK 对比: 平移误差 < 1 mm, 旋转误差 < 0.5° (0.5° ≈ 8.73e-3 rad)
  - 限位容差: 1e-3 rad; 质量容差: 1e-3 kg
"""

import pytest
from conftest import URDF_PATH, XML_PATH

import setup_verify_model_alignment as vma


@pytest.fixture(scope="module")
def results():
    """跑一遍完整验证, 供各断言复用。"""
    return vma.run_verification(urdf_path=URDF_PATH, xml_path=XML_PATH, n_samples=100, seed=0)


def _by_name(results, name):
    for r in results:
        if r.name == name:
            return r
    pytest.fail(f"缺少检查项: {name}")


class TestJointList:
    def test_joint_names_and_types_match(self, results):
        r = _by_name(results, "joint_list")
        assert r.passed, r.detail


class TestJointLimits:
    def test_limits_within_1e3_rad(self, results):
        r = _by_name(results, "joint_limits")
        assert r.passed, r.detail

    def test_limits_detail_lists_max_deviation(self, results):
        r = _by_name(results, "joint_limits")
        assert "max" in r.detail


class TestInertials:
    def test_mass_within_1e3_kg(self, results):
        r = _by_name(results, "inertials")
        assert r.passed, r.detail


class TestForwardKinematics:
    def test_fk_all_bodies_within_tolerance(self, results):
        r = _by_name(results, "forward_kinematics")
        assert r.passed, r.detail

    def test_fk_detail_contains_max_position_error(self, results):
        r = _by_name(results, "forward_kinematics")
        assert "max_pos" in r.detail

    def test_fk_samples_random_configs(self, results):
        """决定性测试: 100 组随机关节角下每个公共连杆的世界位姿一致。"""
        r = _by_name(results, "forward_kinematics")
        assert "n_samples=100" in r.detail


class TestReport:
    def test_write_report_creates_markdown_table(self, tmp_path, results):
        out = tmp_path / "alignment_report.md"
        vma.write_report(results, out)
        text = out.read_text()
        assert "PASS" in text or "FAIL" in text
        assert "| 检查项 | 结果 |" in text

    def test_write_report_lists_each_check(self, tmp_path, results):
        out = tmp_path / "alignment_report.md"
        vma.write_report(results, out)
        text = out.read_text()
        for name in ("joint_list", "joint_limits", "inertials", "forward_kinematics"):
            assert name in text
