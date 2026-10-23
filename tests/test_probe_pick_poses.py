"""mtc_probe_pick_poses 脚本的 TDD 测试 (workflow/plan/mtc.md Phase 0)。

探测 so101 在桌面上抓取 cube 的 pick/place 摆位可达性。
纯函数部分可测; 全量扫描作为脚本运行 (耗时长, 不进 pytest)。
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mtc_probe_pick_poses.py"


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("mtc_probe_pick_poses", SCRIPT)
    m = importlib.util.module_from_spec(spec)
    sys.modules["mtc_probe_pick_poses"] = m
    spec.loader.exec_module(m)
    return m


class TestGraspPose:
    def test_ee_height_from_ideal_rel_pose(self, mod):
        """EE 高度由 T_rel 实测值决定 (R1: cube 中心上方 ~4.8cm), 不再依赖
        grasp_height 参数。"""
        cube = np.array([0.13, 0.0, 0.01])
        pose = mod.grasp_pose(cube)
        assert pose.translation[2] == pytest.approx(
            cube[2] + mod.T_REL.inverse().translation[2])

    def test_rotation_valid(self, mod):
        """旋转为正交矩阵 (det=+1)。"""
        pose = mod.grasp_pose(np.array([0.13, 0.0, 0.01]))
        R = pose.rotation
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert np.linalg.det(R) == pytest.approx(1.0)

    def test_tool_axis_vertical(self, mod):
        """top grasp 理想姿态族 (R2, 实测自 R1 economy 抓取位形): EE frame y
        轴竖直 (旧实现目标 "frame x 竖直" 因 IK 旋转容差松从未真正达成,
        economy 关节解的真实 jaw 朝向即 frame y 竖直)。yaw 绕垂直轴旋转。"""
        pose = mod.grasp_pose(np.array([0.13, 0.0, 0.01]))
        tool_y_world = pose.rotation @ np.array([0.0, 1.0, 0.0])
        assert abs(tool_y_world[2]) == pytest.approx(1.0, abs=1e-2)


class TestCandidates:
    def test_candidate_picks_shape_and_range(self, mod):
        """pick 候选在 [r_min, r_max] x [yaw_min, yaw_max] 网格内。"""
        picks = mod.candidate_picks(r_range=(0.10, 0.16), yaw_range=(-40, 40),
                                    n_r=3, n_yaw=3)
        assert len(picks) == 9
        for c in picks:
            r = np.hypot(c[0], c[1])
            assert 0.099 <= r <= 0.161
            assert c[2] >= 0.0  # 在地面上方

    def test_candidate_places_disjoint_from_pick(self, mod):
        """place 候选与 pick 区域错开 (yaw 偏移), 且成对返回。"""
        pick = mod.candidate_picks(r_range=(0.10, 0.16), yaw_range=(-40, 40),
                                   n_r=2, n_yaw=2)
        places = mod.candidate_places(pick, dyaw=60)
        assert len(places) == len(pick)
        for p, q in zip(pick, places):
            assert q[2] >= 0.0
            # yaw 错开: 方位角不同
            ay = np.degrees(np.arctan2(p[1], p[0]))
            by = np.degrees(np.arctan2(q[1], q[0]))
            d = abs((by - ay + 180) % 360 - 180)
            assert d > 30


class TestGraspSequence:
    def test_pregrasp_above_grasp(self, mod):
        """预抓取 = 抓取位沿世界 z 上移 clearance。"""
        cube = np.array([0.13, 0.0, 0.02])
        g = mod.grasp_pose(cube, grasp_height=0.05)
        pg = mod.pregrasp_pose(g, clearance=0.05)
        assert pg.translation[2] == pytest.approx(g.translation[2] + 0.05)
        assert np.allclose(pg.translation[:2], g.translation[:2])

    def test_sequence_smoke(self, mod):
        """小预算 IK 冒烟: 序列生成 (pre/grasp/lift) 不抛异常且结果结构完整。"""
        cube = np.array([0.13, 0.0, 0.02])
        seq = mod.grasp_sequence(cube, grasp_height=0.05, lift=0.03)
        assert len(seq) == 3  # (名称, 位姿) x3
        poses = [p for _, p in seq]
        for p in poses:
            assert p.translation.shape == (3,)
        # lift 高于 grasp
        assert poses[2].translation[2] > poses[1].translation[2]


class TestIdealGraspFromRelPose:
    """R2 (workflow/plan/mtc_fix.md): grasp_pose 由 R1 实测 ideal_rel_pose 构造。

    旧实现 EE 在 cube 正上方 (水平偏移 0), 而理想夹持几何中 EE (动爪) 原点
    相对 cube 中心有 ~28mm 水平 standoff —— 这是 "cube 贴一边夹爪提起" 的
    根因之一。新实现: T_ee = T_cube ∘ inv(T_rel), T_rel 来自 probe_grasp_
    geometry.ideal_rel_pose (R1 输出)。
    """

    def test_ee_target_composes_back_to_cube_center(self, mod):
        """EE 目标经 T_rel 正映射必须精确回到 cube 中心 (构造自洽)。"""
        cube = np.array([0.117, -0.077, 0.01])
        pose = mod.grasp_pose(cube, yaw_deg=-33.0)
        R_rel = pin.Quaternion(mod.IDEAL_REL_POSE[3], mod.IDEAL_REL_POSE[4],
                               mod.IDEAL_REL_POSE[5], mod.IDEAL_REL_POSE[6]).matrix()
        t_rel = np.array(mod.IDEAL_REL_POSE[:3])
        back = pin.SE3(pose.rotation, pose.translation) * pin.SE3(R_rel, t_rel)
        assert back.translation == pytest.approx(cube, abs=1e-9)

    def test_ee_has_horizontal_standoff_offset(self, mod):
        """EE 原点不再位于 cube 正上方: 水平偏移 = standoff (R1 实测 27.9mm)。"""
        cube = np.array([0.117, -0.077, 0.01])
        pose = mod.grasp_pose(cube, yaw_deg=-33.0)
        dxy = np.linalg.norm(pose.translation[:2] - cube[:2])
        assert dxy == pytest.approx(mod.STANDOFF, abs=2e-3)

    def test_standoff_independent_of_yaw(self, mod):
        """yaw 只旋转偏移方向, 不改变偏移大小。"""
        cube = np.array([0.117, -0.077, 0.01])
        d0 = np.linalg.norm(
            mod.grasp_pose(cube, yaw_deg=0.0).translation[:2] - cube[:2])
        d1 = np.linalg.norm(
            mod.grasp_pose(cube, yaw_deg=-33.0).translation[:2] - cube[:2])
        assert d0 == pytest.approx(d1, abs=1e-9)


class TestSelfCollisionFilter:
    """R2 (workflow/plan/mtc_fix.md): probe IK 解必须过 mujoco 自碰撞校验。

    economy 教训: 原 probe 只查关节限位, 曾给出 gripper_link 穿 shoulder_link
    3mm 的解, 导致 move_group 规划目标 state 无效。
    """

    def test_known_bad_solution_rejected(self, mod):
        """已知穿肩解 (economy 会话根因 A) 必须被判为自碰撞。"""
        q_bad = np.array([0.7612, -0.1760, 1.4566, 0.2903, -0.7610, 0.0])
        assert mod.has_self_collision(q_bad)

    def test_known_good_solution_accepted(self, mod):
        """economy 无自碰撞解 (现行节点默认 grasp_joints) 必须通过。"""
        q_good = np.array([0.59347, -0.82848, 1.31785, 1.0817, -1.50441, -0.02])
        assert not mod.has_self_collision(q_good)

    def test_probe_pair_filters_self_collision(self, mod, monkeypatch):
        """probe_pair 丢弃自碰撞解 (即使 IK 收敛)。"""
        monkeypatch.setattr(mod, "has_self_collision",
                            lambda q: True)
        ok, sols = mod.probe_pair(np.array([0.117, -0.077, 0.01]),
                                  np.array([0.125, 0.063, 0.01]),
                                  max_iters=5)
        assert not ok and sols is None
