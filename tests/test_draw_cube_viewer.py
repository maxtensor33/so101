"""robotics_draw_cube_viewer 脚本的 TDD 测试 (plan/robotics.md Phase 4/5)。

对标 lab-2 c3_draw_cube.py: 立方体几何 → 边列表 → 连续描线路径。
脚本本体: scripts/robotics_draw_cube_viewer.py (纯函数部分可测, viewer 部分为集成)。
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "robotics_draw_cube_viewer.py"


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("robotics_draw_cube_viewer", SCRIPT)
    m = importlib.util.module_from_spec(spec)
    sys.modules["robotics_draw_cube_viewer"] = m
    spec.loader.exec_module(m)
    return m


class TestCubeGeometry:
    def test_cube_has_8_vertices(self, mod):
        verts = mod.cube_vertices(np.array([0.1, 0.0, 0.05]), 0.06)
        assert len(verts) == 8

    def test_cube_side_length(self, mod):
        verts = mod.cube_vertices(np.zeros(3), 0.06)
        # 任取两顶点, 最大距离 = 体对角线 side*sqrt(3)
        d = np.linalg.norm(verts[0] - verts[6])
        assert d == pytest.approx(0.06 * np.sqrt(3))

    def test_cube_centered(self, mod):
        center = np.array([0.12, -0.03, 0.04])
        verts = mod.cube_vertices(center, 0.06)
        assert np.allclose(np.mean(verts, axis=0), center)


class TestCubeEdges:
    def test_12_edges(self, mod):
        assert len(mod.CUBE_EDGES) == 12

    def test_each_vertex_degree_3(self, mod):
        deg = np.zeros(8, dtype=int)
        for a, b in mod.CUBE_EDGES:
            deg[a] += 1
            deg[b] += 1
        assert (deg == 3).all()

    def test_all_edges_equal(self, mod):
        verts = mod.cube_vertices(np.zeros(3), 0.06)
        lengths = [np.linalg.norm(verts[a] - verts[b]) for a, b in mod.CUBE_EDGES]
        assert np.allclose(lengths, 0.06)


class TestEdgeWaypoints:
    def test_points_per_edge(self, mod):
        verts = mod.cube_vertices(np.zeros(3), 0.06)
        pts = mod.edge_waypoints(verts, points_per_edge=6)
        # 12 边 x 6 点 (端点重复)
        assert len(pts) == 12 * 6

    def test_first_and_last_waypoints_are_vertices(self, mod):
        verts = mod.cube_vertices(np.zeros(3), 0.06)
        pts = mod.edge_waypoints(verts, points_per_edge=6)
        a0, b0 = mod.CUBE_EDGES[0]
        assert np.allclose(pts[0], verts[a0])
        a, b = mod.CUBE_EDGES[-1]
        assert np.allclose(pts[-1], verts[b])

    def test_waypoints_lie_on_edges(self, mod):
        verts = mod.cube_vertices(np.zeros(3), 0.06)
        pts = mod.edge_waypoints(verts, points_per_edge=5)
        for k, (a, b) in enumerate(mod.CUBE_EDGES):
            for s in range(5):
                p = pts[k * 5 + s]
                # 点到边线段的距离为 0
                ab = verts[b] - verts[a]
                t = np.clip(np.dot(p - verts[a], ab) / np.dot(ab, ab), 0, 1)
                d = np.linalg.norm(p - (verts[a] + t * ab))
                assert d < 1e-9


class TestDefaultConfig:
    """默认配置与 ROS draw_cube_node 保持一致。"""

    def test_sides_to_try(self, mod):
        assert mod.SIDES_TO_TRY == [0.08, 0.07, 0.06]

    def test_points_per_edge(self, mod):
        assert mod.POINTS_PER_EDGE == 6

    def test_torque_limit(self, mod):
        assert mod.TAU_LIMIT == pytest.approx(3.35)


class TestSceneModel:
    """viewer 脚本必须加载 scene（含地板）而非 robot-only MJCF。"""

    def test_scene_xml_points_to_sim_scene(self, mod):
        assert mod.SCENE_XML.name == "scene_torque.xml"
        assert mod.SCENE_XML.is_file()

    def test_scene_loads_with_floor(self, mod):
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(mod.SCENE_XML))
        floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        assert floor_id >= 0
        assert model.nu == 6

    def test_joint_adrs_index_6_joints(self, mod):
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(mod.SCENE_XML))
        qadr, vadr = mod.joint_adrs(model)
        assert qadr == list(range(6))
        assert vadr == list(range(6))
