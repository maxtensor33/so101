"""draw_cube_node RViz marker 构造的 TDD 测试 (plan/robotics.md Phase 5)。

cube 线框 marker: 72 个航点按每边连续 POINTS_PER_EDGE=6 个排列,
线段必须沿边连接 (段长 = side/(POINTS_PER_EDGE-1)), 不得跨边跳变。
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

NODE = (Path(__file__).resolve().parents[1] / "src" / "so101_robotics_torque_app"
        / "so101_robotics_torque_app" / "draw_cube_node.py")

SIDE = 0.06
PPE = 6


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("draw_cube_node", NODE)
    m = importlib.util.module_from_spec(spec)
    sys.modules["draw_cube_node"] = m
    spec.loader.exec_module(m)
    return m


@pytest.fixture()
def waypoints(mod):
    """12 边 x 6 点的航点 (单位边长 SIDE), 每边连续 PPE 个。"""
    verts = mod.cube_vertices(np.zeros(3), SIDE)
    pts = []
    for a, b in mod.CUBE_EDGES:
        for s in np.linspace(0.0, 1.0, PPE):
            pts.append(verts[a] * (1 - s) + verts[b] * s)
    return np.array(pts)


class TestCubeWireframeMarker:
    def test_marker_type_and_frame(self, mod, waypoints):
        marker = mod.cube_wireframe_marker(waypoints)
        assert marker.type == 5  # Marker.LINE_LIST
        assert marker.header.frame_id == "world"
        assert marker.ns == "cube_wireframe"

    def test_segment_count(self, mod, waypoints):
        """12 边 x (PPE-1) 段, LINE_LIST 每段 2 点。"""
        marker = mod.cube_wireframe_marker(waypoints)
        assert len(marker.points) == 12 * (PPE - 1) * 2

    def test_all_segments_lie_on_edges(self, mod, waypoints):
        """每段长度必须 = side/(PPE-1), 不允许跨边跳变。"""
        marker = mod.cube_wireframe_marker(waypoints)
        pts = np.array([[p.x, p.y, p.z] for p in marker.points])
        seg = np.linalg.norm(pts[1::2] - pts[0::2], axis=1)
        assert np.allclose(seg, SIDE / (PPE - 1))

    def test_segments_are_contiguous(self, mod, waypoints):
        """同一边内的段首尾相接 (pb[i] == pa[i+1])。"""
        marker = mod.cube_wireframe_marker(waypoints)
        pts = np.array([[p.x, p.y, p.z] for p in marker.points])
        for e in range(12):
            blk = pts[e * (PPE - 1) * 2:(e + 1) * (PPE - 1) * 2]
            pb = blk[1::2]
            pa_next = blk[2::2]
            assert np.allclose(pb[:-1], pa_next)
