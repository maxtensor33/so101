"""pick 场景的 TDD 测试 (workflow/plan/mtc.md Phase 1)。

双后端抓取场景:
  - MuJoCo: scene_pick.xml = 位置控制版场景 + 可抓取 cube + weld equality (默认
    inactive, 由 mujoco plugin 依 gripper 命令阈值激活 —— "磁吸"抓取兜底)
  - Gazebo: pick_place.sdf world = floor + cube + DetachableJoint system plugin
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import pytest

REPO = Path(__file__).resolve().parents[1]
SCENE_PICK = REPO / "src" / "so101_sim" / "mujoco" / "scene_pick.xml"
GZ_WORLD = REPO / "src" / "so101_bringup" / "worlds" / "pick_place.sdf"

# 摆位: Phase 0 探测脚本 (mtc_probe_pick_poses.py) 输出的推荐组合
# R2 (workflow/plan/mtc_fix.md): cube 4cm -> 2cm, 底面贴地 z = 0.01
PICK_CUBE = [0.117, -0.077]
PLACE_CUBE = [0.125, 0.063]
CUBE_SIZE = 0.02  # 全边长 (mujoco box size 为半边长 0.01)


class TestMuJoCoPickScene:
    @pytest.fixture(scope="class")
    def model(self):
        return mujoco.MjModel.from_xml_path(str(SCENE_PICK))

    def test_scene_loads(self, model):
        assert model.nq == 13  # 6 关节 + cube freejoint(7)

    def test_cube_body_exists(self, model):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        assert bid >= 0

    def test_cube_at_probed_pick_position(self, model):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        assert model.body_pos[bid][:2] == pytest.approx(PICK_CUBE, abs=1e-3)
        # 底面贴地 (2cm cube: 中心 z = 0.01)
        assert model.body_pos[bid][2] == pytest.approx(CUBE_SIZE / 2, abs=1e-3)

    def test_cube_size_2cm(self, model):
        """R2: cube 缩小到 2cm (三处同步之一: mujoco)。
        y 向保留原 4cm 版的抗接触对称抖动 epsilon (半边长 0.0101)。"""
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        assert model.geom_size[gid][0] == pytest.approx(CUBE_SIZE / 2, abs=1e-6)
        assert model.geom_size[gid][2] == pytest.approx(CUBE_SIZE / 2, abs=1e-6)
        assert CUBE_SIZE / 2 <= model.geom_size[gid][1] <= CUBE_SIZE / 2 + 2e-4

    def test_no_equality_constraints(self, model):
        """抓取用 kinematic attach (plugin 写 qpos), 场景无 equality。"""
        assert model.neq == 0

    def test_robot_is_position_control_version(self, model):
        """pick 场景基于位置控制版 (MTC 经 JTC 执行), 而非 torque 版。"""
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                 for i in range(model.nu)]
        assert names == ["shoulder_pan", "shoulder_lift", "elbow_flex",
                         "wrist_flex", "wrist_roll", "gripper"]
        # 位置伺服: biastype=affine (非 NONE)
        assert model.actuator_biastype[0] != int(mujoco.mjtBias.mjBIAS_NONE)

    def test_place_marker_present(self):
        """place 区域标记 (visual site/geom) 存在于场景 XML。"""
        root = ET.parse(SCENE_PICK).getroot()
        texts = [e.get("name", "") for e in root.iter()]
        assert any("place" in t.lower() for t in texts)


class TestGazeboPickWorld:
    def test_world_file_exists_and_parses(self):
        assert GZ_WORLD.is_file()
        ET.parse(GZ_WORLD)

    def test_world_contains_cube(self):
        root = ET.parse(GZ_WORLD).getroot()
        names = [e.get("name") for e in root.iter("model")]
        assert "cube" in names

    def test_cube_pose_matches_pick(self):
        root = ET.parse(GZ_WORLD).getroot()
        for model_el in root.iter("model"):
            if model_el.get("name") != "cube":
                continue
            pose = model_el.find("pose")
            vals = [float(x) for x in pose.text.split()]
            assert vals[:2] == pytest.approx(PICK_CUBE, abs=1e-3)
            # 底面贴地 (2cm cube)
            assert vals[2] == pytest.approx(CUBE_SIZE / 2, abs=1e-3)

    def test_cube_size_2cm(self):
        """R2: cube 缩小到 2cm (三处同步之一: gazebo sdf)。"""
        root = ET.parse(GZ_WORLD).getroot()
        for model_el in root.iter("model"):
            if model_el.get("name") != "cube":
                continue
            sizes = [float(x)
                     for box in model_el.iter("box")
                     for x in box.find("size").text.split()]
            assert all(s == pytest.approx(CUBE_SIZE, abs=1e-6) for s in sizes)

    def test_detachable_joint_plugin(self):
        root = ET.parse(GZ_WORLD).getroot()
        plugins = [p.get("filename", "") for p in root.iter("plugin")]
        assert any("detachable" in p for p in plugins)
