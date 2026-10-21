"""so101_mtc_pick_place 包结构测试 (plan/mtc.md Phase 2/3)。"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1] / "src" / "so101_mtc_pick_place"


class TestPackageLayout:
    def test_package_xml(self):
        root = ET.parse(PKG / "package.xml").getroot()
        assert root.findtext("name") == "so101_mtc_pick_place"
        assert root.findtext("export/build_type") == "ament_cmake"
        deps = {d.text for d in root.iter() if d.tag in ("depend", "exec_depend", "build_depend")}
        assert "moveit_task_constructor_core" in deps
        assert "moveit_ros_move_group" in deps

    def test_cmake_links_mtc(self):
        text = (PKG / "CMakeLists.txt").read_text()
        assert "moveit_task_constructor_core" in text
        assert "mtc_pick_place_node" in text
        assert "grasp_attach_plugin" in text

    def test_plugin_registration(self):
        root = ET.parse(PKG / "grasp_attach_plugin.xml").getroot()
        classes = {c.get("name"): c.get("type") for c in root.iter("class")}
        assert "so101_mtc_pick_place/GraspAttachPlugin" in classes
        assert "GraspAttachPlugin" in classes["so101_mtc_pick_place/GraspAttachPlugin"]


class TestSources:
    def test_mtc_node_exists(self):
        text = (PKG / "src" / "mtc_pick_place_node.cpp").read_text()
        # 扁平 stage 链 (节点头注释记录了弃用 Connect/SerialContainer 的原因):
        # CurrentState -> MoveTo(open) -> MoveTo(pick, OMPL 关节目标) ->
        # ModifyPlanningScene -> MoveTo(close) -> ... -> MoveRelative(retreat)
        # grasp/place 位姿经 Phase 0 探测预求解为关节目标, 不用 ComputeIK
        for stage in ["CurrentState", "MoveTo", "MoveRelative",
                      "ModifyPlanningScene"]:
            assert stage in text
        # 预求解关节解架构: 不用 Connect/GeneratePose/ComputeIK
        for removed in ["stages::Connect", "stages::GeneratePose",
                        "stages::ComputeIK"]:
            assert removed not in text
        # 执行经 move_group 的 execute_task_solution
        assert "execute_task_solution" in text or "ExecuteTaskSolution" in text

    def test_cube_size_default_2cm(self):
        """R2 (plan/mtc_fix.md): planning scene cube 缩小到 2cm
        (三处同步之三: mtc 节点 cube_size 默认值)。"""
        text = (PKG / "src" / "mtc_pick_place_node.cpp").read_text()
        assert 'declare_parameter<double>("cube_size", 0.02)' in text

    def test_attach_plugin_semantics(self):
        text = (PKG / "src" / "grasp_attach_plugin.cpp").read_text()
        # kinematic attach: pre_step 写 cube qpos 跟随 jaw; gripper 命令阈值触发
        assert "pre_step" in text
        assert "gripper" in text
        assert "qpos" in text

    def test_attach_plugin_ideal_rel_pose(self):
        """R3 (plan/mtc_fix.md): 插件支持 ideal_rel_pose 刚体跟随,
        阈值方向与纠正后的 SRDF (closed=-0.02/open=1.0) 一致。"""
        text = (PKG / "src" / "grasp_attach_plugin.cpp").read_text()
        assert "ideal_rel_pose" in text
        assert "mju_mulQuat" in text  # 刚体跟随: wq = q_jaw ⊗ rel_q
        # 方向约定: attach 当 ref 降到 close_threshold 以下, release 当升过
        assert "last_grip_ < close_threshold_" in text
        assert "last_grip_ > release_threshold_" in text

    def test_srdf_gripper_direction(self):
        """R3: SRDF open/closed 与物理方向一致 (qpos 大 = 张开, R1 实测)。"""
        srdf = (PKG.parent / "so101_moveit_config" / "config" / "so101.srdf")
        root = ET.parse(srdf).getroot()
        states = {g.get("name"): g.find("joint").get("value")
                  for g in root.iter("group_state") if g.get("group") == "gripper"}
        assert float(states["closed"]) < float(states["open"])
        assert float(states["open"]) == pytest.approx(1.0)
        assert float(states["closed"]) == pytest.approx(-0.02)


class TestLaunchAndRviz:
    def test_launch_supports_both_sims(self):
        text = (PKG / "launch" / "mtc_pick_place.launch.py").read_text()
        assert "mujoco" in text and "gazebo" in text
        assert "sim" in text  # sim 参数

    def test_rviz_config_has_mtc_panel(self):
        text = (PKG / "rviz" / "mtc_pick_place.rviz").read_text()
        assert "MotionPlanning" in text
        assert "TaskConstructor" in text or "Introspection" in text

    def test_controllers_yaml_has_mujoco_plugins(self):
        import yaml
        cfg = yaml.safe_load((PKG / "config" / "controllers_pick.yaml").read_text())
        cm = cfg["controller_manager"]["ros__parameters"]
        assert "mujoco_plugins" in cm
        plugins = cm["mujoco_plugins"]
        # 列表元素是 plugin 实例名; 其 type 参数 (带点号 key) 引用 GraspAttachPlugin
        assert any("GraspAttachPlugin" in str(v)
                   for k, v in cm.items() if k.startswith("mujoco_plugins."))
        # 控制器与位置版一致 (JTC ×2 + JSB)
        assert cm["arm_controller"]["type"] == "joint_trajectory_controller/JointTrajectoryController"
