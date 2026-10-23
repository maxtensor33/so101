"""力矩控制版本模型对齐与配置验证的 TDD 测试 (workflow/plan/robotics.md Phase 0)。

xxx_torque 命名规则:
  - models/so101_torque.urdf            (验证副本)
  - models/so101_new_calib_torque.xml   (验证副本)
  - src/so101_sim/mujoco/scene_torque.xml
  - src/so101_sim/urdf/so101.ros2_control.xacro (effort 接口宏)
  - src/so101_sim/config/controllers_torque.yaml

行为标准:
  - 力矩版 MJCF 的 actuator 全部为纯力矩 (motor): ctrl 单位 Nm, 无位置伺服弹簧
  - forcerange = ±3.35 Nm (STS3215 舵机力矩上限)
  - 运动学层与位置版 100% 一致 (对齐验证全 PASS)
"""

import xml.etree.ElementTree as ET

import mujoco
import pytest
import yaml
from conftest import (
    CONTROLLERS_TORQUE_YAML,
    ROS2_CONTROL_XACRO,
    SCENE_TORQUE_PATH,
    URDF_TORQUE_PATH,
    XML_TORQUE_PATH,
)

import setup_verify_model_alignment as vma

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_JOINTS = ARM_JOINTS + ["gripper"]
TORQUE_LIMIT = 3.35


@pytest.fixture(scope="module")
def mj_model():
    return mujoco.MjModel.from_xml_path(str(XML_TORQUE_PATH))


class TestTorqueModelFiles:
    def test_torque_xml_exists_and_loads(self, mj_model):
        assert mj_model.nu == 6

    def test_torque_urdf_exists(self):
        assert URDF_TORQUE_PATH.is_file()


class TestTorqueActuators:
    """actuator 行为级测试: ctrl 必须是纯力矩, 而非位置伺服。"""

    def test_all_actuators_are_pure_torque(self, mj_model):
        """motor actuator: gainprm=1 且 biastype=NONE (位置伺服是 affine bias)。"""
        none_bias = int(mujoco.mjtBias.mjBIAS_NONE)
        assert mj_model.actuator_gainprm[:, 0].tolist() == [1.0] * 6
        assert mj_model.actuator_biastype.tolist() == [none_bias] * 6

    def test_forcerange_is_servo_limit(self, mj_model):
        assert mj_model.actuator_forcelimited.all()
        for i in range(mj_model.nu):
            lo, hi = mj_model.actuator_forcerange[i]
            assert lo == pytest.approx(-TORQUE_LIMIT)
            assert hi == pytest.approx(TORQUE_LIMIT)

    def test_ctrlrange_matches_torque_units(self, mj_model):
        """motor 的 ctrl 是 Nm, ctrlrange 应与 forcerange 一致。"""
        assert mj_model.actuator_ctrllimited.all()
        for i in range(mj_model.nu):
            lo, hi = mj_model.actuator_ctrlrange[i]
            assert (lo, hi) == pytest.approx((-TORQUE_LIMIT, TORQUE_LIMIT))

    def test_actuator_names_match_joints(self, mj_model):
        names = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(mj_model.nu)]
        assert names == ALL_JOINTS

    def test_no_position_servo_left_in_defaults(self):
        """XML 文本层面: 不应残留 <position> 伺服默认项。"""
        root = ET.parse(XML_TORQUE_PATH).getroot()
        assert root.iter("position") is not None
        assert len(list(root.iter("position"))) == 0


class TestTorqueKinematicAlignment:
    """力矩版只改 actuator, 运动学必须与位置版完全一致。"""

    @pytest.fixture(scope="class")
    def results(self):
        return vma.run_verification(
            urdf_path=URDF_TORQUE_PATH, xml_path=XML_TORQUE_PATH, n_samples=100, seed=0
        )

    def test_all_checks_pass(self, results):
        failed = [r for r in results if not r.passed]
        assert not failed, [(r.name, r.detail) for r in failed]


class TestSceneTorque:
    def test_scene_loads_with_floor(self):
        model = mujoco.MjModel.from_xml_path(str(SCENE_TORQUE_PATH))
        floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        assert floor_id >= 0

    def test_scene_contains_robot(self):
        model = mujoco.MjModel.from_xml_path(str(SCENE_TORQUE_PATH))
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "shoulder")
        assert body_id >= 0


class TestEffortInterfacesXacro:
    @pytest.fixture(scope="class")
    def xacro_root(self):
        return ET.parse(ROS2_CONTROL_XACRO).getroot()

    @staticmethod
    def _find_macros(root, name):
        # ET 将 xacro:macro 解析为 {namespace}macro, 按局部名匹配
        return [
            m for m in root.iter()
            if m.tag.rsplit("}", 1)[-1] == "macro" and m.get("name") == name
        ]

    def test_has_effort_interfaces_macro(self, xacro_root):
        macros = self._find_macros(xacro_root, "so101_joint_interfaces_effort")
        assert len(macros) == 1
        joint_ifs = {j.get("name") for j in macros[0].iter("joint")}
        assert joint_ifs == set(ALL_JOINTS)

    def test_effort_macro_declares_command_interface(self, xacro_root):
        macros = self._find_macros(xacro_root, "so101_joint_interfaces_effort")
        cis = {ci.get("name") for ci in macros[0].iter("command_interface")}
        assert "effort" in cis
        assert "position" not in cis

    def test_has_mujoco_torque_hardware_macro(self, xacro_root):
        macros = self._find_macros(xacro_root, "so101_ros2_control_mujoco_torque")
        assert len(macros) == 1
        plugins = [p.text for p in macros[0].iter("plugin")]
        assert "mujoco_ros2_control/MujocoSystemInterface" in plugins


class TestControllersTorqueYaml:
    @pytest.fixture(scope="class")
    def config(self):
        return yaml.safe_load(CONTROLLERS_TORQUE_YAML.read_text())

    def test_cm_lists_three_controllers(self, config):
        cm = config["controller_manager"]["ros__parameters"]
        assert cm["joint_state_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
        assert cm["arm_controller"]["type"] == "forward_command_controller/ForwardCommandController"
        assert cm["gripper_controller"]["type"] == "forward_command_controller/ForwardCommandController"

    def test_arm_controller_effort_interface(self, config):
        arm = config["arm_controller"]["ros__parameters"]
        assert arm["joints"] == ARM_JOINTS
        assert arm["interface_name"] == "effort"  # scalar: ForwardCommandController rejects arrays

    def test_gripper_controller_effort_interface(self, config):
        gripper = config["gripper_controller"]["ros__parameters"]
        assert gripper["joints"] == ["gripper"]
        assert gripper["interface_name"] == "effort"  # scalar
