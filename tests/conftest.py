"""so101 模块路径注入：让测试可以 import scripts/setup_verify_model_alignment.py。"""

import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

MODELS_DIR = Path(__file__).resolve().parent.parent / "models"
REPO_ROOT = MODELS_DIR.parent
URDF_PATH = MODELS_DIR / "so101_new_calib.urdf"
XML_PATH = MODELS_DIR / "so101_new_calib.xml"

# 力矩控制版本 (plan/robotics.md Phase 0): xxx_torque 命名
URDF_TORQUE_PATH = MODELS_DIR / "so101_torque.urdf"
XML_TORQUE_PATH = MODELS_DIR / "so101_new_calib_torque.xml"
SCENE_TORQUE_PATH = REPO_ROOT / "src" / "so101_sim" / "mujoco" / "scene_torque.xml"
ROS2_CONTROL_XACRO = REPO_ROOT / "src" / "so101_sim" / "urdf" / "so101.ros2_control.xacro"
CONTROLLERS_TORQUE_YAML = REPO_ROOT / "src" / "so101_sim" / "config" / "controllers_torque.yaml"
