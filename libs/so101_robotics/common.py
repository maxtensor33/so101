"""so101_robotics 公共层: 常量、模型加载器、关节限位 (对标 lab-2 ur5e_common.py)。

默认加载力矩版模型 (workflow/plan/robotics.md: 重点 torque 版本):
  URDF : models/so101_torque.urdf          (Pinocchio 解析计算用)
  MJCF : models/so101_new_calib_torque.xml (MuJoCo 仿真/交叉验证用)
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

REPO_ROOT = Path(__file__).resolve().parents[2]
MODELS_DIR = REPO_ROOT / "models"

DEFAULT_URDF = MODELS_DIR / "so101_torque.urdf"
DEFAULT_XML = MODELS_DIR / "so101_new_calib_torque.xml"

# URDF link 名 ↔ MuJoCo body 名 (与 scripts/verify_model_alignment.py 保持一致)
LINK_BODY_MAP = {
    "base_link": "base",
    "shoulder_link": "shoulder",
    "upper_arm_link": "upper_arm",
    "lower_arm_link": "lower_arm",
    "wrist_link": "wrist",
    "gripper_link": "gripper",
    "moving_jaw_so101_v1_link": "moving_jaw_so101_v1",
}

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper",
]
ARM_JOINT_NAMES = JOINT_NAMES[:5]
N_ARM = 5
N_JOINTS = 6
N_V = 6

# 末端 frame: moving_jaw_so101_v1_link (动爪尖端, "笔尖")
# 不用 gripper_link: 掌部对 gripper 关节的 Jacobian 列恒为 0 (掌部不随夹爪关节运动)
EE_FRAME = "moving_jaw_so101_v1_link"
EE_BODY = LINK_BODY_MAP[EE_FRAME]

# home 位姿: 全零 (所有关节限位内)
Q_HOME = np.zeros(N_JOINTS)

_TORQUE_LIMIT = 3.35  # STS3215 forcerange (Nm)


@lru_cache(maxsize=1)
def get_pinocchio_model() -> pin.Model:
    """加载力矩版 URDF 的 Pinocchio 模型 (缓存单例)。"""
    return pin.buildModelFromUrdf(str(DEFAULT_URDF))


@lru_cache(maxsize=1)
def get_pin_data() -> pin.Data:
    return pin.Data(get_pinocchio_model())


@lru_cache(maxsize=1)
def get_mujoco_model() -> mujoco.MjModel:
    """加载力矩版 MJCF 的 MuJoCo 模型 (缓存单例)。"""
    return mujoco.MjModel.from_xml_path(str(DEFAULT_XML))


@lru_cache(maxsize=1)
def get_mj_data() -> mujoco.MjData:
    return mujoco.MjData(get_mujoco_model())


def get_joint_limits() -> np.ndarray:
    """(6, 2) 关节限位 [lower, upper] (rad), 按 JOINT_NAMES 顺序来自 Pinocchio 模型。"""
    model = get_pinocchio_model()
    rows = []
    for name in JOINT_NAMES:
        iq = model.joints[model.getJointId(name)].idx_q
        rows.append([model.lowerPositionLimit[iq], model.upperPositionLimit[iq]])
    return np.array(rows)