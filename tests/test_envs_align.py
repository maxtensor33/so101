"""envs MJCF ↔ URDF 对齐验证（workflow/plan/envs.md Phase 0, TDD）。

镜像 scripts/setup_verify_model_alignment.py 的模式，但针对 libs/so101_envs/assets/ 的
场景版 MJCF（机器人 + 桌面 + 任务物体）：
- 六关节名与顺序、限位与 models/so101_new_calib.urdf 对齐
- pinocchio(URDF) ↔ mujoco(scene XML) FK 随机采样对齐
- 位置 actuator 清单、TCP/wrist_cam site、fixed_cam 相机、任务物体存在
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
ASSETS = REPO_ROOT / "libs" / "so101_envs" / "assets"
URDF = REPO_ROOT / "models" / "so101_new_calib.urdf"

# MJCF 中机器人六关节（按 body 深度顺序，与 URDF/pinocchio 一致）
ROBOT_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# URDF link 名 ↔ MJCF body 名（同 setup_verify_model_alignment.py）
LINK_BODY_MAP = {
    "base_link": "base",
    "shoulder_link": "shoulder",
    "upper_arm_link": "upper_arm",
    "lower_arm_link": "lower_arm",
    "wrist_link": "wrist",
    "gripper_link": "gripper",
    "moving_jaw_so101_v1_link": "moving_jaw_so101_v1",
}

# 各场景必含的任务 body（桌面为 worldbody geom: table_top/table_body）
SCENE_BODIES = {
    "scene_reach.xml": ["item", "goal_site"],
    "scene_lift.xml": ["item"],
    "scene_place.xml": ["item", "bin"],
    "scene_stack.xml": ["itemA", "itemB"],
}
SCENES = list(SCENE_BODIES)

POS_TOL = 1e-3
ROT_TOL = np.deg2rad(0.5)
LIMIT_TOL = 1e-3
N_SAMPLES = 50
SEED = 0


@pytest.fixture(scope="module")
def pin_model():
    return pin.buildModelFromUrdf(str(URDF))


def _robot_joints(mj_model):
    """MJCF 中机器人六关节（按出现顺序，排除任务物体的 freejoint）。"""
    names = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(mj_model.njnt)]
    return [n for n in names if n in ROBOT_JOINTS]


@pytest.mark.parametrize("scene", SCENES)
def test_model_loads(scene, pin_model):
    """场景 XML 能被 mujoco 加载（资产路径/语法正确）。"""
    mj_model = mujoco.MjModel.from_xml_path(str(ASSETS / scene))
    assert mj_model.njnt >= len(ROBOT_JOINTS)


@pytest.mark.parametrize("scene", SCENES)
def test_joint_list_matches_urdf(scene, pin_model):
    mj_model = mujoco.MjModel.from_xml_path(str(ASSETS / scene))
    assert _robot_joints(mj_model) == ROBOT_JOINTS
    # pinocchio 侧顺序一致（跳过 universe 固定根）
    pin_names = [str(n) for n in pin_model.names][1:]
    assert pin_names == ROBOT_JOINTS


@pytest.mark.parametrize("scene", SCENES)
def test_joint_limits_match_urdf(scene, pin_model):
    mj_model = mujoco.MjModel.from_xml_path(str(ASSETS / scene))
    for i in range(mj_model.njnt):
        name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name not in ROBOT_JOINTS:
            continue
        jid = pin_model.getJointId(name)
        iq = pin_model.joints[jid].idx_q
        lo_pin = float(pin_model.lowerPositionLimit[iq])
        hi_pin = float(pin_model.upperPositionLimit[iq])
        lo_mj, hi_mj = mj_model.jnt_range[i]
        assert abs(lo_pin - lo_mj) <= LIMIT_TOL, f"{scene}:{name} lower"
        assert abs(hi_pin - hi_mj) <= LIMIT_TOL, f"{scene}:{name} upper"


@pytest.mark.parametrize("scene", SCENES)
def test_position_actuators(scene):
    """六个位置 actuator，名称与关节一致（位置控制核心约定）。"""
    mj_model = mujoco.MjModel.from_xml_path(str(ASSETS / scene))
    names = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(mj_model.nu)]
    robot_act = [n for n in names if n in ROBOT_JOINTS]
    assert robot_act == ROBOT_JOINTS
    for name in robot_act:
        idx = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        jid = mj_model.actuator_trnid[idx][0]
        jname = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        assert jname == name
        # 位置伺服: gainprm>0, biasprm=-kp（mujoco position actuator 约定）
        assert mj_model.actuator_gainprm[idx][0] > 0
        assert mj_model.actuator_biasprm[idx][1] == -mj_model.actuator_gainprm[idx][0]


@pytest.mark.parametrize("scene", SCENES)
def test_fk_alignment(scene, pin_model):
    """随机配置下 URDF(pinocchio) ↔ MJCF(mujoco) 各机器人连杆位姿对齐。"""
    mj_model = mujoco.MjModel.from_xml_path(str(ASSETS / scene))
    mj_data = mujoco.MjData(mj_model)
    mj_adr = [
        mj_model.jnt_qposadr[mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, n)]
        for n in ROBOT_JOINTS
    ]
    pin_idx = [pin_model.joints[pin_model.getJointId(n)].idx_q for n in ROBOT_JOINTS]
    pairs = []
    for link, body in LINK_BODY_MAP.items():
        pairs.append((link, pin_model.getFrameId(link, pin.BODY),
                      mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, body)))
    lo = np.array([mj_model.jnt_range[mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, n)][0] for n in ROBOT_JOINTS])
    hi = np.array([mj_model.jnt_range[mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, n)][1] for n in ROBOT_JOINTS])
    rng = np.random.default_rng(SEED)
    pin_data = pin.Data(pin_model)
    max_pos, max_rot = 0.0, 0.0
    for q in rng.uniform(lo, hi, size=(N_SAMPLES, len(ROBOT_JOINTS))):
        mj_data.qpos[:] = 0
        for k, adr in enumerate(mj_adr):
            mj_data.qpos[adr] = q[k]
        mujoco.mj_forward(mj_model, mj_data)
        q_pin = pin.neutral(pin_model)
        for k, iq in enumerate(pin_idx):
            q_pin[iq] = q[k]
        pin.forwardKinematics(pin_model, pin_data, q_pin)
        pin.updateFramePlacements(pin_model, pin_data)
        for link, fid, bid in pairs:
            oMf = pin_data.oMf[fid]
            e_pos = float(np.linalg.norm(oMf.translation - mj_data.xpos[bid]))
            R_mj = mj_data.xmat[bid].reshape(3, 3)
            e_rot = float(np.linalg.norm(pin.log3(oMf.rotation @ R_mj.T)))
            max_pos = max(max_pos, e_pos)
            max_rot = max(max_rot, e_rot)
    assert max_pos <= POS_TOL, f"{scene}: FK pos err {max_pos * 1000:.4f} mm"
    assert max_rot <= ROT_TOL, f"{scene}: FK rot err {np.rad2deg(max_rot):.4f} deg"


@pytest.mark.parametrize("scene", SCENES)
def test_robot_sites_and_wrist_camera(scene):
    """夹爪帧 site（TCP）与 wrist_cam 相机存在且挂在夹爪 body 上。"""
    mj_model = mujoco.MjModel.from_xml_path(str(ASSETS / scene))
    sid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    assert sid >= 0, "gripperframe site (TCP) missing"
    cid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")
    assert cid >= 0, "wrist_cam camera missing"
    gid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    assert mj_model.cam_bodyid[cid] == gid, "wrist_cam must attach to gripper body"


@pytest.mark.parametrize("scene", SCENES)
def test_fixed_camera(scene):
    """世界系固定第三人称相机 fixed_cam 存在。"""
    mj_model = mujoco.MjModel.from_xml_path(str(ASSETS / scene))
    assert mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed_cam") >= 0


@pytest.mark.parametrize("scene,bodies", sorted(SCENE_BODIES.items()))
def test_task_bodies(scene, bodies):
    """任务物体 body 存在且带 freejoint（可被动理推动/抓取）；桌面 geom 存在。"""
    mj_model = mujoco.MjModel.from_xml_path(str(ASSETS / scene))
    for body in bodies:
        bid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, body)
        assert bid >= 0, f"{scene}: body '{body}' missing"
    # 桌面为 worldbody geom
    assert mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_GEOM, "table_top") >= 0, \
        f"{scene}: table_top geom missing"
    for obj in ("item", "itemA", "itemB", "bin"):
        bid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, obj)
        if bid < 0:
            continue
        jid = mj_model.body_jntadr[bid]
        assert jid >= 0, f"{scene}: '{obj}' has no joint"
        assert mj_model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE, f"{scene}: '{obj}' not free"
