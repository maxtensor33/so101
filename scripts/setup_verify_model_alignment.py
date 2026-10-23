"""URDF ↔ MuJoCo XML 对齐验证（SO-ARM101）。

用 pinocchio 加载 models/so101_new_calib.urdf，
用 mujoco 加载 models/so101_new_calib.xml，
逐项对比：关节清单、关节限位、惯性参数、正向运动学（随机采样）。
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_URDF = REPO_ROOT / "models" / "so101_new_calib.urdf"
DEFAULT_XML = REPO_ROOT / "models" / "so101_new_calib.xml"
# 力矩控制版本 (workflow/plan/robotics.md Phase 0): 仅 actuator 不同, 运动学一致
DEFAULT_URDF_TORQUE = REPO_ROOT / "models" / "so101_torque.urdf"
DEFAULT_XML_TORQUE = REPO_ROOT / "models" / "so101_new_calib_torque.xml"

# URDF link 名 ↔ MuJoCo body 名
LINK_BODY_MAP = {
    "base_link": "base",
    "shoulder_link": "shoulder",
    "upper_arm_link": "upper_arm",
    "lower_arm_link": "lower_arm",
    "wrist_link": "wrist",
    "gripper_link": "gripper",
    "moving_jaw_so101_v1_link": "moving_jaw_so101_v1",
}

POS_TOL = 1e-3
ROT_TOL = np.deg2rad(0.5)
LIMIT_TOL = 1e-3
MASS_TOL = 1e-3
COM_TOL = 1e-3
N_SAMPLES = 100


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


def check_joint_limits(pin_model, mj_model):
    """检查各关节限位偏差（rad）。pinocchio 与 mujoco 各自从模型读取。"""
    max_dev = 0.0
    worst = ""
    for i in range(mj_model.njnt):
        name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jid = pin_model.getJointId(name)
        iq = pin_model.joints[jid].idx_q
        lo_pin = float(pin_model.lowerPositionLimit[iq])
        hi_pin = float(pin_model.upperPositionLimit[iq])
        lo_mj, hi_mj = mj_model.jnt_range[i]
        d = max(abs(lo_pin - lo_mj), abs(hi_pin - hi_mj))
        if d > max_dev:
            max_dev, worst = d, name
    passed = max_dev <= LIMIT_TOL
    detail = f"max |d| = {max_dev:.6f} rad at '{worst}' (tol {LIMIT_TOL})"
    return CheckResult("joint_limits", passed, detail)


def check_joint_list(pin_model, mj_model):
    """检查两模型关节名称与顺序一致。"""
    pin_names = [str(n) for n in pin_model.names][1:]  # 跳过 universe
    mj_names = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(mj_model.njnt)]
    if pin_names == mj_names:
        return CheckResult("joint_list", True, f"{len(pin_names)} joints aligned: {pin_names}")
    return CheckResult("joint_list", False, f"pinocchio={pin_names} mujoco={mj_names}")


def urdf_link_masses(urdf_path):
    """URDF 各 link 质量/质心：link -> (mass, com[3])。"""
    root = ET.parse(urdf_path).getroot()
    out = {}
    for link in root.iter("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        com = np.fromstring(inertial.find("origin").get("xyz"), sep=" ")
        out[link.get("name")] = (mass, com)
    return out


def mjcf_body_masses(xml_path):
    """MuJoCo 各 body 质量/质心：body -> (mass, com[3])。"""
    root = ET.parse(xml_path).getroot()
    out = {}
    for body in root.iter("body"):
        inertial = body.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.get("mass"))
        com = np.fromstring(inertial.get("pos"), sep=" ")
        out[body.get("name")] = (mass, com)
    return out


def check_inertials(urdf_path, xml_path):
    """按 LINK_BODY_MAP 对比各连杆质量与质心。"""
    urdf_m = urdf_link_masses(urdf_path)
    mj_m = mjcf_body_masses(xml_path)
    max_dm, max_dc, worst = 0.0, 0.0, ""
    for link, body in LINK_BODY_MAP.items():
        if link not in urdf_m or body not in mj_m:
            continue
        m_u, c_u = urdf_m[link]
        m_j, c_j = mj_m[body]
        dm = abs(m_u - m_j)
        dc = float(np.max(np.abs(c_u - c_j)))
        if dm > max_dm or dc > max_dc:
            worst = f"{link}<->{body}"
        max_dm = max(max_dm, dm)
        max_dc = max(max_dc, dc)
    passed = max_dm <= MASS_TOL and max_dc <= COM_TOL
    detail = f"max |dm| = {max_dm:.6f} kg, max |dc| = {max_dc:.6f} m, worst '{worst}'"
    return CheckResult("inertials", passed, detail)


def _sample_configs(pin_model, mj_model, n_samples, seed):
    """在各关节限位交集内均匀采样 n 组配置。返回 (n, nv_pin) 数组。"""
    rng = np.random.default_rng(seed)
    names = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(mj_model.njnt)]
    lo = np.zeros(pin_model.nq)
    hi = np.zeros(pin_model.nq)
    for i, name in enumerate(names):
        jid = pin_model.getJointId(name)
        iq = pin_model.joints[jid].idx_q
        lo[iq] = max(pin_model.lowerPositionLimit[iq], mj_model.jnt_range[i][0])
        hi[iq] = min(pin_model.upperPositionLimit[iq], mj_model.jnt_range[i][1])
    return rng.uniform(lo, hi, size=(n_samples, pin_model.nq))


def check_fk(pin_model, mj_model, n_samples, seed):
    """随机配置下对比各公共连杆世界位姿。决定性对齐测试。"""
    names = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(mj_model.njnt)]
    # 各关节在 pinocchio q 向量中的索引 与 mujoco qpos 中的地址
    pin_idx = [pin_model.joints[pin_model.getJointId(n)].idx_q for n in names]
    mj_adr = [mj_model.jnt_qposadr[mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
    # 公共连杆的 frame/body id
    pairs = []
    for link, body in LINK_BODY_MAP.items():
        fid = pin_model.getFrameId(link, pin.BODY)
        bid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, body)
        pairs.append((link, fid, bid))

    pin_data = pin.Data(pin_model)
    mj_data = mujoco.MjData(mj_model)
    max_pos, max_rot, worst = 0.0, 0.0, ""
    for q in _sample_configs(pin_model, mj_model, n_samples, seed):
        q_pin = pin.neutral(pin_model)
        for k, iq in enumerate(pin_idx):
            q_pin[iq] = q[k]
        for k, adr in enumerate(mj_adr):
            mj_data.qpos[adr] = q[k]
        mujoco.mj_forward(mj_model, mj_data)
        pin.forwardKinematics(pin_model, pin_data, q_pin)
        pin.updateFramePlacements(pin_model, pin_data)
        for link, fid, bid in pairs:
            oMf = pin_data.oMf[fid]
            p_mj = mj_data.xpos[bid]
            R_mj = mj_data.xmat[bid].reshape(3, 3)
            e_pos = float(np.linalg.norm(oMf.translation - p_mj))
            e_rot = float(np.linalg.norm(pin.log3(oMf.rotation @ R_mj.T)))
            if e_pos > max_pos or e_rot > max_rot:
                worst = link
            max_pos = max(max_pos, e_pos)
            max_rot = max(max_rot, e_rot)
    passed = max_pos <= POS_TOL and max_rot <= ROT_TOL
    detail = (
        f"n_samples={n_samples} max_pos={max_pos * 1000:.4f} mm (tol {POS_TOL * 1000:.1f}), "
        f"max_rot={np.rad2deg(max_rot):.4f} deg (tol {np.rad2deg(ROT_TOL):.2f}), worst '{worst}'"
    )
    return CheckResult("forward_kinematics", passed, detail)


def run_verification(urdf_path, xml_path, n_samples=N_SAMPLES, seed=0):
    """执行全部检查，返回 CheckResult 列表。"""
    pin_model = pin.buildModelFromUrdf(str(urdf_path))
    mj_model = mujoco.MjModel.from_xml_path(str(xml_path))
    results = [
        check_joint_list(pin_model, mj_model),
        check_joint_limits(pin_model, mj_model),
        check_inertials(urdf_path, xml_path),
        check_fk(pin_model, mj_model, n_samples, seed),
    ]
    return results


def write_report(results, out_path):
    """写出 Markdown 对齐报告。"""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# SO-ARM101 URDF ↔ MuJoCo XML 对齐报告",
        "",
        "| 检查项 | 结果 | 详情 |",
        "|---|---|---|",
    ]
    for r in results:
        verdict = "PASS" if r.passed else "FAIL"
        lines.append(f"| {r.name} | {verdict} | {r.detail} |")
    out_path.write_text("\n".join(lines) + "\n")
    return out_path


def main():
    parser = argparse.ArgumentParser(description="URDF ↔ MuJoCo 对齐验证")
    parser.add_argument(
        "--torque",
        action="store_true",
        help="验证力矩控制版本 (so101_torque.urdf ↔ so101_new_calib_torque.xml)",
    )
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--xml", type=Path, default=None)
    parser.add_argument("--n-samples", type=int, default=N_SAMPLES)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "reports" / "alignment_report.md")
    args = parser.parse_args()
    if args.torque:
        urdf = args.urdf or DEFAULT_URDF_TORQUE
        xml = args.xml or DEFAULT_XML_TORQUE
    else:
        urdf = args.urdf or DEFAULT_URDF
        xml = args.xml or DEFAULT_XML
    results = run_verification(urdf, xml, n_samples=args.n_samples, seed=args.seed)
    out = write_report(results, args.out)
    for r in results:
        print(f"[{'PASS' if r.passed else 'FAIL'}] {r.name}: {r.detail}")
    print(f"报告已写入 {out}")
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
