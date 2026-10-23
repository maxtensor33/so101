"""pick/place 摆位可达性探测 (plan/mtc.md Phase 0)。

so101 工作空间小, pick 与 place 双点须同时可达。本脚本在极坐标网格上扫描
cube 摆位, 对每处生成 top-grasp 序列位姿 (pre-grasp / grasp / lift) 并用
DLS IK (6D, Q_REF 温启动) 验证, 输出可行的 (pick, place) 组合。

纯函数 (grasp_pose/candidate_picks/...) 有 pytest 覆盖
(tests/test_probe_pick_poses.py); 全量扫描耗时, 作为脚本运行:

    uv run python scripts/probe_pick_poses.py [--cube-size 0.04]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libs"))

import mujoco
import numpy as np
import pinocchio as pin

from so101_robotics import common, ik

Q_INIT = np.array([0.0, 0.8, -1.5, 0.7, 0.0, 0.3])  # draw cube 验证过的收臂参考
CUBE_HALF = 0.01  # 默认 2cm cube 的半高 (R2: 4cm -> 2cm, cube 底面在地面 z=0)

# R1 实测理想夹持位姿 (scripts/probe_grasp_geometry.py calibrate, cube 2cm):
# cube 理想中心相对动爪 (EE) body 的位姿 [x,y,z,qw,qx,qy,qz],
# cube 朝向对齐 gripper frame, 与两指面等距 (留 1mm 间隙)。
# 旧摆位把 EE 放 cube 正上方 —— cube 从未处于两指面之间, 是 "贴一边提起"
# 与 "attach 后穿模" 的共同根因 (plan/mtc_fix.md)。
IDEAL_REL_POSE = [-0.019618, -0.048269, 0.0188,
                  0.707071, -0.707071, -0.007071, 0.007071]
_q = np.array(IDEAL_REL_POSE[3:], dtype=float)
_q /= np.linalg.norm(_q)  # 常量按 6 位小数粘贴, 构造前归一化保证正交
T_REL = pin.SE3(
    np.array(pin.Quaternion(*_q).matrix()),
    np.array(IDEAL_REL_POSE[:3]))
# standoff: EE 原点到 cube 中心的水平偏移 (T_REL 反解, R1 实测 ~27.9mm)
STANDOFF = float(np.linalg.norm(T_REL.inverse().translation[:2]))


def has_self_collision(q, tol=0.0):
    """mujoco 自碰撞校验 (R2, economy 教训: 原 IK 只查关节限位)。

    q: (6,) [5 臂关节 + gripper]。用 DEFAULT_XML (纯机器人, 无地板/cube
    等环境 geom) 做 mj_forward, 存在任何接触 (dist < -tol) 即自碰撞。
    """
    q = np.asarray(q, dtype=float)
    if not hasattr(has_self_collision, "_model"):
        has_self_collision._model = mujoco.MjModel.from_xml_path(
            str(common.DEFAULT_XML))
        has_self_collision._data = mujoco.MjData(has_self_collision._model)
    model = has_self_collision._model
    data = has_self_collision._data
    for name, v in zip(common.JOINT_NAMES, q):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = v
    mujoco.mj_forward(model, data)
    for i in range(data.ncon):
        if data.contact[i].dist < -tol:
            return True
    return False


def grasp_pose(cube_center, grasp_height=0.05, yaw_deg=0.0):
    """top-grasp EE 目标位姿 (R2: 由 R1 实测 ideal_rel_pose 构造)。

    T_ee = T_cube ∘ inv(T_rel): EE (动爪) 相对 cube 的理想位姿来自
    probe_grasp_geometry.calibrate() —— EE 原点在 cube 侧方 standoff 处
    (正对两指面中点), 不再是 cube 正上方。
    grasp_height 参数保留旧签名兼容但不再使用: 高度由 T_rel 实测值决定
    (R1: EE 在 cube 中心上方 ~4.8cm)。
    """
    del grasp_height  # 高度由 T_REL 决定, 保留参数兼容旧调用
    c = float(np.cos(np.radians(yaw_deg)))
    s = float(np.sin(np.radians(yaw_deg)))
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    T_cube = pin.SE3(Rz, np.asarray(cube_center, dtype=float))
    return T_cube * T_REL.inverse()


def pregrasp_pose(grasp, clearance=0.05):
    """预抓取 = 抓取位沿世界 z 上移 clearance (水平位置不变)。"""
    t = grasp.translation + np.array([0.0, 0.0, clearance])
    return pin.SE3(grasp.rotation.copy(), t)


def candidate_picks(r_range=(0.10, 0.16), yaw_range=(-40, 40), n_r=4, n_yaw=5,
                    half_height=CUBE_HALF):
    """pick 候选: 极坐标网格 (cube 底面贴地, z=半高)。"""
    picks = []
    for r in np.linspace(r_range[0], r_range[1], n_r):
        for yaw in np.linspace(yaw_range[0], yaw_range[1], n_yaw):
            a = np.radians(yaw)
            picks.append(np.array([r * np.cos(a), r * np.sin(a), half_height]))
    return picks


def candidate_places(picks, dyaw=60, half_height=CUBE_HALF):
    """place 候选: 与 pick 同半径, 方位角错开 dyaw。"""
    places = []
    for c in picks:
        yaw = np.degrees(np.arctan2(c[1], c[0])) + dyaw
        a = np.radians(yaw)
        r = np.hypot(c[0], c[1])
        places.append(np.array([r * np.cos(a), r * np.sin(a), half_height]))
    return places


def grasp_sequence(cube_center, grasp_height=0.05, lift=0.03, yaw_deg=0.0):
    """抓取序列 [(pre, pose), (grasp, pose), (lift, pose)]。"""
    g = grasp_pose(cube_center, grasp_height, yaw_deg)
    return [("pre-grasp", pregrasp_pose(g)),
            ("grasp", g),
            ("lift", pregrasp_pose(g, clearance=lift))]


def probe_pair(cube_center, place_center, grasp_height=0.05, lift=0.03,
               place_lift=0.05, max_iters=200):
    """验证单个 (pick, place) 组合: pre/grasp/lift + pre-place/place 序列全收敛。

    返回 (ok, 关节解列表或 None)。
    """
    q0 = Q_INIT.copy()
    sols = []
    seq = grasp_sequence(cube_center, grasp_height, lift)
    # place 阶段: place 上方 → place 高度 (放下前)
    p = grasp_pose(place_center, place_lift)
    seq += [("pre-place", pregrasp_pose(p)), ("place", p)]
    for name, target in seq:
        r = ik.ik_dls(target, q_init=q0, max_iters=max_iters)
        if not r.converged:
            return False, None
        # R2: 自碰撞校验 (economy 教训: 穿肩解曾致 move_group 目标 state 无效)
        if has_self_collision(r.q):
            return False, None
        sols.append(r.q.copy())
        q0 = r.q
    return True, sols


def main():
    ap = argparse.ArgumentParser(description="so101 pick/place 摆位可达性探测")
    ap.add_argument("--cube-size", type=float, default=0.02)
    ap.add_argument("--max-iters", type=int, default=200)
    args = ap.parse_args()
    half = args.cube_size / 2.0

    print("=" * 64)
    print(f"探测 pick/place 摆位 (cube {args.cube_size*100:.0f} cm, "
          f"standoff {STANDOFF*1000:.1f} mm, 自碰撞校验开)")
    print("=" * 64)

    picks = candidate_picks(r_range=(0.10, 0.16), yaw_range=(-50, 50), n_r=4, n_yaw=7,
                            half_height=half)
    places = candidate_places(picks, dyaw=60, half_height=half)

    feasible = []
    for i, (p, q) in enumerate(zip(picks, places)):
        ok, sols = probe_pair(p, q, max_iters=args.max_iters)
        status = "OK " if ok else "FAIL"
        print(f"  [{status}] pick=({p[0]:+.3f},{p[1]:+.3f}) "
              f"place=({q[0]:+.3f},{q[1]:+.3f})")
        if ok:
            feasible.append((p, q, sols))

    print("-" * 64)
    print(f"可行组合: {len(feasible)}/{len(picks)}")
    if feasible:
        p, q, sols = feasible[0]
        print(f"\n推荐摆位 (写入场景配置):")
        print(f"  PICK_CUBE  = [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:.4f}]")
        print(f"  PLACE_CUBE = [{q[0]:+.4f}, {q[1]:+.4f}, {q[2]:.4f}]")
        # 序列关节解: [pre, grasp, lift, pre-place, place] (6 dof 含 gripper)
        print(f"\n推荐关节解 (粘贴进 mtc_pick_place_node 参数, 取前 5 臂关节):")
        print(f"  grasp_joints = {{{', '.join(f'{v:.5f}' for v in sols[1][:5])}}}")
        print(f"  lift_joints  = {{{', '.join(f'{v:.5f}' for v in sols[2][:5])}}}")
        print(f"  place_joints = {{{', '.join(f'{v:.5f}' for v in sols[4][:5])}}}")
        return 0
    print("无可行组合——需调整 grasp_height/立方体尺寸或扩大扫描范围")
    return 1


if __name__ == "__main__":
    sys.exit(main())
