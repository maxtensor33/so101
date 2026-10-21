"""MuJoCo viewer 版 draw cube — 对标 lab-2 c3_draw_cube.py (plan/robotics.md Phase 4/5)。

流水线 (全部复用 libs/so101_robotics):
  立方体 12 边 → 采样航点 → DLS IK (Q_REF 温启动链) → quintic 关节轨迹
  → computed torque (M·ddq_d + C + g + PD) → MuJoCo 力矩仿真
  → viewer.user_scn 覆盖层: 绿色 cube 线框 + 红色 EE 轨迹 trail + 黄色 EE 标记

与 ROS 链路 (so101_robotics_torque_app/draw_cube_node) 使用相同默认配置:
  SIDES_TO_TRY=[0.08,0.07,0.06], POINTS_PER_EDGE=6, Q_REF, CUBE_CENTER_OFFSET

运行 (需要图形环境):
    uv run python scripts/draw_cube_viewer.py [--side 0.06] [--edge-duration 1.5] [--kp 50] [--kd 5]
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libs"))

import mujoco
import mujoco.viewer
import numpy as np

from so101_robotics import common, control, constraints, fk, ik
from so101_robotics.trajectory import JointTrajectory

# 场景版 MJCF (torque 模型 + 地板 + 灯光), 仿真实际加载的就是它
SCENE_XML = Path(__file__).resolve().parents[1] / "src" / "so101_sim" / "mujoco" / "scene_torque.xml"

# ── 配置 (与 ROS draw_cube_node 一致) ────────────────────────────────────────
Q_REF = np.array([0.0, 0.8, -1.5, 0.7, 0.0, 0.3])
CUBE_CENTER_OFFSET = np.array([-0.045, 0.02, -0.005])
SIDES_TO_TRY = [0.08, 0.07, 0.06]
POINTS_PER_EDGE = 6
SEGMENT_DURATION = 1.5   # 每条边 s
TAU_LIMIT = 3.35         # STS3215 力矩上限 Nm
KP = 150.0               # computed torque PD 增益 (500Hz 内环; 实测 RMS 2.4mm)
KD = 15.0

# ── 可视化 ──────────────────────────────────────────────────────────────────
TRAIL_DECIMATE = 10
TRAIL_COLOR = np.array([1.0, 0.15, 0.15, 0.85], dtype=np.float32)
CUBE_COLOR = np.array([0.2, 0.8, 0.2, 0.5], dtype=np.float32)
EE_MARKER_COLOR = np.array([1.0, 1.0, 0.0, 1.0], dtype=np.float32)

CUBE_EDGES = [  # 底面 4 边 + 顶面 4 边 + 立柱 4 边
    (0, 1), (1, 2), (2, 3), (3, 0),   # bottom
    (4, 5), (5, 6), (6, 7), (7, 4),   # top
    (0, 4), (1, 5), (2, 6), (3, 7),   # pillars
]


def joint_adrs(model):
    """scene 模型按关节名索引 qpos/qvel 地址 (兼容未来加入 freejoint)。"""
    jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in common.JOINT_NAMES]
    return ([int(model.jnt_qposadr[j]) for j in jids],
            [int(model.jnt_dofadr[j]) for j in jids])


def maximize_window(title_prefix="mujoco", retries=5, delay=0.4):
    """X11: 将 MuJoCo viewer 窗口最大化 (EWMH _NET_WM_STATE)。

    精确前缀匹配标题 (如 "MuJoCo : scene"), 避免误伤标题含 mujoco 的其他窗口。
    Wayland / 无窗口管理器 / Xlib 不可用时静默跳过 (仅隐藏面板)。
    """
    try:
        from Xlib import X, Xatom
        from Xlib.display import Display
        from Xlib.protocol import event as xevent
    except ImportError:
        print("  tip: uv add python-xlib 后可自动最大化窗口")
        return
    try:
        d = Display()
        root = d.screen().root
        state = d.intern_atom("_NET_WM_STATE")
        maxv = d.intern_atom("_NET_WM_STATE_MAXIMIZED_VERT")
        maxh = d.intern_atom("_NET_WM_STATE_MAXIMIZED_HORZ")
        name_atoms = (d.intern_atom("_NET_WM_NAME"), d.intern_atom("WM_NAME"))
        for _ in range(retries):
            client_list = root.get_full_property(
                d.intern_atom("_NET_CLIENT_LIST"), Xatom.WINDOW)
            if client_list is not None:
                for wid in client_list.value:
                    w = d.create_resource_object("window", int(wid))
                    title = None
                    for atom in name_atoms:
                        prop = w.get_full_property(atom, 0)
                        if prop is not None and prop.value:
                            v = prop.value
                            title = (v.decode("utf-8", "ignore")
                                     if isinstance(v, bytes) else str(v))
                            break
                    if title and title.lower().startswith(title_prefix):
                        ev = xevent.ClientMessage(
                            window=w, client_type=state,
                            data=(32, [1, int(maxv), int(maxh), 0, 0]),
                            message_type=state, format=32)
                        # EWMH: ClientMessage 必须发往 root window
                        root.send_event(ev, event_mask=X.SubstructureRedirectMask
                                        | X.SubstructureNotifyMask)
                        d.flush()
                        return
            time.sleep(delay)
    except Exception:
        pass


def cube_vertices(center, side):
    """8 顶点: 0-3 底面 (z-), 4-7 顶面 (z+)。"""
    h = side / 2.0
    cx, cy, cz = center
    return np.array([
        [cx - h, cy - h, cz - h],
        [cx + h, cy - h, cz - h],
        [cx + h, cy + h, cz - h],
        [cx - h, cy + h, cz - h],
        [cx - h, cy - h, cz + h],
        [cx + h, cy - h, cz + h],
        [cx + h, cy + h, cz + h],
        [cx - h, cy + h, cz + h],
    ])


def edge_waypoints(verts, points_per_edge=POINTS_PER_EDGE):
    """12 条边按 CUBE_EDGES 顺序采样, 每边 points_per_edge 点 (端点重复)。"""
    pts = []
    for a, b in CUBE_EDGES:
        for s in np.linspace(0.0, 1.0, points_per_edge):
            pts.append(verts[a] * (1 - s) + verts[b] * s)
    return pts


def solve_path(center, side, q_init=None, max_iters=500, n_restarts=10):
    """全航点 IK (温启动链); 任一点失败返回 None。"""
    if q_init is None:
        q_init = Q_REF.copy()
    pts = edge_waypoints(cube_vertices(center, side))
    joint_wps = []
    q0 = q_init.copy()
    for p in pts:
        r = ik.ik_dls(p, q_init=q0, max_iters=max_iters, n_restarts=n_restarts)
        if not r.converged:
            return None
        joint_wps.append(r.q)
        q0 = r.q
    return joint_wps


def run(show_viewer=True, side=None, edge_duration=SEGMENT_DURATION, kp=KP, kd=KD):
    """主仿真; 返回 {"errors": [(t, err_m)], "trail": [pos], "side": m}。"""
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    ee_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, common.EE_BODY)
    qadr, vadr = joint_adrs(model)
    kp6, kd6 = np.full(6, kp), np.full(6, kd)

    # 立方体: 从大到小探测首个全可达边长
    center = fk.pinocchio_fk(Q_REF).translation + CUBE_CENTER_OFFSET
    side_used, joint_wps = None, None
    for s in ([side] if side else SIDES_TO_TRY):
        joint_wps = solve_path(center, s)
        if joint_wps is not None:
            side_used = s
            break
    if joint_wps is None:
        raise RuntimeError("no reachable cube side in SIDES_TO_TRY")
    print(f"  cube side = {side_used * 100:.0f} cm, "
          f"{12 * POINTS_PER_EDGE} waypoints all reachable")

    # 轨迹: Q_REF → 各航点。段时长按关节空间距离自适应 (IK 分支翻转会造成
    # 1.5+ rad 的大段, 固定短时长会导致力矩饱和失控), 关节速度限制 ~1 rad/s
    seg_dur = edge_duration / (POINTS_PER_EDGE - 1)
    traj_wps = [Q_REF.copy(), *joint_wps]
    dists = [float(np.linalg.norm(b - a)) for a, b in zip(traj_wps[:-1], traj_wps[1:])]
    durations = [max(seg_dur, d / 1.0) for d in dists]
    traj = JointTrajectory(traj_wps, durations)
    total_t = traj.duration

    # 初始位形 = Q_REF, settle 3s
    data.qpos[qadr] = Q_REF
    data.qvel[vadr] = 0.0
    mujoco.mj_forward(model, data)

    settle_end = 3.0

    def hold_tau():
        return control.computed_torque(
            data.qpos[qadr].copy(), data.qvel[vadr].copy(),
            Q_REF, np.zeros(6), np.zeros(6), kp6, kd6)

    viewer = None
    if show_viewer:
        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False,
                                              show_right_ui=False)
        time.sleep(0.8)  # 等 GLFW 窗口注册进 X11 CLIENT_LIST 再最大化
        maximize_window("mujoco")
        viewer.cam.lookat[:] = center
        viewer.cam.distance = 0.9
        viewer.cam.elevation = -25

    trail, errors = [], []
    last_progress = -2.0
    step = 0
    def draw_overlay(scn):
        scn.ngeom = 0
        verts = cube_vertices(center, side_used)
        for a, b in CUBE_EDGES:
            if scn.ngeom >= scn.maxgeom:
                return
            mujoco.mjv_connector(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_CAPSULE,
                                 0.002, verts[a], verts[b])
            scn.geoms[scn.ngeom].rgba = CUBE_COLOR
            scn.ngeom += 1
        for p in trail[-600:]:
            if scn.ngeom >= scn.maxgeom:
                return
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                size=np.array([0.003, 0, 0]), pos=p,
                                mat=np.eye(3).flatten(), rgba=TRAIL_COLOR)
            scn.ngeom += 1
        if scn.ngeom < scn.maxgeom:
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                size=np.array([0.006, 0, 0]), pos=data.xpos[ee_bid],
                                mat=np.eye(3).flatten(), rgba=EE_MARKER_COLOR)
            scn.ngeom += 1

    t_wall0 = time.time()
    while True:
        if viewer is not None and not viewer.is_running():
            print("  viewer closed by user, stopping.")
            break
        t = data.time
        if t < settle_end:
            tau = hold_tau()
        elif t < settle_end + total_t:
            q_d, dq_d, ddq_d = traj.sample(t - settle_end)
            q, dq = data.qpos[qadr].copy(), data.qvel[vadr].copy()
            tau = control.computed_torque(q, dq, q_d, dq_d, ddq_d, kp6, kd6)
            err = float(np.linalg.norm(
                data.xpos[ee_bid] - fk.mujoco_fk(q_d, body=common.EE_BODY).translation))
            errors.append((t - settle_end, err))
            if step % TRAIL_DECIMATE == 0:
                trail.append(data.xpos[ee_bid].copy())
        else:
            tau = hold_tau()
            if t > settle_end + total_t + 2.0:
                break  # 完成后保持 2s 供观看

        tau = constraints.saturate_torque(tau, TAU_LIMIT)
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)

        if viewer is not None and step % TRAIL_DECIMATE == 0:
            draw_overlay(viewer.user_scn)
            viewer.sync()

        t_tr = t - settle_end
        if 0 <= t_tr < total_t and t_tr - last_progress >= 2.0:
            recent = np.array([e for tt, e in errors if tt > t_tr - 2.0])
            print(f"  t={t_tr:6.2f}/{total_t:.1f}s  "
                  f"recent_rms={np.sqrt(np.mean(recent**2)) * 1000:6.2f} mm")
            last_progress = t_tr
        step += 1

        # 实时节拍 (仅 viewer 模式): 仿真时间对齐 wall clock, 避免全速跑满 CPU
        if viewer is not None:
            remain = (t_wall0 + data.time) - time.time()
            if remain > 0:
                time.sleep(min(remain, 0.01))

    return {"errors": errors, "trail": trail, "side": side_used,
            "wall_time": time.time() - t_wall0}


def main():
    ap = argparse.ArgumentParser(description="SO-ARM101 draw cube (MuJoCo viewer, torque)")
    ap.add_argument("--side", type=float, default=None, help="立方体边长 m (默认探测)")
    ap.add_argument("--edge-duration", type=float, default=SEGMENT_DURATION)
    ap.add_argument("--kp", type=float, default=KP)
    ap.add_argument("--kd", type=float, default=KD)
    args = ap.parse_args()

    print("=" * 64)
    print("SO-ARM101 draw cube — MuJoCo viewer (torque, computed torque ctrl)")
    print("=" * 64)

    t0 = time.time()
    result = run(show_viewer=True, side=args.side, edge_duration=args.edge_duration,
                 kp=args.kp, kd=args.kd)
    errors = result["errors"]
    if errors:
        es = np.array([e for _, e in errors])
        rms = float(np.sqrt(np.mean(es**2)))
        print("=" * 64)
        print("  Final results")
        print(f"    RMS  EE error : {rms * 1000:.2f} mm")
        print(f"    Max  EE error : {es.max() * 1000:.2f} mm")
        print(f"    Samples       : {len(es)}")
        print(f"    Cube side     : {result['side']} m")
        print("=" * 64)


if __name__ == "__main__":
    main()
