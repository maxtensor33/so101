"""夹持几何校准 (plan/mtc_fix.md Phase R1)。

在抓取位形 (GRASP_JOINTS) 下扫描 gripper 闭合角, 实测固定指面与动爪指面的
世界位姿, 计算:
  - 两指面的中点与夹持轴
  - "理想夹持位姿" (cube 中心 = 两指面中点, 留 gap 间隙, 无穿透)
  - 推荐 standoff (EE 原点到 cube 中心水平距离) 与 close_angle (最小闭合角)

几何事实 (2026-10 实测, 见 tests/test_grasp_geometry.py::TestMeasuredGeometry):
  - 闭合轴 = gripper body frame 的 x 轴 (抓取位形下水平)
  - 固定指 = wrist_roll_follower_so101_v1 mesh 的指刃部分 (|y|<Y_BLADE 排除掌部)
  - 指面 x 坐标随高度变化 (指刃斜面), 须在两指 z 重叠带中点切片测量
  - 开口随 grip 增大; grip 下限附近过闭合 (指面重叠)

用法:
    uv run python scripts/probe_grasp_geometry.py
纯函数 (finger_midpoint/ideal_cube_pose) 有 pytest 覆盖。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libs"))

import mujoco
import numpy as np

from so101_robotics import common

GAP = 0.001  # 指面与 cube 之间的目标间隙
CLOSE_SCAN = np.arange(-0.17, 1.75, 0.05)  # 闭合角扫描 (关节范围 -0.175~1.745)
GRASP_JOINTS = [0.59347, -0.82848, 1.31785, 1.0817, -1.50441]  # economy 无自碰撞解

FOLLOWER_MESH = "wrist_roll_follower_so101_v1"  # 固定指所在 mesh
Y_BLADE = 0.013  # 指刃 y 半宽阈值 (排除掌部基座, 基座 |y|~0.024)
SLAB = 0.005  # 指面测量的 z 切片半宽


def finger_midpoint(fixed_point, moving_point):
    """两指面上代表点的中点。"""
    return 0.5 * (np.asarray(fixed_point, dtype=float)
                  + np.asarray(moving_point, dtype=float))


def ideal_cube_pose(base, axis, fixed_coord, moving_coord, cube_half, gap):
    """理想夹持位姿: cube 中心在两指面中点, 沿轴留 gap。

    base: 夹持轴上参考点 (3,)   axis: 夹持轴单位向量 (3,)
    fixed_coord/moving_coord: 两指面在轴上的坐标 (标量, fixed > moving)
    返回 cube 中心 (3,)。
    """
    axis = np.asarray(axis, dtype=float)
    base = np.asarray(base, dtype=float)
    mid = 0.5 * (fixed_coord + moving_coord)
    return base + axis * mid


def _mesh_verts_in_body(model, data, gid, body_id):
    """mesh geom 的顶点变换到指定 body frame。"""
    did = model.geom_dataid[gid]
    vstart = model.mesh_vertadr[did]
    vnum = model.mesh_vertnum[did]
    verts = model.mesh_vert[vstart:vstart + vnum]
    # geom local -> world
    R_g = data.geom_xmat[gid].reshape(3, 3)
    c_g = data.geom_xpos[gid]
    world = verts @ R_g.T + c_g
    # world -> body frame
    R_b = data.xmat[body_id].reshape(3, 3)
    p_b = data.xpos[body_id]
    return (world - p_b) @ R_b


def _first_collision_geom_with_mesh(model, body_id, mesh_name):
    """body 中第一个匹配 mesh 名的 geom id (visual/collision 成对, 取其一)。"""
    for i in range(model.body_geomnum[body_id]):
        gid = model.body_geomadr[body_id] + i
        did = model.geom_dataid[gid]
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, did) == mesh_name:
            return gid
    raise ValueError(f"body {body_id} 中无 mesh {mesh_name}")


def _finger_faces_in_gripper_frame(model, data):
    """在 gripper frame 实测两指内面 (闭合轴 = gripper x)。

    指刃判定: |y| < Y_BLADE, 且位于两指 z 重叠带; 在重叠带中点 ±SLAB 切片
    取内面 x (固定指 x 最大 / 动爪 x 最小)。
    返回 dict: fixed_x/moving_x (gripper 系), band_lo/band_hi (重叠带),
    axis_g (gripper 系单位轴, 指向 fixed 侧), 无重叠带时返回 None。
    """
    gb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    jb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
    gid_f = _first_collision_geom_with_mesh(model, gb, FOLLOWER_MESH)
    gid_j = _first_collision_geom_with_mesh(model, jb, "moving_jaw_so101_v1")

    vf = _mesh_verts_in_body(model, data, gid_f, gb)
    vj = _mesh_verts_in_body(model, data, gid_j, gb)  # 统一到 gripper 系
    fb = vf[np.abs(vf[:, 1]) < Y_BLADE]
    jbl = vj[np.abs(vj[:, 1]) < Y_BLADE]
    if len(fb) == 0 or len(jbl) == 0:
        return None
    band_lo = max(fb[:, 2].min(), jbl[:, 2].min())
    band_hi = min(fb[:, 2].max(), jbl[:, 2].max())
    if band_hi <= band_lo:
        return None  # 无重叠带 (爪全开时 jaw 指尖抬出固定指范围)
    z_mid = 0.5 * (band_lo + band_hi)
    slab_f = fb[np.abs(fb[:, 2] - z_mid) < SLAB]
    slab_j = jbl[np.abs(jbl[:, 2] - z_mid) < SLAB]
    if len(slab_f) == 0 or len(slab_j) == 0:
        return None
    fixed_x = float(slab_f[:, 0].max())   # 内面: 朝动爪一侧
    moving_x = float(slab_j[:, 0].min())  # 内面: 朝固定指一侧
    # 轴指向 fixed 侧, 保证 fixed > moving 的坐标约定
    axis_g = np.array([1.0, 0.0, 0.0]) if fixed_x > moving_x \
        else np.array([-1.0, 0.0, 0.0])
    return {
        "fixed_x": fixed_x,
        "moving_x": moving_x,
        "band_lo": float(band_lo),
        "band_hi": float(band_hi),
        "axis_g": axis_g,
    }


def _finger_face_positions(model, data):
    """实测两指内面的世界系代表点 (z 取两指重叠带中点)。

    返回 (axis_world, fixed_point, moving_point, span)，
    span = dot(fixed_point - moving_point, axis_world) > 0。
    无重叠带时返回 None。
    """
    gb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    faces = _finger_faces_in_gripper_frame(model, data)
    if faces is None:
        return None
    R_g = data.xmat[gb].reshape(3, 3)
    p_g = data.xpos[gb]
    axis = R_g @ faces["axis_g"]
    z_mid = 0.5 * (faces["band_lo"] + faces["band_hi"])

    def to_world(x_g):
        return p_g + R_g @ np.array([x_g, 0.0, z_mid])

    fixed_point = to_world(faces["fixed_x"])
    moving_point = to_world(faces["moving_x"])
    span = float(abs(faces["moving_x"] - faces["fixed_x"]))
    return axis, fixed_point, moving_point, span


def calibrate(cube_size=0.02, yaw_deg=-33.0):
    """主校准: 输出 standoff / close_angle / ideal_rel_pose 等。

    在抓取位形 (grasp_joints) 下从小到大扫描闭合角, 实测指面间距 span(θ)
    与两指 z 重叠带高; close_angle = 第一个满足 span >= cube+2*gap 且
    重叠带 >= cube_size 的 θ (真实解, 无则抛 ValueError)。
    ideal_rel_pose = 该闭合角下 cube 理想中心相对动爪 body 的位姿 (7 维,
    cube 朝向对齐 gripper frame)。
    yaw_deg 保留参数: cube 绕竖直轴对称, 不影响指面测量。
    """
    model = mujoco.MjModel.from_xml_path(str(common.DEFAULT_XML))
    data = mujoco.MjData(model)
    jaw_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                "moving_jaw_so101_v1")
    gripper_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper")

    base_state = dict(zip(
        ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
         "wrist_roll"], GRASP_JOINTS))

    def set_config(grip):
        for k, v in base_state.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, k)
            data.qpos[model.jnt_qposadr[jid]] = v
        data.qpos[model.jnt_qposadr[gripper_jid]] = grip
        mujoco.mj_forward(model, data)

    target_span = cube_size + 2 * GAP
    close_angle = None
    for grip in CLOSE_SCAN:
        set_config(grip)
        faces = _finger_faces_in_gripper_frame(model, data)
        if faces is None:
            continue
        band = faces["band_hi"] - faces["band_lo"]
        if band < cube_size:
            continue  # 竖直方向夹不住
        span = abs(faces["moving_x"] - faces["fixed_x"])
        if span >= target_span:
            close_angle = float(grip)
            break
    if close_angle is None:
        raise ValueError(
            f"cube {cube_size * 1000:.0f}mm 在爪可行族内不可达 "
            f"(需开口 {target_span * 1000:.1f}mm)")

    set_config(close_angle)
    axis, fpt, mpt, span = _finger_face_positions(model, data)
    center = finger_midpoint(fpt, mpt)
    jp = data.xpos[jaw_bid].copy()
    standoff = float(np.linalg.norm((center - jp)[[0, 1]]))

    # ideal_rel_pose: cube 中心相对动爪 body 的位姿 (7 维 [x,y,z,qw,qx,qy,qz]),
    # cube 朝向对齐 gripper frame (指面平行于 cube 面)
    R_j = data.xmat[jaw_bid].reshape(3, 3)
    R_g = data.xmat[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                      "gripper")].reshape(3, 3)
    rel_p = R_j.T @ (center - jp)
    rel_q = np.zeros(4)
    mujoco.mju_mat2Quat(rel_q, (R_j.T @ R_g).flatten())

    return {
        "standoff": standoff,
        "close_angle": close_angle,
        "ideal_rel_pose": np.concatenate([rel_p, rel_q]).tolist(),
        "fixed_surface_y": float(np.dot(fpt, axis)),
        "moving_surface_y": float(np.dot(mpt, axis)),
        "span_closed": float(span),
        "axis": axis.tolist(),
    }


def main():
    out = calibrate(cube_size=0.02, yaw_deg=-33.0)
    print("=" * 60)
    print("夹持几何校准 (cube 2cm)")
    print("=" * 60)
    print(f"  两指面间距 (closed {out['close_angle']:.2f} rad): "
          f"{out['span_closed'] * 1000:.1f} mm")
    print(f"  固定指面/动爪指面 (轴坐标): "
          f"{out['fixed_surface_y'] * 1000:.1f} / "
          f"{out['moving_surface_y'] * 1000:.1f} mm")
    print(f"  推荐 standoff (jaw->cube 水平): {out['standoff'] * 1000:.1f} mm")
    print(f"  推荐 close_angle: {out['close_angle']:.2f} rad")
    print(f"  ideal_rel_pose: {[round(v, 4) for v in out['ideal_rel_pose']]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
