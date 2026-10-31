"""接触检测与四元数工具（squint 同款思路, mujoco 实现）。"""

from __future__ import annotations

import mujoco
import numpy as np

# 两"指"：gripper（固定爪）+ moving_jaw（活动爪）
ROBOT_BODIES = frozenset(
    ["base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"]
)

_MIN_FORCE = 0.5  # N


def mat2quat(R):
    """3x3 旋转矩阵 -> wxyz 四元数。"""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.reshape(-1))
    return q


def _pair_force(model, data, finger_id, item_id):
    """finger 受到 item 的合力矢量 (N) 与指向指外的单位开合轴。

    MuJoCo mj_contactForce 的力施加在 geom2 上（由 geom1 推 geom2）;
    contact.frame[:,0] 法向由 geom1 指向 geom2。据此把每对接触换算到"物体对
    手指的作用力"，并用同一批接触法向作为该指的开合轴（squint 双指方向判定的
    mujoco 等价实现: 指系 y 轴随 URDF/MJCF 帧约定不同，不能直接照搬）。
    """
    force = np.zeros(3)
    outward = np.zeros(3)
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = int(c.geom1)
        g2 = int(c.geom2)
        ids = {int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])}
        if ids != {finger_id, item_id} or finger_id == item_id:
            continue
        f = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, f)
        normal = np.asarray(c.frame).reshape(3, 3)[:, 0]
        if int(model.geom_bodyid[g2]) == finger_id:
            # 手指是 geom2: 力 = +f, 法向由物体指向手指 = 指外方向
            force += f[:3]
            outward += normal
        else:
            # 手指是 geom1: 力 = -f, 法向由手指指向物体, 指外方向取反
            force -= f[:3]
            outward -= normal
    return force, outward


def _body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def is_grasping(model, data, item_body, min_force=_MIN_FORCE, max_angle_deg=110.0):
    """双"指"对物体均有力学抓握 -> True（squint is_grasping 语义）。

    每指: 接触合力 >= min_force, 且合力方向与指外开合轴夹角 <= max_angle_deg
    （排除侧面推挤/切向接触的假抓握）。
    """
    item = _body_id(model, item_body)
    for finger_name in ("gripper", "moving_jaw_so101_v1"):
        force, outward = _pair_force(model, data, _body_id(model, finger_name), item)
        magnitude = float(np.linalg.norm(force))
        if magnitude < min_force:
            return False
        axis = float(np.linalg.norm(outward))
        if axis > 1e-9:
            cos_angle = float(np.dot(force, outward) / (magnitude * axis))
            angle = float(np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0))))
            if angle > max_angle_deg:
                return False
    return True


def is_touching_any(model, data, geom_names, min_force=0.2, bodies=ROBOT_BODIES):
    """任意目标 geom 与指定 body 集合有 >=min_force 接触力。"""
    gids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in geom_names}
    gids.discard(-1)
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = int(c.geom1)
        g2 = int(c.geom2)
        if g1 in gids or g2 in gids:
            other = g2 if g1 in gids else g1
            if model.geom_bodyid[other] in bodies:
                f = np.zeros(6)
                mujoco.mj_contactForce(model, data, i, f)
                if np.linalg.norm(f[:3]) >= min_force:
                    return True
    return False


def is_touching(model, data, geom_name, min_force=0.2, bodies=ROBOT_BODIES):
    """指定 body 集合内的机器人 geom 与目标 geom 有 >=min_force 接触力。"""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if gid < 0:
        return False
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = int(c.geom1)
        g2 = int(c.geom2)
        if gid not in (g1, g2):
            continue
        other = g2 if g1 == gid else g1
        if model.geom_bodyid[other] in bodies:
            f = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, f)
            if np.linalg.norm(f[:3]) >= min_force:
                return True
    return False


def is_static(data, qvel_addrs, threshold=0.15):
    """机器人关节速度均低于阈值 (rad/s)。"""
    v = data.qvel[qvel_addrs]
    return bool(np.max(np.abs(v)) <= threshold)


def is_body_static(model, data, body, threshold=0.15):
    """自由物体（freejoint body）速度均低于阈值 (m/s 与 rad/s)。"""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    vadr = model.jnt_dofadr[model.body_jntadr[bid]]
    v = data.qvel[vadr:vadr + 6]
    return bool(np.max(np.abs(v)) <= threshold)


def maximize_window_by_name(title_substr="MuJoCo", timeout_s=5.0):
    """EWMH 最大化标题含 title_substr 的 X11 窗口（best-effort, Wayland/无显示静默跳过）。

    mujoco viewer 无最大化 API, 经 _NET_WM_STATE ClientMessage 实现。
    """
    try:
        from Xlib import X, display
        from Xlib.protocol import event as xevent
    except Exception:
        return False

    import time

    try:
        d = display.Display()
        root = d.screen().root
        net_client_list = d.intern_atom("_NET_CLIENT_LIST")
        net_wm_name = d.intern_atom("_NET_WM_NAME")
        net_wm_state = d.intern_atom("_NET_WM_STATE")
        vert = d.intern_atom("_NET_WM_STATE_MAXIMIZED_VERT")
        horz = d.intern_atom("_NET_WM_STATE_MAXIMIZED_HORZ")

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            prop = root.get_full_property(net_client_list, X.AnyPropertyType)
            for wid in (prop.value if prop else []):
                w = d.create_resource_object("window", int(wid))
                name_prop = w.get_full_property(net_wm_name, 0)
                if name_prop is None:
                    continue
                name = name_prop.value
                if isinstance(name, bytes):
                    name = name.decode("utf-8", "ignore")
                if title_substr not in str(name):
                    continue
                ev = xevent.ClientMessage(
                    window=w, client_type=net_wm_state,
                    data=(32, [1, vert, horz, 1, 0]),
                )
                root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
                d.flush()
                return True
            time.sleep(0.2)
    except Exception:
        return False
    return False


def set_window_no_input(title_substr="MuJoCo", timeout_s=5.0):
    """WM_HINTS.input=False: 窗口永不抢键盘焦点（X11 best-effort, 鼠标事件不受影响）。

    解决 mujoco viewer 字母键渲染开关与 pynput 全局遥操键的冲突。
    """
    try:
        from Xlib import X, display
    except Exception:
        return False

    import time

    try:
        d = display.Display()
        root = d.screen().root
        net_client_list = d.intern_atom("_NET_CLIENT_LIST")
        net_wm_name = d.intern_atom("_NET_WM_NAME")

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            prop = root.get_full_property(net_client_list, X.AnyPropertyType)
            for wid in (prop.value if prop else []):
                w = d.create_resource_object("window", int(wid))
                name_prop = w.get_full_property(net_wm_name, 0)
                if name_prop is None:
                    continue
                name = name_prop.value
                if isinstance(name, bytes):
                    name = name.decode("utf-8", "ignore")
                if title_substr not in str(name):
                    continue
                hints = w.get_wm_hints() or {}
                flags = int(hints.get("flags", 0)) | X.InputHint
                w.set_wm_hints(flags=flags, input=0)
                d.flush()
                return True
            time.sleep(0.2)
    except Exception:
        return False
    return False
