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


def _pair_force(model, data, b1_id, b2_id):
    """两 body 间接触法向力之和 (N)。参数为 body id。"""
    total = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = int(c.geom1)
        g2 = int(c.geom2)
        ids = {model.geom_bodyid[g1], model.geom_bodyid[g2]}
        if ids == {b1_id, b2_id} and b1_id != b2_id:
            f = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, f)
            total += float(np.linalg.norm(f[:3]))
    return total


def _body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def is_grasping(model, data, item_body, min_force=_MIN_FORCE):
    """双"指"对物体均有 >=min_force 接触力 -> 抓住。"""
    item = _body_id(model, item_body)
    fixed = _pair_force(model, data, _body_id(model, "gripper"), item)
    moving = _pair_force(model, data, _body_id(model, "moving_jaw_so101_v1"), item)
    return bool(fixed >= min_force and moving >= min_force)


def is_touching(model, data, geom_name, min_force=0.2):
    """任一机器人 geom 与指定 geom 有 >=min_force 接触力。"""
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
        if model.geom_bodyid[other] in ROBOT_BODIES:
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
