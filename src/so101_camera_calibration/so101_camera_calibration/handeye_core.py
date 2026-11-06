"""手眼标定核心纯函数（无 ROS 依赖）。

求解: cv2.calibrateHandEye（PARK/HORAUD 双解 + 一致性判定, 对齐 so101-ros-physical-ai 验证准则）。
换算: optical-frame → camera-link（参考项目 README 公式, scipy 实现）。

矩阵方向约定: T_ab 将 b 系坐标映射到 a 系（p_a = T_ab · p_b）。
- eye_in_hand 输出 T_gc（相机在夹爪系位姿: 平移=相机原点在夹爪系, 旋转=光轴系姿态）
- eye_to_hand 输出 T_bc（相机在基座系位姿: 平移=相机原点在基座系）

注意 OpenCV 命名: cv2.calibrateHandEye 的 cam2gripper / cam2base 即上面的
T_gc / T_bc（"camera 到父系"的映射）, **不要再取逆**——取逆后平移会变成
父系原点在相机系的坐标, 正是 sim 相机跑到反方向/朝上的根因。
"""

from __future__ import annotations

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

METHODS = ("PARK", "HORAUD")  # 参考项目实测: 这两解一致即可信; Tsai 深度易错, Andreff/Daniilidis 不稳


def _split(T_list):
    R_list = [np.ascontiguousarray(T[:3, :3]) for T in T_list]
    t_list = [np.ascontiguousarray(T[:3, 3].reshape(3, 1)) for T in T_list]
    return R_list, t_list


def _invert(T):
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def solve_handeye(T_g2b_list, T_t2c_list, mode: str) -> dict:
    """手眼求解, 返回 {method: {"R": 3x3, "t": (3,), "T": 4x4}}。

    mode=eye_in_hand: 输出 T_gc（相机在夹爪系位姿）;
    mode=eye_to_hand: 输出 T_bc（相机在基座系位姿）。

    T_g2b_list: gripper_frame 在 base 的位姿 T_bg。
    T_t2c_list: solvePnP 输出, 即板在相机系的位姿 T_ct。
    """
    if len(T_g2b_list) < 3 or len(T_g2b_list) != len(T_t2c_list):
        raise ValueError("需要 >=3 组且等长的 (臂位姿, 板观测) 样本")
    if mode == "eye_in_hand":
        # OpenCV 输入 (R_gripper2base, R_target2cam), 输出 R_cam2gripper=T_gc
        R_in, t_in = _split(T_g2b_list)
        R_t, t_t = _split(T_t2c_list)
    elif mode == "eye_to_hand":
        # OpenCV 输入 (R_base2gripper, R_target2cam), 输出 R_cam2base=T_bc
        R_in, t_in = _split([_invert(T) for T in T_g2b_list])
        R_t, t_t = _split(T_t2c_list)
    else:
        raise ValueError(f"unknown mode: {mode}")

    out = {}
    for method in METHODS:
        R, t = cv2.calibrateHandEye(R_in, t_in, R_t, t_t, method=getattr(cv2, f"CALIB_HAND_EYE_{method}"))
        # cv2 返回值就是相机在父系(gripper/base)的位姿, 直接使用。
        # 历史上这里取逆过, 导致 t 变成父系原点在相机系的坐标——
        # 固定相机因此从"左前"跑到"右后", wrist 光轴也反了。
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.ravel()
        out[method] = {"R": T[:3, :3].copy(), "t": T[:3, 3].copy(), "T": T}
    return out


def solve_variants(T_g2b_list, T_t2c_list, mode: str) -> list[dict]:
    """枚举输入方向约定变体求解, 返回候选解并按物理先验排序。

    AX=XB 的输入方向有 4 种等价组合（g2b 是否取逆 × t2c 是否取逆）,
    数学上都能"解出来", 只有物理约束能区分对错:
    1. 位置先验: eye_to_hand 相机固定在基座上方 (t.z∈[0.3,1.2]);
       eye_in_hand 相机贴近夹爪 (|t|<0.3) 且反推板不能低于桌面 (z>-0.02)。
    2. **板刚性**（最强）: 板在样本间是刚体固定（eye_in_hand 在桌面、
       eye_to_hand 在夹爪上）, 正确解反推的板位姿标准差最小;
       错误变体通常达到数十厘米。
    候选: {"name", "t", "R", "T", "plausible", "board_mean", "board_std"}
    """
    inv = _invert
    variants = {
        "g2b×t2c 原始": (T_g2b_list, T_t2c_list),
        "g2b取逆": ([inv(T) for T in T_g2b_list], T_t2c_list),
        "t2c取逆": (T_g2b_list, [inv(T) for T in T_t2c_list]),
        "双取逆": ([inv(T) for T in T_g2b_list], [inv(T) for T in T_t2c_list]),
    }
    cands = []
    for name, (g, t) in variants.items():
        try:
            out = solve_handeye(g, t, mode)
        except Exception:
            continue
        for method in METHODS:
            res = out[method]
            tv = res["t"]
            det = float(np.linalg.det(res["R"]))
            if abs(det - 1.0) > 0.1:
                continue  # 非正交旋转（错误约定的 cv2 输出可能是左手系）, 直接剔除
            # 反推板位姿: T = 相机在父系位姿
            #   eye_to_hand: 板在夹爪系 = inv(T_g2b) · T_bc · T_ct
            #   eye_in_hand: 板在基座系 = T_g2b · T_gc · T_ct
            board_pts = []
            for i, t2c in enumerate(t):
                if mode == "eye_to_hand":
                    pb = inv(g[i]) @ res["T"] @ t2c
                else:
                    pb = g[i] @ res["T"] @ t2c
                board_pts.append(pb[:3, 3])
            board_pts = np.asarray(board_pts)
            board_mean = board_pts.mean(axis=0)
            board_std = float(np.linalg.norm(board_pts.std(axis=0)))

            if mode == "eye_to_hand":
                # 相机在基座上方; 板紧固在夹爪附近
                plausible = (0.3 < float(tv[2]) < 1.2
                             and float(np.linalg.norm(board_mean)) < 0.35)
            else:
                # 相机在夹爪附近; 板放在桌面（不能低于桌面）
                plausible = (float(np.linalg.norm(tv)) < 0.3
                             and -0.02 < float(board_mean[2]) < 0.8)
            cands.append({
                "name": f"{name}/{method}", "t": tv, "R": res["R"], "T": res["T"],
                "plausible": plausible, "board_mean": board_mean,
                "board_std": board_std,
            })
    # 先按物理合理性, 再按板刚性（std 越小越可信）, 最后按相机距父系原点距离
    cands.sort(key=lambda c: (not c["plausible"], c["board_std"],
                              float(np.linalg.norm(c["t"]))))
    return cands


def consistency_ok(results: dict, tol_m: float = 0.01) -> bool:
    """Park/Horaud 双解平移一致性（参考准则: <1cm 可信）。"""
    ts = [results[m]["t"] for m in METHODS]
    return float(np.linalg.norm(ts[0] - ts[1])) < tol_m


def optical_to_link(t_opt, rpy_opt_deg):
    """标定输出的 optical-frame 外参 → camera_link 的 (xyz, rpy_deg)。

    URDF 链: camera_link ──(A)──▸ optical_frame, 其中 B 固定旋转 rpy=(-π/2, 0, -π/2)、零平移;
    标定给 A×B, 还原 A = T_cal · inv(T_opt)（移植参考 README 公式; 平移不变）。
    """
    T_cal = np.eye(4)
    T_cal[:3, :3] = Rotation.from_euler("xyz", rpy_opt_deg, degrees=True).as_matrix()
    T_cal[:3, 3] = t_opt
    T_opt = np.eye(4)
    T_opt[:3, :3] = Rotation.from_euler("xyz", [-np.pi / 2, 0, -np.pi / 2]).as_matrix()
    T_link = T_cal @ np.linalg.inv(T_opt)
    rpy = Rotation.from_matrix(T_link[:3, :3]).as_euler("xyz", degrees=True)
    return T_link[:3, 3].copy(), rpy
