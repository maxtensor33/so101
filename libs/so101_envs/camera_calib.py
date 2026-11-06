"""标定结果 → envs 相机定制 (workflow/plan/setup_camera_calibration.md Phase 4)。

单一真源: 标定包 export 的汇总 YAML（fixed/wrist 各自 pos/quat_wxyz/fovy, MuJoCo 相机约定）。
覆写 model.cam_pos/cam_quat/cam_fovy（**MJCF 相机父 body 系**: fixed_cam 挂 world 即全局,
wrist_cam 挂 body "gripper"(= URDF gripper_link)。注意不是手眼的 gripper_frame_link(TCP),
二者间的固定变换由 export_envs_camera_calib.py 负责换算）。
None/缺省 = 现行为完全不变（回归保证）。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

CAMERAS = ("fixed", "wrist")


def load_camera_calib(path: str | Path) -> dict:
    """读汇总 YAML 并校验结构。"""
    data = yaml.safe_load(Path(path).read_text())
    for cam in CAMERAS:
        if cam not in data:
            raise KeyError(f"camera_calib 缺 {cam} 段: {path}")
        for key in ("pos", "quat_wxyz", "fovy"):
            if key not in data[cam]:
                raise KeyError(f"camera_calib.{cam} 缺 {key}")
    return data


def fovy_from_fy(fy: float, height_px: int) -> float:
    """内参 fy → MuJoCo 垂直 fovy（度）。"""
    return float(np.degrees(2.0 * np.arctan(height_px / (2.0 * fy))))


def apply_camera_calib(model, calib: dict | str | Path) -> None:
    """覆写 model 相机字段（幂等; 须在 DR 基位缓存前调用）。"""
    import mujoco

    if isinstance(calib, (str, Path)):
        calib = load_camera_calib(calib)
    for cam in CAMERAS:
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, f"{cam}_cam")
        if cid < 0:
            raise ValueError(f"model 无相机 {cam}_cam")
        spec = calib[cam]
        model.cam_pos[cid] = np.asarray(spec["pos"], dtype=np.float64)
        model.cam_quat[cid] = np.asarray(spec["quat_wxyz"], dtype=np.float64)
        model.cam_fovy[cid] = float(spec["fovy"])
