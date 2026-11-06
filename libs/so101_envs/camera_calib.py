"""标定结果 → envs 相机定制 (workflow/plan/setup_camera_calibration.md Phase 4)。

单一真源: 标定包 export 的汇总 YAML（fixed/wrist 各自 pos/quat_wxyz/fovy, MuJoCo 相机约定）。
覆写 model.cam_pos/cam_quat/cam_fovy（**MJCF 相机父 body 系**: fixed_cam 挂 world 即全局,
wrist_cam 挂 body "gripper"(= URDF gripper_link)。注意不是手眼的 gripper_frame_link(TCP),
二者间的固定变换由 export_envs_camera_calib.py 负责换算）。

默认开启: camera_calib=None（缺省）走 resolve_camera_calib 自动解析默认路径
DEFAULT_CAMERA_CALIB（先 CWD 相对, 再项目根相对）；文件缺失则保持未标定
（静默回退, 回归保证）。显式关闭传 camera_calib=False；显式路径缺文件即报错。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

CAMERAS = ("fixed", "wrist")

# 标定默认开启: 默认汇总 YAML 位置（相对项目根/CWD）
DEFAULT_CAMERA_CALIB = "outputs/calibration/envs_camera_calib.yaml"


def resolve_camera_calib(camera_calib) -> Path | None:
    """解析 camera_calib 参数为标定 YAML 路径。

    - None: 默认开启, 解析 DEFAULT_CAMERA_CALIB（CWD 相对优先, 再沿安装位置
      祖先目录查找命中项目根; 文件缺失返回 None, 保持未标定, 不阻断
      非实机标定场景）
    - falsy (False/""/"none"): 显式关闭, 返回 None
    - 其余: 原样返回 Path（缺文件由 load_camera_calib 报错）
    """
    if camera_calib is None:
        for c in _default_candidates():
            if c.is_file():
                return c
        return None
    if not camera_calib or str(camera_calib).lower() in ("none", "off"):
        return None
    return Path(camera_calib)


def _default_candidates() -> list[Path]:
    """默认标定路径候选: CWD 相对优先, 再沿 __file__ 祖先目录查找
    （wheel 安装于 site-packages 时逐级向上可命中项目根/, 兜底任意 CWD）。"""
    cands = [Path.cwd() / DEFAULT_CAMERA_CALIB]
    cands += [anc / DEFAULT_CAMERA_CALIB for anc in Path(__file__).resolve().parents]
    out, seen = [], set()
    for c in cands:
        if c not in seen:
            seen.add(c)
            out.append(c)
    return out


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
