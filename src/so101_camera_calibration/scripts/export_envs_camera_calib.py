#!/usr/bin/env python3
# Copyright 2026 maxtensor (本项目新增: 标定结果 → envs 相机定制汇总 YAML)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
"""标定结果 → envs 汇总 YAML（训练侧相机定制, Phase 4 桥接）。

输入: 手眼结果 YAML（eye_in_hand=wrist / eye_to_hand=fixed）+ 内参 camera_info YAML。
输出: envs_camera_calib.yaml（fixed/wrist 各自 pos/quat_wxyz/fovy, MuJoCo 相机约定）。

坐标系:
- handeye 的 optical_frame 是相机在标定父系中的位姿（T_gc / T_bc，不再取逆）;
- fixed_cam 在 MJCF 中挂 world → pos 直接用;
- wrist_cam 在 MJCF 中挂 body "gripper"（= URDF gripper_link）, 而手眼父系是
  gripper_frame_link → 需左乘固定关节变换 T_gripper_link←gripper_frame_link。

近似说明: MuJoCo 不支持畸变/主点偏移, 仅对齐 fovy + 位姿（残差见 docs）。
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_camera_calibration.board_gen import BOARD_CONFIGS  # noqa: E402  (占位避免误删)

CONV = np.diag([1.0, -1.0, -1.0])  # optical(z前,y下) → MuJoCo 相机(-z前,y上)

# URDF gripper_frame_joint (so101.urdf: xyz=-0.0079 -0.000218121 -0.0981274, rpy=0 π 0):
# T_{gripper_link ← gripper_frame_link}; MJCF wrist_cam 挂在 body "gripper"(= gripper_link),
# 手眼结果却相对 gripper_frame_link (TCP), 二者不可混用。
T_GRIPPER_LINK_FROM_FRAME = np.eye(4)
T_GRIPPER_LINK_FROM_FRAME[:3, :3] = Rotation.from_euler("xyz", [0.0, np.pi, 0.0]).as_matrix()
T_GRIPPER_LINK_FROM_FRAME[:3, 3] = [-0.0079, -0.000218121, -0.0981274]


def export_entry(handeye_yaml: dict, intrinsic_yaml: dict, height: int = 128,
                 rot_z180: bool = False) -> dict:
    handeye = yaml.safe_load(Path(handeye_yaml).read_text())
    info = yaml.safe_load(Path(intrinsic_yaml).read_text())
    K = np.array(info["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    # fy 与 H 必须同分辨率配对: 用标定时的图像高度（640x480 内参 → fovy≈57°）,
    # fovy 是物理视场, 与 envs 渲染分辨率无关
    calib_h = int(info.get("image_height") or info.get("height") or height)
    fovy = float(np.degrees(2.0 * np.arctan(calib_h / (2.0 * K[1, 1]))))

    CAM_MAP = {"eye_in_hand": "wrist", "eye_to_hand": "fixed"}
    cam = CAM_MAP[handeye["camera"]]
    t = np.array(handeye["optical_frame"]["t"], dtype=np.float64)
    rpy = np.array(handeye["optical_frame"]["rpy_deg"], dtype=np.float64)
    R_opt = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
    if rot_z180:
        # 安装朝向 180° 修正: 绕光轴(optical z)转半圈
        R_opt = R_opt @ np.diag([-1.0, -1.0, 1.0])

    if cam == "wrist":
        # camera 位姿从 gripper_frame_link 换到 wrist_cam 的父 body (gripper_link)
        T_frame_cam = np.eye(4)
        T_frame_cam[:3, :3] = R_opt
        T_frame_cam[:3, 3] = t
        T_link_cam = T_GRIPPER_LINK_FROM_FRAME @ T_frame_cam
        R_opt = T_link_cam[:3, :3]
        t = T_link_cam[:3, 3]

    # MuJoCo 相机: x_mj=x_opt, y_mj=-y_opt, z_mj=-z_opt
    # → R_mj = R_opt @ CONV（矩阵右乘! CONV @ R_opt 会得到错的朝向）
    R_mj = R_opt @ CONV
    quat = Rotation.from_matrix(R_mj).as_quat()[[3, 0, 1, 2]]  # xyzw → wxyz
    return {cam: {"pos": t.tolist(), "quat_wxyz": quat.tolist(), "fovy": fovy}}


def merge_outputs(out_dir: str, files: list[str]):
    """合并多个单相机结果为一份汇总 YAML。"""
    merged: dict = {}
    for f in files:
        merged.update(yaml.safe_load(Path(f).read_text()))
    out = Path(out_dir) / "envs_camera_calib.yaml"
    out.write_text(yaml.safe_dump(merged, sort_keys=False))
    print(f"saved {out}")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--handeye", action="append", required=True,
                    help="handeye_<camera>.yaml（可多次）")
    ap.add_argument("--intrinsic", action="append", required=True,
                    help="内参 camera_info YAML（可多次, 顺序对应 handeye）")
    ap.add_argument("--height", type=int, default=128)
    ap.add_argument("--out-dir", default="outputs/calibration")
    ap.add_argument("--rot-z180-fixed", action="store_true",
                    help="fixed 相机安装朝向 180° 修正（绕光轴）")
    args = ap.parse_args()
    assert len(args.handeye) == len(args.intrinsic)
    files = []
    for hy, iy in zip(args.handeye, args.intrinsic):
        entry = export_entry(hy, iy, args.height,
                             rot_z180=(args.rot_z180_fixed and
                                       yaml.safe_load(Path(hy).read_text())["camera"] == "eye_to_hand"))
        out = Path(args.out_dir) / f"envs_calib_{list(entry)[0]}.yaml"
        out.write_text(yaml.safe_dump(entry, sort_keys=False))
        files.append(str(out))
    merge_outputs(args.out_dir, files)
