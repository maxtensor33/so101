# Copyright 2026 Dmitri Manajev
# Copyright 2026 maxtensor (adaptation: 从 camera_intrinsic_calibration_node 提取纯函数,
#   cv_bridge → 手动 rgb8 解码, numpy2 兼容)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
"""内参标定纯函数（无 ROS/viser 依赖, pytest 可直接测）。

提取自 so101-ros-physical-ai camera_intrinsic_calibration_node。
"""

from __future__ import annotations

import math

import cv2
import cv2.aruco as aruco
import numpy as np

_CV_NEW_API = hasattr(aruco, "CharucoBoard")


def resolve_aruco_dict(name: str):
    """按名查 OpenCV 预定义 ArUco 字典（"DICT_5X5_250" 或 "5X5_250" 均可）。"""
    attr = name if name.startswith("DICT_") else f"DICT_{name}"
    if not hasattr(aruco, attr):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return aruco.getPredefinedDictionary(getattr(aruco, attr))


def decode_image(encoding: str, data: bytes, height: int, width: int) -> np.ndarray:
    """sensor_msgs/Image → BGR ndarray（手动解码, 替代 cv_bridge 以兼容 numpy2 栈）。"""
    dt = np.frombuffer(bytes(data), dtype=np.uint8)
    if encoding in ("rgb8", "RGB8"):
        rgb = dt.reshape(height, width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if encoding in ("bgr8", "BGR8"):
        return dt.reshape(height, width, 3).copy()
    if encoding in ("mono8", "MONO8"):
        return cv2.cvtColor(dt.reshape(height, width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported encoding: {encoding}")


def largest_rectangle_corners(char_corners, char_ids, xdim, ydim):
    """内角点网格上最大全可见轴对齐矩形（参考实现原样）。"""
    ids_flat = char_ids.ravel().astype(int)
    id_to_pixel = {int(i): char_corners[k, 0] for k, i in enumerate(ids_flat)}
    visible = {int(i) for i in ids_flat}

    best_area = 0
    best = None
    for y1 in range(ydim):
        for y2 in range(y1, ydim):
            for x1 in range(xdim):
                for x2 in range(x1, xdim):
                    area = (x2 - x1 + 1) * (y2 - x1 + 1) if False else (x2 - x1 + 1) * (y2 - y1 + 1)
                    if area <= best_area:
                        continue
                    if (y1 * xdim + x1 in visible and y1 * xdim + x2 in visible
                            and y2 * xdim + x1 in visible and y2 * xdim + x2 in visible):
                        best_area = area
                        best = (x1, x2, y1, y2)
    if best is None or best_area < 4:
        return None
    x1, x2, y1, y2 = best
    return (id_to_pixel[y1 * xdim + x1], id_to_pixel[y1 * xdim + x2],
            id_to_pixel[y2 * xdim + x2], id_to_pixel[y2 * xdim + x1])


def quad_area(tl, tr, br, bl) -> float:
    """鞋带公式面积 (px^2)。"""
    p = np.array([tl, tr, br, bl], dtype=np.float64)
    x, y = p[:, 0], p[:, 1]
    return 0.5 * abs(
        x[0] * y[1] - x[1] * y[0] + x[1] * y[2] - x[2] * y[1]
        + x[2] * y[3] - x[3] * y[2] + x[3] * y[0] - x[0] * y[3]
    )


def quad_skew(tl, tr, br) -> float:
    """板倾斜度: tr 角偏离 90° 的程度, [0,1]。"""
    def angle(a, b, c):
        ab, cb = a - b, c - b
        cos = np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb) + 1e-12)
        return math.acos(max(-1.0, min(1.0, cos)))
    return min(1.0, 2.0 * abs((math.pi / 2.0) - angle(tl, tr, br)))


def board_params_charuco(char_corners, char_ids, image_size, xdim, ydim):
    """ROS 风格 [X, Y, Size, Skew] ∈ [0,1]，部分可见视图用最大可见子矩形。"""
    rect = largest_rectangle_corners(char_corners, char_ids, xdim, ydim)
    if rect is None:
        return None
    tl, tr, br, bl = rect
    width, height = image_size
    area = quad_area(tl, tr, br, bl)
    border = math.sqrt(area)
    cx = float(np.mean([tl[0], tr[0], br[0], bl[0]]))
    cy = float(np.mean([tl[1], tr[1], br[1], bl[1]]))
    x = (cx - border / 2) / max(width - border, 1.0)
    y = (cy - border / 2) / max(height - border, 1.0)
    size = math.sqrt(area / (width * height))
    return [float(np.clip(v, 0.0, 1.0)) for v in (x, y, size)] + [float(quad_skew(tl, tr, br))]


def param_distance(a, b) -> float:
    """参数向量 L1 距离。"""
    return sum(abs(x - y) for x, y in zip(a, b))


def calibrate_charuco(all_corners, all_ids, board, image_size):
    """ChArUco 标定（aruco.calibrateCameraCharuco）+ 每视图重投影误差。

    返回 (err, K, D, view_errors)。
    """
    board_obj = (board.getChessboardCorners() if _CV_NEW_API
                 else np.asarray(board.chessboardCorners))
    if hasattr(aruco, "calibrateCameraCharuco"):
        # OpenCV <=4.9 legacy API
        err, K, D, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=all_corners, charucoIds=all_ids, board=board,
            imageSize=image_size, cameraMatrix=None, distCoeffs=None, flags=0,
        )
    else:
        # OpenCV 4.10+: calibrateCameraCharuco 已移除, 用通用 calibrateCamera
        obj_list = [np.ascontiguousarray(board_obj[ids.ravel()], dtype=np.float32).reshape(-1, 1, 3)
                    for ids in all_ids]
        cor_list = [np.ascontiguousarray(c, dtype=np.float32) for c in all_corners]
        err, K, D, rvecs, tvecs = cv2.calibrateCamera(
            obj_list, cor_list, image_size, None, None, flags=0,
        )
    view_errors = []
    for corners, ids, rvec, tvec in zip(all_corners, all_ids, rvecs, tvecs):
        obj = board_obj[ids.ravel()]
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        diff = corners.reshape(-1, 2) - projected.reshape(-1, 2)
        view_errors.append(float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))))
    return float(err), K, D, view_errors
