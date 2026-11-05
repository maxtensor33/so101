"""il_sim 实机部署推理核心（ROS 无关, workflow/plan/il_sim_develop.md Phase 2）。

约定对齐 so101_il_sim_infer/core.py make_obs_batch 与 so101_envs/base_env:
- state: qpos(6) 按 ctrlrange 线性归一化到 [-1,1]
- 图像: (C,H,W) float32 /255, key = 训练数据集相机名（fixed/wrist）
- 动作: [-1,1] 归一化位置目标 -> ctrlrange 内 rad
"""

from __future__ import annotations

import cv2
import numpy as np

JOINT_LOWER = np.array([-1.91986, -1.91986, -1.69, -1.65806, -2.74385, -0.17453])
JOINT_UPPER = np.array([1.91986, 1.91986, 1.69, 1.65806, 2.84121, 1.74533])
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def normalize_qpos(qpos):
    """qpos (rad) -> [-1,1]（base_env.normalize_qpos 同公式）。"""
    return 2.0 * (np.asarray(qpos, dtype=float) - JOINT_LOWER) / (JOINT_UPPER - JOINT_LOWER) - 1.0


def denormalize_action(action):
    """[-1,1] -> ctrlrange 内目标位置 (rad)（base_env.denormalize_action 同公式）。"""
    a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
    return JOINT_LOWER + (a + 1.0) / 2.0 * (JOINT_UPPER - JOINT_LOWER)


def build_obs_batch(qpos, images, camera_keys):
    """qpos(6) + {key: HxWx3 uint8} -> policy 输入 batch（make_obs_batch 同约定）。

    images 须已 resize 到 128×128; camera_keys 缺帧时 KeyError 由上层处理。
    """
    batch = {"observation.state": torch_from(normalize_qpos(qpos))}
    for key in camera_keys:
        arr = np.asarray(images[key]).transpose(2, 0, 1).astype(np.float32) / 255.0
        batch[f"observation.images.{key}"] = torch_from(arr)
    return batch


def torch_from(arr):
    import torch

    return torch.as_tensor(np.ascontiguousarray(arr), dtype=torch.float32)


def resize_to_policy(img, size=128):
    """相机帧 -> 中心裁方 -> 缩放 (size,size,3) uint8（squint 预处理同款）。"""
    h, w = img.shape[:2]
    crop = min(h, w)
    y0, x0 = (h - crop) // 2, (w - crop) // 2
    square = img[y0 : y0 + crop, x0 : x0 + crop]
    if square.shape[0] != size:
        square = cv2.resize(square, (size, size), interpolation=cv2.INTER_AREA)
    return square


def clamp_target(target, prev_target, max_delta=0.1):
    """单步增量 <= max_delta 且绝对范围在 ctrlrange 内。"""
    t = np.asarray(target, dtype=float)
    p = np.asarray(prev_target, dtype=float)
    return np.clip(t, p - max_delta, p + max_delta).clip(JOINT_LOWER, JOINT_UPPER)


def plan_to_rest(current, rest, max_step=0.025):
    """向 rest 走一步, 每步 <= max_step rad（squint reset 同款 bounded step）。"""
    cur = np.asarray(current, dtype=float)
    delta = np.asarray(rest, dtype=float) - cur
    return cur + np.clip(delta, -max_step, max_step)


class EstopLatch:
    """急停锁存: trigger 后保持, clear 显式复位。"""

    def __init__(self):
        self._triggered = False

    @property
    def triggered(self):
        return self._triggered

    def trigger(self):
        self._triggered = True

    def clear(self):
        self._triggered = False
