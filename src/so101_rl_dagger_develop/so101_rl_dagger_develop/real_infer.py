"""rl_dagger 实机部署推理核心（ROS 无关, workflow/plan/rl_dagger.md Phase 5）。

约定对齐 so101_rl_dagger_infer/core.py make_obs_batch（student 纯视觉）:
- 输入: cv2 相机帧 (H, W, 3) uint8, key = meta hparams.camera_keys（训练相机名 fixed/wrist）
- 图像: frame_to_policy_input 中心裁方 + resize 到 policy in_hw（meta hparams.in_hw）
- 动作: [-1,1] 归一化位置目标 -> ctrlrange 内 rad（base_env.denormalize_action 同公式）
"""

from __future__ import annotations

import cv2
import numpy as np

JOINT_LOWER = np.array([-1.91986, -1.91986, -1.69, -1.65806, -2.74385, -0.17453])
JOINT_UPPER = np.array([1.91986, 1.91986, 1.69, 1.65806, 2.84121, 1.74533])
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def frame_to_policy_input(frame_bgr, in_hw=(128, 128)):
    """cv2 BGR 帧 -> (H, W, 3) uint8 RGB, 尺寸对齐 policy in_hw（(H, W) 顺序）。

    预处理惯例沿 il_sim_develop 的 resize_to_policy: 先中心裁方（消除相机帧与
    policy 输入宽高比失配）, 再 resize 到 in_hw（policy 输入分辨率 =
    meta hparams.in_hw, 缺省 (128, 128) = env RENDER_SIZE）。640x480 相机帧先
    中心裁方到 480x480 再 resize 到 128x128（实机分辨率不变）。
    """
    img = cv2.cvtColor(np.asarray(frame_bgr), cv2.COLOR_BGR2RGB)
    h, w = img.shape[:2]
    crop = min(h, w)
    y0, x0 = (h - crop) // 2, (w - crop) // 2
    square = img[y0 : y0 + crop, x0 : x0 + crop]
    if square.shape[:2] != tuple(in_hw):
        square = cv2.resize(square, (int(in_hw[1]), int(in_hw[0])),
                            interpolation=cv2.INTER_AREA)
    return square


def make_env_like_obs(frames: dict) -> dict:
    """{key: (H, W, 3) uint8} -> {"rgb": frames}（make_obs_batch 消费的 env 形状 obs）。

    student 纯视觉: env_obs 只需 rgb 通道, 实机侧把相机帧组装成此结构即可复用
    so101_rl_dagger_infer.core.make_obs_batch（与训练共用预处理链, 不得另写）。
    """
    return {"rgb": frames}


def denormalize_action(action):
    """[-1,1] -> ctrlrange 内目标位置 (rad)（base_env.denormalize_action 同公式, 同 il_sim_develop）。"""
    a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
    return JOINT_LOWER + (a + 1.0) / 2.0 * (JOINT_UPPER - JOINT_LOWER)


def clamp_target(target, prev_target, max_delta=0.1):
    """单步增量 <= max_delta 且绝对范围在 ctrlrange 内（同 il_sim_develop）。"""
    t = np.asarray(target, dtype=float)
    p = np.asarray(prev_target, dtype=float)
    return np.clip(t, p - max_delta, p + max_delta).clip(JOINT_LOWER, JOINT_UPPER)


def plan_to_rest(current, rest, max_step=0.025):
    """向 rest 走一步, 每步 <= max_step rad（squint reset 同款 bounded step, 同 il_sim_develop）。"""
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
