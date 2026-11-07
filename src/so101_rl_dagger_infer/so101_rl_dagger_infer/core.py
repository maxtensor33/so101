"""DAgger student 推理核心（ROS 无关, workflow/plan/rl_dagger.md Phase 4）。

load_student: student checkpoint 目录（支持 <tag>/latest 指针）→ (policy, meta)。
make_obs_batch: env 单步 Dict obs → student action —— 预处理链
（stack_camera_images + images_to_tensor）与训练完全共用, 推理侧不得另写。
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import numpy as np
import torch

from so101_rl.backends.base import read_meta
from so101_rl.dagger import (
    StudentPolicy,
    images_to_tensor,
    load_student_policy,
    stack_camera_images,
)
from so101_rl.runs import resolve_ckpt_dir

DEFAULT_CAMERA_KEYS = ["fixed"]


def load_student(ckpt_dir: str | Path) -> tuple[StudentPolicy, dict]:
    """student checkpoint 目录（backend="dagger"）→ (policy, meta)。

    meta 供调用方读 env_id / env_kwargs（重建同配置 env, 原样透传）与
    hparams.camera_keys（观测组装相机顺序）。
    """
    ckpt = resolve_ckpt_dir(Path(ckpt_dir).expanduser())
    meta = read_meta(ckpt)
    policy = load_student_policy(ckpt)
    return policy, meta


def camera_keys_from_meta(meta: dict) -> list[str]:
    """meta hparams 的相机顺序（训练/推理必须一致）。"""
    return list(meta.get("hparams", {}).get("camera_keys", DEFAULT_CAMERA_KEYS))


def make_obs_batch(env_obs: dict, camera_keys: Sequence[str], policy: StudentPolicy,
                   device: str = "cpu") -> np.ndarray:
    """env 单步 Dict obs（相机版）→ student action (6,) ∈ [-1,1] np.ndarray。

    env_obs["rgb"] 各键 (H, W, 3) uint8 → 加批量维 → stack_camera_images
    （camera_keys 顺序 = 训练顺序, 存于 meta hparams）→ images_to_tensor
    （/255 + CHW）→ policy forward（no_grad, 确定性 tanh 输出）。
    """
    rgb = env_obs["rgb"]
    stacked = stack_camera_images(
        {k: np.asarray(rgb[k])[None] for k in camera_keys}, camera_keys)
    images = images_to_tensor(stacked, device)
    with torch.no_grad():
        action = policy(images)
    return action[0].detach().cpu().numpy()
