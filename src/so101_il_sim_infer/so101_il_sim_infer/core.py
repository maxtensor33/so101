"""il_sim 推理核心（ROS 无关, workflow/plan/il_sim.md Phase 5）。

load_policy: checkpoint → (policy, preprocessor, postprocessor)。
make_obs_batch: env obs → policy 输入 batch（state=归一化 qpos(6,), 图像 CHW float 0-1）。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.pretrained import PreTrainedConfig

ENV_IDS = {
    "reach": "SO101ReachEnv",
    "lift": "SO101LiftEnv",
    "place": "SO101PlaceEnv",
    "stack": "SO101StackEnv",
}


def load_policy(ckpt: str | Path):
    """checkpoint 目录（.../pretrained_model）→ (policy, pre, post)。"""
    ckpt = Path(ckpt)
    config = PreTrainedConfig.from_pretrained(str(ckpt))
    policy_cls = get_policy_class(config.type)
    policy = policy_cls.from_pretrained(str(ckpt))
    pre, post = make_pre_post_processors(policy.config, pretrained_path=str(ckpt))
    # 预处理器的 to_device 步骤决定输入设备，权重必须对齐（checkpoint 保存于 cuda 时）
    device = "cpu"
    for step in pre.steps:
        dev = getattr(step, "device", None)
        if dev:
            device = str(dev)
    if device.startswith("cuda") and not torch.cuda.is_available():
        device = "cpu"  # 无 GPU 机器回退
    policy.to(device)
    policy.eval()
    return policy, pre, post


def make_obs_batch(env_obs: dict, u=None) -> dict[str, torch.Tensor]:
    """env obs（相机版）→ policy 输入 batch。

    state: qpos(6) 归一化到 [-1,1]（与数据集 observation.state 同分布）；
    images: (C,H,W) float32 /255（训练管线同款）。
    """
    state = np.asarray(env_obs["state"])
    if u is not None:
        state = u.normalize_qpos(state[:6])
    batch: dict[str, torch.Tensor] = {
        "observation.state": torch.as_tensor(state, dtype=torch.float32)
    }
    for cam, img in env_obs["rgb"].items():
        arr = np.asarray(img).transpose(2, 0, 1)
        batch[f"observation.images.{cam}"] = torch.as_tensor(arr, dtype=torch.float32) / 255.0
    return batch
