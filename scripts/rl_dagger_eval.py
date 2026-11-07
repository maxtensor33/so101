"""DAgger student 评估 CLI（workflow/plan/rl_dagger.md Phase 3; 镜像 scripts/rl_eval.py）。

用法:
    uv run python scripts/rl_dagger_eval.py --ckpt outputs/rl/dagger-reach/latest --episodes 10

student checkpoint（backend="dagger"）的 meta.json 随存 env_id/env_kwargs（含
camera/render_size/camera_calib）与 hparams.camera_keys, 重建单 env 后确定性推理。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gymnasium as gym
import numpy as np
import torch

import so101_envs  # noqa: F401  注册副作用
from so101_rl.backends.base import read_meta
from so101_rl.dagger import images_to_tensor, load_student_policy, stack_camera_images

DEFAULT_EPISODES = 10


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="SO101 DAgger student 评估")
    parser.add_argument("--ckpt", required=True,
                        help="student checkpoint 目录（也接受 <tag>/latest 指针文件）")
    parser.add_argument("--episodes", type=int, default=DEFAULT_EPISODES)
    args, _ = parser.parse_known_args(argv)  # parse_known_args 忽略 --ros-args

    from so101_rl.runs import resolve_ckpt_dir

    ckpt = resolve_ckpt_dir(Path(args.ckpt).expanduser())
    meta = read_meta(ckpt)
    policy = load_student_policy(ckpt)
    camera_keys = list(meta.get("hparams", {}).get("camera_keys", ["fixed"]))
    env = gym.make(meta["env_id"], **meta.get("env_kwargs", {}))

    n_ok = 0
    total_return = 0.0
    total_steps = 0
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=5000 + ep)
        ep_return = 0.0
        max_steps = env.spec.max_episode_steps or 50
        steps = 0
        for steps in range(1, max_steps + 1):
            # 单 env obs["rgb"][k] 为 (H, W, 3) -> 加批量维 (1, H, W, 3)
            stacked = stack_camera_images(
                {k: np.asarray(obs["rgb"][k])[None] for k in camera_keys}, camera_keys)
            imgs = images_to_tensor(stacked)
            with torch.no_grad():
                action = policy(imgs)[0].cpu().numpy()
            obs, reward, term, trunc, info = env.step(np.clip(action, -1.0, 1.0))
            ep_return += float(reward)
            if term or trunc:
                break
        ok = bool(info.get("success", False))
        n_ok += int(ok)
        total_return += ep_return
        total_steps += steps
        print(f"ep {ep}: success={ok} return={ep_return:.2f} steps={steps}")
    print(f"success rate: {n_ok}/{args.episodes} = {n_ok / args.episodes:.0%} "
          f"avg_return={total_return / args.episodes:.2f} "
          f"avg_steps={total_steps / args.episodes:.1f}")
    env.close()
    return 0


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    # 绕过解释器退出时的 CUDA/EGL 驱动清理（同 scripts/rl_train.py 惯例）。
    import os

    os._exit(0)
