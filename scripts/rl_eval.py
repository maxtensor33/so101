"""RL checkpoint 评估 CLI（workflow/plan/rl.md）。

用法:
    uv run python scripts/rl_eval.py --ckpt outputs/rl/sb3-sac-reach --episodes 10
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from so101_rl.algos import get_trainer
from so101_rl.backends.base import read_meta


def main():
    parser = argparse.ArgumentParser(description="SO101 RL checkpoint 评估")
    parser.add_argument("--ckpt", required=True,
                        help="checkpoint 目录（也接受 <tag>/latest 指针文件）")
    parser.add_argument("--episodes", type=int, default=10)
    args = parser.parse_args()

    from so101_rl.runs import resolve_ckpt_dir

    ckpt = resolve_ckpt_dir(args.ckpt)
    meta = read_meta(ckpt)
    trainer = get_trainer(meta["backend"], meta["algo"])
    policy = trainer.load_policy(ckpt)
    env = trainer.make_env(meta["env_id"], seed=None,
                           env_kwargs=meta.get("env_kwargs", {}))

    n_ok = 0
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=5000 + ep)
        for _ in range(50):
            obs, _, term, trunc, info = env.step(policy(obs))
            if term or trunc:
                break
        n_ok += int(info.get("success", False))
        print(f"ep {ep}: success={info.get('success', False)}")
    print(f"success rate: {n_ok}/{args.episodes} = {n_ok / args.episodes:.0%}")
    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
