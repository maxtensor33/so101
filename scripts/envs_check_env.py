"""单环境快速验证（plan/envs.md 日常工具）。

用法:
    uv run python scripts/envs_check_env.py --id SO101Lift-v1
    uv run python scripts/envs_check_env.py --id SO101StackCamera-v1 --steps 30 --render
"""

from __future__ import annotations

import argparse


def main():
    parser = argparse.ArgumentParser(description="单环境快速验证")
    parser.add_argument("--id", default="SO101Reach-v1", help="gymnasium id")
    parser.add_argument("--steps", type=int, default=20, help="随机策略步数")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--render", action="store_true", help="相机版存 /tmp/envs_{fixed,wrist}.png")
    args = parser.parse_args()

    import gymnasium as gym
    import so101_envs  # noqa: F401  注册副作用

    env = gym.make(args.id)
    obs, _ = env.reset(seed=args.seed)
    print(f"id: {args.id}")
    print(f"obs space: {env.observation_space}")
    r = term = None
    info = {}
    for _ in range(args.steps):
        obs, r, term, trunc, info = env.step(env.action_space.sample())
    print(f"steps: {args.steps}, reward(last)={r:.3f}, terminated={term}")
    print(f"info: {info}")
    if args.render and isinstance(obs, dict):
        from PIL import Image

        Image.fromarray(obs["rgb"]["fixed"]).save("/tmp/envs_fixed.png")
        Image.fromarray(obs["rgb"]["wrist"]).save("/tmp/envs_wrist.png")
        print("saved: /tmp/envs_fixed.png, /tmp/envs_wrist.png")
    env.close()


if __name__ == "__main__":
    main()
