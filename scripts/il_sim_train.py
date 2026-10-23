#!/usr/bin/env python
"""il_sim 训练 CLI（workflow/plan/il_sim.md Phase 4）。

冒烟/联调：
    uv run python scripts/il_sim_train.py --policy act --repo-id il_sim/reach_expert --steps 10

全量训练（迁移到大显存机器时去掉小网络模板，见 docs/il_sim/index.md）：
    用官方 lerobot-train 默认超参，数据集与本脚本一致。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "so101_teleop"))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", choices=["act", "diffusion", "vqbet"], required=True)
    ap.add_argument("--repo-id", required=True, help="数据集 repo_id（outputs/datasets/il_sim/<repo_id>）")
    ap.add_argument("--steps", type=int, default=10)
    ap.add_argument("--batch-size", type=int, default=4)
    args, _ = ap.parse_known_args()

    import so101_teleop  # noqa: F401  注册（保持与 record 侧一致的注册上下文）
    from so101_teleop.train_config import build_train_config

    cfg = build_train_config(
        policy=args.policy, repo_id=args.repo_id, steps=args.steps, batch_size=args.batch_size
    )
    from lerobot.scripts.lerobot_train import train

    train(cfg)


if __name__ == "__main__":
    main()
