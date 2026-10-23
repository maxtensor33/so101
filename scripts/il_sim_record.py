#!/usr/bin/env python
"""il_sim 采集 CLI（workflow/plan/il_sim.md Phase 2）：官方 lerobot-record 薄封装。

import so101_teleop 触发 draccus 注册后透传官方 CLI，用法与 lerobot-record 完全一致：
    uv run python scripts/il_sim_record.py \
        --robot.type=so101_sim --robot.env_id=SO101ReachCamera-v1 \
        --teleop.type=so101_expert \
        --dataset.repo_id=il_sim/reach_expert \
        --dataset.root=outputs/datasets/il_sim/reach_expert \
        --dataset.fps=10 --dataset.episode_time_s=8 --dataset.reset_time_s=0 \
        --dataset.num_episodes=10 --dataset.video=true --dataset.push_to_hub=false \
        --dataset.single_task="reach the goal above the item"

Phase 3 起 record 期间 ros_viz 线程同步发布 /joint_states 与相机图像（RViz 显示）。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "so101_teleop"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "so101_il_sim_infer"))


def ensure_render_default(argv: list[str]) -> list[str]:
    """未显式给 --robot.render 时默认注入 true（MuJoCo 窗口; 显式 false 尊重用户）。"""
    if any(a.startswith("--robot.render") for a in argv):
        return argv
    return [*argv, "--robot.render=true"]


def main() -> None:
    import so101_teleop  # noqa: F401  触发 draccus 注册（so101_sim / so101_expert / so101_keyboard）

    from lerobot.scripts.lerobot_record import main as lerobot_record_main

    args = ensure_render_default(sys.argv[1:])
    sys.argv = ["il_sim_record", *args]
    lerobot_record_main()


if __name__ == "__main__":
    main()
