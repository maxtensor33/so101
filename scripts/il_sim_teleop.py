#!/usr/bin/env python
"""il_sim 遥操主进程（plan/il_sim.md Phase 3）。

- --teleop expert|keyboard  遥操类型
- --record                  默认 false（纯遥操不落盘）；true 走官方 record 管线
- --env reach               任务环境（reach/lift/place/stack）
- --repo-id, --episodes, --episode-time-s  record 模式参数

两种模式均内置启动 ros_viz 线程（RViz 显示 /joint_states、/il/* 图像、物体 TF）。
RViz/robot_state_publisher 由 launch/teleop.launch.py 拉起。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "so101_teleop"))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--teleop", choices=["expert", "keyboard"], default="expert")
    ap.add_argument("--record", action="store_true", help="录制数据（默认 false 不落盘）")
    ap.add_argument("--env", default="reach", help="reach|lift|place|stack")
    ap.add_argument("--repo-id", default="il_sim/reach_expert")
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--episode-time-s", type=float, default=8.0)
    ap.add_argument("--reset-time-s", type=float, default=None,
                    help="episode 间 reset 期秒数（默认 expert=0 自动重置 / keyboard=5 按 c 复位场景）")
    ap.add_argument("--video", action="store_true", default=True)
    ap.add_argument("--no-video", dest="video", action="store_false")
    ap.add_argument("--mujoco", action="store_true", default=True,
                    help="MuJoCo viewer 窗口（默认开; 无显示环境用 --no-mujoco）")
    ap.add_argument("--no-mujoco", dest="mujoco", action="store_false")
    ap.add_argument("--resume", action="store_true",
                    help="续录已存在数据集（lerobot 语义; 与 --repo-id 同用）")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--task", default="reach the goal above the item")
    args, _ = ap.parse_known_args()  # 忽略 launch 注入的 --ros-args

    import so101_teleop  # noqa: F401  draccus 注册
    from so101_teleop.cli_config import build_record_config, build_teleop_mode_config

    # ros_viz 内置启动（无 ROS 环境时静默跳过；等 robot connect 后自动挂上）
    from so101_teleop.ros_viz import RosVizThread

    viz = RosVizThread(rate=10.0)
    viz.start()

    if args.record:
        cfg = build_record_config(
            teleop=args.teleop,
            env=args.env,
            repo_id=args.repo_id,
            episodes=args.episodes,
            episode_time_s=args.episode_time_s,
            video=args.video,
            task=args.task,
            seed=args.seed,
            render=args.mujoco,
            reset_time_s=args.reset_time_s,
            resume=args.resume,
        )
        from lerobot.scripts.lerobot_record import record

        record(cfg)
    else:
        cfg = build_teleop_mode_config(teleop=args.teleop, env=args.env, render=args.mujoco)
        cfg.teleop_time_s = None  # 一直遥操直到 Ctrl+C
        from lerobot.scripts.lerobot_teleoperate import teleoperate

        teleoperate(cfg)

    viz.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
