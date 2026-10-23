"""MuJoCo viewer 交互窗口查看 env（workflow/plan/envs.md 日常工具, 需图形环境）。

复用 env 的 gymnasium 标准 human 渲染（render_mode="human"）,
脚本只附加视角选择与实时节流。

用法:
    uv run python scripts/envs_view_env.py --id SO101Reach-v1
    uv run python scripts/envs_view_env.py --id SO101Lift-v1 --cam wrist
    uv run python scripts/envs_view_env.py --id SO101Place-v1 --random
"""

from __future__ import annotations

import argparse
import time

import mujoco
import numpy as np


def _apply_cam(v, cam_id):
    """viewer 相机切到指定 mjCAMERA_FIXED id; None = 自由相机。"""
    if v is None:
        return
    if cam_id is None:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    else:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        v.cam.fixedcamid = cam_id


def main():
    parser = argparse.ArgumentParser(description="MuJoCo viewer 查看 env")
    parser.add_argument("--id", default="SO101Reach-v1", help="gymnasium id（state 或 Camera 版均可）")
    parser.add_argument("--cam", default="free", choices=["free", "fixed", "wrist"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--random", action="store_true", help="随机动作（默认零动作保持 rest）")
    parser.add_argument("--hz", type=float, default=10.0, help="实时回放节流频率")
    args = parser.parse_args()

    import gymnasium as gym
    import so101_envs  # noqa: F401  注册副作用

    env = gym.make(args.id, render_mode="human")  # human: env 每 step 自动 sync 窗口
    u = env.unwrapped
    env.reset(seed=args.seed)

    cam_id = None
    if args.cam != "free":
        cam_id = mujoco.mj_name2id(u.model, mujoco.mjtObj.mjOBJ_CAMERA,
                                   "fixed_cam" if args.cam == "fixed" else "wrist_cam")

    print(f"viewing {args.id} (cam={args.cam}, {args.hz:.0f}Hz 实时回放)")
    dt = 1.0 / args.hz
    try:
        while True:
            t0 = time.perf_counter()
            action = env.action_space.sample() if args.random else np.zeros(6)
            env.step(action)
            _apply_cam(u._viewer, cam_id)
            wait = dt - (time.perf_counter() - t0)
            if wait > 0:
                time.sleep(wait)
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
