#!/usr/bin/env python
"""用训练好的 RL checkpoint 作专家，自动采集 il_sim 数据集（供 ACT/DP/VQ-BeT 训练）。

思路：RL policy（outputs/rl/<tag>）闭环驱动 SO101*Camera-v1 env，逐 episode 录制
LeRobotDataset v3（observation.state=归一化 qpos(6)、action=绝对归一化关节目标(6)、
双相机图像），可直接交给 scripts/il_sim_train.py。

用法:
    uv run python scripts/il_sim_collect_rl.py \
        --checkpoint outputs/rl/lift-squint \
        --repo-id il_sim/lift_rl_expert --episodes 50

约定:
- checkpoint meta.json 的 env_kwargs.domain_randomization 默认沿用（--no-dr 关闭）；
  动作恒为 position 语义（delta 策略在 RLExpertTeleop 内积分），与 il_sim 推理端一致。
- 每 episode 固定 episode-steps（默认 env TimeLimit=50）后手工 reset；默认只保存
  成功 episode（--keep-failures 保存全部），失败 episode 丢弃并重试。
- 数据落盘 outputs/datasets/il_sim/<repo_id>（不追加时间戳，训练侧 repo_id 可直接复用）。
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "src" / "so101_teleop"))

DEFAULT_TASKS = {
    "reach": "reach the goal above the item",
    "lift": "grasp the cube and lift it above the table",
    "place": "place the cube into the bin",
    "stack": "stack the cube on top of the box",
    # can 变体（reach/lift/place 换任务物体; stack 只换底座 itemB）
    "reachcan": "reach the goal above the can",
    "liftcan": "grasp the can and lift it above the table",
    "placecan": "place the can into the bin",
    "stackcan": "stack the cube on top of the can",
}


def derive_camera_env_id(env_id: str) -> str:
    """训练 env id -> 相机版 id（SO101Lift-v1 -> SO101LiftCamera-v1）。"""
    return env_id if "Camera" in env_id else env_id.replace("-v1", "Camera-v1")


def short_env_name(env_id: str) -> str:
    """SO101LiftCanCamera-v1 -> liftcan（repo_id/task 默认值用）。"""
    return env_id.replace("Camera", "").removeprefix("SO101").removesuffix("-v1").lower()


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", required=True,
                    help="RL checkpoint 目录（或 outputs/rl/<tag>/latest 指针）")
    ap.add_argument("--env", default="",
                    help="覆盖采集 env（短名 lift 或完整注册名; 默认由 checkpoint meta 推导相机版）")
    ap.add_argument("--repo-id", default="",
                    help="数据集 repo_id（默认 il_sim/<env>_rl_expert）")
    ap.add_argument("--root", default="",
                    help="数据集根目录（默认 outputs/datasets/il_sim/<repo_id>）")
    ap.add_argument("--episodes", type=int, default=50, help="要保存的成功 episode 数")
    ap.add_argument("--episode-steps", type=int, default=0,
                    help="每 episode 控制步数（默认 env max_episode_steps=50）")
    ap.add_argument("--fps", type=int, default=10, help="控制/录制频率")
    ap.add_argument("--task", default="", help="single_task 文本（默认按 env 推导）")
    ap.add_argument("--max-attempts", type=int, default=3,
                    help="每个成功 episode 的最大尝试数（失败/丢弃后重试）")
    ap.add_argument("--keep-failures", action="store_true",
                    help="失败 episode 也保存（默认只保存成功）")
    ap.add_argument("--seed", type=int, default=None, help="首个 episode 种子（后续 +1）")
    ap.add_argument("--no-dr", action="store_true",
                    help="关闭 checkpoint 记录域随机化（观测噪声/物体尺寸摩擦随机化）")
    ap.add_argument("--no-video", dest="video", action="store_false",
                    help="图像存 PNG 而非视频")
    ap.add_argument("--no-mujoco", dest="mujoco", action="store_false",
                    help="关闭 MuJoCo viewer 窗口（无显示环境）")
    ap.add_argument("--no-rviz", dest="rviz", action="store_false",
                    help="关闭 ros_viz 线程（/joint_states + /il/*/image）")
    ap.add_argument("--camera-keys", default="fixed,wrist",
                    help="相机子集（VQ-BeT 用单相机: --camera-keys fixed）")
    ap.add_argument("--stamp", action="store_true",
                    help="repo_id 追加时间戳（默认不追加，便于训练侧直接引用）")
    ap.set_defaults(video=True, mujoco=True, rviz=True)
    return ap


def resolve_env_id(args, meta: dict) -> str:
    from so101_teleop.cli_config import ENV_IDS

    if args.env:
        return ENV_IDS.get(args.env, derive_camera_env_id(args.env))
    return derive_camera_env_id(str(meta["env_id"]))


def main() -> int:
    args = build_parser().parse_args()

    import so101_teleop  # noqa: F401  触发 draccus 注册（Robot/Teleop choices）
    from so101_rl.backends.base import read_meta
    from so101_rl.runs import resolve_ckpt_dir
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot
    from so101_teleop.teleoperators.rl_expert import RLExpertTeleop, RLExpertTeleopConfig

    ckpt = resolve_ckpt_dir(Path(args.checkpoint).expanduser())
    meta = read_meta(ckpt)
    env_id = resolve_env_id(args, meta)
    short = short_env_name(env_id)
    repo_id = args.repo_id or f"il_sim/{short}_rl_expert"
    if args.stamp:
        repo_id = f"{repo_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    root = (Path(args.root).expanduser() if args.root
            else _REPO / "outputs" / "datasets" / "il_sim" / repo_id)
    if root.exists() and any(root.iterdir()):
        raise SystemExit(f"数据集目录已存在（不覆盖）: {root}；换 --repo-id 或先手动删除")
    task = args.task or DEFAULT_TASKS.get(short, f"perform the {short} task")

    dr = bool(meta.get("env_kwargs", {}).get("domain_randomization", False)) and not args.no_dr
    print(f"[collect] checkpoint={ckpt} ({meta['backend']}:{meta['algo']})")
    print(f"[collect] env={env_id} dr={dr} repo_id={repo_id}")
    print(f"[collect] root={root} episodes={args.episodes} fps={args.fps} task={task!r}")

    camera_keys = [k.strip() for k in args.camera_keys.split(",") if k.strip()]
    robot = SimSO101Robot(SimSO101RobotConfig(
        env_id=env_id, fps=args.fps, seed=args.seed, render=args.mujoco,
        camera_keys=camera_keys, domain_randomization=dr,
    ))
    teleop = RLExpertTeleop(RLExpertTeleopConfig(checkpoint=str(ckpt)))
    viz = None
    dataset = None
    try:
        teleop.connect()
        robot.connect()
        env = robot._env
        max_steps = env.spec.max_episode_steps or 50
        episode_steps = args.episode_steps or int(max_steps)

        if args.rviz:
            from so101_teleop.ros_viz import RosVizThread

            viz = RosVizThread(rate=args.fps)
            viz.start()

        dataset = make_dataset(robot, repo_id, root, args.fps, args.video)
        teleop_proc, _, obs_proc = make_processors()

        saved = attempts = successes = 0
        max_attempts = max(1, args.episodes * args.max_attempts)
        control_dt = 1.0 / args.fps
        episode = 0
        from lerobot.datasets import VideoEncodingManager

        with VideoEncodingManager(dataset):
            while saved < args.episodes and attempts < max_attempts:
                seed = None if args.seed is None else args.seed + attempts
                robot.reset_env(seed=seed)
                t_start = time.perf_counter()
                for _ in range(episode_steps):
                    t0 = time.perf_counter()
                    obs = robot.get_observation()
                    act = teleop.get_action()
                    robot.send_action(act)
                    frame = build_frame(dataset, teleop_proc, obs_proc, act, obs, task)
                    dataset.add_frame(frame)
                    time.sleep(max(0.0, control_dt - (time.perf_counter() - t0)))
                dt = time.perf_counter() - t_start
                info = env.unwrapped.evaluate()
                ok = bool(info.get("success", False))
                successes += int(ok)
                attempts += 1

                if ok or args.keep_failures:
                    dataset.save_episode()
                    saved += 1
                    state = "saved"
                else:
                    dataset.clear_episode_buffer()
                    state = "dropped"
                print(
                    f"[ep {episode:03d}] {state} success={ok} "
                    f"{episode_detail(info)} "
                    f"({dt:.2f}s, {episode_steps / dt:.1f}Hz) "
                    f"-> {saved}/{args.episodes} saved",
                    flush=True,
                )
                episode += 1
    finally:
        if dataset is not None:
            dataset.finalize()
        if robot.is_connected:
            robot.disconnect()
        if teleop.is_connected:
            teleop.disconnect()
        if viz is not None:
            viz.stop()

    print(
        f"[collect] done: {saved}/{args.episodes} episodes saved, "
        f"attempts={attempts}, success={successes}/{attempts}"
        + ("" if attempts == 0 else f" ({successes / attempts:.0%})"))
    print(f"[collect] dataset: {root}")
    print(f"[collect] train: uv run python scripts/il_sim_train.py --policy act "
          f"--repo-id {repo_id} --steps 10")
    return 0


def episode_detail(info: dict) -> str:
    """episode 收尾信息摘要（8 任务 evaluate() 字段差异较大, 只打印存在的）。"""
    parts = []
    if "is_grasped" in info:
        parts.append(f"grasped={info['is_grasped']}")
    if "item_lifted" in info:
        parts.append(f"lifted={info['item_lifted']}")
    for key in ("tcp_to_item_dist", "tcp_to_goal_dist", "a_to_b_xy_dist",
                "distance_to_rest_qpos"):
        if key in info:
            parts.append(f"{key}={float(info[key]):.4f}")
            break
    return " ".join(parts)


def make_dataset(robot, repo_id: str, root: Path, fps: int, video: bool):
    """按官方 record() 同一套 feature 聚合构造 LeRobotDataset。"""
    from lerobot.datasets import (
        LeRobotDataset,
        aggregate_pipeline_dataset_features,
        create_initial_features,
    )
    from lerobot.processor import make_default_processors
    from lerobot.utils.feature_utils import combine_feature_dicts

    teleop_proc, _, obs_proc = make_default_processors()
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_proc,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=video,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=obs_proc,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=video,
        ),
    )
    return LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        root=root,
        robot_type=robot.name,
        features=features,
        use_videos=video,
        image_writer_processes=0,
        image_writer_threads=4 * len(robot.cameras),
        batch_encoding_size=1,
    )


def make_processors():
    from lerobot.processor import make_default_processors

    return make_default_processors()


def build_frame(dataset, teleop_proc, obs_proc, act, obs, task):
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame

    return {
        **build_dataset_frame(dataset.features, obs_proc(obs), prefix=OBS_STR),
        **build_dataset_frame(dataset.features, teleop_proc((act, obs)), prefix=ACTION),
        "task": task,
    }


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
