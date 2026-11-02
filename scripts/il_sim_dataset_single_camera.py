#!/usr/bin/env python
"""把双相机 il_sim 数据集裁剪为单相机数据集（默认保留 fixed 视角）。

用途：VQ-BeT 的 validate_features 强制 1 路图像；用同一份数据的单相机副本做
ACT/DP/VQ-BeT 对照实验。

用法:
    uv run python scripts/il_sim_dataset_single_camera.py \
        --dataset-root outputs/datasets/il_sim/il_sim/lift_rl_expert

默认输出:
    repo_id = <源 repo_id>_single（如 il_sim/lift_rl_expert_single）
    root    = <源目录>_single（如 outputs/datasets/il_sim/il_sim/lift_rl_expert_single）

- 逐 episode 复制 action/observation.state/task 与保留相机; 视频数据集默认重新编码
  （--no-video 转 PNG）, 保持 fps/robot_type/feature names。
- 源数据集必须 >=2 路相机；目标目录已存在时拒绝覆盖。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
DATASET_BASE = _REPO / "outputs" / "datasets" / "il_sim"
IMAGE_PREFIX = "observation.images."
# create() 自动生成的元数据列，不能作为用户 feature 传给 create
META_FEATURE_KEYS = frozenset(
    {"timestamp", "frame_index", "episode_index", "index", "task_index"})


def infer_repo_id(root: Path) -> str:
    """数据集目录 -> repo_id（优先相对 outputs/datasets/il_sim；否则用目录名）。"""
    root = Path(root).expanduser().resolve()
    try:
        return root.relative_to(DATASET_BASE.resolve()).as_posix()
    except ValueError:
        return root.name


def camera_keys(features: dict) -> list[str]:
    return sorted(k.removeprefix(IMAGE_PREFIX)
                  for k in features if k.startswith(IMAGE_PREFIX))


def to_hwc_uint8(value) -> np.ndarray:
    """数据集帧图像（CHW uint8/float/0-1）-> add_frame 需要的 HWC uint8。

    LeRobotDataset.__getitem__ 对 image/video 一律返回 CHW（3,H,W），因此按首维判断；
    已是 HWC 的输入原样保留。
    """
    arr = value.cpu().numpy() if hasattr(value, "cpu") else np.asarray(value)
    if arr.ndim != 3:
        raise ValueError(f"图像维度应为 3, got {arr.shape}")
    if arr.shape[0] in (1, 3, 4):
        arr = np.transpose(arr, (1, 2, 0))
    elif arr.shape[-1] not in (1, 3, 4):
        raise ValueError(f"无法识别的图像布局: {arr.shape}")
    if arr.dtype != np.uint8:
        if arr.max() <= 1.0:
            arr = arr * 255.0
        arr = np.clip(np.rint(arr), 0, 255).astype(np.uint8)
    return arr


def prepare_features(src_features: dict, keep_key: str, video: bool) -> dict:
    """源 features -> 单相机目标 features（去掉元数据列与其余相机）。"""
    out = {}
    for key, ft in src_features.items():
        if key in META_FEATURE_KEYS:
            continue
        if key.startswith(IMAGE_PREFIX):
            if key != keep_key:
                continue
            ft = dict(ft)
            ft["dtype"] = "video" if video else "image"
            if not video:
                ft.pop("info", None)
            out[key] = ft
        else:
            out[key] = dict(ft)
    return out


def convert_frame(frame: dict, features: dict, keep_key: str) -> dict:
    out = {}
    for key in features:
        value = frame[key]
        if key == keep_key:
            out[key] = to_hwc_uint8(value)
        elif hasattr(value, "cpu"):
            out[key] = value.cpu().numpy()
        else:
            out[key] = np.asarray(value)
    out["task"] = frame["task"]
    return out


def convert_dataset(
    source_root: Path,
    repo_id: str | None = None,
    keep: str = "fixed",
    suffix: str = "_single",
    output_root: Path | None = None,
    video: bool | None = None,
    dry_run: bool = False,
) -> dict:
    from lerobot.datasets import VideoEncodingManager
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source_root = Path(source_root).expanduser()
    repo_id = repo_id or infer_repo_id(source_root)
    if repo_id.endswith(suffix):
        raise SystemExit(f"repo_id 已以 {suffix} 结尾: {repo_id}（避免自我覆盖）")
    new_repo_id = repo_id + suffix
    new_root = (Path(output_root).expanduser() if output_root is not None
                else source_root.with_name(source_root.name + suffix))

    if not source_root.is_dir():
        raise SystemExit(f"数据集目录不存在: {source_root}")
    src = LeRobotDataset(repo_id, root=source_root, return_uint8=True)
    cams = camera_keys(src.features)
    if len(cams) < 2:
        raise SystemExit(f"源数据集已是单相机 {cams}，无需转换")
    if keep not in cams:
        raise SystemExit(f"相机 {keep!r} 不存在（可用: {cams}）")
    keep_key = IMAGE_PREFIX + keep

    if video is None:
        video = src.features[keep_key]["dtype"] == "video"
    features = prepare_features(src.features, keep_key, video)

    print(f"[single] source repo_id={repo_id} root={source_root}")
    print(f"[single] cameras={cams} -> keep={keep} ({'video' if video else 'image'})")
    print(f"[single] target repo_id={new_repo_id} root={new_root}")
    print(f"[single] frames={src.num_frames} episodes={src.num_episodes} fps={src.fps}")
    if dry_run:
        print("[single] dry-run: 未写盘")
        return {"repo_id": new_repo_id, "root": new_root}

    if new_root.exists() and any(new_root.iterdir()):
        raise SystemExit(f"目标目录已存在（不覆盖）: {new_root}")

    dst = LeRobotDataset.create(
        repo_id=new_repo_id,
        fps=src.fps,
        features=features,
        root=new_root,
        robot_type=src.meta.robot_type,
        use_videos=video,
        image_writer_processes=0,
        image_writer_threads=4,
        batch_encoding_size=1,
    )

    with VideoEncodingManager(dst):
        prev_ep = None
        for i in range(src.num_frames):
            frame = src[i]
            ep = int(frame["episode_index"])
            if prev_ep is not None and ep != prev_ep:
                dst.save_episode()
            dst.add_frame(convert_frame(frame, features, keep_key))
            prev_ep = ep
            if (i + 1) % 1000 == 0:
                print(f"[single] {i + 1}/{src.num_frames} frames", flush=True)
        if prev_ep is not None:
            dst.save_episode()

    # 校验：帧/情节数一致、多余相机已移除、action/state 数值一致
    out = LeRobotDataset(new_repo_id, root=new_root, return_uint8=True)
    if out.num_frames != src.num_frames or out.num_episodes != src.num_episodes:
        raise SystemExit(
            f"转换结果不一致: frames {out.num_frames}!={src.num_frames} / "
            f"episodes {out.num_episodes}!={src.num_episodes}")
    if camera_keys(out.features) != [keep]:
        raise SystemExit(f"输出相机异常: {camera_keys(out.features)} != [{keep}]")
    for i in (0, src.num_frames - 1):
        for key in ("action", "observation.state"):
            if not np.allclose(np.asarray(src[i][key]), np.asarray(out[i][key])):
                raise SystemExit(f"第 {i} 帧 {key} 数值不一致")

    print(f"[single] done: {out.num_episodes} episodes / {out.num_frames} frames -> {new_root}")
    print(f"[single] train: uv run python scripts/il_sim_train.py --policy vqbet "
          f"--repo-id {new_repo_id} --steps 10")
    return {"repo_id": new_repo_id, "root": new_root, "features": out.features}


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset-root", required=True, type=Path,
                    help="源 LeRobotDataset 目录（outputs/datasets/il_sim/<repo_id>）")
    ap.add_argument("--repo-id", default="",
                    help="源 repo_id（默认由路径相对 outputs/datasets/il_sim 推导）")
    ap.add_argument("--keep", default="fixed", help="保留的相机键（默认 fixed；另有 wrist）")
    ap.add_argument("--suffix", default="_single", help="新数据集后缀（默认 _single）")
    ap.add_argument("--output-root", type=Path, default=None,
                    help="输出目录（默认 <源目录名>_single 同级目录）")
    ap.add_argument("--no-video", dest="video", action="store_false",
                    help="图像存 PNG 而非视频（默认沿用源数据集）")
    ap.add_argument("--video", dest="video", action="store_true",
                    help="强制输出视频（源为 PNG 时转视频）")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不写盘")
    ap.set_defaults(video=None)
    return ap


def main() -> int:
    args = build_parser().parse_args()
    convert_dataset(
        source_root=args.dataset_root,
        repo_id=args.repo_id or None,
        keep=args.keep,
        suffix=args.suffix,
        output_root=args.output_root,
        video=args.video,
        dry_run=args.dry_run,
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
