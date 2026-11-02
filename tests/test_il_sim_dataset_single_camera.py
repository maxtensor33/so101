"""双相机 -> 单相机数据集转换（VQ-BeT 对照数据）的契约测试。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest


@pytest.fixture
def two_cam_dataset(tmp_path):
    """2 episodes x 3 frames 的合成双相机 LeRobotDataset（PNG, 4x4）。"""
    from lerobot.datasets import VideoEncodingManager
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = tmp_path / "two_cam"
    cam = lambda: {"dtype": "image", "shape": [4, 4, 3],
                   "names": ["height", "width", "channels"]}
    features = {
        "action": {"dtype": "float32", "shape": [2], "names": ["a", "b"]},
        "observation.state": {"dtype": "float32", "shape": [2], "names": ["s0", "s1"]},
        "observation.images.fixed": cam(),
        "observation.images.wrist": cam(),
    }
    ds = LeRobotDataset.create(
        "il_sim/two_cam", fps=10, features=features, root=root, robot_type="so101_sim",
        use_videos=False, image_writer_processes=0, image_writer_threads=2,
        batch_encoding_size=1,
    )
    with VideoEncodingManager(ds):
        for ep in range(2):
            for i in range(3):
                ds.add_frame({
                    "action": np.array([ep, i], dtype=np.float32),
                    "observation.state": np.array([i, ep], dtype=np.float32),
                    "observation.images.fixed": np.full((4, 4, 3), 10 + ep, dtype=np.uint8),
                    "observation.images.wrist": np.full((4, 4, 3), 20 + ep, dtype=np.uint8),
                    "task": "pick",
                })
            ds.save_episode()
    return root


def test_convert_keeps_fixed_camera(two_cam_dataset):
    import il_sim_dataset_single_camera as mod
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    info = mod.convert_dataset(two_cam_dataset, repo_id="il_sim/two_cam", video=False)
    assert info["repo_id"] == "il_sim/two_cam_single"
    out_root = Path(info["root"])
    assert out_root.name.endswith("_single")

    out = LeRobotDataset("il_sim/two_cam_single", root=out_root)
    src = LeRobotDataset("il_sim/two_cam", root=two_cam_dataset)
    assert out.num_episodes == src.num_episodes == 2
    assert out.num_frames == src.num_frames == 6
    assert mod.camera_keys(out.features) == ["fixed"]
    assert "observation.images.wrist" not in out.features
    for i in range(6):
        assert np.allclose(np.asarray(src[i]["action"]), np.asarray(out[i]["action"]))
        assert np.allclose(
            np.asarray(src[i]["observation.state"]),
            np.asarray(out[i]["observation.state"]))


def test_keep_wrist_and_suffix(two_cam_dataset):
    import il_sim_dataset_single_camera as mod
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    info = mod.convert_dataset(two_cam_dataset, keep="wrist", suffix="_wrist", video=False)
    assert info["repo_id"].endswith("_wrist")
    out = LeRobotDataset(info["repo_id"], root=info["root"])
    assert mod.camera_keys(out.features) == ["wrist"]


def test_single_camera_source_rejected(two_cam_dataset):
    import il_sim_dataset_single_camera as mod

    mod.convert_dataset(two_cam_dataset, repo_id="il_sim/two_cam", video=False)
    single_root = two_cam_dataset.with_name(two_cam_dataset.name + "_single")
    with pytest.raises(SystemExit, match="单相机"):
        mod.convert_dataset(single_root, repo_id="il_sim/two_cam", video=False)


def test_dry_run_writes_nothing(two_cam_dataset):
    import il_sim_dataset_single_camera as mod

    info = mod.convert_dataset(two_cam_dataset, repo_id="il_sim/two_cam",
                               dry_run=True, video=False)
    assert not Path(info["root"]).exists()


def test_to_hwc_uint8():
    import il_sim_dataset_single_camera as mod

    chw = np.arange(3 * 4 * 4, dtype=np.uint8).reshape(3, 4, 4)
    assert mod.to_hwc_uint8(chw).shape == (4, 4, 3)
    assert mod.to_hwc_uint8(chw)[0, 0].tolist() == [0, 16, 32]

    f = np.zeros((3, 2, 2), dtype=np.float32)
    f[0] = 1.0
    hwc = mod.to_hwc_uint8(f)
    assert hwc.dtype == np.uint8 and hwc.shape == (2, 2, 3)
    assert hwc[0, 0].tolist() == [255, 0, 0]


def test_infer_repo_id():
    import il_sim_dataset_single_camera as mod

    under = mod.DATASET_BASE / "il_sim" / "lift_rl_expert"
    assert mod.infer_repo_id(under) == "il_sim/lift_rl_expert"
    assert mod.infer_repo_id(Path("/tmp/some_dataset")) == "some_dataset"
