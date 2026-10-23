"""il_sim Phase 1/2: SimSO101Robot 与 Teleoperator 的 lerobot 协议契约测试。

锚定 0.6.1 官方约定：
- observation_features/action_features 键名 `{motor}.pos: float` → 派生 `observation.state`/`action`（shape (6,)）
- 相机键名（fixed/wrist）: (H,W,3) → 派生 `observation.images.<key>`
- record_loop 调用序：teleop.connect → robot.connect → get_observation → get_action → send_action
"""

from __future__ import annotations

import numpy as np
import pytest

from lerobot.datasets.pipeline_features import (
    aggregate_pipeline_dataset_features,
    create_initial_features,
)

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


@pytest.fixture
def sim_config():
    from lerobot.robots.config import RobotConfig
    from so101_teleop.configs import SimSO101RobotConfig

    cfg = SimSO101RobotConfig()
    assert isinstance(cfg, RobotConfig)
    return cfg


@pytest.fixture
def robot(sim_config):
    from so101_teleop.robot import SimSO101Robot

    r = SimSO101Robot(sim_config)
    r.connect()
    yield r
    r.disconnect()


def test_sim_config_registered():
    from lerobot.robots.config import RobotConfig
    from so101_teleop.configs import SimSO101RobotConfig

    # 官方 lerobot-record CLI 经 draccus ChoiceRegistry 解析 --robot.type
    registry = RobotConfig.get_known_choices()
    assert "so101_sim" in registry
    cfg = SimSO101RobotConfig(env_id="SO101ReachCamera-v1")
    assert cfg.env_id == "SO101ReachCamera-v1"


def test_robot_name_and_cameras(robot):
    from so101_teleop.robot import SimSO101Robot

    assert isinstance(robot, SimSO101Robot)
    assert robot.name == "so101_sim"
    # record() 用 len(robot.cameras) 计算图像写线程数
    assert len(robot.cameras) == 2
    assert set(robot.cameras) == {"fixed", "wrist"}


def test_robot_features(robot):
    obs_ft = robot.observation_features
    act_ft = robot.action_features

    # 关节特征: {motor}.pos -> float（派生 observation.state / action, shape (6,)）
    for name in JOINT_NAMES:
        assert obs_ft[f"{name}.pos"] is float
        assert act_ft[f"{name}.pos"] is float
    # 相机特征: (H, W, 3) 元组
    for cam in ("fixed", "wrist"):
        shape = obs_ft[cam]
        assert isinstance(shape, tuple) and len(shape) == 3 and shape[2] == 3
    assert set(act_ft) == {f"{n}.pos" for n in JOINT_NAMES}


def test_derived_dataset_features(robot):
    """用 lerobot 官方推导管线锁定 features 键名，保证 LeRobotDataset 结构正确。"""
    from lerobot.utils.constants import ACTION, OBS_STR

    ds_features = aggregate_pipeline_dataset_features(
        pipeline=_identity_pipeline(),
        initial_features=create_initial_features(
            action=robot.action_features, observation=robot.observation_features
        ),
        use_videos=False,  # 冒烟用 image 而非 video，避免 ffmpeg 依赖
    )
    assert ds_features[f"{OBS_STR}.state"]["shape"] == (6,)
    assert list(ds_features[f"{OBS_STR}.state"]["names"]) == [f"{n}.pos" for n in JOINT_NAMES]
    assert ds_features[ACTION]["shape"] == (6,)
    for cam in ("fixed", "wrist"):
        ft = ds_features[f"{OBS_STR}.images.{cam}"]
        assert ft["dtype"] == "image"
        assert ft["shape"] == robot.observation_features[cam]


def _identity_pipeline():
    """Identity processor：不改变 features（record 默认 IdentityProcessorStep）。"""
    from lerobot.processor import RobotProcessorPipeline
    from lerobot.processor.pipeline import IdentityProcessorStep

    return RobotProcessorPipeline([IdentityProcessorStep()])


def test_robot_observation_roundtrip(robot):
    obs = robot.get_observation()

    # 键与 features 对齐
    assert set(obs) == set(robot.observation_features)
    for name in JOINT_NAMES:
        assert isinstance(obs[f"{name}.pos"], float)
        assert -1.0 <= obs[f"{name}.pos"] <= 1.0  # 归一化到动作空间同尺度
    for cam in ("fixed", "wrist"):
        img = obs[cam]
        assert img.dtype == np.uint8 and img.ndim == 3 and img.shape[2] == 3
        assert img.shape == robot.observation_features[cam]


def test_robot_send_action_and_state_change(robot):
    first = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}

    action = {f"{n}.pos": 0.5 * (i % 3 - 1) for i, n in enumerate(JOINT_NAMES)}
    sent = robot.send_action(action)
    assert set(sent) == set(action)  # 返回实际下发动作（record 存 sent action）

    for _ in range(10):
        robot.send_action(action)
    last = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}

    # 关节应朝目标方向移动（状态连续变化）
    moved = [abs(last[f"{n}.pos"] - first[f"{n}.pos"]) > 1e-3 for n in JOINT_NAMES]
    assert sum(moved) >= 4, f"期望多数关节发生移动, moved={moved}"


def test_robot_lifecycle_stubs(robot):
    # 仿真无需标定：stub 应恒真且不抛异常
    assert robot.is_connected
    robot.calibrate()
    assert robot.is_calibrated
    robot.disconnect()
    assert not robot.is_connected


def test_single_camera_config():
    """VQ-BeT 只支持单相机：config.cameras 可裁剪 features。"""
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot

    r = SimSO101Robot(SimSO101RobotConfig(seed=1, camera_keys=["fixed"]))
    r.connect()
    try:
        assert set(r.cameras) == {"fixed"}
        assert "wrist" not in r.observation_features
        assert "fixed" in r.observation_features
        obs = r.get_observation()
        assert "wrist" not in obs and "fixed" in obs
    finally:
        r.disconnect()


def test_render_mode_plumbing():
    """render=True → env 以 human 模式创建（viewer 惰性, connect 不弹窗）；默认关闭。"""
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot

    r = SimSO101Robot(SimSO101RobotConfig(seed=1, render=True))
    r.connect()
    try:
        assert r._env.render_mode == "human"
    finally:
        r.disconnect()

    r0 = SimSO101Robot(SimSO101RobotConfig(seed=1))
    r0.connect()
    try:
        assert r0._env.render_mode is None
    finally:
        r0.disconnect()


def test_viewer_options_plumbing():
    """viewer_ui/viewer_maximize 经 gym.make kwargs 到达 env（launch 默认隐藏+最大化）。"""
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot

    r = SimSO101Robot(SimSO101RobotConfig(
        seed=1, render=True, viewer_ui=False, viewer_maximize=True))
    r.connect()
    try:
        u = r._env.unwrapped
        assert u._viewer_ui is False
        assert u._viewer_maximize is True
    finally:
        r.disconnect()
