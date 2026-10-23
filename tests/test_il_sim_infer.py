"""il_sim Phase 5: lerobot policy checkpoint 加载器 + predict 形状（ROS 无关部分）。

自含链路：record 1 episode → ACT steps=10 → load_policy → select_action 形状/值域校验。
ROS 推理节点本身（rclpy）不在 uv venv 覆盖范围，launch 验收见 workflow/tdd/il_sim.tdd.md。
"""

from __future__ import annotations

import numpy as np
import pytest
import torch


@pytest.fixture(scope="module")
def act_checkpoint(tmp_path_factory):
    """录 1 条 5s episode + ACT steps=10 → checkpoint 目录。"""
    from lerobot.configs.dataset import DatasetRecordConfig
    from lerobot.scripts.lerobot_record import RecordConfig, record
    from lerobot.scripts.lerobot_train import train
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.teleoperators.expert import ScriptExpertTeleopConfig
    from so101_teleop.train_config import build_train_config

    root = tmp_path_factory.mktemp("infer") / "tiny"
    rec_cfg = RecordConfig(
        robot=SimSO101RobotConfig(seed=13),
        teleop=ScriptExpertTeleopConfig(),
        dataset=DatasetRecordConfig(
            repo_id="il_sim/infer_tiny",
            single_task="reach",
            root=root,
            fps=10,
            episode_time_s=5,
            reset_time_s=0,
            num_episodes=1,
            video=False,
            push_to_hub=False,
            num_image_writer_threads_per_camera=1,
        ),
        play_sounds=False,
    )
    record(rec_cfg)

    cfg = build_train_config(policy="act", repo_id="il_sim/infer_tiny", steps=10)
    cfg.dataset.root = root
    cfg.output_dir = tmp_path_factory.mktemp("infer") / "train_act"
    cfg.num_workers = 0
    train(cfg)
    return cfg.output_dir / "checkpoints" / "000010" / "pretrained_model"


def test_load_policy(act_checkpoint):
    from lerobot.policies.pretrained import PreTrainedConfig
    from so101_il_sim_infer import load_policy

    policy, pre, post = load_policy(act_checkpoint)
    from lerobot.policies.pretrained import PreTrainedPolicy

    assert isinstance(policy, PreTrainedPolicy)
    assert policy.config.type == "act"


def test_predict_action_shape(act_checkpoint):
    from so101_il_sim_infer import load_policy, make_obs_batch

    policy, pre, post = load_policy(act_checkpoint)

    import gymnasium as gym
    import so101_envs  # noqa: F401

    env = gym.make("SO101ReachCamera-v1")
    obs, _ = env.reset(seed=0)
    env.close()

    batch = make_obs_batch(obs, u=env.unwrapped)
    with torch.no_grad():
        action = post(policy.select_action(pre(batch)))
    arr = action.squeeze(0).cpu().numpy() if action.ndim > 1 else action.cpu().numpy()
    assert arr.shape == (6,)
    # 数据集动作空间为 [-1,1] 归一化关节目标
    assert np.all(arr >= -1.5) and np.all(arr <= 1.5)
