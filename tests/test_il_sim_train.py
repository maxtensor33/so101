"""il_sim Phase 4: 训练 config 构造 + ACT steps=10 冒烟（SmolVLA=TODO 见 plan/il_sim.md）。"""

from __future__ import annotations

import pytest


def test_small_policy_configs():
    from so101_teleop.train_config import small_policy_config

    for name in ("act", "diffusion", "vqbet"):
        cfg = small_policy_config(name)
        assert cfg is not None


def test_smolvla_is_todo():
    from so101_teleop.train_config import small_policy_config

    with pytest.raises(NotImplementedError, match="SmolVLA"):
        small_policy_config("smolvla")


def test_build_train_config(tmp_path):
    from lerobot.configs.train import TrainPipelineConfig
    from so101_teleop.train_config import build_train_config

    cfg = build_train_config(policy="act", repo_id="il_sim/smoke", steps=10)
    assert isinstance(cfg, TrainPipelineConfig)
    assert cfg.steps == 10
    assert cfg.batch_size <= 8  # 冒烟小 batch
    assert str(cfg.dataset.root).endswith("outputs/datasets/il_sim/il_sim/smoke")
    assert "act" in str(cfg.output_dir)


@pytest.fixture(scope="module")
def tiny_dataset(tmp_path_factory):
    """录一条 5s episode 作为训练数据（模块级共享）。"""
    from lerobot.configs.dataset import DatasetRecordConfig
    from lerobot.scripts.lerobot_record import RecordConfig, record
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.teleoperators.expert import ScriptExpertTeleopConfig

    root = tmp_path_factory.mktemp("ds") / "tiny"
    rec_cfg = RecordConfig(
        robot=SimSO101RobotConfig(seed=11),
        teleop=ScriptExpertTeleopConfig(),
        dataset=DatasetRecordConfig(
            repo_id="il_sim/tiny",
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
    return root


def test_act_train_smoke(tiny_dataset):
    """steps=10 冒烟：输出 checkpoint 可被 from_pretrained 加载。"""
    from so101_teleop.train_config import build_train_config

    cfg = build_train_config(policy="act", repo_id="il_sim/tiny", steps=10)
    cfg.dataset.root = tiny_dataset
    cfg.output_dir = tiny_dataset.parent / "train_act"
    cfg.num_workers = 0

    from lerobot.scripts.lerobot_train import train

    train(cfg)

    ckpt = cfg.output_dir / "checkpoints" / "000010" / "pretrained_model"
    assert ckpt.is_dir(), f"checkpoint 目录不存在: {ckpt}"
    assert (ckpt / "config.json").exists()

    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.pretrained import PreTrainedPolicy  # noqa: F401

    policy = ACTPolicy.from_pretrained(ckpt)
    assert policy is not None
