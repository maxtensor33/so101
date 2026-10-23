"""il_sim Phase 3: 遥操主进程的 config 构造器 + teleoperate 模式冒烟（ROS 无关部分）。"""

from __future__ import annotations

import pytest


def test_build_record_config():
    from so101_teleop.cli_config import build_record_config

    cfg = build_record_config(
        teleop="expert",
        env="reach",
        repo_id="il_sim/reach_expert",
        episodes=3,
        episode_time_s=8.0,
    )
    from lerobot.scripts.lerobot_record import RecordConfig

    assert isinstance(cfg, RecordConfig)
    assert cfg.robot.env_id == "SO101ReachCamera-v1"
    assert cfg.robot.fps == 10
    # teleop 类型映射
    assert cfg.teleop is not None and "expert" in type(cfg.teleop).__name__.lower()
    # 数据集约定: outputs/datasets/il_sim/<repo_id>
    assert str(cfg.dataset.root).endswith("outputs/datasets/il_sim/il_sim/reach_expert")
    assert cfg.dataset.fps == 10 and cfg.dataset.num_episodes == 3
    assert cfg.dataset.push_to_hub is False and cfg.dataset.video is True


def test_build_teleoperate_config():
    from so101_teleop.cli_config import build_teleop_mode_config

    cfg = build_teleop_mode_config(teleop="keyboard", env="reach")
    assert cfg.robot.env_id == "SO101ReachCamera-v1"
    assert cfg.fps == 10
    assert "keyboard" in type(cfg.teleop).__name__.lower()


def test_unknown_teleop_rejected():
    from so101_teleop.cli_config import build_record_config

    with pytest.raises(ValueError, match="teleop"):
        build_record_config(teleop="leader_xxx", env="reach", repo_id="x", episodes=1)


def test_teleoperate_smoke():
    """teleoperate 模式（不落盘）跑 1.5s：循环无异常即通过。"""
    from so101_teleop import cli_config

    cfg = cli_config.build_teleop_mode_config(teleop="expert", env="reach")
    cfg.teleop_time_s = 1.5
    cfg.display_data = False
    from lerobot.scripts.lerobot_teleoperate import teleoperate

    teleoperate(cfg)  # 内部自建 robot/teleop


def test_keyboard_init_not_connected():
    """keyboard teleop 构造后 is_connected=False（record 异常清理路径依赖）。"""
    from so101_teleop.teleoperators.keyboard import So101KeyboardTeleop, So101KeyboardTeleopConfig

    t = So101KeyboardTeleop(So101KeyboardTeleopConfig())
    assert t.is_connected is False


def test_resume_plumbing():
    """resume 透传 RecordConfig（lerobot 语义: 已存在数据集续录）。"""
    from so101_teleop.cli_config import build_record_config

    cfg = build_record_config(teleop="keyboard", env="reach", repo_id="il_sim/x",
                              episodes=1, resume=True)
    assert cfg.resume is True
    assert build_record_config(teleop="expert", env="reach", repo_id="il_sim/x",
                               episodes=1).resume is False
