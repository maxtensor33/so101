"""CLI config 构造器（plan/il_sim.md Phase 3）：遥操/采集两种模式的 lerobot config 组装。

ROS 无关、可单测；scripts/il_sim_teleop.py 与 launch 验收共用。
"""

from __future__ import annotations

from pathlib import Path

_REPO = None
for _p in Path(__file__).resolve().parents:
    if (_p / "pyproject.toml").is_file():
        _REPO = _p
        break

ENV_IDS = {
    "reach": "SO101ReachCamera-v1",
    "lift": "SO101LiftCamera-v1",
    "place": "SO101PlaceCamera-v1",
    "stack": "SO101StackCamera-v1",
}


def resolve_env_id(env: str) -> str:
    if env in ENV_IDS:
        return ENV_IDS[env]
    return env  # 允许直接传完整注册名


def default_dataset_root(repo_id: str) -> Path:
    return _REPO / "outputs" / "datasets" / "il_sim" / repo_id


def make_teleop_config(teleop: str):
    if teleop == "expert":
        from so101_teleop.teleoperators.expert import ScriptExpertTeleopConfig

        return ScriptExpertTeleopConfig()
    if teleop == "keyboard":
        from so101_teleop.teleoperators.keyboard import So101KeyboardTeleopConfig

        return So101KeyboardTeleopConfig()
    raise ValueError(f"unknown teleop: {teleop} (expected expert|keyboard)")


def build_record_config(
    teleop: str,
    env: str,
    repo_id: str,
    episodes: int,
    episode_time_s: float = 8.0,
    video: bool = True,
    task: str = "reach the goal above the item",
    seed: int | None = None,
    render: bool = False,
    reset_time_s: float | None = None,
    resume: bool = False,
):
    from lerobot.configs.dataset import DatasetRecordConfig
    from lerobot.scripts.lerobot_record import RecordConfig

    from .configs import SimSO101RobotConfig

    if reset_time_s is None:
        # keyboard: 成功即自动重置, reset 期留给 lerobot 的集间缓冲; expert: 状态机自动重置
        reset_time_s = 5.0 if teleop == "keyboard" else 0.0
    auto_reset_on_success = teleop == "keyboard"

    return RecordConfig(
        resume=resume,
        robot=SimSO101RobotConfig(
            env_id=resolve_env_id(env), fps=10, seed=seed, render=render,
            auto_reset_on_success=auto_reset_on_success,
        ),
        teleop=make_teleop_config(teleop),
        dataset=DatasetRecordConfig(
            repo_id=repo_id,
            single_task=task,
            root=default_dataset_root(repo_id),
            fps=10,
            episode_time_s=episode_time_s,
            reset_time_s=reset_time_s,
            num_episodes=episodes,
            video=video,
            push_to_hub=False,
        ),
        play_sounds=False,
    )


def build_teleop_mode_config(teleop: str, env: str, fps: int = 10, render: bool = False):
    from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig

    from .configs import SimSO101RobotConfig

    return TeleoperateConfig(
        robot=SimSO101RobotConfig(env_id=resolve_env_id(env), fps=fps, render=render),
        teleop=make_teleop_config(teleop),
        fps=fps,
        display_data=False,
    )
