"""VecNormalize 开关约定（plan/rl.md）。

默认关闭。启用时训练侧 wrap + 随 checkpoint 保存 vecnormalize.pkl，
推理/评估侧必须从同一 checkpoint 目录加载（契约测试锁定往返一致）。
"""

from __future__ import annotations

from pathlib import Path

NORM_FILE = "vecnormalize.pkl"


def maybe_wrap(env, normalize: bool):
    """normalize=True 时包 VecNormalize（training=True, norm_reward=False）。"""
    if not normalize:
        return env
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    return VecNormalize(DummyVecEnv([lambda: env]), norm_obs=True, norm_reward=False)


def save_norm(vec_env, ckpt_dir: Path) -> bool:
    """若 env 链含 VecNormalize 则保存状态; 返回是否保存。"""
    from stable_baselines3.common.vec_env import VecNormalize

    if isinstance(vec_env, VecNormalize):
        vec_env.save(str(Path(ckpt_dir) / NORM_FILE))
        return True
    return False


def load_norm(ckpt_dir: Path, venv):
    """加载 vecnormalize.pkl（存在则必须加载，否则策略观测分布不匹配）。"""
    path = Path(ckpt_dir) / NORM_FILE
    if not path.exists():
        return venv, False
    from stable_baselines3.common.vec_env import VecNormalize

    return VecNormalize.load(str(path), venv), True
