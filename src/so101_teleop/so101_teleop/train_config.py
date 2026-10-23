"""il_sim 训练 config 组装（plan/il_sim.md Phase 4）。

小网络模板：只跑通流程用（冒烟/联调）；全量训练由用户迁移到大显存机器
（迁移指引见 docs/il_sim/index.md），届时用官方默认超参即可。
"""

from __future__ import annotations

_REPO = None
for _p in __import__("pathlib").Path(__file__).resolve().parents:
    if (_p / "pyproject.toml").is_file():
        _REPO = _p
        break


def small_policy_config(policy: str):
    """三策略小网络模板（可跑通流程的缩减超参）。"""
    if policy == "act":
        from lerobot.policies.act.configuration_act import ACTConfig

        return ACTConfig(
            dim_model=128,
            n_encoder_layers=2,
            n_decoder_layers=2,
            dim_feedforward=512,
            chunk_size=10,
            n_action_steps=5,
        )
    if policy == "diffusion":
        from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig

        return DiffusionConfig(
            down_dims=[16, 32],
            n_obs_steps=2,
            horizon=8,
            n_action_steps=4,
        )
    if policy == "vqbet":
        from lerobot.policies.vqbet.configuration_vqbet import VQBeTConfig

        return VQBeTConfig(
            n_obs_steps=2,
            n_action_pred_token=2,
            action_chunk_size=3,
        )
    if policy == "smolvla":
        raise NotImplementedError(
            "SmolVLA 为 TODO（plan/il_sim.md）：需 smolvla_base 预训练权重 HF hub 下载 + 显存评估"
        )
    raise ValueError(f"unknown policy: {policy} (expected act|diffusion|vqbet)")


def build_train_config(policy: str, repo_id: str, steps: int, batch_size: int = 4):
    from lerobot.configs.default import DatasetConfig
    from lerobot.configs.train import TrainPipelineConfig

    root = _REPO / "outputs" / "datasets" / "il_sim" / repo_id
    output_dir = _REPO / "outputs" / "il_sim" / f"{policy}_{repo_id.split('/')[-1]}"

    policy_cfg = small_policy_config(policy)
    policy_cfg.push_to_hub = False  # 本地训练不推 hub（默认 True 会要求 repo_id）
    cfg = TrainPipelineConfig(
        dataset=DatasetConfig(repo_id=repo_id, root=str(root)),
        policy=policy_cfg,
        # 冒烟/本地训练不推 hub（policy config push_to_hub 默认 True 会要求 repo_id）

        output_dir=output_dir,
        steps=steps,
        batch_size=batch_size,

        num_workers=0,
        log_freq=200,
        save_freq=10_000,
    )
    return cfg
