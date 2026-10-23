"""RL 训练 CLI（workflow/plan/rl.md；命名约定 <项目名>_<功能>.py）。

用法:
    uv run python scripts/rl_train.py --backend sb3 --algo sac --env SO101Reach-v1 \
        --steps 200000 --out outputs/rl/sb3-sac-reach
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gymnasium as gym

import so101_envs  # noqa: F401  注册副作用
from so101_rl.algos import get_trainer, load_defaults


def main():
    parser = argparse.ArgumentParser(description="SO101 RL 训练（sb3/skrl 双后端）")
    parser.add_argument("--backend", required=True, choices=["sb3", "skrl"])
    parser.add_argument("--algo", required=True, choices=["sac", "ppo", "td3"])
    parser.add_argument("--env", default="SO101Reach-v1")
    parser.add_argument("--steps", type=int, default=200_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--eval-freq", type=int, default=10_000)
    parser.add_argument("--out", default=None, help="checkpoint 目录（默认 outputs/rl/<backend>-<algo>-<env>, 已 gitignore）")
    parser.add_argument("--normalize", action="store_true", help="启用 VecNormalize（状态随 checkpoint 保存）")
    parser.add_argument("--n-envs", type=int, default=1, help="并行环境数（DummyVecEnv / gym.make_vec sync）")
    parser.add_argument("--no-tb", action="store_true", help="关闭 tensorboard（默认开启, 写 <out>/tb/）")
    parser.add_argument("--overrides", nargs="*", default=[], help="超参覆盖 key=val")
    args = parser.parse_args()

    from so101_rl.runs import new_run_dir

    tag = f"{args.backend}-{args.algo}-{args.env.removeprefix('SO101').removeprefix('-').lower()}"
    # 显式 --out 原样使用（用户自管）; 默认按 时间戳分层 + latest 指针, 不覆盖历史
    out_dir = Path(args.out) if args.out else new_run_dir(Path("outputs/rl"), tag)
    hparams = load_defaults(args.backend, args.algo, configs_dir=Path(__file__).resolve().parents[1] / "libs" / "so101_rl" / "configs")
    for kv in args.overrides:
        k, v = kv.split("=", 1)
        hparams[k] = json.loads(v) if v.replace(".", "", 1).isdigit() else v

    trainer = get_trainer(args.backend, args.algo)
    env = trainer.make_vec_envs(args.env, seed=args.seed, n_envs=args.n_envs)
    eval_env = trainer.make_env(args.env, seed=None)
    tb_dir = None if args.no_tb else out_dir / "tb"
    model = trainer.make_model(env, seed=args.seed, hparams=hparams, tb_dir=tb_dir)
    info = trainer.train(model, total_timesteps=args.steps, eval_env=eval_env,
                         eval_freq=args.eval_freq, out_dir=out_dir)
    trainer.save(model, out_dir)
    from so101_rl.plotting import plot_eval_history

    curve = plot_eval_history(info["eval_history"], out_dir)
    print(f"checkpoint: {out_dir}")
    print(f"curve: {curve}")
    if tb_dir:
        print(f"tensorboard: tensorboard --logdir {tb_dir}")
    print(f"eval history: {info['eval_history']}")
    print(f"final success: {info.get('final_success')}")
    env.close()
    eval_env.close()
    sys.stdout.flush()
    # 绕过解释器退出时的 CUDA/EGL 驱动清理——该清理在某些驱动上段错误并连带
    # 崩溃图形会话（wezterm/vscode 闪退）。正常输出已 flush，直接退出是安全的。
    os._exit(0)


if __name__ == "__main__":
    main()
