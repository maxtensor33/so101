"""DAgger 训练 CLI（workflow/plan/rl_dagger.md Phase 3; 命名约定 <项目名>_<功能>.py）。

state RL expert -> vision student 两阶段 DAgger（算法本体 libs/so101_rl/dagger.py）。

用法:
    uv run python scripts/rl_dagger_train.py --expert-ckpt outputs/rl/sb3-sac-reach/latest

约定:
- expert 走 resolve_ckpt_dir -> read_meta -> get_trainer -> load_policy 链（D5）;
  --env 缺省由 expert meta env id 推导相机版（镜像 il_sim_collect_rl.py 惯例）,
  与 meta env 不一致时以 --env 为准并 warn。
- env_kwargs: camera=True + render_size（默认 128x128, env 原生）+ camera_calib
  （默认开启, 与 envs 全链路一致, --no-camera-calib 关闭）一并随 checkpoint 落盘;
  action_mode/domain_randomization 沿 expert meta env_kwargs（动力学一致）。
- 超参: 内置默认 < libs/so101_rl/configs/dagger.yaml < --overrides（沿 algos.py 优先级惯例）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gymnasium as gym

import so101_envs  # noqa: F401  注册副作用
from so101_rl.backends.base import read_meta
from so101_rl.dagger import load_dagger_hparams

_REPO = Path(__file__).resolve().parents[1]


def derive_camera_env_id(env_id: str) -> str:
    """训练 env id -> 相机版 id（SO101Lift-v1 -> SO101LiftCamera-v1; 已是相机版原样）。"""
    return env_id if "Camera" in env_id else env_id.replace("-v1", "Camera-v1")


def short_env_name(env_id: str) -> str:
    """SO101ReachCamera-v1 -> reach（out 目录 tag 用, 镜像 il_sim_collect_rl.py）。"""
    return env_id.replace("Camera", "").removeprefix("SO101").removesuffix("-v1").lower()


def parse_render_size(text: str) -> tuple[int, int]:
    """'128x128' -> (128, 128)（HEIGHTxWIDTH, env RENDER_SIZE=(128, 128) 同约定）。"""
    parts = text.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"render size must be HxW, got {text!r}")
    return int(parts[0]), int(parts[1])


def parse_overrides(pairs) -> dict:
    """['lr=0.01', 'collect_size=8'] -> {'lr': 0.01, ...}（数值 json 解析, 其余字符串）。"""
    out: dict = {}
    for kv in pairs:
        k, v = kv.split("=", 1)
        out[k] = json.loads(v) if v.replace(".", "", 1).isdigit() else v
    return out


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="SO101 DAgger 训练（state expert -> vision student）")
    ap.add_argument("--expert-ckpt", required=True,
                    help="RL expert checkpoint 目录（或 <tag>/latest 指针）")
    ap.add_argument("--env", default="",
                    help="采集 env（默认由 expert meta env id 推导相机版）")
    ap.add_argument("--camera-keys", default="fixed",
                    help="逗号分隔相机通道（顺序 = 网络输入顺序, 默认 fixed）")
    ap.add_argument("--rounds", type=int, default=None,
                    help="DAgger 总轮数（total_rounds; 默认 dagger.yaml）")
    ap.add_argument("--n-envs", type=int, default=4,
                    help="gym.make_vec sync 并行环境数")
    ap.add_argument("--render-size", type=parse_render_size, default=(128, 128),
                    metavar="HXW", help="渲染分辨率 HEIGHTxWIDTH（默认 128x128, env 原生）")
    ap.add_argument("--seed", type=int, default=None, help="默认 dagger.yaml 的 seed")
    ap.add_argument("--out", default=None,
                    help="显式 checkpoint 目录; 默认 outputs/rl/dagger-<env 短名>/<时间戳>/ + latest 指针")
    ap.add_argument("--domain-randomization", action="store_true", default=None,
                    dest="domain_randomization", help="强制开启域随机化（默认沿用 expert meta）")
    ap.add_argument("--no-dr", action="store_false", dest="domain_randomization",
                    help="强制关闭域随机化（默认沿用 expert meta）")
    ap.add_argument("--no-camera-calib", action="store_true",
                    help="关闭相机标定（默认开启, 与 envs 全链路一致）")
    ap.add_argument("--overrides", action="append", nargs="*", default=[],
                    metavar="KEY=VAL", help="超参覆盖 key=val（可多次）")
    ap.add_argument("--no-progress", action="store_true",
                    help="关闭 tqdm 轮级进度条（默认开启, 日志改逐行 print）")
    return ap


def build_hparams(args, configs_dir: Path | None = None) -> dict:
    """CLI 旗标 -> train_dagger hparams（内置默认 < dagger.yaml < --overrides < CLI 旗标）。"""
    overrides: dict = {}
    for group in args.overrides:
        overrides.update(parse_overrides(group))
    if args.rounds is not None:
        overrides["total_rounds"] = args.rounds
    hp = load_dagger_hparams(overrides, configs_dir=configs_dir)
    hp["camera_keys"] = [k.strip() for k in args.camera_keys.split(",") if k.strip()]
    hp["n_envs"] = args.n_envs
    hp["render_size"] = list(args.render_size)
    if args.seed is not None:
        hp["seed"] = args.seed
    hp["progress"] = not args.no_progress
    return hp


def build_env_kwargs(meta: dict, args) -> dict:
    """采集 env kwargs。action_mode/domain_randomization 沿 expert meta env_kwargs
    （动力学一致; 缺字段回退 rl_train 默认 delta/True）; 相机语义随存。"""
    meta_kw = dict(meta.get("env_kwargs") or {})
    dr = meta_kw.get("domain_randomization", True)
    if args.domain_randomization is not None:
        dr = args.domain_randomization
    return {
        "action_mode": meta_kw.get("action_mode", "delta"),
        "domain_randomization": dr,
        "camera": True,
        "render_size": list(args.render_size),
        # env 语义: None = 默认路径标定开启（静默回退）, False = 显式关闭
        "camera_calib": False if args.no_camera_calib else None,
    }


def load_expert(ckpt):
    """expert 加载链（D5）: resolve_ckpt_dir -> read_meta -> get_trainer -> load_policy。

    返回 (expert 可调用, meta dict); expert 契约: 单条 state (S,) -> action (6,) in [-1,1]。
    """
    from so101_rl.algos import get_trainer
    from so101_rl.runs import resolve_ckpt_dir

    ckpt = resolve_ckpt_dir(Path(ckpt).expanduser())
    meta = read_meta(ckpt)
    trainer = get_trainer(meta["backend"], meta["algo"])
    return trainer.load_policy(ckpt), meta


def main(argv=None, expert=None) -> int:
    """expert 参数仅供测试注入假 expert; CLI 正常路径走 load_expert 链。"""
    args, _ = build_parser().parse_known_args(argv)  # parse_known_args 忽略 --ros-args

    if expert is None:
        expert, expert_meta = load_expert(args.expert_ckpt)
    else:
        expert_meta = read_meta(Path(args.expert_ckpt).expanduser())
    meta_env = str(expert_meta["env_id"])
    env_id = derive_camera_env_id(args.env) if args.env else derive_camera_env_id(meta_env)
    if args.env and args.env != meta_env:
        print(f"[dagger] warn: --env {args.env} 与 expert meta env {meta_env} 不一致, "
              f"以 --env 为准（D5）")

    hp = build_hparams(args, configs_dir=_REPO / "libs" / "so101_rl" / "configs")
    env_kwargs = build_env_kwargs(expert_meta, args)

    from so101_rl.runs import new_run_dir

    tag = f"dagger-{short_env_name(env_id)}"
    out_dir = Path(args.out) if args.out else new_run_dir(Path("outputs/rl"), tag)

    print(f"[dagger] expert={args.expert_ckpt} ({expert_meta['backend']}:"
          f"{expert_meta['algo']})")
    print(f"[dagger] env={env_id} cameras={hp['camera_keys']} out={out_dir}")
    print(f"[dagger] rounds={hp['total_rounds']} n_envs={hp['n_envs']} "
          f"render_size={hp['render_size']} camera_calib={env_kwargs['camera_calib']} "
          f"action_mode={env_kwargs['action_mode']} "
          f"dr={env_kwargs['domain_randomization']}")

    vec_env = gym.make_vec(env_id, num_envs=hp["n_envs"], vectorization_mode="sync",
                           **env_kwargs)
    try:
        from so101_rl.dagger import train_dagger

        info = train_dagger(expert, vec_env, hp, out_dir=out_dir, env_id=env_id,
                            env_kwargs=env_kwargs, device="cpu", log_fn=print)
    finally:
        vec_env.close()

    print(f"[dagger] checkpoint: {info['out_dir']}")
    print(f"[dagger] rounds={info['rounds']} steps={info['steps']}")
    if info["bc_losses"]:
        print(f"[dagger] final bc_loss={info['bc_losses'][-1]:.6f}")
    return 0


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    # 绕过解释器退出时的 CUDA/EGL 驱动清理——该清理在某些驱动上段错误并连带
    # 崩溃图形会话（同 scripts/rl_train.py 惯例）。正常输出已 flush，直接退出安全。
    import os

    os._exit(0)
