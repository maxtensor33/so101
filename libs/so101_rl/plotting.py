"""训练可视化（workflow/plan/rl.md 增量：训练步骤可视化）。"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless

import matplotlib.pyplot as plt


def plot_eval_history(history, out_dir: Path) -> Path:
    """eval success rate 曲线 -> <out_dir>/curve.png。"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if not history:
        steps, rates = [0], [0.0]
    else:
        steps, rates = zip(*history)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(steps, rates, marker="o", markersize=3)
    ax.set_xlabel("timesteps")
    ax.set_ylabel("eval success rate")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = out_dir / "curve.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out
