#!/usr/bin/env python3
"""lift 抓取诊断专家 (workflow/plan/il_sim_leader.md Phase 4 伴随诊断)。

对准方块 (IK, tool-down) → 缓慢闭爪 (逐 0.05 norm) → 抬升, 逐帧记录:
  gripper qpos / 爪-块接触对与穿透深度 / is_grasping / 方块高度
判定"穿模/抓不住"是物理几何问题还是操作问题。

用法:
  uv run python scripts/il_sim_grasp_diagnose.py [--episodes 3]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
for _p in (str(_REPO / "scripts"), str(_REPO / ".venv/lib/python3.12/site-packages")):
    if _p not in sys.path and Path(_p).is_dir():
        sys.path.insert(0, _p)

import mujoco
import numpy as np


def _item_contacts(m, d):
    out = []
    for c in range(d.ncon):
        con = d.contact[c]
        names = {
            mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) for g in (con.geom1, con.geom2)
        }
        if "item" in names and "world" not in names and "table" not in names:
            other = (names - {"item"}).pop() if names - {"item"} else "?"
            out.append((other, round(float(con.dist), 4)))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="lift 抓取诊断专家")
    ap.add_argument("--episodes", type=int, default=3)
    args = ap.parse_args()

    import gymnasium as gym
    import so101_envs  # noqa
    from envs_run_expert import Expert, grasp_item

    for ep in range(args.episodes):
        env = gym.make("SO101Lift-v1")
        env.reset(seed=100 + ep)
        ex = Expert(env)
        ex._wait(10)
        item = ex.u.free_body_pos("item").copy()

        print(f"\n===== episode {ep} (item 位置 {np.round(item, 3)}) =====")
        grasped = grasp_item(ex, item)  # 对准 + 下探 + 闭爪 (逐帧接触记录)

        # 闭爪后 8 帧接触采样
        post = []
        for _ in range(8):
            a = ex.u.normalize_qpos(ex.u.data.qpos[ex.u._qpos_adr])
            a = ex._grip(a, open_=False)
            *_, info = env.step(a)
            post.append(_item_contacts(ex.u.model, ex.u.data))
        print(f"闭爪: is_grasped={grasped}, gripper_qpos={float(ex.u.data.qpos[ex.u._qpos_adr][-1]):.3f}")
        print(f"闭爪后接触 (8 帧): {post[0]} ... 稳定帧 = {post[-1]}")

        # 抬升
        z0 = ex.u.free_body_pos("item")[2]
        ex._go(item + [0, 0, 0.09], steps=15)
        info = ex._wait(8)
        z1 = ex.u.free_body_pos("item")[2]
        print(f"抬升: item z {z0:.4f} → {z1:.4f} (lifted={info['item_lifted']}), success={info['success']}")
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
