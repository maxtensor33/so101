#!/usr/bin/env python3
"""leader 臂映射探针 (workflow/plan/il_sim_leader.md Phase 3, 只读不驱动)。

连接真实 leader 臂, 实时打印 deg/0-100 → rad → [-1,1] 三列 (换算走 so101_leader_sim
的 follower 标定链), 供操作员校验:
  1. leader 回 home 时各关节 norm 值 (记录 home 偏差)
  2. 各关节方向一致性 (掰动时 norm 同向变化; 反向关节记入 --invert)
  3. 夹爪开合方向
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
for _p in (str(_REPO / "src" / "so101_teleop"), str(_REPO / ".venv/lib/python3.12/site-packages")):
    if _p not in sys.path and Path(_p).is_dir():
        sys.path.insert(0, _p)

from so101_teleop.constants import JOINT_NAMES  # noqa: E402
from so101_teleop.teleoperators.leader import (  # noqa: E402
    JOINT_LIMITS,
    So101LeaderSimTeleop,
    So101LeaderSimTeleopConfig,
    deg_to_norm,
    grip_to_norm,
)


def main() -> int:
    ap = argparse.ArgumentParser(description="leader 映射探针 (只读)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="L07252801")
    ap.add_argument("--follower-calibration", default=None,
                    help="follower 标定 JSON (缺省自动发现)")
    ap.add_argument("--invert", default="", help="镜像方向关节, 逗号分隔")
    args = ap.parse_args()

    t = So101LeaderSimTeleop(
        So101LeaderSimTeleopConfig(
            port=args.port,
            id=args.id,
            follower_calibration=args.follower_calibration,
            invert=[j.strip() for j in args.invert.split(",") if j.strip()],
        )
    )
    t.connect()
    print("== leader 映射探针 (只读, Ctrl+C 退出) ==")
    print(f"{'关节':<14} {'deg/0-100':>10} {'rad':>9} {'norm':>8}")
    try:
        while True:
            raw = t._inner.get_action()
            print("-" * 46)
            for j in JOINT_NAMES:
                val = float(raw[f"{j}.pos"])
                lo, hi = JOINT_LIMITS[j]
                if j == "gripper":
                    norm = grip_to_norm(val, invert=j in t.config.invert)
                else:
                    norm = deg_to_norm(j, val, t._fcalib[j], invert=j in t.config.invert)
                rad = lo + (norm + 1) / 2.0 * (hi - lo)
                print(f"{j:<14} {val:>10.1f} {rad:>9.3f} {norm:>+8.3f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        t.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
