"""ros2 run 入口：bootstraps repo 路径后调 scripts/il_sim_teleop.main()。"""

import sys
from pathlib import Path


def _find_repo_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    return None


_REPO = _find_repo_root()
if _REPO is not None:
    for _p in (
        str(_REPO / "src" / "so101_teleop"),
        str(_REPO / "libs"),
        str(_REPO / ".venv" / "lib" / "python3.12" / "site-packages"),
    ):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)


def main():
    repo = _find_repo_root()
    if repo is None:
        raise SystemExit("repo root (pyproject.toml) not found")
    sys.path.insert(0, str(repo / "scripts"))
    import importlib

    mod = importlib.import_module("il_sim_teleop")
    mod.main()


if __name__ == "__main__":
    main()
