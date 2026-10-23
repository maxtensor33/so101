"""so101_il_sim_infer: lerobot IL policy ROS2 推理包（ROS 无关核心 + 节点）。"""

# ROS 部署（colcon install 树）下先自举 repo 路径（torch/lerobot 在 uv venv），
# 必须在 .core 导入前执行
import sys
from pathlib import Path


def _bootstrap_repo_paths():
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            repo = parent
            for p in (
                str(repo / "libs"),
                str(repo / ".venv" / "lib" / "python3.12" / "site-packages"),
                str(repo / ".venv" / "lib" / "python3.12"
                    / "site-packages" / "cmeel.prefix" / "lib" / "python3.12" / "site-packages"),
            ):
                if Path(p).is_dir() and p not in sys.path:
                    sys.path.insert(0, p)
            return


_bootstrap_repo_paths()

from .core import load_policy, make_obs_batch  # noqa: E402

__all__ = ["load_policy", "make_obs_batch"]
