"""so101_teleop: lerobot Robot/Teleoperator 协议适配层（plan/il_sim.md）。

Robot 后端为 so101_envs 相机版仿真环境；il_real 阶段换真机 Robot 后端即可复用采集管线。
import 本包即完成 draccus ChoiceRegistry 注册（官方 CLI 依赖）。
"""

# ROS 部署（colcon install 树）下先自举 repo 路径（lerobot/so101_envs 在 uv venv 与 libs/），
# 必须在 .configs 导入前执行
import sys
from pathlib import Path


def _bootstrap_repo_paths():
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            repo = parent
            for p in (
                str(repo / "src" / "so101_teleop"),
                str(repo / "libs"),
                str(repo / ".venv" / "lib" / "python3.12" / "site-packages"),
                # pinocchio 经 cmeel.prefix 安装（.pth 机制, 手动 sys.path 不触发）
                str(repo / ".venv" / "lib" / "python3.12"
                    / "site-packages" / "cmeel.prefix" / "lib" / "python3.12" / "site-packages"),
            ):
                if Path(p).is_dir() and p not in sys.path:
                    sys.path.insert(0, p)
            return


_bootstrap_repo_paths()

from .configs import SimSO101RobotConfig  # noqa: E402
from .robot import SimSO101Robot  # noqa: E402
from . import teleoperators  # noqa: E402,F401  触发 Teleoperator 注册

__all__ = ["SimSO101Robot", "SimSO101RobotConfig"]
