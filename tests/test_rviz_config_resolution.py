"""draw_cube.launch.py 的 rviz 配置路径解析测试。

背景: ament_python 的 data_files 在 colcon --symlink-install 下仍是拷贝,
RViz 里 File->Save 会写到 install 拷贝并被下次 build 覆盖。
修复: dev 工作区内 launch 直接加载 src 源文件 (保存即落盘源头),
无源码环境回退 install 拷贝。
"""

import importlib.util
import sys
from pathlib import Path

import pytest

LAUNCH = (Path(__file__).resolve().parents[1] / "src" / "so101_robotics_torque_app"
          / "launch" / "draw_cube.launch.py")
REPO_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("draw_cube_launch", LAUNCH)
    m = importlib.util.module_from_spec(spec)
    sys.modules["draw_cube_launch"] = m
    spec.loader.exec_module(m)
    return m


class TestRvizConfigResolution:
    def test_dev_workspace_returns_source_path(self, mod):
        """给 repo root 时必须返回 src 下的 rviz 源文件。"""
        p = mod.resolve_rviz_config(REPO_ROOT)
        assert p == REPO_ROOT / "src" / "so101_robotics_torque_app" / "rviz" / "draw_cube.rviz"
        assert p.is_file()

    def test_source_path_is_live_editable(self, mod):
        """src 文件与 install 拷贝是不同 inode (保存到 src 不会被 build 覆盖逻辑管理)。"""
        src = mod.resolve_rviz_config(REPO_ROOT)
        assert src.resolve().is_relative_to(REPO_ROOT / "src")

    def test_fallback_without_sources(self, mod, monkeypatch):
        """repo root 不存在 (纯安装环境) 时回退 package share 路径。"""
        monkeypatch.setattr(
            mod, "get_package_share_directory",
            lambda name: "/tmp/fake_prefix/share/" + name)
        p = mod.resolve_rviz_config(Path("/nonexistent/repo"))
        assert p == Path("/tmp/fake_prefix/share/so101_robotics_torque_app/rviz/draw_cube.rviz")

    def test_find_repo_root_in_this_workspace(self, mod):
        """本工作区 (从 tests 运行) 向上能找到 pyproject.toml 所在根。"""
        assert mod._find_repo_root() == REPO_ROOT
