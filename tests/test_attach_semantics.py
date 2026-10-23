"""attach 语义 spike 的 TDD 测试 (plan/mtc_fix.md R3/R4 mujoco 侧)。

scripts/mtc_attach_semantics.py 镜像 grasp_attach_plugin C++ 语义 (纯 mujoco,
无 ROS), ideal_rel_pose 从 controllers_pick.yaml 读出货值。

R3 验收: cube 中心与两指面中点偏差 < 5mm、无穿透。
R4 验收 (落点 3cm) 在 mujoco 端到端脚本运行中测 (耗时, 不进 pytest)。
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
YAML_CFG = REPO / "src" / "so101_mtc_pick_place" / "config" / "controllers_pick.yaml"

PICK_CUBE = np.array([0.117, -0.077, 0.01])
PLACE_CUBE = np.array([0.125, 0.063, 0.01])

sys.path.insert(0, str(REPO / "libs"))


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "mtc_attach_semantics", str(REPO / "scripts" / "mtc_attach_semantics.py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["mtc_attach_semantics"] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def plugin_params():
    """从 controllers_pick.yaml 读 grasp_attach 插件货值参数。"""
    import yaml
    cfg = yaml.safe_load(YAML_CFG.read_text())
    return cfg["/**"]["ros__parameters"]["mujoco_plugins"]["grasp_attach"]


@pytest.fixture(scope="module")
def run(plugin_params):
    """完整 pick->place 仿真日志 (模块级共享, 只跑 TDD 循环一次)。"""
    mod = _load_module()
    return mod.run_pick_place(ideal_rel_pose=plugin_params["ideal_rel_pose"])


class TestIdealAttach:
    def test_attach_happens(self, run):
        """close 阶段触发 kinematic attach + 释放, rel 采样序列非空。"""
        assert run["attach_time"] is not None
        assert run["release_time"] is not None
        assert len(run["rel_samples"]) >= 2

    def test_cube_centered_between_fingers(self, run):
        """R3 验收: 抓稳后 cube 中心与两指面中点偏差 < 5mm。"""
        assert run["center_error"] < 3e-3

    def test_visual_clearance_positive(self, run):
        """R3 验收: 视觉网格间隙 > 0 (两指面间距 - cube 沿闭合轴宽度)。

        注: mujoco mesh 碰撞用凸包 (含掌部连接臂, 向指间隙鼓出), 凸包接触
        穿透是已知超近似; 渲染网格 (视觉真实) 以面间距度量, 见 TDD 报告。
        """
        assert run["visual_clearance"] > 0.0


class TestReleaseAndPlace:
    def test_cube_dropped_on_release(self, run):
        """释放后 cube 落地 (z 回到半高 ~10mm, 而非悬浮在携带高度)。"""
        assert run["final_z"] == pytest.approx(0.01, abs=0.003)

    def test_place_accuracy_3cm(self, run):
        """R4 验收: 落点距 place 目标 < 3cm (旧实现 5~7cm)。"""
        err = np.hypot(run["final_xy"][0] - PLACE_CUBE[0],
                       run["final_xy"][1] - PLACE_CUBE[1])
        assert err < 0.03
