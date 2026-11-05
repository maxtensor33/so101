"""so101_il_sim_develop 包测试 (workflow/plan/il_sim_develop.md Phase 2, TDD)。

契约: 实机 IL 部署包 = ament_python 节点 + ROS 无关推理核心（real_infer）。
命名约定: 包名 il_sim_develop, il_real 命名空间保留给后续正式实机包。
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import torch

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "src" / "so101_il_sim_develop"
_LAUNCH = PKG / "launch" / "il_sim_develop.launch.py"

if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from so101_il_sim_develop.real_infer import (  # noqa: E402
    JOINT_LOWER,
    JOINT_UPPER,
    EstopLatch,
    build_obs_batch,
    clamp_target,
    normalize_qpos,
    plan_to_rest,
    resize_to_policy,
)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


# ---------- 契约: 包结构 ----------


class TestPackageLayout:
    def test_package_xml(self):
        root = ET.parse(PKG / "package.xml").getroot()
        assert root.findtext("name") == "so101_il_sim_develop"
        assert root.findtext("export/build_type") == "ament_python"
        deps = {d.text for d in root.iter() if d.tag in ("depend", "exec_depend", "build_depend")}
        assert {"rclpy", "sensor_msgs", "std_msgs", "trajectory_msgs"} <= deps

    def test_entry_point(self):
        setup_py = (PKG / "setup.py").read_text()
        assert "il_sim_develop_node" in setup_py
        assert "il_sim_camera_node" in setup_py
        assert "so101_il_sim_develop.il_sim_develop_node" in setup_py
        # setuptools>=80 把 console_scripts 装到 bin/, ros2 run 需要 lib/<pkg>/ wrapper（仓库既有坑）
        assert (PKG / "wrapper" / "il_sim_develop_node").is_file()
        assert (PKG / "wrapper" / "il_sim_camera_node").is_file()
        assert 'glob("wrapper/*")' in setup_py

    def test_node_exists_and_reuses_core_loader(self):
        node = (PKG / "so101_il_sim_develop" / "il_sim_develop_node.py").read_text()
        # 复用 il_sim_infer 推理核心（load_policy 三元组）
        assert "load_policy" in node
        assert "so101_il_sim_infer" in node
        # estop 订阅 + confirm_step + reset_on_enter 参数齐备
        for param in ("estop", "confirm_step", "reset_on_enter", "camera_keys"):
            assert param in node, f"节点缺参数 {param}"

    def test_launch_and_rviz(self):
        assert _LAUNCH.is_file()
        launch = _LAUNCH.read_text()
        # 内置 cv2 相机节点 ×2 + 控制底座 real:=true + rviz 开关
        assert "camera_node" in launch
        assert '"real"' in launch and '"true"' in launch
        assert "rviz" in launch
        assert (PKG / "config" / "il_sim_develop.rviz").is_file()
        assert (PKG / "so101_il_sim_develop" / "camera_node.py").is_file()

    def test_rviz_layout_follows_envs_dual_camera(self):
        """RViz 布局沿用 envs 双相机版（so101_envs_viz/rviz/env_viz.rviz）:
        QMainWindow 停靠浮窗布局 + Orbit 视角, 仅 Image 话题换成本包 /il_sim_develop/*。"""
        cfg = (PKG / "config" / "il_sim_develop.rviz").read_text()
        envs_cfg = (REPO / "src" / "so101_envs_viz" / "rviz" / "env_viz.rviz").read_text()
        assert "Value: /il_sim_develop/fixed/image" in cfg
        assert "Value: /il_sim_develop/wrist/image" in cfg
        assert "/envs/" not in cfg
        # 布局特征与 envs 版一致（QMainWindow 停靠态 + Orbit 视角参数）
        for marker in ("QMainWindow State:", "Class: rviz_default_plugins/Orbit", "FixedCamera", "WristCamera"):
            assert marker in cfg, f"rviz 配置缺 envs 版布局特征 {marker}"
        envs_markers = [l.strip() for l in envs_cfg.splitlines() if "Yaw:" in l or "Pitch:" in l]
        for marker in envs_markers:
            assert marker in cfg, f"rviz 视角参数与 envs 版不一致: {marker}"

    def test_camera_key_mapping(self):
        """top 物理相机 → 训练 key 'fixed'（世界系俯视语义）, wrist → 'wrist'。

        设备映射机器相关（随 USB 插入顺序漂移）, 必须走 config/cameras.yaml 按节点名配置,
        launch 不硬编码 /dev/videoN。
        """
        launch = _LAUNCH.read_text()
        assert "fixed" in launch and "wrist" in launch
        assert "cameras.yaml" in launch
        assert "/dev/video" not in launch  # 设备路径不得硬编码
        yaml_text = (PKG / "config" / "cameras.yaml").read_text()
        assert "il_sim_cam_fixed" in yaml_text and "il_sim_cam_wrist" in yaml_text
        assert "device:" in yaml_text


# ---------- 单测: real_infer 纯函数 ----------


class TestNormalizeQpos:
    def test_boundaries(self):
        q = np.stack([JOINT_LOWER, JOINT_UPPER])  # (2,6): 两端极限位形
        n = normalize_qpos(q)
        assert np.allclose(n[0], -1.0, atol=1e-6)
        assert np.allclose(n[1], 1.0, atol=1e-6)

    def test_midpoint(self):
        mid = (JOINT_LOWER + JOINT_UPPER) / 2
        assert np.allclose(normalize_qpos(mid), 0.0, atol=1e-6)

    def test_shape_and_range(self):
        rng = np.random.default_rng(0)
        q = JOINT_LOWER + rng.random(6) * (JOINT_UPPER - JOINT_LOWER)
        n = normalize_qpos(q)
        assert n.shape == (6,)
        assert np.all(n >= -1.0 - 1e-9) and np.all(n <= 1.0 + 1e-9)


class TestBuildObsBatch:
    def _img(self, v=128):
        return np.full((128, 128, 3), v, dtype=np.uint8)

    def test_keys_and_layout(self):
        batch = build_obs_batch(
            np.zeros(6), {"fixed": self._img(0), "wrist": self._img(255)}, ("fixed", "wrist")
        )
        assert set(batch) == {"observation.state", "observation.images.fixed", "observation.images.wrist"}
        assert batch["observation.state"].shape == (6,)
        assert batch["observation.state"].dtype == torch.float32
        # 图像 CHW float /255
        img = batch["observation.images.fixed"]
        assert img.shape == (3, 128, 128)
        assert img.dtype == torch.float32 and img.max() == 0.0
        assert batch["observation.images.wrist"].max() == pytest.approx(1.0)

    def test_state_normalized(self):
        batch = build_obs_batch(JOINT_UPPER.copy(), {}, ())
        assert np.allclose(batch["observation.state"].numpy(), 1.0, atol=1e-6)

    def test_camera_subset(self):
        """VQ-BeT 单相机: camera_keys=("fixed",) 时只喂 fixed。"""
        batch = build_obs_batch(
            np.zeros(6), {"fixed": self._img(), "wrist": self._img()}, ("fixed",)
        )
        assert set(batch) == {"observation.state", "observation.images.fixed"}

    def test_missing_camera_raises(self):
        with pytest.raises(KeyError):
            build_obs_batch(np.zeros(6), {"fixed": self._img()}, ("fixed", "wrist"))


class TestClampTarget:
    def test_delta_clamped(self):
        prev = np.zeros(6)
        raw = prev + 1.0  # 远超单步上限
        out = clamp_target(raw, prev, max_delta=0.1)
        assert np.allclose(out, 0.1)

    def test_absolute_bounds(self):
        prev = JOINT_UPPER - 0.05
        raw = prev + 1.0
        out = clamp_target(raw, prev, max_delta=0.1)
        assert np.all(out <= JOINT_UPPER + 1e-9)

    def test_within_delta_unchanged(self):
        prev = np.full(6, 0.5)
        raw = prev + 0.05
        assert np.allclose(clamp_target(raw, prev, max_delta=0.1), raw)


class TestPlanToRest:
    def test_bounded_steps(self):
        cur = np.full(6, 1.0)
        rest = np.zeros(6)
        pos = cur.copy()
        for _ in range(1000):
            pos = plan_to_rest(pos, rest, max_step=0.025)
            if np.allclose(pos, rest, atol=1e-9):
                break
        assert np.allclose(pos, rest, atol=1e-9)

    def test_step_never_exceeds(self):
        pos = np.full(6, 1.0)
        nxt = plan_to_rest(pos, np.zeros(6), max_step=0.025)
        assert np.allclose(nxt, 1.0 - 0.025)


class TestResizeToPolicy:
    def test_center_crop_square_then_resize(self):
        """640x480 → 中心裁方 480x480 → 128x128（squint 同款预处理）。"""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:, 80:560] = 255  # 中心区
        out = resize_to_policy(img, 128)
        assert out.shape == (128, 128, 3)
        assert out.dtype == np.uint8
        # 裁方后无黑边（全图来自原中心区）
        assert out.min() == 255

    def test_already_square_passthrough_shape(self):
        img = np.full((128, 128, 3), 7, dtype=np.uint8)
        assert resize_to_policy(img, 128).shape == (128, 128, 3)

    def test_downscale_preserves_content(self):
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        img[:128] = 255  # 上半白
        out = resize_to_policy(img, 128)
        assert out[0, 0].max() == 255 and out[-1, -1].max() == 0


class TestEstopLatch:
    def test_initially_clear(self):
        latch = EstopLatch()
        assert not latch.triggered

    def test_trigger_and_hold(self):
        latch = EstopLatch()
        latch.trigger()
        assert latch.triggered
        latch.clear()
        assert not latch.triggered
