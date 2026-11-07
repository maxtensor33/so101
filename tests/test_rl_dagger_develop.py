"""so101_rl_dagger_develop 包测试 (workflow/plan/rl_dagger.md Phase 5, TDD)。

契约: DAgger student 实机部署包 = ament_python 节点 + ROS 无关推理核心（real_infer）。
镜像 tests/test_il_sim_develop.py 的做法: real_infer 单测不 import rclpy;
节点/launch/cameras.yaml 走结构断言。core 复用 so101_rl_dagger_infer（sys.path 自举,
包 __init__ 已 re-export load_student / camera_keys_from_meta / make_obs_batch）。
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from so101_rl.dagger import StudentPolicy, save_student_policy

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "src" / "so101_rl_dagger_develop"
_LAUNCH = PKG / "launch" / "rl_dagger_develop.launch.py"

for _p in (str(PKG), str(REPO / "src" / "so101_rl_dagger_infer")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from so101_rl_dagger_develop.real_infer import (  # noqa: E402
    JOINT_LOWER,
    JOINT_UPPER,
    EstopLatch,
    clamp_target,
    denormalize_action,
    frame_to_policy_input,
    make_env_like_obs,
    plan_to_rest,
)

# env_kwargs 原样落盘语义（scripts/rl_dagger_train.py 同款; 实机不重建 env,
# 但 action_mode 需读出用于动作语义注释）
ENV_KWARGS = {
    "action_mode": "position",
    "domain_randomization": False,
}


@pytest.fixture(scope="module")
def tiny_ckpt(tmp_path_factory):
    """微型 student checkpoint（不训练）: 单相机 + 小分辨率 in_hw（省时）。"""
    policy = StudentPolicy(n_cameras=1, in_hw=(64, 64))
    policy.eval()
    ckpt = tmp_path_factory.mktemp("dagger_develop") / "tiny"
    save_student_policy(policy, ckpt, env_id="SO101ReachCamera-v1",
                        env_kwargs=ENV_KWARGS, hparams={"camera_keys": ["fixed"]},
                        obs_dim=47)
    return ckpt


# ---------- 契约: 包结构 ----------


class TestPackageLayout:
    def test_package_xml(self):
        root = ET.parse(PKG / "package.xml").getroot()
        assert root.findtext("name") == "so101_rl_dagger_develop"
        assert root.findtext("export/build_type") == "ament_python"
        deps = {d.text for d in root.iter() if d.tag in ("depend", "exec_depend", "build_depend")}
        assert {"rclpy", "sensor_msgs", "std_msgs", "trajectory_msgs"} <= deps

    def test_entry_point_and_wrapper(self):
        setup_py = (PKG / "setup.py").read_text()
        assert "rl_dagger_develop_node" in setup_py
        assert "rl_dagger_camera_node" in setup_py
        assert "so101_rl_dagger_develop.dagger_develop_node" in setup_py
        assert "so101_rl_dagger_develop.dagger_camera_node" in setup_py
        # setuptools>=80 把 console_scripts 装到 bin/, ros2 run 需要 lib/<pkg>/ wrapper（仓库既有坑）
        assert (PKG / "wrapper" / "rl_dagger_develop_node").is_file()
        assert (PKG / "wrapper" / "rl_dagger_camera_node").is_file()
        assert 'glob("wrapper/*")' in setup_py

    def test_node_skeleton(self):
        node = (PKG / "so101_rl_dagger_develop" / "dagger_develop_node.py").read_text()
        # 节点骨架: sys.path 自举 + 复用 rl_dagger_infer core + 状态机 + estop + confirm
        assert "_find_repo_root" in node
        assert "so101_rl_dagger_infer" in node
        for name in ("load_student", "camera_keys_from_meta", "make_obs_batch",
                     "make_env_like_obs", "frame_to_policy_input"):
            assert name in node, f"节点缺核心调用 {name}"
        for param in ("checkpoint", "confirm_step", "max_delta", "rate", "reset_on_enter"):
            assert param in node, f"节点缺参数 {param}"
        # 状态机 run|homing|confirm_wait + estop 订阅 + 轨迹发布
        for marker in ('"run"', "homing", "confirm_wait", "rl_dagger_develop/estop",
                       "/real/", "/arm_controller/joint_trajectory",
                       "/gripper_controller/joint_trajectory", "action_mode"):
            assert marker in node, f"节点缺关键词 {marker}"

    def test_real_infer_no_ros_imports(self):
        core = (PKG / "so101_rl_dagger_develop" / "real_infer.py").read_text()
        assert "rclpy" not in core
        assert "sensor_msgs" not in core

    def test_camera_node_exists(self):
        cam = (PKG / "so101_rl_dagger_develop" / "dagger_camera_node.py").read_text()
        assert "_find_repo_root" in cam
        for param in ("camera_key", "device", "fps"):
            assert param in cam, f"相机节点缺参数 {param}"
        assert "/real/" in cam and "image_raw" in cam

    def test_launch(self):
        assert _LAUNCH.is_file()
        launch = _LAUNCH.read_text()
        # 控制底座 real:=true + 内置 cv2 相机节点 + develop 节点 + cameras yaml 优先级
        assert "demo.launch.py" in launch
        assert '"real"' in launch and '"true"' in launch
        assert '"so101_rl_dagger_develop"' in launch
        assert '"rl_dagger_develop_node"' in launch
        assert '"rl_dagger_camera_node"' in launch
        assert "cameras.local.yaml" in launch and "cameras.yaml" in launch
        for arg in ("checkpoint", "confirm_step", "reset_on_enter", "rate", "max_delta",
                    "port", "calibration_file", "cam_fps"):
            assert f'"{arg}"' in launch, f"launch 缺参数 {arg}"
        assert "/dev/video" not in launch  # 设备路径不得硬编码

    def test_cameras_yaml_structure(self):
        yaml_text = (PKG / "config" / "cameras.yaml").read_text()
        assert "cameras.local.yaml" in yaml_text  # local 优先说明注释
        data = yaml.safe_load(yaml_text)
        assert set(data) == {"rl_dagger_cam_fixed", "rl_dagger_cam_wrist"}
        for node_cfg in data.values():
            assert "device" in node_cfg["ros__parameters"]


# ---------- 单测: real_infer 纯函数 ----------


class TestDenormalizeAction:
    def test_boundaries(self):
        assert np.allclose(denormalize_action(-1.0), JOINT_LOWER, atol=1e-9)
        assert np.allclose(denormalize_action(1.0), JOINT_UPPER, atol=1e-9)

    def test_midpoint(self):
        mid = (JOINT_LOWER + JOINT_UPPER) / 2
        assert np.allclose(denormalize_action(0.0), mid, atol=1e-9)

    def test_input_clipped(self):
        out = denormalize_action(np.array([2.0, -2.0, 0.0, 0.0, 0.0, 0.0]))
        assert out[0] == pytest.approx(JOINT_UPPER[0])
        assert out[1] == pytest.approx(JOINT_LOWER[1])


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


class TestFrameToPolicyInput:
    def test_bgr_to_rgb(self):
        """cv2 帧为 BGR: 红色帧 (BGR ch2=255) -> 输出 RGB ch0=255。"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[..., 2] = 255
        out = frame_to_policy_input(frame, (480, 640))
        assert out.shape == (480, 640, 3)
        assert out.dtype == np.uint8
        assert (out[..., 0] == 255).all()
        assert out[..., 1].max() == 0 and out[..., 2].max() == 0

    def test_default_in_hw_center_crop_and_resize(self):
        """640x480 相机帧 + 缺省 in_hw (128,128) = env RENDER_SIZE:
        中心裁方到 480x480 再 resize 到 128x128（实机分辨率不变, resize 属预期）。"""
        frame = np.full((480, 640, 3), 7, dtype=np.uint8)
        out = frame_to_policy_input(frame)
        assert out.shape == (128, 128, 3)
        assert out.dtype == np.uint8

    def test_center_crop_square_then_resize(self):
        """非 policy 分辨率: 中心裁方 -> resize 到 in_hw（il_sim_develop resize_to_policy 惯例）。"""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:, 80:560] = 255  # 中心区
        out = frame_to_policy_input(img, (64, 64))
        assert out.shape == (64, 64, 3)
        assert out.dtype == np.uint8
        # 裁方后无黑边（全图来自原中心区）
        assert out.min() == 255

    def test_larger_frame_center_crop(self):
        frame = np.zeros((960, 640, 3), dtype=np.uint8)
        frame[160:800] = 200  # 中心裁方将取 rows 160..800
        out = frame_to_policy_input(frame, (480, 640))
        assert out.shape == (480, 640, 3)
        assert out.min() == 200


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


# ---------- 端到端: make_env_like_obs + rl_dagger_infer core ----------


class TestMakeEnvLikeObs:
    def test_structure(self):
        frames = {"fixed": np.zeros((64, 64, 3), dtype=np.uint8)}
        obs = make_env_like_obs(frames)
        assert obs == {"rgb": frames}
        assert set(obs["rgb"]) == {"fixed"}


def test_end_to_end_single_camera(tiny_ckpt):
    """实机链路: cv2 帧 -> frame_to_policy_input -> make_env_like_obs
    -> core.make_obs_batch -> action (6,) ∈ [-1,1]（student 纯视觉, 不消费 state）。"""
    from so101_rl_dagger_infer import camera_keys_from_meta, load_student, make_obs_batch

    policy, meta = load_student(tiny_ckpt)
    assert meta["backend"] == "dagger"
    assert meta["env_kwargs"]["action_mode"] == "position"  # 动作语义注释来源
    keys = camera_keys_from_meta(meta)
    assert keys == ["fixed"]
    in_hw = tuple(meta["hparams"]["in_hw"])
    frame = np.zeros((480, 640, 3), dtype=np.uint8)  # cv2 相机原始分辨率
    imgs = {k: frame_to_policy_input(frame, in_hw) for k in keys}
    action = make_obs_batch(make_env_like_obs(imgs), keys, policy)
    assert isinstance(action, np.ndarray)
    assert action.shape == (6,)
    assert np.all(action >= -1.0) and np.all(action <= 1.0)


def test_end_to_end_two_cameras(tmp_path):
    """双相机 checkpoint: camera_keys 顺序决定输入布局（fixed, wrist）。"""
    from so101_rl_dagger_infer import camera_keys_from_meta, load_student, make_obs_batch

    policy = StudentPolicy(n_cameras=2, in_hw=(64, 64))
    policy.eval()
    ckpt = tmp_path / "dual"
    save_student_policy(policy, ckpt, env_id="SO101ReachCamera-v1",
                        env_kwargs=ENV_KWARGS, hparams={"camera_keys": ["fixed", "wrist"]},
                        obs_dim=47)
    loaded, meta = load_student(ckpt)
    assert camera_keys_from_meta(meta) == ["fixed", "wrist"]
    frames = {
        "fixed": np.zeros((480, 640, 3), dtype=np.uint8),
        "wrist": np.full((480, 640, 3), 128, dtype=np.uint8),
    }
    imgs = {k: frame_to_policy_input(v, (64, 64)) for k, v in frames.items()}
    action = make_obs_batch(make_env_like_obs(imgs), ["fixed", "wrist"], loaded)
    assert action.shape == (6,)
    assert np.all(action >= -1.0) and np.all(action <= 1.0)
    # 交换相机顺序必须改变输出（顺序即网络输入顺序）
    action_swapped = make_obs_batch(make_env_like_obs(imgs), ["wrist", "fixed"], loaded)
    assert not np.allclose(action, action_swapped)
