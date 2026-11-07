"""so101_rl_dagger_infer 包测试 (workflow/plan/rl_dagger.md Phase 4, TDD)。

契约: DAgger student 仿真推理包 = ament_python 节点 + ROS 无关核心（core）。
core 不 import rclpy（进程内也不触发 ROS 导入）; 节点/launch/setup.py/package.xml
走结构断言（镜像 tests/test_il_sim_infer.py 与 test_il_sim_develop.py 的做法）。
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import gymnasium as gym
import numpy as np
import pytest
import torch

import so101_envs  # noqa: F401  注册副作用
from so101_rl.dagger import StudentPolicy, save_student_policy, stack_camera_images

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "src" / "so101_rl_dagger_infer"
_LAUNCH = PKG / "launch" / "rl_dagger_infer.launch.py"

if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

ENV_ID = "SO101ReachCamera-v1"
CAMERA_KEYS = ["fixed"]
# env_kwargs 原样落盘语义（scripts/rl_dagger_train.py build_env_kwargs 同款;
# camera_calib=None = 标定默认路径开启, 推理侧必须原样透传不得改写）
ENV_KWARGS = {
    "action_mode": "position",
    "domain_randomization": False,
    "camera": True,
    "render_size": [128, 128],
    "camera_calib": None,
}


@pytest.fixture(scope="module")
def tiny_ckpt(tmp_path_factory):
    """微型 student checkpoint（不训练）: 单相机 + env 默认分辨率。"""
    policy = StudentPolicy(n_cameras=1)  # in_hw 缺省 (128, 128) = env RENDER_SIZE
    policy.eval()
    ckpt = tmp_path_factory.mktemp("dagger_infer") / "tiny"
    save_student_policy(policy, ckpt, env_id=ENV_ID, env_kwargs=ENV_KWARGS,
                        hparams={"camera_keys": CAMERA_KEYS}, obs_dim=47)
    return ckpt


# ---------- 契约: 包结构 ----------


class TestPackageLayout:
    def test_package_xml(self):
        root = ET.parse(PKG / "package.xml").getroot()
        assert root.findtext("name") == "so101_rl_dagger_infer"
        assert root.findtext("export/build_type") == "ament_python"
        deps = {d.text for d in root.iter() if d.tag in ("depend", "exec_depend", "build_depend")}
        assert {"rclpy", "sensor_msgs", "tf2_ros", "robot_state_publisher",
                "so101_description"} <= deps

    def test_entry_point_and_wrapper(self):
        setup_py = (PKG / "setup.py").read_text()
        assert "rl_dagger_infer_node" in setup_py
        assert "so101_rl_dagger_infer.dagger_infer_node" in setup_py
        # setuptools>=80 把 console_scripts 装到 bin/, ros2 run 需要 lib/<pkg>/ wrapper（仓库既有坑）
        assert (PKG / "wrapper" / "rl_dagger_infer_node").is_file()
        assert "wrapper/rl_dagger_infer_node" in setup_py

    def test_node_skeleton(self):
        node = (PKG / "so101_rl_dagger_infer" / "dagger_infer_node.py").read_text()
        # 节点骨架: sys.path 自举 + 参数 + 自持 env + 10Hz tick + Enter 重置
        assert "_find_repo_root" in node
        for param in ("env", "checkpoint", "rate", "mujoco", "reset_on_enter"):
            assert param in node, f"节点缺参数 {param}"
        assert "gym.make" in node  # 自持 env（meta env_kwargs 原样透传）
        assert "make_obs_batch" in node  # 推理走 core 共用预处理链
        assert "rl_dagger/" in node  # 话题前缀
        assert "camera_keys" in node  # 按相机实际数量发布图像
        assert "TASK_BODIES" in node  # 物体 TF 表
        assert "JointState" in node and "TransformStamped" in node

    def test_core_has_no_ros_imports(self):
        core = (PKG / "so101_rl_dagger_infer" / "core.py").read_text()
        assert "rclpy" not in core
        assert "sensor_msgs" not in core
        assert "geometry_msgs" not in core

    def test_launch(self):
        assert _LAUNCH.is_file()
        launch = _LAUNCH.read_text()
        assert "OpaqueFunction" in launch
        assert '"so101_rl_dagger_infer"' in launch
        assert '"rl_dagger_infer_node"' in launch  # node executable
        assert "robot_state_publisher" in launch
        for arg in ("env", "checkpoint", "rate", "rviz", "mujoco", "reset_on_enter"):
            assert f'"{arg}"' in launch, f"launch 缺参数 {arg}"

    def test_rviz_config(self):
        # 与 il_sim_infer 同款: 包内 rviz 配置（Fixed Frame=world + 双相机 Image 话题）
        rviz = PKG / "rviz" / "rl_dagger_infer.rviz"
        assert rviz.is_file()
        cfg = rviz.read_text()
        assert "Fixed Frame: world" in cfg  # 非 bare rviz 默认的 map
        assert "/rl_dagger/fixed/image" in cfg
        assert "/rl_dagger/wrist/image" in cfg
        setup = (PKG / "setup.py").read_text()
        assert 'glob("rviz/*.rviz")' in setup  # data_files 注册
        assert "-d" in _LAUNCH.read_text()  # launch 以 -d 加载配置


# ---------- 契约: core（ROS 无关） ----------


def test_load_student(tiny_ckpt):
    from so101_rl_dagger_infer import camera_keys_from_meta, load_student

    policy, meta = load_student(tiny_ckpt)
    assert isinstance(policy, StudentPolicy)
    assert not policy.training  # eval 模式
    assert meta["backend"] == "dagger"
    assert meta["env_id"] == ENV_ID
    assert meta["env_kwargs"] == ENV_KWARGS
    assert camera_keys_from_meta(meta) == CAMERA_KEYS


def test_make_obs_batch_action_shape_and_range(tiny_ckpt):
    from so101_rl_dagger_infer import camera_keys_from_meta, load_student, make_obs_batch

    policy, meta = load_student(tiny_ckpt)
    # env_kwargs 原样透传（camera_calib=None 语义不得改写, 组 B 实测坑）
    env = gym.make(meta["env_id"], **meta["env_kwargs"])
    try:
        obs, _ = env.reset(seed=0)
        action = make_obs_batch(obs, camera_keys_from_meta(meta), policy)
        assert isinstance(action, np.ndarray)
        assert action.shape == (6,)
        assert np.all(action >= -1.0) and np.all(action <= 1.0)
        assert obs["rgb"]["fixed"].shape == (128, 128, 3)
    finally:
        env.close()


def test_make_obs_batch_matches_training_preprocess(tiny_ckpt):
    """推理侧必须走训练共用预处理: make_obs_batch ≡ stack_camera_images + images_to_tensor。"""
    from so101_rl.dagger import images_to_tensor
    from so101_rl_dagger_infer import camera_keys_from_meta, load_student, make_obs_batch

    policy, meta = load_student(tiny_ckpt)
    env = gym.make(meta["env_id"], **meta["env_kwargs"])
    try:
        obs, _ = env.reset(seed=1)
        keys = camera_keys_from_meta(meta)
        stacked = stack_camera_images({k: np.asarray(obs["rgb"][k])[None] for k in keys}, keys)
        with torch.no_grad():
            manual = policy(images_to_tensor(stacked))[0].numpy()
        action = make_obs_batch(obs, keys, policy)
        assert np.allclose(action, manual, atol=1e-6)
        # 确定性: 同 obs 两次推理一致
        assert np.allclose(action, make_obs_batch(obs, keys, policy), atol=1e-6)
    finally:
        env.close()


def test_make_obs_batch_two_cameras(tmp_path):
    """双相机 checkpoint: camera_keys 顺序决定输入布局（fixed, wrist）。"""
    from so101_rl_dagger_infer import camera_keys_from_meta, load_student, make_obs_batch

    policy = StudentPolicy(n_cameras=2)
    policy.eval()
    ckpt = tmp_path / "dual"
    save_student_policy(policy, ckpt, env_id=ENV_ID, env_kwargs=ENV_KWARGS,
                        hparams={"camera_keys": ["fixed", "wrist"]}, obs_dim=47)
    loaded, meta = load_student(ckpt)
    assert camera_keys_from_meta(meta) == ["fixed", "wrist"]
    env = gym.make(meta["env_id"], **meta["env_kwargs"])
    try:
        obs, _ = env.reset(seed=2)
        action = make_obs_batch(obs, ["fixed", "wrist"], loaded)
        assert action.shape == (6,)
        # 交换相机顺序必须改变输出（顺序即网络输入顺序）
        action_swapped = make_obs_batch(obs, ["wrist", "fixed"], loaded)
        assert not np.allclose(action, action_swapped)
    finally:
        env.close()
