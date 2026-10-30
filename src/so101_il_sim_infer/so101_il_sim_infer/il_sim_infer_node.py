"""SO101 il_sim IL 推理节点（workflow/plan/il_sim.md Phase 5）。

加载 lerobot policy checkpoint（ACT/DP/VQ-BeT 通用加载链路），自持相机版 env，
10Hz tick: obs → preprocess → policy.select_action → postprocess → env.step → ROS 话题
（/joint_states、物体 TF、/il/{fixed,wrist}/image）。RViz 订阅显示（MuJoCo 与 RViz 同步）。
"""

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
        str(_REPO / "libs"),
        str(_REPO / ".venv" / "lib" / "python3.12" / "site-packages"),
        str(_REPO / ".venv" / "lib" / "python3.12"
            / "site-packages" / "cmeel.prefix" / "lib" / "python3.12" / "site-packages"),
    ):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, JointState
from tf2_ros import TransformBroadcaster

import mujoco
import torch
import gymnasium as gym
import so101_envs  # noqa: F401  gym 注册副作用
from lerobot.policies.pretrained import PreTrainedPolicy  # noqa: F401

from so101_teleop.constants import JOINT_NAMES
from so101_teleop.cli_config import resolve_env_id
ENV_ID = {
    "reach": "SO101ReachCamera-v1",
    "lift": "SO101LiftCamera-v1",
    "place": "SO101PlaceCamera-v1",
    "stack": "SO101StackCamera-v1",
    # can 变体（短名 = cube 短名 + "can" 后缀）
    "reachcan": "SO101ReachCanCamera-v1",
    "liftcan": "SO101LiftCanCamera-v1",
    "placecan": "SO101PlaceCanCamera-v1",
    "stackcan": "SO101StackCanCamera-v1",
}

TASK_BODIES = {
    "SO101ReachEnv": ["item", "goal_site"],
    "SO101LiftEnv": ["item"],
    "SO101PlaceEnv": ["item", "goal_site"],
    "SO101StackEnv": ["item", "box"],
    # can 变体（镜像对应 cube 的 TF 物体列表; 消费点有缺省过滤）
    "SO101ReachCanEnv": ["item", "goal_site"],
    "SO101LiftCanEnv": ["item"],
    "SO101PlaceCanEnv": ["item", "goal_site"],
    "SO101StackCanEnv": ["item", "box"],
}


class IlSimInferNode(Node):
    def __init__(self):
        super().__init__("so101_il_sim_infer")
        self.declare_parameter("env", "reach")
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("camera", True)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("mujoco", True)
        env_name = str(self.get_parameter("env").value)
        ckpt = Path(str(self.get_parameter("checkpoint").value)).expanduser()
        camera = bool(self.get_parameter("camera").value)

        from so101_il_sim_infer.core import load_policy, make_obs_batch

        self.get_logger().info(f"loading IL policy from {ckpt} (env=reach)")
        self._make_obs_batch = make_obs_batch
        self.policy, self._pre, self._post = load_policy(ckpt)  # 设备在 load_policy 内对齐

        render_mode = "human" if bool(self.get_parameter("mujoco").value) else None
        self.env = gym.make(
            ENV_ID[env_name],
            render_mode=render_mode,
            viewer_ui=False,       # 默认隐藏左右工具栏(Tab/Shift+Tab 可切)
            viewer_maximize=True,  # X11 EWMH 最大化(best-effort)
        )
        self.u = self.env.unwrapped
        self._obs, _ = self.env.reset(seed=0)
        self._qadr = self.u._qpos_adr
        self._bodies = [
            b for b in TASK_BODIES.get(type(self.u).__name__, ["item"])
            if mujoco.mj_name2id(self.u.model, mujoco.mjtObj.mjOBJ_BODY, b) >= 0
        ]

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.tf_br = TransformBroadcaster(self)
        self.pub_imgs = {}
        if camera:
            for cam in self._obs["rgb"]:
                self.pub_imgs[cam] = self.create_publisher(Image, f"il/{cam}/image", 10)
        rate = float(self.get_parameter("rate").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self._success_logged = False

    def tick(self):
        import numpy as np

        batch = self._make_obs_batch(self._obs, u=self.u)
        with torch.no_grad():
            action = self._post(self.policy.select_action(self._pre(batch)))
        arr = action.squeeze(0).cpu().numpy() if action.ndim > 1 else action.cpu().numpy()
        self._obs, *_ = self.env.step(np.clip(arr, -1.0, 1.0))

        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        js.name = JOINT_NAMES
        js.position = self.u.data.qpos[self._qadr].tolist()
        self.pub_js.publish(js)
        for body in self._bodies:
            bid = mujoco.mj_name2id(self.u.model, mujoco.mjtObj.mjOBJ_BODY, body)
            if bid < 0:
                continue
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "world"
            t.child_frame_id = body
            t.transform.translation.x = float(self.u.data.xpos[bid][0])
            t.transform.translation.y = float(self.u.data.xpos[bid][1])
            t.transform.translation.z = float(self.u.data.xpos[bid][2])
            q = self.u.data.xquat[bid]
            t.transform.rotation.w = float(q[0])
            t.transform.rotation.x = float(q[1])
            t.transform.rotation.y = float(q[2])
            t.transform.rotation.z = float(q[3])
            self.tf_br.sendTransform(t)
        for cam, pub in self.pub_imgs.items():
            img = self._obs["rgb"][cam]
            msg = Image()
            msg.header.stamp = now
            msg.height, msg.width = img.shape[:2]
            msg.encoding = "rgb8"
            msg.step = 3 * msg.width
            msg.data = img.tobytes()
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IlSimInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.env.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
