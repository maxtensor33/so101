"""SO101 RL 推理节点 (plan/rl.md Phase 4)。

加载训练好的 checkpoint (sb3/skrl 双后端), 自持 env 实例, 每 tick:
obs -> policy.predict -> env.step -> 发布 /joint_states、物体 TF、相机图像。
RViz 订阅显示 (纯可视化; MuJoCo 窗口可用 scripts/envs_view_env.py 另开对照)。
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
    for _p in (str(_REPO / "libs"), str(_REPO / ".venv/lib/python3.12/site-packages")):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, JointState
from tf2_ros import TransformBroadcaster

import mujoco
import numpy as np

import so101_envs  # noqa: F401  注册副作用
from so101_rl.algos import get_trainer
from so101_rl.backends.base import read_meta
from so101_rl.runs import resolve_ckpt_dir

ENV_ID = {
    "reach": "SO101Reach-v1",
    "lift": "SO101Lift-v1",
    "place": "SO101Place-v1",
    "stack": "SO101Stack-v1",
}
TASK_BODIES = {
    "reach": ["item", "goal_site"],
    "lift": ["item"],
    "place": ["item", "bin"],
    "stack": ["itemA", "itemB"],
}
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class RlInferNode(Node):
    def __init__(self):
        super().__init__("so101_rl_infer")
        self.declare_parameter("env", "reach")
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("camera", False)
        self.declare_parameter("rate", 10.0)
        env_name = str(self.get_parameter("env").value)
        # 支持最新 run 指针: checkpoint:=outputs/rl/<tag>/latest
        ckpt = resolve_ckpt_dir(Path(str(self.get_parameter("checkpoint").value)))
        camera = bool(self.get_parameter("camera").value)

        meta = read_meta(ckpt)
        if meta["env_id"] != ENV_ID[env_name]:
            self.get_logger().warn(
                f"checkpoint env ({meta['env_id']}) != 参数 env ({ENV_ID[env_name]}), 以 checkpoint 为准")
        self.get_logger().info(
            f"loading {meta['backend']}:{meta['algo']} from {ckpt} "
            f"(env={meta['env_id']}, normalize={meta['normalize']})")

        trainer = get_trainer(meta["backend"], meta["algo"])
        self.policy = trainer.load_policy(ckpt)
        self.env = trainer.make_env(meta["env_id"], seed=None)
        self.u = self.env.unwrapped
        self.obs, _ = self.env.reset(seed=0)
        self._qadr = self.u._qpos_adr
        self._item_bodies = [b for b in TASK_BODIES[env_name]
                             if mujoco.mj_name2id(self.u.model, mujoco.mjtObj.mjOBJ_BODY, b) >= 0]

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.tf_br = TransformBroadcaster(self)
        self.pub_imgs = {}
        if camera:
            for name in ("fixed", "wrist"):
                self.pub_imgs[name] = self.create_publisher(Image, f"rl/{name}/image", 10)

        rate = float(self.get_parameter("rate").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        action = self.policy(self.obs)
        self.obs, _, _, _, info = self.env.step(action)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = self.u.data.qpos[self._qadr].tolist()
        self.pub_js.publish(js)
        for body in self._item_bodies:
            bid = mujoco.mj_name2id(self.u.model, mujoco.mjtObj.mjOBJ_BODY, body)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
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
        for name, pub in self.pub_imgs.items():
            cam = "fixed_cam" if name == "fixed" else "wrist_cam"
            img = self.u._render_camera(cam)
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width = img.shape[:2]
            msg.encoding = "rgb8"
            msg.step = 3 * msg.width
            msg.data = img.tobytes()
            pub.publish(msg)
        if info.get("success"):
            self.get_logger().info("episode success!", throttle_duration_sec=5.0)

    def destroy_node(self):
        self.env.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RlInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
