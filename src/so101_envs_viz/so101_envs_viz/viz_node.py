"""SO101 envs 可视化节点 (workflow/plan/envs.md Phase 5)。

运行 gymnasium env (纯 MuJoCo 内核), 将状态发布到 ROS 供 RViz 显示:
- /joint_states (6 关节位置) -> robot_state_publisher -> RViz RobotModel
- 任务物体 TF (item/itemA/itemB/bin, 父 frame=world)
- camera=true 时发布 /envs/fixed/image 与 /envs/wrist/image

用法:
    ros2 run so101_envs_viz viz_node --ros-args -p env:=reach -p camera:=false
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
from sensor_msgs.msg import Image, JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import gymnasium as gym
import mujoco
import numpy as np

import so101_envs  # noqa: F401  注册副作用

ENV_ID = {
    ("reach", False): "SO101Reach-v1",
    ("reach", True): "SO101ReachCamera-v1",
    ("lift", False): "SO101Lift-v1",
    ("lift", True): "SO101LiftCamera-v1",
    ("place", False): "SO101Place-v1",
    ("place", True): "SO101PlaceCamera-v1",
    ("stack", False): "SO101Stack-v1",
    ("stack", True): "SO101StackCamera-v1",
    # can 变体（短名 = cube 短名 + "can" 后缀）
    ("reachcan", False): "SO101ReachCan-v1",
    ("reachcan", True): "SO101ReachCanCamera-v1",
    ("liftcan", False): "SO101LiftCan-v1",
    ("liftcan", True): "SO101LiftCanCamera-v1",
    ("placecan", False): "SO101PlaceCan-v1",
    ("placecan", True): "SO101PlaceCanCamera-v1",
    ("stackcan", False): "SO101StackCan-v1",
    ("stackcan", True): "SO101StackCanCamera-v1",
}
TASK_BODIES = {
    "reach": ["item", "goal_site"],
    "lift": ["item"],
    "place": ["item", "bin"],
    "stack": ["itemA", "itemB"],
    # can 变体（任务物体语义与 cube 相同）
    "reachcan": ["item", "goal_site"],
    "liftcan": ["item"],
    "placecan": ["item", "bin"],
    "stackcan": ["itemA", "itemB"],
}
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class EnvVizNode(Node):
    def __init__(self):
        super().__init__("so101_envs_viz")
        self.declare_parameter("env", "reach")
        self.declare_parameter("camera", False)
        env_name = self.get_parameter("env").value
        camera = bool(self.get_parameter("camera").value)
        env_id = ENV_ID[(env_name, camera)]
        self.get_logger().info(f"creating env {env_id}")

        self.env = gym.make(env_id)
        self.u = self.env.unwrapped
        self.env.reset(seed=0)
        for _ in range(20):  # settle
            self.env.step(np.zeros(6))

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.tf_br = TransformBroadcaster(self)
        self.pub_imgs = {}
        if camera:
            # Reliable: RViz2 Image 显示默认订阅 QoS 是 Reliable（BestEffort 发布端不兼容）
            for name in ("fixed", "wrist"):
                self.pub_imgs[name] = self.create_publisher(Image, f"envs/{name}/image", 10)

        self._qadr = self.u._qpos_adr
        self._item_bodies = [b for b in TASK_BODIES[env_name]
                             if mujoco.mj_name2id(self.u.model, mujoco.mjtObj.mjOBJ_BODY, b) >= 0]
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        *_, info = self.env.step(np.zeros(6))
        # joint_states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = self.u.data.qpos[self._qadr].tolist()
        self.pub_js.publish(js)
        # TF: 物体 (父 world)
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
        # 相机图像
        for name, pub in self.pub_imgs.items():
            cam = "fixed_cam" if name == "fixed" else "wrist_cam"
            cid = mujoco.mj_name2id(self.u.model, mujoco.mjtObj.mjOBJ_CAMERA, cam)
            img = self.u._render_camera(cam)
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width = img.shape[:2]
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = 3 * msg.width
            msg.data = img.tobytes()
            pub.publish(msg)

    def destroy_node(self):
        self.env.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EnvVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
