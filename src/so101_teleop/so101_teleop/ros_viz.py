"""ros_viz：采集/遥操期间 RViz 同步显示（workflow/plan/il_sim.md Phase 3, 内置启用）。

守护线程 ROS 节点：/joint_states、物体/goal TF、/il/{fixed,wrist}/image。
数据源 = env_backend 共享 env；rclpy 惰性导入（无 ROS 时 teleop/record 照常工作）。
"""

from __future__ import annotations

import threading

from .constants import JOINT_NAMES
from .env_backend import latest_images


class RosVizThread:
    def __init__(self, rate: float = 10.0):
        self._rate = rate
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._started = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._started.wait(timeout=10.0)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    # ---------- internals ----------

    def _run(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import TransformStamped
            from sensor_msgs.msg import Image, JointState
            from tf2_ros import TransformBroadcaster
        except Exception as e:  # 无 ROS 环境: viz 静默跳过
            print(f"[ros_viz] rclpy unavailable ({e}); viz disabled")
            self._started.set()
            return

        import time

        from . import env_backend

        rclpy.init(args=None)
        # 等 robot.connect() 注册 env（record/teleoperate 内部才连接）
        deadline = time.monotonic() + 60.0
        while env_backend.get_active_env_or_none() is None:
            if self._stop.is_set() or time.monotonic() > deadline:
                self._started.set()
                node_destroy_and_shutdown(rclpy, None)
                return
            time.sleep(0.1)
        node = _VizNode(self._rate)
        try:
            self._started.set()
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(node.node, timeout_sec=0.05)
        finally:
            node_destroy_and_shutdown(rclpy, node)


TASK_BODIES = {
    "SO101ReachEnv": ["item", "goal_site"],
    "SO101LiftEnv": ["item"],
    "SO101PlaceEnv": ["item", "goal_site"],
    "SO101StackEnv": ["item", "box"],
    # can 变体（镜像对应 cube 的 TF 物体列表; 缺失 body 已由 mj_name2id 过滤）
    "SO101ReachCanEnv": ["item", "goal_site"],
    "SO101LiftCanEnv": ["item"],
    "SO101PlaceCanEnv": ["item", "goal_site"],
    "SO101StackCanEnv": ["item", "box"],
}


def node_destroy_and_shutdown(rclpy, viz_node) -> None:
    inner = getattr(viz_node, "node", None)
    if inner is not None:
        try:
            inner.destroy_node()
        except Exception:
            pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


class _VizNode:
    """10Hz 发布：/joint_states、物体 TF、/il/{fixed,wrist}/image（数据源=共享 env）。"""

    def __init__(self, rate: float):
        import mujoco
        from geometry_msgs.msg import TransformStamped  # noqa: F401
        from rclpy.node import Node
        from sensor_msgs.msg import Image, JointState
        from tf2_ros import TransformBroadcaster

        self._mujoco = mujoco
        from .env_backend import get_active_env

        env = get_active_env()
        self._u = env.unwrapped
        self._bodies = [
            b for b in TASK_BODIES.get(type(self._u).__name__, ["item"])
            if mujoco.mj_name2id(self._u.model, mujoco.mjtObj.mjOBJ_BODY, b) >= 0
        ]
        self._qadr = self._u._qpos_adr
        self.node = node = Node("so101_teleop_viz")
        self.pub_js = node.create_publisher(JointState, "joint_states", 10)
        self.tf_br = TransformBroadcaster(node)
        self.pub_imgs = {
            name: node.create_publisher(Image, f"il/{name}/image", 10)
            for name in ("fixed", "wrist")
        }
        self._timer = node.create_timer(1.0 / rate, self.tick)

    def tick(self):
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import Image, JointState

        now = self.node.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        js.name = JOINT_NAMES
        js.position = self._u.data.qpos[self._qadr].tolist()
        self.pub_js.publish(js)
        for body in self._bodies:
            bid = self._mujoco.mj_name2id(self._u.model, self._mujoco.mjtObj.mjOBJ_BODY, body)
            if bid < 0:
                continue
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "world"
            t.child_frame_id = body
            t.transform.translation.x = float(self._u.data.xpos[bid][0])
            t.transform.translation.y = float(self._u.data.xpos[bid][1])
            t.transform.translation.z = float(self._u.data.xpos[bid][2])
            q = self._u.data.xquat[bid]
            t.transform.rotation.w = float(q[0])
            t.transform.rotation.x = float(q[1])
            t.transform.rotation.y = float(q[2])
            t.transform.rotation.z = float(q[3])
            self.tf_br.sendTransform(t)
        imgs = latest_images()
        if imgs is None:
            return
        for name, pub in self.pub_imgs.items():
            img = imgs.get(name)
            if img is None:
                continue
            msg = Image()
            msg.header.stamp = now
            msg.height, msg.width = img.shape[:2]
            msg.encoding = "rgb8"
            msg.step = 3 * msg.width
            msg.data = img.tobytes()
            pub.publish(msg)
