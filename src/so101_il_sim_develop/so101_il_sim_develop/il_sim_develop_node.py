"""il_sim 策略实机部署节点 (workflow/plan/il_sim_develop.md Phase 2).

10Hz tick: obs(joint_states + usb_cam) -> policy -> clamp -> JTC trajectory.
estop latch + Enter homing to rest + confirm_step.
Reuses so101_il_sim_infer.core.load_policy.
"""

import sys
import threading
from pathlib import Path


def _find_repo_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    return None


# 自举必须在任何 numpy/cv2/lerobot 导入之前: ros2 run 环境下 user-site 的损坏
# numpy(2.3.5) 会先被加载, accelerate/transformers 链即崩 (numpy._core 无 multiarray)。
_REPO = _find_repo_root()
if _REPO is not None:
    for _p in (
        str(_REPO / "libs"),
        str(_REPO / ".venv/lib/python3.12/site-packages"),
        str(_REPO / "src/so101_il_sim_infer"),
    ):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .real_infer import (
    JOINT_NAMES,
    EstopLatch,
    build_obs_batch,
    clamp_target,
    denormalize_action,
    plan_to_rest,
    resize_to_policy,
)

REST_QPOS = np.zeros(6)


class IlSimDevelopNode(Node):
    def __init__(self):
        super().__init__("il_sim_develop")
        self.declare_parameter("env", "reach")
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("camera_keys", "fixed,wrist")
        self.declare_parameter("confirm_step", False)
        self.declare_parameter("reset_on_enter", True)
        self.declare_parameter("max_delta", 0.1)
        self._env = str(self.get_parameter("env").value)
        ckpt = str(self.get_parameter("checkpoint").value)
        self._camera_keys = tuple(
            k.strip() for k in str(self.get_parameter("camera_keys").value).split(",") if k.strip()
        )
        self._confirm_step = bool(self.get_parameter("confirm_step").value)
        self._reset_on_enter = bool(self.get_parameter("reset_on_enter").value)
        self._max_delta = float(self.get_parameter("max_delta").value)

        from so101_il_sim_infer.core import load_policy

        self.policy, self._pre, self._post = load_policy(ckpt)
        self.get_logger().info(f"loading IL policy from {ckpt} (env={self._env})")

        self._estop = EstopLatch()
        self._prev_target = None
        self._state = "run"  # run | homing | confirm_wait
        self._latest_images = {}
        self._qpos = None
        self._step_requested = False

        self.create_subscription(JointState, "joint_states", self._on_js, 10)
        for key, topic in (("fixed", "/real/fixed/image_raw"), ("wrist", "/real/wrist/image_raw")):
            self.create_subscription(Image, topic, self._make_img_cb(key), 10)
        self.create_subscription(Empty, "il_sim_develop/estop", self._on_estop, 10)

        self.pub_arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.pub_gripper = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )
        self.pub_imgs = {
            key: self.create_publisher(Image, f"/il_sim_develop/{key}/image", 10)
            for key in self._camera_keys
        }
        rate = float(self.get_parameter("rate").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"rate={rate}Hz camera_keys={self._camera_keys}")

    # ---------- callbacks ----------

    def _on_js(self, msg):
        qmap = dict(zip(msg.name, msg.position))
        try:
            self._qpos = np.array([qmap[j] for j in JOINT_NAMES])
        except KeyError:
            return

    def _make_img_cb(self, key):
        def cb(msg):
            if msg.encoding == "rgb8":
                img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                self._latest_images[key] = img.copy()
        return cb

    def _on_estop(self, msg):
        self._estop.trigger()
        self.get_logger().warn("estop triggered, holding position")

    # ---------- keyboard ----------

    def _start_key_listener(self):
        try:
            from pynput import keyboard

            self._listener = keyboard.Listener(on_press=self._on_key)
            self._listener.start()
        except Exception as exc:
            self._listener = None
            self.get_logger().warn(f"pynput unavailable, fallback stdin: {exc}")
        threading.Thread(target=self._stdin_loop, daemon=True).start()

    def _on_key(self, key):
        try:
            from pynput.keyboard import Key

            if key == Key.enter:
                self._key_event()
        except Exception:
            pass

    def _stdin_loop(self):
        try:
            for _line in sys.stdin:
                self._key_event()
        except Exception:
            pass

    def _key_event(self):
        if self._state == "confirm_wait":
            self._step_requested = True
        else:
            self._reset_requested = True

    # ---------- control loop ----------

    def _send_target(self, target):
        stamp_dur = 1.0 / 100.0
        for pub, names, idx in (
            (self.pub_arm, JOINT_NAMES[:5], slice(0, 5)),
            (pub_gripper := self.pub_gripper, JOINT_NAMES[5:], slice(5, 6)),
        ):
            traj = JointTrajectory()
            traj.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in target[idx]]
            pt.time_from_start.sec = 0
            pt.time_from_start.nanosec = int(stamp_dur * 1e9)
            traj.points = [pt]
            pub.publish(traj)
        self._prev_target = np.asarray(target, dtype=float)

    def _forward_imgs(self):
        for key, pub in self.pub_imgs.items():
            img = self._latest_images.get(key)
            if img is None:
                continue
            msg = Image()
            msg.height, msg.width = img.shape[:2]
            msg.encoding = "rgb8"
            msg.step = 3 * msg.width
            msg.data = img.tobytes()
            pub.publish(msg)

    def tick(self):
        if self._estop.triggered:
            return
        self._forward_imgs()
        if self._state == "homing":
            self._tick_homing()
            return
        if self._state == "confirm_wait":
            if not self._step_requested:
                return
            self._step_requested = False
        if self._qpos is None:
            return
        missing = [k for k in self._camera_keys if k not in self._latest_images]
        if missing:
            self.get_logger().warn(f"missing cameras: {missing}", throttle_duration_sec=5.0)
            return
        obs_images = {
            k: resize_to_policy(self._latest_images[k]) for k in self._camera_keys
        }
        batch = build_obs_batch(self._qpos, obs_images, self._camera_keys)
        action = np.asarray(self._post(self.policy.select_action(self._pre(batch))), dtype=float).reshape(-1)
        target = clamp_target(denormalize_action(action), self._prev_target
                              if self._prev_target is not None else self._qpos,
                              self._max_delta)
        if self._confirm_step:
            self._state = "confirm_wait"
        self._send_target(target)

    def _tick_homing(self):
        cur = self._qpos if self._qpos is not None else REST_QPOS
        nxt = plan_to_rest(cur, REST_QPOS, max_step=0.025)
        self._send_target(nxt)
        if np.allclose(nxt, REST_QPOS, atol=1e-3):
            self._state = "run"
            self._prev_target = None
            self.get_logger().info("homing done, back to run")

    def _maybe_start_reset(self):
        if getattr(self, "_reset_requested", False):
            self._reset_requested = False
            self._state = "homing"
            self.get_logger().info("reset: homing to rest")


    def tick_reset_check(self):
        self._maybe_start_reset()

    # ---------- lifecycle ----------

    def destroy_node(self):
        self.get_logger().info("shutdown: homing to rest")
        try:
            for _ in range(200):
                cur = self._qpos if self._qpos is not None else REST_QPOS
                nxt = plan_to_rest(cur, REST_QPOS)
                self._send_target(nxt)
                if np.allclose(nxt, REST_QPOS, atol=1e-3):
                    break
            import time

            time.sleep(0.5)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IlSimDevelopNode()
    if node._reset_on_enter:
        node._start_key_listener()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick_reset_check()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


