"""SO101 DAgger student 推理节点 (workflow/plan/rl_dagger.md Phase 4 / D8)。

镜像 so101_rl_infer/rl_infer_node.py: 加载 student checkpoint (backend="dagger"),
自持相机版 env（meta env_kwargs 原样透传）, 每 tick:
obs -> make_obs_batch -> env.step -> 发布 /joint_states、物体 TF、rl_dagger/{cam}/image。
student 无策略内部状态（无 obs 队列）, Enter 重置只 env.reset。

RViz 订阅显示 (纯可视化; MuJoCo 窗口可用 mujoco:=true 同开)。
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

import gymnasium as gym
import so101_envs  # noqa: F401  注册副作用
from so101_rl.backends.base import read_meta
from so101_rl.runs import resolve_ckpt_dir
from so101_rl_dagger_infer.core import camera_keys_from_meta, load_student, make_obs_batch

# 短名 → 相机版 env id（student checkpoint 的 env 均为 Camera-v1; 也接受完整 id 直传）
ENV_ID = {
    "reach": "SO101ReachCamera-v1",
    "lift": "SO101LiftCamera-v1",
    "place": "SO101PlaceCamera-v1",
    "stack": "SO101StackCamera-v1",
    "reachcan": "SO101ReachCanCamera-v1",
    "liftcan": "SO101LiftCanCamera-v1",
    "placecan": "SO101PlaceCanCamera-v1",
    "stackcan": "SO101StackCanCamera-v1",
}
TASK_BODIES = {
    "reach": ["item", "goal_site"],
    "lift": ["item"],
    "place": ["item", "bin"],
    "stack": ["itemA", "itemB"],
    "reachcan": ["item", "goal_site"],
    "liftcan": ["item"],
    "placecan": ["item", "bin"],
    "stackcan": ["itemA", "itemB"],
}
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def _resolve_env_id(env_name: str, meta_env_id: str) -> str:
    """参数 env: 空 → checkpoint meta 的 env_id; 短名 → Camera-v1; 完整 id 原样。"""
    if not env_name:
        return meta_env_id
    return ENV_ID.get(env_name, env_name)


def _env_task_name(env_id: str) -> str:
    """SO101ReachCanCamera-v1 -> reachcan（TASK_BODIES 查表键）。"""
    name = env_id.removeprefix("SO101").removesuffix("Camera-v1")
    return name.lower()


class RlDaggerInferNode(Node):
    def __init__(self):
        super().__init__("so101_rl_dagger_infer")
        self.declare_parameter("env", "")  # 空 = checkpoint meta 的 env_id
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("rate", 10.0)
        # mujoco:=true: 同一个 env 实例开 MuJoCo 交互窗口（render_mode="human"）
        self.declare_parameter("mujoco", False)
        # 按 Enter 重置 episode（pynput 全局监听; 无 pynput 时回退 stdin）
        self.declare_parameter("reset_on_enter", True)
        env_name = str(self.get_parameter("env").value)
        # 支持最新 run 指针: checkpoint:=outputs/rl/dagger-<tag>/latest
        ckpt = resolve_ckpt_dir(Path(str(self.get_parameter("checkpoint").value)).expanduser())
        self._mujoco = bool(self.get_parameter("mujoco").value)
        reset_on_enter = bool(self.get_parameter("reset_on_enter").value)
        self._reset_requested = False
        self._listener = None

        self.policy, meta = load_student(ckpt)
        self.camera_keys = camera_keys_from_meta(meta)
        env_id = _resolve_env_id(env_name, meta["env_id"])
        if env_id != meta["env_id"]:
            self.get_logger().warn(
                f"参数 env ({env_id}) != checkpoint env ({meta['env_id']}), 以参数为准")
        self.get_logger().info(
            f"loading dagger student from {ckpt} "
            f"(env={env_id}, cameras={self.camera_keys})")

        env_kwargs = dict(meta.get("env_kwargs", {}))
        if self._mujoco:
            env_kwargs["render_mode"] = "human"
        self.env = gym.make(env_id, **env_kwargs)
        self.u = self.env.unwrapped
        self.obs, _ = self.env.reset(seed=0)
        if self._mujoco:
            self.env.render()  # 打开 viewer 并同步首帧
        if reset_on_enter:
            self._start_reset_listener()
        self._qadr = self.u._qpos_adr
        task_name = _env_task_name(env_id)
        self._item_bodies = [b for b in TASK_BODIES.get(task_name, ["item"])
                             if mujoco.mj_name2id(self.u.model, mujoco.mjtObj.mjOBJ_BODY, b) >= 0]

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.tf_br = TransformBroadcaster(self)
        # 相机图像: 按 checkpoint camera_keys 实际数量发布 rl_dagger/{key}/image
        self.pub_imgs = {key: self.create_publisher(Image, f"rl_dagger/{key}/image", 10)
                         for key in self.camera_keys}

        rate = float(self.get_parameter("rate").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        if self._reset_requested:
            self._reset_requested = False
            self.obs, _ = self.env.reset()  # 随机 seed: 每次 Enter 换一个场景
            self.get_logger().info("episode reset (Enter)")
            if self._mujoco:
                self.env.render()
            return
        action = make_obs_batch(self.obs, self.camera_keys, self.policy)
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
        for key, pub in self.pub_imgs.items():
            img = self.obs["rgb"][key]
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width = img.shape[:2]
            msg.encoding = "rgb8"
            msg.step = 3 * msg.width
            msg.data = np.asarray(img).tobytes()
            pub.publish(msg)
        if info.get("success"):
            self.get_logger().info("episode success!", throttle_duration_sec=5.0)

    # ---------- Enter 重置监听 ----------
    def _start_reset_listener(self):
        """pynput 全局 Enter 监听（X11/Wayland 视环境）; 失败回退 stdin 行读取。"""
        try:
            from pynput import keyboard

            self._listener = keyboard.Listener(on_press=self._on_key)
            self._listener.start()
        except Exception as exc:  # 无图形/权限受限: 仍可用 stdin Enter
            self._listener = None
            self.get_logger().warn(f"pynput Enter 监听不可用（回退 stdin）: {exc}")

        import threading

        threading.Thread(target=self._stdin_loop, daemon=True).start()

    def _on_key(self, key):
        try:
            from pynput.keyboard import Key

            if key == Key.enter:
                self._reset_requested = True
        except Exception:
            pass

    def _stdin_loop(self):
        try:
            for _line in sys.stdin:
                self._reset_requested = True
        except Exception:
            pass

    def destroy_node(self):
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception:
                pass
            self._listener = None
        self.env.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RlDaggerInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
