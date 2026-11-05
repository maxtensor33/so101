"""cv2 camera publisher node (plan/il_sim_develop.md D4).

cv2.VideoCapture(index) -> /real/<key>/image_raw (sensor_msgs/Image, rgb8).
Mirrors lerobot OpenCVCamera semantics: index, 640x480, fps.
"""

import sys
from pathlib import Path


def _find_repo_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    return None


# 统一 numpy2 栈: 自举 .venv（numpy 2.2.6 + cv2 4.14.0 互相匹配）。
# 规避两个坑: ~/.local 孤儿 numpy 2.3.5 与系统 cv2 4.6.0(numpy1.x 编译)。
_REPO = _find_repo_root()
if _REPO is not None:
    _VENV = str(_REPO / ".venv/lib/python3.12/site-packages")
    if _VENV not in sys.path and Path(_VENV).is_dir():
        sys.path.insert(0, _VENV)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class IlSimCameraNode(Node):
    def __init__(self):
        super().__init__("il_sim_camera")
        self.declare_parameter("camera_key", "fixed")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("fps", 30.0)
        key = str(self.get_parameter("camera_key").value)
        device = self._resolve_device(str(self.get_parameter("device").value))
        fps = float(self.get_parameter("fps").value)

        import cv2

        self._cap = cv2.VideoCapture(device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera device {device}")

        self._pub = self.create_publisher(Image, f"/real/{key}/image_raw", 10)
        self._timer = self.create_timer(1.0 / fps, self._grab)
        self._frame = None
        self.get_logger().info(f"camera {key}: device={device} {fps}Hz -> /real/{key}/image_raw")

    @staticmethod
    def _resolve_device(v):
        """纯数字当索引 int, 否则当 /dev/videoN 路径透传给 cv2。"""
        return int(v) if v.isdigit() else v

    def _grab(self):
        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warn("camera read failed", throttle_duration_sec=5.0)
            return
        import cv2

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "rgb8"
        msg.step = 3 * msg.width
        msg.data = frame.tobytes()
        self._pub.publish(msg)

    def destroy_node(self):
        self._cap.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IlSimCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # SIGTERM 场景 context 已关, 防双 shutdown RCLError


if __name__ == "__main__":
    main()
