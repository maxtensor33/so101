"""draw_cube 验收节点 (workflow/plan/robotics.md Phase 4/5, 对标 lab-2 c3_draw_cube.py)。

流水线: 立方体 12 边 → 航点 → DLS IK (温启动) → quintic 关节轨迹 → 执行。
  - ctrl_mode:=torque   轨迹采样发 /draw_cube/arm_target (力矩层执行)
  - 位置模式: 轨迹经 follow_joint_trajectory action 下发 (JTC 执行)
同时发布 RViz Marker (期望立方体边 + 实际末端轨迹) 与误差统计。
"""

import sys
from pathlib import Path


def _find_repo_root():
    """Walk up to the repo root (uv + colcon hybrid: so101_robotics lives in libs/)."""
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    return None


_REPO = _find_repo_root()
if _REPO is not None:
    for _p in (str(_REPO / "libs"), str(_REPO / ".venv/lib/python3.12/site-packages")):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from so101_robotics import fk, ik
from so101_robotics import trajectory as traj

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
Q_REF = np.array([0.0, 0.8, -1.5, 0.7, 0.0, 0.3])  # 收臂参考位形 (航点生成基准)

CUBE_EDGES = [  # 12 条边 (顶点索引对)
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]
SIDES_TO_TRY = [0.08, 0.07, 0.06]  # so101 workspace is small; probe says 0.06 works
POINTS_PER_EDGE = 6
CUBE_CENTER_OFFSET = np.array([-0.045, 0.02, -0.005])  # probe-verified reachable center


def cube_vertices(center, side):
    """8 顶点: 0-3 底面 (z-), 4-7 顶面 (z+) — 与 CUBE_EDGES 顶点索引对应。"""
    h = side / 2.0
    cx, cy, cz = center
    return np.array([
        [cx - h, cy - h, cz - h],
        [cx + h, cy - h, cz - h],
        [cx + h, cy + h, cz - h],
        [cx - h, cy + h, cz - h],
        [cx - h, cy - h, cz + h],
        [cx + h, cy - h, cz + h],
        [cx + h, cy + h, cz + h],
        [cx - h, cy + h, cz + h],
    ])


def cube_wireframe_marker(waypoints):
    """cube 线框 LINE_LIST: 航点每边连续 POINTS_PER_EDGE 个, 沿边连接相邻点。"""
    m = Marker()
    m.header.frame_id = "world"
    m.ns = "cube_wireframe"
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.id = 1
    m.scale.x = 0.002
    m.color.g = 1.0
    m.color.a = 1.0
    for e in range(len(CUBE_EDGES)):
        pts = waypoints[e * POINTS_PER_EDGE:(e + 1) * POINTS_PER_EDGE]
        for k in range(len(pts) - 1):
            m.points.append(geometry_point(pts[k]))
            m.points.append(geometry_point(pts[k + 1]))
    return m


def ee_path_marker(points):
    """实际 EE 轨迹 LINE_STRIP (蓝色)。"""
    m = Marker()
    m.header.frame_id = "world"
    m.ns = "ee_path"
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.id = 2
    m.scale.x = 0.003
    m.color.b = 1.0
    m.color.a = 1.0
    for p in points:
        m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
    return m


class DrawCubeNode(Node):
    """Pipeline node: build cube waypoints -> IK -> quintic joint traj -> execute."""

    def __init__(self):
        super().__init__("draw_cube_node")
        self.declare_parameter("ctrl_mode", "torque")
        self.declare_parameter("edge_duration", 1.5)
        self.ctrl_mode = self.get_parameter("ctrl_mode").get_parameter_value().string_value
        self.edge_duration = float(self.get_parameter("edge_duration").value)

        self.q_arm = None
        self.errors = []  # (t_sec, ee_pos_err_m)
        self.ee_path = []  # measured ee positions for the path marker
        self._traj = None
        self._t0 = None
        self._started = False
        self._pending_wps = None
        self._exec_once_timer = None

        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, qos_profile_sensor_data
        )
        marker_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "/draw_cube/markers", marker_qos)
        self._cube_ma = None
        self.target_pub = None
        self.create_timer(2.0, self._start_once)

    def _on_joint_states(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            self.q_arm = np.array([msg.position[idx[n]] for n in ARM_JOINTS])
        except KeyError:
            return
        if self._traj is not None and self._t0 is not None:
            self._record_error()

    def _record_error(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self._t0
        t_c = min(t, self._traj.duration)
        q_d, _, _ = self._traj.sample(t_c)
        ee_actual = fk.mujoco_fk(np.concatenate([self.q_arm, [0.0]])).translation
        ee_des = fk.mujoco_fk(np.concatenate([q_d, [0.0]])).translation
        err = float(np.linalg.norm(ee_actual - ee_des))
        self.errors.append((t, err))
        self.ee_path.append(ee_actual)

    def _start_once(self):
        if self._started or self.q_arm is None:
            return
        self._started = True
        self._build_and_run()

    def _build_and_run(self):
        waypoints = self._build_waypoints()
        if waypoints is None:
            self.get_logger().error("cube waypoints unreachable, abort")
            return
        joint_wps = self._solve_ik(waypoints)
        if joint_wps is None:
            self.get_logger().error("IK failed on some waypoints, abort")
            return
        self._cube_ma = _wrap_marker(cube_wireframe_marker(waypoints))
        self._publish_markers()
        # 周期重发布: cube 线框常驻 + EE 轨迹实时增长 (RViz 晚启动也能看到)
        self.create_timer(0.5, self._publish_markers)
        # IK blocks the executor for tens of seconds; defer execution to a one-shot
        # timer so self.q_arm is fresh (not 25s stale) when the trajectory starts.
        self._pending_wps = joint_wps
        self._exec_once_timer = self.create_timer(0.2, self._exec_once)

    def _exec_once(self):
        self._exec_once_timer.destroy()
        if self.q_arm is None or self._pending_wps is None:
            return
        self._start_execution(self._pending_wps)

    def _build_waypoints(self):
        """Pick the largest cube side whose edge waypoints are all reachable."""
        base = fk.pinocchio_fk(Q_REF).translation + CUBE_CENTER_OFFSET
        for side in SIDES_TO_TRY:
            verts = cube_vertices(base, side)
            pts = []
            for a, b in CUBE_EDGES:
                for s in np.linspace(0.0, 1.0, POINTS_PER_EDGE):
                    pts.append(verts[a] * (1 - s) + verts[b] * s)
            if all(ik.ik_dls(p, q_init=Q_REF, max_iters=500, n_restarts=10).converged for p in pts):
                self.get_logger().info(f"cube side = {side * 100:.0f} cm, "
                                       f"{len(pts)} waypoints all reachable")
                return pts
        return None

    def _solve_ik(self, waypoints):
        """Warm-started IK along the waypoint chain; returns joint waypoints."""
        joint_wps = []
        q0 = Q_REF.copy()  # same start as the reachability check
        for p in waypoints:
            r = ik.ik_dls(p, q_init=q0, max_iters=500, n_restarts=10)
            if not r.converged:
                return None
            joint_wps.append(r.q)
            q0 = r.q
        return joint_wps

    def _start_execution(self, joint_wps):
        # start from current pose for a smooth run-in segment. Segment durations
        # are adaptive in joint space: IK branch flips create 1.5+ rad segments
        # where a fixed short duration saturates torque (limit ~1 rad/s).
        traj_wps = [np.concatenate([self.q_arm, [0.0]]), *joint_wps]
        seg_dur = self.edge_duration / (POINTS_PER_EDGE - 1)
        dists = [float(np.linalg.norm(b - a)) for a, b in zip(traj_wps[:-1], traj_wps[1:])]
        durations = [max(seg_dur, d / 1.0) for d in dists]
        self._traj = traj.JointTrajectory(traj_wps, durations)
        self._t0 = self.get_clock().now().nanoseconds / 1e9
        if self.ctrl_mode == "position":
            self._execute_position_mode()
            # no torque tick in position mode: poll for completion to emit stats
            self._pos_done_timer = self.create_timer(0.5, self._pos_done_check)
        else:
            self.target_pub = self.create_publisher(
                Float64MultiArray, "/draw_cube/arm_target", 10
            )
            self.create_timer(0.02, self._torque_tick)

    def _torque_tick(self):
        if self._traj is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9 - self._t0
        if t <= self._traj.duration:
            q_d, dq_d, ddq_d = self._traj.sample(t)
        else:
            q_d, dq_d, ddq_d = self._traj.sample(self._traj.duration)
            if not hasattr(self, "_hold_started"):
                self._hold_started = t
            if t - self._hold_started > 2.0:
                self._finish()
                return
        msg = Float64MultiArray()
        msg.data = [float(v) for v in np.concatenate([q_d[:5], dq_d[:5], ddq_d[:5]])]
        self.target_pub.publish(msg)

    def _execute_position_mode(self):
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        client.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ARM_JOINTS
        dt = 0.05
        t = 0.0
        while t <= self._traj.duration:
            q, dq, _ = self._traj.sample(t)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q[:5]]
            pt.velocities = [float(v) for v in dq[:5]]
            pt.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
            goal.trajectory.points.append(pt)
            t += dt
        # JTC requires the last point to be at rest
        goal.trajectory.points[-1].velocities = [0.0] * 5
        self.get_logger().info(f"position mode: sending {len(goal.trajectory.points)} points")
        client.send_goal_async(goal)

    def _pos_done_check(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self._t0
        if t > self._traj.duration + 2.0:
            self._pos_done_timer.destroy()
            self._finish()

    def _finish(self):
        if not self.errors:
            self.get_logger().warn("no error samples collected")
            return
        errs_mm = np.array([e * 1000.0 for _, e in self.errors])
        rms = float(np.sqrt(np.mean(errs_mm**2)))
        self.get_logger().info(
            f"DONE samples={len(errs_mm)} rms={rms:.2f} mm max={errs_mm.max():.2f} mm"
        )
        with open("/tmp/draw_cube_errors.csv", "w") as f:
            f.write("t_sec,err_mm\n")
            for t, e in self.errors:
                f.write(f"{t:.4f},{e * 1000.0:.4f}\n")
        self._publish_markers()
        self._traj = None

    def _publish_markers(self):
        """cube 线框 + 当前 EE 轨迹, 由周期定时器与关键事件共同调用。"""
        if self._cube_ma is not None:
            self.marker_pub.publish(self._cube_ma)
        if self.ee_path:
            self.marker_pub.publish(_wrap_marker(ee_path_marker(self.ee_path)))


def geometry_point(arr):
    return Point(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))


def _wrap_marker(m):
    ma = MarkerArray()
    ma.markers.append(m)
    return ma


def main(args=None):
    rclpy.init(args=args)
    node = DrawCubeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()