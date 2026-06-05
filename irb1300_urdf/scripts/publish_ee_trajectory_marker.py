#!/usr/bin/env python3
"""Publish Task 1 EE trajectory as RViz markers and live EE tracking point."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:  # pragma: no cover
    tf2_ros = None  # type: ignore[assignment]
    TransformException = Exception  # type: ignore[misc, assignment]


def _default_trajectory_path() -> Path:
    candidates = [
        Path("/data/IRMV_IMITATION_SEMANTIC_origin/project/irb1300_kin_dyn/results/forward_kinematics.npz"),
        Path(__file__).resolve().parents[2]
        / "irb1300_kin_dyn"
        / "results"
        / "forward_kinematics.npz",
        Path.home() / "irb1300_kin_dyn" / "results" / "forward_kinematics.npz",
    ]
    for path in candidates:
        if path.is_file():
            return path
    return candidates[0]


class EETrajectoryMarkerPublisher(Node):
    def __init__(
        self,
        trajectory_path: Path,
        ee_frame: str,
        base_frame: str,
        track_live_ee: bool,
    ) -> None:
        super().__init__("ee_trajectory_marker_publisher")
        if not trajectory_path.is_file():
            raise FileNotFoundError(
                f"Trajectory file not found: {trajectory_path}\n"
                "Run: python3 scripts/01_forward_kinematics.py  (in irb1300_kin_dyn)"
            )

        data = np.load(trajectory_path)
        if "xyz_traj" in data:
            self._xyz = np.asarray(data["xyz_traj"], dtype=float)
        else:
            self._xyz = np.asarray(data["pose_traj"][:, :3, 3], dtype=float)

        self._base_frame = base_frame
        self._ee_frame = ee_frame
        self._track_live_ee = track_live_ee and tf2_ros is not None

        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(MarkerArray, "ee_trajectory_markers", marker_qos)

        if self._track_live_ee:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._timer = self.create_timer(0.05, self._publish_markers)
        self.get_logger().info(
            f"Publishing EE path ({len(self._xyz)} pts) + live marker in '{base_frame}'"
        )

    def _make_sphere(
        self,
        marker_id: int,
        position: np.ndarray,
        color: tuple[float, float, float, float],
        scale: float,
        namespace: str = "task1_ee",
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime.sec = 0
        return marker

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        path = Marker()
        path.header.frame_id = self._base_frame
        path.header.stamp = now
        path.ns = "task1_ee"
        path.id = 0
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.scale.x = 0.010
        path.color.r = 0.1
        path.color.g = 0.45
        path.color.b = 1.0
        path.color.a = 1.0
        path.pose.orientation.w = 1.0
        path.lifetime.sec = 0
        path.points = [
            Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self._xyz
        ]
        markers.markers.append(path)

        markers.markers.append(
            self._make_sphere(1, self._xyz[0], (0.1, 0.85, 0.2, 1.0), 0.028)
        )
        markers.markers.append(
            self._make_sphere(2, self._xyz[-1], (0.95, 0.15, 0.1, 1.0), 0.024)
        )

        if self._track_live_ee:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._ee_frame,
                    rclpy.time.Time(),
                )
                live = Marker()
                live.header.frame_id = self._base_frame
                live.header.stamp = now
                live.ns = "task1_ee_live"
                live.id = 0
                live.type = Marker.SPHERE
                live.action = Marker.ADD
                live.pose.position.x = tf.transform.translation.x
                live.pose.position.y = tf.transform.translation.y
                live.pose.position.z = tf.transform.translation.z
                live.pose.orientation = tf.transform.rotation
                live.scale.x = 0.020
                live.scale.y = 0.020
                live.scale.z = 0.020
                live.color.r = 1.0
                live.color.g = 0.85
                live.color.b = 0.0
                live.color.a = 1.0
                live.lifetime.sec = 0
                markers.markers.append(live)
            except TransformException:
                pass

        self._pub.publish(markers)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish Task 1 EE trajectory markers for RViz")
    parser.add_argument("--trajectory", type=Path, default=None)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--ee-frame", default="link7")
    parser.add_argument("--no-live", action="store_true", help="Do not publish live EE marker")
    args, ros_args = parser.parse_known_args()

    trajectory_path = (
        args.trajectory.expanduser().resolve()
        if args.trajectory is not None
        else _default_trajectory_path()
    )

    rclpy.init(args=ros_args)
    node = EETrajectoryMarkerPublisher(
        trajectory_path=trajectory_path,
        ee_frame=args.ee_frame,
        base_frame=args.base_frame,
        track_live_ee=not args.no_live,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
