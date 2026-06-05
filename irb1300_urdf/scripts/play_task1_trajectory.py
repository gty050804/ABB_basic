#!/usr/bin/env python3
"""Publish Task 1 joint trajectory to /joint_states for RViz playback."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _default_trajectory_path() -> Path:
    candidates = [
        Path(__file__).resolve().parents[2]
        / "irb1300_kin_dyn"
        / "results"
        / "forward_kinematics.npz",
        Path("/data/IRMV_IMITATION_SEMANTIC_origin/project/irb1300_kin_dyn/results/forward_kinematics.npz"),
        Path.home() / "irb1300_kin_dyn" / "results" / "forward_kinematics.npz",
    ]
    for path in candidates:
        if path.is_file():
            return path
    return candidates[0]


class Task1TrajectoryPlayer(Node):
    def __init__(
        self,
        trajectory_path: Path,
        rate_hz: float,
        loop: bool,
        start_step: int,
    ) -> None:
        super().__init__("task1_trajectory_player")
        if not trajectory_path.is_file():
            raise FileNotFoundError(
                f"Trajectory file not found: {trajectory_path}\n"
                "Run: python3 scripts/01_forward_kinematics.py  (in irb1300_kin_dyn)"
            )

        data = np.load(trajectory_path)
        self._joint_names = [str(n) for n in data["joint_names"]]
        self._q_traj = np.asarray(data["q_traj"], dtype=float)
        self._loop = loop
        self._index = max(0, min(start_step, len(self._q_traj) - 1))

        self._pub = self.create_publisher(JointState, "joint_states", 10)
        period = 1.0 / max(rate_hz, 1e-3)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Playing {len(self._q_traj)} steps from {trajectory_path} "
            f"at {rate_hz:.1f} Hz (loop={loop})"
        )
        self.get_logger().info(f"Joints: {', '.join(self._joint_names)}")

    def _on_timer(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = self._q_traj[self._index].tolist()
        self._pub.publish(msg)

        if self._index + 1 >= len(self._q_traj):
            if self._loop:
                self._index = 0
            else:
                self.get_logger().info("Trajectory finished.")
                self._timer.cancel()
                return
        else:
            self._index += 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Play Task 1 FK trajectory in ROS 2")
    parser.add_argument(
        "--trajectory",
        type=Path,
        default=None,
        help="Path to forward_kinematics.npz",
    )
    parser.add_argument("--rate", type=float, default=20.0, help="Playback rate [Hz]")
    parser.add_argument("--loop", action="store_true", default=True, help="Loop trajectory")
    parser.add_argument("--no-loop", dest="loop", action="store_false", help="Play once")
    parser.add_argument("--start-step", type=int, default=0, help="Start index")
    args, ros_args = parser.parse_known_args()

    trajectory_path = (
        args.trajectory.expanduser().resolve()
        if args.trajectory is not None
        else _default_trajectory_path()
    )
    rclpy.init(args=ros_args)
    node = Task1TrajectoryPlayer(
        trajectory_path=trajectory_path,
        rate_hz=args.rate,
        loop=args.loop,
        start_step=args.start_step,
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
