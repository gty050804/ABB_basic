#!/usr/bin/env python3
"""Demo 1: Forward kinematics — joint trajectory -> end-effector pose trajectory."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from irb1300_kin_dyn.robot import IRB1300Robot
from irb1300_kin_dyn.se3 import transform_to_rpy_xyz
from irb1300_kin_dyn.trajectories import default_home_configuration, joint_sinusoid_trajectory


def main() -> None:
    parser = argparse.ArgumentParser(description="IRB1300 forward kinematics demo")
    parser.add_argument("--steps", type=int, default=50)
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "forward_kinematics.npz")
    args = parser.parse_args()

    robot = IRB1300Robot()
    print(f"POE vs URDF FK error: {robot.verify_poe_against_urdf():.3e}")

    q_center = default_home_configuration(robot.n_joints)
    q_traj, _, _ = joint_sinusoid_trajectory(args.steps, robot.n_joints, dt=0.05, q_center=q_center)
    pose_traj = robot.forward_kinematics_trajectory(q_traj)

    xyz_traj = np.zeros((args.steps, 3))
    rpy_traj = np.zeros((args.steps, 3))
    for i, t in enumerate(pose_traj):
        xyz, rpy = transform_to_rpy_xyz(t)
        xyz_traj[i] = xyz
        rpy_traj[i] = rpy

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        q_traj=q_traj,
        pose_traj=pose_traj,
        xyz_traj=xyz_traj,
        rpy_traj=rpy_traj,
        joint_names=np.array(robot.joint_names),
    )

    print(f"Saved {args.steps} FK samples to {args.output}")
    print("Sample end-effector position (last step):", xyz_traj[-1])


if __name__ == "__main__":
    main()
