#!/usr/bin/env python3
"""Demo 2: Inverse kinematics — EE pose trajectory -> joint trajectory (POE Jacobian)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from irb1300_kin_dyn.robot import IRB1300Robot
from irb1300_kin_dyn.trajectories import default_home_configuration, joint_sinusoid_trajectory


def main() -> None:
    parser = argparse.ArgumentParser(description="IRB1300 inverse kinematics demo")
    parser.add_argument("--steps", type=int, default=30)
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "inverse_kinematics.npz")
    args = parser.parse_args()

    robot = IRB1300Robot()
    q_init = default_home_configuration(robot.n_joints)

    q_traj_ref, _, _ = joint_sinusoid_trajectory(
        args.steps, robot.n_joints, dt=0.05, amplitude=0.15, q_center=q_init
    )
    pose_traj = robot.forward_kinematics_trajectory(q_traj_ref)

    q_traj_ik, success = robot.inverse_kinematics_trajectory(pose_traj, q_init)
    fk_check = robot.forward_kinematics_trajectory(q_traj_ik)
    pos_err = np.linalg.norm(fk_check[:, :3, 3] - pose_traj[:, :3, 3], axis=1)
    success = pos_err < 1e-3

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        pose_traj=pose_traj,
        q_traj_ref=q_traj_ref,
        q_traj_ik=q_traj_ik,
        ik_success=success,
        position_error=pos_err,
        joint_names=np.array(robot.joint_names),
    )

    print(f"IK success rate: {success.mean() * 100:.1f}%")
    print(f"Mean position error: {pos_err.mean():.3e} m")
    print(f"Max position error: {pos_err.max():.3e} m")
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
