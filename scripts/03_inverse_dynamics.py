#!/usr/bin/env python3
"""Demo 3: Inverse dynamics — joint q, qd, qdd + EE wrench -> torques."""

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
    parser = argparse.ArgumentParser(description="IRB1300 inverse dynamics demo")
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "inverse_dynamics.npz")
    args = parser.parse_args()

    robot = IRB1300Robot()
    q_center = default_home_configuration(robot.n_joints)
    q_traj, qd_traj, qdd_traj = joint_sinusoid_trajectory(
        args.steps, robot.n_joints, dt=0.05, amplitude=0.2, q_center=q_center
    )

    f_ext = np.array([0.0, 0.0, -10.0, 0.0, 0.0, 0.0])
    tau_drive = np.zeros((args.steps, robot.n_joints))
    tau_constraint = np.zeros((args.steps, robot.n_joints))
    tau_total = np.zeros((args.steps, robot.n_joints))

    for i in range(args.steps):
        result = robot.inverse_dynamics(q_traj[i], qd_traj[i], qdd_traj[i], f_ext)
        tau_drive[i] = result["tau_drive"]
        tau_constraint[i] = result["tau_constraint"]
        tau_total[i] = result["tau_total"]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        q_traj=q_traj,
        qd_traj=qd_traj,
        qdd_traj=qdd_traj,
        f_ext_ee=f_ext,
        tau_drive=tau_drive,
        tau_constraint=tau_constraint,
        tau_total=tau_total,
        joint_names=np.array(robot.joint_names),
    )

    print(f"External wrench at EE [N, Nm]: {f_ext}")
    print(f"Mean |tau_drive|: {np.linalg.norm(tau_drive, axis=1).mean():.3f} Nm")
    print(f"Mean |tau_constraint|: {np.linalg.norm(tau_constraint, axis=1).mean():.3f} Nm")
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
