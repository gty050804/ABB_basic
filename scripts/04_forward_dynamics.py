#!/usr/bin/env python3
"""Demo 4: Forward dynamics — q, qd, tau, EE wrench -> joint accelerations."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from irb1300_kin_dyn.robot import IRB1300Robot
from irb1300_kin_dyn.trajectories import default_home_configuration


def main() -> None:
    parser = argparse.ArgumentParser(description="IRB1300 forward dynamics demo")
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "forward_dynamics.npz")
    args = parser.parse_args()

    robot = IRB1300Robot()
    q = default_home_configuration(robot.n_joints)
    qd = np.array([0.1, -0.05, 0.08, 0.0, 0.06, -0.04])
    f_ext = np.array([0.0, 0.0, -8.0, 0.0, 0.0, 0.0])
    qdd_target = np.array([0.5, -0.3, 0.4, 0.2, -0.2, 0.3])
    tau_drive = robot.inverse_dynamics(q, qd, qdd_target, f_ext)["tau_total"]

    qdd = robot.forward_dynamics(q, qd, tau_drive, f_ext)

    id_check = robot.inverse_dynamics(q, qd, qdd, f_ext)
    residual = np.linalg.norm(
        id_check["tau_total"] - (id_check["tau_drive"] + id_check["tau_constraint"])
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        q=q,
        qd=qd,
        qdd=qdd,
        qdd_target=qdd_target,
        tau_drive=tau_drive,
        f_ext_ee=f_ext,
        tau_total_check=id_check["tau_total"],
        joint_names=np.array(robot.joint_names),
    )

    print("Joint positions q:", q)
    print("Joint velocities qd:", qd)
    print("Driving torques:", tau_drive)
    print("Computed joint accelerations qdd:", qdd)
    print(f"ID consistency residual: {residual:.3e}")
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
