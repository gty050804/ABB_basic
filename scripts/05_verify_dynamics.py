#!/usr/bin/env python3
"""Verify Task 3 (inverse dynamics) and Task 4 (forward dynamics) results."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from irb1300_kin_dyn.robot import IRB1300Robot


def _require_file(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}. Run the corresponding demo script first.")


def _wrench_joint_torque(robot: IRB1300Robot, q: np.ndarray, qd: np.ndarray, f_ext: np.ndarray) -> np.ndarray:
    return robot.inverse_dynamics(q, qd, np.zeros(robot.n_joints), f_ext)["tau_constraint"]


def verify_inverse_dynamics(robot: IRB1300Robot, path: Path) -> dict[str, float]:
    """Checks saved Task 3 torques against rigid-body consistency relations."""
    d = np.load(path)
    q_traj = d["q_traj"]
    qd_traj = d["qd_traj"]
    qdd_traj = d["qdd_traj"]
    f_ext = d["f_ext_ee"]
    tau_drive = d["tau_drive"]
    tau_constraint = d["tau_constraint"]
    tau_total = d["tau_total"]

    n = len(q_traj)
    decomp_err = np.zeros(n)
    eq_motion_err = np.zeros(n)
    wrench_err = np.zeros(n)
    id_recompute_err = np.zeros(n)
    id_fd_err = np.zeros(n)

    for i in range(n):
        q, qd, qdd = q_traj[i], qd_traj[i], qdd_traj[i]
        decomp_err[i] = np.linalg.norm(tau_total[i] - (tau_drive[i] + tau_constraint[i]))

        m = robot.dynamics.mass_matrix(q)
        c = robot.dynamics.coriolis_gravity(q, qd)
        eq_motion_err[i] = np.linalg.norm(tau_drive[i] - (m @ qdd + c))

        id_now = robot.inverse_dynamics(q, qd, qdd, f_ext)
        wrench_err[i] = np.linalg.norm(tau_constraint[i] - id_now["tau_constraint"])
        id_recompute_err[i] = np.linalg.norm(tau_total[i] - id_now["tau_total"])

        qdd_fd = robot.forward_dynamics_theoretical(q, qd, tau_total[i], f_ext)
        id_fd_err[i] = np.linalg.norm(qdd_fd - qdd)

    return {
        "decomposition_max": float(decomp_err.max()),
        "equation_of_motion_max": float(eq_motion_err.max()),
        "wrench_mapping_max": float(wrench_err.max()),
        "id_recompute_max": float(id_recompute_err.max()),
        "id_fd_roundtrip_max": float(id_fd_err.max()),
    }


def verify_forward_dynamics(robot: IRB1300Robot, path: Path) -> dict[str, float]:
    """Checks saved Task 4 accelerations via FD consistency and equations of motion."""
    d = np.load(path)
    q = d["q"]
    qd = d["qd"]
    qdd = d["qdd"]
    tau_drive = d["tau_drive"]
    f_ext = d["f_ext_ee"]

    qdd_fd = robot.forward_dynamics(q, qd, tau_drive, f_ext)
    fd_recompute_err = float(np.linalg.norm(qdd_fd - qdd))

    metrics = {
        "fd_recompute_max": fd_recompute_err,
        "equation_max": float("nan"),
        "id_total_consistency_max": float("nan"),
    }

    if robot.use_simulator_fd and robot._sim_backend is not None:
        pb = robot._sim_backend
        qdd_sim_no_wrench = robot.forward_dynamics(q, qd, tau_drive, np.zeros(6))
        qdd_pb_no_wrench = pb.forward_dynamics(q, qd, tau_drive)
        metrics["fd_sim_vs_pb_no_wrench_max"] = float(
            np.linalg.norm(qdd_sim_no_wrench - qdd_pb_no_wrench)
        )
        metrics["fd_pb_no_wrench_norm"] = float(np.linalg.norm(qdd_pb_no_wrench))
    else:
        m = robot.dynamics.mass_matrix(q)
        c = robot.dynamics.coriolis_gravity(q, qd)
        jtf = _wrench_joint_torque(robot, q, qd, f_ext)
        metrics["equation_max"] = float(np.linalg.norm(m @ qdd - (tau_drive - c - jtf)))
        id_total = robot.inverse_dynamics(q, qd, qdd, f_ext)["tau_total"]
        metrics["id_total_consistency_max"] = float(
            np.linalg.norm(id_total - (m @ qdd + c + jtf))
        )

    return metrics


def _print_task3(metrics: dict[str, float]) -> None:
    print("Task 3 — Inverse dynamics")
    print("  (1) tau_total = tau_drive + tau_constraint")
    print(f"      max err: {metrics['decomposition_max']:.3e} Nm")
    print("  (2) tau_drive = M(q) qdd + C(q, qd)")
    print(f"      max err: {metrics['equation_of_motion_max']:.3e} Nm")
    print("  (3) tau_constraint = ID(q,qd,qdd,f_ext) - ID(q,qd,qdd,0)")
    print(f"      max err: {metrics['wrench_mapping_max']:.3e} Nm")
    print("  (4) Recompute ID matches saved tau_total")
    print(f"      max err: {metrics['id_recompute_max']:.3e} Nm")
    print("  (5) ID -> FD round trip (theory): FD(q,qd,tau_total,f_ext) ?= qdd")
    print(f"      max err: {metrics['id_fd_roundtrip_max']:.3e} rad/s^2")


def _print_task4(metrics: dict[str, float]) -> None:
    print("Task 4 — Forward dynamics")
    print("  (1) Recompute FD(q, qd, tau_drive, f_ext) ?= saved qdd")
    print(f"      max err: {metrics['fd_recompute_max']:.3e} rad/s^2")
    if "fd_sim_vs_pb_no_wrench_max" in metrics:
        print("  (2) Simulator FD vs PyBullet (no EE wrench, reference)")
        print(f"      max err: {metrics['fd_sim_vs_pb_no_wrench_max']:.3e} rad/s^2")
        print(f"      qdd norm: {metrics['fd_pb_no_wrench_norm']:.3e} rad/s^2")
    else:
        print("  (2) M(q) qdd = tau_drive - C(q,qd) - J^T f_ext  [theoretical FD]")
        print(f"      max err: {metrics['equation_max']:.3e}")
        print("  (3) tau_total from ID equals M qdd + C + J^T f_ext")
        print(f"      max err: {metrics['id_total_consistency_max']:.3e} Nm")


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify inverse / forward dynamics results")
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=ROOT / "results",
        help="Directory containing *.npz files",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results" / "dynamics_verification.npz",
        help="Save numeric verification metrics",
    )
    args = parser.parse_args()

    robot = IRB1300Robot()
    id_path = args.results_dir / "inverse_dynamics.npz"
    fd_path = args.results_dir / "forward_dynamics.npz"
    _require_file(id_path)
    _require_file(fd_path)

    id_metrics = verify_inverse_dynamics(robot, id_path)
    fd_metrics = verify_forward_dynamics(robot, fd_path)

    print("=== Dynamics verification ===\n")
    _print_task3(id_metrics)
    print()
    _print_task4(fd_metrics)

    tol = 1e-6
    id_ok = all(id_metrics[k] < tol for k in id_metrics)
    fd_keys = ["fd_recompute_max"]
    if not np.isnan(fd_metrics.get("equation_max", float("nan"))):
        fd_keys.extend(["equation_max", "id_total_consistency_max"])
    fd_ok = all(fd_metrics[k] < tol for k in fd_keys)

    print()
    print(f"Task 3 overall: {'PASS' if id_ok else 'FAIL'}")
    print(f"Task 4 overall: {'PASS' if fd_ok else 'FAIL'}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        **{f"id_{k}": v for k, v in id_metrics.items()},
        **{f"fd_{k}": v for k, v in fd_metrics.items()},
    )
    print(f"\nMetrics saved to {args.output}")


if __name__ == "__main__":
    main()
