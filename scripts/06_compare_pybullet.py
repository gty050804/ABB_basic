#!/usr/bin/env python3
"""Compare dynamics results against PyBullet (same URDF as simulation)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from irb1300_kin_dyn.pybullet_backend import pybullet_available
from irb1300_kin_dyn.robot import IRB1300Robot


def _require_pybullet() -> None:
    if not pybullet_available():
        raise RuntimeError("pybullet is not installed. Run: pip install pybullet")


def _joint_labels(names: np.ndarray) -> list[str]:
    return [str(n) for n in names]


def print_joint_torque_table(
    step: int,
    names: list[str],
    tau_drive: np.ndarray,
    tau_constraint: np.ndarray,
    tau_total: np.ndarray,
    tau_theory_jtf: np.ndarray,
    tau_pb_drive: np.ndarray,
    tau_pb_wrench: np.ndarray,
) -> None:
    tau_pb_total = tau_pb_drive + tau_pb_wrench
    print(f"\n--- Step {step} ---")
    print(
        f"{'Joint':<8} "
        f"{'drive TH':>10} {'drive PB':>10} {'d diff':>10} | "
        f"{'constr TH':>10} {'J^Tf TH':>10} {'J^Tf PB':>10} | "
        f"{'total TH':>10} {'total PB':>10} {'t diff':>10}"
    )
    print("-" * 118)
    for j, name in enumerate(names):
        print(
            f"{name:<8} "
            f"{tau_drive[j]:10.4f} {tau_pb_drive[j]:10.4f} {tau_drive[j] - tau_pb_drive[j]:10.4f} | "
            f"{tau_constraint[j]:10.4f} {tau_theory_jtf[j]:10.4f} {tau_pb_wrench[j]:10.4f} | "
            f"{tau_total[j]:10.4f} {tau_pb_total[j]:10.4f} {tau_total[j] - tau_pb_total[j]:10.4f}"
        )
    print(
        f"{'||.||':<8} "
        f"{np.linalg.norm(tau_drive):10.4f} {np.linalg.norm(tau_pb_drive):10.4f} "
        f"{np.linalg.norm(tau_drive - tau_pb_drive):10.4f} | "
        f"{np.linalg.norm(tau_constraint):10.4f} {np.linalg.norm(tau_theory_jtf):10.4f} "
        f"{np.linalg.norm(tau_pb_wrench):10.4f} | "
        f"{np.linalg.norm(tau_total):10.4f} {np.linalg.norm(tau_pb_total):10.4f} "
        f"{np.linalg.norm(tau_total - tau_pb_total):10.4f}"
    )


def compare_task3(robot: IRB1300Robot, path: Path, print_tables: bool = True) -> dict[str, float]:
    d = np.load(path)
    q_traj = d["q_traj"]
    qd_traj = d["qd_traj"]
    qdd_traj = d["qdd_traj"]
    f_ext = d["f_ext_ee"]
    tau_drive = d["tau_drive"]
    tau_constraint = d["tau_constraint"]
    tau_total = d["tau_total"]
    names = _joint_labels(d["joint_names"])

    pb = robot._sim_backend
    assert pb is not None

    id_errs = []
    fd_theory_errs = []
    fd_pb_errs = []
    jtf_errs = []
    total_errs = []

    if print_tables:
        print("\n=== Per-joint torques: theory vs PyBullet ===")
        print("drive TH/PB     = inverse dynamics without EE wrench")
        print("constr TH       = ID(with wrench) - ID(without); includes qdd coupling")
        print("J^Tf TH/PB      = static wrench mapping J(q)^T f_ext")
        print("total PB        = drive PB + J^Tf PB")

    for i in range(len(q_traj)):
        q, qd, qdd = q_traj[i], qd_traj[i], qdd_traj[i]
        tau_pb_drive = pb.inverse_dynamics(q, qd, qdd)
        tau_pb_wrench = pb.wrench_to_joint_torques(q, qd, f_ext)
        tau_theory_jtf = robot.inverse_dynamics(
            q, qd, np.zeros(robot.n_joints), f_ext
        )["tau_constraint"]
        tau_pb_total = tau_pb_drive + tau_pb_wrench

        id_errs.append(np.linalg.norm(tau_drive[i] - tau_pb_drive))
        jtf_errs.append(np.linalg.norm(tau_theory_jtf - tau_pb_wrench))
        total_errs.append(np.linalg.norm(tau_total[i] - tau_pb_total))

        qdd_theory = robot.forward_dynamics_theoretical(q, qd, tau_total[i], f_ext)
        m_pb = pb.mass_matrix(q)
        bias_pb = pb.bias_forces(q, qd)
        qdd_pb = np.linalg.solve(m_pb, tau_drive[i] - bias_pb)
        fd_theory_errs.append(np.linalg.norm(qdd_theory - qdd))
        fd_pb_errs.append(np.linalg.norm(qdd_pb - qdd))

        if print_tables:
            print_joint_torque_table(
                i,
                names,
                tau_drive[i],
                tau_constraint[i],
                tau_total[i],
                tau_theory_jtf,
                tau_pb_drive,
                tau_pb_wrench,
            )

    return {
        "id_tau_drive_max": float(np.max(id_errs)),
        "id_tau_drive_mean": float(np.mean(id_errs)),
        "jtf_theory_vs_pb_max": float(np.max(jtf_errs)),
        "jtf_theory_vs_pb_mean": float(np.mean(jtf_errs)),
        "id_tau_total_max": float(np.max(total_errs)),
        "id_tau_total_mean": float(np.mean(total_errs)),
        "fd_theory_roundtrip_max": float(np.max(fd_theory_errs)),
        "fd_pb_roundtrip_max": float(np.max(fd_pb_errs)),
    }


def compare_task4(robot: IRB1300Robot, path: Path) -> dict[str, float]:
    d = np.load(path)
    q = d["q"]
    qd = d["qd"]
    tau_drive = d["tau_drive"]
    f_ext = d["f_ext_ee"]

    qdd_saved = d["qdd"]
    qdd_theory = robot.forward_dynamics_theoretical(q, qd, tau_drive, f_ext)
    qdd_pb_fd = robot.forward_dynamics(q, qd, tau_drive, f_ext)

    return {
        "fd_saved_vs_sim_max": float(np.linalg.norm(qdd_saved - qdd_pb_fd)),
        "fd_theory_vs_sim_max": float(np.linalg.norm(qdd_theory - qdd_pb_fd)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare dynamics with PyBullet simulator")
    parser.add_argument("--results-dir", type=Path, default=ROOT / "results")
    parser.add_argument(
        "--no-tables",
        action="store_true",
        help="Skip per-joint torque tables (summary only)",
    )
    args = parser.parse_args()

    _require_pybullet()
    robot = IRB1300Robot(use_simulator_fd=True)
    if not robot.use_simulator_fd:
        raise RuntimeError("Failed to initialize PyBullet backend")

    id_path = args.results_dir / "inverse_dynamics.npz"
    fd_path = args.results_dir / "forward_dynamics.npz"
    for p in (id_path, fd_path):
        if not p.is_file():
            raise FileNotFoundError(f"Missing {p}. Run demo scripts first.")

    print("=== Dynamics comparison (same URDF) ===")
    print("TH = theoretical recursive Newton-Euler implementation used in Task 3/4")
    print("PB = PyBullet physics engine (same URDF as ROS simulation)")

    t3 = compare_task3(robot, id_path, print_tables=not args.no_tables)
    t4 = compare_task4(robot, fd_path)

    print("\n=== Summary ===")
    print("Task 3")
    print(
        f"  Drive torque   TH vs PB: mean {t3['id_tau_drive_mean']:.3f} Nm, "
        f"max {t3['id_tau_drive_max']:.3f} Nm"
    )
    print(
        f"  J^T f mapping  TH vs PB: mean {t3['jtf_theory_vs_pb_mean']:.3f} Nm, "
        f"max {t3['jtf_theory_vs_pb_max']:.3f} Nm"
    )
    print(
        f"  Total torque   TH vs PB (drive+J^T f): mean {t3['id_tau_total_mean']:.3f} Nm, "
        f"max {t3['id_tau_total_max']:.3f} Nm"
    )
    print(f"  FD round-trip TH (tau_total + f_ext -> qdd): max {t3['fd_theory_roundtrip_max']:.3e} rad/s^2")
    print(f"  FD round-trip PB (tau_drive -> qdd):         max {t3['fd_pb_roundtrip_max']:.3e} rad/s^2")

    print("\nTask 4")
    print(f"  Saved qdd vs PB-based FD: max {t4['fd_saved_vs_sim_max']:.3e} rad/s^2")
    print(f"  TH FD vs PB-based FD:     max {t4['fd_theory_vs_sim_max']:.3e} rad/s^2")

    if robot._sim_backend is not None:
        robot._sim_backend.close()


if __name__ == "__main__":
    main()
