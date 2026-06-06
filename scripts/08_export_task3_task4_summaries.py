#!/usr/bin/env python3
"""Export readable Task 3/4 input-output summaries with PyBullet comparisons."""

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
        raise RuntimeError("pybullet is not installed. Run: python3 -m pip install pybullet")


def _require_file(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}. Run python3 scripts/run_all.py first.")


def _joint_labels(names: np.ndarray) -> list[str]:
    return [str(n) for n in names]


def _fmt(value: float) -> str:
    return f"{float(value):.6g}"


def _vec(values: np.ndarray) -> str:
    return "[" + ", ".join(_fmt(v) for v in np.asarray(values, dtype=float)) + "]"


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _task3_summary(robot: IRB1300Robot, results_dir: Path) -> str:
    path = results_dir / "inverse_dynamics.npz"
    _require_file(path)
    d = np.load(path)

    q = d["q_traj"]
    qd = d["qd_traj"]
    qdd = d["qdd_traj"]
    f_ext = d["f_ext_ee"]
    tau_drive = d["tau_drive"]
    tau_constraint = d["tau_constraint"]
    tau_total = d["tau_total"]
    names = _joint_labels(d["joint_names"])

    pb = robot._sim_backend
    assert pb is not None

    tau_drive_pb = []
    tau_constraint_pb = []
    tau_total_pb = []
    for qi, qdi, qddi in zip(q, qd, qdd):
        drive_pb = pb.inverse_dynamics(qi, qdi, qddi)
        constraint_pb = pb.wrench_to_joint_torques(qi, qdi, f_ext)
        tau_drive_pb.append(drive_pb)
        tau_constraint_pb.append(constraint_pb)
        tau_total_pb.append(drive_pb + constraint_pb)

    tau_drive_pb = np.vstack(tau_drive_pb)
    tau_constraint_pb = np.vstack(tau_constraint_pb)
    tau_total_pb = np.vstack(tau_total_pb)

    drive_err = tau_drive - tau_drive_pb
    constraint_err = tau_constraint - tau_constraint_pb
    total_err = tau_total - tau_total_pb
    decomp_err = tau_total - tau_drive - tau_constraint

    lines = [
        "# Task 3 Inverse Dynamics Summary",
        "",
        "Goal: input joint position, velocity, acceleration and end-effector wrench; output drive torque and constraint torque.",
        "",
        "## Inputs",
        "",
        f"- Joint order: `{', '.join(names)}`",
        f"- End-effector wrench `[Fx, Fy, Fz, Mx, My, Mz]`: `{_vec(f_ext)}`",
        f"- Number of trajectory steps: `{q.shape[0]}`",
        "",
        "| Step | q [rad] | qd [rad/s] | qdd [rad/s^2] | f_ext `[Fx,Fy,Fz,Mx,My,Mz]` |",
        "|---:|---|---|---|---|",
    ]
    for i in range(q.shape[0]):
        lines.append(
            f"| {i} | `{_vec(q[i])}` | `{_vec(qd[i])}` | `{_vec(qdd[i])}` | `{_vec(f_ext)}` |"
        )

    lines.extend(
        [
            "",
            "## Outputs And Simulator Comparison",
            "",
            "Calculation columns are produced by the project recursive Newton-Euler implementation. Simulator columns are produced by PyBullet using the same URDF.",
            "",
            "| Step | drive calc [Nm] | drive sim [Nm] | constraint calc [Nm] | constraint sim J^T f [Nm] | total calc [Nm] | total sim [Nm] | error norms `[drive, constraint, total]` |",
            "|---:|---|---|---|---|---|---|---|",
        ]
    )
    for i in range(q.shape[0]):
        err_norms = np.array(
            [
                np.linalg.norm(drive_err[i]),
                np.linalg.norm(constraint_err[i]),
                np.linalg.norm(total_err[i]),
            ]
        )
        lines.append(
            f"| {i} | `{_vec(tau_drive[i])}` | `{_vec(tau_drive_pb[i])}` | "
            f"`{_vec(tau_constraint[i])}` | `{_vec(tau_constraint_pb[i])}` | "
            f"`{_vec(tau_total[i])}` | `{_vec(tau_total_pb[i])}` | `{_vec(err_norms)}` |"
        )

    lines.extend(
        [
            "",
            "## Error Summary",
            "",
            "| Quantity | Mean norm | Max norm | Max absolute joint error |",
            "|---|---:|---:|---:|",
            f"| drive torque calc - sim | {_fmt(np.mean(np.linalg.norm(drive_err, axis=1)))} | {_fmt(np.max(np.linalg.norm(drive_err, axis=1)))} | {_fmt(np.max(np.abs(drive_err)))} |",
            f"| constraint torque calc - sim | {_fmt(np.mean(np.linalg.norm(constraint_err, axis=1)))} | {_fmt(np.max(np.linalg.norm(constraint_err, axis=1)))} | {_fmt(np.max(np.abs(constraint_err)))} |",
            f"| total torque calc - sim | {_fmt(np.mean(np.linalg.norm(total_err, axis=1)))} | {_fmt(np.max(np.linalg.norm(total_err, axis=1)))} | {_fmt(np.max(np.abs(total_err)))} |",
            f"| total - drive - constraint | {_fmt(np.mean(np.linalg.norm(decomp_err, axis=1)))} | {_fmt(np.max(np.linalg.norm(decomp_err, axis=1)))} | {_fmt(np.max(np.abs(decomp_err)))} |",
            "",
            "## Per-Joint Maximum Absolute Error",
            "",
            "| Joint | drive [Nm] | constraint [Nm] | total [Nm] |",
            "|---|---:|---:|---:|",
        ]
    )
    for j, name in enumerate(names):
        lines.append(
            f"| {name} | {_fmt(np.max(np.abs(drive_err[:, j])))} | "
            f"{_fmt(np.max(np.abs(constraint_err[:, j])))} | "
            f"{_fmt(np.max(np.abs(total_err[:, j])))} |"
        )

    return "\n".join(lines) + "\n"


def _task4_summary(robot: IRB1300Robot, results_dir: Path) -> str:
    path = results_dir / "forward_dynamics.npz"
    _require_file(path)
    d = np.load(path)

    q = d["q"]
    qd = d["qd"]
    qdd = d["qdd"]
    qdd_target = d["qdd_target"] if "qdd_target" in d.files else np.full_like(qdd, np.nan)
    tau_drive = d["tau_drive"]
    f_ext = d["f_ext_ee"]
    names = _joint_labels(d["joint_names"])

    qdd_theory = robot.forward_dynamics_theoretical(q, qd, tau_drive, f_ext)
    qdd_pb = robot.forward_dynamics(q, qd, tau_drive, f_ext)
    saved_err = qdd - qdd_pb
    theory_err = qdd_theory - qdd_pb

    lines = [
        "# Task 4 Forward Dynamics Summary",
        "",
        "Goal: input joint position, velocity, drive torque and end-effector wrench; output joint acceleration.",
        "",
        "## Inputs And Outputs",
        "",
        f"- Joint order: `{', '.join(names)}`",
        f"- End-effector wrench `[Fx, Fy, Fz, Mx, My, Mz]`: `{_vec(f_ext)}`",
        "",
        "| Joint | q [rad] | qd [rad/s] | drive torque [Nm] | target qdd [rad/s^2] | calc qdd [rad/s^2] | sim qdd [rad/s^2] | calc - sim [rad/s^2] |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for j, name in enumerate(names):
        lines.append(
            f"| {name} | {_fmt(q[j])} | {_fmt(qd[j])} | {_fmt(tau_drive[j])} | "
            f"{_fmt(qdd_target[j])} | {_fmt(qdd[j])} | {_fmt(qdd_pb[j])} | {_fmt(saved_err[j])} |"
        )

    lines.extend(
        [
            "",
            "## Vector Form",
            "",
            f"- q [rad]: `{_vec(q)}`",
            f"- qd [rad/s]: `{_vec(qd)}`",
            f"- drive torque [Nm]: `{_vec(tau_drive)}`",
            f"- target qdd [rad/s^2]: `{_vec(qdd_target)}`",
            f"- calculated qdd [rad/s^2]: `{_vec(qdd)}`",
            f"- simulator qdd [rad/s^2]: `{_vec(qdd_pb)}`",
            "",
            "## Error Summary",
            "",
            "| Quantity | Norm | Max absolute joint error |",
            "|---|---:|---:|",
            f"| saved calc qdd - sim qdd | {_fmt(np.linalg.norm(saved_err))} | {_fmt(np.max(np.abs(saved_err)))} |",
            f"| recomputed theoretical qdd - sim qdd | {_fmt(np.linalg.norm(theory_err))} | {_fmt(np.max(np.abs(theory_err)))} |",
            f"| saved calc qdd - target qdd | {_fmt(np.linalg.norm(qdd - qdd_target))} | {_fmt(np.max(np.abs(qdd - qdd_target)))} |",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Export readable Task 3/4 summaries")
    parser.add_argument("--results-dir", type=Path, default=ROOT / "results")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results" / "summaries")
    args = parser.parse_args()

    _require_pybullet()
    robot = IRB1300Robot(use_simulator_fd=True)
    if not robot.use_simulator_fd:
        raise RuntimeError("Failed to initialize PyBullet backend")

    try:
        task3_path = args.output_dir / "task3_inverse_dynamics_summary.md"
        task4_path = args.output_dir / "task4_forward_dynamics_summary.md"
        _write(task3_path, _task3_summary(robot, args.results_dir))
        _write(task4_path, _task4_summary(robot, args.results_dir))
    finally:
        if robot._sim_backend is not None:
            robot._sim_backend.close()

    print(f"Saved {task3_path}")
    print(f"Saved {task4_path}")


if __name__ == "__main__":
    main()
