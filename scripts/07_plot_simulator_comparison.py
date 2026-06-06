#!/usr/bin/env python3
"""Plot theory-vs-simulator comparisons for all four tasks."""

from __future__ import annotations

import argparse
import importlib.util
import sys
import types
import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")


def _patch_mpl_toolkits_3d() -> bool:
    """Load mpl_toolkits.mplot3d from the same install as matplotlib."""
    mpl_root = Path(matplotlib.__file__).resolve().parent.parent
    mplot3d_init = mpl_root / "mpl_toolkits" / "mplot3d" / "__init__.py"
    if not mplot3d_init.is_file():
        return False

    try:
        for name in list(sys.modules):
            if name == "mpl_toolkits" or name.startswith("mpl_toolkits."):
                del sys.modules[name]

        toolkits_pkg = types.ModuleType("mpl_toolkits")
        toolkits_pkg.__path__ = [str(mpl_root / "mpl_toolkits")]
        sys.modules["mpl_toolkits"] = toolkits_pkg

        spec_mplot3d = importlib.util.spec_from_file_location(
            "mpl_toolkits.mplot3d",
            str(mplot3d_init),
            submodule_search_locations=[str(mplot3d_init.parent)],
        )
        mod_mplot3d = importlib.util.module_from_spec(spec_mplot3d)
        sys.modules["mpl_toolkits.mplot3d"] = mod_mplot3d
        spec_mplot3d.loader.exec_module(mod_mplot3d)

        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        return True
    except Exception:
        return False


HAS_3D = _patch_mpl_toolkits_3d()
if not HAS_3D:
    warnings.filterwarnings(
        "ignore",
        message="Unable to import Axes3D.*",
        category=UserWarning,
    )

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from irb1300_kin_dyn.pybullet_backend import pybullet_available
from irb1300_kin_dyn.robot import IRB1300Robot

JOINT_COLORS = plt.cm.tab10(np.linspace(0, 1, 6))


def _require_pybullet() -> None:
    if not pybullet_available():
        raise RuntimeError("pybullet is not installed. Run: python3 -m pip install pybullet")


def _require_file(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}. Run python3 scripts/run_all.py first.")


def _joint_labels(names: np.ndarray) -> list[str]:
    return [str(n) for n in names]


def _angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a - b + np.pi) % (2.0 * np.pi) - np.pi


def _can_use_3d_axes() -> bool:
    if not HAS_3D:
        return False
    try:
        fig = plt.figure()
        fig.add_subplot(111, projection="3d")
        plt.close(fig)
        return True
    except Exception:
        return False


def _set_3d_limits(ax, xyz_list: list[np.ndarray]) -> None:
    xyz = np.vstack(xyz_list)
    span = np.array([
        xyz[:, 0].max() - xyz[:, 0].min(),
        xyz[:, 1].max() - xyz[:, 1].min(),
        xyz[:, 2].max() - xyz[:, 2].min(),
    ])
    mid = xyz.mean(axis=0)
    radius = max(float(span.max()) * 0.5, 0.05)
    ax.set_xlim(mid[0] - radius, mid[0] + radius)
    ax.set_ylim(mid[1] - radius, mid[1] + radius)
    ax.set_zlim(mid[2] - radius, mid[2] + radius)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def _plot_ee_comparison_3d(
    ax,
    xyz_a: np.ndarray,
    xyz_b: np.ndarray,
    label_a: str,
    label_b: str,
    title: str,
) -> None:
    ax.plot(xyz_a[:, 0], xyz_a[:, 1], xyz_a[:, 2], label=label_a, linewidth=1.8)
    ax.plot(xyz_b[:, 0], xyz_b[:, 1], xyz_b[:, 2], "--", label=label_b, linewidth=1.6)
    ax.scatter(
        xyz_a[0, 0], xyz_a[0, 1], xyz_a[0, 2],
        c="limegreen", edgecolors="darkgreen", marker="o", s=70, label="start",
    )
    ax.scatter(
        xyz_a[-1, 0], xyz_a[-1, 1], xyz_a[-1, 2],
        c="red", edgecolors="darkred", marker="s", s=65, label="end",
    )
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title)
    ax.legend(fontsize=8)
    _set_3d_limits(ax, [xyz_a, xyz_b])


def _sim_fk(robot: IRB1300Robot, q_traj: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pb = robot._sim_backend
    assert pb is not None
    xyz = []
    rpy = []
    zeros = np.zeros(robot.n_joints)
    for q in q_traj:
        pb.set_state(q, zeros)
        link_state = p.getLinkState(
            pb.body_id,
            pb.ee_link_index,
            computeForwardKinematics=True,
            physicsClientId=pb._client,
        )
        xyz.append(np.asarray(link_state[4], dtype=float))
        rpy.append(np.asarray(p.getEulerFromQuaternion(link_state[5]), dtype=float))
    return np.vstack(xyz), np.vstack(rpy)


def _save_task1(robot: IRB1300Robot, results_dir: Path, out_dir: Path) -> tuple[Path, float]:
    path = results_dir / "forward_kinematics.npz"
    _require_file(path)
    d = np.load(path)
    q_traj = d["q_traj"]
    xyz_calc = d["xyz_traj"]
    rpy_calc = d["rpy_traj"]
    xyz_sim, rpy_sim = _sim_fk(robot, q_traj)
    rpy_calc_plot = np.unwrap(rpy_calc, axis=0)
    rpy_sim_plot = np.unwrap(rpy_sim, axis=0)
    pos_err = np.linalg.norm(xyz_calc - xyz_sim, axis=1)
    rpy_err = np.linalg.norm(_angle_diff(rpy_calc, rpy_sim), axis=1)
    steps = np.arange(q_traj.shape[0])

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Task 1: Forward Kinematics - Calculation vs PyBullet", fontweight="bold")

    ax1 = fig.add_subplot(2, 2, 1)
    for i, label in enumerate(["x", "y", "z"]):
        ax1.plot(steps, xyz_calc[:, i], label=f"calc {label}")
        ax1.plot(steps, xyz_sim[:, i], "--", label=f"sim {label}")
    ax1.set_xlabel("Step")
    ax1.set_ylabel("Position [m]")
    ax1.set_title("End-effector position")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8, ncol=2)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.semilogy(steps, np.maximum(pos_err, 1e-16), "r-o", markersize=3, label="position")
    ax2.semilogy(steps, np.maximum(rpy_err, 1e-16), "k-s", markersize=3, label="RPY")
    ax2.set_xlabel("Step")
    ax2.set_ylabel("Error norm [m or rad]")
    ax2.set_title("Simulator comparison error")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    ax3 = fig.add_subplot(2, 2, 3)
    for i, label in enumerate(["roll", "pitch", "yaw"]):
        ax3.plot(steps, np.degrees(rpy_calc_plot[:, i]), label=f"calc {label}")
        ax3.plot(steps, np.degrees(rpy_sim_plot[:, i]), "--", label=f"sim {label}")
    ax3.set_xlabel("Step")
    ax3.set_ylabel("Angle [deg]")
    ax3.set_title("End-effector orientation")
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8, ncol=2)

    if _can_use_3d_axes():
        ax4 = fig.add_subplot(2, 2, 4, projection="3d")
        _plot_ee_comparison_3d(
            ax4,
            xyz_calc,
            xyz_sim,
            "calc EE path",
            "sim EE path",
            "End-effector 3D path",
        )
    else:
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(xyz_calc[:, 0], xyz_calc[:, 1], label="calc EE path")
        ax4.plot(xyz_sim[:, 0], xyz_sim[:, 1], "--", label="sim EE path")
        ax4.scatter(xyz_calc[0, 0], xyz_calc[0, 1], c="limegreen", edgecolors="darkgreen", label="start")
        ax4.scatter(xyz_calc[-1, 0], xyz_calc[-1, 1], c="red", edgecolors="darkred", marker="s", label="end")
        ax4.set_xlabel("X [m]")
        ax4.set_ylabel("Y [m]")
        ax4.set_title("End-effector XY path (3D unavailable)")
        ax4.set_aspect("equal", adjustable="box")
        ax4.grid(True, alpha=0.3)
        ax4.legend(fontsize=8)

    fig.tight_layout()
    out = out_dir / "01_forward_kinematics_vs_pybullet.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out, float(np.max(pos_err))


def _save_task2(robot: IRB1300Robot, results_dir: Path, out_dir: Path) -> tuple[Path, float]:
    path = results_dir / "inverse_kinematics.npz"
    _require_file(path)
    d = np.load(path)
    pose_target = d["pose_traj"]
    q_ik = d["q_traj_ik"]
    pos_err_calc = d["position_error"]
    names = _joint_labels(d["joint_names"])
    xyz_target = pose_target[:, :3, 3]
    xyz_sim, _ = _sim_fk(robot, q_ik)
    pos_err_sim = np.linalg.norm(xyz_sim - xyz_target, axis=1)
    steps = np.arange(q_ik.shape[0])

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("Task 2: Inverse Kinematics - Calculation vs PyBullet Replay", fontweight="bold")

    ax1 = fig.add_subplot(2, 2, 1)
    for j, name in enumerate(names):
        ax1.plot(steps, q_ik[:, j], color=JOINT_COLORS[j], label=name)
    ax1.set_xlabel("Step")
    ax1.set_ylabel("Joint angle [rad]")
    ax1.set_title("Calculated IK joint trajectory replayed in simulator")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8, ncol=2)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.semilogy(steps, np.maximum(pos_err_calc, 1e-16), "r-o", markersize=3, label="calc FK vs target")
    ax2.semilogy(steps, np.maximum(pos_err_sim, 1e-16), "k-s", markersize=3, label="sim FK vs target")
    ax2.set_xlabel("Step")
    ax2.set_ylabel("Position error [m]")
    ax2.set_title("IK target tracking error")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    ax3 = fig.add_subplot(2, 2, 3)
    for i, label in enumerate(["x", "y", "z"]):
        ax3.plot(steps, xyz_target[:, i], label=f"target {label}")
        ax3.plot(steps, xyz_sim[:, i], "--", label=f"sim {label}")
    ax3.set_xlabel("Step")
    ax3.set_ylabel("Position [m]")
    ax3.set_title("Target and simulator end-effector position")
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8, ncol=2)

    if _can_use_3d_axes():
        ax4 = fig.add_subplot(2, 2, 4, projection="3d")
        _plot_ee_comparison_3d(
            ax4,
            xyz_target,
            xyz_sim,
            "target path",
            "sim replay path",
            "End-effector 3D path",
        )
    else:
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(xyz_target[:, 0], xyz_target[:, 1], "b--", label="target path")
        ax4.plot(xyz_sim[:, 0], xyz_sim[:, 1], "r-", label="sim replay path")
        ax4.set_xlabel("X [m]")
        ax4.set_ylabel("Y [m]")
        ax4.set_title("End-effector XY path (3D unavailable)")
        ax4.set_aspect("equal", adjustable="box")
        ax4.grid(True, alpha=0.3)
        ax4.legend(fontsize=8)

    fig.tight_layout()
    out = out_dir / "02_inverse_kinematics_vs_pybullet.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out, float(np.max(pos_err_sim))


def _save_task3(robot: IRB1300Robot, results_dir: Path, out_dir: Path) -> tuple[Path, float]:
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

    tau_drive_sim = []
    tau_constraint_sim = []
    tau_total_sim = []
    for qi, qdi, qddi in zip(q, qd, qdd):
        drive = pb.inverse_dynamics(qi, qdi, qddi)
        wrench = pb.wrench_to_joint_torques(qi, qdi, f_ext)
        tau_drive_sim.append(drive)
        tau_constraint_sim.append(wrench)
        tau_total_sim.append(drive + wrench)
    tau_drive_sim = np.vstack(tau_drive_sim)
    tau_constraint_sim = np.vstack(tau_constraint_sim)
    tau_total_sim = np.vstack(tau_total_sim)
    drive_err = np.linalg.norm(tau_drive - tau_drive_sim, axis=1)
    constraint_err = np.linalg.norm(tau_constraint - tau_constraint_sim, axis=1)
    total_err = np.linalg.norm(tau_total - tau_total_sim, axis=1)
    steps = np.arange(q.shape[0])

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Task 3: Inverse Dynamics - Calculation vs PyBullet", fontweight="bold")

    ax1 = fig.add_subplot(2, 2, 1)
    for j, name in enumerate(names):
        ax1.plot(steps, tau_drive[:, j], color=JOINT_COLORS[j], label=f"calc {name}")
        ax1.plot(steps, tau_drive_sim[:, j], "--", color=JOINT_COLORS[j], alpha=0.75)
    ax1.set_xlabel("Step")
    ax1.set_ylabel("Torque [Nm]")
    ax1.set_title("Drive torque: solid=calc, dashed=sim")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=7, ncol=2)

    ax2 = fig.add_subplot(2, 2, 2)
    for j, name in enumerate(names):
        ax2.plot(steps, tau_constraint[:, j], color=JOINT_COLORS[j], label=f"calc {name}")
        ax2.plot(steps, tau_constraint_sim[:, j], "--", color=JOINT_COLORS[j], alpha=0.75)
    ax2.set_xlabel("Step")
    ax2.set_ylabel("Torque [Nm]")
    ax2.set_title("Constraint torque: solid=calc, dashed=sim J^T f")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=7, ncol=2)

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(steps, drive_err, "r-o", markersize=3, label="drive torque")
    ax3.plot(steps, constraint_err, "b-^", markersize=3, label="constraint torque")
    ax3.plot(steps, total_err, "k-s", markersize=3, label="total torque")
    ax3.set_xlabel("Step")
    ax3.set_ylabel("Error norm [Nm]")
    ax3.set_title("Torque error norm")
    ax3.grid(True, alpha=0.3)
    ax3.legend()

    ax4 = fig.add_subplot(2, 2, 4)
    width = 0.25
    x = np.arange(len(names))
    ax4.bar(x - width, np.max(np.abs(tau_drive - tau_drive_sim), axis=0), width, label="drive")
    ax4.bar(x, np.max(np.abs(tau_constraint - tau_constraint_sim), axis=0), width, label="constraint")
    ax4.bar(x + width, np.max(np.abs(tau_total - tau_total_sim), axis=0), width, label="total")
    ax4.set_xticks(x)
    ax4.set_xticklabels(names, rotation=30)
    ax4.set_ylabel("Max abs error [Nm]")
    ax4.set_title("Per-joint maximum torque error")
    ax4.grid(True, axis="y", alpha=0.3)
    ax4.legend()

    fig.tight_layout()
    out = out_dir / "03_inverse_dynamics_vs_pybullet.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out, float(np.max(total_err))


def _save_task4(robot: IRB1300Robot, results_dir: Path, out_dir: Path) -> tuple[Path, float]:
    path = results_dir / "forward_dynamics.npz"
    _require_file(path)
    d = np.load(path)
    q = d["q"]
    qd = d["qd"]
    qdd_calc = d["qdd"]
    tau_drive = d["tau_drive"]
    f_ext = d["f_ext_ee"]
    names = _joint_labels(d["joint_names"])
    qdd_sim = robot.forward_dynamics(q, qd, tau_drive, f_ext)
    err = qdd_calc - qdd_sim
    x = np.arange(len(names))

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    fig.suptitle("Task 4: Forward Dynamics - Calculation vs PyBullet", fontweight="bold")

    width = 0.38
    axes[0].bar(x - width / 2, qdd_calc, width, label="calc", color="tab:blue")
    axes[0].bar(x + width / 2, qdd_sim, width, label="sim", color="tab:orange")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(names, rotation=30)
    axes[0].set_ylabel("Acceleration [rad/s^2]")
    axes[0].set_title("Joint acceleration output")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[0].legend()

    axes[1].bar(x, err, color=JOINT_COLORS)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(names, rotation=30)
    axes[1].set_ylabel("calc - sim [rad/s^2]")
    axes[1].set_title("Acceleration error")
    axes[1].grid(True, axis="y", alpha=0.3)

    axes[2].bar(x, tau_drive, color=JOINT_COLORS)
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(names, rotation=30)
    axes[2].set_ylabel("Torque [Nm]")
    axes[2].set_title("Drive torque input")
    axes[2].grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    out = out_dir / "04_forward_dynamics_vs_pybullet.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out, float(np.linalg.norm(err))


def _save_summary(metrics: dict[str, float], out_dir: Path) -> Path:
    labels = list(metrics)
    values = [metrics[k] for k in labels]
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(11, 5.5))
    bars = ax.bar(x, values, color=["tab:blue", "tab:green", "tab:orange", "tab:red"])
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha="right")
    ax.set_yscale("log")
    ax.set_ylabel("Max comparison error (task units)")
    ax.set_title("Theory vs PyBullet Summary")
    ax.grid(True, axis="y", alpha=0.3)
    ax.bar_label(bars, labels=[f"{v:.2e}" for v in values], padding=3, fontsize=9)
    fig.tight_layout()
    out = out_dir / "00_summary_vs_pybullet.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot calculation-vs-PyBullet comparison figures for all four tasks"
    )
    parser.add_argument("--results-dir", type=Path, default=ROOT / "results")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "results" / "figures" / "simulator_comparison",
    )
    args = parser.parse_args()

    _require_pybullet()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    robot = IRB1300Robot(use_simulator_fd=True)
    if not robot.use_simulator_fd:
        raise RuntimeError("Failed to initialize PyBullet backend")

    saved: list[Path] = []
    metrics: dict[str, float] = {}
    try:
        out, metrics["Task 1 FK pos"] = _save_task1(robot, args.results_dir, args.output_dir)
        saved.append(out)
        out, metrics["Task 2 IK pos"] = _save_task2(robot, args.results_dir, args.output_dir)
        saved.append(out)
        out, metrics["Task 3 ID torque"] = _save_task3(robot, args.results_dir, args.output_dir)
        saved.append(out)
        out, metrics["Task 4 FD qdd"] = _save_task4(robot, args.results_dir, args.output_dir)
        saved.append(out)
        saved.insert(0, _save_summary(metrics, args.output_dir))
    finally:
        if robot._sim_backend is not None:
            robot._sim_backend.close()

    for path in saved:
        print(f"Saved {path}")
    print("\nMaximum comparison errors:")
    for name, value in metrics.items():
        print(f"  {name}: {value:.6e}")


if __name__ == "__main__":
    main()
