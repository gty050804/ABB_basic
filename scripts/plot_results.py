#!/usr/bin/env python3
"""Plot trajectories from results/*.npz for all four simulation tasks."""

from __future__ import annotations

import argparse
import importlib.util
import sys
import types
from pathlib import Path

import matplotlib

matplotlib.use("Agg")


def _patch_mpl_toolkits_3d() -> bool:
    """Load mpl_toolkits.mplot3d from the same install as matplotlib.

    Ubuntu often ships matplotlib via apt while pip upgrades the core package,
    leaving an incompatible system mpl_toolkits that breaks 3D projection.
    """
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

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

JOINT_COLORS = plt.cm.tab10(np.linspace(0, 1, 6))


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


def _plot_start_end_markers_3d(ax, xyz: np.ndarray, offset_m: float = 0.025) -> None:
    """Plot distinguishable start/end markers even when positions coincide."""
    start = xyz[0]
    end = xyz[-1]
    closed = np.linalg.norm(start - end) < 1e-3

    if closed and len(xyz) > 2:
        tangent = xyz[1] - xyz[0]
        if np.linalg.norm(tangent) < 1e-9:
            tangent = xyz[-1] - xyz[-2]
        tangent = tangent / (np.linalg.norm(tangent) + 1e-12)
        start_plot = start - tangent * offset_m
        end_plot = end + tangent * offset_m
        note = " (closed path, markers offset for visibility)"
    else:
        start_plot = start
        end_plot = end
        note = ""

    ax.scatter(
        start_plot[0], start_plot[1], start_plot[2],
        c="limegreen", edgecolors="darkgreen", linewidths=1.5,
        marker="o", s=100, zorder=5, label=f"start{note}",
    )
    ax.scatter(
        end_plot[0], end_plot[1], end_plot[2],
        c="red", edgecolors="darkred", linewidths=1.5,
        marker="s", s=90, zorder=6, label=f"end{note}",
    )
    ax.text(start_plot[0], start_plot[1], start_plot[2], "  S", color="darkgreen", fontsize=9)
    ax.text(end_plot[0], end_plot[1], end_plot[2], "  E", color="darkred", fontsize=9)


def _plot_ee_trajectory_3d(ax, xyz: np.ndarray, title: str) -> None:
    ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], "b-", linewidth=1.8, label="EE path")
    _plot_start_end_markers_3d(ax, xyz)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title)
    ax.legend(fontsize=8, loc="upper left")

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


def _plot_start_end_markers_2d(ax, xy: np.ndarray, xyz: np.ndarray, offset_m: float = 0.025) -> None:
    start = xy[0]
    end = xy[-1]
    closed = np.linalg.norm(xyz[0] - xyz[-1]) < 1e-3
    if closed and len(xy) > 2:
        tangent = xy[1] - xy[0]
        if np.linalg.norm(tangent) < 1e-9:
            tangent = xy[-1] - xy[-2]
        tangent = tangent / (np.linalg.norm(tangent) + 1e-12) * offset_m
        start_plot = start - tangent
        end_plot = end + tangent
    else:
        start_plot = start
        end_plot = end
    ax.scatter(start_plot[0], start_plot[1], c="limegreen", edgecolors="darkgreen",
               s=80, marker="o", zorder=5, label="start")
    ax.scatter(end_plot[0], end_plot[1], c="red", edgecolors="darkred",
               s=70, marker="s", zorder=6, label="end")


def _save_orthographic_figure(xyz: np.ndarray, out_path: Path, title: str) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    fig.suptitle(title, fontsize=12, fontweight="bold")
    planes = [("XY", 0, 1), ("XZ", 0, 2), ("YZ", 1, 2)]
    for ax, (name, i, j) in zip(axes, planes):
        ax.plot(xyz[:, i], xyz[:, j], "b-", linewidth=1.5)
        _plot_start_end_markers_2d(ax, xyz[:, [i, j]], xyz)
        ax.set_xlabel(["X", "X", "Y"][planes.index((name, i, j))] + " [m]")
        ax.set_ylabel(["Y", "Z", "Z"][planes.index((name, i, j))] + " [m]")
        ax.set_title(name)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="box")
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _require_file(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(
            f"Missing {path.name}. Run the corresponding demo script first."
        )


def _joint_labels(names: np.ndarray) -> list[str]:
    return [str(n) for n in names]


def plot_forward_kinematics(results_dir: Path, out_dir: Path) -> list[Path]:
    path = results_dir / "forward_kinematics.npz"
    _require_file(path)
    d = np.load(path)
    q_traj = d["q_traj"]
    xyz = d["xyz_traj"]
    rpy = d["rpy_traj"]
    names = _joint_labels(d["joint_names"])
    steps = np.arange(q_traj.shape[0])
    has_3d = _can_use_3d_axes()

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Task 1: Forward Kinematics", fontsize=14, fontweight="bold")

    ax1 = fig.add_subplot(2, 2, 1)
    for j in range(q_traj.shape[1]):
        ax1.plot(steps, q_traj[:, j], color=JOINT_COLORS[j], label=names[j])
    ax1.set_xlabel("Step")
    ax1.set_ylabel("Joint angle [rad]")
    ax1.set_title("Joint trajectory (input)")
    ax1.legend(loc="upper right", fontsize=8, ncol=2)
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(2, 2, 2)
    for i, label in enumerate(["x", "y", "z"]):
        ax2.plot(steps, xyz[:, i], label=label)
    ax2.set_xlabel("Step")
    ax2.set_ylabel("Position [m]")
    ax2.set_title("End-effector position (output)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 3)
    for i, label in enumerate(["roll", "pitch", "yaw"]):
        ax3.plot(steps, np.degrees(rpy[:, i]), label=label)
    ax3.set_xlabel("Step")
    ax3.set_ylabel("Angle [deg]")
    ax3.set_title("End-effector orientation (RPY)")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    if has_3d:
        ax4 = fig.add_subplot(2, 2, 4, projection="3d")
        _plot_ee_trajectory_3d(ax4, xyz, "End-effector 3D trajectory")
    else:
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(xyz[:, 0], xyz[:, 1], "b-", linewidth=1.5, label="EE path")
        _plot_start_end_markers_2d(ax4, xyz[:, :2], xyz)
        ax4.set_xlabel("X [m]")
        ax4.set_ylabel("Y [m]")
        ax4.set_title("End-effector XY (3D unavailable)")
        ax4.legend(fontsize=8)
        ax4.grid(True, alpha=0.3)
        ax4.set_aspect("equal", adjustable="box")

    fig.tight_layout()
    out_summary = out_dir / "01_forward_kinematics.png"
    fig.savefig(out_summary, dpi=150, bbox_inches="tight")
    plt.close(fig)

    outputs = [out_summary]
    if has_3d:
        fig3d = plt.figure(figsize=(9, 8))
        ax = fig3d.add_subplot(111, projection="3d")
        _plot_ee_trajectory_3d(ax, xyz, "Task 1: End-effector 3D trajectory")
        fig3d.tight_layout()
        out_3d = out_dir / "01_forward_kinematics_3d.png"
        fig3d.savefig(out_3d, dpi=160, bbox_inches="tight")
        plt.close(fig3d)
        outputs.append(out_3d)
    else:
        out_ortho = out_dir / "01_forward_kinematics_3d.png"
        _save_orthographic_figure(
            xyz, out_ortho, "Task 1: End-effector trajectory (orthographic views)"
        )
        outputs.append(out_ortho)

    return outputs


def plot_inverse_kinematics(results_dir: Path, out_dir: Path) -> Path:
    path = results_dir / "inverse_kinematics.npz"
    _require_file(path)
    d = np.load(path)
    q_ref = d["q_traj_ref"]
    q_ik = d["q_traj_ik"]
    pos_err = d["position_error"]
    names = _joint_labels(d["joint_names"])
    steps = np.arange(q_ref.shape[0])

    pose = d["pose_traj"]
    xyz_ref = pose[:, :3, 3]

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Task 2: Inverse Kinematics (POE Jacobian)", fontsize=14, fontweight="bold")

    ax1 = fig.add_subplot(2, 2, 1)
    for j in range(q_ref.shape[1]):
        ax1.plot(steps, q_ref[:, j], color=JOINT_COLORS[j], linestyle="--", alpha=0.7)
        ax1.plot(steps, q_ik[:, j], color=JOINT_COLORS[j], label=names[j])
    ax1.set_xlabel("Step")
    ax1.set_ylabel("Joint angle [rad]")
    ax1.set_title("Joint trajectory: dashed=reference, solid=IK")
    ax1.legend(loc="upper right", fontsize=7, ncol=2)
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.semilogy(steps, np.maximum(pos_err, 1e-16), "r-o", markersize=3)
    ax2.set_xlabel("Step")
    ax2.set_ylabel("Position error [m]")
    ax2.set_title("IK position error (FK vs target)")
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 3)
    dq = np.linalg.norm(q_ik - q_ref, axis=1)
    ax3.plot(steps, dq, "k-")
    ax3.set_xlabel("Step")
    ax3.set_ylabel("||q_ik - q_ref|| [rad]")
    ax3.set_title("Joint-space deviation from reference")
    ax3.grid(True, alpha=0.3)

    from irb1300_kin_dyn import IRB1300Robot

    robot = IRB1300Robot()
    robot_xyz = np.array([robot.forward_kinematics(q)[:3, 3] for q in q_ik])

    if _can_use_3d_axes():
        ax4 = fig.add_subplot(2, 2, 4, projection="3d")
        ax4.plot(xyz_ref[:, 0], xyz_ref[:, 1], xyz_ref[:, 2], "b--", alpha=0.6, label="target EE")
        ax4.plot(robot_xyz[:, 0], robot_xyz[:, 1], robot_xyz[:, 2], "r-", label="IK EE")
        _plot_start_end_markers_3d(ax4, robot_xyz)
        ax4.set_xlabel("X [m]")
        ax4.set_ylabel("Y [m]")
        ax4.set_zlabel("Z [m]")
        ax4.set_title("End-effector trajectory")
        ax4.legend(fontsize=8)
    else:
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(xyz_ref[:, 0], xyz_ref[:, 1], "b--", alpha=0.6, label="target EE")
        ax4.plot(robot_xyz[:, 0], robot_xyz[:, 1], "r-", label="IK EE")
        ax4.set_xlabel("X [m]")
        ax4.set_ylabel("Y [m]")
        ax4.set_title("End-effector XY trajectory")
        ax4.legend(fontsize=8)
        ax4.grid(True, alpha=0.3)
        ax4.set_aspect("equal", adjustable="box")

    fig.tight_layout()
    out = out_dir / "02_inverse_kinematics.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def plot_inverse_dynamics(results_dir: Path, out_dir: Path) -> Path:
    path = results_dir / "inverse_dynamics.npz"
    _require_file(path)
    d = np.load(path)
    q = d["q_traj"]
    qd = d["qd_traj"]
    qdd = d["qdd_traj"]
    tau_drive = d["tau_drive"]
    tau_constraint = d["tau_constraint"]
    tau_total = d["tau_total"]
    names = _joint_labels(d["joint_names"])
    steps = np.arange(q.shape[0])

    fig = plt.figure(figsize=(14, 11))
    fig.suptitle("Task 3: Inverse Dynamics", fontsize=14, fontweight="bold")

    ax1 = fig.add_subplot(3, 1, 1)
    for j in range(q.shape[1]):
        ax1.plot(steps, q[:, j], color=JOINT_COLORS[j], label=names[j])
    ax1.set_ylabel("q [rad]")
    ax1.set_title("Joint position / velocity / acceleration")
    ax1.legend(loc="upper right", fontsize=7, ncol=3)
    ax1.grid(True, alpha=0.3)

    ax1b = ax1.twinx()
    ax1b.plot(steps, np.linalg.norm(qd, axis=1), "k--", alpha=0.4, label="||qd||")
    ax1b.plot(steps, np.linalg.norm(qdd, axis=1), "k:", alpha=0.4, label="||qdd||")
    ax1b.set_ylabel("Norm [rad/s, rad/s²]")

    ax2 = fig.add_subplot(3, 1, 2)
    for j in range(tau_drive.shape[1]):
        ax2.plot(steps, tau_drive[:, j], color=JOINT_COLORS[j], label=f"{names[j]} drive")
    ax2.set_ylabel("Torque [Nm]")
    ax2.set_title("Joint driving torques")
    ax2.legend(loc="upper right", fontsize=7, ncol=2)
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(3, 1, 3)
    for j in range(tau_constraint.shape[1]):
        ax3.plot(steps, tau_constraint[:, j], color=JOINT_COLORS[j], linestyle="--", label=names[j])
    ax3.plot(steps, np.linalg.norm(tau_total, axis=1), "k-", linewidth=2, label="|tau_total|")
    ax3.set_xlabel("Step")
    ax3.set_ylabel("Torque [Nm]")
    ax3.set_title("Constraint torques (external wrench) & total")
    ax3.legend(loc="upper right", fontsize=7, ncol=2)
    ax3.grid(True, alpha=0.3)

    fig.tight_layout()
    out = out_dir / "03_inverse_dynamics.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def plot_forward_dynamics(results_dir: Path, out_dir: Path) -> Path:
    path = results_dir / "forward_dynamics.npz"
    _require_file(path)
    d = np.load(path)
    q = d["q"]
    qd = d["qd"]
    qdd = d["qdd"]
    tau = d["tau_drive"]
    names = _joint_labels(d["joint_names"])
    x = np.arange(len(names))

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle("Task 4: Forward Dynamics (single configuration)", fontsize=14, fontweight="bold")

    axes[0, 0].bar(x, q, color=JOINT_COLORS)
    axes[0, 0].set_xticks(x)
    axes[0, 0].set_xticklabels(names, rotation=30)
    axes[0, 0].set_ylabel("[rad]")
    axes[0, 0].set_title("Joint positions q")
    axes[0, 0].grid(True, axis="y", alpha=0.3)

    axes[0, 1].bar(x, qd, color=JOINT_COLORS)
    axes[0, 1].set_xticks(x)
    axes[0, 1].set_xticklabels(names, rotation=30)
    axes[0, 1].set_ylabel("[rad/s]")
    axes[0, 1].set_title("Joint velocities qd")
    axes[0, 1].grid(True, axis="y", alpha=0.3)

    axes[1, 0].bar(x, tau, color=JOINT_COLORS)
    axes[1, 0].set_xticks(x)
    axes[1, 0].set_xticklabels(names, rotation=30)
    axes[1, 0].set_ylabel("[Nm]")
    axes[1, 0].set_title("Driving torques (input)")
    axes[1, 0].grid(True, axis="y", alpha=0.3)

    axes[1, 1].bar(x, qdd, color=JOINT_COLORS)
    axes[1, 1].set_xticks(x)
    axes[1, 1].set_xticklabels(names, rotation=30)
    axes[1, 1].set_ylabel("[rad/s²]")
    axes[1, 1].set_title("Joint accelerations qdd (output)")
    if np.max(np.abs(qdd)) > 100 * np.median(np.abs(qdd[qdd != 0]) + 1e-9):
        axes[1, 1].set_yscale("symlog", linthresh=1.0)
    axes[1, 1].grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    out = out_dir / "04_forward_dynamics.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot IRB1300 simulation results")
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=ROOT / "results",
        help="Directory containing *.npz files",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "results" / "figures",
        help="Directory to save PNG figures",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively (requires GUI)",
    )
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    plots = [
        plot_forward_kinematics,
        plot_inverse_kinematics,
        plot_inverse_dynamics,
        plot_forward_dynamics,
    ]

    saved = []
    for fn in plots:
        out = fn(args.results_dir, args.output_dir)
        paths = out if isinstance(out, list) else [out]
        saved.extend(paths)
        for p in paths:
            print(f"Saved {p}")

    if args.show:
        for p in saved:
            img = plt.imread(p)
            plt.figure(figsize=(12, 8))
            plt.imshow(img)
            plt.axis("off")
            plt.title(p.name)
        plt.show()

    print(f"\nAll figures saved to {args.output_dir}")


if __name__ == "__main__":
    main()
