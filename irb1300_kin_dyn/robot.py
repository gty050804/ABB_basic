"""High-level IRB 1300 robot model."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .dynamics import NewtonEulerDynamics
from .poe_kinematics import POEKinematics
from .se3 import transform_to_rpy_xyz
from .urdf_model import (
    build_poe_parameters,
    load_urdf_model,
    urdf_forward_kinematics,
)


def default_urdf_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "irb1300_urdf"
        / "urdf"
        / "1300_urdf.urdf"
    )


class IRB1300Robot:
    joint_names = ("joint2", "joint3", "joint4", "joint5", "joint6", "joint7")
    ee_link = "link7"

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        use_simulator_fd: bool | None = None,
    ):
        urdf_path = Path(urdf_path) if urdf_path else default_urdf_path()
        if not urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        self.urdf_path = urdf_path
        self.joints, self.links, self.base_link = load_urdf_model(urdf_path)
        self.n_joints = len(self.joints)
        screws, m_home = build_poe_parameters(self.joints, self.base_link, self.ee_link)
        self.kinematics = POEKinematics(screws, m_home)
        self.dynamics = NewtonEulerDynamics(self.joints, self.links, self.base_link)
        self.q_lower = np.array([j.lower for j in self.joints])
        self.q_upper = np.array([j.upper for j in self.joints])

        if use_simulator_fd is None:
            use_simulator_fd = False
        self._sim_backend = None
        if use_simulator_fd:
            from .pybullet_backend import default_backend

            self._sim_backend = default_backend(urdf_path)
        self.use_simulator_fd = self._sim_backend is not None

    def clip_joint_positions(self, q: np.ndarray) -> np.ndarray:
        return np.clip(q, self.q_lower, self.q_upper)

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        return self.kinematics.forward_kinematics(q)

    def forward_kinematics_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        t = self.forward_kinematics(q)
        return transform_to_rpy_xyz(t)

    def forward_kinematics_trajectory(self, q_traj: np.ndarray) -> np.ndarray:
        return self.kinematics.forward_kinematics_trajectory(q_traj)

    def space_jacobian(self, q: np.ndarray) -> np.ndarray:
        return self.kinematics.space_jacobian(q)

    def inverse_kinematics(
        self,
        target_pose: np.ndarray,
        q_init: np.ndarray,
        **kwargs,
    ) -> tuple[np.ndarray, bool, int]:
        return self.kinematics.inverse_kinematics(
            target_pose,
            q_init,
            q_lower=self.q_lower,
            q_upper=self.q_upper,
            **kwargs,
        )

    def inverse_kinematics_trajectory(
        self,
        pose_traj: np.ndarray,
        q_init: np.ndarray,
        **kwargs,
    ) -> tuple[np.ndarray, np.ndarray]:
        return self.kinematics.inverse_kinematics_trajectory(
            pose_traj,
            q_init,
            q_lower=self.q_lower,
            q_upper=self.q_upper,
            **kwargs,
        )

    def inverse_dynamics(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        f_ext_ee: np.ndarray | None = None,
    ) -> dict[str, np.ndarray]:
        if f_ext_ee is None:
            f_ext_ee = np.zeros(6)
        tau_total, _ = self.dynamics.inverse_dynamics(q, qd, qdd, f_ext_ee)
        tau_no_ext, _ = self.dynamics.inverse_dynamics(q, qd, qdd, np.zeros(6))
        tau_constraint = tau_total - tau_no_ext
        tau_drive = tau_no_ext
        return {
            "tau_drive": tau_drive,
            "tau_constraint": tau_constraint,
            "tau_total": tau_total,
        }

    def forward_dynamics(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        tau_drive: np.ndarray,
        f_ext_ee: np.ndarray | None = None,
    ) -> np.ndarray:
        if f_ext_ee is None:
            f_ext_ee = np.zeros(6)

        if self._sim_backend is not None:
            tau_constraint = None
            if np.linalg.norm(f_ext_ee) > 0.0:
                tau_constraint = self.inverse_dynamics(
                    q, qd, np.zeros(self.n_joints), f_ext_ee
                )["tau_constraint"]
            return self._sim_backend.forward_dynamics(q, qd, tau_drive, tau_constraint)

        return self.dynamics.forward_dynamics(q, qd, tau_drive, f_ext_ee)

    def forward_dynamics_theoretical(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        tau_drive: np.ndarray,
        f_ext_ee: np.ndarray | None = None,
    ) -> np.ndarray:
        """Theoretical recursive Newton-Euler forward dynamics."""
        return self.dynamics.forward_dynamics(q, qd, tau_drive, f_ext_ee)

    def verify_poe_against_urdf(self, q: np.ndarray | None = None) -> float:
        q = np.zeros(self.n_joints) if q is None else q
        t_poe = self.forward_kinematics(q)
        t_urdf = urdf_forward_kinematics(self.joints, q, self.base_link, self.ee_link)
        return float(np.linalg.norm(t_poe - t_urdf))
