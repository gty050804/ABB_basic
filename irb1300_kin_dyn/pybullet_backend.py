"""PyBullet dynamics backend — matches the same URDF used in simulation."""

from __future__ import annotations

from pathlib import Path

import numpy as np

try:
    import pybullet as p
    import pybullet_data
except ImportError:  # pragma: no cover - optional dependency
    p = None  # type: ignore[assignment]
    pybullet_data = None  # type: ignore[assignment]


class PyBulletDynamicsBackend:
    """Reuse a headless PyBullet instance for M(q) and bias forces."""

    def __init__(self, urdf_path: Path, ee_link_index: int = 5):
        if p is None:
            raise ImportError("pybullet is not installed")

        self.urdf_path = Path(urdf_path)
        mesh_dir = self.urdf_path.parent.parent / "meshes"
        self._client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setAdditionalSearchPath(str(mesh_dir))
        p.setGravity(0.0, 0.0, -9.81)
        self.body_id = p.loadURDF(
            str(self.urdf_path),
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )
        self.n_joints = p.getNumJoints(self.body_id)
        self.ee_link_index = ee_link_index

    def set_state(self, q: np.ndarray, qd: np.ndarray) -> None:
        for j in range(self.n_joints):
            p.resetJointState(self.body_id, j, float(q[j]), float(qd[j]))

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        self.set_state(q, np.zeros(self.n_joints))
        return np.array(p.calculateMassMatrix(self.body_id, q.tolist()), dtype=float)

    def bias_forces(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        """Coriolis + gravity (no joint acceleration, no external wrench)."""
        self.set_state(q, qd)
        return np.array(
            p.calculateInverseDynamics(
                self.body_id,
                q.tolist(),
                qd.tolist(),
                [0.0] * self.n_joints,
            ),
            dtype=float,
        )

    def inverse_dynamics(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
    ) -> np.ndarray:
        self.set_state(q, qd)
        return np.array(
            p.calculateInverseDynamics(
                self.body_id,
                q.tolist(),
                qd.tolist(),
                qdd.tolist(),
            ),
            dtype=float,
        )

    def wrench_to_joint_torques(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        f_ext_ee: np.ndarray,
        local_pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> np.ndarray:
        """Map public [Fx,Fy,Fz,Mx,My,Mz] EE wrench to joint torques."""
        self.set_state(q, qd)
        linear_jac, angular_jac = p.calculateJacobian(
            self.body_id,
            self.ee_link_index,
            list(local_pos),
            q.tolist(),
            qd.tolist(),
            [0.0] * self.n_joints,
        )
        link_state = p.getLinkState(self.body_id, self.ee_link_index, computeForwardKinematics=True)
        link_to_world = np.array(p.getMatrixFromQuaternion(link_state[5])).reshape(3, 3)
        f_ext_ee = np.asarray(f_ext_ee, dtype=float)
        f_ext_world = np.r_[
            link_to_world @ f_ext_ee[:3],
            link_to_world @ f_ext_ee[3:],
        ]
        jacobian = np.vstack([np.array(linear_jac), np.array(angular_jac)])
        return jacobian.T @ f_ext_world

    def forward_dynamics(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        tau_drive: np.ndarray,
        tau_constraint: np.ndarray | None = None,
    ) -> np.ndarray:
        """Solve M(q) qdd = tau_drive - bias(q, qd) - tau_constraint."""
        bias = self.bias_forces(q, qd)
        m = self.mass_matrix(q)
        rhs = np.asarray(tau_drive, dtype=float) - bias
        if tau_constraint is not None:
            rhs = rhs - np.asarray(tau_constraint, dtype=float)
        return np.linalg.solve(m, rhs)

    def close(self) -> None:
        if p is not None and self._client is not None:
            p.disconnect(self._client)
            self._client = None


def pybullet_available() -> bool:
    return p is not None


def default_backend(urdf_path: Path) -> PyBulletDynamicsBackend | None:
    if not pybullet_available():
        return None
    try:
        return PyBulletDynamicsBackend(urdf_path)
    except Exception:
        return None
