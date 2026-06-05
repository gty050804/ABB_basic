"""Product of Exponentials forward/inverse kinematics."""

from __future__ import annotations

import numpy as np

from .se3 import adjoint, matrix_exp_se3, pose_error_twist


class POEKinematics:
    def __init__(self, screws: np.ndarray, home_transform: np.ndarray):
        self.screws = screws
        self.m_home = home_transform
        self.n_joints = screws.shape[1]

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        t = np.eye(4)
        for i in range(self.n_joints):
            t = t @ matrix_exp_se3(self.screws[:, i], q[i])
        return t @ self.m_home

    def space_jacobian(self, q: np.ndarray) -> np.ndarray:
        j = np.zeros((6, self.n_joints))
        t = np.eye(4)
        for i in range(self.n_joints):
            j[:, i] = adjoint(t) @ self.screws[:, i]
            t = t @ matrix_exp_se3(self.screws[:, i], q[i])
        return j

    def inverse_kinematics(
        self,
        target_pose: np.ndarray,
        q_init: np.ndarray,
        max_iter: int = 200,
        pos_tol: float = 1e-4,
        rot_tol: float = 1e-3,
        damping: float = 0.05,
        q_lower: np.ndarray | None = None,
        q_upper: np.ndarray | None = None,
    ) -> tuple[np.ndarray, bool, int]:
        q = q_init.astype(float).copy()
        best_q = q.copy()
        best_err = np.inf

        for it in range(max_iter):
            t_curr = self.forward_kinematics(q)
            twist = pose_error_twist(t_curr, target_pose)
            pos_err = np.linalg.norm(twist[3:])
            rot_err = np.linalg.norm(twist[:3])
            total_err = pos_err + rot_err
            if total_err < best_err:
                best_err = total_err
                best_q = q.copy()
            if pos_err < pos_tol and rot_err < rot_tol:
                return q, True, it + 1
            if total_err < pos_tol + rot_tol:
                return best_q, True, it + 1

            j_s = self.space_jacobian(q)
            j_b = adjoint(np.linalg.inv(t_curr)) @ j_s
            weight = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            j_w = weight @ j_b
            twist_w = weight @ twist
            lam = damping * (0.5 + total_err)
            dq = j_w.T @ np.linalg.solve(j_w @ j_w.T + lam**2 * np.eye(6), twist_w)
            dq = np.clip(dq, -0.15, 0.15)

            alpha = 1.0
            accepted = False
            for _ in range(12):
                q_try = q + alpha * dq
                if q_lower is not None and q_upper is not None:
                    q_try = np.clip(q_try, q_lower, q_upper)
                twist_try = pose_error_twist(self.forward_kinematics(q_try), target_pose)
                if np.linalg.norm(twist_try[3:]) + np.linalg.norm(twist_try[:3]) < total_err:
                    q = q_try
                    accepted = True
                    break
                alpha *= 0.5
            if not accepted:
                q = q + 0.05 * dq
                if q_lower is not None and q_upper is not None:
                    q = np.clip(q, q_lower, q_upper)

        return best_q, False, max_iter

    def forward_kinematics_trajectory(self, q_traj: np.ndarray) -> np.ndarray:
        return np.stack([self.forward_kinematics(q) for q in q_traj], axis=0)

    def inverse_kinematics_trajectory(
        self,
        pose_traj: np.ndarray,
        q_init: np.ndarray,
        q_lower: np.ndarray | None = None,
        q_upper: np.ndarray | None = None,
        **ik_kwargs,
    ) -> tuple[np.ndarray, np.ndarray]:
        n = pose_traj.shape[0]
        q_traj = np.zeros((n, self.n_joints))
        q_curr = q_init.copy()
        success = np.zeros(n, dtype=bool)
        for i in range(n):
            q_curr, ok, _ = self.inverse_kinematics(
                pose_traj[i],
                q_curr,
                q_lower=q_lower,
                q_upper=q_upper,
                **ik_kwargs,
            )
            q_traj[i] = q_curr
            success[i] = ok
        return q_traj, success
