"""Sample joint and pose trajectories for demos."""

from __future__ import annotations

import numpy as np


def joint_sinusoid_trajectory(
    n_steps: int,
    n_joints: int,
    dt: float,
    amplitude: float = 0.25,
    q_center: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if q_center is None:
        q_center = np.zeros(n_joints)
    t = np.linspace(0.0, 1.85 * np.pi, n_steps)
    q = np.zeros((n_steps, n_joints))
    qd = np.zeros((n_steps, n_joints))
    qdd = np.zeros((n_steps, n_joints))
    for j in range(n_joints):
        phase = j * np.pi / n_joints
        q[:, j] = q_center[j] + amplitude * np.sin(t + phase)
        qd[:, j] = amplitude * np.cos(t + phase) * (2.0 * np.pi / (t[-1] - t[0] + 1e-9))
        qdd[:, j] = -amplitude * np.sin(t + phase) * (2.0 * np.pi / (t[-1] - t[0] + 1e-9)) ** 2
    return q, qd, qdd


def default_home_configuration(n_joints: int = 6) -> np.ndarray:
    return np.array([0.0, -0.4, 0.6, 0.0, 0.5, 0.0])[:n_joints]
