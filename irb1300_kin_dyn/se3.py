"""SE(3) utilities for Product of Exponentials kinematics."""

from __future__ import annotations

import numpy as np


def skew(w: np.ndarray) -> np.ndarray:
    wx, wy, wz = w
    return np.array(
        [[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]],
        dtype=float,
    )


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def xyz_rpy_to_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(rpy[0], rpy[1], rpy[2])
    t[:3, 3] = xyz
    return t


def adjoint(T: np.ndarray) -> np.ndarray:
    r = T[:3, :3]
    p = T[:3, 3]
    p_hat = skew(p)
    upper = np.hstack([r, np.zeros((3, 3))])
    lower = np.hstack([p_hat @ r, r])
    return np.vstack([upper, lower])


def matrix_exp_se3(screw: np.ndarray, theta: float) -> np.ndarray:
    w = screw[:3]
    v = screw[3:]
    w_norm = np.linalg.norm(w)
    t = np.eye(4)
    if w_norm < 1e-12:
        t[:3, 3] = v * theta
        return t

    wn = w / w_norm
    w_hat = skew(wn)
    angle = w_norm * theta
    r = (
        np.eye(3)
        + np.sin(angle) * w_hat
        + (1.0 - np.cos(angle)) * (w_hat @ w_hat)
    )
    p = (
        np.eye(3) * theta
        + (1.0 - np.cos(angle)) * w_hat
        + (angle - np.sin(angle)) * (w_hat @ w_hat)
    ) @ (v / w_norm)
    t[:3, :3] = r
    t[:3, 3] = p
    return t


def matrix_log_se3(T: np.ndarray) -> np.ndarray:
    r = T[:3, :3]
    p = T[:3, 3]
    cos_angle = (np.trace(r) - 1.0) * 0.5
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)

    if angle < 1e-8:
        w = np.array(
            [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]],
            dtype=float,
        ) * 0.5
        v = p.copy()
        return np.concatenate([w, v])

    w_hat = (r - r.T) / (2.0 * np.sin(angle)) * angle
    w = np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]], dtype=float)
    wn = w / angle
    w_hat_n = skew(wn)
    inv_g = (
        np.eye(3) / angle
        - 0.5 * w_hat_n
        + (1.0 / angle - 0.5 / np.tan(angle / 2.0)) * (w_hat_n @ w_hat_n)
    )
    v = inv_g @ p
    return np.concatenate([w, v])


def pose_error_twist(T_current: np.ndarray, T_desired: np.ndarray) -> np.ndarray:
  return matrix_log_se3(np.linalg.inv(T_current) @ T_desired)


def transform_to_rpy_xyz(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xyz = T[:3, 3].copy()
    r = T[:3, :3]
    pitch = np.arctan2(-r[2, 0], np.sqrt(r[2, 1] ** 2 + r[2, 2] ** 2))
    if abs(np.cos(pitch)) > 1e-8:
        roll = np.arctan2(r[2, 1], r[2, 2])
        yaw = np.arctan2(r[1, 0], r[0, 0])
    else:
        roll = np.arctan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return xyz, np.array([roll, pitch, yaw])
