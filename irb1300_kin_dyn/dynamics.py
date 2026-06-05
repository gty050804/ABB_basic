"""Inverse / forward dynamics using recursive Newton-Euler equations."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .se3 import adjoint, matrix_exp_se3, skew, xyz_rpy_to_transform
from .urdf_model import JointInfo, LinkInertial, build_poe_parameters, load_urdf_model


def _spatial_inertia(mass: float, com: np.ndarray, inertia: np.ndarray) -> np.ndarray:
    cx = skew(com)
    top_left = inertia - mass * (cx @ cx)
    top_right = mass * cx
    bot_left = (mass * cx).T
    bot_right = mass * np.eye(3)
    return np.vstack([np.hstack([top_left, top_right]), np.hstack([bot_left, bot_right])])


def _spatial_wrench(f_ext_ee: np.ndarray) -> np.ndarray:
    """Convert public [Fx,Fy,Fz,Mx,My,Mz] wrench to [Mx,My,Mz,Fx,Fy,Fz]."""
    f_ext_ee = np.asarray(f_ext_ee, dtype=float)
    return np.r_[f_ext_ee[3:], f_ext_ee[:3]]


def _ad(twist: np.ndarray) -> np.ndarray:
    w = twist[:3]
    v = twist[3:]
    return np.vstack(
        [
            np.hstack([skew(w), np.zeros((3, 3))]),
            np.hstack([skew(v), skew(w)]),
        ]
    )


def _transform_inverse(t: np.ndarray) -> np.ndarray:
    t_inv = np.eye(4)
    r = t[:3, :3]
    p = t[:3, 3]
    t_inv[:3, :3] = r.T
    t_inv[:3, 3] = -(r.T @ p)
    return t_inv


def _build_mlist(joints: list[JointInfo]) -> list[np.ndarray]:
    mlist = [xyz_rpy_to_transform(j.origin_xyz, j.origin_rpy) for j in joints]
    mlist.append(np.eye(4))
    return mlist


def _build_glist(joints: list[JointInfo], links: dict[str, LinkInertial]) -> list[np.ndarray]:
    return [_spatial_inertia(links[j.child].mass, links[j.child].com, links[j.child].inertia) for j in joints]


class NewtonEulerDynamics:
    def __init__(
        self,
        joints: list[JointInfo],
        links: dict[str, LinkInertial],
        base_link: str = "base_link",
        ee_link: str | None = None,
        gravity: np.ndarray | None = None,
    ):
        self.joints = joints
        self.links = links
        self.base_link = base_link
        self.ee_link = ee_link or joints[-1].child
        self.n = len(joints)
        self.gravity = np.array([0.0, 0.0, -9.81] if gravity is None else gravity, dtype=float)

        screws, m_home = build_poe_parameters(joints, base_link, self.ee_link)
        self.slist = screws
        self.m_home = m_home
        self.mlist = _build_mlist(joints)
        self.glist = _build_glist(joints, links)
        self.alist = self._body_screws_in_link_frames()

    def _body_screws_in_link_frames(self) -> np.ndarray:
        """Convert space-frame screw axes into each joint's local home frame."""
        screws = np.zeros_like(self.slist)
        t_base_to_joint = np.eye(4)
        for i in range(self.n):
            t_base_to_joint = t_base_to_joint @ self.mlist[i]
            screws[:, i] = adjoint(_transform_inverse(t_base_to_joint)) @ self.slist[:, i]
        return screws

    def _inverse_dynamics_with_gravity(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        gravity: np.ndarray,
        ftip: np.ndarray,
    ) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        qdd = np.asarray(qdd, dtype=float)

        transforms: list[np.ndarray] = [np.eye(4) for _ in range(self.n + 2)]
        velocities = [np.zeros(6) for _ in range(self.n + 1)]
        accelerations = [np.zeros(6) for _ in range(self.n + 1)]
        accelerations[0][3:] = -np.asarray(gravity, dtype=float)

        for i in range(self.n):
            screw = self.alist[:, i]
            transforms[i + 1] = matrix_exp_se3(-screw, q[i]) @ _transform_inverse(self.mlist[i])
            velocities[i + 1] = adjoint(transforms[i + 1]) @ velocities[i] + screw * qd[i]
            accelerations[i + 1] = (
                adjoint(transforms[i + 1]) @ accelerations[i]
                + _ad(velocities[i + 1]) @ (screw * qd[i])
                + screw * qdd[i]
            )

        transforms[self.n + 1] = _transform_inverse(self.mlist[-1])
        wrench = np.asarray(ftip, dtype=float).copy()
        tau = np.zeros(self.n)

        for i in reversed(range(self.n)):
            wrench = (
                adjoint(transforms[i + 2]).T @ wrench
                + self.glist[i] @ accelerations[i + 1]
                - _ad(velocities[i + 1]).T @ (self.glist[i] @ velocities[i + 1])
            )
            tau[i] = wrench @ self.alist[:, i]

        return tau

    def inverse_dynamics(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        f_ext_ee: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        if f_ext_ee is None:
            f_ext_ee = np.zeros(6)
        ftip = _spatial_wrench(f_ext_ee)
        tau = self._inverse_dynamics_with_gravity(q, qd, qdd, self.gravity, ftip)
        return tau, np.zeros(self.n)

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        m = np.zeros((self.n, self.n))
        zero = np.zeros(self.n)
        for i in range(self.n):
            qdd = np.zeros(self.n)
            qdd[i] = 1.0
            m[:, i] = self._inverse_dynamics_with_gravity(
                q,
                zero,
                qdd,
                np.zeros(3),
                np.zeros(6),
            )
        return m

    def coriolis_gravity(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        return self.inverse_dynamics(q, qd, np.zeros(self.n), np.zeros(6))[0]

    def forward_dynamics(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        tau_drive: np.ndarray,
        f_ext_ee: np.ndarray | None = None,
    ) -> np.ndarray:
        if f_ext_ee is None:
            f_ext_ee = np.zeros(6)
        m = self.mass_matrix(q)
        bias = self.inverse_dynamics(q, qd, np.zeros(self.n), f_ext_ee)[0]
        return np.linalg.solve(m, np.asarray(tau_drive, dtype=float) - bias)


def load_dynamics_from_urdf(urdf_path: str | Path) -> NewtonEulerDynamics:
    joints, links, base_link = load_urdf_model(urdf_path)
    return NewtonEulerDynamics(joints, links, base_link)
