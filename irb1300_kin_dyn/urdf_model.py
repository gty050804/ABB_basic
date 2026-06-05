"""Parse IRB 1300 URDF into kinematic and dynamic model parameters."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .se3 import rpy_to_matrix, xyz_rpy_to_transform


@dataclass
class JointInfo:
    name: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    effort: float
    velocity: float


@dataclass
class LinkInertial:
    name: str
    mass: float
    com: np.ndarray
    inertia: np.ndarray  # 3x3 about COM


def _parse_xyz_rpy(elem) -> tuple[np.ndarray, np.ndarray]:
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if elem is not None:
        if "xyz" in elem.attrib:
            xyz = np.array([float(v) for v in elem.attrib["xyz"].split()], dtype=float)
        if "rpy" in elem.attrib:
            rpy = np.array([float(v) for v in elem.attrib["rpy"].split()], dtype=float)
    return xyz, rpy


def load_urdf_model(urdf_path: str | Path) -> tuple[list[JointInfo], dict[str, LinkInertial], str]:
    root = ET.parse(urdf_path).getroot()
    robot_name = root.attrib.get("name", "robot")

    links: dict[str, LinkInertial] = {}
    for link in root.findall("link"):
        name = link.attrib["name"]
        inertial = link.find("inertial")
        if inertial is None:
            continue
        com, _ = _parse_xyz_rpy(inertial.find("origin"))
        mass = float(inertial.find("mass").attrib["value"])
        inertia_elem = inertial.find("inertia")
        inertia = np.array(
            [
                [float(inertia_elem.attrib["ixx"]), float(inertia_elem.attrib["ixy"]), float(inertia_elem.attrib["ixz"])],
                [float(inertia_elem.attrib["ixy"]), float(inertia_elem.attrib["iyy"]), float(inertia_elem.attrib["iyz"])],
                [float(inertia_elem.attrib["ixz"]), float(inertia_elem.attrib["iyz"]), float(inertia_elem.attrib["izz"])],
            ],
            dtype=float,
        )
        links[name] = LinkInertial(name=name, mass=mass, com=com, inertia=inertia)

    joints: list[JointInfo] = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "revolute":
            continue
        origin_xyz, origin_rpy = _parse_xyz_rpy(joint.find("origin"))
        axis_elem = joint.find("axis")
        axis = np.array([float(v) for v in axis_elem.attrib["xyz"].split()], dtype=float)
        axis_norm = np.linalg.norm(axis)
        if axis_norm > 0:
            axis = axis / axis_norm
        limit = joint.find("limit")
        joints.append(
            JointInfo(
                name=joint.attrib["name"],
                parent=joint.find("parent").attrib["link"],
                child=joint.find("child").attrib["link"],
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                axis=axis,
                lower=float(limit.attrib["lower"]),
                upper=float(limit.attrib["upper"]),
                effort=float(limit.attrib["effort"]),
                velocity=float(limit.attrib["velocity"]),
            )
        )

    child_links = {j.child for j in joints}
    parent_links = {j.parent for j in joints}
    base_link = next(iter(parent_links - child_links), "base_link")
    ordered: list[JointInfo] = []
    current = base_link
    joint_by_parent = {j.parent: j for j in joints}
    while current in joint_by_parent:
        j = joint_by_parent[current]
        ordered.append(j)
        current = j.child

    return ordered, links, base_link


def urdf_forward_kinematics(
    joints: list[JointInfo],
    q: np.ndarray,
    base_link: str = "base_link",
    ee_link: str | None = None,
) -> np.ndarray:
    if ee_link is None:
        ee_link = joints[-1].child
    transforms: dict[str, np.ndarray] = {base_link: np.eye(4)}
    for joint, angle in zip(joints, q):
        t_origin = xyz_rpy_to_transform(joint.origin_xyz, joint.origin_rpy)
        w_hat = np.array(
            [[0, -joint.axis[2], joint.axis[1]],
             [joint.axis[2], 0, -joint.axis[0]],
             [-joint.axis[1], joint.axis[0], 0]],
            dtype=float,
        )
        r_joint = (
            np.eye(3)
            + np.sin(angle) * w_hat
            + (1.0 - np.cos(angle)) * (w_hat @ w_hat)
        )
        t_joint = np.eye(4)
        t_joint[:3, :3] = r_joint
        transforms[joint.child] = transforms[joint.parent] @ t_origin @ t_joint
    return transforms[ee_link]


def build_poe_parameters(
    joints: list[JointInfo],
    base_link: str = "base_link",
    ee_link: str | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return space-frame screw axes (6 x n) and home configuration M (4x4)."""
    n = len(joints)
    screws = np.zeros((6, n))
    t_accum = np.eye(4)
    for i, joint in enumerate(joints):
        t_origin = xyz_rpy_to_transform(joint.origin_xyz, joint.origin_rpy)
        t_joint_frame = t_accum @ t_origin
        w = t_joint_frame[:3, :3] @ joint.axis
        q_point = t_joint_frame[:3, 3]
        v = -np.cross(w, q_point)
        screws[:, i] = np.concatenate([w, v])
        t_accum = t_joint_frame
    m = urdf_forward_kinematics(joints, np.zeros(n), base_link, ee_link)
    return screws, m
