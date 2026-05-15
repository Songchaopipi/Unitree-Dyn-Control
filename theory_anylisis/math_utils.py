"""Small math helpers shared by the SRBD-MPC analysis scripts."""

from __future__ import annotations

import numpy as np


GRAVITY = 9.80665


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]
    )


def rx(roll: float) -> np.ndarray:
    c, s = np.cos(roll), np.sin(roll)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(pitch: float) -> np.ndarray:
    c, s = np.cos(pitch), np.sin(pitch)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_rot(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    return rz(yaw) @ ry(pitch) @ rx(roll)


def rot_to_rpy(R: np.ndarray) -> np.ndarray:
    return np.array(
        [
            np.arctan2(R[2, 1], R[2, 2]),
            np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)),
            np.arctan2(R[1, 0], R[0, 0]),
        ]
    )


def exp_so3(w: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3) + skew(w)
    axis = w / theta
    K = skew(axis)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def project_to_so3(R: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(R)
    Rn = u @ vt
    if np.linalg.det(Rn) < 0.0:
        u[:, -1] *= -1.0
        Rn = u @ vt
    return Rn


def plane_force_to_wrench_map(
    foot_half_width: float = 0.025,
    foot_front: float = 0.1,
    foot_back: float = -0.04,
) -> np.ndarray:
    """Map six Romoco-style plane force parameters to [Fx,Fy,Fz,Mx,My,Mz]."""
    return np.array(
        [
            [1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 1.0, 1.0],
            [0.0, 0.0, foot_half_width, 0.0, -foot_half_width, 0.0],
            [0.0, 0.0, -foot_front, 0.0, -foot_front, -foot_back],
            [-foot_half_width, foot_front, 0.0, foot_half_width, 0.0, 0.0],
        ]
    )


def distribute_vertical_foot_load(
    fz: float,
    foot_front: float = 0.1,
    foot_back: float = -0.04,
) -> np.ndarray:
    front_total = -foot_back / (foot_front - foot_back) * fz
    back = fz - front_total
    return np.array([0.0, 0.0, 0.5 * front_total, 0.0, 0.5 * front_total, back])


def nominal_vertical_wrench(fz: float) -> np.ndarray:
    return np.array([0.0, 0.0, fz, 0.0, 0.0, 0.0])


def foot_wrench_cone(
    mu: float,
    foot_half_width: float,
    foot_front: float,
    foot_back: float,
    yaw_friction: float,
) -> np.ndarray:
    return np.array(
        [
            [0, 0, -1, 0, 0, 0],
            [1, 0, -mu / np.sqrt(2.0), 0, 0, 0],
            [-1, 0, -mu / np.sqrt(2.0), 0, 0, 0],
            [0, 1, -mu / np.sqrt(2.0), 0, 0, 0],
            [0, -1, -mu / np.sqrt(2.0), 0, 0, 0],
            [0, 0, -foot_half_width, 1, 0, 0],
            [0, 0, -foot_half_width, -1, 0, 0],
            [0, 0, -foot_front, 0, 1, 0],
            [0, 0, foot_back, 0, -1, 0],
            [0, 0, -yaw_friction, 0, 0, 1],
            [0, 0, -yaw_friction, 0, 0, -1],
        ],
        dtype=float,
    )


def romoco_plane_foot_cone(
    mu: float,
    foot_half_width: float,
    foot_front: float,
    foot_back: float,
    yaw_friction: float,
) -> np.ndarray:
    return foot_wrench_cone(
        mu,
        foot_half_width,
        foot_front,
        foot_back,
        yaw_friction,
    ) @ plane_force_to_wrench_map(foot_half_width, foot_front, foot_back)
