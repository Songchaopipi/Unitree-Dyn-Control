"""Dataclasses for the Python SRBD-MPC analysis."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


G1_TORSO_MASS = 7.818
G1_WHOLE_BODY_MASS = 33.341142
G1_TORSO_FZ_LOW = 20.0 * G1_TORSO_MASS / G1_WHOLE_BODY_MASS
G1_TORSO_INERTIA_BODY = np.array(
    [
        [0.121698443164, 0.000031109257, -0.003742289481],
        [0.000031109257, 0.109825081119, -0.000015507434],
        [-0.003742289481, -0.000015507434, 0.027521975718],
    ]
)
G1_FOOT_FRONT = 0.12
G1_FOOT_BACK = -0.05
G1_FOOT_HALF_WIDTH = 0.025


@dataclass
class RobotParams:
    mass: float = G1_TORSO_MASS
    inertia_body: np.ndarray = field(
        default_factory=lambda: G1_TORSO_INERTIA_BODY.copy()
    )
    nominal_com_height: float = 0.6993168
    nominal_foot_y: float = 0.10
    foot_half_width: float = G1_FOOT_HALF_WIDTH
    foot_front: float = G1_FOOT_FRONT
    foot_back: float = G1_FOOT_BACK
    yaw_friction: float = 0.046
    mu: float = 1.0
    f_max: float = 1400.0
    fz_low: float = G1_TORSO_FZ_LOW


@dataclass
class MpcParams:
    horizon: int = 10
    dt: float = 0.04
    solve_every: int = 1
    # These weights are for the isolated torso full-SRB plant, not the full G1
    # robot. The much smaller torso inertia needs stronger angular damping for
    # the standard SRBD-MPC state cost to indirectly stabilize centroidal moment.
    state_weights: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                8000.0,
                8000.0,
                2000.0,
                120.0,
                120.0,
                5000.0,
                1200.0,
                1200.0,
                50.0,
                40.0,
                40.0,
                1000.0,
                0.0,
            ]
        )
    )
    # Direct foot-wrench effort metric for one foot wrench [Fx,Fy,Fz,Mx,My,Mz].
    input_wrench_metric: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                [5.0e-5, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.252e-3, 0.0, 0.0, 0.0, -9.6e-3],
                [0.0, 0.0, 1.061418685121e-4, 2.941176470588e-4, 7.006920415225e-4, 0.0],
                [0.0, 0.0, 2.941176470588e-4, 1.2e-1, -5.882352941176e-3, 0.0],
                [0.0, 0.0, 7.006920415225e-4, -5.882352941176e-3, 9.515570934256e-3, 0.0],
                [0.0, -9.6e-3, 0.0, 0.0, 0.0, 8.0e-2],
            ]
        )
    )
    # Direct metric around nominal support wrench for one foot wrench.
    nominal_wrench_metric: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                [2.5e-2, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 6.26e-1, 0.0, 0.0, 0.0, -4.8],
                [0.0, 0.0, 2.707612456747e-2, 0.0, 1.643598615917e-1, 0.0],
                [0.0, 0.0, 0.0, 4.0e1, 0.0, 0.0],
                [0.0, 0.0, 1.643598615917e-1, 0.0, 2.595155709343, 0.0],
                [0.0, -4.8, 0.0, 0.0, 0.0, 4.0e1],
            ]
        )
    )


@dataclass
class GaitParams:
    single_support_time: float = 0.30
    double_support_time: float = 0.0
    step_width: float = 0.20
    lateral_step_min: float = 0.10
    lateral_step_max: float = 0.60
    use_angular_momentum_state: bool = True
    initial_stance: int = 0


@dataclass
class SimParams:
    dt: float = 0.002
    duration: float = 8.0
    desired_v: np.ndarray = field(default_factory=lambda: np.array([0.50, 0.0, 0.0]))
    desired_yaw_rate: float = 0.0
    initial_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    initial_com: np.ndarray = field(default_factory=lambda: np.array([0.01507763, 0.00008226, 0.6993168]))
    initial_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    initial_omega: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class FullSrbState:
    R: np.ndarray
    p: np.ndarray
    omega: np.ndarray
    v: np.ndarray

    @classmethod
    def from_arrays(cls, R: np.ndarray, p: np.ndarray, omega: np.ndarray, v: np.ndarray) -> "FullSrbState":
        return cls(R=R.copy(), p=p.copy(), omega=omega.copy(), v=v.copy())
