"""Full single-rigid-body plant with Euler/SO(3) integration."""

from __future__ import annotations

import numpy as np

from math_utils import GRAVITY, exp_so3, project_to_so3, rot_to_rpy, skew
from models import FullSrbState, RobotParams


class FullSrbEulerSimulator:
    def __init__(self, robot: RobotParams, dt: float):
        self.robot = robot
        self.dt = dt

    def step(
        self,
        state: FullSrbState,
        foot_wrenches: np.ndarray,
        foot_positions: np.ndarray,
        contact: np.ndarray,
    ) -> FullSrbState:
        force = np.zeros(3)
        moment = np.zeros(3)
        for leg in range(2):
            if not contact[leg]:
                continue
            wrench = foot_wrenches[6 * leg : 6 * (leg + 1)]
            f = wrench[:3]
            tau = wrench[3:]
            force += f
            moment += np.cross(foot_positions[leg] - state.p, f) + tau

        Iw = state.R @ self.robot.inertia_body @ state.R.T
        omega_dot = np.linalg.solve(Iw, moment - np.cross(state.omega, Iw @ state.omega))
        v_dot = force / self.robot.mass + np.array([0.0, 0.0, -GRAVITY])

        new_omega = state.omega + self.dt * omega_dot
        new_v = state.v + self.dt * v_dot
        new_R = project_to_so3(exp_so3(self.dt * state.omega) @ state.R)
        new_p = state.p + self.dt * state.v
        return FullSrbState(new_R, new_p, new_omega, new_v)

    @staticmethod
    def state_vector(state: FullSrbState) -> np.ndarray:
        x = np.zeros(13)
        x[0:3] = rot_to_rpy(state.R)
        x[3:6] = state.p
        x[6:9] = state.omega
        x[9:12] = state.v
        x[12] = GRAVITY
        return x

