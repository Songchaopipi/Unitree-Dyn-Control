"""OSQP centroidal stand MPC for stabilizing the full SRB plant."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp

from math_utils import GRAVITY, foot_wrench_cone, nominal_vertical_wrench, rot_to_rpy, skew
from models import FullSrbState, RobotParams
from osqp_compat import setup_osqp

try:
    import osqp  # type: ignore
except ImportError:  # pragma: no cover - depends on local env
    osqp = None


@dataclass
class StandMpcResult:
    success: bool
    wrenches: np.ndarray
    status: str


@dataclass
class StandMpcGains:
    kp_pos: np.ndarray
    kd_pos: np.ndarray
    kp_rpy: np.ndarray
    kd_omega: np.ndarray

    @classmethod
    def defaults(cls) -> "StandMpcGains":
        return cls(
            kp_pos=np.array([18.0, 18.0, 70.0]),
            kd_pos=np.array([9.0, 9.0, 18.0]),
            kp_rpy=np.array([70.0, 70.0, 35.0]),
            kd_omega=np.array([10.0, 10.0, 6.0]),
        )


class StableStandMpc:
    """One-step centroidal wrench MPC for double-support standing.

    The optimizer allocates foot wrenches so the full SRB receives the
    desired centroidal force and moment. This is intentionally stricter than
    the walking SRBD-MPC and is used only for the double-support stand phase.
    """

    def __init__(self, robot: RobotParams, gains: StandMpcGains | None = None):
        self.robot = robot
        self.gains = gains if gains is not None else StandMpcGains.defaults()
        self.cone = foot_wrench_cone(
            robot.mu,
            robot.foot_half_width,
            robot.foot_front,
            robot.foot_back,
            robot.yaw_friction,
        )

    def _centroidal_map(self, com: np.ndarray, foot_positions: np.ndarray) -> np.ndarray:
        C = np.zeros((6, 12))
        for leg in range(2):
            col = 6 * leg
            G = np.zeros((6, 6))
            G[0:3, 0:3] = np.eye(3)
            G[3:6, 0:3] = skew(foot_positions[leg] - com)
            G[3:6, 3:6] = np.eye(3)
            C[:, col : col + 6] = G
        return C

    def _nominal_wrenches(self, contact: np.ndarray) -> np.ndarray:
        active = max(1, int(np.sum(contact)))
        w_nom = np.zeros(12)
        for leg in range(2):
            if contact[leg]:
                w_nom[6 * leg : 6 * (leg + 1)] = nominal_vertical_wrench(self.robot.mass * GRAVITY / active)
        return w_nom

    def _desired_centroidal_wrench(
        self,
        state: FullSrbState,
        com_ref: np.ndarray,
        yaw_ref: float,
    ) -> np.ndarray:
        pos_err = com_ref - state.p
        vel_err = -state.v
        acc_des = self.gains.kp_pos * pos_err + self.gains.kd_pos * vel_err
        force_des = self.robot.mass * (acc_des + np.array([0.0, 0.0, GRAVITY]))

        rpy = rot_to_rpy(state.R)
        rpy_ref = np.array([0.0, 0.0, yaw_ref])
        rpy_err = rpy_ref - rpy
        rpy_err[2] = np.arctan2(np.sin(rpy_err[2]), np.cos(rpy_err[2]))
        alpha_des = self.gains.kp_rpy * rpy_err + self.gains.kd_omega * (-state.omega)
        Iw = state.R @ self.robot.inertia_body @ state.R.T
        moment_des = Iw @ alpha_des + np.cross(state.omega, Iw @ state.omega)

        wrench_des = np.zeros(6)
        wrench_des[:3] = force_des
        wrench_des[3:6] = moment_des
        return wrench_des

    def solve(
        self,
        state: FullSrbState,
        com_ref: np.ndarray,
        yaw_ref: float,
        foot_positions: np.ndarray,
        contact: np.ndarray,
    ) -> StandMpcResult:
        n_var = 12
        C = self._centroidal_map(state.p, foot_positions)
        wrench_des = self._desired_centroidal_wrench(state, com_ref, yaw_ref)
        w_nom = self._nominal_wrenches(contact)

        wrench_weights = np.array([1.0, 1.0, 2.0, 12.0, 12.0, 4.0])
        reg = np.array([1e-4, 1e-4, 2e-4, 2e-1, 2e-1, 2e-1] * 2)
        WC = wrench_weights[:, None] * C
        H = 2.0 * (C.T @ WC)
        H[np.diag_indices_from(H)] += 2.0 * reg + 1e-8
        g = -2.0 * C.T @ (wrench_weights * wrench_des) - 2.0 * reg * w_nom

        A_rows = []
        lb = []
        ub = []
        for leg in range(2):
            col = 6 * leg
            row = np.zeros((self.cone.shape[0], n_var))
            row[:, col : col + 6] = self.cone
            A_rows.append(row)
            bound = np.zeros(self.cone.shape[0])
            bound[0] = -self.robot.fz_low if contact[leg] else 0.0
            lb.extend([-np.inf] * self.cone.shape[0])
            ub.extend(bound.tolist())

            row = np.zeros((6, n_var))
            row[:, col : col + 6] = np.eye(6)
            A_rows.append(row)
            if contact[leg]:
                lb.extend([-self.robot.f_max] * 6)
                ub.extend([self.robot.f_max] * 6)
            else:
                lb.extend([0.0] * 6)
                ub.extend([0.0] * 6)

        if osqp is None:
            first = w_nom
            status = "osqp_unavailable"
            success = False
        else:
            prob = osqp.OSQP()
            setup_osqp(
                prob,
                P=sp.csc_matrix(0.5 * (H + H.T)),
                q=g,
                A=sp.csc_matrix(np.vstack(A_rows)),
                l=np.asarray(lb),
                u=np.asarray(ub),
                verbose=False,
                polishing=False,
                eps_abs=1e-6,
                eps_rel=1e-6,
                max_iter=4000,
            )
            res = prob.solve()
            success = res.info.status_val in (1, 2)
            first = np.asarray(res.x if success and res.x is not None else w_nom, dtype=float)
            status = res.info.status

        return StandMpcResult(success, first, status)
