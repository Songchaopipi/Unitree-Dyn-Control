"""Condensed SRBD-MPC QP with OSQP backend and SciPy fallback."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp
from scipy.optimize import minimize

from math_utils import (
    GRAVITY,
    foot_wrench_cone,
    nominal_vertical_wrench,
    rz,
    skew,
)
from models import MpcParams, RobotParams
from osqp_compat import setup_osqp

try:
    import osqp  # type: ignore
except ImportError:  # pragma: no cover - depends on local env
    osqp = None


@dataclass
class MpcResult:
    success: bool
    wrenches: np.ndarray
    status: str
    objective: float = np.nan


class OsqpSrbdMpc:
    def __init__(self, robot: RobotParams, params: MpcParams):
        self.robot = robot
        self.params = params
        self.cone = foot_wrench_cone(
            robot.mu,
            robot.foot_half_width,
            robot.foot_front,
            robot.foot_back,
            robot.yaw_friction,
        )

    def _build_discrete_model(
        self,
        x_current: np.ndarray,
        R: np.ndarray,
        reference: np.ndarray,
        contact_positions_horizon: list[np.ndarray],
        knot: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        nx, nu = 13, 12
        A = np.eye(nx)
        B = np.zeros((nx, nu))
        Iw = R @ self.robot.inertia_body @ R.T
        Iinv = np.linalg.inv(Iw)

        Ac = np.zeros((nx, nx))
        Ac[0:3, 6:9] = rz(x_current[2])
        Ac[3:6, 9:12] = np.eye(3)
        Ac[11, 12] = -1.0

        com = reference[3:6, knot] if knot < reference.shape[1] else x_current[3:6]
        contact_positions = contact_positions_horizon[knot]
        Bc = np.zeros((nx, nu))
        for leg in range(2):
            r = contact_positions[leg] - com
            wrench_to_centroidal = np.zeros((6, 6))
            wrench_to_centroidal[0:3, 0:3] = np.eye(3)
            wrench_to_centroidal[3:6, 0:3] = skew(r)
            wrench_to_centroidal[3:6, 3:6] = np.eye(3)
            col = 6 * leg
            Bc[9:12, col : col + 6] = wrench_to_centroidal[0:3, :] / self.robot.mass
            Bc[6:9, col : col + 6] = Iinv @ wrench_to_centroidal[3:6, :]

        dt = self.params.dt
        A += dt * Ac
        B = dt * Bc
        return A, B

    def _condense(
        self,
        x0: np.ndarray,
        R: np.ndarray,
        reference: np.ndarray,
        contact_positions_horizon: list[np.ndarray],
    ) -> tuple[np.ndarray, np.ndarray]:
        N, nx, nu = self.params.horizon, 13, 12
        Aqp = np.zeros((N * nx, nx))
        Bqp = np.zeros((N * nx, N * nu))
        Apow = np.eye(nx)
        Ad_list: list[np.ndarray] = []
        Bd_list: list[np.ndarray] = []
        for i in range(N):
            Ad, Bd = self._build_discrete_model(x0, R, reference, contact_positions_horizon, i)
            Ad_list.append(Ad)
            Bd_list.append(Bd)
            Apow = Ad @ Apow
            Aqp[i * nx : (i + 1) * nx, :] = Apow
            Aij = np.eye(nx)
            for j in range(i, -1, -1):
                Bqp[i * nx : (i + 1) * nx, j * nu : (j + 1) * nu] = Aij @ Bd_list[j]
                Aij = Aij @ Ad_list[j]
        return Aqp, Bqp

    def _nominal_wrenches(self, contact_table: np.ndarray) -> np.ndarray:
        N, nu = self.params.horizon, 12
        w_nom = np.zeros(N * nu)
        for k in range(N):
            active = int(np.sum(contact_table[k]))
            if active == 0:
                continue
            nominal_fz = self.robot.mass * GRAVITY / active
            for leg in range(2):
                if contact_table[k, leg]:
                    w_nom[k * nu + 6 * leg : k * nu + 6 * (leg + 1)] = nominal_vertical_wrench(nominal_fz)
        return w_nom

    def _wrench_metrics(self, horizon: int) -> tuple[np.ndarray, np.ndarray]:
        nu = 12
        input_metric = np.zeros((horizon * nu, horizon * nu))
        nominal_metric = np.zeros((horizon * nu, horizon * nu))
        for k in range(horizon):
            for leg in range(2):
                sl = slice(k * nu + 6 * leg, k * nu + 6 * (leg + 1))
                input_metric[sl, sl] = self.params.input_wrench_metric
                nominal_metric[sl, sl] = self.params.nominal_wrench_metric
        return input_metric, nominal_metric

    def solve(
        self,
        x0: np.ndarray,
        R: np.ndarray,
        reference: np.ndarray,
        contact_table: np.ndarray,
        contact_positions_horizon: list[np.ndarray],
    ) -> MpcResult:
        N, nx, nu = self.params.horizon, 13, 12
        n_var = N * nu
        Aqp, Bqp = self._condense(x0, R, reference, contact_positions_horizon)
        # reference is stored as shape (nx, horizon). The condensed dynamics
        # stack states by knot: [x_0(13), x_1(13), ...]. A plain C-order
        # reshape of reference.T would interleave state channels incorrectly.
        x_ref = np.concatenate([reference[:, k] for k in range(N)])

        w_diag = np.tile(self.params.state_weights, N)
        w_nom = self._nominal_wrenches(contact_table)
        input_metric, nominal_metric = self._wrench_metrics(N)

        WB = w_diag[:, None] * Bqp
        H = 2.0 * (Bqp.T @ WB)
        H += 2.0 * input_metric + 2.0 * nominal_metric
        H[np.diag_indices_from(H)] += 1e-8
        g = 2.0 * Bqp.T @ (w_diag * (Aqp @ x0 - x_ref))
        g -= 2.0 * (nominal_metric @ w_nom)

        A_rows = []
        lb = []
        ub = []
        for k in range(N):
            for leg in range(2):
                col = k * nu + 6 * leg
                row = np.zeros((self.cone.shape[0], n_var))
                row[:, col : col + 6] = self.cone
                A_rows.append(row)
                bound = np.zeros(self.cone.shape[0])
                bound[0] = -self.robot.fz_low if contact_table[k, leg] else 0.0
                lb.extend([-np.inf] * self.cone.shape[0])
                ub.extend(bound.tolist())

                row = np.zeros((6, n_var))
                row[:, col : col + 6] = np.eye(6)
                A_rows.append(row)
                if contact_table[k, leg]:
                    lb.extend([-self.robot.f_max] * 6)
                    ub.extend([self.robot.f_max] * 6)
                else:
                    lb.extend([0.0] * 6)
                    ub.extend([0.0] * 6)

        Acon = np.vstack(A_rows)
        lb_arr = np.asarray(lb)
        ub_arr = np.asarray(ub)

        if osqp is not None:
            prob = osqp.OSQP()
            setup_osqp(
                prob,
                P=sp.csc_matrix(0.5 * (H + H.T)),
                q=g,
                A=sp.csc_matrix(Acon),
                l=lb_arr,
                u=ub_arr,
                verbose=False,
                polishing=False,
                eps_abs=1e-5,
                eps_rel=1e-5,
                max_iter=4000,
            )
            res = prob.solve()
            success = res.info.status_val in (1, 2)
            U = res.x if success and res.x is not None else w_nom.copy()
            status = res.info.status
            objective = float(res.info.obj_val)
        else:
            U, success, status, objective = self._solve_with_scipy(H, g, Acon, lb_arr, ub_arr, w_nom)

        first = np.asarray(U[:nu], dtype=float)
        return MpcResult(success, first, status, objective)

    def _solve_with_scipy(
        self,
        H: np.ndarray,
        g: np.ndarray,
        A: np.ndarray,
        lb: np.ndarray,
        ub: np.ndarray,
        x_init: np.ndarray,
    ) -> tuple[np.ndarray, bool, str, float]:
        def obj(x: np.ndarray) -> float:
            return 0.5 * float(x @ H @ x) + float(g @ x)

        def jac(x: np.ndarray) -> np.ndarray:
            return H @ x + g

        constraints = []
        for i in range(A.shape[0]):
            ai = A[i].copy()
            if np.isfinite(lb[i]):
                constraints.append({"type": "ineq", "fun": lambda x, ai=ai, li=lb[i]: ai @ x - li})
            if np.isfinite(ub[i]):
                constraints.append({"type": "ineq", "fun": lambda x, ai=ai, ui=ub[i]: ui - ai @ x})

        res = minimize(
            obj,
            x_init,
            jac=jac,
            constraints=constraints,
            method="SLSQP",
            options={"maxiter": 200, "ftol": 1e-6, "disp": False},
        )
        return np.asarray(res.x), bool(res.success), str(res.message), float(res.fun)
