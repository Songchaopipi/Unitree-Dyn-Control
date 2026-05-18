"""Linear affine SRBD discrete model and 6D foot-wrench constraints.

The analysis state is:

    x = [roll, pitch, yaw,
         com_x, com_y, com_z,
         omega_x, omega_y, omega_z,
         vel_x, vel_y, vel_z]

    u_ds = [Fx, Fy, Fz, Mx, My, Mz]_left
         + [Fx, Fy, Fz, Mx, My, Mz]_right

    u_left_ss  = [Fx, Fy, Fz, Mx, My, Mz]_left
    u_right_ss = [Fx, Fy, Fz, Mx, My, Mz]_right
    u_flight = empty

The discrete system is affine:

    x[k+1] = Ad x[k] + Bd u[k] + dd

where dd carries gravity. The 13D augmented gravity-state form used by the
existing OSQP-MPC is kept only as a compatibility wrapper.

The default numbers are the existing G1 analysis defaults in models.py.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:  # Imported as theory_anylisis.dynamics.*
    from ..math_utils import GRAVITY, foot_wrench_cone, rz, skew
    from ..models import (
        G1_FOOT_BACK,
        G1_FOOT_FRONT,
        G1_FOOT_HALF_WIDTH,
        G1_TORSO_FZ_LOW,
        G1_TORSO_INERTIA_BODY,
        G1_TORSO_MASS,
        RobotParams,
        SimParams,
    )
except ImportError:  # Imported from scripts inside theory_anylisis/.
    from math_utils import GRAVITY, foot_wrench_cone, rz, skew
    from models import (
        G1_FOOT_BACK,
        G1_FOOT_FRONT,
        G1_FOOT_HALF_WIDTH,
        G1_TORSO_FZ_LOW,
        G1_TORSO_INERTIA_BODY,
        G1_TORSO_MASS,
        RobotParams,
        SimParams,
    )


NX_SRBD = 12
NX_SRBD_AUGMENTED = 13
NU_FOOT_WRENCH = 6
N_CONTACTS = 2
NU_SRBD = N_CONTACTS * NU_FOOT_WRENCH
LEFT_FOOT = 0
RIGHT_FOOT = 1


@dataclass(frozen=True)
class LinearConstraint:
    """Linear constraint in the form lb <= A @ x <= ub."""

    A: np.ndarray
    lb: np.ndarray
    ub: np.ndarray


@dataclass(frozen=True)
class AffineDynamics:
    """Discrete affine system x_next = A @ x + B @ u + d."""

    A: np.ndarray
    B: np.ndarray
    d: np.ndarray


def _vec3(value: np.ndarray | None, default: np.ndarray, name: str) -> np.ndarray:
    out = np.asarray(default if value is None else value, dtype=float).reshape(-1)
    if out.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {out.shape}")
    return out.copy()


def _mat3(value: np.ndarray | None, default: np.ndarray, name: str) -> np.ndarray:
    out = np.asarray(default if value is None else value, dtype=float)
    if out.shape != (3, 3):
        raise ValueError(f"{name} must have shape (3, 3), got {out.shape}")
    return out.copy()


def _contact_positions2(
    value: np.ndarray | None,
    *,
    nominal_foot_y: float | None = None,
) -> np.ndarray:
    out = np.asarray(
        default_g1_contact_positions(nominal_foot_y) if value is None else value,
        dtype=float,
    )
    if out.shape != (N_CONTACTS, 3):
        raise ValueError(f"contact_positions must have shape (2, 3), got {out.shape}")
    return out.copy()


def _active_legs_tuple(active_legs: tuple[int, ...] | list[int]) -> tuple[int, ...]:
    legs = tuple(int(leg) for leg in active_legs)
    if any(leg not in (LEFT_FOOT, RIGHT_FOOT) for leg in legs):
        raise ValueError(f"active_legs must contain only 0(left) and 1(right), got {legs}")
    if len(set(legs)) != len(legs):
        raise ValueError(f"active_legs contains duplicates: {legs}")
    return legs


def default_g1_contact_positions(nominal_foot_y: float | None = None) -> np.ndarray:
    """Return the existing double-support foot positions used by the analysis."""

    y = RobotParams().nominal_foot_y if nominal_foot_y is None else float(nominal_foot_y)
    return np.array([[0.0, y, 0.0], [0.0, -y, 0.0]], dtype=float)


def default_g1_srbd_state(
    initial_rpy: np.ndarray | None = None,
    initial_com: np.ndarray | None = None,
    initial_omega: np.ndarray | None = None,
    initial_vel: np.ndarray | None = None,
) -> np.ndarray:
    """Return the existing 12D G1 SRBD initial state."""

    sim = SimParams()
    x = np.zeros(NX_SRBD)
    x[0:3] = _vec3(initial_rpy, sim.initial_rpy, "initial_rpy")
    x[3:6] = _vec3(initial_com, sim.initial_com, "initial_com")
    x[6:9] = _vec3(initial_omega, sim.initial_omega, "initial_omega")
    x[9:12] = _vec3(initial_vel, sim.initial_vel, "initial_vel")
    return x


def default_g1_augmented_srbd_state(
    *,
    gravity: float = GRAVITY,
    initial_rpy: np.ndarray | None = None,
    initial_com: np.ndarray | None = None,
    initial_omega: np.ndarray | None = None,
    initial_vel: np.ndarray | None = None,
) -> np.ndarray:
    """Return the existing 13D augmented SRBD state used by OSQP-MPC."""

    x = np.zeros(NX_SRBD_AUGMENTED)
    x[0:NX_SRBD] = default_g1_srbd_state(
        initial_rpy=initial_rpy,
        initial_com=initial_com,
        initial_omega=initial_omega,
        initial_vel=initial_vel,
    )
    x[12] = float(gravity)
    return x


def srbd_continuous_matrices(
    *,
    mass: float = G1_TORSO_MASS,
    inertia_body: np.ndarray | None = None,
    yaw: float = 0.0,
    com: np.ndarray | None = None,
    contact_positions: np.ndarray | None = None,
    R: np.ndarray | None = None,
    active_legs: tuple[int, ...] | list[int] = (LEFT_FOOT, RIGHT_FOOT),
    gravity: float = GRAVITY,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build the continuous-time affine SRBD matrices Ac, Bc, dc.

    The model is:

        rpy_dot = Rz(yaw) * omega
        p_dot = v
        omega_dot = I_W^-1 * sum_i ((p_foot_i - p_com) x f_i + tau_i)
        v_dot = (1 / m) * sum_i f_i - gravity * e_z

    Bc has 6 columns per active contact, ordered by active_legs. The default is
    double support: left foot wrench followed by right foot wrench.
    """

    mass = float(mass)
    if mass <= 0.0:
        raise ValueError(f"mass must be positive, got {mass}")

    inertia_body_mat = _mat3(inertia_body, G1_TORSO_INERTIA_BODY, "inertia_body")
    R_world_body = rz(float(yaw)) if R is None else _mat3(R, np.eye(3), "R")
    Iw = R_world_body @ inertia_body_mat @ R_world_body.T
    Iinv = np.linalg.inv(Iw)

    com_vec = _vec3(com, SimParams().initial_com, "com")
    contacts = _contact_positions2(contact_positions)
    active = _active_legs_tuple(active_legs)

    Ac = np.zeros((NX_SRBD, NX_SRBD))
    Bc = np.zeros((NX_SRBD, NU_FOOT_WRENCH * len(active)))
    dc = np.zeros(NX_SRBD)

    Ac[0:3, 6:9] = rz(float(yaw))
    Ac[3:6, 9:12] = np.eye(3)
    dc[11] = -float(gravity)

    for input_index, leg in enumerate(active):
        r = contacts[leg] - com_vec
        wrench_to_centroidal = np.zeros((6, 6))
        wrench_to_centroidal[0:3, 0:3] = np.eye(3)
        wrench_to_centroidal[3:6, 0:3] = skew(r)
        wrench_to_centroidal[3:6, 3:6] = np.eye(3)

        col = NU_FOOT_WRENCH * input_index
        Bc[9:12, col : col + NU_FOOT_WRENCH] = wrench_to_centroidal[0:3, :] / mass
        Bc[6:9, col : col + NU_FOOT_WRENCH] = Iinv @ wrench_to_centroidal[3:6, :]

    return Ac, Bc, dc


def srbd_discrete_matrices(
    *,
    dt: float = 0.04,
    mass: float = G1_TORSO_MASS,
    inertia_body: np.ndarray | None = None,
    yaw: float = 0.0,
    com: np.ndarray | None = None,
    contact_positions: np.ndarray | None = None,
    R: np.ndarray | None = None,
    active_legs: tuple[int, ...] | list[int] = (LEFT_FOOT, RIGHT_FOOT),
    gravity: float = GRAVITY,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build the Euler-discretized affine SRBD matrices Ad, Bd, dd.

    The returned system is:

        x[k+1] = Ad x[k] + Bd u[k] + dd
    """

    dt = float(dt)
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    Ac, Bc, dc = srbd_continuous_matrices(
        mass=mass,
        inertia_body=inertia_body,
        yaw=yaw,
        com=com,
        contact_positions=contact_positions,
        R=R,
        active_legs=active_legs,
        gravity=gravity,
    )
    return np.eye(NX_SRBD) + dt * Ac, dt * Bc, dt * dc


def srbd_augmented_discrete_matrices(
    *,
    dt: float = 0.04,
    mass: float = G1_TORSO_MASS,
    inertia_body: np.ndarray | None = None,
    yaw: float = 0.0,
    com: np.ndarray | None = None,
    contact_positions: np.ndarray | None = None,
    R: np.ndarray | None = None,
    active_legs: tuple[int, ...] | list[int] = (LEFT_FOOT, RIGHT_FOOT),
) -> tuple[np.ndarray, np.ndarray]:
    """Build the 13D gravity-state model used by the existing OSQP-MPC.

    This is only a compatibility wrapper around the affine model:

        x_aug = [x12, gravity]
        x_aug[k+1] = A_aug x_aug[k] + B_aug u[k]
    """

    Ad, Bd, gravity_column = srbd_discrete_matrices(
        dt=dt,
        mass=mass,
        inertia_body=inertia_body,
        yaw=yaw,
        com=com,
        contact_positions=contact_positions,
        R=R,
        active_legs=active_legs,
        gravity=1.0,
    )
    A_aug = np.eye(NX_SRBD_AUGMENTED)
    A_aug[0:NX_SRBD, 0:NX_SRBD] = Ad
    A_aug[0:NX_SRBD, NX_SRBD] = gravity_column
    B_aug = np.zeros((NX_SRBD_AUGMENTED, Bd.shape[1]))
    B_aug[0:NX_SRBD, :] = Bd
    return A_aug, B_aug


def srbd_discrete_dynamics_double_support(
    *,
    dt: float = 0.04,
    mass: float = G1_TORSO_MASS,
    inertia_body: np.ndarray | None = None,
    yaw: float = 0.0,
    com: np.ndarray | None = None,
    contact_positions: np.ndarray | None = None,
    R: np.ndarray | None = None,
    gravity: float = GRAVITY,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Discrete SRBD dynamics for double support.

    Input u has 12 dimensions:
    [w_left(6), w_right(6)].
    """

    return srbd_discrete_matrices(
        dt=dt,
        mass=mass,
        inertia_body=inertia_body,
        yaw=yaw,
        com=com,
        contact_positions=contact_positions,
        R=R,
        active_legs=(LEFT_FOOT, RIGHT_FOOT),
        gravity=gravity,
    )


def srbd_discrete_dynamics_left_support(
    *,
    dt: float = 0.04,
    mass: float = G1_TORSO_MASS,
    inertia_body: np.ndarray | None = None,
    yaw: float = 0.0,
    com: np.ndarray | None = None,
    contact_positions: np.ndarray | None = None,
    R: np.ndarray | None = None,
    gravity: float = GRAVITY,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Discrete SRBD dynamics for left-foot single support.

    Input u has 6 dimensions: [w_left].
    """

    return srbd_discrete_matrices(
        dt=dt,
        mass=mass,
        inertia_body=inertia_body,
        yaw=yaw,
        com=com,
        contact_positions=contact_positions,
        R=R,
        active_legs=(LEFT_FOOT,),
        gravity=gravity,
    )


def srbd_discrete_dynamics_right_support(
    *,
    dt: float = 0.04,
    mass: float = G1_TORSO_MASS,
    inertia_body: np.ndarray | None = None,
    yaw: float = 0.0,
    com: np.ndarray | None = None,
    contact_positions: np.ndarray | None = None,
    R: np.ndarray | None = None,
    gravity: float = GRAVITY,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Discrete SRBD dynamics for right-foot single support.

    Input u has 6 dimensions: [w_right].
    """

    return srbd_discrete_matrices(
        dt=dt,
        mass=mass,
        inertia_body=inertia_body,
        yaw=yaw,
        com=com,
        contact_positions=contact_positions,
        R=R,
        active_legs=(RIGHT_FOOT,),
        gravity=gravity,
    )


def srbd_discrete_dynamics_flight(
    *,
    dt: float = 0.04,
    mass: float = G1_TORSO_MASS,
    inertia_body: np.ndarray | None = None,
    yaw: float = 0.0,
    com: np.ndarray | None = None,
    contact_positions: np.ndarray | None = None,
    R: np.ndarray | None = None,
    gravity: float = GRAVITY,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Discrete SRBD dynamics for flight/no-contact phase.

    Input u has 0 dimensions, so Bd has shape (12, 0). Gravity appears in dd.
    """

    return srbd_discrete_matrices(
        dt=dt,
        mass=mass,
        inertia_body=inertia_body,
        yaw=yaw,
        com=com,
        contact_positions=contact_positions,
        R=R,
        active_legs=(),
        gravity=gravity,
    )


def foot_wrench_planar_cone_matrix(
    *,
    mu: float = 1.0,
    foot_half_width: float = G1_FOOT_HALF_WIDTH,
    foot_front: float = G1_FOOT_FRONT,
    foot_back: float = G1_FOOT_BACK,
    yaw_friction: float = 0.046,
) -> np.ndarray:
    """Return C for the single-foot 6D wrench cone C @ w <= 0.

    w = [Fx, Fy, Fz, Mx, My, Mz].
    """

    return foot_wrench_cone(
        float(mu),
        float(foot_half_width),
        float(foot_front),
        float(foot_back),
        float(yaw_friction),
    )


def foot_wrench_planar_cone_constraints(
    *,
    active: bool = True,
    mu: float = 1.0,
    foot_half_width: float = G1_FOOT_HALF_WIDTH,
    foot_front: float = G1_FOOT_FRONT,
    foot_back: float = G1_FOOT_BACK,
    yaw_friction: float = 0.046,
    fz_low: float = G1_TORSO_FZ_LOW,
    f_max: float = 1400.0,
    include_wrench_box: bool = True,
) -> LinearConstraint:
    """Return lb <= A @ w <= ub for one 6D foot wrench.

    Active contact uses the same convention as the current MPC:
    Fz >= fz_low, C @ w <= 0, and optionally -f_max <= w_i <= f_max.
    Inactive contact is forced to zero when include_wrench_box=True.
    """

    C = foot_wrench_planar_cone_matrix(
        mu=mu,
        foot_half_width=foot_half_width,
        foot_front=foot_front,
        foot_back=foot_back,
        yaw_friction=yaw_friction,
    )
    rows = [C]
    lb = [-np.inf] * C.shape[0]
    ub = np.zeros(C.shape[0]).tolist()
    ub[0] = -float(fz_low) if active else 0.0

    if include_wrench_box:
        rows.append(np.eye(NU_FOOT_WRENCH))
        if active:
            lb.extend([-float(f_max)] * NU_FOOT_WRENCH)
            ub.extend([float(f_max)] * NU_FOOT_WRENCH)
        else:
            lb.extend([0.0] * NU_FOOT_WRENCH)
            ub.extend([0.0] * NU_FOOT_WRENCH)

    return LinearConstraint(A=np.vstack(rows), lb=np.asarray(lb), ub=np.asarray(ub))


def double_support_wrench_constraints(
    *,
    mu: float = 1.0,
    foot_half_width: float = G1_FOOT_HALF_WIDTH,
    foot_front: float = G1_FOOT_FRONT,
    foot_back: float = G1_FOOT_BACK,
    yaw_friction: float = 0.046,
    fz_low: float = G1_TORSO_FZ_LOW,
    f_max: float = 1400.0,
    include_wrench_box: bool = True,
) -> LinearConstraint:
    """Constraints for double-support input u = [w_left, w_right]."""

    local = foot_wrench_planar_cone_constraints(
        active=True,
        mu=mu,
        foot_half_width=foot_half_width,
        foot_front=foot_front,
        foot_back=foot_back,
        yaw_friction=yaw_friction,
        fz_low=fz_low,
        f_max=f_max,
        include_wrench_box=include_wrench_box,
    )
    rows = np.zeros((2 * local.A.shape[0], NU_SRBD))
    rows[0 : local.A.shape[0], 0:NU_FOOT_WRENCH] = local.A
    rows[local.A.shape[0] :, NU_FOOT_WRENCH:NU_SRBD] = local.A
    return LinearConstraint(
        A=rows,
        lb=np.concatenate([local.lb, local.lb]),
        ub=np.concatenate([local.ub, local.ub]),
    )


def left_support_wrench_constraints(
    *,
    mu: float = 1.0,
    foot_half_width: float = G1_FOOT_HALF_WIDTH,
    foot_front: float = G1_FOOT_FRONT,
    foot_back: float = G1_FOOT_BACK,
    yaw_friction: float = 0.046,
    fz_low: float = G1_TORSO_FZ_LOW,
    f_max: float = 1400.0,
    include_wrench_box: bool = True,
) -> LinearConstraint:
    """Constraints for left single-support input u = [w_left]."""

    return foot_wrench_planar_cone_constraints(
        active=True,
        mu=mu,
        foot_half_width=foot_half_width,
        foot_front=foot_front,
        foot_back=foot_back,
        yaw_friction=yaw_friction,
        fz_low=fz_low,
        f_max=f_max,
        include_wrench_box=include_wrench_box,
    )


def right_support_wrench_constraints(
    *,
    mu: float = 1.0,
    foot_half_width: float = G1_FOOT_HALF_WIDTH,
    foot_front: float = G1_FOOT_FRONT,
    foot_back: float = G1_FOOT_BACK,
    yaw_friction: float = 0.046,
    fz_low: float = G1_TORSO_FZ_LOW,
    f_max: float = 1400.0,
    include_wrench_box: bool = True,
) -> LinearConstraint:
    """Constraints for right single-support input u = [w_right]."""

    return foot_wrench_planar_cone_constraints(
        active=True,
        mu=mu,
        foot_half_width=foot_half_width,
        foot_front=foot_front,
        foot_back=foot_back,
        yaw_friction=yaw_friction,
        fz_low=fz_low,
        f_max=f_max,
        include_wrench_box=include_wrench_box,
    )


def flight_wrench_constraints() -> LinearConstraint:
    """Empty constraints for flight input u = []."""

    return LinearConstraint(A=np.zeros((0, 0)), lb=np.zeros(0), ub=np.zeros(0))


def stacked_foot_wrench_constraints(
    contact_table: np.ndarray,
    *,
    mu: float = 1.0,
    foot_half_width: float = G1_FOOT_HALF_WIDTH,
    foot_front: float = G1_FOOT_FRONT,
    foot_back: float = G1_FOOT_BACK,
    yaw_friction: float = 0.046,
    fz_low: float = G1_TORSO_FZ_LOW,
    f_max: float = 1400.0,
    include_wrench_box: bool = True,
) -> LinearConstraint:
    """Stack per-knot, per-foot wrench constraints for an MPC horizon.

    contact_table has shape (horizon, 2). The decision vector is stacked as
    [u_0, u_1, ...], with each u_k in R^12.
    """

    contacts = np.asarray(contact_table, dtype=bool)
    if contacts.ndim != 2 or contacts.shape[1] != N_CONTACTS:
        raise ValueError(f"contact_table must have shape (horizon, 2), got {contacts.shape}")

    horizon = contacts.shape[0]
    n_var = horizon * NU_SRBD
    rows: list[np.ndarray] = []
    lb: list[float] = []
    ub: list[float] = []

    for knot in range(horizon):
        for leg in range(N_CONTACTS):
            local = foot_wrench_planar_cone_constraints(
                active=bool(contacts[knot, leg]),
                mu=mu,
                foot_half_width=foot_half_width,
                foot_front=foot_front,
                foot_back=foot_back,
                yaw_friction=yaw_friction,
                fz_low=fz_low,
                f_max=f_max,
                include_wrench_box=include_wrench_box,
            )
            row = np.zeros((local.A.shape[0], n_var))
            col = knot * NU_SRBD + leg * NU_FOOT_WRENCH
            row[:, col : col + NU_FOOT_WRENCH] = local.A
            rows.append(row)
            lb.extend(local.lb.tolist())
            ub.extend(local.ub.tolist())

    return LinearConstraint(A=np.vstack(rows), lb=np.asarray(lb), ub=np.asarray(ub))
