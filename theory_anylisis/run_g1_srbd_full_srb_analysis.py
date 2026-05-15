#!/usr/bin/env python3
"""Run SRBD-MPC on a full single-rigid-body plant.

The goal is not to reproduce MuJoCo. It is to isolate the model mismatch:
linearized SRBD-MPC controller versus nonlinear SRB Euler dynamics.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from alip_footstep import AlipFootstepPlanner, ContactPlan
from full_srb_sim import FullSrbEulerSimulator
from math_utils import GRAVITY, nominal_vertical_wrench, rot_to_rpy, rpy_to_rot
from models import (
    FullSrbState,
    G1_TORSO_FZ_LOW,
    G1_TORSO_INERTIA_BODY,
    G1_TORSO_MASS,
    GaitParams,
    MpcParams,
    RobotParams,
    SimParams,
)
from osqp_srbd_mpc import OsqpSrbdMpc
from stable_stand_mpc import StableStandMpc
from visualize_srb import Srb3DAnimator, VisualFrame


G1_INITIAL_COM = np.array([0.01507763, 0.00008226, 0.6993168])
G1_INITIAL_RIGHT_FOOT_REL_COM = np.array([-0.00781144, -0.11858872, -0.69349124])
G1_INITIAL_FOOT_X = G1_INITIAL_COM[0] + G1_INITIAL_RIGHT_FOOT_REL_COM[0]
G1_INITIAL_FOOT_Y = abs(G1_INITIAL_RIGHT_FOOT_REL_COM[1])
G1_BALANCED_STAND_COM = np.array([G1_INITIAL_FOOT_X, 0.0, G1_INITIAL_COM[2]])
G1_DEFAULT_MASS = G1_TORSO_MASS
G1_WALK_COM_X_CLAMP = 0.025
G1_WALK_COM_Y_CLAMP = 0.045
G1_DEFAULT_LATERAL_OFFSET = 0.0
G1_DEFAULT_PREWALK_SHIFT_Y = 0.035


@dataclass
class RunConfig:
    duration: float = 8.0
    start_time: float = 0.0
    stand_time: float = 2.0
    velocity_ramp: float = 2.0

    sim_dt: float = 0.002
    mpc_dt: float = 0.04
    horizon: int = 10
    mpc_solve_every: int = 1

    vx: float = 0.50
    vy: float = 0.0
    yaw_rate: float = 0.0

    t_swing: float = 0.30
    t_ds: float = 0.0
    step_width: float = 2.0 * G1_INITIAL_FOOT_Y
    lateral_step_min: float = 0.10
    lateral_step_max: float = 0.60
    initial_stance: str = "left"

    mass: float = G1_DEFAULT_MASS
    ixx: float | None = None
    iyy: float | None = None
    izz: float | None = None

    com_x: float = float(G1_BALANCED_STAND_COM[0])
    com_y: float = float(G1_BALANCED_STAND_COM[1])
    com_z: float = float(G1_INITIAL_COM[2])
    lateral_offset: float = G1_DEFAULT_LATERAL_OFFSET
    prewalk_shift_y: float = G1_DEFAULT_PREWALK_SHIFT_Y
    prewalk_shift_duration: float = 1.0

    foot_x: float = float(G1_INITIAL_FOOT_X)
    foot_y: float = float(G1_INITIAL_FOOT_Y)
    mu: float = 1.0
    fz_low: float = G1_TORSO_FZ_LOW

    roll0: float = 0.0
    pitch0: float = 0.0
    yaw0: float = 0.0

    csv: Path | None = None
    plot: bool = False
    print_wrench_summary: bool = False
    stand_only: bool = False
    stable_stand: bool = True

    animate: bool = True
    realtime: bool = False
    vis_stride: int = 10


# 修改这里的变量，然后直接运行：
# python theory_anylisis/run_g1_srbd_full_srb_analysis.py
CONFIG = RunConfig(
    duration=8.0,
    start_time=0.0,
    sim_dt=0.002,
    mpc_dt=0.04,
    horizon=10,
    mpc_solve_every=1,
    stand_time=2.0,
    velocity_ramp=2.0,
    vx=0.50,
    vy=0.0,
    yaw_rate=0.0,
    t_swing=0.30,
    t_ds=0.0,
    step_width=2.0 * G1_INITIAL_FOOT_Y,
    lateral_step_min=0.10,
    lateral_step_max=0.60,
    initial_stance="left",
    mass=G1_DEFAULT_MASS,
    ixx=None,
    iyy=None,
    izz=None,
    com_x=float(G1_BALANCED_STAND_COM[0]),
    com_y=float(G1_BALANCED_STAND_COM[1]),
    com_z=float(G1_INITIAL_COM[2]),
    lateral_offset=G1_DEFAULT_LATERAL_OFFSET,
    prewalk_shift_y=G1_DEFAULT_PREWALK_SHIFT_Y,
    prewalk_shift_duration=1.0,
    foot_x=float(G1_INITIAL_FOOT_X),
    foot_y=float(G1_INITIAL_FOOT_Y),
    mu=1.0,
    fz_low=G1_TORSO_FZ_LOW,
    roll0=0.0,
    pitch0=0.0,
    yaw0=0.0,
    csv=None,
    plot=False,
    print_wrench_summary=False,
    stand_only=False,
    stable_stand=True,
    animate=True,
    realtime=False,
    vis_stride=10,
)


def smooth_step(s: float) -> float:
    u = float(np.clip(s, 0.0, 1.0))
    return u * u * (3.0 - 2.0 * u)


def ramp_value(target: float, elapsed: float, time_to_reach: float) -> float:
    return target * min(1.0, max(0.0, elapsed) / max(1e-3, time_to_reach))


def prewalk_com_reference(
    t: float,
    walk_start_time: float,
    initial_com: np.ndarray,
    shift_y: float,
    shift_duration: float,
    enabled: bool,
) -> np.ndarray:
    ref = initial_com.copy()
    if not enabled:
        return ref
    shift_start = max(0.0, walk_start_time - max(1e-3, shift_duration))
    phase = smooth_step((t - shift_start) / max(1e-3, shift_duration))
    ref[1] += shift_y * phase
    return ref


def build_reference(
    x0: np.ndarray,
    com_des: np.ndarray,
    desired_v: np.ndarray,
    yaw_des: float,
    yaw_rate: float,
    mpc: MpcParams,
) -> np.ndarray:
    ref = np.zeros((13, mpc.horizon))
    for k in range(mpc.horizon):
        dt = (k + 1) * mpc.dt
        p_ref = com_des + dt * desired_v
        ref[:, k] = np.array(
            [
                0.0,
                0.0,
                yaw_des + dt * yaw_rate,
                p_ref[0],
                p_ref[1],
                p_ref[2],
                0.0,
                0.0,
                yaw_rate,
                desired_v[0],
                desired_v[1],
                desired_v[2],
                GRAVITY,
            ]
        )
    ref[0:3, 0] = x0[0:3]
    ref[3:6, 0] = x0[3:6]
    return ref


def standing_contact_plan(planner: AlipFootstepPlanner, horizon: int) -> ContactPlan:
    return ContactPlan(
        contact_table=np.ones((horizon, 2), dtype=int),
        contact_positions=[planner.feet.copy() for _ in range(horizon)],
        next_feet=planner.feet.copy(),
        next_swing_leg=-1,
        stance_leg=planner.stance_leg,
        phase=0.0,
    )


def nominal_double_support_wrench(robot: RobotParams) -> np.ndarray:
    wrench = nominal_vertical_wrench(0.5 * robot.mass * GRAVITY)
    out = np.zeros(12)
    out[:6] = wrench
    out[6:12] = wrench
    return out


def centroidal_angular_momentum(robot: RobotParams, state: FullSrbState) -> np.ndarray:
    return state.R @ robot.inertia_body @ state.R.T @ state.omega


def applied_centroidal_wrench(
    com: np.ndarray,
    foot_wrenches: np.ndarray,
    foot_positions: np.ndarray,
    contacts: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    force = np.zeros(3)
    moment = np.zeros(3)
    for leg in range(2):
        if not contacts[leg]:
            continue
        wrench = foot_wrenches[6 * leg : 6 * (leg + 1)]
        foot_force = wrench[:3]
        foot_moment = wrench[3:]
        force += foot_force
        moment += np.cross(foot_positions[leg] - com, foot_force) + foot_moment
    return force, moment


def single_support_roll_feasibility(
    com: np.ndarray,
    foot_wrenches: np.ndarray,
    foot_positions: np.ndarray,
    contacts: np.ndarray,
    robot: RobotParams,
) -> tuple[float, float, float, float]:
    active = np.flatnonzero(contacts)
    if len(active) != 1:
        return np.nan, np.nan, np.nan, np.nan
    leg = int(active[0])
    wrench = foot_wrenches[6 * leg : 6 * (leg + 1)]
    _, applied_moment = applied_centroidal_wrench(com, foot_wrenches, foot_positions, contacts)
    required_mx = wrench[3] - applied_moment[0]
    mx_limit = robot.foot_half_width * max(0.0, wrench[2])
    margin = mx_limit - abs(required_mx)
    foot_com_y = foot_positions[leg, 1] - com[1]
    return required_mx, mx_limit, margin, foot_com_y


def run(args: RunConfig) -> tuple[list[dict[str, float]], list[VisualFrame]]:
    inertia_body = G1_TORSO_INERTIA_BODY.copy()
    if args.ixx is not None or args.iyy is not None or args.izz is not None:
        diag = np.diag(inertia_body).copy()
        if args.ixx is not None:
            diag[0] = args.ixx
        if args.iyy is not None:
            diag[1] = args.iyy
        if args.izz is not None:
            diag[2] = args.izz
        inertia_body = np.diag(diag)

    robot = RobotParams(
        mass=args.mass,
        inertia_body=inertia_body,
        nominal_com_height=args.com_z,
        nominal_foot_y=0.5 * args.step_width,
        mu=args.mu,
        fz_low=args.fz_low,
    )
    mpc_params = MpcParams(horizon=args.horizon, dt=args.mpc_dt, solve_every=args.mpc_solve_every)
    gait = GaitParams(
        single_support_time=args.t_swing,
        double_support_time=args.t_ds,
        step_width=args.step_width,
        lateral_step_min=args.lateral_step_min,
        lateral_step_max=args.lateral_step_max,
        initial_stance=0 if args.initial_stance == "left" else 1,
    )
    sim = SimParams(
        dt=args.sim_dt,
        duration=args.duration,
        desired_v=np.array([args.vx, args.vy, 0.0]),
        desired_yaw_rate=args.yaw_rate,
        initial_com=np.array([args.com_x, args.com_y + args.lateral_offset, args.com_z]),
        initial_rpy=np.array([args.roll0, args.pitch0, args.yaw0]),
    )

    controller = OsqpSrbdMpc(robot, mpc_params)
    stand_controller = StableStandMpc(robot)
    planner = AlipFootstepPlanner(robot, gait)
    planner.feet[:] = np.array(
        [
            [args.foot_x, args.foot_y, 0.0],
            [args.foot_x, -args.foot_y, 0.0],
        ],
        dtype=float,
    )
    plant = FullSrbEulerSimulator(robot, sim.dt)
    state = FullSrbState.from_arrays(
        rpy_to_rot(sim.initial_rpy),
        sim.initial_com,
        sim.initial_omega,
        sim.initial_vel,
    )

    n_steps = int(np.ceil(sim.duration / sim.dt))
    solve_period = max(1, int(mpc_params.solve_every))
    last_wrench = nominal_double_support_wrench(robot)
    last_status = "not_solved"
    last_contacts: np.ndarray | None = None
    log: list[dict[str, float]] = []
    frames: list[VisualFrame] = []
    walk_start_time = np.inf if args.stand_only else max(args.start_time, args.stand_time)
    walking_started = False
    walk_js_pos = sim.initial_com[:2].copy()
    walk_initial_pos = walk_js_pos.copy()
    yaw_des = sim.initial_rpy[2]
    walk_anchor_com = sim.initial_com.copy()

    for i in range(n_steps):
        t = i * sim.dt
        walking_enabled = t >= walk_start_time
        stand_target_com = prewalk_com_reference(
            t,
            walk_start_time,
            sim.initial_com,
            args.prewalk_shift_y,
            args.prewalk_shift_duration,
            not args.stand_only,
        )
        if walking_enabled and not walking_started:
            walking_started = True
            walk_anchor_com = stand_target_com.copy()
            walk_anchor_com[2] = robot.nominal_com_height
            walk_js_pos = state.p[:2].copy()
            walk_initial_pos = walk_js_pos.copy()
            planner.start_walking()

        if walking_enabled:
            walk_elapsed = t - walk_start_time
            vel_des = np.array(
                [
                    ramp_value(sim.desired_v[0], walk_elapsed, args.velocity_ramp),
                    ramp_value(sim.desired_v[1], walk_elapsed, args.velocity_ramp),
                    0.0,
                ]
            )
            yaw_rate_des = ramp_value(sim.desired_yaw_rate, walk_elapsed, 1.0)
            walk_js_pos += vel_des[:2] * sim.dt
            yaw_des += yaw_rate_des * sim.dt
            yaw_cur = rot_to_rpy(state.R)[2]
            angular_momentum = centroidal_angular_momentum(robot, state)
            planner.step(sim.dt, state.p, state.v, vel_des, yaw_des, angular_momentum, current_yaw=yaw_cur)
            contact_plan = planner.preview(
                mpc_params.horizon,
                mpc_params.dt,
                state.p,
                state.v,
                vel_des,
                yaw_des,
                angular_momentum,
                current_yaw=yaw_cur,
            )
        else:
            vel_des = np.zeros(3)
            yaw_rate_des = 0.0
            contact_plan = standing_contact_plan(planner, mpc_params.horizon)

        contacts = np.asarray(planner.contacts(), dtype=int)

        x = plant.state_vector(state)
        if walking_enabled:
            target_com = walk_anchor_com.copy()
            target_com[0] += np.clip(walk_js_pos[0] - walk_initial_pos[0], -G1_WALK_COM_X_CLAMP, G1_WALK_COM_X_CLAMP)
            target_com[1] += np.clip(walk_js_pos[1] - walk_initial_pos[1], -G1_WALK_COM_Y_CLAMP, G1_WALK_COM_Y_CLAMP)
            target_com[2] = robot.nominal_com_height
        else:
            target_com = stand_target_com.copy()
            target_com[2] = robot.nominal_com_height

        force_resolve = last_contacts is None or not np.array_equal(contacts, last_contacts)
        if i % solve_period == 0 or force_resolve:
            if args.stable_stand and not walking_enabled:
                result = stand_controller.solve(state, target_com, yaw_des, planner.feet, contacts)
            else:
                ref = build_reference(x, target_com, vel_des, yaw_des, yaw_rate_des, mpc_params)
                result = controller.solve(
                    x,
                    state.R,
                    ref,
                    contact_plan.contact_table,
                    contact_plan.contact_positions,
                )
            last_wrench = result.wrenches
            last_status = result.status
            last_contacts = contacts.copy()

        applied_force, applied_moment = applied_centroidal_wrench(state.p, last_wrench, planner.feet, contacts)
        required_mx, mx_limit, mx_margin, foot_com_y = single_support_roll_feasibility(
            state.p,
            last_wrench,
            planner.feet,
            contacts,
            robot,
        )
        state = plant.step(state, last_wrench, planner.feet, contacts)
        rpy = rot_to_rpy(state.R)
        if args.animate:
            frames.append(
                VisualFrame(
                    time=t,
                    R=state.R.copy(),
                    p=state.p.copy(),
                    feet=planner.feet.copy(),
                    next_feet=contact_plan.next_feet.copy(),
                    next_swing_leg=contact_plan.next_swing_leg,
                    contacts=contacts.copy(),
                    wrenches=last_wrench.copy(),
                )
            )
        log.append(
            {
                "time": t,
                "roll": rpy[0],
                "pitch": rpy[1],
                "yaw": rpy[2],
                "com_x": state.p[0],
                "com_y": state.p[1],
                "com_z": state.p[2],
                "vel_x": state.v[0],
                "vel_y": state.v[1],
                "vel_z": state.v[2],
                "omega_x": state.omega[0],
                "omega_y": state.omega[1],
                "omega_z": state.omega[2],
                "des_x": target_com[0],
                "des_y": target_com[1],
                "des_z": target_com[2],
                "lateral_offset": args.lateral_offset,
                "prewalk_shift_y": args.prewalk_shift_y,
                "vel_des_x": vel_des[0],
                "vel_des_y": vel_des[1],
                "yaw_rate_des": yaw_rate_des,
                "walking_enabled": float(walking_enabled),
                "left_contact": float(contacts[0]),
                "right_contact": float(contacts[1]),
                "stance_leg": float(contact_plan.stance_leg),
                "phase": contact_plan.phase,
                "next_swing_leg": float(contact_plan.next_swing_leg),
                "next_left_foot_x": contact_plan.next_feet[0, 0],
                "next_left_foot_y": contact_plan.next_feet[0, 1],
                "next_right_foot_x": contact_plan.next_feet[1, 0],
                "next_right_foot_y": contact_plan.next_feet[1, 1],
                "left_step_x": contact_plan.next_feet[0, 0] - planner.feet[0, 0],
                "left_step_y": contact_plan.next_feet[0, 1] - planner.feet[0, 1],
                "right_step_x": contact_plan.next_feet[1, 0] - planner.feet[1, 0],
                "right_step_y": contact_plan.next_feet[1, 1] - planner.feet[1, 1],
                "w_l_fx": last_wrench[0],
                "w_l_fy": last_wrench[1],
                "w_l_fz": last_wrench[2],
                "w_l_mx": last_wrench[3],
                "w_l_my": last_wrench[4],
                "w_l_mz": last_wrench[5],
                "w_r_fx": last_wrench[6],
                "w_r_fy": last_wrench[7],
                "w_r_fz": last_wrench[8],
                "w_r_mx": last_wrench[9],
                "w_r_my": last_wrench[10],
                "w_r_mz": last_wrench[11],
                "sum_fx": applied_force[0],
                "sum_fy": applied_force[1],
                "sum_fz": applied_force[2],
                "sum_mx": applied_moment[0],
                "sum_my": applied_moment[1],
                "sum_mz": applied_moment[2],
                "required_mx_zero_roll": required_mx,
                "mx_limit_active": mx_limit,
                "mx_margin_active": mx_margin,
                "active_foot_minus_com_y": foot_com_y,
                "fz_l": last_wrench[2],
                "fz_r": last_wrench[8],
                "status_ok": float("solved" in last_status.lower() or "optimal" in last_status.lower()),
            }
        )
    return log, frames


def write_csv(path: Path, log: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(log[0].keys()))
        writer.writeheader()
        writer.writerows(log)


def plot_log(log: list[dict[str, float]]) -> None:
    t = np.array([row["time"] for row in log])
    com = np.array([[row["com_x"], row["com_y"], row["com_z"]] for row in log])
    des = np.array([[row["des_x"], row["des_y"], row["des_z"]] for row in log])
    rpy = np.array([[row["roll"], row["pitch"], row["yaw"]] for row in log])
    fz = np.array([[row["fz_l"], row["fz_r"]] for row in log])

    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
    axs[0].plot(t, com[:, 0], label="com x")
    axs[0].plot(t, des[:, 0], "--", label="des x")
    axs[0].plot(t, com[:, 1], label="com y")
    axs[0].plot(t, des[:, 1], "--", label="des y")
    axs[0].legend()
    axs[0].set_ylabel("COM xy [m]")

    axs[1].plot(t, com[:, 2], label="com z")
    axs[1].plot(t, des[:, 2], "--", label="des z")
    axs[1].legend()
    axs[1].set_ylabel("COM z [m]")

    axs[2].plot(t, rpy[:, 0], label="roll")
    axs[2].plot(t, rpy[:, 1], label="pitch")
    axs[2].plot(t, rpy[:, 2], label="yaw")
    axs[2].legend()
    axs[2].set_ylabel("RPY [rad]")

    axs[3].plot(t, fz[:, 0], label="left Fz")
    axs[3].plot(t, fz[:, 1], label="right Fz")
    axs[3].legend()
    axs[3].set_ylabel("Fz [N]")
    axs[3].set_xlabel("time [s]")
    fig.tight_layout()
    plt.show()


def print_wrench_summary(log: list[dict[str, float]]) -> None:
    ss = [
        row
        for row in log
        if row["walking_enabled"] > 0.5 and abs(row["left_contact"] + row["right_contact"] - 1.0) < 1e-9
    ]
    if not ss:
        print("single support summary: no single-support samples")
        return
    max_roll = max(ss, key=lambda row: abs(row["roll"]))
    max_sum_mx = max(ss, key=lambda row: abs(row["sum_mx"]))
    min_margin = min(ss, key=lambda row: row["mx_margin_active"])
    print(
        "single support summary: "
        f"t=[{ss[0]['time']:.3f}, {ss[-1]['time']:.3f}], "
        f"max|roll|={abs(max_roll['roll']):.4f} at {max_roll['time']:.3f}, "
        f"max|sum_mx|={abs(max_sum_mx['sum_mx']):.4f} Nm at {max_sum_mx['time']:.3f}, "
        f"min mx margin={min_margin['mx_margin_active']:.4f} Nm at {min_margin['time']:.3f}"
    )


def main() -> None:
    args = CONFIG
    log, frames = run(args)
    final = log[-1]
    if args.csv is not None:
        write_csv(args.csv, log)
        print(f"wrote {args.csv}")
    print(
        "final com="
        f"({final['com_x']:.3f}, {final['com_y']:.3f}, {final['com_z']:.3f}), "
        f"rpy=({final['roll']:.4f}, {final['pitch']:.4f}, {final['yaw']:.4f})"
    )
    if args.plot:
        plot_log(log)
    if args.print_wrench_summary:
        print_wrench_summary(log)
    if args.animate:
        Srb3DAnimator(frames, stride=args.vis_stride, realtime=args.realtime).show()


if __name__ == "__main__":
    main()
