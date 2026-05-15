"""C++ FootPlacement-style HLIP/LQR foot placement and contact preview."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import scipy.linalg

from math_utils import GRAVITY, rz
from models import GaitParams, RobotParams


LEFT = 0
RIGHT = 1


@dataclass
class ContactPlan:
    contact_table: np.ndarray
    contact_positions: list[np.ndarray]
    next_feet: np.ndarray
    next_swing_leg: int
    stance_leg: int
    phase: float


class AlipFootstepPlanner:
    """Minimal Python mirror of the C++ GaitScheduler + FootPlacement pipeline.

    The planner updates at the simulation dt, just like the C++ demo calls
    GaitScheduler::step() and FootPlacement::getSwingPos() every MuJoCo step.
    Swing trajectories are ignored; only contact state and touchdown targets
    are exposed to SRBD-MPC.
    """

    def __init__(self, robot: RobotParams, gait: GaitParams):
        self.robot = robot
        self.gait = gait
        y = 0.5 * gait.step_width
        self.feet = np.array([[0.0, y, 0.0], [0.0, -y, 0.0]], dtype=float)
        self.stance_leg = gait.initial_stance
        self.phase_time = 0.0
        self.in_double_support = True
        self.target_yaw = 0.0

    def reset(self) -> None:
        y = 0.5 * self.gait.step_width
        self.feet[:] = np.array([[0.0, y, 0.0], [0.0, -y, 0.0]])
        self.stance_leg = self.gait.initial_stance
        self.phase_time = 0.0
        self.in_double_support = True
        self.target_yaw = 0.0

    def start_walking(self) -> None:
        self.stance_leg = self.gait.initial_stance
        self.phase_time = 0.0
        self.in_double_support = self.gait.double_support_time > 1e-9

    def _opposite(self, leg: int) -> int:
        return RIGHT if leg == LEFT else LEFT

    def _duration(self) -> float:
        return self.gait.double_support_time if self.in_double_support else self.gait.single_support_time

    def contacts(self) -> tuple[bool, bool]:
        if self.in_double_support:
            return True, True
        return self.stance_leg == LEFT, self.stance_leg == RIGHT

    @staticmethod
    def _dlqr_gain_2x1(A: np.ndarray, B: np.ndarray) -> np.ndarray:
        Q = np.eye(2)
        R = np.array([[20.0]])
        P = scipy.linalg.solve_discrete_are(A, B.reshape(2, 1), Q, R)
        denom = float(B.reshape(1, 2) @ P @ B.reshape(2, 1) + R[0, 0])
        return -((B.reshape(1, 2) @ P @ A) / denom).reshape(2)

    def _hlip_axis(self, z_nom: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if self.gait.use_angular_momentum_state:
            ass = np.array([[0.0, 1.0 / z_nom], [GRAVITY, 0.0]])
            ads = np.array([[0.0, 1.0 / z_nom], [0.0, 0.0]])
        else:
            ass = np.array([[0.0, 1.0], [GRAVITY / z_nom, 0.0]])
            ads = np.array([[0.0, 1.0], [0.0, 0.0]])

        t_ss = max(1e-3, self.gait.single_support_time)
        t_ds = max(0.0, self.gait.double_support_time)
        step_input_map = np.array([-1.0, 0.0])
        exp_ss = scipy.linalg.expm(t_ss * ass)
        exp_ds = scipy.linalg.expm(t_ds * ads)
        b_step = exp_ss @ step_input_map
        a_step = exp_ss @ exp_ds
        k_lqr = self._dlqr_gain_2x1(a_step, b_step)
        return ass, a_step, b_step, k_lqr

    @staticmethod
    def _period_one_target(a_step: np.ndarray, b_step: np.ndarray, nominal_step: float) -> np.ndarray:
        return np.linalg.solve(np.eye(2) - a_step, b_step * nominal_step)

    @staticmethod
    def _period_two_target(
        a_step: np.ndarray,
        b_step: np.ndarray,
        nominal_step_this: float,
        nominal_step_next: float,
    ) -> np.ndarray:
        rhs = a_step @ (b_step * nominal_step_this) + b_step * nominal_step_next
        return np.linalg.solve(np.eye(2) - a_step @ a_step, rhs)

    def _reduced_state(
        self,
        stance: np.ndarray,
        com: np.ndarray,
        vel: np.ndarray,
        yaw: float,
        angular_momentum: np.ndarray | None,
    ) -> np.ndarray:
        RtargetT = rz(yaw).T
        com_rel = RtargetT @ (com - stance)
        com_vel = RtargetT @ vel
        if self.gait.use_angular_momentum_state:
            normalized_angular_momentum = np.zeros(3)
            if angular_momentum is not None and self.robot.mass > 1e-6:
                normalized_angular_momentum = RtargetT @ (angular_momentum / self.robot.mass)
            orbital = np.cross(com_rel, com_vel)
            pivot_lx = -normalized_angular_momentum[0] - orbital[0]
            pivot_ly = normalized_angular_momentum[1] + orbital[1]
            return np.array([com_rel[0], pivot_ly, com_rel[1], pivot_lx])
        return np.array([com_rel[0], com_vel[0], com_rel[1], com_vel[1]])

    def _plan_hlip_footstep(
        self,
        reduced_state: np.ndarray,
        t_remain: float,
        desired_v: np.ndarray,
        yaw: float,
    ) -> np.ndarray:
        z_nom = max(0.45, self.robot.nominal_com_height)
        t_ss = max(1e-3, self.gait.single_support_time)
        step_width_nominal = max(0.20, self.gait.step_width)
        ass, a_step, b_step, k_lqr = self._hlip_axis(z_nom)

        sagittal_pre_impact = scipy.linalg.expm(max(0.0, t_remain) * ass) @ reduced_state[:2]
        lateral_pre_impact = scipy.linalg.expm(max(0.0, t_remain) * ass) @ reduced_state[2:4]

        desired_vx = float(desired_v @ rz(yaw)[:, 0])
        desired_vy = float(desired_v @ rz(yaw)[:, 1])
        sagittal_nominal_step = desired_vx * t_ss
        sagittal_target = self._period_one_target(a_step, b_step, sagittal_nominal_step)

        left_stance = self.stance_leg == LEFT
        left_nominal_step = -step_width_nominal
        right_nominal_step = 2.0 * desired_vy * t_ss - left_nominal_step
        if left_stance:
            lateral_target = self._period_two_target(a_step, b_step, left_nominal_step, right_nominal_step)
            lateral_nominal_step = left_nominal_step
        else:
            lateral_target = self._period_two_target(a_step, b_step, right_nominal_step, left_nominal_step)
            lateral_nominal_step = right_nominal_step

        return np.array(
            [
                k_lqr @ (sagittal_pre_impact - sagittal_target) + sagittal_nominal_step,
                k_lqr @ (lateral_pre_impact - lateral_target) + lateral_nominal_step,
            ]
        )

    def _target_swing_foot(
        self,
        com: np.ndarray,
        vel: np.ndarray,
        desired_v: np.ndarray,
        target_yaw: float,
        angular_momentum: np.ndarray | None = None,
        current_yaw: float | None = None,
        phase: float | None = None,
    ) -> np.ndarray:
        swing_leg = self._opposite(self.stance_leg)
        stance = self.feet[self.stance_leg]
        phi = self.phase() if phase is None else float(np.clip(phase, 0.0, 1.0))
        t_remain = max(0.0, (1.0 - phi) * self.gait.single_support_time)
        yaw_cur = target_yaw if current_yaw is None else current_yaw
        reduced_state = self._reduced_state(stance, com, vel, target_yaw, angular_momentum)
        step_local_xy = self._plan_hlip_footstep(reduced_state, t_remain, desired_v, target_yaw)
        step_in_base_yaw = rz(target_yaw - yaw_cur) @ np.array([step_local_xy[0], step_local_xy[1], 0.0])
        if self.stance_leg == LEFT:
            step_in_base_yaw[1] = np.clip(
                step_in_base_yaw[1],
                -self.gait.lateral_step_max,
                -self.gait.lateral_step_min,
            )
        else:
            step_in_base_yaw[1] = np.clip(
                step_in_base_yaw[1],
                self.gait.lateral_step_min,
                self.gait.lateral_step_max,
            )
        target = stance + rz(yaw_cur) @ step_in_base_yaw
        target[2] = stance[2] - 0.01
        return target

    def phase(self) -> float:
        duration = self._duration()
        if self.in_double_support or duration <= 1e-9:
            return 0.0
        return float(np.clip(self.phase_time / duration, 0.0, 1.0))

    def step(
        self,
        dt: float,
        com: np.ndarray,
        vel: np.ndarray,
        desired_v: np.ndarray,
        yaw: float = 0.0,
        angular_momentum: np.ndarray | None = None,
        current_yaw: float | None = None,
    ) -> None:
        self.target_yaw = yaw
        self.phase_time += dt
        while self.phase_time >= self._duration():
            duration = self._duration()
            if duration <= 1e-9:
                self.phase_time = 0.0
            else:
                self.phase_time -= duration
            if self.in_double_support:
                self.in_double_support = False
            else:
                swing_leg = self._opposite(self.stance_leg)
                self.feet[swing_leg] = self._target_swing_foot(
                    com,
                    vel,
                    desired_v,
                    yaw,
                    angular_momentum=angular_momentum,
                    current_yaw=current_yaw,
                    phase=1.0,
                )
                self.stance_leg = swing_leg
                self.in_double_support = self.gait.double_support_time > 1e-9
            if self._duration() <= 1e-9:
                break

    def _preview_mode(self, t_future: float) -> tuple[bool, int, float]:
        in_ds = self.in_double_support
        stance = self.stance_leg
        phase = self.phase_time
        rem = max(0.0, t_future)
        while rem > 1e-12:
            duration = self.gait.double_support_time if in_ds else self.gait.single_support_time
            if duration <= 1e-9:
                if in_ds:
                    in_ds = False
                    phase = 0.0
                    continue
                duration = 1e-9
            left = max(0.0, duration - phase)
            if rem <= left + 1e-12:
                phase += rem
                break
            rem -= left
            phase = 0.0
            if in_ds:
                in_ds = False
            else:
                stance = self._opposite(stance)
                in_ds = self.gait.double_support_time > 1e-9
        return in_ds, stance, phase

    def preview(
        self,
        horizon: int,
        mpc_dt: float,
        com: np.ndarray,
        vel: np.ndarray,
        desired_v: np.ndarray,
        yaw: float = 0.0,
        angular_momentum: np.ndarray | None = None,
        current_yaw: float | None = None,
    ) -> ContactPlan:
        table = np.ones((horizon, 2), dtype=int)
        positions: list[np.ndarray] = []
        next_feet = self.feet.copy()
        if not self.in_double_support:
            swing = self._opposite(self.stance_leg)
            next_feet[swing] = self._target_swing_foot(
                com,
                vel,
                desired_v,
                yaw,
                angular_momentum=angular_momentum,
                current_yaw=current_yaw,
            )

        for k in range(horizon):
            in_ds, stance, _ = self._preview_mode(k * mpc_dt)
            if in_ds:
                table[k, :] = 1
            else:
                table[k, LEFT] = 1 if stance == LEFT else 0
                table[k, RIGHT] = 1 if stance == RIGHT else 0
            pos = self.feet.copy()
            for leg in (LEFT, RIGHT):
                if table[k, leg] and not self.contacts()[leg]:
                    pos[leg] = next_feet[leg]
            positions.append(pos)

        phase = self.phase()
        next_swing_leg = self._opposite(self.stance_leg) if not self.in_double_support else -1
        return ContactPlan(table, positions, next_feet, next_swing_leg, self.stance_leg, phase)
