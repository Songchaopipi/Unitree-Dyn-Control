"""Minimal 3D visualization for the full SRB analysis."""

from __future__ import annotations

import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from models import G1_FOOT_BACK, G1_FOOT_FRONT, G1_FOOT_HALF_WIDTH


@dataclass
class VisualFrame:
    time: float
    R: np.ndarray
    p: np.ndarray
    feet: np.ndarray
    next_feet: np.ndarray
    next_swing_leg: int
    contacts: np.ndarray
    wrenches: np.ndarray


def _ellipsoid_mesh(radii: np.ndarray, n_u: int = 36, n_v: int = 18) -> np.ndarray:
    u = np.linspace(0.0, 2.0 * np.pi, n_u)
    v = np.linspace(0.0, np.pi, n_v)
    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
    return np.stack([x, y, z], axis=-1)


def _foot_polygon(
    center: np.ndarray,
    yaw: float,
    front: float = G1_FOOT_FRONT,
    back: float = G1_FOOT_BACK,
    half_width: float = G1_FOOT_HALF_WIDTH,
) -> np.ndarray:
    front_width = 1.2 * half_width
    rear_width = half_width
    local = np.array(
        [
            [front, front_width, 0.0],
            [front, -front_width, 0.0],
            [back, -rear_width, 0.0],
            [back, rear_width, 0.0],
        ]
    )
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return center + local @ Rz.T


class Srb3DAnimator:
    def __init__(
        self,
        frames: list[VisualFrame],
        body_radii: tuple[float, float, float] = (0.15, 0.15, 0.36),
        stride: int = 10,
        realtime: bool = False,
    ):
        self.frames = frames[:: max(1, stride)]
        self.body_radii = np.asarray(body_radii, dtype=float)
        self.local_body_mesh = _ellipsoid_mesh(self.body_radii)
        self.realtime = realtime
        self.trail: list[np.ndarray] = []

        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_zlabel("z [m]")
        self.ax.set_box_aspect((1.0, 1.0, 1.0))
        self.ax.view_init(elev=22.0, azim=-55.0)

    def _set_limits(self) -> None:
        pts = np.array([frame.p for frame in self.frames])
        feet = np.concatenate([frame.feet for frame in self.frames], axis=0)
        all_pts = np.vstack([pts, feet])
        center = all_pts.mean(axis=0)
        span = np.maximum(all_pts.max(axis=0) - all_pts.min(axis=0), np.array([0.8, 0.8, 0.8]))
        xy_half_range = 0.55 * max(span[0], span[1])
        z_min = 0.0
        z_max = max(1.0, all_pts[:, 2].max() + self.body_radii[2] + 0.15)
        z_range = z_max - z_min
        self.ax.set_xlim(center[0] - xy_half_range, center[0] + xy_half_range)
        self.ax.set_ylim(center[1] - xy_half_range, center[1] + xy_half_range)
        self.ax.set_zlim(z_min, z_max)
        self.ax.set_box_aspect((2.0 * xy_half_range, 2.0 * xy_half_range, z_range))

    def _draw_foot(self, foot: np.ndarray, active: bool, yaw: float = 0.0) -> None:
        face = "#f59e0b" if active else "#c7cdd6"
        edge = "#92400e" if active else "#6b7280"
        foot_poly = _foot_polygon(foot, yaw)
        patch = Poly3DCollection([foot_poly], facecolor=face, edgecolor=edge, linewidth=1.1, alpha=0.92)
        self.ax.add_collection3d(patch)
        corners = np.vstack([foot_poly, foot_poly[0]])
        self.ax.plot(corners[:, 0], corners[:, 1], corners[:, 2] + 0.002, color=edge, linewidth=1.1)
        self.ax.scatter([foot[0]], [foot[1]], [foot[2] + 0.006], color=edge, s=12)

    def _draw_next_foot(self, foot: np.ndarray, yaw: float = 0.0) -> None:
        next_poly = _foot_polygon(foot, yaw)
        next_patch = Poly3DCollection(
            [next_poly],
            facecolor="#86efac",
            edgecolor="#16a34a",
            linewidth=1.0,
            alpha=0.35,
        )
        self.ax.add_collection3d(next_patch)
        self.ax.scatter(
            [foot[0]],
            [foot[1]],
            [foot[2] + 0.018],
            color="#22c55e",
            s=65,
            marker="x",
            linewidths=2.5,
        )

    def _draw_grf(self, foot: np.ndarray, force: np.ndarray) -> None:
        scale = 1.0 / 800.0
        self.ax.quiver(
            foot[0],
            foot[1],
            foot[2] + 0.01,
            force[0] * scale,
            force[1] * scale,
            force[2] * scale,
            color="#ef4444",
            linewidth=1.8,
            arrow_length_ratio=0.18,
        )

    def draw_frame(self, frame: VisualFrame) -> None:
        self.ax.cla()
        self._set_limits()
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_zlabel("z [m]")
        self.ax.set_title(f"Full SRB state  t={frame.time:.2f}s")
        self.ax.view_init(elev=22.0, azim=-55.0)

        world_body = np.einsum("ij,uvj->uvi", frame.R, self.local_body_mesh) + frame.p
        self.ax.plot_surface(
            world_body[:, :, 0],
            world_body[:, :, 1],
            world_body[:, :, 2],
            color="#7aa6d8",
            edgecolor="#35506b",
            linewidth=0.25,
            alpha=0.62,
            antialiased=True,
            rstride=1,
            cstride=1,
        )

        axis_len = 0.22
        colors = ["#d62728", "#2ca02c", "#1f77b4"]
        for i, color in enumerate(colors):
            axis = frame.R[:, i] * axis_len
            self.ax.quiver(frame.p[0], frame.p[1], frame.p[2], axis[0], axis[1], axis[2], color=color, linewidth=2.0)

        self.trail.append(frame.p.copy())
        trail = np.array(self.trail[-250:])
        self.ax.plot(trail[:, 0], trail[:, 1], trail[:, 2], color="#111827", linewidth=1.2)
        self.ax.scatter([frame.p[0]], [frame.p[1]], [frame.p[2]], color="#111827", s=28)

        for leg in range(2):
            foot = frame.feet[leg]
            self._draw_foot(foot, active=bool(frame.contacts[leg]))
            next_foot = frame.next_feet[leg]
            if frame.next_swing_leg == leg:
                self._draw_next_foot(next_foot)
                self.ax.plot(
                    [foot[0], next_foot[0]],
                    [foot[1], next_foot[1]],
                    [foot[2], next_foot[2]],
                    color="#22c55e",
                    linestyle="--",
                    linewidth=1.0,
                )
            if frame.contacts[leg]:
                f = frame.wrenches[6 * leg : 6 * leg + 3]
                self._draw_grf(foot, f)

        ground_x = np.linspace(self.ax.get_xlim()[0], self.ax.get_xlim()[1], 2)
        ground_y = np.linspace(self.ax.get_ylim()[0], self.ax.get_ylim()[1], 2)
        gx, gy = np.meshgrid(ground_x, ground_y)
        self.ax.plot_surface(gx, gy, np.zeros_like(gx), color="#e5e7eb", alpha=0.25, linewidth=0)

    def show(self) -> None:
        if not self.frames:
            return
        last_t = self.frames[0].time
        for frame in self.frames:
            start = time.time()
            self.draw_frame(frame)
            plt.pause(0.001)
            if self.realtime:
                dt = max(0.0, frame.time - last_t)
                time.sleep(max(0.0, dt - (time.time() - start)))
                last_t = frame.time
        plt.show()
