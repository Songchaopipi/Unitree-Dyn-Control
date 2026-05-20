import os
import sys
from pathlib import Path

cwd = os.getcwd()
script_dir = str(Path(__file__).resolve().parent)

for p in [cwd, script_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

from theory_anylisis.dynamics import foot_wrench_planar_cone_constraints



def linear_constraint_to_scipy(A, lb, ub):
    """
    Convert lb <= A x <= ub to scipy linprog form:
        A_ub x <= b_ub
    """
    A_ub_list = []
    b_ub_list = []

    for i in range(A.shape[0]):
        if np.isfinite(ub[i]):
            A_ub_list.append(A[i])
            b_ub_list.append(ub[i])

        if np.isfinite(lb[i]):
            A_ub_list.append(-A[i])
            b_ub_list.append(-lb[i])

    return np.vstack(A_ub_list), np.asarray(b_ub_list)


def fibonacci_sphere(n):
    """
    Approximately uniform directions on S^2.
    Here each direction is in projected moment space:
        d = [d_Mx, d_My, d_Mz]
    """
    dirs = []
    golden = (1.0 + np.sqrt(5.0)) / 2.0

    for k in range(n):
        z = 1.0 - 2.0 * (k + 0.5) / n
        r = np.sqrt(max(0.0, 1.0 - z * z))
        theta = 2.0 * np.pi * k / golden

        x = r * np.cos(theta)
        y = r * np.sin(theta)

        dirs.append([x, y, z])

    return np.asarray(dirs)


def project_6d_wrench_cone_to_moment_space(
    *,
    mu=0.6,
    foot_half_width=0.025,
    foot_front=0.12,
    foot_back=-0.05,
    yaw_friction=0.046,
    fz_low=20.0,
    f_max=1400.0,
    include_wrench_box=True,
    num_dirs=2000,
):
    """
    Project 6D wrench feasible set

        W = { w in R^6 | lb <= A w <= ub }

    to moment space:

        M = { [Mx,My,Mz] | exists [Fx,Fy,Fz],
              [Fx,Fy,Fz,Mx,My,Mz] in W }

    Projection is sampled by support-function LPs:

        max_d d^T [Mx,My,Mz]
        s.t.  lb <= A w <= ub
    """

    con = foot_wrench_planar_cone_constraints(
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

    A_ub, b_ub = linear_constraint_to_scipy(con.A, con.lb, con.ub)

    dirs = fibonacci_sphere(num_dirs)

    moment_points = []
    full_wrenches = []

    for d in dirs:
        c = np.zeros(6)

        # 关键：目标只优化 [Mx, My, Mz]
        # linprog 是 minimize，所以 max d^T M 等价于 min -d^T M
        c[3:6] = -d

        res = linprog(
            c,
            A_ub=A_ub,
            b_ub=b_ub,
            bounds=[(None, None)] * 6,
            method="highs",
        )

        if res.success:
            w = res.x
            moment_points.append(w[3:6])
            full_wrenches.append(w)

    return np.asarray(moment_points), np.asarray(full_wrenches), con


def plot_projected_moment_polytope(
    points,
    title="Projection of 6D wrench cone onto Mx-My-Mz",
):
    hull = ConvexHull(points)

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    for simplex in hull.simplices:
        tri = points[simplex]
        tri_closed = np.vstack([tri, tri[0]])
        ax.plot(
            tri_closed[:, 0],
            tri_closed[:, 1],
            tri_closed[:, 2],
            linewidth=0.6,
        )

    ax.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        s=4,
        alpha=0.35,
    )

    ax.set_xlabel("Mx [Nm]")
    ax.set_ylabel("My [Nm]")
    ax.set_zlabel("Mz [Nm]")
    ax.set_title(title)

    max_range = np.max(np.ptp(points, axis=0))
    center = np.mean(points, axis=0)

    ax.set_xlim(center[0] - max_range / 2, center[0] + max_range / 2)
    ax.set_ylim(center[1] - max_range / 2, center[1] + max_range / 2)
    ax.set_zlim(center[2] - max_range / 2, center[2] + max_range / 2)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    points_M, full_wrenches, con = project_6d_wrench_cone_to_moment_space(
        mu=0.6,
        foot_half_width=0.025,
        foot_front=0.12,
        foot_back=-0.05,
        yaw_friction=0.046,
        fz_low=20.0,
        f_max=1400.0,
        include_wrench_box=True,
        num_dirs=3000,
    )

    print("Projected moment points shape:", points_M.shape)
    print("Mx range:", points_M[:, 0].min(), points_M[:, 0].max())
    print("My range:", points_M[:, 1].min(), points_M[:, 1].max())
    print("Mz range:", points_M[:, 2].min(), points_M[:, 2].max())

    plot_projected_moment_polytope(points_M)