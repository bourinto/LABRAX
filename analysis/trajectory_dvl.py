"""
Reconstruct and visualize an AUV trajectory from a CSV log containing DVL
velocity, depth, and attitude measurements.

Assumptions:
- The world frame uses the NED convention:
    X = North
    Y = East
    Z = depth (positive downward)
- DVL velocities are expressed in the vehicle body frame:
    X = forward (surge)
    Y = right (sway)
    Z = down (heave)
- Attitude is provided as:
    yaw   = heading
    pitch = attitude (positive nose up)
    roll  = roll angle
- Rotations use the ZYX convention (yaw → pitch → roll).
- All three DVL velocity components are used during the transformation to the
  NED frame and integration. The resulting vertical coordinate is then
  replaced with the IMU depth measurement.
- Samples whose DVL speed exceeds MAX_DVL_SPEED are rejected.

Method:
1. Load and filter the log.
2. Transform velocities from the body frame to the world frame.
3. Integrate velocities with the trapezoidal rule to estimate position.
4. Replace the Z coordinate with the measured depth.
5. Compensate for the fixed 0.98 m offset between the DVL/depth sensor and the
   vehicle's center of rotation.
6. Display the trajectory and vehicle orientation in three dimensions.

The reconstructed trajectory represents the vehicle's center of rotation.
"""
import argparse
from collections import namedtuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


MAX_DVL_SPEED = 3.0
# Distance to the center of rotation, not the center of gravity.
DVL_OFFSET_X_FROM_ROBOT = 0.98
DVL_TO_ROBOT = np.array(
    [
        [1.0, 0.0, 0.0, -DVL_OFFSET_X_FROM_ROBOT],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
AUV_DIAMETER = 0.14
AUV_LENGTH = 1.76


LogColumns = namedtuple("LogColumns", "timestamp depth dvl_velocity angles")


def configure_latex_like_font():
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": [
                "CMU Serif",
                "Computer Modern Unicode",
                "Computer Modern Roman",
                "DejaVu Serif",
            ],
            "mathtext.fontset": "cm",
            "axes.unicode_minus": False,
        }
    )


def load_log_columns(csv_path):
    log = pd.read_csv(csv_path, skipinitialspace=True)
    log.columns = log.columns.str.strip()
    log = log.apply(pd.to_numeric, errors="coerce")

    dvl_velocity = log[["vsurge", "vsway", "vheave"]].to_numpy(float)
    dvl_speed = np.linalg.norm(dvl_velocity, axis=1)
    angles = log[["yaw", "pitch", "roll"]].to_numpy(float)
    timestamp = log["timestamp"].to_numpy(float)
    keep = (
        np.isfinite(timestamp)
        & np.isfinite(dvl_speed)
        & (dvl_speed <= MAX_DVL_SPEED)
        & np.isfinite(angles).all(axis=1)
    )

    return LogColumns(
        timestamp=timestamp[keep],
        depth=log.loc[keep, "depth"].to_numpy(float),
        dvl_velocity=dvl_velocity[keep],
        angles=angles[keep],
    )


def world_velocities(columns):
    body_vel = columns.dvl_velocity.copy()

    return Rotation.from_euler("ZYX", columns.angles, degrees=True).apply(body_vel)


def trajectory(columns):
    if len(columns.timestamp) == 0:
        raise ValueError("No valid DVL samples in log")

    velocity = world_velocities(columns)
    dvl_position = np.zeros((len(columns.timestamp), 3))

    dvl_position[0, 2] = columns.depth[0] if np.isfinite(columns.depth[0]) else 0.0
    for i in range(1, len(columns.timestamp)):
        dt = max(columns.timestamp[i] - columns.timestamp[i - 1], 0.0)
        dvl_position[i] = dvl_position[i - 1] + 0.5 * (velocity[i - 1] + velocity[i]) * dt
        if np.isfinite(columns.depth[i]):
            dvl_position[i, 2] = columns.depth[i]

    frames = Rotation.from_euler("ZYX", columns.angles, degrees=True).as_matrix()
    robot_offset = frames @ DVL_TO_ROBOT[:3, 3]
    return dvl_position + robot_offset


def plot_auv_cylinder(ax, center, frame):
    surge = frame[:, 0]
    sway = frame[:, 1]
    heave = frame[:, 2]
    radius = AUV_DIAMETER / 2.0

    along = np.linspace(-AUV_LENGTH / 2.0, AUV_LENGTH / 2.0, 18)
    theta = np.linspace(0.0, 2.0 * np.pi, 24)
    centerline = center + np.outer(along, surge)
    circle = radius * (
        np.cos(theta)[None, :, None] * sway
        + np.sin(theta)[None, :, None] * heave
    )
    surface = centerline[:, None, :] + circle

    ax.plot_surface(
        surface[:, :, 0],
        surface[:, :, 1],
        surface[:, :, 2],
        color="#4c78a8",
        alpha=0.20,
        linewidth=0,
        shade=True,
    )


def set_equal_axes(ax, position, extra_points):
    points = [position]
    if extra_points is not None:
        points.append(extra_points)

    points = np.vstack(points)
    x_min, y_min, z_min = np.nanmin(points, axis=0)
    x_max, y_max, z_max = np.nanmax(points, axis=0)
    z_min = min(z_min, 0.0)
    z_max = max(z_max, 6.0)

    center = np.array(
        [
            (x_min + x_max) / 2.0,
            (y_min + y_max) / 2.0,
            (z_min + z_max) / 2.0,
        ]
    )
    half_range = max(x_max - x_min, y_max - y_min, z_max - z_min) / 2.0
    if half_range == 0.0:
        half_range = 1.0

    ax.set_xlim(center[0] - half_range, center[0] + half_range)
    ax.set_ylim(center[1] - half_range, center[1] + half_range)
    ax.set_zlim(center[2] + half_range, center[2] - half_range)
    ax.set_box_aspect((1, 1, 1))
    ax.set_proj_type("ortho")


def plot(columns, position, show_cylinder):
    configure_latex_like_font()

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(position[:, 0], position[:, 1], position[:, 2], color="#555555", linewidth=2.0)
    ax.scatter(*position[0], color="#2ca02c", s=60, label="Start")
    ax.scatter(*position[-1], color="#d62728", s=60, label="End")

    sample_count = min(12, len(columns.timestamp))
    sample_ids = np.linspace(0, len(columns.timestamp) - 1, sample_count, dtype=int)
    frames = Rotation.from_euler("ZYX", columns.angles[sample_ids], degrees=True).as_matrix()
    scale = max(np.ptp(position[:, 0]), np.ptp(position[:, 1]), 6.0) * 0.08
    cylinder_ends = None
    if show_cylinder:
        cylinder_ends = np.vstack(
            (
                position[sample_ids] - frames[:, :, 0] * (AUV_LENGTH / 2.0),
                position[sample_ids] + frames[:, :, 0] * (AUV_LENGTH / 2.0),
            )
        )

    for i, frame in zip(sample_ids, frames):
        origin = position[i]
        if show_cylinder:
            plot_auv_cylinder(ax, origin, frame)
        ax.quiver(*origin, *(frame[:, 0] * scale), color="r", linewidth=1.2)
        ax.quiver(*origin, *(frame[:, 1] * scale), color="g", linewidth=1.2)
        ax.quiver(*origin, *(frame[:, 2] * scale), color="b", linewidth=1.2)

    ax.set(
        xlabel="World X [m]",
        ylabel="World Y [m]",
        zlabel="World Z [m]",
        title="Estimated trajectory",
    )
    set_equal_axes(ax, position, cylinder_ends)
    ax.invert_yaxis()
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--cylinder", action="store_true")
    args = parser.parse_args()

    columns = load_log_columns(args.csv_path)
    position = trajectory(columns)
    distance = float(np.sum(np.linalg.norm(np.diff(position, axis=0), axis=1)))
    x, y, z = position[-1]
    print("Final [m]: x=%.3f y=%.3f z=%.3f | dist~ %.3f m | samples=%d" % (x, y, z, distance, len(columns.timestamp)))
    plot(columns, position, args.cylinder)
