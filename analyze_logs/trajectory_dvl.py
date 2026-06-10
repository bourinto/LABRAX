"""
Reconstruit et visualise la trajectoire d'un AUV à partir d'un journal CSV
contenant les mesures DVL, la profondeur et l'attitude.

Hypothèses :
- Le repère monde est NED :
    X = Nord
    Y = Est
    Z = profondeur (positive vers le bas)
- Les vitesses DVL sont exprimées dans le repère du robot :
    X = avant (surge)
    Y = droite (sway)
    Z = bas (heave)
- L'attitude est fournie sous la forme :
    yaw   = cap (heading)
    pitch = assiette (positive nez vers le haut)
    roll  = roulis
- Les rotations sont appliquées selon la convention ZYX
  (yaw → pitch → roll).
- Les vitesses verticales DVL sont ignorées par défaut
  (VHEAVE_MODEL_WEIGHT = 0), la profondeur étant considérée
  comme la source de référence pour l'axe vertical.
- Les échantillons dont la norme de vitesse DVL dépasse
  MAX_DVL_SPEED sont rejetés.

Méthode :
1. Chargement et filtrage du journal.
2. Transformation des vitesses du repère corps vers le repère monde.
3. Intégration trapézoïdale des vitesses pour estimer la position.
4. Correction de la coordonnée Z à partir de la profondeur mesurée.
5. Compensation du déport fixe entre le DVL/profondimètre et le centre robot.
6. Affichage 3D de la trajectoire et de l'orientation du véhicule.

La trajectoire reconstruite correspond à la trajectoire du centre robot.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


DEPTH_GAIN = 0
VHEAVE_MODEL_WEIGHT = 0.8
MAX_DVL_SPEED = 3.0
DVL_OFFSET_X_FROM_ROBOT = 1.08
DVL_TO_ROBOT = np.array(
    [
        [1.0, 0.0, 0.0, -DVL_OFFSET_X_FROM_ROBOT],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
AUV_DIAMETER = 0.14
AUV_LENGTH = 1.70


@dataclass
class LogColumns:
    timestamp: np.ndarray
    depth: np.ndarray
    dvl_velocity: np.ndarray
    angles: np.ndarray


def load_log_columns(csv_path: str) -> LogColumns:
    log = pd.read_csv(csv_path, skipinitialspace=True)
    log.columns = log.columns.str.strip()
    log = log.apply(pd.to_numeric, errors="coerce")

    dvl_velocity = log[["vsurge", "vsway", "vheave"]].to_numpy(float)
    dvl_speed = np.linalg.norm(dvl_velocity, axis=1)
    keep = ~np.isfinite(dvl_speed) | (dvl_speed <= MAX_DVL_SPEED)

    return LogColumns(
        timestamp=log.loc[keep, "timestamp"].to_numpy(float),
        depth=log.loc[keep, "depth"].to_numpy(float),
        dvl_velocity=dvl_velocity[keep],
        angles=log.loc[keep, ["yaw", "pitch", "roll"]].to_numpy(float),
    )


def world_velocities(columns: LogColumns) -> np.ndarray:
    body_vel = columns.dvl_velocity.copy()
    body_vel[:, 2] *= VHEAVE_MODEL_WEIGHT
    if VHEAVE_MODEL_WEIGHT == 0.0:
        body_vel[:, 2] = 0.0

    valid = np.isfinite(body_vel).all(axis=1)
    body_vel[~valid] = 0.0

    return Rotation.from_euler("ZYX", columns.angles, degrees=True).apply(body_vel)


def trajectory(columns: LogColumns) -> np.ndarray:
    velocity = world_velocities(columns)
    dvl_position = np.zeros((len(columns.timestamp), 3))

    dvl_position[0, 2] = columns.depth[0] if np.isfinite(columns.depth[0]) else 0.0
    for i in range(1, len(columns.timestamp)):
        dt = max(columns.timestamp[i] - columns.timestamp[i - 1], 0.0)
        dvl_position[i] = dvl_position[i - 1] + 0.5 * (velocity[i - 1] + velocity[i]) * dt
        if np.isfinite(columns.depth[i]):
            dvl_position[i, 2] += DEPTH_GAIN * (columns.depth[i] - dvl_position[i, 2])

    frames = Rotation.from_euler("ZYX", columns.angles, degrees=True).as_matrix()
    robot_offset = frames @ DVL_TO_ROBOT[:3, 3]
    return dvl_position + robot_offset


def plot_auv_cylinder(ax, center: np.ndarray, frame: np.ndarray) -> None:
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


def set_equal_axes(ax, position: np.ndarray, extra_points: np.ndarray | None) -> None:
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


def plot(columns: LogColumns, position: np.ndarray, show_cylinder: bool) -> None:
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(position[:, 0], position[:, 1], position[:, 2], color="#555555", linewidth=2.0)
    ax.scatter(*position[0], color="#2ca02c", s=60, label="Depart")
    ax.scatter(*position[-1], color="#d62728", s=60, label="Arrivee")

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

    ax.set(xlabel="X monde [m]", ylabel="Y monde [m]", zlabel="Z monde [m]", title="Trajectoire estimée")
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
