#!/usr/bin/env python3
"""Read a CSV log and estimate the torpedo rotation lever arm."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import List, Tuple


def _parse_float(value: str | None) -> float:
    if value is None:
        return float("nan")
    text = str(value).strip()
    if not text:
        return float("nan")
    try:
        return float(text)
    except ValueError:
        return float("nan")


def read_samples(csv_path: Path) -> Tuple[List[float], List[float], List[float], int]:
    timestamps: List[float] = []
    yaws: List[float] = []
    vsways: List[float] = []
    total_rows = 0

    with csv_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file, skipinitialspace=True)
        if reader.fieldnames:
            reader.fieldnames = [name.strip() for name in reader.fieldnames]

        for row in reader:
            total_rows += 1
            timestamp = _parse_float(row.get("timestamp"))
            yaw = _parse_float(row.get("heading", row.get("yaw")))
            vsway = _parse_float(row.get("vsway"))

            if (
                math.isfinite(timestamp)
                and math.isfinite(yaw)
                and math.isfinite(vsway)
            ):
                timestamps.append(timestamp)
                yaws.append(yaw)
                vsways.append(vsway)

    return timestamps, yaws, vsways, total_rows


def wrap_angle_delta(delta_deg: float) -> float:
    return (delta_deg + 180.0) % 360.0 - 180.0


def unwrap_angles(angles: List[float]) -> List[float]:
    if not angles:
        return []

    unwrapped = [angles[0]]
    for previous_raw, current_raw in zip(angles, angles[1:]):
        delta = wrap_angle_delta(current_raw - previous_raw)
        unwrapped.append(unwrapped[-1] + delta)

    return unwrapped


def filter_timestamps(
    timestamps: List[float],
    yaws: List[float],
    vsways: List[float],
    start_timestamp: float | None,
    end_timestamp: float | None,
) -> Tuple[List[float], List[float], List[float]]:
    if start_timestamp is None and end_timestamp is None:
        return timestamps, yaws, vsways

    filtered_timestamps: List[float] = []
    filtered_yaws: List[float] = []
    filtered_vsways: List[float] = []
    for timestamp, yaw, vsway in zip(timestamps, yaws, vsways):
        if (
            (start_timestamp is None or timestamp >= start_timestamp)
            and (end_timestamp is None or timestamp <= end_timestamp)
        ):
            filtered_timestamps.append(timestamp)
            filtered_yaws.append(yaw)
            filtered_vsways.append(vsway)

    return filtered_timestamps, filtered_yaws, filtered_vsways


def linear_fit(timestamps: List[float], angles: List[float]) -> Tuple[float, float, float]:
    if len(angles) < 2:
        raise SystemExit("At least two samples are required to compute a linear fit.")

    mean_t = sum(timestamps) / len(timestamps)
    mean_angle = sum(angles) / len(angles)
    ss_tt = sum((timestamp - mean_t) ** 2 for timestamp in timestamps)
    if ss_tt == 0.0:
        raise SystemExit("Cannot compute a linear fit: all timestamps are equal.")

    slope = sum(
        (timestamp - mean_t) * (angle - mean_angle)
        for timestamp, angle in zip(timestamps, angles)
    ) / ss_tt
    intercept = mean_angle - slope * mean_t
    ss_res = sum(
        (angle - (slope * timestamp + intercept)) ** 2
        for timestamp, angle in zip(timestamps, angles)
    )
    ss_tot = sum((angle - mean_angle) ** 2 for angle in angles)
    r_squared = 1.0 if ss_tot == 0.0 else 1.0 - ss_res / ss_tot

    return slope, intercept, r_squared


def plot_lever_arm_inputs(
    *,
    timestamps: List[float],
    yaws: List[float],
    vsways: List[float],
    slope: float,
    intercept: float,
    r_squared: float,
    mean_vsway: float,
    median_vsway: float,
    lever_arm_mean: float,
    lever_arm_median: float,
    output: Path | None,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "matplotlib is required for plotting. Install with: pip install matplotlib"
        ) from exc

    if not yaws:
        raise SystemExit("No valid samples to plot.")

    yaw_color = "#2563eb"
    fit_color = "#f97316"
    sway_color = "#059669"
    mean_color = "#dc2626"
    median_color = "#7c3aed"

    fig, (yaw_ax, sway_ax) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    fig.suptitle(
        "Rotation lever arm estimate: mean %.3f m, median %.3f m"
        % (lever_arm_mean, lever_arm_median)
    )

    yaw_ax.plot(
        timestamps,
        yaws,
        marker=".",
        linestyle="-",
        linewidth=1.0,
        color=yaw_color,
        label="yaw",
    )
    yaw_ax.plot(
        timestamps,
        [slope * timestamp + intercept for timestamp in timestamps],
        linestyle="--",
        linewidth=1.4,
        color=fit_color,
        label="linear fit, R^2=%.4f" % r_squared,
    )
    yaw_ax.set_title("Yaw over time")
    yaw_ax.set_ylabel("yaw (deg)")
    yaw_ax.legend()
    yaw_ax.grid(True, alpha=0.3)

    sway_ax.plot(
        timestamps,
        vsways,
        marker=".",
        linestyle="-",
        linewidth=1.0,
        color=sway_color,
        label="vsway",
    )
    sway_ax.axhline(
        mean_vsway,
        linestyle="--",
        linewidth=1.3,
        color=mean_color,
        label="mean: %.6f m/s" % mean_vsway,
    )
    sway_ax.axhline(
        median_vsway,
        linestyle=":",
        linewidth=1.5,
        color=median_color,
        label="median: %.6f m/s" % median_vsway,
    )
    sway_ax.set_title("Sway velocity over time")
    sway_ax.set_xlabel("time (s)")
    sway_ax.set_ylabel("vsway (m/s)")
    sway_ax.set_ylim(-1.5, 1.5)
    sway_ax.legend()
    sway_ax.grid(True, alpha=0.3)

    fig.tight_layout()

    if output is not None:
        fig.savefig(output, dpi=160)
    else:
        plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read a CSV log and estimate the torpedo rotation lever arm."
    )
    parser.add_argument("csv_path", type=Path, help="Path to the CSV log.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. If omitted, opens an interactive plot window.",
    )
    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help="Ignore samples with timestamp lower than this value.",
    )
    parser.add_argument(
        "--end",
        type=float,
        default=None,
        help="Ignore samples with timestamp greater than this value.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    timestamps, yaws, vsways, total_rows = read_samples(args.csv_path)
    valid_rows = len(yaws)
    timestamps, yaws, vsways = filter_timestamps(
        timestamps, yaws, vsways, args.start, args.end
    )
    yaws = unwrap_angles(yaws)
    if not vsways:
        raise SystemExit("No valid samples after filtering.")

    slope, intercept, r_squared = linear_fit(timestamps, yaws)
    theta_dot = math.radians(slope)
    if theta_dot == 0.0:
        raise SystemExit("Cannot compute lever arm: theta_dot is zero.")

    mean_vsway = float(statistics.fmean(vsways))
    median_vsway = float(statistics.median(vsways))
    lever_arm_mean = mean_vsway / theta_dot
    lever_arm_median = median_vsway / theta_dot

    print("Rows total: %d" % total_rows)
    print("Rows valid: %d" % valid_rows)
    print("Rows plotted: %d" % len(yaws))
    print("Linear fit: yaw = %.6f * timestamp + %.6f" % (slope, intercept))
    print("R^2: %.6f" % r_squared)
    print("Theta dot: %.6f deg/s" % slope)
    print("Theta dot: %.6f rad/s" % theta_dot)
    print("Mean vsway: %.6f m/s" % mean_vsway)
    print("Median vsway: %.6f m/s" % median_vsway)
    print("Lever arm estimate using mean vsway: %.6f m" % lever_arm_mean)
    print("Lever arm estimate using median vsway: %.6f m" % lever_arm_median)
    plot_lever_arm_inputs(
        timestamps=timestamps,
        yaws=yaws,
        vsways=vsways,
        slope=slope,
        intercept=intercept,
        r_squared=r_squared,
        mean_vsway=mean_vsway,
        median_vsway=median_vsway,
        lever_arm_mean=lever_arm_mean,
        lever_arm_median=lever_arm_median,
        output=args.output,
    )


if __name__ == "__main__":
    main()
