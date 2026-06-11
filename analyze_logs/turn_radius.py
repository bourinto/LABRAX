#!/usr/bin/env python3
"""Read a CSV log and plot heading and vsurge over time."""

import argparse
import csv
import math
import statistics
from pathlib import Path

import matplotlib.pyplot as plt


def _parse_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def read_samples(csv_path):
    timestamps = []
    headings = []
    vsurges = []

    with csv_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file, skipinitialspace=True)
        if reader.fieldnames:
            reader.fieldnames = [name.strip() for name in reader.fieldnames]

        for row in reader:
            timestamp = _parse_float(row.get("timestamp"))
            heading = _parse_float(row.get("heading", row.get("yaw")))
            vsurge = _parse_float(row.get("vsurge"))

            if (
                math.isfinite(timestamp)
                and math.isfinite(heading)
                and math.isfinite(vsurge)
            ):
                timestamps.append(timestamp)
                headings.append(heading)
                vsurges.append(vsurge)

    return timestamps, headings, vsurges


def wrap_angle_delta(delta_deg):
    return (delta_deg + 180.0) % 360.0 - 180.0


def unwrap_headings(headings):
    if not headings:
        return []

    unwrapped = [headings[0]]
    for previous_raw, current_raw in zip(headings, headings[1:]):
        delta = wrap_angle_delta(current_raw - previous_raw)
        unwrapped.append(unwrapped[-1] + delta)

    return unwrapped


def filter_start(timestamps, headings, vsurges, start_timestamp):
    samples = [
        (timestamp, heading, vsurge)
        for timestamp, heading, vsurge in zip(timestamps, headings, vsurges)
        if start_timestamp is None or timestamp >= start_timestamp
    ]
    if not samples:
        return [], [], []
    return [list(values) for values in zip(*samples)]


def linear_fit(timestamps, headings):
    if len(headings) < 2:
        raise SystemExit("At least two samples are required to compute a linear fit.")

    mean_t = sum(timestamps) / len(timestamps)
    mean_h = sum(headings) / len(headings)
    ss_tt = sum((timestamp - mean_t) ** 2 for timestamp in timestamps)
    if ss_tt == 0.0:
        raise SystemExit("Cannot compute a linear fit: all timestamps are equal.")

    slope = sum(
        (timestamp - mean_t) * (heading - mean_h)
        for timestamp, heading in zip(timestamps, headings)
    ) / ss_tt
    intercept = mean_h - slope * mean_t
    ss_res = sum(
        (heading - (slope * timestamp + intercept)) ** 2
        for timestamp, heading in zip(timestamps, headings)
    )
    ss_tot = sum((heading - mean_h) ** 2 for heading in headings)
    r_squared = 1.0 if ss_tot == 0.0 else 1.0 - ss_res / ss_tot

    return slope, intercept, r_squared


def plot_radius_inputs(
    *,
    timestamps,
    headings,
    vsurges,
    slope,
    intercept,
    r_squared,
    mean_vsurge,
    median_vsurge,
    radius_mean,
    radius_median,
    output,
):
    heading_color = "#2563eb"
    fit_color = "#f97316"
    surge_color = "#059669"
    mean_color = "#dc2626"
    median_color = "#7c3aed"

    fig, (heading_ax, surge_ax) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    fig.suptitle(
        "Curvature radius estimate: mean %.3f m, median %.3f m"
        % (radius_mean, radius_median)
    )

    heading_ax.plot(
        timestamps,
        headings,
        marker=".",
        linestyle="-",
        linewidth=1.0,
        color=heading_color,
        label="heading",
    )
    heading_ax.plot(
        timestamps,
        [slope * timestamp + intercept for timestamp in timestamps],
        linestyle="--",
        linewidth=1.4,
        color=fit_color,
        label="linear fit, R^2=%.4f" % r_squared,
    )
    heading_ax.set_title("Heading over time")
    heading_ax.set_ylabel("heading (deg)")
    heading_ax.legend()
    heading_ax.grid(True, alpha=0.3)

    surge_ax.plot(
        timestamps,
        vsurges,
        marker=".",
        linestyle="-",
        linewidth=1.0,
        color=surge_color,
        label="vsurge",
    )
    surge_ax.axhline(
        mean_vsurge,
        linestyle="--",
        linewidth=1.3,
        color=mean_color,
        label="mean: %.6f m/s" % mean_vsurge,
    )
    surge_ax.axhline(
        median_vsurge,
        linestyle=":",
        linewidth=1.5,
        color=median_color,
        label="median: %.6f m/s" % median_vsurge,
    )
    surge_ax.set_title("Surge velocity over time")
    surge_ax.set_xlabel("time (s)")
    surge_ax.set_ylabel("vsurge (m/s)")
    surge_ax.set_ylim(-1.5, 1.5)
    surge_ax.legend()
    surge_ax.grid(True, alpha=0.3)

    fig.tight_layout()

    if output is not None:
        fig.savefig(output, dpi=160)
    else:
        plt.show()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read a CSV log and plot heading, vsurge, and radius estimate."
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
    return parser.parse_args()


def main():
    args = parse_args()
    timestamps, headings, vsurges = read_samples(args.csv_path)
    timestamps, headings, vsurges = filter_start(
        timestamps, headings, vsurges, args.start
    )
    headings = unwrap_headings(headings)
    if not vsurges:
        raise SystemExit("No valid samples after filtering.")

    slope, intercept, r_squared = linear_fit(timestamps, headings)
    yaw_dot = math.radians(slope)
    if yaw_dot == 0.0:
        raise SystemExit("Cannot compute radius: yaw_dot is zero.")

    mean_vsurge = statistics.fmean(vsurges)
    median_vsurge = statistics.median(vsurges)
    radius_mean = mean_vsurge / yaw_dot
    radius_median = median_vsurge / yaw_dot

    print("Linear fit: heading = %.6f * timestamp + %.6f" % (slope, intercept))
    print("R^2: %.6f" % r_squared)
    print("Yaw dot: %.6f deg/s" % slope)
    print("Yaw dot: %.6f rad/s" % yaw_dot)
    print("Mean vsurge: %.6f m/s" % mean_vsurge)
    print("Median vsurge: %.6f m/s" % median_vsurge)
    print("Radius estimate using mean vsurge: %.6f m" % radius_mean)
    print("Radius estimate using median vsurge: %.6f m" % radius_median)
    plot_radius_inputs(
        timestamps=timestamps,
        headings=headings,
        vsurges=vsurges,
        slope=slope,
        intercept=intercept,
        r_squared=r_squared,
        mean_vsurge=mean_vsurge,
        median_vsurge=median_vsurge,
        radius_mean=radius_mean,
        radius_median=radius_median,
        output=args.output,
    )


if __name__ == "__main__":
    main()
