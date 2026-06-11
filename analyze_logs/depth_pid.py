#!/usr/bin/env python3
"""Read a CSV log and plot depth, thrust, and pitch over time."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def _parse_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def read_samples(csv_path):
    timestamps = []
    depths = []
    thrusts = []
    pitches = []

    with csv_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file, skipinitialspace=True)
        if reader.fieldnames:
            reader.fieldnames = [name.strip() for name in reader.fieldnames]

        for row in reader:
            timestamp = _parse_float(row.get("timestamp"))
            depth = _parse_float(row.get("depth"))
            thrust = _parse_float(row.get("ut"))
            pitch = _parse_float(row.get("pitch"))
            if all(
                math.isfinite(value)
                for value in (timestamp, depth, thrust, pitch)
            ):
                timestamps.append(timestamp)
                depths.append(depth)
                thrusts.append(thrust)
                pitches.append(pitch)

    return timestamps, depths, thrusts, pitches


def filter_timestamps(
    timestamps, depths, thrusts, pitches, start_timestamp, end_timestamp
):
    samples = [
        (timestamp, depth, thrust, pitch)
        for timestamp, depth, thrust, pitch in zip(
            timestamps, depths, thrusts, pitches
        )
        if (start_timestamp is None or timestamp >= start_timestamp)
        and (end_timestamp is None or timestamp <= end_timestamp)
    ]
    if not samples:
        return [], [], [], []
    return [list(values) for values in zip(*samples)]


def plot_depth(timestamps, depths, thrusts, pitches, output):
    fig, (depth_ax, thrust_ax, pitch_ax) = plt.subplots(
        3, 1, figsize=(11, 9), sharex=True
    )
    depth_ax.axhline(
        3.0,
        color="black",
        linestyle="--",
        linewidth=1.2,
        label="target depth: 3.0 m",
    )
    depth_ax.plot(
        timestamps, depths, color="#2563eb", linewidth=1.2, label="depth"
    )
    depth_ax.set_title("Depth over time")
    depth_ax.set_ylabel("depth (m)")
    depth_ax.set_ylim(6.0, 0.0)
    depth_ax.grid(True, alpha=0.3)
    depth_ax.legend()

    thrust_ax.axhline(
        -0.5,
        color="black",
        linestyle="--",
        linewidth=1.2,
        label="reference thrust: -0.5",
    )
    thrust_ax.plot(
        timestamps, thrusts, color="#059669", linewidth=1.2, label="ut"
    )
    thrust_ax.set_title("Longitudinal thrust over time")
    thrust_ax.set_ylabel("ut")
    thrust_ax.set_ylim(-1.0, 0.0)
    thrust_ax.grid(True, alpha=0.3)
    thrust_ax.legend()

    pitch_ax.axhline(
        0.0,
        color="black",
        linestyle="--",
        linewidth=1.2,
        label="reference pitch: 0 deg",
    )
    pitch_ax.plot(
        timestamps, pitches, color="#f97316", linewidth=1.2, label="pitch"
    )
    pitch_ax.set_title("Pitch over time")
    pitch_ax.set_xlabel("time (s)")
    pitch_ax.set_ylabel("pitch (deg)")
    pitch_ax.set_ylim(-25.0, 25.0)
    pitch_ax.grid(True, alpha=0.3)
    pitch_ax.legend()
    fig.tight_layout()

    if output is not None:
        fig.savefig(output, dpi=160)
    else:
        plt.show()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read a CSV log and plot depth, thrust, and pitch over time."
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


def main():
    args = parse_args()
    timestamps, depths, thrusts, pitches = read_samples(args.csv_path)
    timestamps, depths, thrusts, pitches = filter_timestamps(
        timestamps, depths, thrusts, pitches, args.start, args.end
    )
    if not depths:
        raise SystemExit("No valid samples after filtering.")

    plot_depth(timestamps, depths, thrusts, pitches, args.output)


if __name__ == "__main__":
    main()
