#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Plot intended vs actual path from a path_logger CSV.

Loads:
  - The CSV produced by path_logger (--csv)
  - The waypoints YAML used during the run (--waypoints)

Produces a PNG with:
  - Waypoint goals as numbered red squares
  - Connecting straight-line segments (the intended path)
  - Actual robot path coloured by navigation state

Usage:
    python3 scripts/plot_path.py \\
        --csv /tmp/h1_path_*.csv \\
        --waypoints config/waypoints.yaml \\
        --out docs/waypoint_validation.png
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import yaml


STATE_COLOURS = {
    "ROTATING": "#FFB000",   # amber
    "WALKING":  "#1F77B4",   # blue
    "REACHED":  "#2CA02C",   # green
    "COMPLETE": "#2CA02C",
    "IDLE":     "#888888",
    "UNKNOWN":  "#888888",
}


def load_waypoints(path: Path) -> list:
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return [(float(p["x"]), float(p["y"])) for p in data["waypoints"]]


def load_csv(path: Path) -> tuple:
    times, xs, ys, states = [], [], [], []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["wall_time"]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            states.append(row["status"])
    return times, xs, ys, states


def plot(waypoints, xs, ys, states, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)

    # Intended path: connecting segments through waypoints, starting at origin
    intended_x = [0.0] + [p[0] for p in waypoints]
    intended_y = [0.0] + [p[1] for p in waypoints]
    ax.plot(intended_x, intended_y,
            linestyle="--", color="#888888", linewidth=1.5,
            label="Intended path", zorder=1)

    # Waypoint markers, numbered
    for i, (x, y) in enumerate(waypoints):
        ax.plot(x, y, marker="s", markersize=12,
                markerfacecolor="#D62728", markeredgecolor="black",
                linestyle="None", zorder=3)
        ax.annotate(f" {i+1}", (x, y), fontsize=11, fontweight="bold",
                    color="#D62728")

    # Actual path, segmented by state for colour
    # Plot segments where state is constant
    if xs:
        seg_start = 0
        for i in range(1, len(states)):
            if states[i] != states[seg_start]:
                colour = STATE_COLOURS.get(states[seg_start], "#888888")
                ax.plot(xs[seg_start:i+1], ys[seg_start:i+1],
                        color=colour, linewidth=2.0, zorder=2)
                seg_start = i
        # Tail segment
        colour = STATE_COLOURS.get(states[seg_start], "#888888")
        ax.plot(xs[seg_start:], ys[seg_start:],
                color=colour, linewidth=2.0, zorder=2)

        # Start and end markers
        ax.plot(xs[0], ys[0], marker="o", markersize=10,
                markerfacecolor="#2CA02C", markeredgecolor="black",
                linestyle="None", label="Start", zorder=4)
        ax.plot(xs[-1], ys[-1], marker="X", markersize=12,
                markerfacecolor="#9467BD", markeredgecolor="black",
                linestyle="None", label="End", zorder=4)

    # State legend (manually, since segments share colour by state)
    from matplotlib.lines import Line2D
    state_legend = [
        Line2D([0], [0], color=STATE_COLOURS["ROTATING"], lw=2, label="Rotating"),
        Line2D([0], [0], color=STATE_COLOURS["WALKING"],  lw=2, label="Walking"),
        Line2D([0], [0], color=STATE_COLOURS["REACHED"],  lw=2, label="Reached/Complete"),
        Line2D([0], [0], linestyle="--", color="#888888", lw=1.5, label="Intended"),
    ]
    ax.legend(handles=state_legend, loc="lower right", framealpha=0.9)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("H1 waypoint navigation — intended vs actual path")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"Wrote {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv", type=Path, required=True,
                        help="Path to path_logger CSV")
    parser.add_argument("--waypoints", type=Path, required=True,
                        help="Path to waypoints YAML")
    parser.add_argument("--out", type=Path, default=Path("docs/waypoint_validation.png"),
                        help="Output PNG path")
    args = parser.parse_args()

    if not args.csv.exists():
        print(f"CSV not found: {args.csv}", file=sys.stderr)
        return 1
    if not args.waypoints.exists():
        print(f"Waypoints not found: {args.waypoints}", file=sys.stderr)
        return 1

    waypoints = load_waypoints(args.waypoints)
    _, xs, ys, states = load_csv(args.csv)

    if not xs:
        print("CSV had no data rows", file=sys.stderr)
        return 1

    plot(waypoints, xs, ys, states, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())