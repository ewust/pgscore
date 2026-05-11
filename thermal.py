#!/usr/bin/env python3
"""Plot a vertical-speed PDF produced by pgscore --thermalCDF."""

import sys
import csv
import matplotlib.pyplot as plt


def load_cdf(path):
    speeds, times = [], []
    with open(path) as f:
        reader = csv.DictReader(f, delimiter="\t")
        for row in reader:
            speeds.append(float(row["vertical_speed_ms"]))
            times.append(float(row["time_seconds"]))
    return speeds, times


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <thermal.cdf> [label ...]", file=sys.stderr)
        sys.exit(1)

    normalize = len(sys.argv) > 2

    fig, ax = plt.subplots()

    for path in sys.argv[1:]:
        label = path
        speeds, times = load_cdf(path)
        if normalize:
            total = sum(times)
            times = [t / total * 100 for t in times]
        ax.plot(speeds, times, label=label)

    ax.set_xlabel("Vertical speed (m/s)")
    ax.set_ylabel("% of flight" if normalize else "Time (seconds)")
    ax.set_title("Vertical speed distribution")
    ax.axvline(0, color="black", linewidth=0.8, linestyle="--")
    if len(sys.argv) > 2:
        ax.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
