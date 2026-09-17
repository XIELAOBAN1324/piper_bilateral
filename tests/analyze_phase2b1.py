#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Summarize a Phase 2B-1 CSV log.

Usage:
    python tests/analyze_phase2b1.py data/mit_teleop/xxx_phase2b1.csv
"""

import sys
import numpy as np
import pandas as pd


def stats_ms(series):
    x = np.asarray(series.dropna(), dtype=float)

    if x.size == 0:
        return "n/a"

    return (
        "median={:.3f}, p95={:.3f}, max={:.3f}"
        .format(
            np.median(x),
            np.percentile(x, 95),
            np.max(x),
        )
    )


def stats_sec_to_ms(series):
    return stats_ms(series * 1000.0)


def main():
    if len(sys.argv) != 2:
        print(
            "Usage: {} <phase2b1.csv>"
            .format(sys.argv[0])
        )
        sys.exit(1)

    path = sys.argv[1]
    df = pd.read_csv(path)

    if len(df) < 2:
        raise RuntimeError("not enough samples")

    duration = (
        float(df["time"].iloc[-1])
        - float(df["time"].iloc[0])
    )

    effective_hz = (
        (len(df) - 1) / duration
        if duration > 0.0
        else float("nan")
    )

    missed = (
        np.asarray(
            df["deadline_late_ms"],
            dtype=float,
        )
        > 0.0
    )

    print("=" * 72)
    print("Phase 2B-1 log summary")
    print("=" * 72)

    print("samples       :", len(df))
    print("duration [s]  :", round(duration, 3))
    print(
        "effective Hz  :",
        round(effective_hz, 2),
    )

    print()
    print("Timing")
    print(
        "loop dt [ms]  :",
        stats_sec_to_ms(df["loop_dt"]),
    )
    print(
        "leader age[ms]:",
        stats_sec_to_ms(df["leader_age"]),
    )
    print(
        "MIT send [ms] :",
        stats_ms(df["mit_send_ms"]),
    )
    print(
        "feedback [ms] :",
        stats_ms(df["feedback_read_ms"]),
    )
    print(
        "work [ms]     :",
        stats_ms(df["loop_work_ms"]),
    )
    print(
        "deadline miss : {:.2f}%"
        .format(
            100.0 * np.mean(missed)
        )
    )

    print()
    print("Per-joint errors [deg]")
    print(
        "{:>5} {:>12} {:>12} {:>12} {:>12}"
        .format(
            "joint",
            "track_bias",
            "track_p95",
            "motion_p95",
            "abs_gap_med",
        )
    )

    for j in range(1, 7):
        tracking = (
            np.asarray(
                df[
                    "tracking_error_{}"
                    .format(j)
                ],
                dtype=float,
            )
            * 180.0
            / np.pi
        )

        motion = (
            np.asarray(
                df[
                    "motion_error_{}"
                    .format(j)
                ],
                dtype=float,
            )
            * 180.0
            / np.pi
        )

        absolute_gap = (
            np.asarray(
                df[
                    "absolute_gap_{}"
                    .format(j)
                ],
                dtype=float,
            )
            * 180.0
            / np.pi
        )

        print(
            "J{:<4} {:>12.3f} {:>12.3f} "
            "{:>12.3f} {:>12.3f}"
            .format(
                j,
                np.mean(tracking),
                np.percentile(
                    np.abs(tracking),
                    95,
                ),
                np.percentile(
                    np.abs(motion),
                    95,
                ),
                np.median(absolute_gap),
            )
        )

    print()
    print("Interpretation:")
    print(
        "- large absolute_gap but small tracking_error: "
        "startup relative-map offset, not MIT tracking failure"
    )
    print(
        "- persistent tracking_error that varies with pose: "
        "gravity/friction/impedance equilibrium is likely"
    )
    print(
        "- large motion_error mainly during fast motion: "
        "dynamic lag / timing / bandwidth is likely"
    )
    print(
        "- work p95 > 5 ms or many deadline misses: "
        "the 200 Hz Python control path is not being sustained"
    )


if __name__ == "__main__":
    main()
