#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Analyze Phase 2B-2 static tracking characterization CSV.

Usage:
    python tests/analyze_phase2b2_static.py data/mit_teleop/xxx_phase2b2_static.csv
"""

import sys
import math
import numpy as np
import pandas as pd


RAD_TO_DEG = 180.0 / math.pi


def finite(x):
    return np.asarray(x, dtype=float)[
        np.isfinite(np.asarray(x, dtype=float))
    ]


def main():
    if len(sys.argv) != 2:
        print(
            "Usage: {} <phase2b2_static.csv>"
            .format(sys.argv[0])
        )
        sys.exit(1)

    path = sys.argv[1]
    df = pd.read_csv(path)

    if len(df) < 10:
        raise RuntimeError("not enough samples")

    duration = float(df["time"].iloc[-1] - df["time"].iloc[0])
    hz = (len(df) - 1) / duration if duration > 0 else float("nan")

    loop_ms = finite(df["loop_dt"]) * 1000.0
    work_ms = finite(df["loop_work_ms"])
    motor_ms = finite(df["motor_read_ms"])

    print("=" * 78)
    print("Phase 2B-2 static tracking analysis")
    print("=" * 78)
    print("samples      :", len(df))
    print("duration [s] :", round(duration, 3))
    print("effective Hz :", round(hz, 2))

    print()
    print("Timing")
    print(
        "loop ms      : median={:.3f}, p95={:.3f}, max={:.3f}"
        .format(
            np.median(loop_ms),
            np.percentile(loop_ms, 95),
            np.max(loop_ms),
        )
    )
    print(
        "work ms      : median={:.3f}, p95={:.3f}, max={:.3f}"
        .format(
            np.median(work_ms),
            np.percentile(work_ms, 95),
            np.max(work_ms),
        )
    )

    motor_nonzero = motor_ms[motor_ms > 0.0]
    if motor_nonzero.size:
        print(
            "motor read ms: median={:.3f}, p95={:.3f}, max={:.3f}"
            .format(
                np.median(motor_nonzero),
                np.percentile(motor_nonzero, 95),
                np.max(motor_nonzero),
            )
        )

    # Use only settled static rows.
    static = df[
        (df["static_hold"] == 1)
        & (df["static_hold_time"] >= 0.5)
    ].copy()

    print()
    print("settled static samples:", len(static))

    if len(static) == 0:
        print("No settled static segments found.")
        return

    # Split static samples into episodes using resets in static_hold_time.
    episode = 0
    episodes = []
    last_hold_time = None
    current_rows = []

    for idx, row in static.iterrows():
        hold_time = float(row["static_hold_time"])

        if (
            last_hold_time is not None
            and hold_time < last_hold_time
        ):
            if current_rows:
                episodes.append(
                    pd.DataFrame(current_rows)
                )
            current_rows = []
            episode += 1

        current_rows.append(row)
        last_hold_time = hold_time

    if current_rows:
        episodes.append(pd.DataFrame(current_rows))

    print("static episodes:", len(episodes))
    print()

    for ep_i, ep in enumerate(episodes, start=1):
        if len(ep) < 5:
            continue

        print("Episode {}  t={:.2f}..{:.2f}s  n={}".format(
            ep_i,
            float(ep["time"].iloc[0]),
            float(ep["time"].iloc[-1]),
            len(ep),
        ))

        print(
            "{:>5} {:>11} {:>11} {:>11} {:>11}"
            .format(
                "joint",
                "target°",
                "err_mean°",
                "err_std°",
                "tau_mean",
            )
        )

        for j in range(1, 7):
            target = finite(ep["target_{}".format(j)]) * RAD_TO_DEG
            err = finite(ep["tracking_error_{}".format(j)]) * RAD_TO_DEG
            tau = finite(ep["motor_torque_{}".format(j)])

            if not len(err):
                continue

            print(
                "J{:<4} {:>11.2f} {:>11.3f} {:>11.3f} {:>11.3f}"
                .format(
                    j,
                    float(np.mean(target)) if len(target) else float("nan"),
                    float(np.mean(err)),
                    float(np.std(err)),
                    float(np.mean(tau)) if len(tau) else float("nan"),
                )
            )

        print()

    print("Interpretation:")
    print(
        "- same joint, different poses -> different static error: "
        "gravity/friction/load-dependent equilibrium is likely"
    )
    print(
        "- nearly constant error across poses: "
        "check joint-zero/geometric calibration first"
    )
    print(
        "- error sign/magnitude follows motor torque: "
        "good candidate for gravity/load feed-forward"
    )
    print(
        "- large error std while holding: "
        "look for oscillation, noisy feedback, or insufficient damping"
    )


if __name__ == "__main__":
    main()
