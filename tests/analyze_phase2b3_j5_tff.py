#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Analyze Phase 2B-3 J5 t_ff sweep.

Usage:
    python tests/analyze_phase2b3_j5_tff.py \
        data/mit_teleop/xxx_phase2b3_j5_tff.csv
"""

import sys
import math
import numpy as np
import pandas as pd


RAD_TO_DEG = 180.0 / math.pi


def main():
    if len(sys.argv) != 2:
        print(
            "Usage: {} <phase2b3_j5_tff.csv>"
            .format(sys.argv[0])
        )
        sys.exit(1)

    path = sys.argv[1]
    df = pd.read_csv(path)

    measure = df[
        df["phase"] == "measure"
    ].copy()

    if len(measure) == 0:
        raise RuntimeError(
            "no measure rows found"
        )

    grouped = []

    for stage_index, g in measure.groupby(
        "stage_index"
    ):
        ff = float(
            np.mean(g["t_ff_cmd"])
        )

        err = np.asarray(
            g["j5_error"],
            dtype=float,
        )

        tau = np.asarray(
            g["j5_motor_torque"],
            dtype=float,
        )

        current = np.asarray(
            g["j5_motor_current"],
            dtype=float,
        )

        grouped.append({
            "stage_index": int(stage_index),
            "t_ff": ff,
            "err_mean_rad": float(np.mean(err)),
            "err_std_rad": float(np.std(err)),
            "tau_mean": float(np.nanmean(tau)),
            "current_mean": float(np.nanmean(current)),
            "n": len(g),
        })

    result = pd.DataFrame(grouped)

    print("=" * 78)
    print("Phase 2B-3 J5 t_ff sweep")
    print("=" * 78)

    print(
        "{:>5} {:>9} {:>12} {:>12} {:>12} {:>12}"
        .format(
            "step",
            "t_ff",
            "err_mean°",
            "err_std°",
            "motor_tau",
            "current",
        )
    )

    for _, row in result.iterrows():
        print(
            "{:>5d} {:>+9.3f} {:>+12.3f} "
            "{:>12.3f} {:>+12.3f} {:>+12.3f}"
            .format(
                int(row["stage_index"]),
                row["t_ff"],
                row["err_mean_rad"]
                * RAD_TO_DEG,
                row["err_std_rad"]
                * RAD_TO_DEG,
                row["tau_mean"],
                row["current_mean"],
            )
        )

    # Use all stages, including repeated zero points.
    x = np.asarray(
        result["t_ff"],
        dtype=float,
    )

    y = np.asarray(
        result["err_mean_rad"],
        dtype=float,
    )

    if len(np.unique(x)) >= 3:
        a, b = np.polyfit(
            x,
            y,
            1,
        )

        y_hat = a * x + b

        ss_res = float(
            np.sum(
                (y - y_hat) ** 2
            )
        )

        ss_tot = float(
            np.sum(
                (y - np.mean(y)) ** 2
            )
        )

        r2 = (
            1.0 - ss_res / ss_tot
            if ss_tot > 0.0
            else float("nan")
        )

        print()
        print("Linear fit:")
        print(
            "  error_rad = {:.6f} * t_ff + {:.6f}"
            .format(
                a,
                b,
            )
        )
        print(
            "  error_deg = {:.3f} * t_ff + {:.3f}"
            .format(
                a * RAD_TO_DEG,
                b * RAD_TO_DEG,
            )
        )
        print(
            "  R^2 = {:.4f}"
            .format(r2)
        )

        if abs(a) > 1e-9:
            ff_zero = -b / a

            print(
                "  estimated t_ff for zero error "
                "= {:+.3f} N*m"
                .format(ff_zero)
            )

            if abs(ff_zero) > 0.5:
                print(
                    "  NOTE: zero-crossing is outside this "
                    "experiment's +/-0.5 N*m software guard."
                )

    # Compare repeated zero-feedforward measurements to expose drift/hysteresis.
    zero = result[
        np.isclose(
            result["t_ff"],
            0.0,
            atol=1e-6,
        )
    ]

    if len(zero) >= 2:
        zero_err_deg = (
            np.asarray(
                zero["err_mean_rad"],
                dtype=float,
            )
            * RAD_TO_DEG
        )

        print()
        print(
            "Repeated t_ff=0 error [deg]:",
            [
                round(x, 3)
                for x in zero_err_deg
            ],
        )

        print(
            "zero-point spread [deg]: {:.3f}"
            .format(
                float(
                    np.max(zero_err_deg)
                    - np.min(zero_err_deg)
                )
            )
        )

    print()
    print("How to read this:")
    print(
        "- monotonic error vs t_ff with high R^2: "
        "feed-forward torque is acting predictably"
    )
    print(
        "- zero crossing inside the tested range: "
        "static error can be compensated at this pose"
    )
    print(
        "- repeated t_ff=0 points differ strongly: "
        "hysteresis/friction or pose drift is significant"
    )
    print(
        "- do not use the fitted zero-crossing globally; "
        "repeat at multiple poses before building tau_ff(q)"
    )


if __name__ == "__main__":
    main()
