#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Piper Phase 2B-3 rev2: adaptive J5 static t_ff characterization

Why rev2
--------
The first sweep tried positive and negative feed-forward blindly. On the tested
pose, J5 had:
    error < 0
    motor torque < 0
and +t_ff made the error more negative.

This revision:
1. Measures a zero-t_ff baseline first.
2. Chooses the correction direction from the baseline position-error sign.
3. Sweeps only in that direction in 0.1 N*m increments.
4. Stops early when:
   - |error| <= 1 degree, or
   - the error crosses zero, or
   - |error| becomes > 2 degrees worse than baseline.
5. Ramps every t_ff change and returns smoothly to zero at the end.

RIGHT arm only. No Leader / teleoperation.

Run:
    python tests/step2b3_j5_tff_adaptive.py
    python tests/step2b3_j5_tff_adaptive.py --live

The existing analyze_phase2b3_j5_tff.py can analyze the generated CSV.
"""

import argparse
import csv
import math
import os
import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


RIGHT_CAN = "can_piper_right"

CONTROL_RATE = 200.0
PERIOD = 1.0 / CONTROL_RATE

KP = [5.0, 5.0, 5.0, 4.0, 4.0, 3.0]
KD = [1.0, 1.0, 1.0, 0.6, 0.6, 0.5]

TEST_JOINT = 5
J = TEST_JOINT - 1

RAMP_SEC = 0.50
SETTLE_SEC = 0.80
MEASURE_SEC = 1.00

FF_STEP = 0.10
MAX_FF_MAG = 1.00

ZERO_ERROR_STOP_DEG = 1.0
WORSENING_ABORT_DEG = 2.0

MAX_ANY_JOINT_ERROR = 0.28   # rad ~= 16.0 deg
MAX_J5_ERROR = 0.25          # rad ~= 14.3 deg

RAD_TO_DEG = 180.0 / math.pi


def create_piper():
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=RIGHT_CAN,
    )
    return AgxArmFactory.create_arm(cfg)


def send_mit(arm, q_des, j5_tff):
    for i in range(6):
        arm.move_mit(
            i + 1,
            p_des=q_des[i],
            v_des=0.0,
            kp=KP[i],
            kd=KD[i],
            t_ff=(j5_tff if i == J else 0.0),
        )


def read_q(arm):
    fb = arm.get_joint_angles()
    if fb is None:
        raise RuntimeError("RIGHT joint feedback unavailable")
    return list(fb.msg)


def read_j5_motor(arm):
    state = arm.get_motor_states(TEST_JOINT)
    if state is None:
        return {
            "position": float("nan"),
            "velocity": float("nan"),
            "current": float("nan"),
            "torque": float("nan"),
        }

    msg = state.msg
    return {
        "position": msg.position,
        "velocity": msg.velocity,
        "current": msg.current,
        "torque": msg.torque,
    }


def errors(q_des, q):
    return [
        q_des[i] - q[i]
        for i in range(6)
    ]


def guard(q_des, q):
    e = errors(q_des, q)

    if max(abs(x) for x in e) > MAX_ANY_JOINT_ERROR:
        raise RuntimeError(
            "joint deviation guard: max error {:.3f} rad > {:.3f} rad"
            .format(
                max(abs(x) for x in e),
                MAX_ANY_JOINT_ERROR,
            )
        )

    if abs(e[J]) > MAX_J5_ERROR:
        raise RuntimeError(
            "J5 absolute deviation guard: error {:.3f} rad > {:.3f} rad"
            .format(
                e[J],
                MAX_J5_ERROR,
            )
        )

    return e


def ramp_ff(arm, q_des, ff0, ff1, seconds):
    start = time.monotonic()
    end = start + seconds
    next_tick = start

    while True:
        now = time.monotonic()
        if now >= end:
            break

        if now < next_tick:
            time.sleep(next_tick - now)

        loop_now = time.monotonic()
        alpha = min(
            1.0,
            (loop_now - start) / seconds,
        )
        ff = ff0 + alpha * (ff1 - ff0)

        send_mit(
            arm,
            q_des,
            ff,
        )

        q = read_q(arm)
        guard(q_des, q)

        next_tick += PERIOD

    send_mit(
        arm,
        q_des,
        ff1,
    )


def hold_zero_and_disable(arm):
    if arm is None:
        return

    try:
        q_hold = read_q(arm)
        next_tick = time.monotonic()

        for _ in range(40):
            send_mit(
                arm,
                q_hold,
                0.0,
            )
            next_tick += PERIOD
            dt = next_tick - time.monotonic()
            if dt > 0.0:
                time.sleep(dt)

        arm.set_motion_mode("p")
        arm.disable()

    except Exception as exc:
        print("[WARN] shutdown:", exc)


def run_phase(
    arm,
    q_des,
    writer,
    experiment_start,
    stage_index,
    phase_name,
    ff_cmd,
    duration,
):
    if duration <= 0.0:
        return []

    samples = []

    phase_start = time.monotonic()
    phase_end = phase_start + duration
    next_tick = phase_start
    previous_loop = phase_start

    while True:
        now = time.monotonic()

        if now >= phase_end:
            break

        if now < next_tick:
            time.sleep(next_tick - now)

        loop_start = time.monotonic()
        loop_dt = loop_start - previous_loop
        previous_loop = loop_start
        next_tick += PERIOD

        send_mit(
            arm,
            q_des,
            ff_cmd,
        )

        q = read_q(arm)
        e = guard(q_des, q)
        motor = read_j5_motor(arm)

        row = {
            "time":
                loop_start - experiment_start,
            "stage_index":
                stage_index,
            "phase":
                phase_name,
            "phase_time":
                loop_start - phase_start,
            "t_ff_cmd":
                ff_cmd,
            "j5_target":
                q_des[J],
            "j5_actual":
                q[J],
            "j5_error":
                e[J],
            "j5_motor_position":
                motor["position"],
            "j5_motor_velocity":
                motor["velocity"],
            "j5_motor_current":
                motor["current"],
            "j5_motor_torque":
                motor["torque"],
            "loop_dt":
                loop_dt,
        }

        for i in range(6):
            row["q_{}".format(i + 1)] = q[i]
            row["error_{}".format(i + 1)] = e[i]

        writer.writerow(row)

        if phase_name == "measure":
            samples.append({
                "error": e[J],
                "torque": motor["torque"],
                "current": motor["current"],
            })

    return samples


def summarize(samples):
    if not samples:
        raise RuntimeError("no measurement samples")

    err = [
        x["error"]
        for x in samples
    ]

    tau = [
        x["torque"]
        for x in samples
        if math.isfinite(x["torque"])
    ]

    current = [
        x["current"]
        for x in samples
        if math.isfinite(x["current"])
    ]

    return {
        "error":
            sum(err) / len(err),

        "torque":
            (
                sum(tau) / len(tau)
                if tau
                else float("nan")
            ),

        "current":
            (
                sum(current) / len(current)
                if current
                else float("nan")
            ),
    }


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--live",
        action="store_true",
    )

    parser.add_argument(
        "--step",
        type=float,
        default=FF_STEP,
        help="t_ff increment magnitude [N*m], default 0.10",
    )

    parser.add_argument(
        "--max-ff",
        type=float,
        default=MAX_FF_MAG,
        help="maximum |t_ff| tested [N*m], default 1.00",
    )

    parser.add_argument(
        "--ramp",
        type=float,
        default=RAMP_SEC,
    )

    parser.add_argument(
        "--settle",
        type=float,
        default=SETTLE_SEC,
    )

    parser.add_argument(
        "--measure",
        type=float,
        default=MEASURE_SEC,
    )

    args = parser.parse_args()

    if args.step <= 0.0:
        raise ValueError("--step must be > 0")

    if args.max_ff <= 0.0:
        raise ValueError("--max-ff must be > 0")

    if args.max_ff > 1.5:
        raise ValueError(
            "Phase 2B-3 software guard: --max-ff must be <= 1.5 N*m"
        )

    arm = None
    f = None

    try:
        print("=" * 78)
        print("Piper Phase 2B-3 rev2: Adaptive J5 t_ff Characterization")
        print("=" * 78)

        arm = create_piper()
        arm.connect()
        time.sleep(1.0)

        if not arm.is_ok():
            raise RuntimeError("RIGHT CAN failed")

        q_before = read_q(arm)

        print(
            "RIGHT J5 before MIT: {:.3f} rad / {:.2f} deg"
            .format(
                q_before[J],
                q_before[J] * RAD_TO_DEG,
            )
        )

        if not args.live:
            print(
                "[DRY RUN] Adaptive sweep will choose correction "
                "direction from the live zero-t_ff baseline."
            )
            print(
                "Maximum test magnitude: +/-{:.2f} N*m"
                .format(args.max_ff)
            )
            return

        input(
            "Confirm RIGHT workspace is clear and pose is safe, "
            "then press Enter to enable MIT... "
        )

        arm.enable()
        time.sleep(0.15)
        arm.set_motion_mode("mit")
        time.sleep(0.05)

        # Capture target after the mode transition. This means the test is about
        # static MIT load error from this post-transition pose, not about the
        # transition displacement itself.
        q_des = read_q(arm)

        print(
            "J5 test target after MIT transition: "
            "{:.3f} rad / {:.2f} deg"
            .format(
                q_des[J],
                q_des[J] * RAD_TO_DEG,
            )
        )

        log_path = os.path.join(
            "data",
            "mit_teleop",
            time.strftime("%Y%m%d_%H%M%S")
            + "_phase2b3_j5_tff_adaptive.csv",
        )

        os.makedirs(
            os.path.dirname(log_path),
            exist_ok=True,
        )

        f = open(
            log_path,
            "w",
            newline="",
        )

        fields = [
            "time",
            "stage_index",
            "phase",
            "phase_time",
            "t_ff_cmd",
            "j5_target",
            "j5_actual",
            "j5_error",
            "j5_motor_position",
            "j5_motor_velocity",
            "j5_motor_current",
            "j5_motor_torque",
            "loop_dt",
        ]

        for i in range(6):
            fields.append(
                "q_{}".format(i + 1)
            )
            fields.append(
                "error_{}".format(i + 1)
            )

        writer = csv.DictWriter(
            f,
            fieldnames=fields,
        )
        writer.writeheader()

        print("Logging:", log_path)

        experiment_start = time.monotonic()

        # ----------------------------------------------------
        # Baseline t_ff = 0
        # ----------------------------------------------------
        print()
        print("[BASELINE] t_ff=+0.000 N*m")

        ramp_ff(
            arm,
            q_des,
            0.0,
            0.0,
            args.ramp,
        )

        run_phase(
            arm,
            q_des,
            writer,
            experiment_start,
            0,
            "settle",
            0.0,
            args.settle,
        )

        baseline_samples = run_phase(
            arm,
            q_des,
            writer,
            experiment_start,
            0,
            "measure",
            0.0,
            args.measure,
        )

        baseline = summarize(
            baseline_samples
        )

        baseline_error = baseline["error"]

        print(
            "  J5 err={:+.3f} deg  "
            "motor_tau={:+.3f} N*m  "
            "current={:+.3f} A"
            .format(
                baseline_error * RAD_TO_DEG,
                baseline["torque"],
                baseline["current"],
            )
        )

        if abs(
            baseline_error * RAD_TO_DEG
        ) <= ZERO_ERROR_STOP_DEG:
            print(
                "[DONE] Baseline error is already within "
                "{:.1f} deg."
                .format(ZERO_ERROR_STOP_DEG)
            )
            return

        # For positive Kp, feed-forward with the SAME SIGN as the position
        # error supplies part of the torque otherwise generated by Kp*error.
        direction = (
            1.0
            if baseline_error > 0.0
            else -1.0
        )

        print(
            "Adaptive correction direction: {} t_ff"
            .format(
                "positive"
                if direction > 0.0
                else "negative"
            )
        )

        previous_ff = 0.0
        previous_error = baseline_error
        crossed_zero = False

        max_steps = int(
            math.floor(
                args.max_ff / args.step
                + 1e-9
            )
        )

        for step_index in range(
            1,
            max_steps + 1,
        ):
            ff = (
                direction
                * args.step
                * step_index
            )

            print()
            print(
                "[STEP {:02d}] t_ff={:+.3f} N*m"
                .format(
                    step_index,
                    ff,
                )
            )

            ramp_ff(
                arm,
                q_des,
                previous_ff,
                ff,
                args.ramp,
            )

            run_phase(
                arm,
                q_des,
                writer,
                experiment_start,
                step_index,
                "settle",
                ff,
                args.settle,
            )

            samples = run_phase(
                arm,
                q_des,
                writer,
                experiment_start,
                step_index,
                "measure",
                ff,
                args.measure,
            )

            result = summarize(
                samples
            )

            error = result["error"]

            print(
                "  J5 err={:+.3f} deg  "
                "motor_tau={:+.3f} N*m  "
                "current={:+.3f} A"
                .format(
                    error * RAD_TO_DEG,
                    result["torque"],
                    result["current"],
                )
            )

            # ------------------------------------------------
            # Adaptive stop criteria
            # ------------------------------------------------
            if (
                abs(error)
                > abs(baseline_error)
                + WORSENING_ABORT_DEG / RAD_TO_DEG
            ):
                raise RuntimeError(
                    "adaptive worsening guard: "
                    "|J5 error| became > baseline by {:.1f} deg"
                    .format(
                        WORSENING_ABORT_DEG
                    )
                )

            if (
                error == 0.0
                or (
                    error > 0.0
                    and previous_error < 0.0
                )
                or (
                    error < 0.0
                    and previous_error > 0.0
                )
            ):
                print(
                    "[STOP] J5 tracking error crossed zero."
                )
                crossed_zero = True
                previous_ff = ff
                previous_error = error
                break

            if (
                abs(error * RAD_TO_DEG)
                <= ZERO_ERROR_STOP_DEG
            ):
                print(
                    "[STOP] |J5 error| <= {:.1f} deg."
                    .format(
                        ZERO_ERROR_STOP_DEG
                    )
                )
                previous_ff = ff
                previous_error = error
                break

            previous_ff = ff
            previous_error = error

        print()
        print(
            "Return t_ff smoothly to zero..."
        )

        ramp_ff(
            arm,
            q_des,
            previous_ff,
            0.0,
            args.ramp,
        )

        print("Experiment complete.")
        print("CSV:", log_path)

        if crossed_zero:
            print(
                "The zero-error compensation lies between "
                "the last two tested t_ff values."
            )

    except KeyboardInterrupt:
        print("\nSTOP")

    except Exception as exc:
        print("\n[FAULT]", exc)

    finally:
        if f is not None:
            f.close()

        if args.live and arm is not None:
            try:
                # Always command zero feed-forward before leaving MIT.
                q_hold = read_q(arm)
                for _ in range(40):
                    send_mit(
                        arm,
                        q_hold,
                        0.0,
                    )
                    time.sleep(PERIOD)

                arm.set_motion_mode("p")
                arm.disable()

            except Exception as exc:
                print(
                    "[WARN] shutdown:",
                    exc,
                )

        if arm is not None:
            try:
                arm.disconnect()
            except Exception as exc:
                print(
                    "[WARN] disconnect:",
                    exc,
                )


if __name__ == "__main__":
    main()
