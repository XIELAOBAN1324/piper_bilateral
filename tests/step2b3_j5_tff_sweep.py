#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Piper Phase 2B-3: J5 static MIT feed-forward torque sweep

Purpose
-------
Identify whether J5 MIT feed-forward torque (t_ff) can reduce the static
position error produced by load/gravity while keeping the same teleoperation
impedance gains.

This is intentionally a SINGLE-ARM static experiment:
- RIGHT arm only
- all six joints hold their captured current pose
- only J5 t_ff is swept
- no Leader / teleoperation mapping is involved

Default J5 t_ff sequence [N*m]:
    0, +0.1, +0.2, +0.3, 0, -0.1, -0.2, -0.3, 0

For every step:
    ramp 0.5 s -> settle 0.8 s -> measure 1.0 s

Run
---
Dry run / inspect target only:
    python tests/step2b3_j5_tff_sweep.py

Live:
    python tests/step2b3_j5_tff_sweep.py --live

Custom sweep:
    python tests/step2b3_j5_tff_sweep.py --live \
        --ff-values=0,0.05,0.10,0.15,0,-0.05,-0.10,-0.15,0

Safety
------
- Keep the workspace clear.
- Use a mechanically safe pose.
- Support the arm if needed before enabling.
- The script captures the CURRENT right-arm pose as q_des.
- It ramps t_ff smoothly and aborts if any joint deviates too far.
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
TEST_INDEX = TEST_JOINT - 1

DEFAULT_FF_VALUES = [
    0.0,
    +0.10,
    +0.20,
    +0.30,
    0.0,
    -0.10,
    -0.20,
    -0.30,
    0.0,
]

RAMP_SEC = 0.50
SETTLE_SEC = 0.80
MEASURE_SEC = 1.00

# Conservative software guards for this characterization experiment.
MAX_ABS_TFF = 0.50           # N*m, J5 experiment guard
MAX_ANY_JOINT_ERROR = 0.25   # rad (~14.3 deg)
MAX_J5_ERROR = 0.20          # rad (~11.5 deg)

RAD_TO_DEG = 180.0 / math.pi


def create_piper():
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=RIGHT_CAN,
    )
    return AgxArmFactory.create_arm(cfg)


def parse_ff_values(text):
    try:
        values = [
            float(x.strip())
            for x in text.split(",")
            if x.strip()
        ]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc))

    if not values:
        raise argparse.ArgumentTypeError(
            "at least one t_ff value is required"
        )

    for value in values:
        if abs(value) > MAX_ABS_TFF:
            raise argparse.ArgumentTypeError(
                "requested t_ff {:.3f} exceeds experiment guard +/-{:.3f} N*m"
                .format(value, MAX_ABS_TFF)
            )

    return values


def send_mit(arm, q_des, j5_tff):
    for i in range(6):
        arm.move_mit(
            i + 1,
            p_des=q_des[i],
            v_des=0.0,
            kp=KP[i],
            kd=KD[i],
            t_ff=(j5_tff if i == TEST_INDEX else 0.0),
        )


def read_joint_state(arm):
    fb = arm.get_joint_angles()
    if fb is None:
        raise RuntimeError("joint feedback unavailable")
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


def check_position_guard(q_des, q):
    errors = [
        q_des[i] - q[i]
        for i in range(6)
    ]

    max_any = max(abs(x) for x in errors)

    if max_any > MAX_ANY_JOINT_ERROR:
        raise RuntimeError(
            "joint deviation guard: max error {:.3f} rad > {:.3f} rad"
            .format(
                max_any,
                MAX_ANY_JOINT_ERROR,
            )
        )

    if abs(errors[TEST_INDEX]) > MAX_J5_ERROR:
        raise RuntimeError(
            "J5 deviation guard: error {:.3f} rad > {:.3f} rad"
            .format(
                errors[TEST_INDEX],
                MAX_J5_ERROR,
            )
        )

    return errors


def hold_and_disable(arm):
    if arm is None:
        return

    try:
        q_hold = read_joint_state(arm)

        next_tick = time.monotonic()

        for _ in range(40):
            send_mit(
                arm,
                q_hold,
                0.0,
            )

            next_tick += PERIOD
            remain = next_tick - time.monotonic()

            if remain > 0.0:
                time.sleep(remain)

        arm.set_motion_mode("p")
        arm.disable()

    except Exception as exc:
        print("[WARN] shutdown:", exc)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--live",
        action="store_true",
        help="actually enable RIGHT MIT control",
    )

    parser.add_argument(
        "--ff-values",
        type=parse_ff_values,
        default=DEFAULT_FF_VALUES,
        help=(
            "comma-separated J5 feed-forward torque sequence [N*m], "
            "e.g. --ff-values=0,0.1,0.2,0.3,0,-0.1,-0.2,-0.3,0"
        ),
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

    if args.ramp <= 0.0:
        raise ValueError("--ramp must be > 0")

    if args.settle < 0.0:
        raise ValueError("--settle must be >= 0")

    if args.measure <= 0.0:
        raise ValueError("--measure must be > 0")

    arm = None
    csv_file = None
    writer = None

    try:
        print("=" * 78)
        print("Piper Phase 2B-3: J5 t_ff Static Sweep")
        print("=" * 78)

        arm = create_piper()
        arm.connect()
        time.sleep(1.0)

        if not arm.is_ok():
            raise RuntimeError("RIGHT CAN failed")

        q0 = read_joint_state(arm)

        print("Captured RIGHT q_des [rad]:")
        print(q0)

        print(
            "J5 target: {:.3f} rad / {:.2f} deg"
            .format(
                q0[TEST_INDEX],
                q0[TEST_INDEX] * RAD_TO_DEG,
            )
        )

        print(
            "Sweep [N*m]:",
            args.ff_values,
        )

        if not args.live:
            print()
            print("[DRY RUN] No MIT commands sent.")
            print("Use --live after confirming the pose is safe.")
            return

        input(
            "Confirm RIGHT arm pose/workspace is safe, "
            "then press Enter to enable MIT... "
        )

        arm.enable()
        time.sleep(0.2)

        arm.set_motion_mode("mit")
        time.sleep(0.1)

        # Re-capture target after mode transition, immediately before test.
        q_des = read_joint_state(arm)

        log_path = os.path.join(
            "data",
            "mit_teleop",
            time.strftime("%Y%m%d_%H%M%S")
            + "_phase2b3_j5_tff.csv",
        )

        os.makedirs(
            os.path.dirname(log_path),
            exist_ok=True,
        )

        csv_file = open(
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
            fields.append("q_{}".format(i + 1))
            fields.append("error_{}".format(i + 1))

        writer = csv.DictWriter(
            csv_file,
            fieldnames=fields,
        )
        writer.writeheader()

        print("Logging:", log_path)
        print(
            "Final q_des J5: {:.3f} rad / {:.2f} deg"
            .format(
                q_des[TEST_INDEX],
                q_des[TEST_INDEX] * RAD_TO_DEG,
            )
        )

        start = time.monotonic()
        previous_loop = start
        next_tick = start
        previous_ff = 0.0

        # Initial zero-torque hold to avoid a first-step surprise.
        initial_hold_end = time.monotonic() + 0.5

        while time.monotonic() < initial_hold_end:
            send_mit(
                arm,
                q_des,
                0.0,
            )
            next_tick += PERIOD
            remain = next_tick - time.monotonic()
            if remain > 0.0:
                time.sleep(remain)

        for stage_index, target_ff in enumerate(
            args.ff_values
        ):
            print()
            print(
                "[STEP {:02d}] target t_ff={:+.3f} N*m"
                .format(
                    stage_index,
                    target_ff,
                )
            )

            stage_phases = (
                ("ramp", args.ramp),
                ("settle", args.settle),
                ("measure", args.measure),
            )

            for phase_name, phase_duration in stage_phases:
                if phase_duration <= 0.0:
                    continue

                phase_start = time.monotonic()
                phase_end = phase_start + phase_duration

                j5_err_samples = []
                torque_samples = []
                current_samples = []

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

                    phase_time = (
                        loop_start - phase_start
                    )

                    if phase_name == "ramp":
                        alpha = min(
                            1.0,
                            phase_time / phase_duration,
                        )
                        t_ff_cmd = (
                            previous_ff
                            + alpha
                            * (target_ff - previous_ff)
                        )
                    else:
                        t_ff_cmd = target_ff

                    send_mit(
                        arm,
                        q_des,
                        t_ff_cmd,
                    )

                    q = read_joint_state(arm)
                    errors = check_position_guard(
                        q_des,
                        q,
                    )

                    motor = read_j5_motor(arm)

                    if phase_name == "measure":
                        j5_err_samples.append(
                            errors[TEST_INDEX]
                        )
                        torque_samples.append(
                            motor["torque"]
                        )
                        current_samples.append(
                            motor["current"]
                        )

                    row = {
                        "time":
                            loop_start - start,

                        "stage_index":
                            stage_index,

                        "phase":
                            phase_name,

                        "phase_time":
                            phase_time,

                        "t_ff_cmd":
                            t_ff_cmd,

                        "j5_target":
                            q_des[TEST_INDEX],

                        "j5_actual":
                            q[TEST_INDEX],

                        "j5_error":
                            errors[TEST_INDEX],

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
                        row[
                            "q_{}".format(i + 1)
                        ] = q[i]
                        row[
                            "error_{}".format(i + 1)
                        ] = errors[i]

                    writer.writerow(row)

                if (
                    phase_name == "measure"
                    and j5_err_samples
                ):
                    mean_err = sum(
                        j5_err_samples
                    ) / len(j5_err_samples)

                    finite_torque = [
                        x for x in torque_samples
                        if math.isfinite(x)
                    ]

                    finite_current = [
                        x for x in current_samples
                        if math.isfinite(x)
                    ]

                    mean_tau = (
                        sum(finite_torque)
                        / len(finite_torque)
                        if finite_torque
                        else float("nan")
                    )

                    mean_current = (
                        sum(finite_current)
                        / len(finite_current)
                        if finite_current
                        else float("nan")
                    )

                    print(
                        "  measure: "
                        "J5 err={:+.3f} deg  "
                        "motor_tau={:+.3f} N*m  "
                        "current={:+.3f} A"
                        .format(
                            mean_err * RAD_TO_DEG,
                            mean_tau,
                            mean_current,
                        )
                    )

            previous_ff = target_ff

        # Return to zero t_ff before shutdown.
        print()
        print("Return J5 t_ff to zero...")

        ramp_start = time.monotonic()
        ramp_end = ramp_start + args.ramp
        start_ff = previous_ff

        while True:
            now = time.monotonic()

            if now >= ramp_end:
                break

            alpha = (
                (now - ramp_start)
                / args.ramp
            )

            t_ff_cmd = (
                start_ff
                * (1.0 - alpha)
            )

            send_mit(
                arm,
                q_des,
                t_ff_cmd,
            )

            time.sleep(PERIOD)

        send_mit(
            arm,
            q_des,
            0.0,
        )

        print("Sweep complete.")
        print("CSV:", log_path)

    except KeyboardInterrupt:
        print("\nSTOP")

    except Exception as exc:
        print("\n[FAULT]", exc)

    finally:
        if csv_file is not None:
            csv_file.close()

        if args.live and arm is not None:
            hold_and_disable(arm)

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
