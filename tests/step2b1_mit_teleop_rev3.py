#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Piper Phase 2B-1 rev3: timing-correct MIT teleoperation baseline

Purpose
-------
This stage does NOT try to make the controller stiffer.
It first makes the software teleoperation path measurable and internally
consistent:

1. LEFT raw SocketCAN reader publishes coherent 6-joint snapshots.
2. Leader velocity uses real sample timestamps, not a fixed 1/200 s dt.
3. Velocity low-pass state is kept unscaled; VEL_SCALE is applied only when
   producing MIT v_des.
4. Main loop uses deadline-based scheduling instead of "work + sleep(5 ms)".
5. Leader freshness is enforced with a watchdog.
6. RIGHT joint feedback is read once per control cycle.
7. Logs separate:
      tracking_error = q_target - q_slave
      absolute_gap   = q_master - q_slave
      motion_error   = (q_master-q_master0) - (q_slave-q_slave0)
   so startup pose offset is not confused with MIT tracking error.
8. CSV logging is moved to a background thread.

Run
---
Dry run first:
    python tests/step2b1_mit_teleop.py

Live:
    python tests/step2b1_mit_teleop.py --live

Live + log:
    python tests/step2b1_mit_teleop.py --live --record

30 s diagnostic:
    python tests/step2b1_mit_teleop.py --live --record --duration 30

Optional constant software joint-zero correction, degrees:
    python tests/step2b1_mit_teleop.py --live \
        --cal-offset-deg 0,0,0,2.0,0,0

Safety
------
- Start without --live.
- Keep the robot workspace clear.
- cal-offset is only for a diagnosed constant joint-zero/geometric bias.
  Do not use it to hide posture-dependent MIT tracking error.
"""

import argparse
import csv
import math
import os
import queue
import socket
import struct
import threading
import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


# ============================================================
# Hardware / control configuration
# ============================================================

LEFT_CAN = "can_piper_left"
RIGHT_CAN = "can_piper_right"

CONTROL_RATE = 200.0
CONTROL_PERIOD = 1.0 / CONTROL_RATE

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)

CAN_J12 = 0x155
CAN_J34 = 0x156
CAN_J56 = 0x157

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


# Keep the already-tested v2 gains unchanged in Phase 2B-1.
MIT_KP = [5.0, 5.0, 5.0, 4.0, 4.0, 3.0]
MIT_KD = [1.0, 1.0, 1.0, 0.6, 0.6, 0.5]

VEL_SCALE = 0.30

# v2 FILTER=0.7 at about 200 Hz corresponds roughly to an ~11 Hz
# first-order low-pass. Use a time-based cutoff so behavior remains sensible
# when leader sample timing varies.
VEL_LPF_HZ = 10.0

MAX_V_DES = [1.0] * 6

# Leader watchdog has two stages:
# - SOFT: freeze follower target and force v_des=0 while waiting for recovery.
# - HARD: leave MIT and shut down if the outage persists.
#
# The S-V1.8-9 Leader stream observed on the real arm can briefly pause during
# role transition, so a single 100 ms threshold is too aggressive.
LEADER_SOFT_TIMEOUT = 0.050
LEADER_HARD_TIMEOUT = 0.500

# Before enabling follower MIT, require a continuously healthy Leader stream.
LEADER_STABLE_TIME = 0.50

# Guard against a corrupted/de-synchronized leader snapshot. This is NOT a
# joint limit; it only rejects implausibly large single-update target jumps.
MAX_TARGET_STEP_RAD = 0.20

STATUS_HZ = 5.0
STATUS_PERIOD = 1.0 / STATUS_HZ


# ============================================================
# Helpers
# ============================================================

def create_piper(channel):
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )
    return AgxArmFactory.create_arm(cfg)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def parse_six_deg(text):
    parts = [x.strip() for x in text.split(",")]

    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            "expected 6 comma-separated values, e.g. 0,0,0,2,0,0"
        )

    try:
        return [float(x) * DEG_TO_RAD for x in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc))


def vector_sub(a, b):
    return [x - y for x, y in zip(a, b)]


# ============================================================
# LEFT leader raw SocketCAN reader
# ============================================================

class LeaderReader:
    """
    Read the three leader joint-pair frames into one protected cache.

    Unlike the first Phase 2B-1 draft, this does NOT require J12/J34/J56 to
    arrive in a strict "all three, then publish" cycle. On the real Piper CAN
    stream the pair frames are asynchronous, so that rule can unnecessarily
    prevent publication.

    Instead:
    - every pair updates its own timestamp and counter;
    - get() returns the latest 6-joint cache only when all three pairs have
      been seen at least once;
    - watchdog age is based on the OLDEST of the three pair timestamps;
    - pair ages/counters are exposed for diagnosis.
    """

    def __init__(self, channel):
        self.channel = channel

        self.sock = None
        self.thread = None
        self.running = False

        self.lock = threading.Lock()

        self.q = [None] * 6
        self.pair_ts = [0.0, 0.0, 0.0]
        self.pair_count = [0, 0, 0]

        # seq increments whenever any relevant pair frame is received.
        self.seq = 0

        # sample_ts follows the newest relevant leader pair frame.
        self.sample_ts = 0.0

        self.last_exception = None

    @staticmethod
    def decode_joint(data):
        raw = int.from_bytes(data, "big", signed=True)
        return raw * 0.001 * DEG_TO_RAD

    def start(self):
        self.sock = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )
        self.sock.bind((self.channel,))
        self.sock.settimeout(0.1)

        self.running = True
        self.thread = threading.Thread(
            target=self._loop,
            name="leader-can-reader",
            daemon=True,
        )
        self.thread.start()

    def _loop(self):
        try:
            while self.running:
                try:
                    frame = self.sock.recv(CAN_FRAME_SIZE)
                except socket.timeout:
                    continue
                except OSError:
                    if self.running:
                        raise
                    break

                if len(frame) != CAN_FRAME_SIZE:
                    continue

                can_id, _dlc, data = struct.unpack(
                    CAN_FRAME_FORMAT,
                    frame,
                )
                can_id &= 0x1FFFFFFF

                now = time.monotonic()

                with self.lock:
                    if can_id == CAN_J12:
                        self.q[0] = self.decode_joint(data[0:4])
                        self.q[1] = self.decode_joint(data[4:8])
                        self.pair_ts[0] = now
                        self.pair_count[0] += 1

                    elif can_id == CAN_J34:
                        self.q[2] = self.decode_joint(data[0:4])
                        self.q[3] = self.decode_joint(data[4:8])
                        self.pair_ts[1] = now
                        self.pair_count[1] += 1

                    elif can_id == CAN_J56:
                        self.q[4] = self.decode_joint(data[0:4])
                        self.q[5] = self.decode_joint(data[4:8])
                        self.pair_ts[2] = now
                        self.pair_count[2] += 1

                    else:
                        continue

                    self.seq += 1
                    self.sample_ts = now

        except Exception as exc:
            self.last_exception = repr(exc)
            self.running = False

    def get(self):
        now = time.monotonic()

        with self.lock:
            pair_count = list(self.pair_count)
            pair_ts = list(self.pair_ts)

            diagnostic = {
                "counts": pair_count,
                "ages": [
                    (now - ts) if ts > 0.0 else float("inf")
                    for ts in pair_ts
                ],
                "exception": self.last_exception,
            }

            if any(x is None for x in self.q):
                return (
                    None,
                    float("inf"),
                    self.seq,
                    self.sample_ts,
                    diagnostic,
                )

            # Conservative leader freshness: all three joint-pair values
            # must still be recent enough.
            age = now - min(pair_ts)

            return (
                list(self.q),
                age,
                self.seq,
                self.sample_ts,
                diagnostic,
            )

    def stop(self):
        self.running = False

        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

        if self.thread is not None:
            self.thread.join(timeout=1.0)


# ============================================================
# Async logger
# ============================================================

class AsyncCsvLogger:
    """
    Background CSV writer.

    Avoid file.flush() in every real-time control iteration. Rows are queued
    by the control loop and written by this thread.
    """

    def __init__(self, path):
        self.path = path
        self.queue = queue.Queue(maxsize=20000)
        self.stop_event = threading.Event()
        self.dropped = 0

        self.thread = threading.Thread(
            target=self._run,
            name="teleop-csv-logger",
            daemon=True,
        )
        self.thread.start()

    @staticmethod
    def fieldnames():
        fields = [
            "time",
            "loop_dt",
            "leader_age",
            "leader_seq",
            "leader_sample_dt",
            "mit_send_ms",
            "feedback_read_ms",
            "loop_work_ms",
            "deadline_late_ms",
        ]

        for prefix in (
            "master",
            "master_delta",
            "dq_raw",
            "dq_filt",
            "v_des",
            "target",
            "slave",
            "slave_delta",
            "tracking_error",
            "motion_error",
            "absolute_gap",
        ):
            for i in range(6):
                fields.append("{}_{}".format(prefix, i + 1))

        return fields

    def write(self, row):
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        with open(self.path, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=self.fieldnames(),
            )
            writer.writeheader()

            while (
                not self.stop_event.is_set()
                or not self.queue.empty()
            ):
                try:
                    row = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                writer.writerow(row)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)


# ============================================================
# Velocity estimator
# ============================================================

class LeaderVelocityEstimator:
    """
    Velocity from actual leader snapshot timestamps.

    dq_filt is an UN-SCALED physical velocity estimate [rad/s].
    VEL_SCALE is applied later, once, when constructing MIT v_des.
    """

    def __init__(self, q0, sample_ts0):
        self.last_q = list(q0)
        self.last_sample_ts = sample_ts0

        self.dq_raw = [0.0] * 6
        self.dq_filt = [0.0] * 6

    def update(self, q, sample_ts):
        dt = sample_ts - self.last_sample_ts

        # Reject pathological timestamp intervals.
        if dt <= 0.0005 or dt > LEADER_HARD_TIMEOUT:
            self.last_q = list(q)
            self.last_sample_ts = sample_ts
            return 0.0, list(self.dq_raw), list(self.dq_filt)

        self.dq_raw = [
            (q[i] - self.last_q[i]) / dt
            for i in range(6)
        ]

        # First-order LPF:
        # old_weight = exp(-2*pi*fc*dt)
        old_weight = math.exp(
            -2.0 * math.pi * VEL_LPF_HZ * dt
        )

        self.dq_filt = [
            old_weight * self.dq_filt[i]
            + (1.0 - old_weight) * self.dq_raw[i]
            for i in range(6)
        ]

        self.last_q = list(q)
        self.last_sample_ts = sample_ts

        return (
            dt,
            list(self.dq_raw),
            list(self.dq_filt),
        )


# ============================================================
# MIT output / shutdown
# ============================================================

def send_mit(right, q_target, v_des):
    for i in range(6):
        right.move_mit(
            i + 1,
            p_des=q_target[i],
            v_des=v_des[i],
            kp=MIT_KP[i],
            kd=MIT_KD[i],
            t_ff=0.0,
        )


def safe_hold_and_disable(right):
    """
    Briefly hold the current follower pose in MIT, then leave MIT mode
    and disable.
    """
    if right is None:
        return

    try:
        feedback = right.get_joint_angles()
        if feedback is None:
            raise RuntimeError("no RIGHT joint feedback during shutdown")

        q_hold = list(feedback.msg)

        for _ in range(20):
            for i in range(6):
                right.move_mit(
                    i + 1,
                    p_des=q_hold[i],
                    v_des=0.0,
                    kp=5.0,
                    kd=0.5,
                    t_ff=0.0,
                )

            time.sleep(0.01)

        right.set_motion_mode("p")
        right.disable()

    except Exception as exc:
        print("[WARN] safe shutdown:", exc)


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--live",
        action="store_true",
        help="actually send MIT commands to RIGHT",
    )

    parser.add_argument(
        "--record",
        action="store_true",
        help="record Phase 2B-1 CSV diagnostics",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after N seconds; 0 means run until Ctrl+C",
    )

    parser.add_argument(
        "--cal-offset-deg",
        type=parse_six_deg,
        default=[0.0] * 6,
        help=(
            "constant software joint-zero correction in degrees, "
            "e.g. 0,0,0,2,0,0"
        ),
    )

    args = parser.parse_args()

    right = None
    left = None
    leader = None
    logger = None

    try:
        print("=" * 78)
        print("Piper Phase 2B-1 MIT Teleoperation (rev3)")
        print("=" * 78)

        # ----------------------------------------------------
        # RIGHT follower connection
        # ----------------------------------------------------
        print("[1] Connect RIGHT")

        right = create_piper(RIGHT_CAN)
        right.connect()
        time.sleep(1.0)

        if not right.is_ok():
            raise RuntimeError("RIGHT CAN failed")

        # ----------------------------------------------------
        # LEFT raw CAN + SDK leader mode
        # ----------------------------------------------------
        print("[2] Start LEFT raw SocketCAN reader")

        leader = LeaderReader(LEFT_CAN)
        leader.start()

        print("[3] Connect LEFT SDK path")

        left = create_piper(LEFT_CAN)
        left.connect()
        time.sleep(0.5)

        input(
            "Place BOTH arms in any safe starting poses, then press Enter "
            "to capture synchronization anchors... "
        )

        left.set_leader_mode()

        # ----------------------------------------------------
        # Wait for all three leader joint-pair frames AND require
        # a continuously healthy stream before capturing anchors.
        #
        # No pose matching is required: qm0 and q_slave0 are
        # arbitrary synchronization anchors for relative motion.
        # ----------------------------------------------------
        print("[4] Wait for stable leader joint-pair data")

        wait_deadline = time.monotonic() + 8.0
        last_diag_print = 0.0
        stable_since = None
        stable_counts0 = None

        while True:
            (
                qm,
                age,
                leader_seq,
                sample_ts,
                leader_diag,
            ) = leader.get()

            now = time.monotonic()

            healthy = (
                qm is not None
                and age < LEADER_SOFT_TIMEOUT
                and leader_diag["exception"] is None
            )

            if healthy:
                if stable_since is None:
                    stable_since = now
                    stable_counts0 = list(
                        leader_diag["counts"]
                    )

                counts_advanced = all(
                    leader_diag["counts"][i]
                    > stable_counts0[i]
                    for i in range(3)
                )

                if (
                    counts_advanced
                    and now - stable_since
                    >= LEADER_STABLE_TIME
                ):
                    break

            else:
                stable_since = None
                stable_counts0 = None

            if now - last_diag_print > 0.5:
                ages_ms = [
                    round(x * 1000.0, 1)
                    if math.isfinite(x)
                    else None
                    for x in leader_diag["ages"]
                ]

                stable_ms = (
                    round(
                        (now - stable_since)
                        * 1000.0,
                        1,
                    )
                    if stable_since is not None
                    else 0.0
                )

                print(
                    "  leader counts J12/J34/J56={}  "
                    "ages_ms={}  stable_ms={}  thread_error={}"
                    .format(
                        leader_diag["counts"],
                        ages_ms,
                        stable_ms,
                        leader_diag["exception"],
                    )
                )
                last_diag_print = now

            if now > wait_deadline:
                raise RuntimeError(
                    "leader stream did not become stable; "
                    "counts={} ages_ms={} error={}"
                    .format(
                        leader_diag["counts"],
                        [
                            round(x * 1000.0, 1)
                            if math.isfinite(x)
                            else None
                            for x in leader_diag["ages"]
                        ],
                        leader_diag["exception"],
                    )
                )

            time.sleep(0.01)

        qm0 = list(qm)

        right_feedback = right.get_joint_angles()
        if right_feedback is None:
            raise RuntimeError(
                "no RIGHT joint feedback"
            )

        q_slave0 = list(right_feedback.msg)

        print("Master reference [rad]:", qm0)
        print("Slave reference  [rad]:", q_slave0)

        startup_gap = [
            qm0[i] - q_slave0[i]
            for i in range(6)
        ]

        print(
            "Startup master-slave gap [deg]:",
            [
                round(x * RAD_TO_DEG, 3)
                for x in startup_gap
            ],
        )

        print(
            "Software calibration [deg]:",
            [
                round(x * RAD_TO_DEG, 3)
                for x in args.cal_offset_deg
            ],
        )

        # ----------------------------------------------------
        # Logger
        # ----------------------------------------------------
        if args.record:
            log_path = os.path.join(
                "data",
                "mit_teleop",
                time.strftime("%Y%m%d_%H%M%S")
                + "_phase2b1.csv",
            )

            logger = AsyncCsvLogger(log_path)
            print("Logging:", log_path)

        # ----------------------------------------------------
        # Enable MIT only after all references are valid
        # ----------------------------------------------------
        if args.live:
            print("[5] Enable RIGHT MIT")

            right.enable()
            time.sleep(0.3)

            right.set_motion_mode("mit")
            time.sleep(0.3)

        else:
            print(
                "[DRY RUN] RIGHT MIT commands are NOT sent"
            )

        # ----------------------------------------------------
        # Control state
        # ----------------------------------------------------
        velocity_init_ts = time.monotonic()

        velocity = LeaderVelocityEstimator(
            qm,
            velocity_init_ts,
        )

        previous_velocity_q = list(qm)
        previous_velocity_ts = time.monotonic()

        previous_target = [
            q_slave0[i] + args.cal_offset_deg[i]
            for i in range(6)
        ]

        dq_raw = [0.0] * 6
        dq_filt = [0.0] * 6

        start = time.monotonic()
        previous_loop_start = start
        next_tick = start
        next_status = start

        was_leader_stale = False
        stale_since = None

        print("[6] Phase 2B-1 loop started")

        # ----------------------------------------------------
        # Control loop
        # ----------------------------------------------------
        while True:
            now = time.monotonic()

            if now < next_tick:
                time.sleep(next_tick - now)

            loop_start = time.monotonic()
            loop_dt = loop_start - previous_loop_start
            previous_loop_start = loop_start

            if (
                args.duration > 0.0
                and loop_start - start >= args.duration
            ):
                break

            # Absolute deadline for this iteration.
            next_tick += CONTROL_PERIOD

            (
                qm,
                age,
                leader_seq,
                sample_ts,
                leader_diag,
            ) = leader.get()

            if qm is None:
                raise RuntimeError(
                    "leader state unavailable"
                )

            if leader_diag["exception"] is not None:
                raise RuntimeError(
                    "leader reader thread failed: {}"
                    .format(
                        leader_diag["exception"]
                    )
                )

            # --------------------------------------------
            # Two-stage Leader watchdog
            # --------------------------------------------
            leader_stale = (
                age > LEADER_SOFT_TIMEOUT
            )

            if age > LEADER_HARD_TIMEOUT:
                ages_ms = [
                    round(x * 1000.0, 1)
                    if math.isfinite(x)
                    else None
                    for x in leader_diag["ages"]
                ]

                raise RuntimeError(
                    "leader HARD timeout: age={:.1f} ms; "
                    "counts={} pair_ages_ms={}"
                    .format(
                        age * 1000.0,
                        leader_diag["counts"],
                        ages_ms,
                    )
                )

            if leader_stale:
                # Freeze follower at the last safe target.
                q_target = list(previous_target)
                v_des = [0.0] * 6
                leader_sample_dt = 0.0

                # Keep the velocity estimator from differentiating
                # across the missing-data interval after recovery.
                velocity_init_ts = time.monotonic()
                velocity = LeaderVelocityEstimator(
                    qm,
                    velocity_init_ts,
                )
                previous_velocity_ts = (
                    velocity_init_ts
                )
                dq_raw = [0.0] * 6
                dq_filt = [0.0] * 6

            else:
                # ----------------------------------------
                # Velocity update from latest 6-joint cache.
                #
                # The three CAN pair frames are asynchronous,
                # so raw CAN seq is not treated as a complete
                # six-joint sample clock.
                # ----------------------------------------
                velocity_now = time.monotonic()
                leader_sample_dt = (
                    velocity_now
                    - previous_velocity_ts
                )

                if leader_sample_dt > 0.0005:
                    (
                        _velocity_dt,
                        dq_raw,
                        dq_filt,
                    ) = velocity.update(
                        qm,
                        velocity_now,
                    )

                    previous_velocity_q = list(qm)
                    previous_velocity_ts = velocity_now

                v_des = [
                    clamp(
                        VEL_SCALE * dq_filt[i],
                        -MAX_V_DES[i],
                        MAX_V_DES[i],
                    )
                    for i in range(6)
                ]

                # ----------------------------------------
                # Keep v2 RELATIVE mapping.
                # ----------------------------------------
                q_target = [
                    q_slave0[i]
                    + (qm[i] - qm0[i])
                    + args.cal_offset_deg[i]
                    for i in range(6)
                ]

            target_step = max(
                abs(
                    q_target[i]
                    - previous_target[i]
                )
                for i in range(6)
            )

            if target_step > MAX_TARGET_STEP_RAD:
                raise RuntimeError(
                    "target jump {:.3f} rad "
                    "exceeds guard {:.3f} rad".format(
                        target_step,
                        MAX_TARGET_STEP_RAD,
                    )
                )

            # --------------------------------------------
            # Leader stale/recovery transition diagnostics
            # --------------------------------------------
            if leader_stale and not was_leader_stale:
                stale_since = loop_start

                print(
                    "[WATCHDOG] Leader stale -> HOLD: "
                    "age={:.1f} ms counts={} pair_ages_ms={}"
                    .format(
                        age * 1000.0,
                        leader_diag["counts"],
                        [
                            round(x * 1000.0, 1)
                            if math.isfinite(x)
                            else None
                            for x in leader_diag["ages"]
                        ],
                    )
                )

            elif (
                not leader_stale
                and was_leader_stale
            ):
                outage_ms = (
                    (loop_start - stale_since)
                    * 1000.0
                    if stale_since is not None
                    else 0.0
                )

                print(
                    "[WATCHDOG] Leader recovered after "
                    "{:.1f} ms; resume tracking"
                    .format(outage_ms)
                )

                stale_since = None

            was_leader_stale = leader_stale

            # --------------------------------------------
            # MIT send timing
            # --------------------------------------------
            send_start = time.monotonic()

            if args.live:
                send_mit(
                    right,
                    q_target,
                    v_des,
                )

            send_end = time.monotonic()

            # --------------------------------------------
            # Exactly one follower feedback read
            # --------------------------------------------
            feedback_start = time.monotonic()

            feedback = right.get_joint_angles()

            if feedback is None:
                raise RuntimeError(
                    "RIGHT joint feedback unavailable"
                )

            q_slave = list(feedback.msg)

            feedback_end = time.monotonic()

            # --------------------------------------------
            # Separate three different error concepts
            # --------------------------------------------
            master_delta = vector_sub(
                qm,
                qm0,
            )

            slave_delta = vector_sub(
                q_slave,
                q_slave0,
            )

            # Actual MIT tracking:
            # if this remains nonzero in a static pose,
            # gravity/friction / low Kp are suspects.
            tracking_error = vector_sub(
                q_target,
                q_slave,
            )

            # Relative motion transparency:
            # should approach zero if follower motion
            # reproduces leader displacement.
            motion_error = vector_sub(
                master_delta,
                slave_delta,
            )

            # Raw joint-coordinate difference:
            # relative mapping does NOT force this to zero.
            absolute_gap = vector_sub(
                qm,
                q_slave,
            )

            work_end = time.monotonic()

            deadline_late = max(
                0.0,
                work_end - next_tick,
            )

            # --------------------------------------------
            # Logger
            # --------------------------------------------
            if logger is not None:
                row = {
                    "time":
                        loop_start - start,

                    "loop_dt":
                        loop_dt,

                    "leader_age":
                        age,

                    "leader_seq":
                        leader_seq,

                    "leader_sample_dt":
                        leader_sample_dt,

                    "mit_send_ms":
                        (send_end - send_start)
                        * 1000.0,

                    "feedback_read_ms":
                        (
                            feedback_end
                            - feedback_start
                        )
                        * 1000.0,

                    "loop_work_ms":
                        (work_end - loop_start)
                        * 1000.0,

                    "deadline_late_ms":
                        deadline_late
                        * 1000.0,
                }

                vectors = (
                    ("master", qm),
                    ("master_delta", master_delta),
                    ("dq_raw", dq_raw),
                    ("dq_filt", dq_filt),
                    ("v_des", v_des),
                    ("target", q_target),
                    ("slave", q_slave),
                    ("slave_delta", slave_delta),
                    (
                        "tracking_error",
                        tracking_error,
                    ),
                    (
                        "motion_error",
                        motion_error,
                    ),
                    (
                        "absolute_gap",
                        absolute_gap,
                    ),
                )

                for prefix, values in vectors:
                    for i in range(6):
                        row[
                            "{}_{}".format(
                                prefix,
                                i + 1,
                            )
                        ] = values[i]

                logger.write(row)

            # --------------------------------------------
            # Low-rate terminal diagnostics
            # --------------------------------------------
            if loop_start >= next_status:
                max_tracking = max(
                    abs(x)
                    for x in tracking_error
                )

                print(
                    "age={:5.1f}ms  "
                    "loop={:5.2f}ms  "
                    "work={:5.2f}ms  "
                    "send={:5.2f}ms  "
                    "fb={:5.2f}ms  "
                    "track_max={:6.3f}rad  "
                    "J4_track={:+6.3f}rad  "
                    "J4_abs_gap={:+6.3f}rad  "
                    "leader={}".format(
                        age * 1000.0,
                        loop_dt * 1000.0,
                        (
                            work_end
                            - loop_start
                        ) * 1000.0,
                        (
                            send_end
                            - send_start
                        ) * 1000.0,
                        (
                            feedback_end
                            - feedback_start
                        ) * 1000.0,
                        max_tracking,
                        tracking_error[3],
                        absolute_gap[3],
                        (
                            "HOLD"
                            if leader_stale
                            else "OK"
                        ),
                    )
                )

                next_status = (
                    loop_start
                    + STATUS_PERIOD
                )

            previous_target = q_target

            # Do not run "catch-up bursts" after a missed deadline.
            # If work took >5 ms, start the next iteration immediately
            # and re-anchor the schedule to the current time.
            if deadline_late > 0.0:
                next_tick = work_end

    except KeyboardInterrupt:
        print("\nSTOP")

    except Exception as exc:
        print("\n[FAULT]", exc)

    finally:
        print("Shutdown")

        if (
            args.live
            and right is not None
        ):
            safe_hold_and_disable(right)

        if logger is not None:
            logger.close()

            if logger.dropped:
                print(
                    "[WARN] logger dropped "
                    "{} rows".format(
                        logger.dropped
                    )
                )

        if right is not None:
            try:
                right.disconnect()
            except Exception as exc:
                print(
                    "[WARN] RIGHT disconnect:",
                    exc,
                )

        if left is not None:
            try:
                left.disconnect()
            except Exception as exc:
                print(
                    "[WARN] LEFT disconnect:",
                    exc,
                )

        if leader is not None:
            leader.stop()


if __name__ == "__main__":
    main()
