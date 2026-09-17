#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Piper Phase 2B-2 Static Tracking Characterization

Purpose
-------
Keep the validated Phase 2B-1 rev4 teleoperation behavior unchanged, while
measuring static tracking error versus pose and motor load.

This stage does NOT add gravity/friction compensation yet.

Additional measurements
-----------------------
1. Leader 0x155/0x156/0x157 silence is NOT treated as a fault.
   On the tested Piper Leader stream, these frames can stop while the Leader
   is stationary. Silence therefore means HOLD LAST TARGET, v_des -> 0.

2. Publish coherent six-joint Leader snapshots only after all three joint-pair
   frames have refreshed. Velocity is updated only on a new complete snapshot.

3. Capture master/slave synchronization anchors AFTER the RIGHT arm has entered
   MIT mode and has been briefly held at its current pose. This removes the
   large startup target/velocity jump seen in rev3.

4. Default velocity feed-forward scale is 0.12, not 0.30. The empirically good
   v2 implementation accidentally fed the already-scaled velocity back into
   its filter, so its effective gain was much closer to ~0.11-0.12.

5. Keep the good rev3 timing structure: 200 Hz deadline scheduling, one RIGHT
   feedback read per cycle, async CSV logging, timing diagnostics.

Run
---
Dry run:
    python tests/step2b2_static_tracking.py

Live 30 s diagnostic:
    python tests/step2b2_static_tracking.py --live --record --duration 30

A/B velocity feed-forward:
    python tests/step2b2_static_tracking.py --live --record --duration 30 --vel-scale 0
    python tests/step2b2_static_tracking.py --live --record --duration 30 --vel-scale 0.12

Safety
------
- Both arms may start in arbitrary safe poses; they do NOT need to match.
- Relative mapping is used:
      q_target = q_slave_anchor + (q_master - q_master_anchor)
- A long period with no Leader joint-command frames is treated as zero motion,
  not as CAN failure. Reader-thread exceptions still fault immediately.
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
# Configuration
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

# Keep the already-tested v2 impedance gains unchanged in 2B-1.
MIT_KP = [5.0, 5.0, 5.0, 4.0, 4.0, 3.0]
MIT_KD = [1.0, 1.0, 1.0, 0.6, 0.6, 0.5]

DEFAULT_VEL_SCALE = 0.12
VEL_LPF_HZ = 10.0
MAX_V_DES = [0.5] * 6

# If no complete Leader sample has arrived for this long, v_des is forced to
# zero. The position target remains the last valid target.
VEL_ZERO_GAP = 0.030

# After a long quiet gap, do not differentiate across the gap. The first sample
# after recovery re-primes the velocity estimator with zero velocity.
VEL_RESET_GAP = 0.050

# Corrupted/de-synchronized target guard. This is a per-new-snapshot jump guard,
# not a robot joint limit.
MAX_TARGET_STEP_RAD = 0.20

# Brief RIGHT hold after entering MIT, before anchors are captured.
MIT_STARTUP_HOLD_SEC = 0.15

STATUS_HZ = 5.0
STATUS_PERIOD = 1.0 / STATUS_HZ

# Phase 2B-2 diagnostics
MOTOR_SAMPLE_HZ = 50.0
MOTOR_SAMPLE_PERIOD = 1.0 / MOTOR_SAMPLE_HZ

# When the Leader command stream has been quiet this long, consider the
# operator to be holding a pose. Wait SETTLE_TIME before collecting a
# static-error report.
STATIC_DETECT_TIME = 0.40
STATIC_SETTLE_TIME = 0.50
STATIC_REPORT_PERIOD = 1.00


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


def vec_sub(a, b):
    return [x - y for x, y in zip(a, b)]


# ============================================================
# Leader raw SocketCAN reader
# ============================================================

class LeaderReader:
    """
    Build coherent six-joint snapshots from 0x155/0x156/0x157.

    A new public snapshot is published only after every pair has refreshed
    since the previous publication. Silence does not invalidate the last
    snapshot because the observed Leader stream may be quiet at rest.
    """

    def __init__(self, channel):
        self.channel = channel

        self.sock = None
        self.thread = None
        self.running = False
        self.lock = threading.Lock()

        self.working_q = [None] * 6
        self.pair_ts = [0.0, 0.0, 0.0]
        self.pair_count = [0, 0, 0]
        self.dirty_mask = 0

        self.snapshot_q = None
        self.snapshot_ts = 0.0
        self.snapshot_seq = 0
        self.snapshot_skew = 0.0

        self.any_frame_count = 0
        self.last_any_frame_ts = 0.0
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

    def _publish_if_complete(self):
        if self.dirty_mask != 0b111:
            return
        if any(x is None for x in self.working_q):
            return

        self.snapshot_q = list(self.working_q)
        self.snapshot_ts = max(self.pair_ts)
        self.snapshot_skew = max(self.pair_ts) - min(self.pair_ts)
        self.snapshot_seq += 1
        self.dirty_mask = 0

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
                    self.any_frame_count += 1
                    self.last_any_frame_ts = now

                    if can_id == CAN_J12:
                        self.working_q[0] = self.decode_joint(data[0:4])
                        self.working_q[1] = self.decode_joint(data[4:8])
                        self.pair_ts[0] = now
                        self.pair_count[0] += 1
                        self.dirty_mask |= 0b001

                    elif can_id == CAN_J34:
                        self.working_q[2] = self.decode_joint(data[0:4])
                        self.working_q[3] = self.decode_joint(data[4:8])
                        self.pair_ts[1] = now
                        self.pair_count[1] += 1
                        self.dirty_mask |= 0b010

                    elif can_id == CAN_J56:
                        self.working_q[4] = self.decode_joint(data[0:4])
                        self.working_q[5] = self.decode_joint(data[4:8])
                        self.pair_ts[2] = now
                        self.pair_count[2] += 1
                        self.dirty_mask |= 0b100

                    else:
                        continue

                    self._publish_if_complete()

        except Exception as exc:
            self.last_exception = repr(exc)
            self.running = False

    def get(self):
        now = time.monotonic()

        with self.lock:
            if self.snapshot_q is None:
                quiet = float("inf")
            else:
                quiet = now - self.snapshot_ts

            if self.last_any_frame_ts > 0.0:
                bus_age = now - self.last_any_frame_ts
            else:
                bus_age = float("inf")

            return {
                "q": (
                    None
                    if self.snapshot_q is None
                    else list(self.snapshot_q)
                ),
                "seq": self.snapshot_seq,
                "sample_ts": self.snapshot_ts,
                "quiet": quiet,
                "skew": self.snapshot_skew,
                "pair_count": list(self.pair_count),
                "any_frame_count": self.any_frame_count,
                "bus_age": bus_age,
                "exception": self.last_exception,
            }

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
# Event-driven velocity estimator
# ============================================================

class LeaderVelocityEstimator:
    def __init__(self):
        self.q = None
        self.ts = None
        self.dq_raw = [0.0] * 6
        self.dq_filt = [0.0] * 6

    def reset(self, q, ts):
        self.q = list(q)
        self.ts = ts
        self.dq_raw = [0.0] * 6
        self.dq_filt = [0.0] * 6

    def update(self, q, ts):
        if self.q is None or self.ts is None:
            self.reset(q, ts)
            return 0.0, list(self.dq_raw), list(self.dq_filt)

        dt = ts - self.ts

        # A large gap is normal when the Leader has been stationary. Do not
        # differentiate across that quiet interval.
        if dt <= 0.0005 or dt > VEL_RESET_GAP:
            self.reset(q, ts)
            return dt, list(self.dq_raw), list(self.dq_filt)

        self.dq_raw = [
            (q[i] - self.q[i]) / dt
            for i in range(6)
        ]

        old_weight = math.exp(
            -2.0 * math.pi * VEL_LPF_HZ * dt
        )

        self.dq_filt = [
            old_weight * self.dq_filt[i]
            + (1.0 - old_weight) * self.dq_raw[i]
            for i in range(6)
        ]

        self.q = list(q)
        self.ts = ts

        return dt, list(self.dq_raw), list(self.dq_filt)


# ============================================================
# Async CSV logger
# ============================================================

class AsyncCsvLogger:
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
    def fields():
        out = [
            "time",
            "loop_dt",
            "leader_new",
            "leader_seq",
            "leader_quiet",
            "leader_sample_dt",
            "leader_skew_ms",
            "leader_bus_age",
            "mit_send_ms",
            "feedback_read_ms",
            "loop_work_ms",
            "deadline_late_ms",
            "motor_read_ms",
            "static_hold",
            "static_hold_time",
        ]

        for name in (
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
            "motor_q",
            "motor_dq",
            "motor_current",
            "motor_torque",
        ):
            for i in range(6):
                out.append("{}_{}".format(name, i + 1))

        return out

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
                fieldnames=self.fields(),
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
# MIT helpers
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


def hold_current_pose(right, q_hold, seconds):
    end = time.monotonic() + seconds
    next_tick = time.monotonic()

    while time.monotonic() < end:
        send_mit(
            right,
            q_hold,
            [0.0] * 6,
        )

        next_tick += CONTROL_PERIOD
        remain = next_tick - time.monotonic()
        if remain > 0.0:
            time.sleep(remain)


def safe_hold_and_disable(right):
    if right is None:
        return

    try:
        fb = right.get_joint_angles()
        if fb is None:
            raise RuntimeError(
                "no RIGHT feedback during shutdown"
            )

        q_hold = list(fb.msg)
        hold_current_pose(right, q_hold, 0.20)

        right.set_motion_mode("p")
        right.disable()

    except Exception as exc:
        print("[WARN] shutdown:", exc)


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
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
    )

    parser.add_argument(
        "--vel-scale",
        type=float,
        default=DEFAULT_VEL_SCALE,
        help="leader dq -> MIT v_des scale (default: 0.12)",
    )

    parser.add_argument(
        "--cal-offset-deg",
        type=parse_six_deg,
        default=[0.0] * 6,
    )

    args = parser.parse_args()

    if args.vel_scale < 0.0:
        raise ValueError("--vel-scale must be >= 0")

    right = None
    left = None
    leader = None
    logger = None

    try:
        print("=" * 78)
        print("Piper Phase 2B-2 Static Tracking Characterization")
        print("=" * 78)

        # ----------------------------------------------------
        # RIGHT connection and pre-MIT hold pose
        # ----------------------------------------------------
        print("[1] Connect RIGHT")

        right = create_piper(RIGHT_CAN)
        right.connect()
        time.sleep(1.0)

        if not right.is_ok():
            raise RuntimeError("RIGHT CAN failed")

        fb = right.get_joint_angles()
        if fb is None:
            raise RuntimeError("no initial RIGHT feedback")

        right_hold = list(fb.msg)

        # ----------------------------------------------------
        # LEFT raw reader + Leader mode
        # ----------------------------------------------------
        print("[2] Start LEFT raw SocketCAN reader")

        leader = LeaderReader(LEFT_CAN)
        leader.start()

        print("[3] Connect LEFT SDK path")

        left = create_piper(LEFT_CAN)
        left.connect()
        time.sleep(0.5)

        input(
            "Place both arms in any SAFE starting poses. "
            "Support LEFT, then press Enter to enter Leader mode... "
        )

        left.set_leader_mode()

        # We only need one valid complete Leader snapshot. Continuous traffic
        # is NOT required because stationary Leader command frames may stop.
        print("[4] Wait for first complete Leader snapshot")

        deadline = time.monotonic() + 8.0
        last_print = 0.0

        while True:
            state = leader.get()

            if state["exception"] is not None:
                raise RuntimeError(
                    "Leader reader failed: {}"
                    .format(state["exception"])
                )

            if state["q"] is not None:
                break

            now = time.monotonic()
            if now - last_print > 0.5:
                print(
                    "  pair_counts={} any_frames={}"
                    .format(
                        state["pair_count"],
                        state["any_frame_count"],
                    )
                )
                last_print = now

            if now > deadline:
                raise RuntimeError(
                    "no complete Leader snapshot; pair_counts={}"
                    .format(state["pair_count"])
                )

            time.sleep(0.01)

        # ----------------------------------------------------
        # Enter RIGHT MIT and hold the RIGHT arm BEFORE anchor capture.
        # This is the key startup ordering change vs rev3.
        # ----------------------------------------------------
        if args.live:
            print("[5] Enable RIGHT MIT and hold current pose")

            right.enable()
            time.sleep(0.1)

            right.set_motion_mode("mit")
            time.sleep(0.05)

            hold_current_pose(
                right,
                right_hold,
                MIT_STARTUP_HOLD_SEC,
            )

        else:
            print("[DRY RUN] RIGHT MIT commands are NOT sent")

        # ----------------------------------------------------
        # Capture anchors NOW, immediately before control starts.
        # ----------------------------------------------------
        state = leader.get()

        if state["q"] is None:
            raise RuntimeError(
                "Leader snapshot disappeared before anchor capture"
            )

        if state["exception"] is not None:
            raise RuntimeError(
                "Leader reader failed: {}"
                .format(state["exception"])
            )

        qm0 = list(state["q"])

        fb = right.get_joint_angles()
        if fb is None:
            raise RuntimeError(
                "no RIGHT feedback at anchor capture"
            )

        qs0 = list(fb.msg)

        print("Master anchor [rad]:", qm0)
        print("Slave anchor  [rad]:", qs0)
        print(
            "Startup master-slave gap [deg]:",
            [
                round((qm0[i] - qs0[i]) * RAD_TO_DEG, 3)
                for i in range(6)
            ],
        )
        print("Velocity scale:", args.vel_scale)

        # ----------------------------------------------------
        # Logger
        # ----------------------------------------------------
        if args.record:
            log_path = os.path.join(
                "data",
                "mit_teleop",
                time.strftime("%Y%m%d_%H%M%S")
                + "_phase2b2_static.csv",
            )
            logger = AsyncCsvLogger(log_path)
            print("Logging:", log_path)

        # ----------------------------------------------------
        # Control state
        # ----------------------------------------------------
        velocity = LeaderVelocityEstimator()
        velocity.reset(qm0, state["sample_ts"])

        last_snapshot_seq = state["seq"]

        q_master = list(qm0)
        q_target = list(qs0)
        previous_target = list(qs0)

        dq_raw = [0.0] * 6
        dq_filt = [0.0] * 6
        v_des = [0.0] * 6

        start = time.monotonic()
        previous_loop_start = start
        next_tick = start
        next_status = start

        # Phase 2B-2 motor/load sampling state
        next_motor_sample = start
        motor_q = [float("nan")] * 6
        motor_dq = [float("nan")] * 6
        motor_current = [float("nan")] * 6
        motor_torque = [float("nan")] * 6
        motor_read_ms = 0.0

        # Static-hold characterization state
        static_hold_start = None
        next_static_report = start

        print("[6] Phase 2B-2 loop started")

        # ----------------------------------------------------
        # 200 Hz control loop
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

            next_tick += CONTROL_PERIOD

            state = leader.get()

            if state["exception"] is not None:
                raise RuntimeError(
                    "Leader reader failed: {}"
                    .format(state["exception"])
                )

            if state["q"] is None:
                raise RuntimeError(
                    "Leader snapshot unavailable"
                )

            leader_new = int(
                state["seq"] != last_snapshot_seq
            )
            leader_sample_dt = 0.0

            if leader_new:
                q_master = list(state["q"])

                (
                    leader_sample_dt,
                    dq_raw,
                    dq_filt,
                ) = velocity.update(
                    q_master,
                    state["sample_ts"],
                )

                candidate_target = [
                    qs0[i]
                    + (q_master[i] - qm0[i])
                    + args.cal_offset_deg[i]
                    for i in range(6)
                ]

                max_step = max(
                    abs(
                        candidate_target[i]
                        - previous_target[i]
                    )
                    for i in range(6)
                )

                if max_step > MAX_TARGET_STEP_RAD:
                    raise RuntimeError(
                        "Leader target jump {:.3f} rad exceeds guard {:.3f} rad"
                        .format(
                            max_step,
                            MAX_TARGET_STEP_RAD,
                        )
                    )

                q_target = candidate_target
                previous_target = list(q_target)
                last_snapshot_seq = state["seq"]

            # Repeat latest velocity command only briefly between active Leader
            # samples. Once the Leader stream goes quiet, velocity feed-forward
            # must be zero while position target is held.
            if state["quiet"] > VEL_ZERO_GAP:
                v_des = [0.0] * 6
            else:
                v_des = [
                    clamp(
                        args.vel_scale * dq_filt[i],
                        -MAX_V_DES[i],
                        MAX_V_DES[i],
                    )
                    for i in range(6)
                ]

            # ------------------------------------------------
            # MIT send
            # ------------------------------------------------
            send_start = time.monotonic()

            if args.live:
                send_mit(
                    right,
                    q_target,
                    v_des,
                )

            send_end = time.monotonic()

            # ------------------------------------------------
            # One follower feedback read
            # ------------------------------------------------
            feedback_start = time.monotonic()

            fb = right.get_joint_angles()
            if fb is None:
                raise RuntimeError(
                    "RIGHT joint feedback unavailable"
                )

            q_slave = list(fb.msg)

            feedback_end = time.monotonic()

            # ------------------------------------------------
            # Error definitions
            # ------------------------------------------------
            master_delta = vec_sub(q_master, qm0)
            slave_delta = vec_sub(q_slave, qs0)

            tracking_error = vec_sub(
                q_target,
                q_slave,
            )

            motion_error = vec_sub(
                master_delta,
                slave_delta,
            )

            absolute_gap = vec_sub(
                q_master,
                q_slave,
            )

            # ------------------------------------------------
            # Phase 2B-2 motor-state sampling (50 Hz by default)
            # ------------------------------------------------
            motor_read_ms = 0.0

            if loop_start >= next_motor_sample:
                motor_start = time.monotonic()

                new_motor_q = []
                new_motor_dq = []
                new_motor_current = []
                new_motor_torque = []

                motor_valid = True

                for joint_index in range(1, 7):
                    state_motor = right.get_motor_states(
                        joint_index
                    )

                    if state_motor is None:
                        motor_valid = False
                        break

                    msg = state_motor.msg

                    new_motor_q.append(msg.position)
                    new_motor_dq.append(msg.velocity)
                    new_motor_current.append(msg.current)
                    new_motor_torque.append(msg.torque)

                motor_end = time.monotonic()
                motor_read_ms = (
                    motor_end - motor_start
                ) * 1000.0

                if motor_valid:
                    motor_q = new_motor_q
                    motor_dq = new_motor_dq
                    motor_current = new_motor_current
                    motor_torque = new_motor_torque

                next_motor_sample = (
                    loop_start + MOTOR_SAMPLE_PERIOD
                )

            # ------------------------------------------------
            # Static-hold detector
            #
            # On this Piper Leader stream, no joint command frames while
            # stationary is normal. Therefore leader quiet time is a useful
            # indicator for static-pose characterization.
            # ------------------------------------------------
            static_hold = (
                state["quiet"] >= STATIC_DETECT_TIME
            )

            if static_hold:
                if static_hold_start is None:
                    static_hold_start = loop_start
            else:
                static_hold_start = None

            static_hold_time = (
                loop_start - static_hold_start
                if static_hold_start is not None
                else 0.0
            )

            if (
                static_hold
                and static_hold_time >= STATIC_SETTLE_TIME
                and loop_start >= next_static_report
            ):
                err_deg = [
                    x * RAD_TO_DEG
                    for x in tracking_error
                ]

                print(
                    "[STATIC] hold={:.2f}s  "
                    "err_deg={}  torque={}"
                    .format(
                        static_hold_time,
                        [
                            round(x, 2)
                            for x in err_deg
                        ],
                        [
                            round(x, 3)
                            if math.isfinite(x)
                            else None
                            for x in motor_torque
                        ],
                    )
                )

                next_static_report = (
                    loop_start
                    + STATIC_REPORT_PERIOD
                )

            work_end = time.monotonic()
            deadline_late = max(
                0.0,
                work_end - next_tick,
            )

            # ------------------------------------------------
            # Logging
            # ------------------------------------------------
            if logger is not None:
                row = {
                    "time": loop_start - start,
                    "loop_dt": loop_dt,
                    "leader_new": leader_new,
                    "leader_seq": state["seq"],
                    "leader_quiet": state["quiet"],
                    "leader_sample_dt": leader_sample_dt,
                    "leader_skew_ms": state["skew"] * 1000.0,
                    "leader_bus_age": state["bus_age"],
                    "mit_send_ms": (send_end - send_start) * 1000.0,
                    "feedback_read_ms": (
                        feedback_end - feedback_start
                    ) * 1000.0,
                    "loop_work_ms": (
                        work_end - loop_start
                    ) * 1000.0,
                    "deadline_late_ms": deadline_late * 1000.0,
                    "motor_read_ms": motor_read_ms,
                    "static_hold": int(static_hold),
                    "static_hold_time": static_hold_time,
                }

                vectors = (
                    ("master", q_master),
                    ("master_delta", master_delta),
                    ("dq_raw", dq_raw),
                    ("dq_filt", dq_filt),
                    ("v_des", v_des),
                    ("target", q_target),
                    ("slave", q_slave),
                    ("slave_delta", slave_delta),
                    ("tracking_error", tracking_error),
                    ("motion_error", motion_error),
                    ("absolute_gap", absolute_gap),
                    ("motor_q", motor_q),
                    ("motor_dq", motor_dq),
                    ("motor_current", motor_current),
                    ("motor_torque", motor_torque),
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

            # ------------------------------------------------
            # Console status
            # ------------------------------------------------
            if loop_start >= next_status:
                max_tracking = max(
                    abs(x)
                    for x in tracking_error
                )

                quiet_ms = state["quiet"] * 1000.0
                bus_ms = state["bus_age"] * 1000.0

                print(
                    "loop={:5.2f}ms  work={:5.2f}ms  "
                    "send={:5.2f}ms  motor={:5.2f}ms  "
                    "quiet={:7.1f}ms  bus={:6.1f}ms  new={}  "
                    "track_max={:6.3f}rad  J4={:+6.3f}  J5={:+6.3f}"
                    .format(
                        loop_dt * 1000.0,
                        (work_end - loop_start) * 1000.0,
                        (send_end - send_start) * 1000.0,
                        motor_read_ms,
                        quiet_ms,
                        bus_ms,
                        leader_new,
                        max_tracking,
                        tracking_error[3],
                        tracking_error[4],
                    )
                )

                next_status = loop_start + STATUS_PERIOD

            # Avoid catch-up bursts if a deadline is ever missed.
            if deadline_late > 0.0:
                next_tick = work_end

    except KeyboardInterrupt:
        print("\nSTOP")

    except Exception as exc:
        print("\n[FAULT]", exc)

    finally:
        print("Shutdown")

        if args.live and right is not None:
            safe_hold_and_disable(right)

        if logger is not None:
            logger.close()
            if logger.dropped:
                print(
                    "[WARN] logger dropped {} rows"
                    .format(logger.dropped)
                )

        if right is not None:
            try:
                right.disconnect()
            except Exception as exc:
                print("[WARN] RIGHT disconnect:", exc)

        if left is not None:
            try:
                left.disconnect()
            except Exception as exc:
                print("[WARN] LEFT disconnect:", exc)

        if leader is not None:
            leader.stop()


if __name__ == "__main__":
    main()
