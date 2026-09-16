#!/usr/bin/env python3

import argparse
import csv
import math
import os
import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

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

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)

CAN_ID_J12 = 0x155
CAN_ID_J34 = 0x156
CAN_ID_J56 = 0x157
CAN_ID_GRIPPER = 0x159

DEG_TO_RAD = math.pi / 180.0


# Piper joint limits, rad
JOINT_LIMITS = [
    (-2.617994, 2.617994),   # J1
    (0.0,       3.141593),   # J2
    (-2.967060, 0.0),        # J3
    (-1.745330, 1.745330),   # J4
    (-1.221730, 1.221730),   # J5
    (-2.094396, 2.094396),   # J6
]


# Same-model Piper -> Piper
JOINT_DIRECTION = [1.0] * 6
JOINT_SCALE = [1.0] * 6
GRIPPER_SCALE = 1.0


# ============================================================
# State structures
# ============================================================

@dataclass
class LeaderState:
    q: Optional[List[float]]
    gripper_m: Optional[float]

    joint_age: float
    gripper_age: float


@dataclass
class MotorStateVector:
    q: List[float]
    dq: List[float]
    current: List[float]
    torque: List[float]

    timestamp_min: float
    timestamp_max: float
    timestamp_spread: float

    hz_min: float


# ============================================================
# Native SocketCAN Leader Reader
# ============================================================

class PiperLeaderReader:

    def __init__(self, channel: str):

        self.channel = channel

        self.sock = None
        self.thread = None
        self.running = False

        self.lock = threading.Lock()

        self.q = [None] * 6

        # timestamps for:
        # 0x155 / 0x156 / 0x157
        self.joint_pair_stamp = [
            None,
            None,
            None,
        ]

        self.gripper_m = None
        self.gripper_stamp = None

    @staticmethod
    def decode_joint(raw4: bytes) -> float:

        value_mdeg = int.from_bytes(
            raw4,
            byteorder="big",
            signed=True,
        )

        return (
            value_mdeg
            * 0.001
            * DEG_TO_RAD
        )

    @staticmethod
    def decode_gripper(data: bytes) -> float:

        raw = int.from_bytes(
            data[0:4],
            byteorder="big",
            signed=True,
        )

        # 0.001 mm -> 1e-6 m
        return raw * 1e-6

    def start(self):

        self.sock = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )

        self.sock.bind(
            (self.channel,)
        )

        self.sock.settimeout(
            0.1
        )

        self.running = True

        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
        )

        self.thread.start()

    def _run(self):

        while self.running:

            try:

                frame = self.sock.recv(
                    CAN_FRAME_SIZE
                )

            except socket.timeout:

                continue

            except OSError:

                break

            if len(frame) != CAN_FRAME_SIZE:
                continue

            can_id, dlc, data = struct.unpack(
                CAN_FRAME_FORMAT,
                frame,
            )

            can_id &= 0x1FFFFFFF

            now = time.monotonic()

            with self.lock:

                if (
                    can_id == CAN_ID_J12
                    and dlc >= 8
                ):

                    self.q[0] = self.decode_joint(
                        data[0:4]
                    )

                    self.q[1] = self.decode_joint(
                        data[4:8]
                    )

                    self.joint_pair_stamp[0] = now

                elif (
                    can_id == CAN_ID_J34
                    and dlc >= 8
                ):

                    self.q[2] = self.decode_joint(
                        data[0:4]
                    )

                    self.q[3] = self.decode_joint(
                        data[4:8]
                    )

                    self.joint_pair_stamp[1] = now

                elif (
                    can_id == CAN_ID_J56
                    and dlc >= 8
                ):

                    self.q[4] = self.decode_joint(
                        data[0:4]
                    )

                    self.q[5] = self.decode_joint(
                        data[4:8]
                    )

                    self.joint_pair_stamp[2] = now

                elif (
                    can_id == CAN_ID_GRIPPER
                    and dlc >= 8
                ):

                    self.gripper_m = (
                        self.decode_gripper(
                            data
                        )
                    )

                    self.gripper_stamp = now

    def get_state(self) -> LeaderState:

        now = time.monotonic()

        with self.lock:

            joints_valid = (
                all(
                    v is not None
                    for v in self.q
                )
                and
                all(
                    t is not None
                    for t in self.joint_pair_stamp
                )
            )

            if joints_valid:

                q = list(
                    self.q
                )

                joint_age = (
                    now
                    - min(
                        self.joint_pair_stamp
                    )
                )

            else:

                q = None
                joint_age = float("inf")

            if (
                self.gripper_m is not None
                and
                self.gripper_stamp is not None
            ):

                gripper_m = float(
                    self.gripper_m
                )

                gripper_age = (
                    now
                    - self.gripper_stamp
                )

            else:

                gripper_m = None
                gripper_age = float("inf")

        return LeaderState(
            q=q,
            gripper_m=gripper_m,
            joint_age=joint_age,
            gripper_age=gripper_age,
        )

    def stop(self):

        self.running = False

        if self.thread is not None:

            self.thread.join(
                timeout=1.0
            )

        if self.sock is not None:

            try:
                self.sock.close()

            except Exception:
                pass


# ============================================================
# Async CSV Logger
# ============================================================

class AsyncCSVLogger:

    def __init__(
        self,
        path: str,
        fieldnames: List[str],
    ):

        self.path = path
        self.fieldnames = fieldnames

        self.queue = queue.Queue(
            maxsize=20000
        )

        self.thread = None
        self.running = False

        self.rows_written = 0
        self.rows_dropped = 0

    def start(self):

        os.makedirs(
            os.path.dirname(
                os.path.abspath(
                    self.path
                )
            ),
            exist_ok=True,
        )

        self.running = True

        self.thread = threading.Thread(
            target=self._worker,
            daemon=True,
        )

        self.thread.start()

    def log(self, row):

        try:

            self.queue.put_nowait(
                row
            )

        except queue.Full:

            self.rows_dropped += 1

    def _worker(self):

        with open(
            self.path,
            "w",
            newline="",
        ) as f:

            writer = csv.DictWriter(
                f,
                fieldnames=self.fieldnames,
            )

            writer.writeheader()

            pending = 0

            while (
                self.running
                or
                not self.queue.empty()
            ):

                try:

                    row = self.queue.get(
                        timeout=0.1
                    )

                except queue.Empty:

                    continue

                writer.writerow(
                    row
                )

                self.rows_written += 1
                pending += 1

                if pending >= 100:

                    f.flush()
                    pending = 0

            f.flush()

    def stop(self):

        self.running = False

        if self.thread is not None:

            self.thread.join(
                timeout=5.0
            )


# ============================================================
# pyAgxArm helpers
# ============================================================

def create_piper(channel: str):

    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )

    return AgxArmFactory.create_arm(
        cfg
    )


def wait_joint_feedback(
    arm,
    timeout=3.0,
):

    deadline = (
        time.monotonic()
        + timeout
    )

    while time.monotonic() < deadline:

        state = (
            arm.get_joint_angles()
        )

        if state is not None:

            return list(
                state.msg
            )

        time.sleep(
            0.02
        )

    return None


def wait_gripper_feedback(
    gripper,
    timeout=3.0,
):

    deadline = (
        time.monotonic()
        + timeout
    )

    while time.monotonic() < deadline:

        state = (
            gripper.get_gripper_status()
        )

        if state is not None:

            return state

        time.sleep(
            0.02
        )

    return None


def wait_leader_joints(
    reader,
    timeout=4.0,
    max_age=0.1,
):

    deadline = (
        time.monotonic()
        + timeout
    )

    last_state = None

    while time.monotonic() < deadline:

        state = (
            reader.get_state()
        )

        last_state = state

        if (
            state.q is not None
            and
            state.joint_age <= max_age
        ):

            return state

        time.sleep(
            0.01
        )

    return last_state


def ensure_enabled(
    arm,
    timeout=3.0,
):

    deadline = (
        time.monotonic()
        + timeout
    )

    while time.monotonic() < deadline:

        states = (
            arm
            .get_joints_enable_status_list()
        )

        if (
            len(states) == 6
            and
            all(states)
        ):

            return True

        arm.enable()

        time.sleep(
            0.05
        )

    states = (
        arm
        .get_joints_enable_status_list()
    )

    return (
        len(states) == 6
        and
        all(states)
    )


def get_motor_state_vector(
    arm,
) -> Optional[MotorStateVector]:

    q = []
    dq = []
    current = []
    torque = []

    timestamps = []
    frequencies = []

    for joint in range(1, 7):

        state = (
            arm.get_motor_states(
                joint
            )
        )

        if state is None:
            return None

        msg = state.msg

        q.append(
            float(msg.position)
        )

        dq.append(
            float(msg.velocity)
        )

        current.append(
            float(msg.current)
        )

        torque.append(
            float(msg.torque)
        )

        timestamps.append(
            float(state.timestamp)
        )

        frequencies.append(
            float(state.hz)
        )

    timestamp_min = min(
        timestamps
    )

    timestamp_max = max(
        timestamps
    )

    return MotorStateVector(
        q=q,
        dq=dq,
        current=current,
        torque=torque,

        timestamp_min=timestamp_min,
        timestamp_max=timestamp_max,

        timestamp_spread=(
            timestamp_max
            - timestamp_min
        ),

        hz_min=min(
            frequencies
        ),
    )


# ============================================================
# Mapping / safety
# ============================================================

def clamp(
    value,
    low,
    high,
):

    return min(
        max(value, low),
        high,
    )


def clamp_joints(q):

    return [
        clamp(
            value,
            low,
            high,
        )

        for value, (low, high)
        in zip(
            q,
            JOINT_LIMITS,
        )
    ]


def joint_slew_limit(
    previous,
    target,
    max_delta,
):

    output = []

    for prev, tgt in zip(
        previous,
        target,
    ):

        delta = clamp(
            tgt - prev,
            -max_delta,
            max_delta,
        )

        output.append(
            prev + delta
        )

    return output


def scalar_slew_limit(
    previous,
    target,
    max_delta,
):

    delta = clamp(
        target - previous,
        -max_delta,
        max_delta,
    )

    return (
        previous
        + delta
    )


# ============================================================
# CSV schema
# ============================================================

def build_csv_fields():

    fields = [
        "sample_index",
        "time",
        "unix_time",

        "loop_dt",
        "loop_hz",

        "leader_joint_age",
        "leader_gripper_age",
        "leader_gripper_valid",

        "motor_timestamp_min",
        "motor_timestamp_max",
        "motor_timestamp_spread",

        "motor_hz_min",
    ]

    for prefix in (
        "q_master",
        "q_target",
        "q_cmd",
        "q_actual",
        "dq",
        "delta_qd",
        "current",
        "tau_motor",
    ):

        for j in range(1, 7):

            fields.append(
                f"{prefix}_{j}"
            )

    fields += [
        "gripper_master",
        "gripper_target",
        "gripper_cmd",
        "gripper_actual",
        "gripper_force_actual",
    ]

    return fields


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Piper Phase 1C: "
            "software teleoperation + NEXT logger"
        )
    )

    parser.add_argument(
        "--live",
        action="store_true",
    )

    parser.add_argument(
        "--record",
        action="store_true",
    )

    parser.add_argument(
        "--output",
        type=str,
        default=None,
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help=(
            "Auto-stop after N seconds; "
            "0 = until Ctrl+C"
        ),
    )

    parser.add_argument(
        "--max-joint-speed",
        type=float,
        default=0.18,
    )

    parser.add_argument(
        "--max-gripper-speed",
        type=float,
        default=0.030,
    )

    parser.add_argument(
        "--arm-speed-percent",
        type=int,
        default=20,
    )

    parser.add_argument(
        "--gripper-force",
        type=float,
        default=1.0,
    )

    parser.add_argument(
        "--gripper-max",
        type=float,
        default=0.070,
    )

    parser.add_argument(
        "--joint-watchdog",
        type=float,
        default=0.100,
    )

    parser.add_argument(
        "--gripper-watchdog",
        type=float,
        default=0.300,
    )

    args = parser.parse_args()

    if args.rate <= 0:

        raise ValueError(
            "--rate must be > 0"
        )

    if (
        args.record
        and
        not args.live
    ):

        raise RuntimeError(
            "--record requires --live"
        )

    # --------------------------------------------------------
    # Output
    # --------------------------------------------------------

    output_path = None

    if args.record:

        if args.output is None:

            stamp = time.strftime(
                "%Y%m%d_%H%M%S"
            )

            output_path = (
                "data/next/"
                f"free_motion_{stamp}.csv"
            )

        else:

            output_path = args.output

    # --------------------------------------------------------
    # Config display
    # --------------------------------------------------------

    print("=" * 78)

    print(
        "Piper Phase 1C — "
        "Teleoperation + NEXT Logger"
    )

    print("=" * 78)

    print(
        f"Mode              : "
        f"{'LIVE' if args.live else 'DRY RUN'}"
    )

    print(
        f"Control rate      : "
        f"{args.rate:.1f} Hz"
    )

    print(
        f"Record            : "
        f"{args.record}"
    )

    if output_path:

        print(
            f"Output            : "
            f"{output_path}"
        )

    print(
        f"Joint watchdog    : "
        f"{args.joint_watchdog * 1000:.0f} ms"
    )

    print(
        f"Gripper watchdog  : "
        f"{args.gripper_watchdog * 1000:.0f} ms"
    )

    print()

    # --------------------------------------------------------
    # Objects
    # --------------------------------------------------------

    left_arm = None

    right_arm = None
    right_gripper = None

    leader_reader = None

    logger = None

    # --------------------------------------------------------
    # Statistics
    # --------------------------------------------------------

    sample_index = 0

    watchdog_count = 0
    motor_missing_count = 0

    loop_dt_min = float("inf")
    loop_dt_max = 0.0
    loop_dt_sum = 0.0
    loop_dt_count = 0

    # --------------------------------------------------------
    # Run
    # --------------------------------------------------------

    try:

        # ====================================================
        # RIGHT Piper
        # ====================================================

        print(
            "[1] Connecting RIGHT Piper..."
        )

        right_arm = create_piper(
            RIGHT_CAN
        )

        right_gripper = (
            right_arm.init_effector(
                right_arm
                .OPTIONS
                .EFFECTOR
                .AGX_GRIPPER
            )
        )

        right_arm.connect()

        time.sleep(
            1.0
        )

        if not right_arm.is_ok():

            raise RuntimeError(
                "RIGHT Piper communication "
                "is not OK."
            )

        q_slave_0 = (
            wait_joint_feedback(
                right_arm
            )
        )

        if q_slave_0 is None:

            raise RuntimeError(
                "Unable to read "
                "RIGHT joint feedback."
            )

        gripper_fb = (
            wait_gripper_feedback(
                right_gripper
            )
        )

        if gripper_fb is None:

            raise RuntimeError(
                "Unable to read "
                "RIGHT gripper feedback."
            )

        if (
            gripper_fb.msg.mode
            != "width"
        ):

            raise RuntimeError(
                "RIGHT gripper is not "
                "in width mode."
            )

        gripper_slave_0 = float(
            gripper_fb.msg.value
        )

        print(
            "RIGHT q0 = ["
            + ", ".join(
                f"{x:+.4f}"
                for x in q_slave_0
            )
            + "]"
        )

        print(
            f"RIGHT gripper0 = "
            f"{gripper_slave_0 * 1000:.2f} mm"
        )

        # ====================================================
        # LEFT raw SocketCAN
        # ====================================================

        print()

        print(
            "[2] Starting LEFT "
            "raw SocketCAN reader..."
        )

        leader_reader = (
            PiperLeaderReader(
                LEFT_CAN
            )
        )

        leader_reader.start()

        # ====================================================
        # LEFT pyAgxArm:
        # mode control only
        # ====================================================

        left_arm = create_piper(
            LEFT_CAN
        )

        left_arm.connect()

        time.sleep(
            0.3
        )

        print()

        print(
            "[WARNING] 左臂即将进入 "
            "Leader / zero-force 模式。"
        )

        print(
            "请扶住左臂。"
        )

        input(
            "准备好后按 Enter..."
        )

        left_arm.set_leader_mode()

        # ====================================================
        # Wait ONLY for joints
        # ====================================================

        leader_initial = (
            wait_leader_joints(
                leader_reader,
                timeout=4.0,
                max_age=args.joint_watchdog,
            )
        )

        if (
            leader_initial is None
            or
            leader_initial.q is None
            or
            leader_initial.joint_age
            > args.joint_watchdog
        ):

            if leader_initial is None:

                age_ms = float("inf")

            else:

                age_ms = (
                    leader_initial.joint_age
                    * 1000.0
                )

            raise RuntimeError(
                "No fresh Leader joint state. "
                f"joint_age={age_ms:.1f} ms"
            )

        q_master_0 = list(
            leader_initial.q
        )

        print(
            "LEFT q0 = ["
            + ", ".join(
                f"{x:+.4f}"
                for x in q_master_0
            )
            + "]"
        )

        # ====================================================
        # Gripper is NOT startup-critical
        # ====================================================

        gripper_master_0 = None

        if (
            leader_initial.gripper_m
            is not None
            and
            leader_initial.gripper_age
            <= args.gripper_watchdog
        ):

            gripper_master_0 = float(
                leader_initial.gripper_m
            )

            print(
                f"LEFT gripper0 = "
                f"{gripper_master_0 * 1000:.2f} mm"
            )

        else:

            print(
                "[INFO] No fresh LEFT gripper "
                "frame at startup."
            )

            print(
                "[INFO] Arm teleoperation will "
                "start normally."
            )

            print(
                "[INFO] Gripper will activate "
                "when first fresh 0x159 arrives."
            )

        # ====================================================
        # Logger
        # ====================================================

        if args.record:

            logger = AsyncCSVLogger(
                output_path,
                build_csv_fields(),
            )

            logger.start()

            print()

            print(
                f"[LOGGER] Ready: "
                f"{output_path}"
            )

        # ====================================================
        # Prepare RIGHT
        # ====================================================

        if args.live:

            print()

            print(
                "[3] Preparing RIGHT Piper..."
            )

            if not ensure_enabled(
                right_arm
            ):

                raise RuntimeError(
                    "Unable to enable "
                    "all RIGHT joints."
                )

            right_arm.set_speed_percent(
                args.arm_speed_percent
            )

            right_arm.set_motion_mode(
                "j"
            )

            # First command = actual pose
            right_arm.move_j(
                list(q_slave_0)
            )

            right_gripper.move_gripper_m(
                value=gripper_slave_0,
                force=args.gripper_force,
            )

            if args.record:

                print(
                    "本次训练数据要求右臂保持"
                    "自由空间，不要接触环境。"
                )

            input(
                "按 Enter 开始..."
            )

        else:

            print()

            print(
                "DRY RUN: "
                "RIGHT arm will not move."
            )

        # ====================================================
        # Control loop
        # ====================================================

        period = (
            1.0 / args.rate
        )

        previous_q_cmd = list(
            q_slave_0
        )

        previous_gripper_cmd = (
            gripper_slave_0
        )

        gripper_target = (
            gripper_slave_0
        )

        gripper_cmd = (
            gripper_slave_0
        )

        last_gripper_send = 0.0

        experiment_start = (
            time.monotonic()
        )

        previous_loop_time = (
            experiment_start
        )

        next_tick = (
            experiment_start
        )

        last_console_time = 0.0

        watchdog_active = False

        print()

        print(
            "Teleoperation running. "
            "Ctrl+C to stop."
        )

        print()

        while True:

            loop_start = (
                time.monotonic()
            )

            elapsed = (
                loop_start
                - experiment_start
            )

            if (
                args.duration > 0
                and
                elapsed >= args.duration
            ):

                print(
                    f"[INFO] Duration "
                    f"{args.duration:.1f}s reached."
                )

                break

            dt = (
                loop_start
                - previous_loop_time
            )

            previous_loop_time = (
                loop_start
            )

            if dt > 0:

                loop_dt_min = min(
                    loop_dt_min,
                    dt,
                )

                loop_dt_max = max(
                    loop_dt_max,
                    dt,
                )

                loop_dt_sum += dt
                loop_dt_count += 1

            # --------------------------------------------
            # Leader
            # --------------------------------------------

            leader = (
                leader_reader
                .get_state()
            )

            # ONLY joints determine arm watchdog
            joints_valid = (
                leader.q is not None
                and
                leader.joint_age
                <= args.joint_watchdog
            )

            if not joints_valid:

                if not watchdog_active:

                    print(
                        "[WATCHDOG] "
                        "Leader joint data stale — "
                        "holding RIGHT arm target."
                    )

                    watchdog_active = True
                    watchdog_count += 1

                next_tick += period

                sleep_time = (
                    next_tick
                    - time.monotonic()
                )

                if sleep_time > 0:

                    time.sleep(
                        sleep_time
                    )

                else:

                    next_tick = (
                        time.monotonic()
                    )

                continue

            if watchdog_active:

                print(
                    "[WATCHDOG] "
                    "Leader joint data recovered."
                )

                watchdog_active = False

            # --------------------------------------------
            # Joint mapping
            # --------------------------------------------

            q_target = []

            for i in range(6):

                delta = (
                    leader.q[i]
                    - q_master_0[i]
                )

                target = (
                    q_slave_0[i]
                    +
                    JOINT_DIRECTION[i]
                    *
                    JOINT_SCALE[i]
                    *
                    delta
                )

                q_target.append(
                    target
                )

            q_target = (
                clamp_joints(
                    q_target
                )
            )

            max_joint_delta = (
                args.max_joint_speed
                *
                max(
                    dt,
                    1e-4,
                )
            )

            q_cmd = (
                joint_slew_limit(
                    previous_q_cmd,
                    q_target,
                    max_joint_delta,
                )
            )

            # --------------------------------------------
            # Gripper:
            # completely independent channel
            # --------------------------------------------

            gripper_fresh = (
                leader.gripper_m
                is not None
                and
                leader.gripper_age
                <= args.gripper_watchdog
            )

            if gripper_fresh:

                if gripper_master_0 is None:

                    # First valid 0x159 establishes
                    # the master gripper zero/reference.
                    gripper_master_0 = float(
                        leader.gripper_m
                    )

                    print(
                        "[GRIPPER] Leader gripper "
                        "armed at "
                        f"{gripper_master_0 * 1000:.2f} mm"
                    )

                gripper_target = (
                    gripper_slave_0
                    +
                    GRIPPER_SCALE
                    *
                    (
                        leader.gripper_m
                        -
                        gripper_master_0
                    )
                )

                gripper_target = clamp(
                    gripper_target,
                    0.0,
                    args.gripper_max,
                )

                max_gripper_delta = (
                    args.max_gripper_speed
                    *
                    max(
                        dt,
                        1e-4,
                    )
                )

                gripper_cmd = (
                    scalar_slew_limit(
                        previous_gripper_cmd,
                        gripper_target,
                        max_gripper_delta,
                    )
                )

            else:

                # No fresh 0x159:
                # hold gripper only.
                # Arm continues normally.
                gripper_cmd = (
                    previous_gripper_cmd
                )

            # --------------------------------------------
            # Commands
            # --------------------------------------------

            if args.live:

                right_arm.move_j(
                    list(q_cmd)
                )

                # gripper ~20 Hz
                if (
                    gripper_fresh
                    and
                    loop_start
                    - last_gripper_send
                    >= 0.05
                ):

                    right_gripper.move_gripper_m(
                        value=gripper_cmd,
                        force=args.gripper_force,
                    )

                    last_gripper_send = (
                        loop_start
                    )

            # --------------------------------------------
            # Motor feedback
            # --------------------------------------------

            motor = (
                get_motor_state_vector(
                    right_arm
                )
            )

            if motor is None:

                motor_missing_count += 1

            # --------------------------------------------
            # Right gripper feedback
            # --------------------------------------------

            gripper_actual = None
            gripper_force_actual = None

            gripper_state = (
                right_gripper
                .get_gripper_status()
            )

            if (
                gripper_state is not None
                and
                gripper_state.msg.mode
                == "width"
            ):

                gripper_actual = float(
                    gripper_state
                    .msg
                    .value
                )

                gripper_force_actual = float(
                    gripper_state
                    .msg
                    .force
                )

            # --------------------------------------------
            # CSV
            # --------------------------------------------

            if (
                logger is not None
                and
                motor is not None
            ):

                loop_hz = (
                    1.0 / dt
                    if dt > 0
                    else float("nan")
                )

                # THIS is NEXT's commanded
                # tracking error.
                delta_qd = [
                    cmd - actual
                    for cmd, actual
                    in zip(
                        q_cmd,
                        motor.q,
                    )
                ]

                row = {
                    "sample_index":
                        sample_index,

                    "time":
                        elapsed,

                    "unix_time":
                        time.time(),

                    "loop_dt":
                        dt,

                    "loop_hz":
                        loop_hz,

                    "leader_joint_age":
                        leader.joint_age,

                    "leader_gripper_age":
                        (
                            leader.gripper_age
                            if math.isfinite(
                                leader.gripper_age
                            )
                            else ""
                        ),

                    "leader_gripper_valid":
                        int(
                            gripper_fresh
                        ),

                    "motor_timestamp_min":
                        motor.timestamp_min,

                    "motor_timestamp_max":
                        motor.timestamp_max,

                    "motor_timestamp_spread":
                        motor.timestamp_spread,

                    "motor_hz_min":
                        motor.hz_min,

                    "gripper_master":
                        (
                            leader.gripper_m
                            if leader.gripper_m
                            is not None
                            else ""
                        ),

                    "gripper_target":
                        gripper_target,

                    "gripper_cmd":
                        gripper_cmd,

                    "gripper_actual":
                        (
                            gripper_actual
                            if gripper_actual
                            is not None
                            else ""
                        ),

                    "gripper_force_actual":
                        (
                            gripper_force_actual
                            if gripper_force_actual
                            is not None
                            else ""
                        ),
                }

                for i in range(6):

                    j = i + 1

                    row[
                        f"q_master_{j}"
                    ] = leader.q[i]

                    row[
                        f"q_target_{j}"
                    ] = q_target[i]

                    row[
                        f"q_cmd_{j}"
                    ] = q_cmd[i]

                    row[
                        f"q_actual_{j}"
                    ] = motor.q[i]

                    row[
                        f"dq_{j}"
                    ] = motor.dq[i]

                    row[
                        f"delta_qd_{j}"
                    ] = delta_qd[i]

                    row[
                        f"current_{j}"
                    ] = motor.current[i]

                    row[
                        f"tau_motor_{j}"
                    ] = motor.torque[i]

                logger.log(
                    row
                )

                sample_index += 1

            # --------------------------------------------
            # Console ~2 Hz
            # --------------------------------------------

            if (
                loop_start
                - last_console_time
                >= 0.5
            ):

                if motor is not None:

                    tracking_error = max(
                        abs(
                            cmd - actual
                        )

                        for cmd, actual
                        in zip(
                            q_cmd,
                            motor.q,
                        )
                    )

                    spread_ms = (
                        motor
                        .timestamp_spread
                        * 1000.0
                    )

                else:

                    tracking_error = (
                        float("nan")
                    )

                    spread_ms = (
                        float("nan")
                    )

                average_hz = (
                    loop_dt_count
                    / loop_dt_sum
                    if loop_dt_sum > 0
                    else float("nan")
                )

                if (
                    gripper_master_0
                    is not None
                ):

                    gripper_text = (
                        f"{gripper_cmd * 1000:5.1f}mm"
                    )

                else:

                    gripper_text = "WAIT"

                print(
                    f"[{sample_index:06d}] "
                    f"loop={average_hz:6.1f}Hz  "
                    f"leader="
                    f"{leader.joint_age * 1000:4.1f}ms  "
                    f"motor_spread="
                    f"{spread_ms:4.1f}ms  "
                    f"track_err="
                    f"{tracking_error:.3f}rad  "
                    f"gripper="
                    f"{gripper_text}"
                )

                last_console_time = (
                    loop_start
                )

            previous_q_cmd = list(
                q_cmd
            )

            previous_gripper_cmd = (
                gripper_cmd
            )

            # --------------------------------------------
            # Fixed-rate timing
            # --------------------------------------------

            next_tick += period

            sleep_time = (
                next_tick
                - time.monotonic()
            )

            if sleep_time > 0:

                time.sleep(
                    sleep_time
                )

            else:

                next_tick = (
                    time.monotonic()
                )

    except KeyboardInterrupt:

        print()

        print(
            "[INFO] Stopped by user."
        )

    finally:

        if logger is not None:

            logger.stop()

        if leader_reader is not None:

            leader_reader.stop()

        if left_arm is not None:

            try:
                left_arm.disconnect()

            except Exception:
                pass

        if right_arm is not None:

            try:
                right_arm.disconnect()

            except Exception:
                pass

        print()

        print("=" * 78)

        print(
            "Phase 1C summary"
        )

        print("=" * 78)

        if loop_dt_count > 0:

            avg_dt = (
                loop_dt_sum
                / loop_dt_count
            )

            print(
                f"Average loop rate : "
                f"{1.0 / avg_dt:.2f} Hz"
            )

            print(
                f"Min loop dt       : "
                f"{loop_dt_min * 1000:.3f} ms"
            )

            print(
                f"Max loop dt       : "
                f"{loop_dt_max * 1000:.3f} ms"
            )

        print(
            f"Watchdog events   : "
            f"{watchdog_count}"
        )

        print(
            f"Missing motor state: "
            f"{motor_missing_count}"
        )

        if logger is not None:

            print(
                f"Rows written      : "
                f"{logger.rows_written}"
            )

            print(
                f"Rows dropped      : "
                f"{logger.rows_dropped}"
            )

            print(
                f"CSV               : "
                f"{logger.path}"
            )

        print()

        print(
            "[INFO] No motor-disable "
            "command was sent."
        )

        print(
            "[INFO] LEFT remains "
            "in Leader role."
        )


if __name__ == "__main__":
    main()