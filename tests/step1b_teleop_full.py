#!/usr/bin/env python3

import argparse
import math
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
# Basic configuration
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


# Official Piper joint limits, rad
# pyAgxArm constants.py
JOINT_LIMITS = [
    (-2.617994, 2.617994),  # J1 -150 ~ +150 deg
    (0.0,       3.141593),  # J2    0 ~ +180 deg
    (-2.967060, 0.0),       # J3 -170 ~    0 deg
    (-1.745330, 1.745330),  # J4 -100 ~ +100 deg
    (-1.221730, 1.221730),  # J5  -70 ~  +70 deg
    (-2.094396, 2.094396),  # J6 -120 ~ +120 deg
]


# Same-model Piper -> Piper mapping
JOINT_DIRECTION = [1.0] * 6
JOINT_SCALE = [1.0] * 6

GRIPPER_SCALE = 1.0


# ============================================================
# Leader state
# ============================================================

@dataclass
class LeaderState:
    q: Optional[List[float]]
    gripper_m: Optional[float]

    joint_age: float
    gripper_age: float


# ============================================================
# Leader raw SocketCAN reader
# ============================================================

class PiperLeaderReader:

    def __init__(self, channel: str):
        self.channel = channel

        self.sock = None
        self.thread = None
        self.running = False

        self.lock = threading.Lock()

        self.q = [None] * 6

        # Individual timestamps for:
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

        # raw unit:
        # 0.001 mm = 1e-6 m
        return raw * 1e-6

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
                        self.decode_gripper(data)
                    )

                    self.gripper_stamp = now

    def get_state(self) -> LeaderState:

        now = time.monotonic()

        with self.lock:

            joints_valid = (
                all(v is not None for v in self.q)
                and
                all(
                    t is not None
                    for t in self.joint_pair_stamp
                )
            )

            if joints_valid:

                # Oldest of the three joint frame groups.
                oldest_joint_stamp = min(
                    self.joint_pair_stamp
                )

                joint_age = (
                    now - oldest_joint_stamp
                )

                q = list(self.q)

            else:

                joint_age = float("inf")
                q = None

            if (
                self.gripper_m is not None
                and
                self.gripper_stamp is not None
            ):

                gripper_m = self.gripper_m

                gripper_age = (
                    now - self.gripper_stamp
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
# pyAgxArm helpers
# ============================================================

def create_piper(channel: str):

    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )

    return AgxArmFactory.create_arm(cfg)


def wait_joint_feedback(
    arm,
    timeout=3.0,
):

    deadline = (
        time.monotonic() + timeout
    )

    while time.monotonic() < deadline:

        state = arm.get_joint_angles()

        if state is not None:
            return list(state.msg)

        time.sleep(0.02)

    return None


def wait_gripper_feedback(
    gripper,
    timeout=3.0,
):

    deadline = (
        time.monotonic() + timeout
    )

    while time.monotonic() < deadline:

        state = gripper.get_gripper_status()

        if state is not None:
            return state

        time.sleep(0.02)

    return None


def wait_leader_state(
    reader,
    timeout=3.0,
):

    deadline = (
        time.monotonic() + timeout
    )

    while time.monotonic() < deadline:

        state = reader.get_state()

        if (
            state.q is not None
            and
            state.gripper_m is not None
            and
            state.joint_age < 0.1
            and
            state.gripper_age < 0.1
        ):

            return state

        time.sleep(0.01)

    return None


def ensure_enabled(
    arm,
    timeout=3.0,
):

    deadline = (
        time.monotonic() + timeout
    )

    while time.monotonic() < deadline:

        states = (
            arm.get_joints_enable_status_list()
        )

        if (
            len(states) == 6
            and all(states)
        ):
            return True

        arm.enable()

        time.sleep(0.05)

    return all(
        arm.get_joints_enable_status_list()
    )


# ============================================================
# Mapping and safety
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

    output = []

    for value, limits in zip(
        q,
        JOINT_LIMITS,
    ):

        output.append(
            clamp(
                value,
                limits[0],
                limits[1],
            )
        )

    return output


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

        delta = tgt - prev

        delta = clamp(
            delta,
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

    delta = target - previous

    delta = clamp(
        delta,
        -max_delta,
        max_delta,
    )

    return previous + delta


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--live",
        action="store_true",
        help=(
            "Actually command the RIGHT Piper. "
            "Without this flag the program is dry-run only."
        ),
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Teleoperation loop rate Hz",
    )

    parser.add_argument(
        "--max-joint-speed",
        type=float,
        default=0.12,
        help=(
            "Software joint velocity limit rad/s. "
            "Default = 0.12 rad/s (~6.9 deg/s)"
        ),
    )

    parser.add_argument(
        "--max-gripper-speed",
        type=float,
        default=0.020,
        help=(
            "Software gripper speed limit m/s. "
            "Default = 0.020 m/s"
        ),
    )

    parser.add_argument(
        "--arm-speed-percent",
        type=int,
        default=15,
        help=(
            "Piper MOVE_J speed percentage. "
            "Default = 15"
        ),
    )

    parser.add_argument(
        "--gripper-force",
        type=float,
        default=1.0,
        help=(
            "Right gripper force in N. "
            "Default = 1.0 N"
        ),
    )

    parser.add_argument(
        "--gripper-max",
        type=float,
        default=0.070,
        help=(
            "Maximum commanded gripper width m. "
            "Default = 0.070 m"
        ),
    )

    parser.add_argument(
        "--watchdog",
        type=float,
        default=0.100,
        help=(
            "Maximum Leader data age seconds. "
            "Default = 0.100"
        ),
    )

    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("--rate must be > 0")

    mode_name = (
        "LIVE"
        if args.live
        else "DRY RUN"
    )

    print("=" * 78)
    print(
        f"Piper Software Teleoperation — {mode_name}"
    )
    print("=" * 78)

    print(
        f"Leader  : {LEFT_CAN}"
    )

    print(
        f"Follower: {RIGHT_CAN}"
    )

    print(
        f"Loop rate: "
        f"{args.rate:.1f} Hz"
    )

    print(
        f"Max joint speed: "
        f"{args.max_joint_speed:.3f} rad/s"
    )

    print(
        f"Max gripper speed: "
        f"{args.max_gripper_speed:.3f} m/s"
    )

    print()

    left_arm = None
    right_arm = None
    right_gripper = None
    leader_reader = None

    try:

        # ====================================================
        # 1. Connect RIGHT Piper first
        # ====================================================

        print("[1] Creating RIGHT Piper...")

        right_arm = create_piper(
            RIGHT_CAN
        )

        right_gripper = (
            right_arm.init_effector(
                right_arm.OPTIONS
                .EFFECTOR
                .AGX_GRIPPER
            )
        )

        right_arm.connect()

        time.sleep(0.8)

        if not right_arm.is_ok():

            raise RuntimeError(
                "RIGHT Piper communication is not OK."
            )

        q_slave_0 = wait_joint_feedback(
            right_arm
        )

        if q_slave_0 is None:

            raise RuntimeError(
                "Cannot read RIGHT joint feedback."
            )

        right_gripper_state = (
            wait_gripper_feedback(
                right_gripper
            )
        )

        if right_gripper_state is None:

            if args.live:

                raise RuntimeError(
                    "Cannot read RIGHT gripper "
                    "feedback. LIVE mode aborted."
                )

            print(
                "[WARN] RIGHT gripper feedback "
                "not available."
            )

            gripper_slave_0 = 0.0

        else:

            if (
                right_gripper_state.msg.mode
                != "width"
            ):

                raise RuntimeError(
                    "RIGHT gripper is not "
                    "in width mode."
                )

            gripper_slave_0 = (
                right_gripper_state.msg.value
            )

        status = right_arm.get_arm_status()

        print()
        print("RIGHT initial state:")

        print(
            "  q = ["
            + ", ".join(
                f"{x:+.4f}"
                for x in q_slave_0
            )
            + "]"
        )

        print(
            f"  gripper = "
            f"{gripper_slave_0 * 1000:.2f} mm"
        )

        if status is not None:

            print(
                f"  arm_status = "
                f"{status.msg.arm_status}"
            )

            print(
                f"  ctrl_mode = "
                f"{status.msg.ctrl_mode}"
            )

            print(
                f"  motion_mode = "
                f"{status.msg.mode_feedback}"
            )

        # ====================================================
        # 2. Open raw Leader reader BEFORE mode change
        # ====================================================

        print()
        print(
            "[2] Starting LEFT raw "
            "SocketCAN reader..."
        )

        leader_reader = (
            PiperLeaderReader(
                LEFT_CAN
            )
        )

        leader_reader.start()

        # ====================================================
        # 3. Connect left arm + enter Leader
        # ====================================================

        left_arm = create_piper(
            LEFT_CAN
        )

        left_arm.connect()

        time.sleep(0.3)

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
        # 4. Wait for complete Leader state
        # ====================================================

        leader_initial = (
            wait_leader_state(
                leader_reader,
                timeout=4.0,
            )
        )

        if leader_initial is None:

            raise RuntimeError(
                "No complete Leader state "
                "(6 joints + gripper)."
            )

        q_master_0 = list(
            leader_initial.q
        )

        gripper_master_0 = (
            leader_initial.gripper_m
        )

        print()
        print("LEFT initial state:")

        print(
            "  q = ["
            + ", ".join(
                f"{x:+.4f}"
                for x in q_master_0
            )
            + "]"
        )

        print(
            f"  gripper = "
            f"{gripper_master_0 * 1000:.2f} mm"
        )

        # ====================================================
        # 5. Relative mapping explanation
        # ====================================================

        print()
        print(
            "Mapping:"
        )

        print(
            "  q_des = "
            "q_slave_0 + "
            "(q_master - q_master_0)"
        )

        print(
            "  g_des = "
            "g_slave_0 + "
            "(g_master - g_master_0)"
        )

        # ====================================================
        # 6. Prepare follower if LIVE
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
                    "Failed to enable all "
                    "RIGHT joints."
                )

            print(
                "  joints enabled =",
                right_arm
                .get_joints_enable_status_list()
            )

            right_arm.set_speed_percent(
                args.arm_speed_percent
            )

            right_arm.set_motion_mode(
                "j"
            )

            # First command = current pose.
            # This prevents command discontinuity.
            right_arm.move_j(
                list(q_slave_0)
            )

            # First gripper command = actual current width.
            right_gripper.move_gripper_m(
                value=gripper_slave_0,
                force=args.gripper_force,
            )

            print()
            print(
                "RIGHT Piper prepared."
            )

            print(
                "第一轮只建议缓慢移动 J1 "
                "约 2~3 度。"
            )

            print(
                "准备好急停。"
            )

            input(
                "按 Enter 开始 LIVE 跟随..."
            )

        else:

            print()
            print(
                "DRY RUN："
            )

            print(
                "RIGHT Piper 不会收到任何 "
                "运动/夹爪控制命令。"
            )

            print(
                "现在拖动左臂和夹爪，"
                "观察 q_des / g_des。"
            )

        # ====================================================
        # 7. Main teleoperation loop
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

        last_loop_time = (
            time.monotonic()
        )

        next_tick = (
            last_loop_time
        )

        last_print_time = 0.0
        last_gripper_send = 0.0

        watchdog_active = False

        while True:

            now = time.monotonic()

            leader = (
                leader_reader.get_state()
            )

            # --------------------------------------------
            # Watchdog
            # --------------------------------------------

            data_valid = (
                leader.q is not None
                and
                leader.gripper_m is not None
                and
                leader.joint_age
                <= args.watchdog
                and
                leader.gripper_age
                <= args.watchdog
            )

            if not data_valid:

                if not watchdog_active:

                    print(
                        "\n[WATCHDOG] "
                        "Leader data stale."
                    )

                    print(
                        "停止更新 RIGHT target."
                    )

                    watchdog_active = True

                time.sleep(
                    min(period, 0.01)
                )

                next_tick = (
                    time.monotonic()
                )

                continue

            if watchdog_active:

                print(
                    "[WATCHDOG] "
                    "Leader data recovered."
                )

                watchdog_active = False

            # --------------------------------------------
            # Relative joint mapping
            # --------------------------------------------

            q_target = []

            for i in range(6):

                delta_master = (
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
                    delta_master
                )

                q_target.append(
                    target
                )

            q_target = clamp_joints(
                q_target
            )

            # --------------------------------------------
            # Relative gripper mapping
            # --------------------------------------------

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

            # --------------------------------------------
            # Software slew rate limiting
            # --------------------------------------------

            dt = max(
                now - last_loop_time,
                1e-4,
            )

            max_joint_delta = (
                args.max_joint_speed
                * dt
            )

            q_cmd = joint_slew_limit(
                previous_q_cmd,
                q_target,
                max_joint_delta,
            )

            max_gripper_delta = (
                args.max_gripper_speed
                * dt
            )

            gripper_cmd = (
                scalar_slew_limit(
                    previous_gripper_cmd,
                    gripper_target,
                    max_gripper_delta,
                )
            )

            # --------------------------------------------
            # Send commands
            # --------------------------------------------

            if args.live:

                right_arm.move_j(
                    list(q_cmd)
                )

                # Gripper doesn't need 50 Hz commands.
                # Send max ~20 Hz.
                if (
                    now
                    - last_gripper_send
                    >= 0.05
                ):

                    right_gripper.move_gripper_m(
                        value=gripper_cmd,
                        force=args.gripper_force,
                    )

                    last_gripper_send = now

            previous_q_cmd = list(
                q_cmd
            )

            previous_gripper_cmd = (
                gripper_cmd
            )

            last_loop_time = now

            # --------------------------------------------
            # Console output ~5 Hz
            # --------------------------------------------

            if (
                now - last_print_time
                >= 0.2
            ):

                right_feedback = (
                    right_arm
                    .get_joint_angles()
                )

                if right_feedback is not None:

                    q_actual = list(
                        right_feedback.msg
                    )

                    max_error = max(
                        abs(c - a)
                        for c, a
                        in zip(
                            q_cmd,
                            q_actual,
                        )
                    )

                else:

                    max_error = float(
                        "nan"
                    )

                prefix = (
                    "LIVE"
                    if args.live
                    else "DRY "
                )

                print(
                    f"[{prefix}] "
                    f"leader_age="
                    f"{leader.joint_age * 1000:4.1f} ms  "
                    f"q_des=["
                    + ", ".join(
                        f"{x:+.3f}"
                        for x in q_cmd
                    )
                    + "]  "
                    f"g_des="
                    f"{gripper_cmd * 1000:5.1f} mm  "
                    f"err_max="
                    f"{max_error:.3f} rad"
                )

                last_print_time = now

            # --------------------------------------------
            # Fixed-rate loop
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

                # Do not accumulate timing drift.
                next_tick = (
                    time.monotonic()
                )

    except KeyboardInterrupt:

        print()
        print(
            "[INFO] Teleoperation stopped."
        )

    finally:

        # ====================================================
        # Cleanup
        # ====================================================

        if leader_reader is not None:

            leader_reader.stop()

        # IMPORTANT:
        # Do NOT disable motors automatically.
        #
        # Right arm remains holding its last position.
        #
        # Do NOT switch left to follower automatically,
        # because that may suddenly change its mechanical
        # behavior / stiffness.

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
        print(
            "[INFO] CAN connections closed."
        )

        print(
            "[INFO] No motor-disable command sent."
        )

        print(
            "[INFO] LEFT remains in Leader role."
        )


if __name__ == "__main__":
    main()