#!/usr/bin/env python3

import argparse
import math
import socket
import struct
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

DEFAULT_CHANNEL = "can_piper_left"

# Linux struct can_frame:
#
# struct can_frame {
#     canid_t can_id;   // uint32
#     __u8    len;      // uint8
#     __u8    __pad;
#     __u8    __res0;
#     __u8    len8_dlc;
#     __u8    data[8];
# };
#
# 对 classic CAN 可以使用这个格式解析 16 字节 can_frame。
CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)

# Piper Leader CAN IDs
CAN_ID_MODE = 0x151
CAN_ID_J12 = 0x155
CAN_ID_J34 = 0x156
CAN_ID_J56 = 0x157
CAN_ID_GRIPPER = 0x159

LEADER_CAN_IDS = {
    CAN_ID_MODE,
    CAN_ID_J12,
    CAN_ID_J34,
    CAN_ID_J56,
    CAN_ID_GRIPPER,
}

DEG_TO_RAD = math.pi / 180.0


# ============================================================
# State
# ============================================================

@dataclass
class LeaderState:
    """
    Piper Leader input state.

    q:
        6 joint angles, rad.

    gripper_m:
        Gripper stroke/opening command, meters.

    gripper_effort:
        0x159 effort control field, N·m according to protocol.
        NOTE:
        This is a Leader gripper COMMAND field, not an external
        force measurement.

    gripper_status:
        0x159 status_code:
            0x00 disabled
            0x01 enabled
            0x02 disabled + clear error
            0x03 enabled + clear error

    gripper_set_zero:
        0x00 invalid
        0xAE set current position as zero
    """

    q: List[Optional[float]]
    gripper_m: Optional[float] = None
    gripper_effort: Optional[float] = None
    gripper_status: Optional[int] = None
    gripper_set_zero: Optional[int] = None

    last_joint_update: Optional[float] = None
    last_gripper_update: Optional[float] = None

    @property
    def joints_valid(self) -> bool:
        return all(value is not None for value in self.q)

    @property
    def gripper_valid(self) -> bool:
        return self.gripper_m is not None

    @property
    def valid(self) -> bool:
        return self.joints_valid and self.gripper_valid


# ============================================================
# Decode helpers
# ============================================================

def decode_joint(raw4: bytes) -> float:
    """
    Decode Piper Leader joint command.

    CAN 0x155 / 0x156 / 0x157:
        signed int32 big-endian
        unit = 0.001 degree

    Returns:
        joint angle in radians
    """

    value_mdeg = int.from_bytes(
        raw4,
        byteorder="big",
        signed=True,
    )

    angle_deg = value_mdeg * 0.001

    return angle_deg * DEG_TO_RAD


def decode_gripper(data: bytes):
    """
    Decode Piper Leader gripper command CAN ID 0x159.

    bytes 0..3:
        grippers_angle
        signed int32
        unit = 0.001 mm

    bytes 4..5:
        grippers_effort
        uint16
        unit = 0.001 N·m

    byte 6:
        status_code

    byte 7:
        set_zero
    """

    if len(data) < 8:
        raise ValueError("0x159 payload shorter than 8 bytes")

    # 0.001 mm
    gripper_raw = int.from_bytes(
        data[0:4],
        byteorder="big",
        signed=True,
    )

    # 0.001 mm -> m
    #
    # 1 raw
    # = 0.001 mm
    # = 0.000001 m
    gripper_m = gripper_raw * 1e-6

    effort_raw = int.from_bytes(
        data[4:6],
        byteorder="big",
        signed=False,
    )

    gripper_effort = effort_raw * 0.001

    status_code = int(data[6])
    set_zero = int(data[7])

    return (
        gripper_m,
        gripper_effort,
        status_code,
        set_zero,
    )


# ============================================================
# pyAgxArm
# ============================================================

def create_left_arm(channel: str):
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )

    return AgxArmFactory.create_arm(cfg)


# ============================================================
# Display
# ============================================================

def format_joint_state(state: LeaderState) -> str:
    if not state.joints_valid:
        return "q=[waiting...]"

    return (
        "q=["
        + ", ".join(
            f"{value:+.4f}"
            for value in state.q
        )
        + "]"
    )


def format_gripper_state(state: LeaderState) -> str:
    if not state.gripper_valid:
        return "gripper=waiting..."

    status_text = {
        0x00: "DISABLED",
        0x01: "ENABLED",
        0x02: "DISABLE_CLEAR_ERR",
        0x03: "ENABLE_CLEAR_ERR",
    }.get(
        state.gripper_status,
        "UNKNOWN",
    )

    return (
        f"gripper={state.gripper_m * 1000.0:6.2f} mm"
        f"  effort={state.gripper_effort:5.3f}"
        f"  status=0x{state.gripper_status:02X}"
        f"({status_text})"
        f"  zero=0x{state.gripper_set_zero:02X}"
    )


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Read Piper Leader joints and gripper directly "
            "from native Linux SocketCAN."
        )
    )

    parser.add_argument(
        "--channel",
        default=DEFAULT_CHANNEL,
        help=(
            "SocketCAN channel "
            f"(default: {DEFAULT_CHANNEL})"
        ),
    )

    parser.add_argument(
        "--no-set-leader",
        action="store_true",
        help=(
            "Do not send set_leader_mode(). "
            "Use this if the Piper is already in Leader mode."
        ),
    )

    args = parser.parse_args()

    channel = args.channel

    # --------------------------------------------------------
    # State
    # --------------------------------------------------------

    state = LeaderState(
        q=[None] * 6,
    )

    counts = {
        CAN_ID_MODE: 0,
        CAN_ID_J12: 0,
        CAN_ID_J34: 0,
        CAN_ID_J56: 0,
        CAN_ID_GRIPPER: 0,
    }

    total_frames = 0

    first_frame_time = None
    last_print_time = 0.0

    sock = None
    arm = None

    # --------------------------------------------------------
    # Open raw CAN socket FIRST
    # --------------------------------------------------------

    try:
        print("=" * 78)
        print("Piper Leader SocketCAN Probe")
        print("=" * 78)

        print(f"[1] Opening native SocketCAN: {channel}")

        sock = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )

        sock.bind((channel,))
        sock.settimeout(0.2)

        print("[OK] SocketCAN reader ready.")

        # ----------------------------------------------------
        # Connect pyAgxArm
        # ----------------------------------------------------

        print()
        print("[2] Connecting pyAgxArm...")

        arm = create_left_arm(channel)
        arm.connect()

        time.sleep(0.5)

        # pyAgxArm may still see normal feedback here if the arm
        # was previously in Follower mode.
        print(
            f"[INFO] pyAgxArm is_ok before Leader: "
            f"{arm.is_ok()}"
        )

        print(
            f"[INFO] pyAgxArm fps before Leader: "
            f"{arm.get_fps():.1f}"
        )

        # ----------------------------------------------------
        # Enter Leader
        # ----------------------------------------------------

        if not args.no_set_leader:

            print()
            print(
                "[WARNING] 左臂即将进入 "
                "Leader / zero-force drag mode."
            )
            print("请扶住机械臂。")

            input("准备好后按 Enter...")

            print("[3] Sending set_leader_mode() ...")

            arm.set_leader_mode()

            print("[OK] Leader command sent.")

        else:

            print()
            print(
                "[3] --no-set-leader specified; "
                "not changing Piper mode."
            )

        # ----------------------------------------------------
        # Read Leader frames
        # ----------------------------------------------------

        print()
        print("=" * 78)
        print("Leader input monitor")
        print("=" * 78)

        print()
        print("现在可以：")
        print("  1. 缓慢拖动 J1 ~ J6")
        print("  2. 手动开合左臂夹爪")
        print()
        print("Ctrl+C 结束。")
        print()

        while True:

            try:
                frame = sock.recv(CAN_FRAME_SIZE)

            except socket.timeout:
                continue

            if len(frame) != CAN_FRAME_SIZE:
                continue

            can_id, dlc, data = struct.unpack(
                CAN_FRAME_FORMAT,
                frame,
            )

            # Remove Linux CAN flags:
            # EFF / RTR / ERR flags, retain identifier.
            can_id &= 0x1FFFFFFF

            total_frames += 1

            if first_frame_time is None:
                first_frame_time = time.monotonic()

            if can_id not in LEADER_CAN_IDS:
                continue

            counts[can_id] += 1

            now = time.monotonic()

            # ------------------------------------------------
            # J1 / J2
            # ------------------------------------------------

            if can_id == CAN_ID_J12:

                if dlc < 8:
                    continue

                state.q[0] = decode_joint(
                    data[0:4]
                )

                state.q[1] = decode_joint(
                    data[4:8]
                )

                if state.joints_valid:
                    state.last_joint_update = now

            # ------------------------------------------------
            # J3 / J4
            # ------------------------------------------------

            elif can_id == CAN_ID_J34:

                if dlc < 8:
                    continue

                state.q[2] = decode_joint(
                    data[0:4]
                )

                state.q[3] = decode_joint(
                    data[4:8]
                )

                if state.joints_valid:
                    state.last_joint_update = now

            # ------------------------------------------------
            # J5 / J6
            # ------------------------------------------------

            elif can_id == CAN_ID_J56:

                if dlc < 8:
                    continue

                state.q[4] = decode_joint(
                    data[0:4]
                )

                state.q[5] = decode_joint(
                    data[4:8]
                )

                if state.joints_valid:
                    state.last_joint_update = now

            # ------------------------------------------------
            # Gripper
            # ------------------------------------------------

            elif can_id == CAN_ID_GRIPPER:

                if dlc < 8:
                    continue

                (
                    state.gripper_m,
                    state.gripper_effort,
                    state.gripper_status,
                    state.gripper_set_zero,
                ) = decode_gripper(data)

                state.last_gripper_update = now

            # ------------------------------------------------
            # 10 Hz console output
            # ------------------------------------------------

            if (
                now - last_print_time >= 0.1
                and (
                    state.joints_valid
                    or state.gripper_valid
                )
            ):

                joint_text = format_joint_state(
                    state
                )

                gripper_text = format_gripper_state(
                    state
                )

                # Data age helps us later build watchdogs.
                if state.last_joint_update is not None:
                    joint_age_ms = (
                        now
                        - state.last_joint_update
                    ) * 1000.0
                else:
                    joint_age_ms = float("nan")

                if state.last_gripper_update is not None:
                    gripper_age_ms = (
                        now
                        - state.last_gripper_update
                    ) * 1000.0
                else:
                    gripper_age_ms = float("nan")

                print(
                    f"{joint_text}"
                    f"  |  "
                    f"{gripper_text}"
                    f"  |  "
                    f"age="
                    f"{joint_age_ms:4.1f}/"
                    f"{gripper_age_ms:4.1f} ms"
                )

                last_print_time = now

    except KeyboardInterrupt:

        print()
        print("[INFO] Stopped by user.")

    finally:

        # ----------------------------------------------------
        # Statistics
        # ----------------------------------------------------

        print()
        print("=" * 78)
        print("CAN statistics")
        print("=" * 78)

        for can_id in (
            CAN_ID_MODE,
            CAN_ID_J12,
            CAN_ID_J34,
            CAN_ID_J56,
            CAN_ID_GRIPPER,
        ):
            print(
                f"0x{can_id:03X}: "
                f"{counts[can_id]}"
            )

        if first_frame_time is not None:

            elapsed = (
                time.monotonic()
                - first_frame_time
            )

            print(
                f"Total CAN frames seen: "
                f"{total_frames}"
            )

            print(
                f"Elapsed: {elapsed:.2f} s"
            )

        # ----------------------------------------------------
        # Final Leader state
        # ----------------------------------------------------

        print()
        print("=" * 78)
        print("Final Leader state")
        print("=" * 78)

        print(format_joint_state(state))
        print(format_gripper_state(state))

        # ----------------------------------------------------
        # Cleanup
        # ----------------------------------------------------

        if sock is not None:

            try:
                sock.close()

            except Exception:
                pass

            print("[INFO] Raw SocketCAN closed.")

        if arm is not None:

            try:
                arm.disconnect()

            except Exception:
                pass

            print("[INFO] pyAgxArm disconnected.")

        print()
        print(
            "[IMPORTANT] 没有发送 set_follower_mode()。"
        )

        print(
            "[IMPORTANT] 左臂会保持当前 Leader role。"
        )

        print(
            "[IMPORTANT] 如需恢复普通反馈，"
            "再单独运行 follower 恢复程序。"
        )


if __name__ == "__main__":
    main()