#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Piper Phase 2A MIT Teleoperation v2

Baseline MIT teleoperation:
- LEFT raw SocketCAN leader input
- RIGHT six-joint MIT control
- master/slave offset calibration
- filtered velocity feed-forward
- safer MIT shutdown

Run:
  python step2a_mit_teleop_v2.py
  python step2a_mit_teleop_v2.py --live
"""

import argparse
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


LEFT_CAN = "can_piper_left"
RIGHT_CAN = "can_piper_right"

RATE = 200.0

FRAME = "=IB3x8s"
SIZE = struct.calcsize(FRAME)

J12 = 0x155
J34 = 0x156
J56 = 0x157

RAD = 3.141592653589793 / 180.0

KP = [5, 5, 5, 4, 4, 3]
KD = [1, 1, 1, 0.6, 0.6, 0.5]

VEL_SCALE = 0.3
FILTER = 0.7


def create_piper(channel):
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )
    return AgxArmFactory.create_arm(cfg)


class Leader:
    def __init__(self, channel):
        self.channel = channel
        self.q = [None] * 6
        self.ts = [0, 0, 0]
        self.running = False

    def start(self):
        self.sock = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW
        )
        self.sock.bind((self.channel,))
        self.running = True
        self.thread = threading.Thread(
            target=self.loop,
            daemon=True
        )
        self.thread.start()

    def decode(self, b):
        return int.from_bytes(
            b,
            "big",
            signed=True
        ) * 0.001 * RAD

    def loop(self):
        while self.running:
            try:
                frame = self.sock.recv(SIZE)
            except:
                continue

            cid, dlc, data = struct.unpack(FRAME, frame)
            cid &= 0x1fffffff
            t = time.monotonic()

            if cid == J12:
                self.q[0] = self.decode(data[:4])
                self.q[1] = self.decode(data[4:8])
                self.ts[0] = t

            elif cid == J34:
                self.q[2] = self.decode(data[:4])
                self.q[3] = self.decode(data[4:8])
                self.ts[1] = t

            elif cid == J56:
                self.q[4] = self.decode(data[:4])
                self.q[5] = self.decode(data[4:8])
                self.ts[2] = t

    def get(self):
        if any(x is None for x in self.q):
            return None, 999

        return list(self.q), (
            time.monotonic() - min(self.ts)
        )

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except:
            pass


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--live",
        action="store_true"
    )
    args = parser.parse_args()

    right = create_piper(RIGHT_CAN)
    right.connect()
    time.sleep(1)

    q_slave0 = list(
        right.get_joint_angles().msg
    )

    leader = Leader(LEFT_CAN)
    leader.start()

    left = create_piper(LEFT_CAN)
    left.connect()

    input(
        "Move LEFT to zero pose, press Enter..."
    )

    left.set_leader_mode()

    while True:
        qm, age = leader.get()
        if qm and age < 0.1:
            break
        time.sleep(0.01)

    qm0 = list(qm)

    print("Master zero:", qm0)
    print("Slave zero :", q_slave0)

    if args.live:
        right.enable()
        right.set_motion_mode("mit")

    last_q = qm
    last_dq = [0]*6

    try:
        while True:

            qm, age = leader.get()

            if qm is None:
                continue

            dt = 1.0 / RATE

            dq = [
                (a-b)/dt
                for a,b in zip(qm,last_q)
            ]

            dq = [
                FILTER*o +
                (1-FILTER)*n
                for n,o in zip(dq,last_dq)
            ]

            dq = [
                VEL_SCALE*x
                for x in dq
            ]

            target = [
                q_slave0[i]
                +
                qm[i]
                -
                qm0[i]
                for i in range(6)
            ]

            if args.live:
                for i in range(6):
                    right.move_mit(
                        i+1,
                        target[i],
                        dq[i],
                        KP[i],
                        KD[i],
                        0
                    )

            print(
                "err max:",
                max(
                    abs(target[i] -
                        right.get_joint_angles().msg[i])
                    for i in range(6)
                )
            )

            last_q = qm
            last_dq = dq

            time.sleep(
                1.0/RATE
            )

    except KeyboardInterrupt:
        pass

    finally:
        try:
            q = list(right.get_joint_angles().msg)
            for _ in range(20):
                for i in range(6):
                    right.move_mit(
                        i+1,q[i],0,5,0.5,0
                    )
                time.sleep(0.01)

            right.set_motion_mode("p")
            right.disable()
        except:
            pass

        right.disconnect()
        left.disconnect()
        leader.stop()


if __name__ == "__main__":
    main()
