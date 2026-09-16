#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Piper Phase 2A MIT Teleoperation

LEFT:
    Raw SocketCAN Leader reader

RIGHT:
    MIT impedance control

Features:
    - 6 joint teleoperation
    - velocity estimation
    - velocity filtering
    - watchdog
    - CSV logging
    - safe MIT shutdown

"""

import argparse
import csv
import os
import socket
import struct
import threading
import time
import math


from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


# ============================================================
# Config
# ============================================================

LEFT_CAN = "can_piper_left"
RIGHT_CAN = "can_piper_right"


CONTROL_RATE = 200.0


CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(
    CAN_FRAME_FORMAT
)


CAN_J12 = 0x155
CAN_J34 = 0x156
CAN_J56 = 0x157


DEG_TO_RAD = math.pi / 180.0


# MIT parameters

MIT_KP = [
    5.0,
    5.0,
    5.0,
    3.0,
    3.0,
    2.0,
]


MIT_KD = [
    0.5,
    0.5,
    0.5,
    0.3,
    0.3,
    0.2,
]


MAX_DQ = [
    1.0,
] * 6


LEADER_TIMEOUT = 0.1


DQ_FILTER_ALPHA = 0.7



# ============================================================
# Piper helpers
# ============================================================

def create_piper(channel):

    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )

    return AgxArmFactory.create_arm(cfg)



# ============================================================
# Leader reader
# ============================================================


class LeaderReader:


    def __init__(self, channel):

        self.channel = channel

        self.sock = None

        self.running = False

        self.thread = None


        self.lock = threading.Lock()


        self.q = [
            None
        ] * 6


        self.stamps = [
            None,
            None,
            None,
        ]


    def start(self):

        self.sock = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )


        self.sock.bind(
            (
                self.channel,
            )
        )


        self.sock.settimeout(
            0.1
        )


        self.running = True


        self.thread = threading.Thread(
            target=self.loop,
            daemon=True,
        )


        self.thread.start()



    @staticmethod
    def decode_joint(data):

        value = int.from_bytes(
            data,
            "big",
            signed=True,
        )


        return (
            value
            *0.001
            *DEG_TO_RAD
        )



    def loop(self):

        while self.running:

            try:

                frame = self.sock.recv(
                    CAN_FRAME_SIZE
                )


            except socket.timeout:

                continue


            if len(frame) != CAN_FRAME_SIZE:

                continue



            can_id, dlc, data = struct.unpack(
                CAN_FRAME_FORMAT,
                frame,
            )


            can_id &= 0x1FFFFFFF


            now=time.monotonic()


            with self.lock:


                if can_id == CAN_J12:

                    self.q[0]=self.decode_joint(
                        data[0:4]
                    )

                    self.q[1]=self.decode_joint(
                        data[4:8]
                    )

                    self.stamps[0]=now



                elif can_id == CAN_J34:


                    self.q[2]=self.decode_joint(
                        data[0:4]
                    )

                    self.q[3]=self.decode_joint(
                        data[4:8]
                    )

                    self.stamps[1]=now



                elif can_id == CAN_J56:


                    self.q[4]=self.decode_joint(
                        data[0:4]
                    )

                    self.q[5]=self.decode_joint(
                        data[4:8]
                    )

                    self.stamps[2]=now




    def get(self):

        now=time.monotonic()


        with self.lock:

            if any(
                x is None
                for x in self.q
            ):

                return None,None


            age = (
                now
                -
                min(self.stamps)
            )


            return list(self.q), age



    def stop(self):

        self.running=False


        if self.thread:

            self.thread.join(
                1
            )


        if self.sock:

            self.sock.close()



# ============================================================
# CSV logger
# ============================================================


class Logger:


    def __init__(self,path):

        self.file=open(
            path,
            "w",
            newline=""
        )


        fields=[
            "time",
            "leader_age",
        ]


        for n in [
            "master",
            "target",
            "slave",
            "dq",
            "error",
        ]:

            for i in range(6):

                fields.append(
                    f"{n}_{i+1}"
                )


        self.writer=csv.DictWriter(
            self.file,
            fields
        )

        self.writer.writeheader()



    def write(self,row):

        self.writer.writerow(row)
        self.file.flush()



    def close(self):

        self.file.close()



# ============================================================
# Utility
# ============================================================


def estimate_velocity(
        q_now,
        q_last,
        dt,
):

    if q_last is None:

        return [
            0.0
        ]*6


    return [
        (a-b)/dt
        for a,b
        in zip(
            q_now,
            q_last
        )
    ]



def filter_velocity(
        dq,
        dq_old,
):

    return [
        DQ_FILTER_ALPHA*o
        +
        (1-DQ_FILTER_ALPHA)*n

        for n,o
        in zip(
            dq,
            dq_old
        )
    ]



# ============================================================
# main
# ============================================================


def main():


    parser=argparse.ArgumentParser()


    parser.add_argument(
        "--live",
        action="store_true"
    )


    parser.add_argument(
        "--record",
        action="store_true"
    )


    parser.add_argument(
        "--duration",
        type=float,
        default=0
    )


    args=parser.parse_args()



    print("="*70)

    print(
        "Piper Phase 2A MIT Teleoperation"
    )

    print("="*70)



    left_arm=None

    right_arm=None

    leader=None

    logger=None



    try:


        print(
            "[1] Connect RIGHT"
        )


        right_arm=create_piper(
            RIGHT_CAN
        )


        right_arm.connect()


        time.sleep(1)



        if not right_arm.is_ok():

            raise RuntimeError(
                "RIGHT CAN failed"
            )



        q0=list(
            right_arm
            .get_joint_angles()
            .msg
        )



        print(
            "RIGHT q0:",
            q0
        )



        print(
            "[2] Enable MIT mode"
        )


        right_arm.enable()


        time.sleep(
            0.5
        )


        right_arm.set_motion_mode(
            "mit"
        )


        time.sleep(
            0.5
        )



        print(
            "[3] Start LEFT leader"
        )


        leader=LeaderReader(
            LEFT_CAN
        )


        leader.start()



        left_arm=create_piper(
            LEFT_CAN
        )

        left_arm.connect()


        time.sleep(
            0.5
        )


        print(
            "Prepare LEFT leader."
        )

        input(
            "Press Enter..."
        )


        left_arm.set_leader_mode()



        # wait leader


        while True:

            q_master,age=leader.get()

            if (
                q_master
                and
                age <0.1
            ):

                break

            time.sleep(
                0.01
            )



        q_master0=list(
            q_master
        )


        print(
            "Leader zero:",
            q_master0
        )



        if args.record:

            os.makedirs(
                "data/mit_teleop",
                exist_ok=True
            )


            logger=Logger(
                "data/mit_teleop/"
                +
                time.strftime(
                    "%Y%m%d_%H%M%S.csv"
                )
            )



        print()

        print(
            "START MIT TELEOP"
        )


        period=1.0/CONTROL_RATE


        last_q=None

        dq_old=[
            0
        ]*6


        start=time.monotonic()

        last=time.monotonic()



        while True:


            now=time.monotonic()


            if (
                args.duration>0
                and
                now-start>args.duration
            ):

                break



            dt=now-last

            last=now



            q_master,age=leader.get()



            if (
                q_master is None
                or
                age>LEADER_TIMEOUT
            ):

                print(
                    "Leader timeout"
                )

                q_target=q0

                dq_target=[
                    0
                ]*6


            else:


                q_target=[]


                for i in range(6):

                    q_target.append(
                        q0[i]
                        +
                        (
                            q_master[i]
                            -
                            q_master0[i]
                        )
                    )



                dq=estimate_velocity(
                    q_master,
                    last_q,
                    dt
                )


                dq=filter_velocity(
                    dq,
                    dq_old
                )



                dq_target=[
                    max(
                        min(
                            x,
                            MAX_DQ[i]
                        ),
                        -MAX_DQ[i]
                    )

                    for i,x in enumerate(dq)
                ]



                dq_old=dq_target



            last_q=q_master



            if args.live:


                for i in range(6):

                    right_arm.move_mit(
                        i+1,

                        p_des=q_target[i],

                        v_des=dq_target[i],

                        kp=MIT_KP[i],

                        kd=MIT_KD[i],

                        t_ff=0.0
                    )



            q_slave=list(
                right_arm
                .get_joint_angles()
                .msg
            )


            if logger:


                logger.write(
                    {

                    "time":
                        now-start,

                    "leader_age":
                        age,


                    **{
                    f"master_{i+1}":
                        q_master[i]
                        if q_master
                        else 0

                    for i in range(6)
                    },


                    **{
                    f"target_{i+1}":
                        q_target[i]

                    for i in range(6)
                    },


                    **{
                    f"slave_{i+1}":
                        q_slave[i]

                    for i in range(6)
                    },


                    **{
                    f"dq_{i+1}":
                        dq_target[i]

                    for i in range(6)
                    },


                    **{
                    f"error_{i+1}":
                        q_target[i]-q_slave[i]

                    for i in range(6)
                    },

                    }
                )



            sleep=period-(time.monotonic()-now)


            if sleep>0:

                time.sleep(
                    sleep
                )




    except KeyboardInterrupt:

        print(
            "STOP"
        )



    finally:


        print(
            "Shutdown MIT"
        )


        if right_arm:


            try:

                q=list(
                    right_arm
                    .get_joint_angles()
                    .msg
                )


                for _ in range(20):

                    for i in range(6):

                        right_arm.move_mit(
                            i+1,
                            q[i],
                            0,
                            5,
                            0.5,
                            0
                        )

                    time.sleep(
                        0.01
                    )



                right_arm.set_motion_mode(
                    "p"
                )


                right_arm.disable()



            except Exception as e:

                print(e)



            right_arm.disconnect()



        if left_arm:

            left_arm.disconnect()



        if leader:

            leader.stop()



        if logger:

            logger.close()



if __name__=="__main__":

    main()