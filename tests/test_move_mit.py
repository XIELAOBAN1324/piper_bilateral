#!/usr/bin/env python3

import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


RIGHT_CAN = "can_piper_right"


def create_piper():

    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=RIGHT_CAN,
    )

    return AgxArmFactory.create_arm(cfg)


def main():

    print("="*60)
    print("Piper MIT mode test")
    print("="*60)


    arm = create_piper()

    arm.connect()

    time.sleep(1)


    if not arm.is_ok():

        raise RuntimeError(
            "CAN communication failed"
        )


    print(
        "joint:"
    )

    q0 = (
        arm.get_joint_angles()
        .msg
    )

    print(q0)


    print()

    print(
        "Enable joints"
    )


    arm.enable()

    time.sleep(1)


    print(
        arm.get_joints_enable_status_list()
    )


    print()

    print(
        "Current mode:"
    )

    print(
        arm.get_arm_status()
    )


    print()

    print(
        "Start MIT hold test"
    )


    q_hold = list(q0)


    # conservative gains
    kp = [
        10,
        10,
        10,
        5,
        5,
        5,
    ]


    kd = [
        0.5,
        0.5,
        0.5,
        0.2,
        0.2,
        0.2,
    ]


    torque = [
        0,
        0,
        0,
        0,
        0,
        0,
    ]


    try:

        while True:


            arm.move_mit(
                q=q_hold,
                qd=[
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                ],
                kp=kp,
                kd=kd,
                tau=torque,
            )


            state = (
                arm.get_joint_angles()
            )


            if state:

                print(
                    [
                        round(x,4)
                        for x in state.msg
                    ]
                )


            time.sleep(
                0.01
            )


    except KeyboardInterrupt:

        print(
            "stop"
        )


    finally:

        arm.disconnect()



if __name__ == "__main__":

    main()