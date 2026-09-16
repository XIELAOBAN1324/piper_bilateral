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

    print("=" * 60)
    print("Piper single joint MIT test")
    print("=" * 60)


    arm = create_piper()

    arm.connect()

    time.sleep(1)


    if not arm.is_ok():

        raise RuntimeError(
            "CAN failed"
        )


    arm.enable()

    time.sleep(1)


    q0 = (
        arm.get_joint_angles()
        .msg
    )


    print(
        "Initial q:"
    )

    print(q0)


    # 保持当前姿态
    q_hold = list(q0)


    print()

    print(
        "Start MIT hold..."
    )

    print(
        "Push robot gently."
    )

    print(
        "Ctrl+C stop."
    )


    try:

        while True:

            for i in range(6):

                arm.move_mit(
                    i + 1,

                    p_des=q_hold[i],

                    v_des=0.0,

                    # 初始保守参数
                    kp=10.0,

                    kd=0.8,

                    t_ff=0.0,
                )


            time.sleep(
                0.005
            )


    except KeyboardInterrupt:

        print(
            "Stopped"
        )


    finally:

        arm.disconnect()



if __name__ == "__main__":

    main()