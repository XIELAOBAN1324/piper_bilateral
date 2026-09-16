import sys
import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


def safe_msg(x):
    if x is None:
        return None
    return x.msg


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <can_channel>")
        sys.exit(1)

    channel = sys.argv[1]

    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )

    arm = AgxArmFactory.create_arm(cfg)

    try:
        arm.connect()
        time.sleep(1.0)

        print("=" * 70)
        print("channel:", channel)
        print("is_ok :", arm.is_ok())
        print("fps   :", arm.get_fps())
        print("=" * 70)

        for i in range(20):
            normal = arm.get_joint_angles()
            leader = arm.get_leader_joint_angles()
            status = arm.get_arm_status()

            print(f"\n[{i}]")

            if normal is None:
                print("normal joints : None")
            else:
                print(
                    "normal joints :",
                    normal.msg,
                    f"hz={normal.hz:.1f}",
                )

            if leader is None:
                print("leader joints : None")
            else:
                print(
                    "leader joints :",
                    leader.msg,
                    f"hz={leader.hz:.1f}",
                )

            if status is None:
                print("arm status    : None")
            else:
                print("arm status    :", status.msg)

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()