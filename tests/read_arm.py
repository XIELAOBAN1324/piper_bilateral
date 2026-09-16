import sys
import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <can_channel>")
        print(f"Example: {sys.argv[0]} can_piper_left")
        sys.exit(1)

    channel = sys.argv[1]

    print(f"[INFO] Connecting to Piper on {channel}")

    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )

    arm = AgxArmFactory.create_arm(cfg)

    try:
        arm.connect()

        print("[INFO] Connected.")
        print("[INFO] Reading joint feedback only. No control commands will be sent.")

        while True:
            joints = arm.get_joint_angles()

            if joints is not None:
                print(
                    f"q={joints.msg}, "
                    f"hz={joints.hz:.1f}, "
                    f"timestamp={joints.timestamp}"
                )
            else:
                print("[WARN] No joint feedback yet")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping.")

    finally:
        arm.disconnect()
        print("[INFO] Disconnected.")


if __name__ == "__main__":
    main()