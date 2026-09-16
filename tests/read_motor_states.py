import sys
import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


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
    time.sleep(1)

    for joint in range(1, 7):

        state = arm.get_motor_states(joint)

        print("=" * 60)
        print(f"Joint {joint}")

        if state is None:
            print("None")
        else:
            print(state.msg)
            print("hz:", state.hz)
            print("timestamp:", state.timestamp)

finally:
    arm.disconnect()