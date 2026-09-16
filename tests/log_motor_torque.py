import sys
import csv
import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <can_channel> <output.csv>")
    sys.exit(1)

channel = sys.argv[1]
output_csv = sys.argv[2]

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.V189,
    interface="socketcan",
    channel=channel,
)

arm = AgxArmFactory.create_arm(cfg)

arm.connect()
time.sleep(1.0)

print(f"Logging {channel}")
print("Press Ctrl+C to stop.")

with open(output_csv, "w", newline="") as f:
    writer = csv.writer(f)

    header = ["time"]

    for j in range(1, 7):
        header += [
            f"q{j}",
            f"dq{j}",
            f"current{j}",
            f"tau{j}",
        ]

    writer.writerow(header)

    t0 = time.monotonic()

    try:
        while True:
            row = [time.monotonic() - t0]

            valid = True

            for joint in range(1, 7):
                state = arm.get_motor_states(joint)

                if state is None:
                    valid = False
                    break

                msg = state.msg

                row += [
                    msg.position,
                    msg.velocity,
                    msg.current,
                    msg.torque,
                ]

            if valid:
                writer.writerow(row)

            # 约 200 Hz
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        arm.disconnect()