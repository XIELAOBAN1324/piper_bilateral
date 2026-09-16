import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


CHANNEL = "can_piper_left"


cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.V189,
    interface="socketcan",
    channel=CHANNEL,
)

arm = AgxArmFactory.create_arm(cfg)

try:
    print("[INFO] Connecting...")
    arm.connect()

    # 即使当前 RX = 0，TX 仍然可以发送 follower 配置帧。
    print("[INFO] Sending follower mode...")

    # 发送几次只是增加配置帧可靠性，不是旧固件 workaround。
    for _ in range(3):
        arm.set_follower_mode()
        time.sleep(0.1)

    print("[INFO] Waiting for normal feedback...")
    time.sleep(2.0)

    print("is_ok:", arm.is_ok())
    print("fps:", arm.get_fps())

    joints = arm.get_joint_angles()
    status = arm.get_arm_status()

    print(
        "joints:",
        None if joints is None else joints.msg
    )

    print(
        "status:",
        None if status is None else status.msg
    )

finally:
    arm.disconnect()