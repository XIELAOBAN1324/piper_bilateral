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
    arm.connect()
    time.sleep(1)

    print("BEFORE:")
    print("  is_ok:", arm.is_ok())
    print("  fps:", arm.get_fps())

    input(
        "\n扶住左臂。\n"
        "按 Enter 切换 Leader，随后轻轻拖动机械臂 5~10 秒..."
    )

    print("[INFO] set_leader_mode()")
    arm.set_leader_mode()

    t0 = time.monotonic()

    while time.monotonic() - t0 < 10.0:

        leader = arm.get_leader_joint_angles()

        if leader is not None:
            print(
                "LEADER:",
                leader.msg,
                "hz:",
                leader.hz
            )

        time.sleep(0.01)

finally:
    print("\n[INFO] Restoring follower mode...")

    # 退出时一定恢复
    for _ in range(3):
        arm.set_follower_mode()
        time.sleep(0.1)

    time.sleep(1)

    print("fps after follower:", arm.get_fps())

    joints = arm.get_joint_angles()

    print(
        "normal joints after follower:",
        None if joints is None else joints.msg
    )

    arm.disconnect()