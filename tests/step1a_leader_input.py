import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


CHANNEL = "can_piper_left"


def main():
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=CHANNEL,
    )

    arm = AgxArmFactory.create_arm(cfg)

    try:
        print(f"[INFO] Connecting to {CHANNEL} ...")
        arm.connect()
        time.sleep(1.0)

        # ---------- 切换前状态 ----------
        print("\n========== BEFORE ==========")

        status = arm.get_arm_status()
        joints = arm.get_joint_angles()

        print("is_ok:", arm.is_ok())
        print("fps:", arm.get_fps())
        print(
            "normal joints:",
            None if joints is None else joints.msg,
        )

        if status is not None:
            print("status:")
            print(status.msg)

        print("\n"
              "[WARNING] 左臂即将进入 Leader / zero-force drag mode.\n"
              "请扶住机械臂，准备好后按 Enter。")
        input()

        # ---------- 进入 Leader ----------
        print("[INFO] Sending set_leader_mode()")
        arm.set_leader_mode()

        # 给控制器一些时间切换
        time.sleep(2.0)

        print("\n========== LEADER MODE ==========")
        print("拖动左臂，观察 q 是否实时变化。")
        print("Ctrl+C 结束。\n")

        last_print = 0.0
        valid_count = 0
        none_count = 0

        while True:
            now = time.monotonic()

            leader = arm.get_leader_joint_angles()

            if leader is None:
                none_count += 1
            else:
                valid_count += 1

                # 控制台只打印约 10 Hz，
                # 不影响底层 CAN 200 Hz 接收
                if now - last_print >= 0.1:
                    q = leader.msg

                    print(
                        "q = ["
                        + ", ".join(f"{x:+.4f}" for x in q)
                        + "]"
                        + f"    hz={leader.hz:.1f}"
                    )

                    last_print = now

            # 读取循环本身约 200 Hz
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")

    finally:
        # 注意：
        # 这里故意不调用 set_follower_mode()
        # 避免 Ctrl+C 时机械臂突然从低阻模式切回刚性状态。
        arm.disconnect()
        print("[INFO] CAN disconnected.")
        print("[INFO] 左臂保持当前 Leader 配置。")


if __name__ == "__main__":
    main()