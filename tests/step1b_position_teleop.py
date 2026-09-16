import argparse
import math
import threading
import time

import can

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


LEFT_CAN = "can_piper_left"
RIGHT_CAN = "can_piper_right"

CONTROL_HZ = 50.0

# 第一轮故意限制得很慢
MAX_JOINT_SPEED = 0.15  # rad/s ≈ 8.6 deg/s

# 与旧主从系统保持 1:1
SCALE = [1.0] * 6
DIRECTION = [1.0] * 6

# 标准 Piper 官方关节范围，rad
JOINT_LIMITS = [
    (-2.617994,  2.617994),  # J1
    ( 0.0,       3.141593),  # J2
    (-2.967060,  0.0),       # J3
    (-1.745330,  1.745330),  # J4
    (-1.221730,  1.221730),  # J5
    (-2.094396,  2.094396),  # J6
]


def create_arm(channel):
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V189,
        interface="socketcan",
        channel=channel,
    )
    return AgxArmFactory.create_arm(cfg)


def decode_joint(raw):
    """
    Piper Leader joint command:
      int32 signed
      unit = 0.001 degree
    """
    mdeg = int.from_bytes(
        raw,
        byteorder="big",
        signed=True,
    )
    return mdeg * 0.001 * math.pi / 180.0


class LeaderReader:
    def __init__(self, channel):
        self.channel = channel
        self.bus = None
        self.thread = None
        self.running = False

        self.lock = threading.Lock()
        self.q = [None] * 6
        self.last_update = None

    def start(self):
        self.bus = can.interface.Bus(
            interface="socketcan",
            channel=self.channel,
        )

        self.running = True

        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
        )
        self.thread.start()

    def _run(self):
        while self.running:
            msg = self.bus.recv(timeout=0.1)

            if msg is None:
                continue

            can_id = msg.arbitration_id
            data = msg.data

            updated = False

            with self.lock:
                if can_id == 0x155:
                    self.q[0] = decode_joint(data[0:4])
                    self.q[1] = decode_joint(data[4:8])
                    updated = True

                elif can_id == 0x156:
                    self.q[2] = decode_joint(data[0:4])
                    self.q[3] = decode_joint(data[4:8])
                    updated = True

                elif can_id == 0x157:
                    self.q[4] = decode_joint(data[0:4])
                    self.q[5] = decode_joint(data[4:8])
                    updated = True

                if updated and all(x is not None for x in self.q):
                    self.last_update = time.monotonic()

    def get(self):
        with self.lock:
            if self.last_update is None:
                return None, None

            return list(self.q), self.last_update

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.bus is not None:
            self.bus.shutdown()


def set_left_leader():
    """
    pyAgxArm 当前 Piper get_leader_joint_angles() 实机存在问题，
    但 set_leader_mode() 已确认能够正确触发
    0x151 / 0x155 / 0x156 / 0x157 / 0x159。
    """
    arm = create_arm(LEFT_CAN)

    try:
        arm.connect()
        time.sleep(0.3)

        arm.set_leader_mode()

        time.sleep(0.5)

    finally:
        arm.disconnect()


def wait_for_leader(reader, timeout=3.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        q, stamp = reader.get()

        if q is not None:
            return q

        time.sleep(0.01)

    raise RuntimeError(
        "No Leader joint frames received. "
        "Check 0x155/0x156/0x157 with candump."
    )


def clamp_joint_positions(q):
    result = []

    for value, (lo, hi) in zip(q, JOINT_LIMITS):
        result.append(min(max(value, lo), hi))

    return result


def slew_limit(previous, target, max_step):
    result = []

    for prev, tgt in zip(previous, target):
        delta = tgt - prev

        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step

        result.append(prev + delta)

    return result


def ensure_slave_enabled(arm, timeout=3.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        states = arm.get_joints_enable_status_list()

        if all(states):
            return True

        arm.enable()
        time.sleep(0.05)

    return all(arm.get_joints_enable_status_list())


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--live",
        action="store_true",
        help="Actually command the right arm",
    )

    args = parser.parse_args()

    print("=" * 70)

    if args.live:
        print("MODE: LIVE")
    else:
        print("MODE: DRY RUN — right arm will NOT move")

    print("=" * 70)

    # ------------------------------------------------------
    # 1. 连接右臂
    # ------------------------------------------------------

    slave = create_arm(RIGHT_CAN)
    leader_reader = None

    try:
        print("[1] Connecting right arm...")

        slave.connect()
        time.sleep(1.0)

        if not slave.is_ok():
            raise RuntimeError(
                "Right Piper communication is not OK."
            )

        js = slave.get_joint_angles()

        if js is None:
            raise RuntimeError(
                "Cannot read right arm joint state."
            )

        q_slave_0 = list(js.msg)

        status = slave.get_arm_status()

        print("Right initial q:")
        print(
            "  ["
            + ", ".join(f"{x:+.4f}" for x in q_slave_0)
            + "]"
        )

        if status is not None:
            print("Right status:")
            print(status.msg)

        # --------------------------------------------------
        # 2. 左臂切 Leader
        # --------------------------------------------------

        print()
        print(
            "[2] Left arm will enter zero-force Leader mode."
        )
        print("    Support the left arm before continuing.")

        input("Press Enter when ready...")

        set_left_leader()

        # --------------------------------------------------
        # 3. raw CAN 读取 Leader
        # --------------------------------------------------

        leader_reader = LeaderReader(LEFT_CAN)
        leader_reader.start()

        q_master_0 = wait_for_leader(leader_reader)

        print()
        print("Left initial q:")
        print(
            "  ["
            + ", ".join(f"{x:+.4f}" for x in q_master_0)
            + "]"
        )

        print()
        print("Relative teleoperation mapping:")
        print()
        print(
            "q_slave_des = q_slave_0 "
            "+ (q_master - q_master_0)"
        )

        # --------------------------------------------------
        # 4. LIVE 时才使能右臂
        # --------------------------------------------------

        if args.live:
            print()
            print("[3] Preparing right arm...")

            if not ensure_slave_enabled(slave):
                raise RuntimeError(
                    "Failed to enable all right-arm joints."
                )

            print(
                "Enable states:",
                slave.get_joints_enable_status_list(),
            )

            # 第一轮速度限制
            slave.set_speed_percent(20)

            # 明确设置关节模式
            slave.set_motion_mode("j")

            # 首条命令必须就是当前姿态
            slave.move_j(list(q_slave_0))

            print()
            print(
                "LIVE control is ready.\n"
                "Keep one hand near emergency stop.\n"
                "Move the LEFT arm slowly."
            )

            input("Press Enter to START right-arm following...")

        else:
            print()
            print(
                "DRY RUN started. Move LEFT arm slowly.\n"
                "Only q_des will be printed; "
                "RIGHT arm will not receive commands."
            )

        # --------------------------------------------------
        # 5. 控制循环
        # --------------------------------------------------

        period = 1.0 / CONTROL_HZ
        previous_cmd = list(q_slave_0)

        last_loop = time.monotonic()
        next_tick = last_loop

        last_print = 0.0

        while True:

            now = time.monotonic()

            q_master, master_stamp = leader_reader.get()

            if q_master is None:
                raise RuntimeError(
                    "Lost Leader joint state."
                )

            age = now - master_stamp

            # 100 ms 内完全没有新的 Leader 数据就停止更新命令
            if age > 0.100:
                print(
                    f"[WATCHDOG] Leader data stale: "
                    f"{age * 1000:.1f} ms"
                )

                time.sleep(period)
                continue

            # 相对位置映射
            target = []

            for i in range(6):
                delta = (
                    q_master[i] - q_master_0[i]
                )

                target.append(
                    q_slave_0[i]
                    + DIRECTION[i]
                    * SCALE[i]
                    * delta
                )

            target = clamp_joint_positions(target)

            # 软件速度限制
            dt = max(now - last_loop, 1e-4)

            max_step = MAX_JOINT_SPEED * dt

            command = slew_limit(
                previous_cmd,
                target,
                max_step,
            )

            if args.live:
                slave.move_j(list(command))

            previous_cmd = command
            last_loop = now

            # 10 Hz 打印
            if now - last_print > 0.1:
                mode = "LIVE" if args.live else "DRY"

                print(
                    f"[{mode}] "
                    f"age={age * 1000:5.1f} ms  "
                    "q_des=["
                    + ", ".join(
                        f"{x:+.3f}" for x in command
                    )
                    + "]"
                )

                last_print = now

            # 固定控制周期
            next_tick += period

            sleep_time = next_tick - time.monotonic()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 如果程序落后，不累计延迟
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\n[INFO] Teleoperation stopped.")

    finally:

        if leader_reader is not None:
            leader_reader.stop()

        # 非常重要：
        # 不在退出时 disable 右臂，否则机械臂可能失去支撑。
        try:
            slave.disconnect()
        except Exception:
            pass

        print("[INFO] Right arm CAN disconnected.")
        print(
            "[INFO] Left arm remains in Leader mode."
        )
        print(
            "[INFO] No motor-disable command was sent."
        )


if __name__ == "__main__":
    main()