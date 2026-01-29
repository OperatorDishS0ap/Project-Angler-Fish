#!/usr/bin/env python3
"""
mavlink_motor_test_pwm_serial0.py

Connect to FC over UART (/dev/serial0 @ 115200), then test motors 1..4:
- command 1600us for 1 second
- then command 1500us (neutral) briefly

Run:
  pip install pymavlink
  python3 mavlink_motor_test_pwm_serial0.py
"""

import time
from pymavlink import mavutil

MAV_CMD_DO_MOTOR_TEST = 209
THROTTLE_TYPE_PWM = 1  # common: 1 = PWM (firmware-dependent)

CONN = "/dev/serial0"
BAUD = 115200

MOTORS = 4
TEST_PWM = 1600
NEUTRAL_PWM = 1500
TEST_TIME_S = 1.0
NEUTRAL_TIME_S = 0.3


def wait_heartbeat(master, timeout_s=15):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hb = master.recv_match(type="HEARTBEAT", blocking=False)
        if hb is not None:
            return hb
        time.sleep(0.05)
    raise TimeoutError("Timed out waiting for HEARTBEAT.")


def motor_test_pwm(master, motor_index_1based: int, pwm_us: int, duration_s: float, ack_timeout_s=2.0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        MAV_CMD_DO_MOTOR_TEST,
        0,  # confirmation
        float(motor_index_1based),   # param1 motor instance (1-based)
        float(THROTTLE_TYPE_PWM),    # param2 throttle type
        float(pwm_us),               # param3 throttle value (PWM us)
        float(duration_s),           # param4 duration
        0, 0, 0
    )

    # Best-effort ACK wait (some firmwares may not ACK consistently)
    t0 = time.time()
    while time.time() - t0 < ack_timeout_s:
        msg = master.recv_match(type="COMMAND_ACK", blocking=False)
        if msg and msg.command == MAV_CMD_DO_MOTOR_TEST:
            return msg.result
        time.sleep(0.02)
    return None


def main():
    print(f"[INFO] Connecting: {CONN} @ {BAUD}")
    master = mavutil.mavlink_connection(CONN, baud=BAUD)

    print("[INFO] Waiting for heartbeat...")
    hb = wait_heartbeat(master)
    print(f"[OK] Heartbeat from sys={master.target_system} comp={master.target_component}")

    # Optional: if your firmware requires arming for motor output, uncomment:
    # print("[INFO] Arming...")
    # master.arducopter_arm()
    # master.motors_armed_wait(timeout=10)
    # print("[OK] Armed")

    # Neutral all (best effort)
    print("[INFO] Sending neutral to all motors...")
    for m in range(1, MOTORS + 1):
        motor_test_pwm(master, m, NEUTRAL_PWM, NEUTRAL_TIME_S)
    time.sleep(0.2)

    for m in range(1, MOTORS + 1):
        print(f"\n[TEST] Motor {m}: {TEST_PWM}us for {TEST_TIME_S:.2f}s")
        res = motor_test_pwm(master, m, TEST_PWM, TEST_TIME_S)
        if res is not None:
            print(f"[ACK] result={res}")

        time.sleep(TEST_TIME_S + 0.05)

        print(f"[INFO] Motor {m}: neutral {NEUTRAL_PWM}us for {NEUTRAL_TIME_S:.2f}s")
        motor_test_pwm(master, m, NEUTRAL_PWM, NEUTRAL_TIME_S)
        time.sleep(NEUTRAL_TIME_S + 0.05)

    print("\n[OK] Motor test sequence complete.")

    # Optional: disarm after (uncomment if you armed above)
    # print("[INFO] Disarming...")
    # master.arducopter_disarm()
    # master.motors_disarmed_wait(timeout=10)
    # print("[OK] Disarmed")


if __name__ == "__main__":
    main()
