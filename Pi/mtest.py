#!/usr/bin/env python3
"""
mavlink_motor_test_pwm.py

Connects to a flight controller via MAVLink, then tests each motor by commanding
1600us PWM for 1 second, returning to 1500us PWM after each test.

Typical connection strings:
  - Serial (Linux):  /dev/ttyACM0
  - Serial (Win):    COM5
  - UDP (SITL):      udp:127.0.0.1:14550
  - TCP:             tcp:127.0.0.1:5760
"""

import argparse
import sys
import time
from pymavlink import mavutil


MAV_CMD_DO_MOTOR_TEST = 209  # MAVLink enum value


def wait_heartbeat(master, timeout_s=10):
    t0 = time.time()
    while True:
        hb = master.recv_match(type="HEARTBEAT", blocking=False)
        if hb is not None:
            return hb
        if time.time() - t0 > timeout_s:
            raise TimeoutError("Timed out waiting for HEARTBEAT.")
        time.sleep(0.05)


def arm(master, timeout_s=10):
    master.arducopter_arm()
    master.motors_armed_wait(timeout=timeout_s)


def disarm(master, timeout_s=10):
    master.arducopter_disarm()
    master.motors_disarmed_wait(timeout=timeout_s)


def motor_test_pwm(master, motor_index_1based: int, pwm_us: int, duration_s: float, timeout_s=3):
    """
    Uses MAV_CMD_DO_MOTOR_TEST:
      param1: motor instance (1-based)
      param2: throttle type (use 1 for PWM in many firmwares)
      param3: throttle value (PWM microseconds if throttle type is PWM)
      param4: duration (seconds)
      param5-7: unused
    """
    # Common throttle types (firmware-dependent):
    #  0: percent, 1: PWM, 2: RPM (varies by implementation)
    THROTTLE_TYPE_PWM = 1

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        MAV_CMD_DO_MOTOR_TEST,
        0,  # confirmation
        float(motor_index_1based),
        float(THROTTLE_TYPE_PWM),
        float(pwm_us),
        float(duration_s),
        0, 0, 0
    )

    # Try to wait briefly for an ACK (not all stacks ACK reliably)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="COMMAND_ACK", blocking=False)
        if msg is not None and msg.command == MAV_CMD_DO_MOTOR_TEST:
            # 0 = MAV_RESULT_ACCEPTED
            return msg.result
        time.sleep(0.02)
    return None  # no ack seen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", required=True, help="Connection string, e.g. /dev/ttyACM0 or udp:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=115200, help="Baudrate for serial connections (ignored for UDP/TCP)")
    ap.add_argument("--motors", type=int, default=4, help="How many motors to test (default: 4)")
    ap.add_argument("--test_pwm", type=int, default=1600, help="PWM for test (default: 1600)")
    ap.add_argument("--neutral_pwm", type=int, default=1500, help="Neutral PWM (default: 1500)")
    ap.add_argument("--test_time", type=float, default=1.0, help="Seconds per motor test (default: 1.0)")
    ap.add_argument("--neutral_time", type=float, default=0.3, help="Seconds to command neutral after each test (default: 0.3)")
    ap.add_argument("--arm", action="store_true", help="Attempt to ARM before testing (some firmwares require this)")
    ap.add_argument("--disarm_after", action="store_true", help="Attempt to DISARM after testing")
    args = ap.parse_args()

    print(f"[INFO] Connecting to: {args.conn}")
    master = mavutil.mavlink_connection(args.conn, baud=args.baud)

    print("[INFO] Waiting for heartbeat...")
    hb = wait_heartbeat(master, timeout_s=15)
    print(f"[OK] Heartbeat from system {master.target_system}, component {master.target_component}")
    print(f"[INFO] Autopilot={hb.autopilot}, type={hb.type}, base_mode={hb.base_mode}")

    if args.arm:
        print("[INFO] Arming...")
        try:
            arm(master, timeout_s=10)
            print("[OK] Armed.")
        except Exception as e:
            print(f"[WARN] Arm attempt failed or timed out: {e}")
            print("[WARN] Continuing anyway (motor output may be blocked unless armed).")

    def safe_neutral_all():
        # Best-effort: send neutral to all motors quickly.
        for m in range(1, args.motors + 1):
            motor_test_pwm(master, m, args.neutral_pwm, args.neutral_time)

    try:
        # Make sure we're at neutral before starting.
        print("[INFO] Sending neutral to all motors (best-effort)...")
        safe_neutral_all()
        time.sleep(0.2)

        for m in range(1, args.motors + 1):
            print(f"\n[TEST] Motor {m}: {args.test_pwm}us for {args.test_time:.2f}s")
            res = motor_test_pwm(master, m, args.test_pwm, args.test_time)
            if res is not None:
                print(f"[ACK] result={res}")
            time.sleep(args.test_time + 0.05)

            print(f"[INFO] Motor {m}: returning to neutral {args.neutral_pwm}us for {args.neutral_time:.2f}s")
            motor_test_pwm(master, m, args.neutral_pwm, args.neutral_time)
            time.sleep(args.neutral_time + 0.05)

        print("\n[OK] Motor test sequence complete.")

    except KeyboardInterrupt:
        print("\n[WARN] Ctrl+C received. Sending neutral to all motors...")
        safe_neutral_all()

    finally:
        if args.disarm_after:
            print("[INFO] Disarming...")
            try:
                disarm(master, timeout_s=10)
                print("[OK] Disarmed.")
            except Exception as e:
                print(f"[WARN] Disarm attempt failed or timed out: {e}")

        print("[INFO] Done.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
