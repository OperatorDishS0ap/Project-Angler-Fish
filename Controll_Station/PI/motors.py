import json
import socket
import struct
import time
import os
import fcntl
import pigpio

# -------------------------
# NETWORK
# -------------------------
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9000
SOCK_TIMEOUT_S = 0.2

# -------------------------
# GPIO MAP
# -------------------------
GPIO_M1 = 19
GPIO_M2 = 13
GPIO_M3 = 18
GPIO_M4 = 12
ALL_GPIOS = (GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4)

# -------------------------
# ESC / PWM SETTINGS
# -------------------------
ESC_FREQ_HZ = 400
PERIOD_US = int(1_000_000 / ESC_FREQ_HZ)  # 2500us @ 400Hz
PWM_RANGE = PERIOD_US                      # range=2500 => dutycycle "counts" == microseconds

PULSE_MIN = 1400
PULSE_MAX = 1600

PULSE_NEUTRAL = 1460

AVOID_LO = 1406
AVOID_HI = 1514
FORWARD_START = AVOID_HI + 1  # 1515

ARM_TIME_S = 3.0
LOOP_SLEEP_S = 0.005
COMMAND_TIMEOUT_S = 1.0
LOCK_PATH = "/tmp/anglerfish_motors.lock"
POWER_STATE_PATH = os.environ.get("ANGLERFISH_POWER_STATE_PATH", "/tmp/anglerfish_power_state.json")
POWER_STATE_CHECK_S = float(os.environ.get("ANGLERFISH_POWER_STATE_CHECK_S", "0.1"))
BATTERY_CUTOFF_THROTTLE_V = float(os.environ.get("ANGLERFISH_BATTERY_CUTOFF_THROTTLE_V", "5.8"))
PITCH_KP = float(os.environ.get("ANGLERFISH_STABILIZATION_PITCH_KP", "2.4"))
PITCH_KD = float(os.environ.get("ANGLERFISH_STABILIZATION_PITCH_KD", "0.18"))
ROLL_KP = float(os.environ.get("ANGLERFISH_STABILIZATION_ROLL_KP", "2.4"))
ROLL_KD = float(os.environ.get("ANGLERFISH_STABILIZATION_ROLL_KD", "0.18"))
ANGLE_ERROR_MARGIN_PCT = float(os.environ.get("ANGLERFISH_STABILIZATION_ERROR_MARGIN_PCT", "5.0"))
ANGLE_ERROR_MARGIN_MIN_DEG = float(os.environ.get("ANGLERFISH_STABILIZATION_ERROR_MARGIN_MIN_DEG", "3.0"))
STABILIZE_TARGET_PITCH_DEG = float(os.environ.get("ANGLERFISH_STABILIZATION_TARGET_PITCH_DEG", "0.0"))
STABILIZE_TARGET_ROLL_DEG = float(os.environ.get("ANGLERFISH_STABILIZATION_TARGET_ROLL_DEG", "0.0"))

# Small command deadband: treat tiny commands as neutral
PCT_DEADBAND = 2.0  # percent

# -------------------------
# LEGACY BINARY PROTOCOL
# -------------------------
CMD_FMT = "<4sI5h"
CMD_FMT_OLD = "<4sI4h"
CMD_MAGIC = b"SUB1"
CMD_SIZE = struct.calcsize(CMD_FMT)
CMD_SIZE_OLD = struct.calcsize(CMD_FMT_OLD)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def apply_error_margin(error_deg: float, target_deg: float, margin_pct: float) -> float:
    margin_deg = max(abs(target_deg) * (margin_pct / 100.0), ANGLE_ERROR_MARGIN_MIN_DEG)
    if abs(error_deg) <= margin_deg:
        return 0.0
    return error_deg


def mix_stabilization_motors(pitch_command: float, roll_command: float):
    m3 = clamp(-pitch_command - roll_command, -100.0, 100.0)
    m4 = clamp(-pitch_command + roll_command, -100.0, 100.0)
    return m3, m4


def acquire_single_instance_lock(lock_path: str):
    lock_file = open(lock_path, "w")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        raise SystemExit("Another motors.py instance is already running")
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file


def connect_pigpio_with_retry(retries: int = 5, delay_s: float = 0.5) -> pigpio.pi:
    last_pi = None
    for _ in range(retries):
        pi = pigpio.pi()
        last_pi = pi
        if pi.connected:
            return pi
        try:
            pi.stop()
        except Exception:
            pass
        time.sleep(delay_s)
    if last_pi is not None:
        try:
            last_pi.stop()
        except Exception:
            pass
    raise SystemExit("pigpio daemon not running or unstable. Start with: sudo systemctl restart pigpiod")


def pigpio_call(pi: pigpio.pi, method_name: str, *args):
    method = getattr(pi, method_name)
    try:
        return pi, method(*args)
    except (BrokenPipeError, OSError):
        try:
            pi.stop()
        except Exception:
            pass
        time.sleep(0.2)
        pi = connect_pigpio_with_retry(retries=5, delay_s=0.3)
        method = getattr(pi, method_name)
        return pi, method(*args)


def i16_to_pct(v_i16: int) -> float:
    # -1000..+1000 => -100..+100
    return clamp((float(v_i16) / 1000.0) * 100.0, -100.0, 100.0)


def pct_to_pulse_us(pct: float, pulse_min_us: int, pulse_max_us: int) -> int:
    pct = clamp(pct, -100.0, 100.0)

    if abs(pct) <= PCT_DEADBAND:
        return PULSE_NEUTRAL

    pulse_min_us = int(clamp(pulse_min_us, 1000, AVOID_LO))
    pulse_max_us = int(clamp(pulse_max_us, AVOID_HI, 2000))

    if pct < 0:
        # Reverse: -100 => 800, 0 => 1500
        # Linear map: pulse = 1500 + (1500-800)*(pct/100)
        return int(PULSE_NEUTRAL + (PULSE_NEUTRAL - pulse_min_us) * (pct / 100.0))

    # Forward: +0 => 1601, +100 => 2100
    # Linear map: pulse = 1601 + (2100-1601)*(pct/100)
    return int(FORWARD_START + (pulse_max_us - FORWARD_START) * (pct / 100.0))


def setup_pwm_esc(pi: pigpio.pi, gpio: int):
    pi, _ = pigpio_call(pi, "set_mode", gpio, pigpio.OUTPUT)

    # Ensure servo mode is off (servo mode is ~50Hz)
    pi, _ = pigpio_call(pi, "set_servo_pulsewidth", gpio, 0)

    # Configure PWM
    pi, _ = pigpio_call(pi, "set_PWM_frequency", gpio, ESC_FREQ_HZ)
    pi, _ = pigpio_call(pi, "set_PWM_range", gpio, PWM_RANGE)

    # Neutral output to arm
    pi, _ = pigpio_call(pi, "set_PWM_dutycycle", gpio, PULSE_NEUTRAL)
    return pi


def set_pulse_us(pi: pigpio.pi, gpio: int, pulse_us: int, pulse_min_us: int, pulse_max_us: int):
    pulse_us = int(pulse_us)

    # Avoid creep zone (1501..1600)
    if AVOID_LO < pulse_us <= AVOID_HI:
        pulse_us = PULSE_NEUTRAL

    pulse_min_us = int(clamp(pulse_min_us, 1000, AVOID_LO))
    pulse_max_us = int(clamp(pulse_max_us, AVOID_HI, 2000))
    pulse_us = clamp(pulse_us, pulse_min_us, pulse_max_us)
    pi, _ = pigpio_call(pi, "set_PWM_dutycycle", gpio, pulse_us)
    return pi


def read_power_state(path: str):
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        if not isinstance(data, dict):
            return {}
        return data
    except Exception:
        return {}


def main():
    lock_file = acquire_single_instance_lock(LOCK_PATH)
    pi = connect_pigpio_with_retry()

    # Setup all ESC outputs
    for g in ALL_GPIOS:
        pi = setup_pwm_esc(pi, g)

    # Debug: confirm PWM config (PWM mode; do NOT call get_servo_pulsewidth)
    for g in ALL_GPIOS:
        pi, pwm_freq = pigpio_call(pi, "get_PWM_frequency", g)
        pi, pwm_range = pigpio_call(pi, "get_PWM_range", g)
        pi, duty = pigpio_call(pi, "get_PWM_dutycycle", g)
        print(
            f"GPIO {g}: pwm_freq={pwm_freq}Hz "
            f"pwm_range={pwm_range} duty={duty}"
        )

    print(f"[sub_motors_400hz] Arming at neutral ({PULSE_NEUTRAL}us) for {ARM_TIME_S:.1f}s ...")
    time.sleep(ARM_TIME_S)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(SOCK_TIMEOUT_S)

    last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}
    arm_requested = False
    arm_active = False
    arm_started_at = None
    last_command_ts = time.time()
    pulse_min_us = PULSE_MIN
    pulse_max_us = PULSE_MAX
    battery_cutoff_active = False
    esc_overtemp_active = False
    last_battery_v = None
    last_esc_temp_c = None
    last_power_state_check = 0.0
    attitude_ready = False
    pitch_deg = 0.0
    roll_deg = 0.0
    pitch_rate_dps = 0.0
    roll_rate_dps = 0.0
    stabilize_horizontal = False
    last_stabilize_horizontal = False

    print(f"[sub_motors_400hz] Listening UDP on {LISTEN_IP}:{LISTEN_PORT}")

    try:
        while True:
            # Receive
            try:
                data, _addr = sock.recvfrom(2048)

                # 1) Legacy binary
                if len(data) >= CMD_SIZE_OLD and data[:4] == CMD_MAGIC:
                    if len(data) >= CMD_SIZE:
                        magic, _seq, m1_i16, m2_i16, m3_i16, m4_i16, arm_i16 = struct.unpack(CMD_FMT, data[:CMD_SIZE])
                    else:
                        magic, _seq, m1_i16, m2_i16, m3_i16, m4_i16 = struct.unpack(CMD_FMT_OLD, data[:CMD_SIZE_OLD])
                        arm_i16 = 1000  # Backward-compatible: old packets are treated as armed.

                    if magic != CMD_MAGIC:
                        raise ValueError("Not legacy magic")

                    last = {
                        "m1": i16_to_pct(m1_i16),
                        "m2": i16_to_pct(m2_i16),
                        "m3": i16_to_pct(m3_i16),
                        "m4": i16_to_pct(m4_i16),
                    }
                    arm_requested = arm_i16 > 0
                    last_command_ts = time.time()

                else:
                    # JSON fallback
                    msg = json.loads(data.decode("utf-8", errors="ignore"))

                    if msg.get("type") == "tune":
                        if "pulse_min_us" in msg:
                            pulse_min_us = int(clamp(float(msg["pulse_min_us"]), 1000, AVOID_LO))
                        if "pulse_max_us" in msg:
                            pulse_max_us = int(clamp(float(msg["pulse_max_us"]), AVOID_HI, 2000))

                        if pulse_min_us >= pulse_max_us:
                            pulse_min_us = min(pulse_min_us, AVOID_LO)
                            pulse_max_us = max(pulse_max_us, AVOID_HI)
                        print(f"[sub_motors_400hz] Tune update: PULSE_MIN={pulse_min_us} PULSE_MAX={pulse_max_us}")
                    else:
                        last = {
                            "m1": float(msg.get("m1", last["m1"])),
                            "m2": float(msg.get("m2", last["m2"])),
                            "m3": float(msg.get("m3", last["m3"])),
                            "m4": float(msg.get("m4", last["m4"])),
                        }
                        arm_requested = bool(msg.get("arm", arm_requested))
                        stabilize_horizontal = bool(msg.get("stabilize_horizontal", stabilize_horizontal))
                        last_command_ts = time.time()

                        if stabilize_horizontal != last_stabilize_horizontal:
                            if stabilize_horizontal:
                                print(
                                    f"[sub_motors_400hz] Horizontal stabilization ENABLED "
                                    f"(target_pitch={STABILIZE_TARGET_PITCH_DEG:.2f}, target_roll={STABILIZE_TARGET_ROLL_DEG:.2f})"
                                )
                            else:
                                print("[sub_motors_400hz] Horizontal stabilization DISABLED")
                            last_stabilize_horizontal = stabilize_horizontal


            except socket.timeout:
                pass
            except Exception:
                pass

            now = time.time()
            if (now - last_power_state_check) >= max(0.02, POWER_STATE_CHECK_S):
                last_power_state_check = now
                power_state = read_power_state(POWER_STATE_PATH)
                battery_v = power_state.get("battery_v")
                battery_cutoff_state = bool(power_state.get("battery_cutoff_active", False))
                esc_overtemp_state = bool(power_state.get("esc_overtemp_active", False))
                esc_max_temp_c = power_state.get("esc_max_temp_c")
                attitude_ready = bool(power_state.get("attitude_ready", False))
                pitch_deg = float(power_state.get("pitch_deg", 0.0) or 0.0)
                roll_deg = float(power_state.get("roll_deg", 0.0) or 0.0)
                pitch_rate_dps = float(power_state.get("pitch_rate_dps", 0.0) or 0.0)
                roll_rate_dps = float(power_state.get("roll_rate_dps", 0.0) or 0.0)
                if battery_v is not None:
                    try:
                        last_battery_v = float(battery_v)
                    except Exception:
                        pass
                if esc_max_temp_c is not None:
                    try:
                        last_esc_temp_c = float(esc_max_temp_c)
                    except Exception:
                        pass

                if battery_cutoff_state and not battery_cutoff_active:
                    msg_v = f"{last_battery_v:.3f}V" if last_battery_v is not None else "unknown voltage"
                    print(f"[sub_motors_400hz] Battery cutoff ACTIVE ({msg_v}); motor output blocked.")
                elif (not battery_cutoff_state) and battery_cutoff_active:
                    msg_v = f"{last_battery_v:.3f}V" if last_battery_v is not None else "unknown voltage"
                    print(f"[sub_motors_400hz] Battery cutoff CLEARED ({msg_v}); motor output allowed.")
                battery_cutoff_active = battery_cutoff_state

                if esc_overtemp_state and not esc_overtemp_active:
                    msg_t = f"{last_esc_temp_c:.2f}C" if last_esc_temp_c is not None else "unknown temp"
                    print(f"[sub_motors_400hz] ESC overtemp ACTIVE ({msg_t}); motor output blocked.")
                elif (not esc_overtemp_state) and esc_overtemp_active:
                    msg_t = f"{last_esc_temp_c:.2f}C" if last_esc_temp_c is not None else "unknown temp"
                    print(f"[sub_motors_400hz] ESC overtemp CLEARED ({msg_t}); motor output allowed.")
                esc_overtemp_active = esc_overtemp_state

            if (time.time() - last_command_ts) > COMMAND_TIMEOUT_S:
                arm_requested = False
                last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

            throttle_active = any(abs(v) > PCT_DEADBAND for v in last.values())
            hard_cutoff = (last_battery_v is not None and last_battery_v <= BATTERY_CUTOFF_THROTTLE_V)
            if (battery_cutoff_active and not throttle_active) or hard_cutoff or esc_overtemp_active:
                arm_requested = False
                last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

            # Arm/disarm state machine for ESC safety.
            if not arm_requested:
                if arm_active:
                    print("[sub_motors_400hz] DISARM command received; forcing neutral.")
                arm_active = False
                arm_started_at = None
                last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}
            else:
                if arm_started_at is None:
                    arm_started_at = time.time()
                    arm_active = False
                    print(f"[sub_motors_400hz] ARM command received; holding neutral for {ARM_TIME_S:.1f}s.")
                elif not arm_active and (time.time() - arm_started_at) >= ARM_TIME_S:
                    arm_active = True
                    print("[sub_motors_400hz] ESC output armed.")

            output = dict(last)
            manual_attitude_override = abs(output["m3"]) > PCT_DEADBAND or abs(output["m4"]) > PCT_DEADBAND
            stabilization_active = arm_active and stabilize_horizontal and attitude_ready and not manual_attitude_override

            if stabilization_active:
                pitch_error = apply_error_margin(
                    STABILIZE_TARGET_PITCH_DEG - pitch_deg,
                    STABILIZE_TARGET_PITCH_DEG,
                    ANGLE_ERROR_MARGIN_PCT,
                )
                roll_error = apply_error_margin(
                    STABILIZE_TARGET_ROLL_DEG - roll_deg,
                    STABILIZE_TARGET_ROLL_DEG,
                    ANGLE_ERROR_MARGIN_PCT,
                )
                pitch_command = (PITCH_KP * pitch_error) - (PITCH_KD * pitch_rate_dps)
                roll_command = (ROLL_KP * roll_error) - (ROLL_KD * roll_rate_dps)
                stabilize_m3, stabilize_m4 = mix_stabilization_motors(pitch_command, roll_command)
                output["m3"] = clamp(output["m3"] + stabilize_m3, -100.0, 100.0)
                output["m4"] = clamp(output["m4"] + stabilize_m4, -100.0, 100.0)

            # Apply outputs (PWM @ 400Hz)
            if arm_active:
                pi = set_pulse_us(pi, GPIO_M1, pct_to_pulse_us(output["m1"], pulse_min_us, pulse_max_us), pulse_min_us, pulse_max_us)
                pi = set_pulse_us(pi, GPIO_M2, pct_to_pulse_us(output["m2"], pulse_min_us, pulse_max_us), pulse_min_us, pulse_max_us)
                pi = set_pulse_us(pi, GPIO_M3, pct_to_pulse_us(output["m3"], pulse_min_us, pulse_max_us), pulse_min_us, pulse_max_us)
                pi = set_pulse_us(pi, GPIO_M4, pct_to_pulse_us(output["m4"], pulse_min_us, pulse_max_us), pulse_min_us, pulse_max_us)
            else:
                for g in ALL_GPIOS:
                    pi = set_pulse_us(pi, g, PULSE_NEUTRAL, pulse_min_us, pulse_max_us)

            time.sleep(LOOP_SLEEP_S)

    finally:
        # Neutral on exit
        for g in ALL_GPIOS:
            pi = set_pulse_us(pi, g, PULSE_NEUTRAL, pulse_min_us, pulse_max_us)
        time.sleep(0.5)
        pi.stop()
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        except Exception:
            pass
        try:
            lock_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
