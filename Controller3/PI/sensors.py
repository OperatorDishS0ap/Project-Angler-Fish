#!/usr/bin/env python3
import json
import os
import random
import socket
import time
import math
from typing import Optional
from bar30_sensor import init_bar30, read_bar30
from mpu6050_sensor import init_imu_with_retry, read_imu
from ads1015_sensor import init_ads1015, read_ads1015

USE_BROADCAST = os.environ.get("ANGLERFISH_USE_BROADCAST", "1") == "1"
BROADCAST_IP = os.environ.get("ANGLERFISH_BROADCAST_IP", "255.255.255.255")
PC_IP = os.environ.get("ANGLERFISH_PC_IP", "192.168.137.1")  # used when ANGLERFISH_USE_BROADCAST=0
PC_PORT = int(os.environ.get("ANGLERFISH_PC_PORT", "9001"))
ENABLE_BAR30 = os.environ.get("ANGLERFISH_ENABLE_BAR30", "0") == "1"
ENABLE_MPU6050 = os.environ.get("ANGLERFISH_ENABLE_MPU6050", "0") == "1"
ENABLE_ADS1015 = os.environ.get("ANGLERFISH_ENABLE_ADS1015", "1") == "1"
DEBUG = os.environ.get("ANGLERFISH_SENSOR_DEBUG", "1") == "1"

TELEMETRY_SEND_HZ = float(os.environ.get("ANGLERFISH_TELEMETRY_SEND_HZ", "2.0"))
BAR30_HZ = float(os.environ.get("ANGLERFISH_BAR30_HZ", "2.0"))
MPU6050_HZ = float(os.environ.get("ANGLERFISH_MPU6050_HZ", "20.0"))
PI_TEMP_HZ = float(os.environ.get("ANGLERFISH_PI_TEMP_HZ", "2.0"))
BATTERY_HZ = float(os.environ.get("ANGLERFISH_BATTERY_HZ", "2.0"))
ADS1015_HZ = float(os.environ.get("ANGLERFISH_ADS1015_HZ", "8.0"))
BATTERY_CUTOFF_V = float(os.environ.get("ANGLERFISH_BATTERY_CUTOFF_V", "7.2"))
BATTERY_CUTOFF_CLEAR_V = float(os.environ.get("ANGLERFISH_BATTERY_CUTOFF_CLEAR_V", "7.3"))
ESC_OVERTEMP_C = float(os.environ.get("ANGLERFISH_ESC_OVERTEMP_C", "90.0"))
ESC_OVERTEMP_CLEAR_C = float(os.environ.get("ANGLERFISH_ESC_OVERTEMP_CLEAR_C", "85.0"))
BATTERY_EMA_ALPHA = float(os.environ.get("ANGLERFISH_BATTERY_EMA_ALPHA", "0.2"))
POWER_STATE_PATH = os.environ.get("ANGLERFISH_POWER_STATE_PATH", "/tmp/anglerfish_power_state.json")


def _hz_to_interval(hz: float, min_hz: float = 0.1) -> float:
    return 1.0 / max(min_hz, hz)


def _ema(prev_value: Optional[float], sample: float, alpha: float) -> float:
    if prev_value is None:
        return float(sample)
    a = max(0.0, min(1.0, alpha))
    return (a * float(sample)) + ((1.0 - a) * float(prev_value))


def _write_power_state(path: str, battery_v: float, cutoff_active: bool, esc_overtemp_active: bool, esc_max_temp_c: float):
    payload = {
        "ts": time.time(),
        "battery_v": float(battery_v),
        "battery_cutoff_active": bool(cutoff_active),
        "esc_overtemp_active": bool(esc_overtemp_active),
        "esc_max_temp_c": float(esc_max_temp_c),
    }
    tmp_path = f"{path}.tmp"
    try:
        with open(tmp_path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh)
        os.replace(tmp_path, path)
    except Exception:
        try:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
        except OSError:
            pass



def main():
    sensor = init_bar30() if ENABLE_BAR30 else None
    imu = init_imu_with_retry() if ENABLE_MPU6050 else None
    ads = init_ads1015() if ENABLE_ADS1015 else None

    print(
        f"[sensors] Sensor toggles: BAR30={'ON' if ENABLE_BAR30 else 'OFF'} "
        f"MPU6050={'ON' if ENABLE_MPU6050 else 'OFF'} "
        f"ADS1015={'ON' if ENABLE_ADS1015 else 'OFF'}"
    )

    if ENABLE_BAR30 and sensor is None:
        print("[sensors] BAR30 enabled but unavailable; publishing zeros for BAR30 fields")
    if ENABLE_MPU6050 and imu is None:
        print("[sensors] MPU6050 enabled but unavailable; publishing zeros for MPU fields")
    if ENABLE_ADS1015 and ads is None:
        print("[sensors] ADS1015 enabled but unavailable; publishing zeros for ADC fields")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if USE_BROADCAST:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    target_ip = BROADCAST_IP if USE_BROADCAST else PC_IP
    print(f"[sensors] Telemetry target: {target_ip}:{PC_PORT} (broadcast={USE_BROADCAST})")
    if DEBUG:
        print(f"[sensors] PC_IP={PC_IP}, BROADCAST_IP={BROADCAST_IP}")
        print(
            "[sensors] Rates (Hz): "
            f"send={TELEMETRY_SEND_HZ}, bar30={BAR30_HZ}, mpu6050={MPU6050_HZ}, "
            f"pi_temp={PI_TEMP_HZ}, battery={BATTERY_HZ}, ads1015={ADS1015_HZ}"
        )

    send_interval = _hz_to_interval(TELEMETRY_SEND_HZ)
    bar30_interval = _hz_to_interval(BAR30_HZ)
    mpu_interval = _hz_to_interval(MPU6050_HZ)
    pi_temp_interval = _hz_to_interval(PI_TEMP_HZ)
    battery_interval = _hz_to_interval(BATTERY_HZ)
    ads_interval = _hz_to_interval(ADS1015_HZ)

    loop_sleep_s = 0.01
    speed = 0.0  # Integrated speed from acceleration
    GRAVITY = 9.8  # m/s²

    telemetry = {
        "battery_v": 12.0,
        "depth_m": 0.0,
        "pressure_bar": 0.0,
        "water_temp_c": 0.0,
        "enclosure_temp_c": 0.0,
        "esc_temp_1_c": 0.0,
        "esc_temp_2_c": 0.0,
        "esc_max_temp_c": 0.0,
        "esc_overtemp_active": False,
        "speed_mps": 0.0,
        "accel_mps2": 0.0,
        "current_a": 0.0,
        "current_adc_v": 0.0,
        "pi_temp_c": 0.0,
        "battery_cutoff_active": False,
    }

    now = time.time()
    next_send_ts = now
    next_bar30_ts = now
    next_mpu_ts = now
    next_pi_temp_ts = now
    next_battery_ts = now
    next_ads_ts = now

    last_mpu_update_ts = now
    last_imu_reconnect_try = 0.0
    last_ads_reconnect_try = 0.0
    battery_filtered_v: Optional[float] = None
    battery_cutoff_active = False
    esc_overtemp_active = False

    while True:
        now = time.time()

        if now >= next_bar30_ts:
            if ENABLE_BAR30:
                pressure, temp_env, depth = read_bar30(sensor)
            else:
                pressure, temp_env, depth = 0.0, 0.0, 0.0

            telemetry["pressure_bar"] = round(float(pressure), 3)
            telemetry["water_temp_c"] = round(float(temp_env), 3)
            telemetry["depth_m"] = round(float(depth), 4)
            next_bar30_ts = now + bar30_interval

        if now >= next_mpu_ts:
            accel = {"x": 0.0, "y": 0.0, "z": GRAVITY}
            imu_temp = 0.0

            if imu is not None:
                try:
                    accel, imu_temp = read_imu(imu)
                except OSError as exc:
                    print(f"[sensors] MPU6050 read failed: {exc}")
                    imu = None

            if ENABLE_MPU6050 and imu is None and (now - last_imu_reconnect_try) >= 2.0:
                last_imu_reconnect_try = now
                imu = init_imu_with_retry(retries=1, delay_s=0.0)

            accel_magnitude = math.sqrt(accel["x"] ** 2 + accel["y"] ** 2 + accel["z"] ** 2)
            motion_accel = max(0.0, accel_magnitude - GRAVITY)

            elapsed = max(0.0, now - last_mpu_update_ts)
            speed += motion_accel * elapsed
            decay_factor = math.exp(-elapsed / 5.0)
            speed *= decay_factor
            speed = max(0.0, speed)
            last_mpu_update_ts = now

            telemetry["enclosure_temp_c"] = round(float(imu_temp), 3)
            telemetry["speed_mps"] = round(float(speed), 4)
            telemetry["accel_mps2"] = round(float(motion_accel), 4)
            next_mpu_ts = now + mpu_interval

        if now >= next_pi_temp_ts:
            try:
                with open("/sys/class/thermal/thermal_zone0/temp") as _tf:
                    telemetry["pi_temp_c"] = round(float(_tf.read().strip()) / 1000.0, 1)
            except Exception:
                telemetry["pi_temp_c"] = 0.0
            next_pi_temp_ts = now + pi_temp_interval

        if now >= next_battery_ts:
            if not ENABLE_ADS1015:
                battery_sample_v = 12.0 + (0.2 * random.random())
                battery_filtered_v = _ema(battery_filtered_v, battery_sample_v, BATTERY_EMA_ALPHA)
                telemetry["battery_v"] = round(float(battery_filtered_v), 3)
            next_battery_ts = now + battery_interval

        if now >= next_ads_ts:
            if ads is not None:
                esc1_c, esc2_c, battery_v, current_a, current_adc_v = read_ads1015(ads)
                telemetry["esc_temp_1_c"] = round(float(esc1_c), 2)
                telemetry["esc_temp_2_c"] = round(float(esc2_c), 2)
                battery_filtered_v = _ema(battery_filtered_v, float(battery_v), BATTERY_EMA_ALPHA)
                telemetry["battery_v"] = round(float(battery_filtered_v), 3)
                telemetry["current_a"] = round(float(current_a), 3)
                telemetry["current_adc_v"] = round(float(current_adc_v), 3)

            if ENABLE_ADS1015 and ads is None and (now - last_ads_reconnect_try) >= 2.0:
                last_ads_reconnect_try = now
                ads = init_ads1015()

            next_ads_ts = now + ads_interval

        battery_v_now = float(telemetry.get("battery_v", 0.0))
        if not battery_cutoff_active and battery_v_now <= BATTERY_CUTOFF_V:
            battery_cutoff_active = True
            print(f"[sensors] Battery cutoff activated at {battery_v_now:.3f}V")
        elif battery_cutoff_active and battery_v_now >= BATTERY_CUTOFF_CLEAR_V:
            battery_cutoff_active = False
            print(f"[sensors] Battery cutoff cleared at {battery_v_now:.3f}V")
        telemetry["battery_cutoff_active"] = battery_cutoff_active

        esc_max_temp_c = max(float(telemetry.get("esc_temp_1_c", 0.0)), float(telemetry.get("esc_temp_2_c", 0.0)))
        if not esc_overtemp_active and esc_max_temp_c >= ESC_OVERTEMP_C:
            esc_overtemp_active = True
            print(f"[sensors] ESC overtemp activated at {esc_max_temp_c:.2f}C")
        elif esc_overtemp_active and esc_max_temp_c <= ESC_OVERTEMP_CLEAR_C:
            esc_overtemp_active = False
            print(f"[sensors] ESC overtemp cleared at {esc_max_temp_c:.2f}C")
        telemetry["esc_max_temp_c"] = round(esc_max_temp_c, 2)
        telemetry["esc_overtemp_active"] = esc_overtemp_active

        if now >= next_send_ts:
            payload = {
                "ts": now,
                "type": "telemetry",
                "telemetry": telemetry,
            }
            try:
                sock.sendto(json.dumps(payload).encode("utf-8"), (target_ip, PC_PORT))
                _write_power_state(
                    POWER_STATE_PATH,
                    battery_v_now,
                    battery_cutoff_active,
                    esc_overtemp_active,
                    esc_max_temp_c,
                )
                if DEBUG:
                    print(
                        f"[sensors] Sent telemetry to {target_ip}:{PC_PORT}: "
                        f"pi_temp_c={telemetry['pi_temp_c']}, battery_v={telemetry['battery_v']}, "
                        f"current_a={telemetry['current_a']}"
                    )
            except Exception as exc:
                print(f"[sensors] ERROR sending telemetry: {exc}")
            next_send_ts = now + send_interval

        time.sleep(loop_sleep_s)

if __name__ == "__main__":
    main()
