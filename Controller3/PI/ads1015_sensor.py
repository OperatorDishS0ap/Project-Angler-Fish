#!/usr/bin/env python3
import math
import os
from typing import Optional, Tuple

try:
    import board  # type: ignore[import-not-found]
    import busio  # type: ignore[import-not-found]
    import adafruit_ads1x15.ads1015 as ADS  # type: ignore[import-not-found]
    from adafruit_ads1x15.analog_in import AnalogIn  # type: ignore[import-not-found]
except ImportError:
    board = None
    busio = None
    ADS = None
    AnalogIn = None


ADC_ADDRESS = int(os.environ.get("ANGLERFISH_ADS1015_ADDR", "0x49"), 0)
ADC_GAIN = float(os.environ.get("ANGLERFISH_ADS1015_GAIN", "1"))
ADC_VREF = float(os.environ.get("ANGLERFISH_ADC_VREF", "3.3"))

# Thermistor model/config
THERM_R_FIXED_OHM = float(os.environ.get("ANGLERFISH_ESC_THERM_R_FIXED_OHM", "10000"))
THERM_R0_OHM = float(os.environ.get("ANGLERFISH_ESC_THERM_R0_OHM", "10000"))
THERM_BETA = float(os.environ.get("ANGLERFISH_ESC_THERM_BETA", "3950"))
THERM_T0_C = float(os.environ.get("ANGLERFISH_ESC_THERM_T0_C", "25.0"))
THERM_TO_GND = os.environ.get("ANGLERFISH_ESC_THERM_TO_GND", "1") == "1"

# Battery and current scaling
BATTERY_DIVIDER_RATIO = float(os.environ.get("ANGLERFISH_BATTERY_DIVIDER_RATIO", "2.58"))
CURRENT_OFFSET_V = float(os.environ.get("ANGLERFISH_CURRENT_OFFSET_V", "2.5"))
CURRENT_SCALE_A_PER_V = float(os.environ.get("ANGLERFISH_CURRENT_SCALE_A_PER_V", "10.0"))


def apply_ads1015_tuning(tuning: dict):
    global THERM_R_FIXED_OHM
    global THERM_R0_OHM
    global THERM_BETA
    global THERM_T0_C
    global THERM_TO_GND
    global BATTERY_DIVIDER_RATIO
    global CURRENT_OFFSET_V
    global CURRENT_SCALE_A_PER_V

    if not isinstance(tuning, dict):
        return

    try:
        if "esc_therm_r_fixed_ohm" in tuning:
            THERM_R_FIXED_OHM = max(1.0, float(tuning["esc_therm_r_fixed_ohm"]))
        if "esc_therm_r0_ohm" in tuning:
            THERM_R0_OHM = max(1.0, float(tuning["esc_therm_r0_ohm"]))
        if "esc_therm_beta" in tuning:
            THERM_BETA = max(1.0, float(tuning["esc_therm_beta"]))
        if "esc_therm_t0_c" in tuning:
            THERM_T0_C = float(tuning["esc_therm_t0_c"])
        if "esc_therm_to_gnd" in tuning:
            THERM_TO_GND = bool(tuning["esc_therm_to_gnd"])

        if "battery_divider_ratio" in tuning:
            BATTERY_DIVIDER_RATIO = max(0.01, float(tuning["battery_divider_ratio"]))
        if "current_offset_v" in tuning:
            CURRENT_OFFSET_V = float(tuning["current_offset_v"])
        if "current_scale_a_per_v" in tuning:
            CURRENT_SCALE_A_PER_V = max(0.0, float(tuning["current_scale_a_per_v"]))
    except Exception as exc:
        print(f"[ads1015] Tuning update ignored due to invalid values: {exc}")


def _safe_float(value: float, fallback: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return fallback


def _voltage_to_therm_resistance(v_out: float) -> Optional[float]:
    v_out = _safe_float(v_out, 0.0)
    if v_out <= 0.0 or v_out >= ADC_VREF:
        return None

    if THERM_TO_GND:
        # Divider: VREF -> R_FIXED -> Vout -> THERM -> GND
        denom = ADC_VREF - v_out
        if denom <= 0.0:
            return None
        return THERM_R_FIXED_OHM * (v_out / denom)

    # Divider: VREF -> THERM -> Vout -> R_FIXED -> GND
    return THERM_R_FIXED_OHM * ((ADC_VREF / v_out) - 1.0)


def _resistance_to_temp_c(resistance_ohm: Optional[float]) -> float:
    if resistance_ohm is None or resistance_ohm <= 0.0:
        return 0.0

    t0_k = THERM_T0_C + 273.15
    try:
        inv_t = (1.0 / t0_k) + (1.0 / THERM_BETA) * math.log(resistance_ohm / THERM_R0_OHM)
        return (1.0 / inv_t) - 273.15
    except Exception:
        return 0.0


def init_ads1015():
    if None in (board, busio, ADS, AnalogIn):
        print("[ads1015] Missing dependency: install adafruit-circuitpython-ads1x15")
        return None

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1015(i2c, address=ADC_ADDRESS)
        ads.gain = ADC_GAIN
        # Channel indices 0-3 work across all adafruit-ads1x15 library versions.
        channels = {
            "ch0": AnalogIn(ads, 0),
            "ch1": AnalogIn(ads, 1),
            "ch2": AnalogIn(ads, 2),
            "ch3": AnalogIn(ads, 3),
        }
        print(f"[ads1015] Initialized at 0x{ADC_ADDRESS:02x} gain={ADC_GAIN}")
        return channels
    except Exception as exc:
        print(f"[ads1015] Init failed: {exc}")
        return None


def read_ads1015(channels) -> Tuple[float, float, float, float, float]:
    if channels is None:
        return 0.0, 0.0, 0.0, 0.0, 0.0

    try:
        v0 = _safe_float(channels["ch0"].voltage, 0.0)
        v1 = _safe_float(channels["ch1"].voltage, 0.0)
        v2 = _safe_float(channels["ch2"].voltage, 0.0)
        v3 = _safe_float(channels["ch3"].voltage, 0.0)

        esc_temp_1_c = _resistance_to_temp_c(_voltage_to_therm_resistance(v0))
        esc_temp_2_c = _resistance_to_temp_c(_voltage_to_therm_resistance(v1))
        battery_v = max(0.0, v2 * BATTERY_DIVIDER_RATIO)
        current_a = max(0.0, (v3 - CURRENT_OFFSET_V) * CURRENT_SCALE_A_PER_V)

        return esc_temp_1_c, esc_temp_2_c, battery_v, current_a, v3
    except Exception as exc:
        print(f"[ads1015] Read failed: {exc}")
        return 0.0, 0.0, 0.0, 0.0, 0.0
