#!/usr/bin/env python3
import json, socket, struct, time
import pigpio

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9000

GPIO_M1 = 18
GPIO_M2 = 12
GPIO_M3 = 13
GPIO_M4 = 19

ESC_FREQ = 400
PERIOD_US = int(1_000_000 / ESC_FREQ)  # 2500us

PULSE_NEUTRAL = 1500
PULSE_MIN = 1100
PULSE_MAX = 1900

COMMAND_TIMEOUT_S = 0.5

CMD_FMT = "<4sI4h"
CMD_MAGIC = b"SUB1"
CMD_SIZE = struct.calcsize(CMD_FMT)

def clamp(v, lo, hi): return max(lo, min(hi, v))

def i16_to_pct(v_i16: int) -> float:
    return clamp((float(v_i16) / 1000.0) * 100.0, -100.0, 100.0)

def pct_to_pulse(pct: float) -> int:
    pct = clamp(pct, -100.0, 100.0)
    if pct >= 0:
        return int(PULSE_NEUTRAL + (PULSE_MAX - PULSE_NEUTRAL) * (pct / 100.0))
    else:
        return int(PULSE_NEUTRAL + (PULSE_NEUTRAL - PULSE_MIN) * (pct / 100.0))

def setup_esc(pi, gpio):
    pi.set_mode(gpio, pigpio.OUTPUT)
    pi.set_servo_pulsewidth(gpio, 0)     # IMPORTANT: disable servo mode (50 Hz)
    pi.set_PWM_frequency(gpio, ESC_FREQ)
    pi.set_PWM_range(gpio, PERIOD_US)    # range=2500 => dutycycle units are microseconds
    pi.set_PWM_dutycycle(gpio, PULSE_NEUTRAL)

def set_pulse(pi, gpio, us):
    us = clamp(int(us), PULSE_MIN, PULSE_MAX)
    pi.set_PWM_dutycycle(gpio, us)

def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio daemon not running")

    for g in (GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4):
        setup_esc(pi, g)

    # Optional: print actual config
    for g in (GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4):
        print("GPIO", g,
              "freq", pi.get_PWM_frequency(g),
              "range", pi.get_PWM_range(g),
              "servo_pw", pi.get_servo_pulsewidth(g))

    print("[sub_motors_400hz] Arming at neutral...")
    time.sleep(3.0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(0.2)

    last_rx = time.time()
    last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

    print(f"[sub_motors_400hz] Listening UDP on {LISTEN_IP}:{LISTEN_PORT}")

    try:
        while True:
            try:
                data, _ = sock.recvfrom(2048)

                if len(data) >= CMD_SIZE:
                    magic, _seq, m1, m2, m3, m4 = struct.unpack(CMD_FMT, data[:CMD_SIZE])
                    if magic == CMD_MAGIC:
                        last = {"m1": i16_to_pct(m1), "m2": i16_to_pct(m2),
                                "m3": i16_to_pct(m3), "m4": i16_to_pct(m4)}
                        last_rx = time.time()
                        continue

                msg = json.loads(data.decode("utf-8", errors="ignore"))
                last = {
                    "m1": float(msg.get("m1", last["m1"])),
                    "m2": float(msg.get("m2", last["m2"])),
                    "m3": float(msg.get("m3", last["m3"])),
                    "m4": float(msg.get("m4", last["m4"])),
                }
                last_rx = time.time()

            except socket.timeout:
                pass
            except Exception:
                pass

            if time.time() - last_rx > COMMAND_TIMEOUT_S:
                last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

            set_pulse(pi, GPIO_M1, pct_to_pulse(last["m1"]))
            set_pulse(pi, GPIO_M2, pct_to_pulse(last["m2"]))
            set_pulse(pi, GPIO_M3, pct_to_pulse(last["m3"]))
            set_pulse(pi, GPIO_M4, pct_to_pulse(last["m4"]))

            time.sleep(0.005)

    finally:
        for g in (GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4):
            set_pulse(pi, g, PULSE_NEUTRAL)
        time.sleep(0.5)
        pi.stop()

if __name__ == "__main__":
    main()
