#!/usr/bin/env python3
"""直接调用爬壁工程 gamepad 脚本做 init，验证 COM4 是否有回传。"""

import sys
import time
from pathlib import Path

REF = Path(r"C:\Users\DELL\Desktop\new\爬壁\ESP32-S3-RS485-CAN-Demo")
sys.path.insert(0, str(REF))

from gamepad_hybrid_motor_control import (  # noqa: E402
    can_enable,
    can_init,
    can_open_serial_port,
    can_send_hex_command,
    can_set_accel,
    can_set_current_limit,
    can_set_speed_mode,
    can_start,
)


def run_motor(ser, motor: str) -> int:
    total = 0
    steps = [
        ("init", can_init(motor, "com")),
        ("speed_mode", can_set_speed_mode(motor, "com")),
        ("current", can_set_current_limit(motor, "com", 2.0)),
        ("accel", can_set_accel(motor, "com", 10.0)),
        ("enable", can_enable(motor, "com")),
        ("start", can_start(motor, "com")),
    ]
    for name, hx in steps:
        ser.reset_input_buffer()
        can_send_hex_command(ser, hx, f"{motor}-{name}", verbose=True)
        time.sleep(1.0)
        rx = ser.read(512)
        print(f"  RX {motor}-{name}: {len(rx)} {rx.hex() if rx else 'empty'}")
        total += len(rx)
    return total


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    ser = can_open_serial_port(port, 921600)
    if not ser:
        raise SystemExit(1)
    try:
        t1 = run_motor(ser, "motor1")
        t2 = run_motor(ser, "motor2")
        print("passive listen 5s ...")
        ser.reset_input_buffer()
        buf = bytearray()
        end = time.time() + 5
        while time.time() < end:
            buf.extend(ser.read(512) or b"")
            time.sleep(0.02)
        print(f"passive RX: {len(buf)} {buf.hex() if buf else 'empty'}")
        print(f"TOTAL RX motor1={t1} motor2={t2} passive={len(buf)}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
