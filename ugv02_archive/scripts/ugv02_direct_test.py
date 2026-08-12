#!/usr/bin/env python3
import json
import argparse
import threading
import time

import serial


PORT = "/dev/ttyTHS1"
BAUD = 115200
TEST_SECONDS = 1.0
TEST_SPEED = 0.0


def send(ser, data):
    ser.write((json.dumps(data, separators=(",", ":")) + "\n").encode("utf-8"))
    ser.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("speed", "pwm", "query"), default="query")
    parser.add_argument("--value", type=float, default=TEST_SPEED)
    parser.add_argument("--seconds", type=float, default=TEST_SECONDS)
    args = parser.parse_args()

    latest = {}
    running = True

    with serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=0.2) as ser:
        ser.reset_input_buffer()
        time.sleep(0.5)

        def reader():
            nonlocal latest
            buffer = bytearray()
            while running:
                chunk = ser.read(max(1, min(ser.in_waiting, 512)))
                if not chunk:
                    continue
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    text = raw.decode("utf-8", errors="ignore")
                    start, end = text.find("{"), text.rfind("}")
                    if start < 0 or end <= start:
                        continue
                    try:
                        data = json.loads(text[start : end + 1])
                    except json.JSONDecodeError:
                        continue
                    if data.get("T") == 1001:
                        latest = data

        thread = threading.Thread(target=reader, daemon=True)
        thread.start()

        send(ser, {"T": 143, "cmd": 0})
        send(ser, {"T": 131, "cmd": 1})
        send(ser, {"T": 130})
        time.sleep(0.8)
        before = dict(latest)

        if args.mode != "query":
            command_type = 11 if args.mode == "pwm" else 1
            try:
                deadline = time.monotonic() + args.seconds
                while time.monotonic() < deadline:
                    send(ser, {"T": command_type, "L": args.value, "R": args.value})
                    time.sleep(0.05)
            finally:
                for _ in range(5):
                    send(ser, {"T": command_type, "L": 0, "R": 0})
                    time.sleep(0.05)

        time.sleep(0.5)
        after = dict(latest)
        running = False
        thread.join(timeout=0.3)

    fields = ("T", "L", "R", "odl", "odr", "v")
    print("BEFORE:", {key: before.get(key) for key in fields})
    print("AFTER: ", {key: after.get(key) for key in fields})
    print("FULL:  ", after)


if __name__ == "__main__":
    main()
