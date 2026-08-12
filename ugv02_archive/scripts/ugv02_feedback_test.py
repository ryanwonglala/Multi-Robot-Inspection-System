#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.2)

print("opened", PORT)

# 开启连续反馈
ser.write(b'{"T":131,"cmd":1}\n')
time.sleep(0.5)

start = time.time()
while time.time() - start < 8:
    line = ser.readline()
    if line:
        text = line.decode(errors="ignore").strip()
        print(text)

# 关闭连续反馈
ser.write(b'{"T":131,"cmd":0}\n')
time.sleep(0.2)

ser.close()
print("done")
