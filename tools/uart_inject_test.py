#!/usr/bin/env python3
"""Validate the RX-pin probe by sending bytes down the USB line.

The PC's bytes reach the ESP32 on GPIO3 (CH340 TXD -> ESP32 RXD0). If the probe
is working, a dwell window on GPIO3 must count them. This separates "the probe
cannot see anything" from "there is nothing on the wire" - the two explanations
for an all-zero sweep.

Usage: uart_inject_test.py [seconds] [port]
"""
import sys
import threading
import time

import serial

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"

s = serial.Serial(port, 9600, timeout=1)

# Run the app, do not sit in the download stub.
s.setDTR(False)
s.setRTS(True)
time.sleep(0.15)
s.setRTS(False)
time.sleep(0.2)
s.reset_input_buffer()

stop = threading.Event()
sent = [0]


def writer():
    payload = b"ATGM-INJECT-TEST-0123456789\r\n" * 2
    while not stop.is_set():
        try:
            n = s.write(payload)
            sent[0] += n or 0
        except Exception:
            pass
        time.sleep(0.3)


threading.Thread(target=writer, daemon=True).start()

end = time.time() + seconds
buf = b""
while time.time() < end:
    data = s.read(512)
    if data:
        buf += data
stop.set()

text = buf.decode("utf-8", "replace")
probe = [l for l in text.splitlines() if "[probe]" in l]
print("\n".join(probe[-20:]))
print("---- sent %d bytes from the PC ----" % sent[0])
