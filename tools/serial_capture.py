#!/usr/bin/env python3
"""Capture GNSS Explorer diagnostics from the CYD over the CH340.

The board mirrors boot + link diagnostics to UART0. GPIO1 is the ESP32's TX, so
this only listens — it never sends anything to the board.

One USB-serial port cannot read at one baud and write at another, so the baud
must match the firmware's Serial.begin(). Diagnostic build environments that
sample a UART at 9600 use -DPROBE_LOG_BAUD=9600 for exactly this reason.

Usage: serial_capture.py [seconds] [port] [baud]
"""
import sys
import time

import serial

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"
baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

s = serial.Serial(port, baud, timeout=1)

# Hard reset: pulse EN (RTS) with GPIO0 (DTR) held high, so the chip runs the
# app rather than entering the download stub. Same trick the OneBoard bring-up
# uses — the board boots in ~1s, so the read has to start before the pulse.
s.setDTR(False)
s.setRTS(True)
time.sleep(0.15)
s.setRTS(False)
time.sleep(0.2)
s.reset_input_buffer()

end = time.time() + seconds
buf = b""
while time.time() < end:
    data = s.read(512)
    if data:
        buf += data

text = buf.decode("utf-8", "replace")
print(text)
print("---- exit: %d bytes captured ----" % len(buf))

# quick verdict
boot = "[boot]" in text
gnss = [l for l in text.splitlines() if l.startswith("[gnss]")]
zeros = [l for l in gnss if "bytes=0 " in l or l.strip().endswith("bytes=0")]
print("[verdict] boot banner seen: %s" % boot)
print("[verdict] gnss status lines: %d" % len(gnss))
if gnss:
    print("[verdict] last line: %s" % gnss[-1])
if gnss and len(zeros) == len(gnss):
    print("[verdict] ZERO bytes on this rx pin -> wiring is elsewhere")
