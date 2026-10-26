#!/usr/bin/env python3
"""Read raw NMEA straight off the USB port with the ESP32 held in reset.

Held in reset, nothing on the board drives GPIO1, so if the receiver's TX is
wired to P1's TX pin (GPIO1 - the pin HaleHound's GPS_bus names as rx) the
CH340's RXD sees the NMEA stream directly and the PC can read it at 9600 with no
firmware involved at all. That separates "the module is silent" from "the pin is
wrong", and it hands back real sentences to test the parser against.

If the board is not actually held in reset you will see the firmware's own log
lines instead ("[pad]"/"[probe]"), which is itself the answer.

If a third argument is given, the captured sentences are saved there, so real
receiver output can be used as host-test data instead of hand-typed guesses.

Usage: passive_nmea_read.py [seconds] [port] [save_path]
"""
import sys
import time

import serial

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"
save_path = sys.argv[3] if len(sys.argv) > 3 else None

s = serial.Serial(port, 9600, timeout=0.5)
s.setDTR(False)
s.setRTS(True)          # hold EN low: ESP32 in reset, its pins high-Z
time.sleep(0.3)
s.reset_input_buffer()

buf = b""
end = time.time() + seconds
while time.time() < end:
    data = s.read(256)
    if data:
        buf += data
s.setRTS(False)

text = buf.decode("utf-8", "replace")
lines = [l for l in text.replace("\r", "\n").split("\n") if l.strip()]
print("raw bytes: %d" % len(buf))
print("lines: %d" % len(lines))
print("NMEA sentences: %d" % sum(1 for l in lines if l.startswith("$")))
if save_path:
    # Only complete sentences: the window can end mid-sentence, and a truncated
    # line has a bogus checksum that would look like a parser bug in the tests.
    import re
    complete = re.compile(r"^\$[A-Z]{2}[A-Z]{3},.*\*[0-9A-Fa-f]{2}$")
    good = [l for l in lines if l.startswith("$") and complete.match(l)]
    with open(save_path, "w") as fh:
        fh.write("\n".join(good) + "\n")
    print("saved to: %s (%d complete, %d dropped)"
          % (save_path, len(good), sum(1 for l in lines if l.startswith("$")) - len(good)))
for l in lines[:20]:
    print("  " + repr(l[:90]))
