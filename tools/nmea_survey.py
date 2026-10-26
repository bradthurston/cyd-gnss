#!/usr/bin/env python3
"""Survey what a GNSS receiver is actually saying, not just where it is.

Reads the raw stream (with the ESP32 held in reset by default, or while the app
runs) and reports: every sentence type and talker, fix quality and DOPs, a
per-constellation satellite table, position drift, receiver text messages, and
anything on the line that is NOT NMEA text at all - proprietary/binary frames,
garbage, or a bootloader talking.

Read-only: nothing is ever written to the receiver.

Usage: nmea_survey.py [seconds] [port] [--no-reset]
"""
import collections
import re
import statistics
import sys
import time

import serial

# group(3) is the payload WITHOUT the trailing *checksum, so the last data
# field is not glued to it ("1*3E" would parse as neither a number nor a systemID).
SENT = re.compile(r"^\$([A-Z]{2})([A-Z]{3}),(.*)\*[0-9A-Fa-f]{2}$")


def checksum_ok(sent):
    if "*" not in sent:
        return False
    body, _, cs = sent[1:].partition("*")
    acc = 0
    for ch in body:
        acc ^= ord(ch)
    try:
        return acc == int(cs[:2], 16)
    except ValueError:
        return False


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    port = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else "/dev/ttyUSB0"
    hold_reset = "--no-reset" not in sys.argv

    ser = serial.Serial(port, 9600, timeout=0.3)
    if hold_reset:
        ser.setDTR(False)
        ser.setRTS(True)          # EN low: chip parked, line belongs to the module
    else:
        ser.setDTR(False)
        ser.setRTS(False)         # leave the running app alone
    time.sleep(2.0)
    ser.reset_input_buffer()

    raw = b""
    end = time.time() + seconds
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            raw += chunk
    ser.close()

    text = raw.decode("utf-8", "replace")
    lines = [l.strip() for l in text.replace("\r", "\n").split("\n") if l.strip()]

    sentences, junk = [], []
    for l in lines:
        if l.startswith("$") and "*" in l:
            sentences.append(l)
        else:
            junk.append(l)

    print("=" * 66)
    print("capture: %d bytes in %.0fs  ->  %d sentences, %d non-sentence lines"
          % (len(raw), seconds, len(sentences), len(junk)))

    bad = [s for s in sentences if not checksum_ok(s)]
    print("checksums: %d ok, %d bad" % (len(sentences) - len(bad), len(bad)))

    # ---- sentence inventory ------------------------------------------------
    kinds = collections.Counter()
    for s in sentences:
        m = SENT.match(s)
        kinds[(m.group(1), m.group(2)) if m else ("?", "?")] += 1
    print("\nsentence types (talker + type, count):")
    for (tk, ty), n in sorted(kinds.items(), key=lambda kv: -kv[1]):
        print("  $%s%s  %4d   (%.1f/s)" % (tk, ty, n, n / seconds))

    # ---- per-constellation satellite table ---------------------------------
    sats = {}
    for s in sentences:
        m = SENT.match(s)
        if not m or m.group(2) != "GSV":
            continue
        f = m.group(3).split(",")
        for base in range(3, len(f) - 3, 4):
            if base + 3 >= len(f):
                continue
            prn_s = f[base].strip()
            if not prn_s:
                continue
            prn = int(prn_s) if prn_s.isdigit() else 0
            if prn <= 0:
                continue
            def num(x):
                x = x.strip()
                return int(x) if x.lstrip("-").isdigit() else -1
            key = (m.group(1), prn)
            rec = sats.setdefault(key, {"elev": -1, "az": -1, "snr": []})
            e, a, sn = num(f[base + 1]), num(f[base + 2]), num(f[base + 3])
            if e >= 0: rec["elev"] = e
            if a >= 0: rec["az"] = a
            if sn >= 0: rec["snr"].append(sn)

    used = collections.defaultdict(set)
    for s in sentences:
        m = SENT.match(s)
        if not m or m.group(2) != "GSA":
            continue
        f = m.group(3).split(",")
        # NMEA 4.11: ...VDOP,systemID. 1=GPS 2=GLONASS 3=Galileo 4=BeiDou 5=QZSS
        tk = m.group(1)
        if tk == "GN" and len(f) > 17 and f[17].strip().isdigit():
            tk = {1: "GP", 2: "GL", 3: "GA", 4: "BD", 5: "GQ"}.get(int(f[17]), "GN")
        for x in f[2:14]:
            if x.strip().isdigit() and int(x) > 0:
                used[tk].add(int(x))

    print("\nsatellites seen (%d distinct):" % len(sats))
    print("  SYS  PRN   ELEV    AZ    SNR      used-in-fix")
    for (tk, prn) in sorted(sats):
        rec = sats[(tk, prn)]
        snr = rec["snr"]
        snr_s = ("%2d-%2d (med %2d)" % (min(snr), max(snr), int(statistics.median(snr)))) if snr else "  --"
        print("  %s  %4d   %4s  %4s   %-16s %s"
              % (tk, prn, rec["elev"] if rec["elev"] >= 0 else "--",
                 rec["az"] if rec["az"] >= 0 else "---", snr_s,
                 "yes" if prn in used.get(tk, ()) else "no"))

    print("\nconstellation mix: %s" % ", ".join(
        "%s=%d" % (tk, sum(1 for k in sats if k[0] == tk)) for tk in sorted({k[0] for k in sats})))
    print("used in fix: %s" % (", ".join("%s=%d" % (tk, len(v)) for tk, v in sorted(used.items())) or "none"))

    # ---- fix / navigation --------------------------------------------------
    print("\nnavigation:")
    lats, lons, hdops, pdops, vdops, quals, used_n, hgts, speeds = [], [], [], [], [], [], [], [], []
    for s in sentences:
        m = SENT.match(s)
        if not m:
            continue
        ty, f = m.group(2), m.group(3).split(",")

        def num(x):
            x = x.strip()
            try:
                return float(x)
            except ValueError:
                return None

        if ty == "GGA" and len(f) > 8:
            def dm(v, hemi):
                if v is None:
                    return None
                d = int(v // 100) + (v % 100) / 60.0
                return -d if hemi in ("S", "W") else d
            la, lo = dm(num(f[1]), f[2].strip()), dm(num(f[3]), f[4].strip())
            if la is not None: lats.append(la)
            if lo is not None: lons.append(lo)
            q = num(f[5])
            if q is not None: quals.append(int(q))
            n = num(f[6])
            if n is not None: used_n.append(int(n))
            h = num(f[7])
            if h is not None: hdops.append(h)
            hg = num(f[8])
            if hg is not None: hgts.append(hg)
        elif ty == "GSA" and len(f) > 16:
            for name, idx in (("pdop", 14), ("hdop", 15), ("vdop", 16)):
                v = num(f[idx])
                if v is not None:
                    {"pdop": pdops, "hdop": hdops, "vdop": vdops}[name].append(v)
        elif ty == "RMC" and len(f) > 6:
            v = num(f[6])
            if v is not None: speeds.append(v * 1.852)   # knots -> km/h

    def rng(name, vals, fmt="%.2f"):
        if not vals:
            print("  %-12s --" % name)
            return
        line = "  %-12s " + fmt + " .. " + fmt + "   (median " + fmt + ")"
        print(line % (name, min(vals), max(vals), statistics.median(vals)))

    rng("lat", lats, "%.6f")
    rng("lon", lons, "%.6f")
    rng("hdop", hdops)
    rng("pdop", pdops)
    rng("vdop", vdops)
    rng("altitude m", hgts, "%.1f")
    rng("speed km/h", speeds, "%.2f")
    if quals:
        print("  fix quality   %s" % collections.Counter(quals).most_common())
    if used_n:
        print("  sats used     %d .. %d" % (min(used_n), max(used_n)))

    # ---- receiver's own text messages --------------------------------------
    txts = collections.Counter()
    for s in sentences:
        m = SENT.match(s)
        if m and m.group(2) == "TXT":
            txts[s.split(",", 4)[-1].split("*")[0]] += 1
    print("\nreceiver TXT messages (status/health, not position):")
    for msg, n in txts.most_common():
        print("  %-40s x%d" % (msg, n))

    # ---- anything that is not NMEA ----------------------------------------
    print("\nnon-NMEA content on the line:")
    if not junk and len(raw) == len(text.encode("utf-8", "replace")):
        print("  none - the line carries pure ASCII NMEA")
    else:
        weird = [b for b in raw if b < 9 or (13 < b < 32) or b > 126]
        print("  %d non-printable bytes, %d stray lines" % (len(weird), len(junk)))
        for l in junk[:10]:
            print("   ", repr(l[:100]))
    if bad:
        print("  first bad checksum: %s" % bad[0][:90])


if __name__ == "__main__":
    main()
