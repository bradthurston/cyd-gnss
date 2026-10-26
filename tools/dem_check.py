#!/usr/bin/env python3
"""dem_check.py — is the receiver's altitude wrong, and by how much, and why?

Compares every fix in a GNSS log against USGS 3DEP terrain (NAVD88 orthometric,
the same datum the receiver claims for GGA height) and reports the vertical error
with the geometry that produced it.

The point is to separate two very different claims:

  * "the altitude is wrong"  — a fixed offset, fixable with a constant,
  * "the altitude is noisy"  — geometry/multipath, fixable only with sky.

GGA gives altitude AND the geoid separation the receiver applied; GSA gives
PDOP/HDOP/VDOP for that epoch. Vertical error scales with VDOP (which on a real
fix runs 2-4x HDOP), so if the error tracks VDOP it is sky geometry, and taking
the unit outdoors should shrink both. If the error is flat across VDOP, it is a
datum/constant problem and going outside will not help.

Usage:
    ./dem_check.py LOG.nmea [--csv fixes.csv] [--pad-km 0.4]
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def _load_topo():
    """Import the sibling topo_map module (it is a script, so do it by path)."""
    path = Path(__file__).resolve().with_name("topo_map.py")
    spec = importlib.util.spec_from_file_location("topo_map", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


topo = _load_topo()


# --------------------------------------------------------------------- parsing
def _ok(line: str) -> bool:
    """NMEA XOR checksum gate — a bad sentence is not evidence about anything."""
    if "*" not in line:
        return False
    body, _, cks = line[1:].rpartition("*")
    try:
        want = int(cks[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def _dm(value: str, hemi: str):
    if not value or not hemi:
        return None
    dot = value.find(".")
    if dot < 3:
        return None
    try:
        deg = float(value[: dot - 2])
        minutes = float(value[dot - 2:])
    except ValueError:
        return None
    dd = deg + minutes / 60.0
    return -dd if hemi in ("S", "W") else dd


def parse_fixes(path: Path):
    """GGA fixes with their geometry. GSA values are attached to the fix that
    follows them (the receiver emits GSA just before the next GGA epoch)."""
    fixes = []
    pdop = hdop_g = vdop = float("nan")
    for raw in path.read_text(errors="replace").splitlines():
        line = raw.strip()
        if not line.startswith("$") or not _ok(line):
            continue
        kind = line[3:6]
        f = line.split("*")[0].split(",")

        if kind == "GSA" and len(f) >= 18:
            try:
                pdop, hdop_g, vdop = float(f[15]), float(f[16]), float(f[17])
            except ValueError:
                pass
            continue

        if kind != "GGA" or len(f) < 12:
            continue
        lat, lon = _dm(f[2], f[3]), _dm(f[4], f[5])
        if lat is None or lon is None:
            continue

        def num(i):
            try:
                return float(f[i]) if f[i] else float("nan")
            except ValueError:
                return float("nan")

        fixes.append(dict(
            time=f[1], lat=lat, lon=lon,
            fixq=(int(f[6]) if f[6].isdigit() else 0),
            sats=(int(f[7]) if f[7].isdigit() else 0),
            hdop=num(8), alt=num(9), geoid=num(11),
            pdop=pdop, hdop_g=hdop_g, vdop=vdop,
        ))
    return fixes


# --------------------------------------------------------------------- analysis
def main(argv=None):
    ap = argparse.ArgumentParser(description="Vertical accuracy vs real terrain.")
    ap.add_argument("log")
    ap.add_argument("--csv", default=None, help="write per-fix comparison here")
    ap.add_argument("--pad-km", type=float, default=0.4,
                    help="padding around the fixes when fetching terrain")
    ap.add_argument("--size", type=int, default=900)
    ap.add_argument("--antenna-height-m", type=float, default=0.0,
                    help="how high the antenna sat above BARE GROUND, in metres. 3DEP is a "
                         "bare-earth surface, so a receiver on a deck, rail, mast or truck roof "
                         "is genuinely that far above the modelled terrain — that height is part "
                         "of the measurement, not an error. Subtracted from every fix before "
                         "scoring; without it the median error is simply the antenna height.")
    ap.add_argument("--antenna-ft", type=float, default=None,
                    help="same, in feet (a deck rail 12 ft up = 3.66 m)")
    a = ap.parse_args(argv)
    ant_m = a.antenna_height_m if a.antenna_height_m else (a.antenna_ft or 0.0) * 0.3048

    path = Path(a.log).expanduser()
    if not path.exists():
        print(f"no such log: {path}")
        return 2

    fixes = parse_fixes(path)
    if not fixes:
        print(f"no checksum-valid GGA fixes in {path}")
        return 1

    lats = np.array([f["lat"] for f in fixes])
    lons = np.array([f["lon"] for f in fixes])
    alts = np.array([f["alt"] for f in fixes])
    vdops = np.array([f["vdop"] for f in fixes])
    good = np.isfinite(alts) & (alts != 0.0)

    span_m = max(np.ptp(lats) * 110574.0,
                 np.ptp(lons) * 111320.0 * math.cos(math.radians(lats.mean())))

    pad = a.pad_km / 111.0
    bbox = (lons.min() - pad, lats.min() - pad, lons.max() + pad, lats.max() + pad)
    print(f"log        {path}")
    print(f"fixes      {len(fixes)} valid GGA ({int(good.sum())} with an altitude), "
          f"span {span_m:.1f} m -> {'STATIONARY' if span_m < 50 else 'the device moved'}")
    print(f"terrain    USGS 3DEP over {bbox[0]:.5f},{bbox[1]:.5f} .. {bbox[2]:.5f},{bbox[3]:.5f}")
    if ant_m:
        print(f"antenna    {ant_m:.2f} m ({ant_m / 0.3048:.1f} ft) above bare earth "
              f"-> subtracted from every fix")
    dem = topo.fetch_dem(bbox, size=a.size)

    errs, rows = [], []
    for f in fixes:
        if not (np.isfinite(f["alt"]) and f["alt"] != 0.0):
            continue
        d = topo.dem_at(dem, bbox, f["lat"], f["lon"])
        if not np.isfinite(d):
            continue
        e = f["alt"] - d - ant_m
        errs.append(e)
        rows.append((f, d, e))
    if not errs:
        print("no fix landed on a terrain sample — bbox or coordinates look wrong")
        return 1
    errs = np.array(errs)

    med = float(np.median(errs))
    print()
    print(f"ALTITUDE vs TERRAIN   n={len(errs)}")
    print(f"  median error   {med:+8.1f} m   (receiver minus terrain)")
    print(f"  mean   error   {float(errs.mean()):+8.1f} m   sd {float(errs.std(ddof=1)) if len(errs) > 1 else 0:6.1f} m")
    print(f"  range          {errs.min():+8.1f} .. {errs.max():+.1f} m")
    q1, q3 = np.percentile(errs, [25, 75])
    print(f"  IQR            {q1:+.1f} .. {q3:+.1f} m  ({q3 - q1:.1f} m wide)")

    ok_v = np.isfinite(vdops)
    if ok_v.any():
        v = vdops[ok_v]
        print()
        print("GEOMETRY (from GSA, per epoch)")
        print(f"  VDOP  median {np.median(v):.1f}  min {v.min():.1f}  max {v.max():.1f}")
        hv = np.array([f["hdop_g"] for f in fixes])
        hv = hv[np.isfinite(hv)]
        if hv.size:
            print(f"  HDOP  median {np.median(hv):.1f}  ->  VDOP/HDOP ratio "
                  f"{np.median(v) / max(np.median(hv), 1e-6):.1f}x")
        # Implied 1-sigma ranging error: this is the number that should DROP when
        # the antenna sees open sky. A flat ratio across a better sky view means
        # the error is not geometry at all.
        if ant_m:
            # With the antenna height removed the median is an offset we just
            # subtracted, not an accuracy figure — scoring it would flatter the unit
            # to a fraction of a metre. Score the residual SPREAD instead: IQR/1.349
            # is a robust 1-sigma, which is the shape ranging error actually has.
            sigma = float(q3 - q1) / 1.349
            implied = sigma / max(float(np.median(v)), 1e-6)
            print(f"  implied unit ranging error  residual sigma {sigma:.2f} m "
                  f"(IQR/1.349) / median VDOP = {implied:.1f} m")
        else:
            implied = abs(med) / max(float(np.median(v)), 1e-6)
            print(f"  implied unit ranging error |median err| / median VDOP = {implied:.1f} m")
        print(f"  (open-sky u-blox-class units land around 1.5-3 m; >5 m means the")
        print(f"   sky view or the antenna is costing accuracy, not the arithmetic)")
        # Does the error track geometry within this capture?
        if len(errs) > 8 and np.isfinite(vdops[:len(errs)]).sum() > 8:
            vv = vdops[:len(errs)]
            m = np.isfinite(vv)
            if m.sum() > 8 and np.std(vv[m]) > 0:
                r = float(np.corrcoef(np.abs(errs[m]), vv[m])[0, 1])
                print(f"  corr(|error|, VDOP) within this capture = {r:+.2f}")

    # A stationary capture that wanders vertically is the clearest sign the sky
    # view is the problem: the antenna never moved, so any height change is the
    # receiver's own error moving. Report the walk, not just the endpoints.
    if len(errs) >= 8:
        k = max(3, len(errs) // 4)
        first, last = float(np.median(errs[:k])), float(np.median(errs[-k:]))
        print()
        print("STABILITY (antenna did not move)")
        print(f"  altitude wandered {float(np.ptp(alts[good])):.1f} m over "
              f"{len(errs)} fixes while sitting still")
        print(f"  median error: first {k} fixes {first:+.1f} m -> last {k} fixes {last:+.1f} m "
              f"(converging, {last - first:+.1f} m)")
        print(f"  i.e. it is still {abs(last):.0f} m off after settling, so time alone will not fix it")

    geoids = np.array([f["geoid"] for f in fixes])
    geoids = geoids[np.isfinite(geoids)]
    if geoids.size:
        print()
        print(f"GEOID separation the receiver applied: {np.median(geoids):+.1f} m "
              f"(constant across the capture: {np.ptp(geoids) < 0.6})")

    print()
    print("first 8 fixes:  time        lat          lon          alt     terrain   err    VDOP")
    for f, d, e in rows[:8]:
        print(f"  {f['time'][:6]}  {f['lat']:.6f}  {f['lon']:.6f}  "
              f"{f['alt']:6.1f}  {d:7.1f}  {e:+7.1f}  {f['vdop']:5.1f}")

    if a.csv:
        out = Path(a.csv).expanduser()
        with out.open("w") as fh:
            fh.write("time,lat,lon,fixq,sats,hdop,vdop,alt_msl,geoid,dem_navd88,error\n")
            for f, d, e in rows:
                fh.write(f"{f['time']},{f['lat']:.6f},{f['lon']:.6f},{f['fixq']},"
                         f"{f['sats']},{f['hdop']},{f['vdop']},{f['alt']},{f['geoid']},"
                         f"{d:.2f},{e:.2f}\n")
        print(f"\nper-fix comparison written to {out}")

    print()
    print("VERDICT")
    if span_m < 50:
        print("  stationary capture: the error figures describe one antenna position,")
        print("  so they measure this spot's sky view, not the receiver's general accuracy.")
    if abs(med) > 15 and ok_v.any() and np.median(v) > 2.5:
        print(f"  geometry is poor (VDOP {np.median(v):.1f}) and the altitude sits "
              f"{med:+.0f} m off.")
        print("  -> take it outside, open sky, and re-run: this is the testable claim.")
    elif abs(med) > 15:
        print(f"  altitude sits {med:+.0f} m off but the geometry is decent — suspect a")
        print("  datum/offset problem rather than sky view. Test outdoors to confirm.")
    else:
        print("  altitude agrees with terrain to within a few metres: vertical is usable.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
