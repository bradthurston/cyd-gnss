#!/usr/bin/env python3
"""Render a USGS topo-style map (contours + hillshade) with an optional GNSS track.

Purpose: the GNSS Explorer logs positions to the CYD's microSD card; this turns
those logs (or any GPX/CSV) into something you can actually read against terrain.

Elevation data comes from USGS 3DEP (public, no API key) via the ImageServer
exportImage endpoint, which returns a 32-bit float GeoTIFF in EPSG:4326 for the
requested bbox. Responses are cached in tools/.cache so re-renders are offline.

Usage:
    # bare topo map of an area (decimal degrees)
    ./topo_map.py --bbox -87.22,35.01,-87.16,35.05 --out out/topo.png

    # centred on a point, ~3 km across
    ./topo_map.py --center 35.0288,-87.1917 --span-km 3 --out out/topo.png

    # with a logged track laid on top
    ./topo_map.py --center 35.0288,-87.1917 --span-km 3 \
                  --track /media/brad/SDCARD/gps/2026-10-26_0300.csv --out out/track.png

Track formats:
    .gpx                 -> <trkpt lat lon> / <ele>
    .csv                 -> header row with lat,lon and optional alt/ele
    .nmea / .log / .txt  -> raw NMEA, GGA + RMC parsed (what the SD log writes)

Honesty notes:
    * The vertical quality is set by the sky view, not by the receiver being junk:
      the same unit gave 57 m p-p altitude scatter indoors (VDOP 4.3, blocked sky)
      and 8 m p-p outdoors (VDOP 1.1, open sky). Every figure here is therefore
      computed from the capture in hand — never quoted from another run.
    * Nodata (outside coverage) stays NaN and renders as blank, never as sea level.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import requests
from PIL import Image
from matplotlib import patheffects as pe

CACHE = Path(__file__).with_name(".cache")
DEP3_URL = ("https://elevation.nationalmap.gov/arcgis/rest/services/"
            "3DEPElevation/ImageServer/exportImage")
M_PER_DEG_LAT = 110574.0


# --------------------------------------------------------------------------- DEM
def fetch_dem(bbox: tuple[float, float, float, float], size: int = 900) -> np.ndarray:
    """Return a float32 elevation array (metres, NaN for nodata) for (w,s,e,n)."""
    w, s, e, n = bbox
    key = f"{w:.5f}_{s:.5f}_{e:.5f}_{n:.5f}_{size}"
    CACHE.mkdir(exist_ok=True)
    tif = CACHE / f"dem_{key}.tif"
    if not tif.exists():
        params = {
            "bbox": f"{w},{s},{e},{n}", "bboxSR": "4326", "imageSR": "4326",
            "size": f"{size},{size}", "format": "tiff", "pixelType": "F32",
            "interpolation": "RSP_BilinearInterpolation", "f": "image",
        }
        r = requests.get(DEP3_URL, params=params, timeout=120)
        r.raise_for_status()
        if not r.content.startswith(b"II*\x00") and not r.content.startswith(b"MM\x00*"):
            raise RuntimeError(f"3DEP returned non-TIFF: {r.headers.get('content-type')} "
                               f"{r.content[:200]!r}")
        tif.write_bytes(r.content)
    a = np.array(Image.open(tif), dtype=np.float32)
    # 3DEP uses 0 or a large sentinel for nodata; the service clamps to 0 only
    # outside CONUS, so treat exact zeros and absurd values as nodata.
    a[(a == 0) | (a < -500) | (a > 9000)] = np.nan
    return a


def hillshade(dem: np.ndarray, dx_m: float, dy_m: float,
              az_deg: float = 315.0, alt_deg: float = 45.0) -> np.ndarray:
    """Standard hillshade, 0..1. NaNs become flat so edges don't produce streaks."""
    z = np.nan_to_num(dem, nan=float(np.nanmedian(dem)))
    gy, gx = np.gradient(z, dy_m, dx_m)
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    az, alt = np.radians(az_deg), np.radians(alt_deg)
    hs = (np.sin(alt) * np.cos(slope)
          + np.cos(alt) * np.sin(slope) * np.cos(az - aspect))
    return np.clip(hs, 0.0, 1.0)


# ------------------------------------------------------------------------- track
def _fix_from_within(zone: float) -> str:
    return f"{zone:02d}"


def parse_nmea(path: Path) -> list[tuple[float, float, float | None]]:
    """GGA (best: has altitude) or RMC. Requires a valid XOR checksum."""
    pts: list[tuple[float, float, float | None]] = []
    n_bad = 0
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line.startswith("$") or "*" not in line:
            continue
        body, _, cks = line[1:].rpartition("*")
        try:
            want = int(cks[:2], 16)
        except ValueError:
            n_bad += 1
            continue
        got = 0
        for ch in body:
            got ^= ord(ch)
        if got != want:
            n_bad += 1
            continue
        f = body.split(",")
        typ = f[0][-3:]
        if typ == "GGA" and len(f) > 9:
            lat, lon = _dm_to_dd(f[2], f[3]), _dm_to_dd(f[4], f[5])
            alt = float(f[9]) if f[9] else None
            if lat is not None and lon is not None and f[6] != "0":
                pts.append((lat, lon, alt))
        elif typ == "RMC" and len(f) > 7 and f[2] == "A":
            lat, lon = _dm_to_dd(f[3], f[4]), _dm_to_dd(f[5], f[6])
            if lat is not None and lon is not None:
                pts.append((lat, lon, None))
    if n_bad:
        print(f"  {path.name}: skipped {n_bad} line(s) with bad/missing checksum")
    return pts


def _dm_to_dd(value: str, hemi: str) -> float | None:
    """NMEA ddmm.mmmm + hemisphere -> decimal degrees."""
    if not value or not hemi:
        return None
    try:
        dot = value.index(".")
    except ValueError:
        return None
    deg = float(value[:dot - 2])
    mins = float(value[dot - 2:])
    dd = deg + mins / 60.0
    return -dd if hemi in ("S", "W") else dd


def parse_csv_track(path: Path) -> list[tuple[float, float, float | None]]:
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return []
    key = {k.lower().strip(): k for k in rows[0]}
    latk = key.get("lat") or key.get("latitude")
    lonk = key.get("lon") or key.get("longitude")
    if not latk or not lonk:
        raise SystemExit(f"{path}: need lat/lon columns, found {list(rows[0])}")
    altk = key.get("alt") or key.get("alt_m") or key.get("ele") or key.get("altitude") \
        or key.get("alt_msl")   # our own logs (dem_check, waypoints.csv) use alt_msl
    out = []
    for r in rows:
        try:
            lat, lon = float(r[latk]), float(r[lonk])
        except (TypeError, ValueError):
            continue
        alt = None
        if altk and r[altk] not in (None, "", "--", "-"):   # "-" = field left empty
            try:
                alt = float(r[altk])
            except ValueError:
                alt = None
        out.append((lat, lon, alt))
    return out


def parse_gpx(path: Path) -> list[tuple[float, float, float | None]]:
    # stdlib ElementTree is fine here: the input is a file Brad exports from his
    # own device or CalTopo, read locally, never fetched from a URL. No XXE/DTD
    # processing is enabled for external entities in CPython's default parser.
    pts = []
    for el in ET.parse(path).getroot().iter():
        if el.tag.split("}")[-1] not in ("trkpt", "wpt", "rtept"):
            continue
        try:
            lat, lon = float(el.get("lat")), float(el.get("lon"))
        except (TypeError, ValueError):
            continue
        alt = None
        for child in el:
            if child.tag.split("}")[-1] == "ele" and child.text:
                try:
                    alt = float(child.text)
                except ValueError:
                    pass
        pts.append((lat, lon, alt))
    return pts


def load_track(path: Path) -> list[tuple[float, float, float | None]]:
    suf = path.suffix.lower()
    if suf == ".gpx":
        return parse_gpx(path)
    if suf == ".csv":
        return parse_csv_track(path)
    return parse_nmea(path)          # .nmea / .log / .txt / anything else


# ------------------------------------------------------------------------ render
def dem_at(dem, bbox, lat: float, lon: float) -> float:
    """Sample the DEM at a lat/lon (nearest pixel). Returns NaN outside."""
    w, s, e, n = bbox
    ny, nx = dem.shape
    if not (w <= lon <= e and s <= lat <= n):
        return float("nan")
    j = min(nx - 1, max(0, int((lon - w) / (e - w) * nx)))
    i = min(ny - 1, max(0, int((n - lat) / (n - s) * ny)))
    return float(dem[i, j])


def render(bbox, dem, track, out: Path, title: str, contour_m: float, size: int):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    from matplotlib import ticker as mticker

    w, s, e, n = bbox
    ny, nx = dem.shape
    dx = (e - w) * 111320.0 * math.cos(math.radians((s + n) / 2)) / nx
    dy = (n - s) * M_PER_DEG_LAT / ny

    ext = [w, e, s, n]
    vmin, vmax = float(np.nanmin(dem)), float(np.nanmax(dem))
    hs = hillshade(dem, dx, dy)

    has_profile = bool(track) and len(track) > 1
    fig = plt.figure(figsize=(9.0, 9.4 if has_profile else 7.4), dpi=200)
    fig.patch.set_facecolor("#f7f3e8")
    gs = GridSpec(2 if has_profile else 1, 1, figure=fig,
                  height_ratios=[4.1, 1.0] if has_profile else [1.0],
                  hspace=0.42 if has_profile else 0.0,
                  left=0.10, right=0.97, top=0.93, bottom=0.075)
    ax = fig.add_subplot(gs[0])
    ax.set_facecolor("#f7f3e8")

    ax.imshow(dem, extent=ext, cmap="Greys", alpha=0.26, origin="upper")
    ax.imshow(hs, extent=ext, cmap="gray", alpha=0.32, origin="upper",
              vmin=0, vmax=1, interpolation="bilinear")

    levels = np.arange(math.floor(vmin / contour_m) * contour_m,
                       vmax + contour_m, contour_m)
    cs = ax.contour(dem, levels=levels, extent=ext, colors="#8a6b3a",
                    linewidths=0.5, alpha=0.85, origin="upper")
    keep = [levels[i] for i in range(0, len(levels), 5) if i < len(levels)]
    labs = ax.clabel(cs, levels=keep, inline=True, fontsize=5.5,
                     fmt="%d m", colors="#6b4f22")
    for lb in labs:                       # halo so labels stay readable on the hillshade
        lb.set_path_effects([pe.withStroke(linewidth=1.6, foreground="#f7f3e8")])

    notes = []
    if track:
        lats = np.array([q[0] for q in track])
        lons = np.array([q[1] for q in track])
        span_m = max(np.ptp(lats) * M_PER_DEG_LAT,
                     np.ptp(lons) * 111320 * math.cos(math.radians(lats.mean())))
        if len(track) > 1:
            ax.plot(lons, lats, "-", color="#c0392b", lw=1.1, alpha=0.85, zorder=5)
        ax.plot(lons, lats, ".", color="#c0392b",
                ms=3.0 if span_m < 100 else 1.8, alpha=0.6, zorder=4)
        ax.plot(lons[0], lats[0], "o", mfc="#2e7d32", mec="white", mew=0.8, ms=7, zorder=6)
        ax.plot(lons[-1], lats[-1], "s", mfc="#1565c0", mec="white", mew=0.8, ms=7, zorder=6)
        clat, clon = float(lats.mean()), float(lons.mean())
        alts = np.array([q[2] if q[2] is not None else np.nan for q in track], dtype=float)
        dem_here = dem_at(dem, bbox, clat, clon)
        if span_m < 100:
            notes.append(f"stationary capture: {len(track)} fixes inside {span_m:.0f} m "
                         f"(scatter, not travel)")
        if np.isfinite(alts).any() and np.isfinite(dem_here):
            notes.append(f"receiver alt median {np.nanmedian(alts):.0f} m vs DEM "
                         f"{dem_here:.0f} m here -> vertical off by "
                         f"{np.nanmedian(alts) - dem_here:+.0f} m; trust the horizontal")
    else:
        clat, clon = (s + n) / 2, (w + e) / 2

    ax.plot(clon, clat, "+", color="#111", ms=11, mew=1.0, zorder=7)  # darkest point = track mean / centre
    ax.set_xlim(w, e); ax.set_ylim(s, n)
    ax.set_xlabel("longitude", fontsize=7.5)
    ax.set_ylabel("latitude", fontsize=7.5)
    ax.tick_params(labelsize=6.5)
    ax.xaxis.set_major_formatter(mticker.FormatStrFormatter("%.4f"))
    ax.yaxis.set_major_formatter(mticker.FormatStrFormatter("%.4f"))
    ax.set_title(title, fontsize=9.5, color="#2b2b2b", pad=7)
    for sp in ax.spines.values():
        sp.set_color("#9a8f7a")

    span_km = (e - w) * 111.32 * math.cos(math.radians((s + n) / 2))
    meta = (f"USGS 3DEP {nx}px  |  {span_km:.2f} x {(n - s) * 111.32:.2f} km  |  "
            f"{vmin:.0f}-{vmax:.0f} m  |  contours {contour_m:.0f} m")
    ax.text(0.008, 0.982, meta, transform=ax.transAxes, fontsize=5.8, va="top",
            color="#4a4a4a", zorder=20,
            bbox=dict(fc="#f7f3e8", ec="#c9bda6", lw=0.4, alpha=0.95, pad=1.8))
    for k, note in enumerate(notes):
        ax.text(0.008, 0.925 - 0.045 * k, note, transform=ax.transAxes,
                fontsize=5.8, va="top", color="#a03a20", zorder=20,
                bbox=dict(fc="#f7f3e8", ec="#e0b7a8", lw=0.4, alpha=0.95, pad=1.8))

    # Scale bar. Distances on a hillshade are otherwise unreadable, and the span
    # varies per invocation, so pick the round number nearest a fifth of the width.
    deg_per_km_lon = 1.0 / (111.32 * math.cos(math.radians((s + n) / 2)))
    want_km = span_km / 5.0
    bar_km = min([0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20],
                 key=lambda v: abs(v - want_km))
    bar_deg = bar_km * deg_per_km_lon
    x1 = e - (e - w) * 0.035
    y0 = s + (n - s) * 0.045
    ax.plot([x1 - bar_deg, x1], [y0, y0], "-", color="#111", lw=1.7, zorder=21,
            solid_capstyle="butt")
    for xx in (x1 - bar_deg, x1):
        ax.plot([xx, xx], [y0 - (n - s) * 0.013, y0 + (n - s) * 0.013], "-",
                color="#111", lw=1.7, zorder=21)
    ax.text((x1 + x1 - bar_deg) / 2, y0 + (n - s) * 0.022,
            f"{bar_km:g} km" if bar_km >= 1 else f"{bar_km * 1000:g} m",
            ha="center", va="bottom", fontsize=6.5, color="#111", zorder=22,
            bbox=dict(fc="#f7f3e8", ec="none", alpha=0.72, pad=1.0))

    if has_profile:
        axt = fig.add_subplot(gs[1])
        axt.set_facecolor("#fffdf7")
        d = np.concatenate([[0], np.cumsum(
            np.hypot(np.diff(lons) * 111320 * math.cos(math.radians(lats.mean())),
                     np.diff(lats) * M_PER_DEG_LAT))])
        # GGA carries altitude, RMC does not, so alternating fixes are NaN and a
        # bare line breaks into invisible one-point segments — markers always show.
        axt.plot(d / 1000.0, alts, color="#c0392b", lw=0.9, marker=".",
                 ms=2.6, alpha=0.95)
        n_alt = int(np.isfinite(alts).sum())
        axt.set_title(f"GPS altitude samples: {n_alt} of {len(alts)} fixes carry altitude "
                      f"(GGA only)", fontsize=6, color="#6a6a6a", pad=3)
        if np.isfinite(dem_here):
            axt.axhline(dem_here, color="#6b4f22", lw=0.8, ls="--", alpha=0.8)
            axt.text(d[-1] / 1000.0, dem_here, f"  DEM {dem_here:.0f} m",
                     fontsize=5.6, va="center", color="#6b4f22")
        axt.set_xlabel(f"distance along track (km) — total {d[-1] / 1000:.3f} km", fontsize=7)
        axt.set_ylabel("GPS alt (m)", fontsize=7)
        axt.tick_params(labelsize=6.5)
        axt.grid(alpha=0.25, lw=0.4)
        axt.set_xlim(0, max(d[-1] / 1000.0, 1e-4))
        for sp in axt.spines.values():
            sp.set_color("#b9ad95")
        if np.isfinite(alts).any():
            fin = alts[np.isfinite(alts)]
            if len(fin) > 3:
                axt.text(0.005, 0.94,
                         f"this capture's own vertical spread: {np.ptp(fin):.1f} m "
                         f"p-p (median {np.median(fin):.1f} m) — read the trend, not the value",
                         transform=axt.transAxes, fontsize=5.4, va="top", color="#8a3a2a")
            else:
                axt.text(0.005, 0.94, f"only {len(fin)} altitude fixes — no spread figure",
                         transform=axt.transAxes, fontsize=5.4, va="top", color="#8a3a2a")
        else:
            axt.text(0.005, 0.94, "no altitude in this capture (GLL/RMC only)",
                     transform=axt.transAxes, fontsize=5.4, va="top", color="#8a3a2a")

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bbox", help="west,south,east,north in decimal degrees")
    p.add_argument("--center", help="lat,lon — used with --span-km")
    p.add_argument("--span-km", type=float, default=3.0, help="across, km (default 3)")
    p.add_argument("--track", help=".gpx / .csv / raw NMEA log")
    p.add_argument("--out", default="out/topo.png")
    p.add_argument("--title", default=None)
    p.add_argument("--contour-m", type=float, default=2.0)
    p.add_argument("--size", type=int, default=900, help="DEM pixels per side")
    a = p.parse_args(argv)

    track = []
    if a.track:
        tp = Path(a.track)
        if not tp.exists():
            raise SystemExit(f"track not found: {tp}")
        track = load_track(tp)
        if not track:
            raise SystemExit(f"no positions parsed from {tp}")
        print(f"track: {len(track)} fixes from {tp.name}")

    if a.bbox:
        bbox = tuple(float(x) for x in a.bbox.split(","))
        if len(bbox) != 4:
            raise SystemExit("--bbox needs w,s,e,n")
    elif a.center:
        clat, clon = (float(x) for x in a.center.split(","))
        dlat = a.span_km / 2 / (M_PER_DEG_LAT / 1000)
        dlon = dlat / math.cos(math.radians(clat))
        bbox = (clon - dlon, clat - dlat, clon + dlon, clat + dlat)
    elif track:
        lats = [x[0] for x in track]
        lons = [x[1] for x in track]
        pad = 0.15
        dw = max(max(lons) - min(lons), 0.004) * pad + 0.002
        dh = max(max(lats) - min(lats), 0.004) * pad + 0.002
        bbox = (min(lons) - dw, min(lats) - dh, max(lons) + dw, max(lats) + dh)
    else:
        p.error("give --bbox, --center, or --track")

    print(f"bbox: {bbox[0]:.6f},{bbox[1]:.6f} -> {bbox[2]:.6f},{bbox[3]:.6f}")
    dem = fetch_dem(bbox, a.size)
    print(f"dem: {dem.shape[0]}x{dem.shape[1]} px  "
          f"{np.nanmin(dem):.1f}-{np.nanmax(dem):.1f} m  "
          f"nodata {int(np.isnan(dem).sum())}")
    if np.isnan(dem).all():
        raise SystemExit("DEM is entirely nodata — 3DEP has no coverage here")

    title = a.title or "USGS 3DEP topo"
    if track:
        title += f"  +  {len(track)} GNSS fixes"
    out = render(bbox, dem, track, Path(a.out), title, a.contour_m, a.size)
    h = hashlib.sha256(out.read_bytes()).hexdigest()[:12]
    print(f"wrote {out}  {out.stat().st_size:,} bytes  sha256 {h}")


if __name__ == "__main__":
    sys.exit(main())
