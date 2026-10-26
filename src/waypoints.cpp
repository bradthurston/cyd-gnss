// waypoints.cpp — see waypoints.h. No Arduino, no SD: this file compiles on a PC
// as-is, which is what makes the host tests possible.
#include "waypoints.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const double DEG2RAD = 0.017453292519943295;
static const double R_EARTH_M = 6371008.8;

const char* wptResultText(WptResult r) {
  switch (r) {
    case WPT_OK: return "saved";
    case WPT_NO_FIX: return "no fix";
    case WPT_STALE: return "fix stale";
  }
  return "refused";
}

const Waypoint& WaypointStore::at(int newestIndex) const {
  if (newestIndex < 0) newestIndex = 0;
  if (newestIndex >= count_) newestIndex = count_ - 1;
  int i = (start_ + count_ - 1 - newestIndex) % WPT_MAX;
  if (i < 0) i += WPT_MAX;
  return ring_[i];
}

WptResult WaypointStore::add(const WptFix& f, uint32_t atMs, char* csv, size_t csvLen) {
  WptResult r = WPT_OK;
  // Stale is checked first: a stale fix also fails the freshness test below, and
  // "fix stale" is the more useful answer on screen than a blanket "no fix".
  if (f.stale) r = WPT_STALE;
  else if (!f.posValid || !f.fixValid) r = WPT_NO_FIX;
  if (r != WPT_OK) {
    refused_++;
    lastRefusal_ = r;
    return r;                    // nothing stored, no row emitted, ids do not advance
  }

  Waypoint w = {};
  w.id = nextId_++;
  if (f.clockValid)
    snprintf(w.utc, sizeof(w.utc), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             (unsigned)f.year, (unsigned)f.month, (unsigned)f.day,
             (unsigned)f.hour, (unsigned)f.minute, (unsigned)f.second);
  else
    // No UTC clock yet (no fix since boot). Naming the mark by boot time is
    // honest and keeps it distinguishable; an empty field would not be.
    snprintf(w.utc, sizeof(w.utc), "boot+%lus", (unsigned long)(atMs / 1000));
  w.lat = f.lat;
  w.lon = f.lon;
  w.altValid = f.altValid; w.altM = f.altM;
  w.sats = f.satsUsed;
  w.hdopValid = f.hdopValid; w.hdop = f.hdop;
  w.atMs = atMs;

  int slot;
  if (count_ < WPT_MAX) {
    slot = (start_ + count_) % WPT_MAX;
    count_++;
  } else {
    slot = start_;                    // full: overwrite the oldest
    start_ = (start_ + 1) % WPT_MAX;
  }
  ring_[slot] = w;

  lastId_ = w.id;
  total_++;
  lastRefusal_ = WPT_OK;
  if (csv && csvLen) wptFormatRow(w, csv, csvLen);
  return WPT_OK;
}

int wptFormatRow(const Waypoint& w, char* out, size_t n) {
  if (!out || !n) return 0;
  char alt[16], hd[16];
  if (w.altValid) snprintf(alt, sizeof(alt), "%.1f", w.altM);
  else snprintf(alt, sizeof(alt), "-");
  if (w.hdopValid) snprintf(hd, sizeof(hd), "%.2f", w.hdop);
  else snprintf(hd, sizeof(hd), "-");
  return snprintf(out, n, "%u,%s,%.6f,%.6f,%s,%u,%s",
                  (unsigned)w.id, w.utc, w.lat, w.lon, alt, (unsigned)w.sats, hd);
}

double wptDistanceM(double lat1, double lon1, double lat2, double lon2) {
  const double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
  const double dp = (lat2 - lat1) * DEG2RAD, dl = (lon2 - lon1) * DEG2RAD;
  const double sp = sin(dp / 2), sl = sin(dl / 2);
  double a = sp * sp + cos(p1) * cos(p2) * sl * sl;
  if (a < 0) a = 0;
  if (a > 1) a = 1;
  return 2 * R_EARTH_M * asin(sqrt(a));
}

double wptBearingDeg(double lat1, double lon1, double lat2, double lon2) {
  const double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
  const double dl = (lon2 - lon1) * DEG2RAD;
  const double y = sin(dl) * cos(p2);
  const double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  if (y == 0 && x == 0) return 0;          // same point: no bearing to report
  double b = atan2(y, x) / DEG2RAD;
  if (b < 0) b += 360;
  if (b >= 360) b -= 360;
  return b;
}

void wptCompass(char* out, size_t n, double bearingDeg) {
  static const char* WIND[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                 "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  if (!out || !n) return;
  double b = bearingDeg;
  if (b < 0) b += 360;
  if (b >= 360) b -= 360;
  int i = (int)((b + 11.25) / 22.5) % 16;
  snprintf(out, n, "%s", WIND[i]);
}
