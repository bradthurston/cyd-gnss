// Host tests for the waypoint layer: capture rules, the CSV row, the geo maths.
//
//   ./tools/host_tests/run.sh
//
// waypoints.cpp is compiled in directly (not reimplemented) and needs no Arduino
// shim: it touches nothing but stdio/math. What is worth testing here is what
// fails silently on the board - a CSV row that parses wrong on the PC, and a
// distance/bearing that is wrong by a factor nobody notices until they walk to
// the wrong end of a field.
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "waypoints.h"
#include "waypoints.cpp"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...)                            \
  do {                                              \
    checks++;                                       \
    if (!(cond)) {                                  \
      failures++;                                   \
      printf("FAIL  ");                             \
      printf(__VA_ARGS__);                          \
      printf("   (%s:%d)\n", __FILE__, __LINE__);   \
    } else {                                        \
      printf("ok    ");                             \
      printf(__VA_ARGS__);                          \
      printf("\n");                                 \
    }                                               \
  } while (0)

// A good fix: Minor Hill TN, from the real capture in sample_nmea.txt.
static WptFix goodFix() {
  WptFix f = {};
  f.posValid = true;
  f.fixValid = true;
  f.lat = 35.028848;
  f.lon = -87.192328;
  f.altValid = true; f.altM = 215.1f;
  f.satsUsed = 7;
  f.hdopValid = true; f.hdop = 1.70f;
  f.clockValid = true;
  f.year = 2026; f.month = 10; f.day = 26;
  f.hour = 9; f.minute = 45; f.second = 12;
  return f;
}

// Metres per degree of latitude at 35 deg (the WGS84 meridional figure): used so
// the test's "1 km north" really is 1 km instead of a made-up constant.
static const double M_PER_DEG_LAT_35 = 110941.0;
static double northOf(double lat, double m) { return lat + m / M_PER_DEG_LAT_35; }
static double eastOf(double lat, double lon, double m) {
  return lon + m / (111320.0 * cos(lat * 0.017453292519943295));
}

int main() {
  printf("\nWaypoint host tests (real src/waypoints.cpp)\n");
  printf("---------------------------------------------------------------\n");

  // ------------------------------------------------- capture: the happy path
  {
    WaypointStore st;
    char row[128] = {0};
    WptResult r = st.add(goodFix(), 123456, row, sizeof(row));
    CHECK(r == WPT_OK, "valid fix accepted (got %s)", wptResultText(r));
    CHECK(strcmp(row, "1,2026-10-26T09:45:12Z,35.028848,-87.192328,215.1,7,1.70") == 0,
          "CSV row is byte-exact: '%s'", row);
    CHECK(row[strlen(row) - 1] != '\n', "CSV row carries no newline (the writer adds it)");
    CHECK(st.count() == 1 && st.total() == 1, "one mark held (count=%d total=%lu)",
          st.count(), (unsigned long)st.total());
    CHECK(st.at(0).id == 1, "mark id = %u (want 1)", (unsigned)st.at(0).id);
    CHECK(strcmp(st.at(0).utc, "2026-10-26T09:45:12Z") == 0,
          "mark utc = '%s'", st.at(0).utc);

    // The row must read back as the same numbers at 6 decimal places, because
    // that is what tools/topo_map.py will parse.
    unsigned id = 0, sats = 0;
    double lat = 0, lon = 0;
    int n = sscanf(row, "%u,%*[^,],%lf,%lf", &id, &lat, &lon);
    CHECK(n == 3 && id == 1, "row re-parses: id=%u lat=%.6f lon=%.6f", id, lat, lon);
    CHECK(fabs(lat - 35.028848) < 1e-6 && fabs(lon - -87.192328) < 1e-6,
          "row lat/lon survive a round trip");
    sats = 7;   // silence the unused warning on the shim-less build
    (void)sats;
  }

  // --------------------------------------------- capture: refusals are counted
  {
    WaypointStore st;
    char row[128] = {0};
    WptFix f = goodFix();
    f.posValid = false;
    CHECK(st.add(f, 1000, row, sizeof(row)) == WPT_NO_FIX, "no position -> NO_FIX");
    CHECK(st.refused() == 1 && st.total() == 0, "refusal counted, nothing stored (refused=%lu)",
          (unsigned long)st.refused());
    CHECK(st.lastRefusal() == WPT_NO_FIX, "lastRefusal = %d (want NO_FIX)", st.lastRefusal());

    f = goodFix();
    f.fixValid = false;
    CHECK(st.add(f, 1000, row, sizeof(row)) == WPT_NO_FIX,
          "sentences flowing but no fix -> NO_FIX");

    f = goodFix();
    f.stale = true;
    CHECK(st.add(f, 1000, row, sizeof(row)) == WPT_STALE,
          "stale fix -> STALE (not a silent success)");
    CHECK(st.refused() == 3, "three refusals counted (%lu)", (unsigned long)st.refused());
    CHECK(st.nextId() == 1, "a refused mark does not burn an id (next=%u)",
          (unsigned)st.nextId());

    // A refused mark must never have written a row.
    CHECK(row[0] == 0, "refused mark wrote no CSV row ('%s')", row);
  }

  // ------------------------------------------- capture: partial data is honest
  {
    WaypointStore st;
    char row[128] = {0};
    WptFix f = goodFix();
    f.altValid = false;
    f.hdopValid = false;
    CHECK(st.add(f, 5000, row, sizeof(row)) == WPT_OK,
          "a fix with no altitude/HDOP is still a markable position");
    CHECK(strcmp(row, "1,2026-10-26T09:45:12Z,35.028848,-87.192328,-,7,-") == 0,
          "missing fields are '-' not 0: '%s'", row);

    f = goodFix();
    f.clockValid = false;
    CHECK(st.add(f, 123456, row, sizeof(row)) == WPT_OK, "mark accepted before a UTC clock");
    CHECK(strcmp(row, "2,boot+123s,35.028848,-87.192328,215.1,7,1.70") == 0,
          "no clock -> boot-relative time: '%s'", row);
  }

  // ------------------------------------------------------- ids and the ring
  {
    WaypointStore st;
    st.setNextId(50);
    char row[128];
    CHECK(st.add(goodFix(), 1000, row, sizeof(row)) == WPT_OK && st.lastId() == 50,
          "setNextId(50) -> first mark is #%u", (unsigned)st.lastId());

    // Fill past WPT_MAX: the ring drops the oldest, the total keeps counting,
    // because the card is the real store and the RAM ring is only a window.
    st.setNextId(1);
    for (int i = 0; i < WPT_MAX + 5; ++i) st.add(goodFix(), 2000 + i * 10, row, sizeof(row));
    CHECK(st.count() == WPT_MAX, "ring holds WPT_MAX = %d (got %d)", WPT_MAX, st.count());
    CHECK(st.total() == (uint32_t)(WPT_MAX + 6), "total counts every accepted mark (%lu)",
          (unsigned long)st.total());
    CHECK(st.at(0).id == WPT_MAX + 5, "newest is the last one added (#%u)", (unsigned)st.at(0).id);
    // 38 marks have passed through a 32-slot window, so the 6 oldest are gone
    // (#50, then #1..#5). The second batch was numbered 1..37, so the oldest kept
    // is #6: 37 - (WPT_MAX - 1).
    const uint32_t n2 = WPT_MAX + 5;
    uint32_t oldestHeld = n2 - (WPT_MAX - 1);
    CHECK(st.at(st.count() - 1).id == oldestHeld, "oldest held is #%u (want %lu)",
          (unsigned)st.at(st.count() - 1).id, (unsigned long)oldestHeld);
  }

  // ------------------------------------------------------------- geo maths
  {
    const double lat = 35.0288, lon = -87.1921;
    CHECK(wptDistanceM(lat, lon, lat, lon) == 0.0, "distance to the same point = 0");

    // Spherical vs ellipsoidal: a true 1 km north offset measures ~1002 m on a
    // 6371 km sphere. That 0.24% (~2.4 m/km) is the documented cost of using
    // haversine, and it is below the receiver's own horizontal error.
    double dN = wptDistanceM(lat, lon, northOf(lat, 1000.0), lon);
    CHECK(dN > 995 && dN < 1010, "1 km north measures %.1f m on the sphere", dN);
    double bN = wptBearingDeg(lat, lon, northOf(lat, 1000.0), lon);
    CHECK(fabs(bN - 0.0) < 0.2 || fabs(bN - 360.0) < 0.2, "bearing due north = %.2f deg", bN);

    double dS = wptDistanceM(lat, lon, northOf(lat, -1000.0), lon);
    double bS = wptBearingDeg(lat, lon, northOf(lat, -1000.0), lon);
    CHECK(dS > 995 && dS < 1010, "1 km south measures %.1f m", dS);
    CHECK(fabs(bS - 180.0) < 0.2, "bearing due south = %.2f deg", bS);

    double dE = wptDistanceM(lat, lon, lat, eastOf(lat, lon, 1000.0));
    double bE = wptBearingDeg(lat, lon, lat, eastOf(lat, lon, 1000.0));
    CHECK(dE > 995 && dE < 1010, "1 km east measures %.1f m", dE);
    CHECK(bE > 89 && bE < 91, "bearing due east = %.2f deg (want ~90)", bE);

    double dW = wptDistanceM(lat, lon, lat, eastOf(lat, lon, -1000.0));
    double bW = wptBearingDeg(lat, lon, lat, eastOf(lat, lon, -1000.0));
    CHECK(dW > 995 && dW < 1010, "1 km west measures %.1f m", dW);
    CHECK(bW > 269 && bW < 271, "bearing due west = %.2f deg (want ~270)", bW);

    // A mark ~100 m away, the scale a mark is actually used at.
    double lat2 = northOf(lat, 100.0), lon2 = eastOf(lat, lon, 100.0);
    double d = wptDistanceM(lat, lon, lat2, lon2);
    double b = wptBearingDeg(lat, lon, lat2, lon2);
    CHECK(d > 138 && d < 144, "100 m north + 100 m east = %.1f m (want ~141.4)", d);
    CHECK(b > 44 && b < 46, "…bearing = %.1f deg (want ~45)", b);
  }

  // ------------------------------------------------------------- compass
  {
    char w[8];
    struct { double b; const char* want; } cases[] = {
        {0, "N"}, {22.5, "NNE"}, {45, "NE"}, {90, "E"}, {135, "SE"},
        {180, "S"}, {225, "SW"}, {270, "W"}, {315, "NW"}, {359, "N"}, {360, "N"},
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
      wptCompass(w, sizeof(w), cases[i].b);
      CHECK(strcmp(w, cases[i].want) == 0, "compass %.1f deg = %s (want %s)",
            cases[i].b, w, cases[i].want);
    }
    wptCompass(w, sizeof(w), -45.0);          // negative wraps instead of going blank
    CHECK(strcmp(w, "NW") == 0, "compass -45 deg = %s (want NW)", w);
  }

  printf("---------------------------------------------------------------\n");
  printf("%d checks, %d failed\n\n", checks, failures);
  return failures ? 1 : 0;
}
