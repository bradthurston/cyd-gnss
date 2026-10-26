// waypoints.h — marked locations: capture rules, RAM ring, CSV row, geo maths.
//
// Deliberately free of Arduino and SD so the whole thing runs in the PC host
// tests. The two things worth testing that way are the ones that fail silently:
// the CSV row (a mark that parses wrong on the PC is worthless a fortnight
// later) and the distance/bearing maths.
//
// One rule, enforced in one place: a mark is only ever created from a position
// that is valid AND fresh. Refusing to mark is a feature. A coordinate with no
// position behind it cannot be told apart from a good one once it is on the
// card, so a bad mark is worse than no mark.
#pragma once

#include <stdint.h>
#include <stddef.h>

#define WPT_MAX 32                                    // held in RAM; the card keeps all
#define WPT_CSV_HEADER "id,utc,lat,lon,alt_msl,sats,hdop"
#define WPT_CSV_FILE "waypoints.csv"

enum WptResult {
  WPT_OK = 0,
  WPT_NO_FIX,     // no usable position: nothing heard, or the receiver has not fixed yet
  WPT_STALE,      // a fix existed and updates stopped
};

// The parts of a fix a mark needs, decoupled from GnssSnapshot so the host tests
// can build one by hand.
struct WptFix {
  bool posValid;        // lat/lon usable
  bool fixValid;        // receiver claims a fix AND it is current (not stale)
  bool stale;           // had a fix, updates stopped
  double lat, lon;
  float altM;  bool altValid;
  uint8_t satsUsed;
  float hdop;  bool hdopValid;
  bool clockValid;      // UTC date + time known, so the mark can name its own time
  uint16_t year; uint8_t month, day, hour, minute, second;
};

struct Waypoint {
  uint16_t id;
  char utc[24];         // "2026-10-26T09:45:12Z", or "boot+123s" before the first fix
  double lat, lon;
  float altM;  bool altValid;
  uint8_t sats;
  float hdop;  bool hdopValid;
  uint32_t atMs;        // millis() when the mark was taken
};

class WaypointStore {
 public:
  // Ids continue from whatever is already on the card, so mark #12 after a
  // reboot is still #12 in the file rather than a second #1.
  void setNextId(uint16_t id) { nextId_ = id ? id : 1; }
  uint16_t nextId() const { return nextId_; }
  uint16_t lastId() const { return lastId_; }
  int count() const { return count_; }          // marks held in RAM right now
  uint32_t total() const { return total_; }     // every mark accepted this session
  uint32_t refused() const { return refused_; }
  WptResult lastRefusal() const { return lastRefusal_; }
  const Waypoint& at(int newestIndex) const;    // 0 = newest

  // An accepted mark gets its CSV row written into `csv` (no trailing newline).
  WptResult add(const WptFix& fix, uint32_t atMs, char* csv, size_t csvLen);

 private:
  Waypoint ring_[WPT_MAX] = {};
  int start_ = 0, count_ = 0;
  uint16_t nextId_ = 1, lastId_ = 0;
  uint32_t total_ = 0, refused_ = 0;
  WptResult lastRefusal_ = WPT_OK;
};

const char* wptResultText(WptResult r);

// Spherical, R = 6371.0088 km. That is ~0.3% (about 3 m per km) against the
// ellipsoid, which is well below the accuracy of the marks themselves - the
// receiver's own horizontal error is metres.
double wptDistanceM(double lat1, double lon1, double lat2, double lon2);
// Initial great-circle bearing from point 1 to point 2, degrees true, 0..360.
double wptBearingDeg(double lat1, double lon1, double lat2, double lon2);
void wptCompass(char* out, size_t n, double bearingDeg);   // "N" .. "NNW", 16 winds
int wptFormatRow(const Waypoint& w, char* out, size_t n);  // chars written, no newline
