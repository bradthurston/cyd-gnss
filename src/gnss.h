// gnss.h — passive NMEA 0183 reader + fix state machine.
//
// Design rules (from Brad's spec):
//   * Passive only. We never write a byte to the receiver, so its saved
//     configuration is untouched.
//   * Talker-agnostic: any xxGGA / xxRMC / xxGSA / xxGSV is accepted, because
//     a GPS+BeiDou receiver emits GP*, BD* and sometimes GN* in the same stream.
//   * Multi-part sentences (GSV part 1..n, GSA per system) are all accepted;
//     see Snapshot::gsvPartsSeen for how many arrived.
//   * Every field is optional. A field the receiver never sends stays at its
//     "unavailable" sentinel, and the UI prints "--" for it.
#pragma once

#include <Arduino.h>

#define GNSS_LINE_MAX 110
// Deep enough that filtering the Raw view still has history to show: a 14-line
// ring holds barely one second of this receiver's 12 sentences/second, so
// filtering to GGA would leave a single row on screen.
#define GNSS_RING 96
#define GNSS_MAX_SATS 48

enum GnssState {
  ST_NO_DATA = 0,      // nothing has ever arrived, or the stream stopped
  ST_DATA_NO_FIX,      // sentences flowing, receiver says no fix yet
  ST_FIX,              // valid fix, fresh
  ST_STALE,            // we had a fix, updates stopped (receiver still talking)
};

struct NmeaLine {
  char text[GNSS_LINE_MAX];
  uint32_t atMs;
  uint8_t talker;      // 0 = unknown
  char type[4];        // "GGA", "RMC", ...
  bool checksumOk;
};

// One tracked satellite. prn == 0 means the slot is unused.
struct GnssSat {
  uint8_t prn;
  char talker[3];      // "GP" GPS, "BD" BeiDou, "GL" GLONASS, "GA" Galileo
  int8_t elevDeg;      // 0..90, -1 = not reported
  int16_t azDeg;       // 0..359, -1 = not reported
  int8_t snr;          // dB-Hz, -1 = not reported
  bool used;           // listed in GSA as used in the current solution
  uint32_t heardMs;    // millis() of the last GSV mention
};

struct GnssSnapshot {
  GnssState state;
  char stateText[16];

  // link-level
  uint32_t bytesTotal;
  uint32_t linesTotal;
  uint32_t linesPerSec;
  uint32_t msSinceByte;
  uint32_t msSinceFix;
  uint32_t msSinceBoot;
  bool firstByteSeen;

  // what the receiver is actually emitting
  char talkers[40];         // unique 2-char prefixes, space separated
  uint8_t talkerCount;
  char typesSeen[40];       // unique 3-char sentence types
  uint8_t typeCount;
  uint32_t badChecksums;
  uint8_t gsvPartsThisCycle;   // highest part number seen in current cycle

  // fix (from GGA / RMC / GSA)
  bool fixValid;
  uint8_t fixQuality;          // GGA field 6: 0 none, 1 GPS, 2 DGPS, 4 RTK, 5 float
  char fixQualityText[14];     // none / GPS / DGPS / RTK / RTK-float / ...
  uint8_t satsUsed;            // GGA field 7
  uint8_t satsInView;          // highest "total satellites in view" seen in GSV
  double lat, lon;             // decimal degrees
  bool latLonValid;
  float altM;
  bool altValid;
  float hdop, vdop, pdop;
  bool hdopValid, vdopValid, pdopValid;
  float geoidSepM;
  bool geoidValid;

  // kinematics (RMC + GSA)
  float speedKts, speedKph, courseDeg;
  bool speedValid, courseValid;

  // time (UTC)
  uint8_t hour, minute, second;
  uint8_t day, month;
  uint16_t year;
  bool timeValid, dateValid;

  uint32_t gsvSentences;
  uint32_t ggaSentences;
  uint32_t rmcSentences;

  // satellite table summary (Stage 2 views)
  uint8_t satsTracked;         // sats heard in GSV, summed across constellations
  uint8_t bestSnr;             // strongest signal in the table, dB-Hz
};

// Pin the receiver was found on at boot (GPS_AUTO_DETECT) or GPS_RX_PIN.
extern int gpsActivePin;

class GnssReader {
 public:
  void begin();
  void poll();                                  // call often; non-blocking
  const GnssSnapshot& snap() const { return s_; }
  int lineCount() const { return ringCount_; }
  const NmeaLine& line(int newestIndex) const;  // 0 = newest
  void clearLines();

  // Satellite table (Stage 2). Index 0..GNSS_MAX_SATS-1; slots with prn == 0
  // are unused. Satellites are keyed by (talker, prn) so a GPS PRN 5 and a
  // BeiDou PRN 5 stay distinct.
  int satCount() const { return satCount_; }
  const GnssSat& sat(int i) const { return sats_[i]; }

  // Test hook: hand the parser one already-formed sentence (no CR/LF). The
  // host tests use this; the firmware always goes through poll()/feed().
  void injectLine(const char* sentence);

 private:
  void feed(uint8_t b);
  void handleLine(char* line);
  void handleGga(char* body);
  void handleRmc(char* body);
  void handleGsa(char* body);
  void handleGsv(char* body);
  void mergeGsvSat(int prn, int elev, int az, int snr);
  void applyGsaSatUse(char* f[], int n);
  void recountSats();
  void noteTalkerType(const char* line);
  void pushLine(const char* text, uint8_t talker, const char* type, bool ok);
  void classify();

  GnssSnapshot s_ = {};
  NmeaLine ring_[GNSS_RING] = {};
  int ringHead_ = 0;
  int ringCount_ = 0;
  char buf_[GNSS_LINE_MAX];
  int bufLen_ = 0;
  bool dropUntilNewline_ = false;
  uint32_t lastByteMs_ = 0;
  uint32_t lastFixMs_ = 0;
  uint32_t secWindowStart_ = 0;
  uint32_t secWindowLines_ = 0;
  char talkerList_[40] = {};
  char typeList_[40] = {};
  char curTalker_[3] = {};                     // talker of the sentence being handled
  GnssSat sats_[GNSS_MAX_SATS] = {};
  uint8_t satCount_ = 0;
};
