// Host-side parser tests for the GNSS explorer.
//
// These run the REAL parser (src/gnss.cpp is #included, not reimplemented) on a
// PC, because the parser is the one part of this project that can be fully
// verified without the receiver: NMEA is a text protocol and the expected values
// are arithmetic, not opinions.
//
// The field offsets in particular deserve this: an earlier revision read every
// sentence one field to the left (latitude from the hemisphere letter, fix
// quality from the satellite count). With no live data it compiled, ran, and
// looked fine. Nothing but a real sentence would have caught it.
//
//   ./tools/host_tests/run.sh
#include <Arduino.h>

#include <string>

// Definitions for the shim's globals declared in Arduino.h
uint32_t g_hostMillis = 1000;
HostSerial Serial;
HostSerial Serial1;

#include "gnss.h"
#include "gnss.cpp"   // the real parser source, compiled into this test

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...)                        \
  do {                                          \
    checks++;                                   \
    if (!(cond)) {                              \
      failures++;                               \
      printf("FAIL  ");                         \
      printf(__VA_ARGS__);                      \
      printf("   (%s:%d)\n", __FILE__, __LINE__); \
    } else {                                    \
      printf("ok    ");                         \
      printf(__VA_ARGS__);                      \
      printf("\n");                             \
    }                                           \
  } while (0)

// Build a sentence with a correct checksum so the tests exercise parsing rather
// than my ability to copy checksums out of a manual.
static std::string nmea(const char* body) {
  std::string s = std::string("$") + body;
  uint8_t sum = 0;
  for (size_t i = 1; i < s.size(); ++i) sum ^= (uint8_t)s[i];
  char tail[8];
  snprintf(tail, sizeof(tail), "*%02X", sum);
  return s + tail;
}

static GnssReader gnss;

static void inject(const char* body) {
  std::string s = nmea(body);
  gnss.injectLine(s.c_str());
}

// One "second" of receiver output, as a GPS+BeiDou unit actually emits it.
static void feedCycle() {
  g_hostMillis += 1000;
  inject("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00");
  inject("GPGSV,3,2,11,15,32,045,20,17,10,200,15,19,60,090,40,22,25,300,30");
  inject("GPGSV,3,3,11,24,45,180,35,28,20,150,22");
  inject("BDGSV,1,1,03,05,40,120,33,07,25,220,28,09,70,010,41");
  inject("GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1");
  inject("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00");
  inject("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
  inject("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
  gnss.poll();
}

int main() {
  printf("\nGNSS parser host tests (real src/gnss.cpp)\n");
  printf("---------------------------------------------------------------\n");

  // ---------------------------------------------------------------- GGA
  gnss.begin();
  g_hostMillis += 1000;
  inject("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
  const GnssSnapshot& s = gnss.snap();

  CHECK(s.fixQuality == 1, "GGA fix quality = %u (want 1)", s.fixQuality);
  CHECK(s.satsUsed == 8, "GGA sats used = %u (want 8)", s.satsUsed);
  CHECK(fabs(s.hdop - 0.9) < 0.001, "GGA hdop = %.3f (want 0.9)", s.hdop);
  CHECK(fabs(s.altM - 545.4) < 0.01, "GGA altitude = %.1f m (want 545.4)", s.altM);
  CHECK(fabs(s.geoidSepM - 46.9) < 0.01, "GGA geoid sep = %.1f m (want 46.9)", s.geoidSepM);
  CHECK(fabs(s.lat - 48.11730) < 0.0005, "GGA lat = %.5f (want 48.11730)", s.lat);
  CHECK(fabs(s.lon - 11.516667) < 0.0005, "GGA lon = %.5f (want 11.51667)", s.lon);
  CHECK(s.hour == 12 && s.minute == 35 && s.second == 19,
        "GGA time = %02u:%02u:%02u (want 12:35:19)", s.hour, s.minute, s.second);
  CHECK(s.latLonValid && s.altValid, "GGA validity flags set");
  CHECK(strcmp(s.fixQualityText, "GPS fix") == 0, "GGA quality text = '%s'", s.fixQualityText);

  // ------------------------------- what gets logged must be byte-exact raw NMEA
  // Regression: handleLine() used to tokenize the LIVE buffer, so the sentence
  // stored for the Raw tab and the SD card was whatever was left when the parser
  // had finished with it — a prefix ending at the first NUL'd comma, with the
  // checksum stripped. On the card that meant GGA/RMC/GSV/GSA truncated at their
  // 2nd field and no checksums anywhere, while the on-screen parser looked fine.
  {
    const char* ggaBody = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    std::string full = nmea(ggaBody);
    gnss.begin();
    gnss.injectLine(full.c_str());
    CHECK(strcmp(gnss.line(0).text, full.c_str()) == 0,
          "logged GGA is byte-exact ('%s')", gnss.line(0).text);
    CHECK(strlen(gnss.line(0).text) == full.size(),
          "logged GGA length = %u (want %u)", (unsigned)strlen(gnss.line(0).text),
          (unsigned)full.size());
    CHECK(strchr(gnss.line(0).text, '*') != nullptr, "logged GGA keeps its checksum");
    CHECK(fabs(gnss.snap().altM - 545.4) < 0.01,
          "logged GGA still parsed (alt %.1f, want 545.4)", gnss.snap().altM);

    const char* gsvBody = "GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00";
    std::string gsvFull = nmea(gsvBody);
    gnss.begin();
    gnss.injectLine(gsvFull.c_str());
    CHECK(strcmp(gnss.line(0).text, gsvFull.c_str()) == 0,
          "logged GSV keeps all 4 satellite groups ('%s')", gnss.line(0).text);
    CHECK(gnss.snap().satsInView == 11, "…and still parsed (in view %u, want 11)",
          gnss.snap().satsInView);
  }

  // ---------------------------------------------------------------- RMC
  gnss.begin();
  inject("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
  CHECK(fabs(s.speedKts - 22.4) < 0.001, "RMC speed = %.1f kt (want 22.4)", s.speedKts);
  CHECK(fabs(s.speedKph - 41.48) < 0.1, "RMC speed = %.2f kph (want 41.48)", s.speedKph);
  CHECK(fabs(s.courseDeg - 84.4) < 0.001, "RMC course = %.1f deg (want 84.4)", s.courseDeg);
  CHECK(s.day == 23 && s.month == 3 && s.year == 1994,
        "RMC date = %04u-%02u-%02u (want 1994-03-23)", s.year, s.month, s.day);
  CHECK(fabs(s.lat - 48.11730) < 0.0005, "RMC lat = %.5f (want 48.11730)", s.lat);
  CHECK(fabs(s.lon - 11.516667) < 0.0005, "RMC lon = %.5f (want 11.51667)", s.lon);
  CHECK(s.fixValid, "RMC status 'A' counts as fix evidence");

  // ---------------------------------------------------------------- GSA
  gnss.begin();
  inject("GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1");
  CHECK(fabs(s.pdop - 2.5) < 0.001, "GSA pdop = %.1f (want 2.5)", s.pdop);
  CHECK(fabs(s.hdop - 1.3) < 0.001, "GSA hdop = %.1f (want 1.3)", s.hdop);
  CHECK(fabs(s.vdop - 2.1) < 0.001, "GSA vdop = %.1f (want 2.1)", s.vdop);
  CHECK(s.fixValid, "GSA fix type 3 = valid fix");

  // ---------------------------------------------------------------- GSV
  gnss.begin();
  inject("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00");
  CHECK(s.satsInView == 11, "GSV sats in view = %u (want 11)", s.satsInView);
  CHECK(s.gsvPartsThisCycle == 1, "GSV cycle part = %u (want 1)", s.gsvPartsThisCycle);
  CHECK(gnss.satCount() == 4, "GSV part 1 tracked %d sats (want 4)", gnss.satCount());

  inject("GPGSV,3,2,11,15,32,045,20,17,10,200,15,19,60,090,40,22,25,300,30");
  CHECK(s.gsvPartsThisCycle == 2, "GSV cycle part = %u (want 2)", s.gsvPartsThisCycle);
  CHECK(gnss.satCount() == 8, "GSV part 2 tracked %d sats (want 8)", gnss.satCount());

  inject("GPGSV,3,3,11,24,45,180,35,28,20,150,22");
  CHECK(s.gsvPartsThisCycle == 0, "GSV cycle closed after last part = %u", s.gsvPartsThisCycle);
  CHECK(gnss.satCount() == 10, "GSV part 3 tracked %d sats (want 10)", gnss.satCount());
  CHECK(s.satsTracked == 10, "snapshot satsTracked = %u (want 10)", s.satsTracked);

  // a satellite's own numbers, and that a later cycle updates rather than adds
  const GnssSat* p19 = nullptr;
  for (int i = 0; i < GNSS_MAX_SATS; ++i) {
    if (gnss.sat(i).prn == 19 && strncmp(gnss.sat(i).talker, "GP", 2) == 0) p19 = &gnss.sat(i);
  }
  CHECK(p19 != nullptr, "GP PRN 19 present in the table");
  if (p19) {
    CHECK(p19->elevDeg == 60 && p19->azDeg == 90, "GP 19 el/az = %d/%d (want 60/90)",
          p19->elevDeg, p19->azDeg);
    CHECK(p19->snr == 40, "GP 19 snr = %d (want 40)", p19->snr);
    CHECK(!p19->used, "GP 19 not marked used before GSA says so");
  }
  CHECK(s.bestSnr == 40, "best SNR = %u dB-Hz (want 40)", s.bestSnr);

  // ------------------------------------------- constellations do not collide
  // Put GPS PRN 5 in the table too, so the (talker, prn) key is actually tested:
  // BeiDou 5 and GPS 5 must coexist.
  inject("BDGSV,1,1,03,05,40,120,33,07,25,220,28,09,70,010,41");
  inject("GPGSV,1,1,03,05,50,315,45,09,30,060,38,12,20,240,26");
  int gp5 = 0, bd5 = 0;
  for (int i = 0; i < GNSS_MAX_SATS; ++i) {
    const GnssSat& st = gnss.sat(i);
    if (st.prn != 5) continue;
    if (strncmp(st.talker, "GP", 2) == 0) gp5++;
    if (strncmp(st.talker, "BD", 2) == 0) bd5++;
  }
  CHECK(gp5 == 1 && bd5 == 1, "PRN 5 held per constellation (GP=%d BD=%d, want 1/1)", gp5, bd5);

  // ------------------------------------------------- GSA marks the used set
  inject("GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1");
  int usedGp = 0, usedBd = 0;
  for (int i = 0; i < GNSS_MAX_SATS; ++i) {
    const GnssSat& st = gnss.sat(i);
    if (!st.used) continue;
    if (strncmp(st.talker, "GP", 2) == 0) usedGp++;
    if (strncmp(st.talker, "BD", 2) == 0) usedBd++;
  }
  CHECK(usedGp == 5, "GSA marked %d GPS sats used (want 5: 04,05,09,12,24)", usedGp);
  CHECK(usedBd == 0, "GSA for GP left %d BeiDou sats marked used (want 0)", usedBd);

  // ------------------------------------------------------- full live cycle
  gnss.begin();
  feedCycle();
  CHECK(s.state == ST_FIX, "full cycle state = %d (want FIX=%d)", s.state, ST_FIX);
  CHECK(s.satsTracked == 13, "full cycle tracked %u sats (want 13)", s.satsTracked);
  CHECK(fabs(s.lat - 48.11730) < 0.0005, "full cycle lat = %.5f (want 48.11730)", s.lat);

  // ------------------------------------------------- malformed input is safe
  gnss.begin();
  std::string bad = nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
  bad[bad.size() - 1] = (bad[bad.size() - 1] == '0') ? '1' : '0';   // corrupt checksum
  // A sentence whose checksum fails is counted as a line but never parsed, so it
  // cannot inject a bogus position or fix state.
  gnss.injectLine(bad.c_str());
  CHECK(s.linesTotal == 1, "corrupt sentence still counted as a line = %lu",
        (unsigned long)s.linesTotal);
  CHECK(s.ggaSentences == 0, "corrupt sentence NOT counted as parsed GGA = %lu",
        (unsigned long)s.ggaSentences);
  CHECK(s.badChecksums == 1, "corrupt checksum counted = %lu", (unsigned long)s.badChecksums);
  CHECK(s.fixQuality == 0, "corrupt sentence did NOT set fix quality (%u)", s.fixQuality);
  CHECK(!s.latLonValid, "corrupt sentence did NOT set a position");
  CHECK(!s.fixValid, "corrupt sentence did NOT claim a fix");

  // ------------------------------------------------- no-data state
  gnss.begin();
  gnss.poll();
  CHECK(s.state == ST_NO_DATA, "no bytes at all = NO DATA (got %d)", s.state);

  // ------------------------------------------------- satellite staleness
  gnss.begin();
  feedCycle();
  uint8_t tracked = s.satsTracked;
  g_hostMillis += 20000;                      // nothing heard for 20s
  inject("GPGSV,1,1,01,31,12,045,18");        // only one satellite still reported
  CHECK(tracked == 13, "before the gap: %u tracked", tracked);
  CHECK(s.satsTracked == 1, "after a 20s gap only fresh sats counted = %u (want 1)",
        s.satsTracked);

  // ------------------------------------------------- real receiver capture
  // sample_nmea.txt is real output captured off the board's USB port with the
  // receiver wired to GPIO1 (2026-10-25, Minor Hill TN). Synthetic sentences do
  // not catch what real firmware emits: $GNGSA once per constellation with a
  // trailing systemID, multi-part $GPGSV with a signalID after the last quad,
  // and $GNGLL/$GNVTG/$GNZDA/$GPTXT sentences this parser ignores.
  {
    FILE* fh = fopen("tools/host_tests/sample_nmea.txt", "r");
    CHECK(fh != nullptr, "sample_nmea.txt opens (must run from the repo root)");
    if (fh) {
      char line[128];
      gnss.begin();
      int fed = 0;
      while (fgets(line, sizeof(line), fh)) {
        size_t len = strlen(line);
        while (len && (line[len - 1] == '\n' || line[len - 1] == '\r')) line[--len] = '\0';
        if (!len) continue;
        gnss.injectLine(line);
        fed++;
      }
      fclose(fh);
      gnss.poll();   // classify() runs in poll(); injectLine() only parses

      CHECK(fed >= 200, "fed %d captured sentences", fed);
      CHECK(s.badChecksums == 0, "real capture had %lu bad checksums (want 0)",
            (unsigned long)s.badChecksums);
      CHECK(s.state == ST_FIX, "real capture state = %d (want FIX=%d)", s.state, ST_FIX);
      CHECK(s.fixValid, "real capture reports a valid fix");
      CHECK(s.fixQuality == 1, "real capture fix quality = %u (want 1 = GPS)",
            s.fixQuality);
      CHECK(s.satsUsed == 7, "real capture sats used = %u (want 7 from the last GGA)",
            s.satsUsed);
      CHECK(s.hour == 19 && s.dateValid, "real capture time = %02u:%02u valid=%d",
            s.hour, s.minute, (int)s.dateValid);
      CHECK(s.month == 10 && s.day == 25 && s.year == 2026,
            "real capture date = %04u-%02u-%02u (want 2026-10-25)",
            s.year, s.month, s.day);
      CHECK(fabs(s.lat - 35.0288) < 0.01, "real capture lat = %.5f (want ~35.0288)", s.lat);
      CHECK(fabs(s.lon - -87.1921) < 0.01, "real capture lon = %.5f (want ~-87.1921)", s.lon);
      CHECK(s.bestSnr >= 25, "real capture best SNR = %u dB-Hz (want >= 25)", s.bestSnr);

      int gp = 0, bd = 0, other = 0, usedGp = 0, usedBd = 0;
      for (int i = 0; i < gnss.satCount(); ++i) {
        const GnssSat& st = gnss.sat(i);
        if (st.prn == 0) continue;
        if (strncmp(st.talker, "GP", 2) == 0) { gp++; if (st.used) usedGp++; }
        else if (strncmp(st.talker, "BD", 2) == 0) { bd++; if (st.used) usedBd++; }
        else other++;
      }
      CHECK(gp == 9, "real capture tracked %d GPS sats (want 9, from 3 GSV parts)", gp);
      CHECK(bd == 4, "real capture tracked %d BeiDou sats (want 4)", bd);
      // No satellite may be keyed to the GN talker: GN is the *combined* talker,
      // its GSA names satellites that GSV reports per constellation.
      CHECK(other == 0, "sats keyed to a non-GP/BD talker = %d (want 0)", other);
      CHECK(s.satsTracked == 13, "real capture tracked total = %u (want 13)", s.satsTracked);
      // The regression this test exists for: a $GNGSA for BeiDou follows the one
      // for GPS, and without the systemID mapping the second sentence wipes out
      // the first one's flags.
      CHECK(usedGp == 4, "real capture GPS used-in-fix = %d (want 4: 03,07,09,26)", usedGp);
      CHECK(usedBd == 3, "real capture BeiDou used-in-fix = %d (want 3: 12,19,20)", usedBd);
      CHECK(strstr(s.talkers, "BD") && strstr(s.talkers, "GN") && strstr(s.talkers, "GP"),
            "real capture talkers = '%s' (want GP, BD and GN all present)", s.talkers);
    }
  }

  printf("---------------------------------------------------------------\n");
  printf("%d checks, %d failed\n\n", checks, failures);
  return failures ? 1 : 0;
}
