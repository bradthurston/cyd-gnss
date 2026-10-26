// GNSS Explorer — Stage 5 bring-up: SD logging + marked places.
//
// Goal of this stage, and nothing more: prove that the display, the touch
// tabs and passive GPS reception all work together on one board.
//
//   * four tabs, tap to switch, ~250ms debounce (the stock XPT2046 lib
//     flickers during a hold and would otherwise fire twice per press)
//   * live link diagnostics: bytes, sentences, sentences/sec, talkers seen
//   * the four reception states, visually distinct:
//       NO DATA (grey) / DATA NO FIX (amber) / FIX (green) / STALE (red)
//   * raw NMEA tail, mirrored to USB serial for off-board inspection
//
// Stage 2: Satellites and Sky views are live, driven by the per-satellite
// table built from GSV (position + signal) and GSA (used in fix).
//
// Stage 3: a footer with PAUSE/RESUME and Raw filter chips (all / fix /
// satellites / receiver text), compact Info rows so nothing falls off the
// bottom, and "n/a" wherever the receiver simply reports no value. The
// receiver's saved configuration is read, never written: the sentence set it
// emits is the configuration, so the Info view lists it and names what this
// module does not provide at all.
//
// Stage 4: logging to the microSD card. The card and the touch controller sit on
// the SAME VSPI peripheral with different pins, so the bus is re-routed per
// access (see src/sdlog.h). The card records the raw NMEA stream - complete
// sentences, exactly as the receiver sent them - and nothing else, so anything
// done with the log later works from ground truth rather than our interpretation.
//
// Stage 5: marking places. One MARK button in the footer, on every tab, writes
// the current position to /gps/waypoints.csv and keeps the newest 32 in RAM so
// they can be read back and walked to on the panel. A mark is only taken from a
// fix that is valid AND current: refusing is the feature (see waypoints.h).
// Distances are spherical (~0.3%, ~3 m/km) which is far below the accuracy of
// the marks themselves.
#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include "gnss.h"
#include "sdlog.h"
#include "waypoints.h"   // marked locations: rules, RAM ring, CSV row, geo maths
#include <SD.h>   // the SD object itself: the probe talks to the card directly

// ------------------------------------------------------------- pin defaults
// ENABLE_USB_SERIAL=0 for a build whose GPS shares GPIO1 with the USB console:
// UART0's TX and a receiver's TX cannot both drive that pad, and the console is
// the one that has to give way.
#ifndef ENABLE_USB_SERIAL
#define ENABLE_USB_SERIAL 1
#endif

// Console TX pad. GPIO1 is UART0's default but it is also the pin a receiver's TX
// uses on this board, so a diagnostic build moves the console aside (GPIO22) to
// free GPIO1 for listening. The field build leaves this alone: its console is off.
#ifndef GPS_CONSOLE_TX
#define GPS_CONSOLE_TX 1
#endif
#ifndef TOUCH_CS
#define TOUCH_CS 33
#endif
#ifndef TOUCH_IRQ
#define TOUCH_IRQ 36
#endif
#ifndef TOUCH_SCLK
#define TOUCH_SCLK 25
#endif
#ifndef TOUCH_MOSI
#define TOUCH_MOSI 32
#endif
#ifndef TOUCH_MISO
#define TOUCH_MISO 39
#endif
// microSD on the SAME VSPI peripheral as the touch panel (CYD schematic, and
// Bruce's own saved config on this board agrees: sck 18 / miso 19 / mosi 23 / cs 5).
#ifndef SD_CS
#define SD_CS 5
#endif
#ifndef SD_SCLK
#define SD_SCLK 18
#endif
#ifndef SD_MISO
#define SD_MISO 19
#endif
#ifndef SD_MOSI
#define SD_MOSI 23
#endif

static const int SCREEN_W = 240;
static const int SCREEN_H = 320;
static const int TAB_H = 34;
static const int CONTENT_Y = TAB_H;

// Stage 3 footer. Pause/resume and the Raw filter chips live out here so they are
// always reachable and never scroll away with the content they control. Content
// views are clamped to CONTENT_BOTTOM, so a content repaint can never paint over
// the controls.
static const int FOOTER_H = 32;
static const int CONTENT_BOTTOM = SCREEN_H - FOOTER_H;              // 288
static const int BTN_Y = CONTENT_BOTTOM + 3, BTN_H = FOOTER_H - 6;  // 26px target
static const int PAUSE_X0 = 4, PAUSE_X1 = 72;
static const int MARK_X0 = 76, MARK_X1 = 118;
static const int CHIP_X0 = 122, CHIP_W = 27, CHIP_GAP = 2, CHIP_N = 4;

// tab geometry: six equal tabs across the width. 40px still fits a 4-char label
// at font 2 (proportional, ~7px/char) with a margin either side.
static const int TAB_COUNT = 6;
static const int TAB_W = SCREEN_W / TAB_COUNT;   // 40px
static const char* TAB_LABEL[TAB_COUNT] = {"Info", "Sats", "Sky", "Raw", "Log", "Mark"};

// --------------------------------------------------------------- raw touch
// Proportional fallback map from the documented CYD raw range. Good enough to
// START, but the tab bar is a stacked layout, so it needs a real 4-corner
// measurement before we trust it. The Raw tab prints raw + mapped so the
// corners can be sampled and the constants below replaced with measured ones.
static const int RAW_X_MIN = 200, RAW_X_MAX = 3700;
static const int RAW_Y_MIN = 240, RAW_Y_MAX = 3800;

static TFT_eSPI tft = TFT_eSPI();
// ONE bus object for two devices: the touch panel and the microSD card both
// live on VSPI but on different pins, so each access re-points this bus.
static SPIClass vspi(VSPI);
static XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
static GnssReader gnss;

// Stage 5: marks live in their own store (RAM window + CSV row) so the rules and
// the maths can be tested on a PC. The banner below is the on-screen answer to a
// press: every press produces either "marked #7" or a reason it was refused.
static WaypointStore wpts;
static char markResult[40] = {};
static int markResultLevel = 0;        // 0 ok, 1 warning (RAM only), 2 refused
static uint32_t markBannerUntil = 0;   // banner lifetime, millis()
static bool markBannerLive = false;    // set by drawContent while the banner is up
static bool wptIdInit = false;         // mark ids synced from the card yet?

static int activeTab = 0;
static bool needFullRedraw = true;
static uint32_t lastTouchMs = 0;
static int lastRawX = 0, lastRawY = 0, lastMappedX = 0, lastMappedY = 0;
static uint32_t tapCount = 0;

// Stage 3. Pause freezes the pixels, not the parser: receivers keep talking while
// you read, and nothing is lost - the ring keeps filling, so resuming jumps
// straight to current data instead of replaying from where it stopped.
static bool paused = false;
static int rawFilter = 0;      // 0 all, 1 fix, 2 satellites, 3 receiver text
static const char* FILTER_LABEL[CHIP_N] = {"ALL", "FIX", "SAT", "TXT"};

static const uint16_t C_BG = 0x0000;
static const uint16_t C_TABBAR = 0x2104;
static const uint16_t C_TAB_ON = 0x0250;   // deep blue
static const uint16_t C_TEXT = TFT_WHITE;
static const uint16_t C_DIM = 0x8410;
static const uint16_t C_ACCENT = TFT_CYAN;
static const uint16_t C_OK = TFT_GREEN;
static const uint16_t C_WARN = 0xFD20;     // amber
static const uint16_t C_BAD = TFT_RED;

static uint16_t stateColor(GnssState s) {
  switch (s) {
    case ST_FIX: return C_OK;
    case ST_DATA_NO_FIX: return C_WARN;
    case ST_STALE: return C_BAD;
    default: return C_DIM;
  }
}

// ------------------------------------------------------------------ drawing
static void drawTabBar() {
  tft.fillRect(0, 0, SCREEN_W, TAB_H, C_TABBAR);
  tft.setTextFont(2);
  tft.setTextSize(1);
  for (int i = 0; i < TAB_COUNT; ++i) {
    int x = i * TAB_W;
    bool on = (i == activeTab);
    if (on) tft.fillRect(x + 1, 1, TAB_W - 2, TAB_H - 2, C_TAB_ON);
    tft.drawRect(x, 0, TAB_W, TAB_H, C_DIM);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(on ? TFT_WHITE : C_DIM, on ? C_TAB_ON : C_TABBAR);
    tft.drawString(TAB_LABEL[i], x + TAB_W / 2, TAB_H / 2);
  }
  tft.setTextDatum(TL_DATUM);
}

// ------------------------------------------------------------------- footer
// Stage 3 controls. Drawn outside the content area so no content refresh can
// erase them, and repainted only when their own state changes. Pause halts the
// automatic 500ms refresh, but a deliberate touch (tab switch, filter change)
// still repaints - otherwise the controls look dead while paused.
static void drawFooter() {
  tft.fillRect(0, CONTENT_BOTTOM, SCREEN_W, FOOTER_H, C_TABBAR);
  tft.setTextFont(2);
  tft.setTextSize(1);

  const uint16_t btn = paused ? C_WARN : C_OK;
  tft.fillRoundRect(PAUSE_X0, BTN_Y, PAUSE_X1 - PAUSE_X0, BTN_H, 4, C_BG);
  tft.drawRoundRect(PAUSE_X0, BTN_Y, PAUSE_X1 - PAUSE_X0, BTN_H, 4, btn);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(btn, C_BG);
  tft.drawString(paused ? "RESUME" : "PAUSE", (PAUSE_X0 + PAUSE_X1) / 2, BTN_Y + BTN_H / 2);

  // MARK on every tab: a mark is taken while you are looking at something else.
  // Feedback is not swallowed - doMark() puts the outcome on screen either way.
  tft.fillRoundRect(MARK_X0, BTN_Y, MARK_X1 - MARK_X0, BTN_H, 4, C_BG);
  tft.drawRoundRect(MARK_X0, BTN_Y, MARK_X1 - MARK_X0, BTN_H, 4, C_ACCENT);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.drawString("MARK", (MARK_X0 + MARK_X1) / 2, BTN_Y + BTN_H / 2);

  if (activeTab == 3) {
    for (int i = 0; i < CHIP_N; ++i) {
      const int x = CHIP_X0 + i * (CHIP_W + CHIP_GAP);
      const bool on = (rawFilter == i);
      tft.fillRoundRect(x, BTN_Y, CHIP_W, BTN_H, 4, on ? C_TAB_ON : C_BG);
      tft.drawRoundRect(x, BTN_Y, CHIP_W, BTN_H, 4, on ? C_ACCENT : C_DIM);
      tft.setTextColor(on ? TFT_WHITE : C_DIM, on ? C_TAB_ON : C_BG);
      tft.drawString(FILTER_LABEL[i], x + CHIP_W / 2, BTN_Y + BTN_H / 2);
    }
  } else {
    // keep the reception state visible on every tab without hunting for Info
    const GnssSnapshot& s = gnss.snap();
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(paused ? C_WARN : stateColor(s.state), C_TABBAR);
    tft.drawString(paused ? "PAUSED" : s.stateText, SCREEN_W - 6, BTN_Y + BTN_H / 2);
  }
  tft.setTextDatum(TL_DATUM);
}

static void clearContent() {
  tft.fillRect(0, CONTENT_Y, SCREEN_W, CONTENT_BOTTOM - CONTENT_Y, C_BG);
}

// Clearing the whole content area and repainting it every refresh makes the panel
// strobe: the area stays dark for as long as the repaint takes, and on the Raw
// and Sky tabs that is a large slice of the cycle, so it reads as a ~1 Hz blink.
// A full clear is therefore only for a tab switch. Steady-state updates clear just
// the band they are about to draw, and whatever is left over below the last
// painted row is cleared once, centrally, at the end of drawContent().
static void clearRow(int y, int h) {
  if (y < CONTENT_Y) { h -= (CONTENT_Y - y); y = CONTENT_Y; }
  if (y + h > CONTENT_BOTTOM) h = CONTENT_BOTTOM - y;
  if (h > 0) tft.fillRect(0, y, SCREEN_W, h, C_BG);
}

static int rowBottom = SCREEN_H;   // lowest y the current tab painted

static int rowY = 0;
static void rowReset() { rowY = CONTENT_Y + 8; }

// label on the left, value right-aligned on the same line
static void rowKV(const char* label, const char* value, uint16_t vcol = C_TEXT) {
  if (rowY > CONTENT_BOTTOM - 16) return;
  clearRow(rowY - 2, 17);
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString(label, 6, rowY);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(vcol, C_BG);
  tft.drawString(value, SCREEN_W - 6, rowY);
  tft.setTextDatum(TL_DATUM);
  rowY += 18;
}

static void rowNote(const char* text, uint16_t col = C_DIM) {
  if (rowY > CONTENT_BOTTOM - 16) return;
  clearRow(rowY - 2, 17);
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(col, C_BG);
  tft.drawString(text, 6, rowY);
  rowY += 18;
}

// Two label/value pairs on one line. The single-column Info layout needed 22 rows
// and only 254px exist, so its last five rows - speed, course, UTC time, UTC date
// and fix quality - were being silently dropped off the bottom of the screen.
static void rowKV2(const char* l1, const char* v1, const char* l2, const char* v2,
                   uint16_t c1 = C_TEXT, uint16_t c2 = C_TEXT) {
  if (rowY > CONTENT_BOTTOM - 16) return;
  clearRow(rowY - 2, 17);
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString(l1, 6, rowY);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(c1, C_BG);
  tft.drawString(v1, 114, rowY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString(l2, 122, rowY);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(c2, C_BG);
  tft.drawString(v2, SCREEN_W - 6, rowY);
  tft.setTextDatum(TL_DATUM);
  rowY += 18;
}

// Font 1 row, for the reference lines at the bottom of the Info view.
static void rowTiny(const char* text, uint16_t col = C_DIM) {
  if (rowY > CONTENT_BOTTOM - 11) return;
  clearRow(rowY - 2, 12);
  tft.setTextFont(1);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(col, C_BG);
  tft.drawString(text, 6, rowY);
  tft.setTextFont(2);
  rowY += 12;
}

static void fmtCoord(char* out, size_t n, double v, bool valid, int width) {
  if (!valid) { snprintf(out, n, "--"); return; }
  snprintf(out, n, "%.*f", width, v);
}

// Stage 3: two pairs per line. The old single-column layout needed 22 rows for a
// 254px content area, so its bottom five rows were silently off-screen - speed,
// course, UTC time, UTC date and fix quality never showed at all. Every field is
// here now, and anything the receiver does not report says "n/a" instead of "--",
// which is the same convention as the Sats table.
static void drawOverview() {
  const GnssSnapshot& s = gnss.snap();
  char v[64], w[64];

  rowReset();
  tft.setTextFont(2);
  tft.setTextColor(stateColor(s.state), C_BG);
  tft.drawString("STATE", 6, rowY);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(stateColor(s.state), C_BG);
  tft.drawString(s.stateText, SCREEN_W - 6, rowY);
  tft.setTextDatum(TL_DATUM);
  rowY += 22;
  tft.drawFastHLine(6, rowY - 5, SCREEN_W - 12, C_DIM);

  snprintf(v, sizeof(v), "%lu", (unsigned long)s.bytesTotal);
  snprintf(w, sizeof(w), "%lu/s", (unsigned long)s.linesPerSec);
  rowKV2("RX bytes", v, "Rate", w);

  // Full width: "BD GN GP" plus a label does not fit a half-width column at
  // font 2, which is proportional (~7px/char, not the 8 of a fixed font).
  rowKV("Talkers", s.talkerCount ? s.talkers : "n/a", C_ACCENT);

  snprintf(v, sizeof(v), "%u", (unsigned)s.badChecksums);
  snprintf(w, sizeof(w), "%u", (unsigned)s.satsInView);
  rowKV2("Bad cksum", v, "In view", s.satsInView ? w : "n/a",
         s.badChecksums ? C_WARN : C_TEXT);

  snprintf(v, sizeof(v), "%u", (unsigned)s.satsUsed);
  snprintf(w, sizeof(w), "%s", s.fixQuality ? s.fixQualityText : "n/a");
  rowKV2("Sats used", s.satsUsed ? v : "n/a", "Fix", w,
         C_TEXT, s.fixQuality ? C_TEXT : C_DIM);

  fmtCoord(v, sizeof(v), s.lat, s.latLonValid, 6);
  rowKV("Latitude", v);
  fmtCoord(v, sizeof(v), s.lon, s.latLonValid, 6);
  rowKV("Longitude", v);

  if (s.altValid) snprintf(v, sizeof(v), "%.1fm", s.altM); else snprintf(v, sizeof(v), "n/a");
  if (s.hdopValid) snprintf(w, sizeof(w), "%.2f", s.hdop); else snprintf(w, sizeof(w), "n/a");
  rowKV2("Altitude", v, "HDOP", w);

  if (s.vdopValid) snprintf(v, sizeof(v), "%.2f", s.vdop); else snprintf(v, sizeof(v), "n/a");
  if (s.pdopValid) snprintf(w, sizeof(w), "%.2f", s.pdop); else snprintf(w, sizeof(w), "n/a");
  rowKV2("VDOP", v, "PDOP", w);

  if (s.speedValid) snprintf(v, sizeof(v), "%.1f km/h", s.speedKph); else snprintf(v, sizeof(v), "n/a");
  if (s.courseValid) snprintf(w, sizeof(w), "%.0f deg", s.courseDeg); else snprintf(w, sizeof(w), "n/a");
  rowKV2("Speed", v, "Course", w);

  if (s.timeValid) snprintf(v, sizeof(v), "%02u:%02u:%02u", s.hour, s.minute, s.second); else snprintf(v, sizeof(v), "n/a");
  if (s.dateValid) snprintf(w, sizeof(w), "%04u-%02u-%02u", s.year, s.month, s.day); else snprintf(w, sizeof(w), "n/a");
  rowKV2("Time", v, "Date", w);

  snprintf(v, sizeof(v), "%lu", (unsigned long)s.linesTotal);
  if (s.firstByteSeen) snprintf(w, sizeof(w), "%lums", (unsigned long)s.msSinceByte);
  else snprintf(w, sizeof(w), "never");
  rowKV2("Sentences", v, "Since byte", w, C_TEXT, s.msSinceByte > 5000 ? C_BAD : C_TEXT);

  // The receiver's saved configuration, read and never written: the sentence set
  // it is emitting IS the configuration, and what is absent is just as telling -
  // this module has no GLONASS, Galileo, QZSS, SBAS or DGPS at all.
  snprintf(v, sizeof(v), "emits: %.30s", s.typeCount ? s.typesSeen : "n/a");
  rowTiny(v, C_ACCENT);
  rowTiny("absent here: SBAS/DGPS/RTK, GLONASS,", C_DIM);
  rowTiny("Galileo, QZSS - config never written", C_OK);
}

// ------------------------------------------------------- satellite views
// Ordered index of tracked satellites: strongest signal first, and a satellite
// used in the current solution beats an equal-signal one that is not.
static int satOrder[GNSS_MAX_SATS];

static int orderSats() {
  int n = 0;
  for (int i = 0; i < GNSS_MAX_SATS; ++i) {
    if (gnss.sat(i).prn) satOrder[n++] = i;
  }
  for (int a = 0; a < n - 1; ++a) {
    int pick = a;
    for (int b = a + 1; b < n; ++b) {
      const GnssSat& x = gnss.sat(satOrder[pick]);
      const GnssSat& y = gnss.sat(satOrder[b]);
      if (y.snr > x.snr || (y.snr == x.snr && y.used && !x.used)) pick = b;
    }
    if (pick != a) {
      int t = satOrder[a]; satOrder[a] = satOrder[pick]; satOrder[pick] = t;
    }
  }
  return n;
}

static uint16_t snrColor(int snr) {
  if (snr < 0) return C_DIM;      // not reported
  if (snr == 0) return C_BAD;     // in view, no usable signal
  if (snr < 25) return C_WARN;
  return C_OK;
}

static void drawSats() {
  const GnssSnapshot& s = gnss.snap();
  char v[64];

  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  int y = CONTENT_Y + 8;

  snprintf(v, sizeof(v), "tracked %u   in view %u   ",
           (unsigned)s.satsTracked, (unsigned)s.satsInView);
  clearRow(y - 2, 17);
  tft.setTextColor(C_TEXT, C_BG);
  tft.drawString(v, 6, y);
  y += 18;

  if (s.satsTracked == 0) {
    clearRow(y - 2, 17);
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString("no satellite data yet", 6, y);
    y += 18;
    clearRow(y - 2, 17);
    tft.drawString("GSV sentences: ", 6, y);
    snprintf(v, sizeof(v), "%lu   ", (unsigned long)s.gsvSentences);
    tft.drawString(v, 120, y);
    y += 18;
    rowBottom = y;
    return;
  }

  clearRow(y - 2, 19);
  snprintf(v, sizeof(v), "best SNR %u dB-Hz   ", (unsigned)s.bestSnr);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.drawString(v, 6, y);
  y += 20;

  clearRow(y - 2, 15);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("SYS PRN EL  AZ  SNR", 6, y);
  y += 16;
  tft.setTextColor(C_DIM, C_BG);
  tft.drawFastHLine(6, y - 4, SCREEN_W - 12, C_DIM);

  const int maxRows = 9;    // + the legend row below, inside CONTENT_BOTTOM
  int n = orderSats();
  for (int i = 0; i < n && i < maxRows; ++i) {
    const GnssSat& st = gnss.sat(satOrder[i]);
    const uint16_t col = snrColor(st.snr);
    clearRow(y - 2, 17);
    tft.setTextColor(col, C_BG);

    // constellation tag with a * when the satellite is used in the fix
    snprintf(v, sizeof(v), "%c%c%c", st.talker[0], st.talker[1], st.used ? '*' : ' ');
    tft.drawString(v, 6, y);

    snprintf(v, sizeof(v), "%3u", (unsigned)st.prn);
    tft.drawString(v, 36, y);

    if (st.elevDeg >= 0) snprintf(v, sizeof(v), "%2d", st.elevDeg); else strcpy(v, "n/a");
    tft.drawString(v, 72, y);

    if (st.azDeg >= 0) snprintf(v, sizeof(v), "%3d", st.azDeg); else strcpy(v, "n/a");
    tft.drawString(v, 100, y);

    if (st.snr >= 0) snprintf(v, sizeof(v), "%2d", st.snr); else strcpy(v, "n/a");
    tft.drawString(v, 138, y);

    int barW = 0;
    if (st.snr > 0) barW = (st.snr * 56) / 50;
    if (barW > 56) barW = 56;
    tft.drawRect(168, y - 1, 60, 15, C_DIM);
    if (barW > 0) tft.fillRect(169, y, barW, 13, col);

    y += 18;
  }
  clearRow(y - 2, 17);
  if (n > maxRows) {
    snprintf(v, sizeof(v), "+%d more tracked", n - maxRows);
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString(v, 6, y);
  }
  // Some satellites are in view but never report elevation, azimuth or signal.
  // A dash there could be read as a zero reading; say unavailable instead.
  tft.setTextFont(1);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("n/a = not reported by the receiver", 6, CONTENT_BOTTOM - 11);
  tft.setTextFont(2);
  rowBottom = CONTENT_BOTTOM;   // nothing below this belongs to the view
}

static void drawSky() {
  const GnssSnapshot& s = gnss.snap();
  char v[32];
  const int R = 88;
  const int cx = SCREEN_W / 2;
  const int cy = CONTENT_Y + 26 + R;      // circle sits clear of the header

  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_TEXT, C_BG);
  snprintf(v, sizeof(v), "sky view   %u sats   ", (unsigned)s.satsTracked);
  clearRow(CONTENT_Y + 2, 16);
  tft.drawString(v, 6, CONTENT_Y + 4);

  // Wipe only the previous frame's markers, then redraw the static base over
  // those same boxes and lay the new markers down. Nothing large ever goes dark,
  // so the plot does not strobe like a full clear-and-repaint does.
  static int prevX[GNSS_MAX_SATS], prevY[GNSS_MAX_SATS], prevN = 0;
  for (int i = 0; i < prevN; ++i) {
    tft.fillRect(prevX[i] - 8, prevY[i] - 8, 30, 20, C_BG);
  }

  tft.drawCircle(cx, cy, R, C_DIM);
  tft.drawCircle(cx, cy, (R * 2) / 3, C_DIM);
  tft.drawCircle(cx, cy, R / 3, C_DIM);
  tft.drawFastHLine(cx - R, cy, 2 * R + 1, C_DIM);
  tft.drawFastVLine(cx, cy - R, 2 * R + 1, C_DIM);

  // compass letters inside the outer ring so they cannot collide with the header
  tft.setTextFont(1);
  tft.setTextColor(C_DIM, C_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("N", cx, cy - R + 3);
  tft.drawString("S", cx, cy + R - 12);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("W", cx - R + 3, cy - 4);
  tft.setTextDatum(TR_DATUM);
  tft.drawString("E", cx + R - 3, cy - 4);
  tft.setTextDatum(TL_DATUM);

  // North-up: azimuth 0 = up, 90 = right. Radius maps elevation 90 (zenith)
  // to the centre and 0 (horizon) to the outer ring.
  int n = orderSats();
  int drawn = 0;
  for (int i = 0; i < n; ++i) {
    const GnssSat& st = gnss.sat(satOrder[i]);
    if (st.elevDeg < 0 || st.azDeg < 0) continue;
    float az = st.azDeg * 0.0174533f;
    int r = (R * (90 - st.elevDeg)) / 90;
    int x = cx + (int)(r * sinf(az));
    int y = cy - (int)(r * cosf(az));
    const uint16_t col = snrColor(st.snr);

    tft.fillCircle(x, y, st.used ? 4 : 3, col);
    if (!st.used) tft.drawCircle(x, y, 6, C_DIM);   // tracked, not in the fix
    snprintf(v, sizeof(v), "%u", (unsigned)st.prn);
    tft.setTextColor(col, C_BG);
    tft.drawString(v, x + 7, y - 4);

    if (drawn < GNSS_MAX_SATS) { prevX[drawn] = x; prevY[drawn] = y; drawn++; }
  }
  prevN = drawn;
  rowBottom = CONTENT_BOTTOM;

  tft.setTextFont(1);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("rings: 60/30/0 deg elev", 6, CONTENT_BOTTOM - 40);
  tft.setTextColor(C_OK, C_BG);
  tft.drawString("green", 6, CONTENT_BOTTOM - 26);
  tft.setTextColor(C_WARN, C_BG);
  tft.drawString("amber", 50, CONTENT_BOTTOM - 26);
  tft.setTextColor(C_BAD, C_BG);
  tft.drawString("red", 96, CONTENT_BOTTOM - 26);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("./.. = SNR  ring = unused", 130, CONTENT_BOTTOM - 26);
  tft.drawString("SYS tag: GP=GPS BD=BeiDou", 6, CONTENT_BOTTOM - 14);
}

// Which sentences the Raw view is allowed to show. Matched on the parsed type,
// not by string-searching the line, so it works for any talker (GP/BD/GN).
static bool rawLineMatches(const NmeaLine& l) {
  const char* t = l.type;
  if (rawFilter == 1) {          // anything that carries the position/kinematics
    return !strcmp(t, "GGA") || !strcmp(t, "RMC") || !strcmp(t, "GLL") ||
           !strcmp(t, "VTG") || !strcmp(t, "ZDA");
  }
  if (rawFilter == 2) {          // satellite geometry and signal
    return !strcmp(t, "GSV") || !strcmp(t, "GSA");
  }
  if (rawFilter == 3) {          // the receiver's own health text
    return !strcmp(t, "TXT");
  }
  return true;                   // 0: everything
}

static void drawRaw() {
  const GnssSnapshot& s = gnss.snap();
  char v[80];

  // diagnostics first, then the scrolling sentence tail
  rowReset();
  snprintf(v, sizeof(v), "rx=%d baud=%d", gpsActivePin, GPS_BAUD);
  rowNote(v, C_ACCENT);
  snprintf(v, sizeof(v), "raw %d,%d -> %d,%d  taps %lu",
           lastRawX, lastRawY, lastMappedX, lastMappedY, (unsigned long)tapCount);
  rowNote(v, C_DIM);
  snprintf(v, sizeof(v), "%lu bytes %lu lines %lu/s %s",
           (unsigned long)s.bytesTotal, (unsigned long)s.linesTotal,
           (unsigned long)s.linesPerSec, FILTER_LABEL[rawFilter]);
  rowNote(v, C_DIM);
  tft.drawFastHLine(6, rowY - 2, SCREEN_W - 12, C_DIM);

  // Rows sized to whatever is left under the diagnostics, 40 chars at font 1.
  // The ring holds 96 sentences (~8s of this stream), so filtering still leaves
  // a full screen of history instead of one lonely row.
  tft.setTextFont(1);
  tft.setTextDatum(TL_DATUM);
  int rows = (CONTENT_BOTTOM - rowY - 4) / 12;
  if (rows > 14) rows = 14;
  if (rows < 1) rows = 1;
  const NmeaLine* pick[14];
  int shown = 0;
  for (int i = 0; i < gnss.lineCount() && shown < rows; ++i) {
    if (!rawLineMatches(gnss.line(i))) continue;
    pick[shown++] = &gnss.line(i);
  }
  for (int i = 0; i < shown; ++i) {
    const NmeaLine& l = *pick[i];
    // padded to the full 40 columns: a line that shrinks then repaints its own
    // glyph cells instead of leaving the old tail on screen, so no clearing
    char forty[41];
    memset(forty, ' ', 40);
    forty[40] = '\0';
    size_t len = strlen(l.text);
    if (len > 40) len = 40;
    memcpy(forty, l.text, len);
    tft.setTextColor(l.checksumOk ? C_TEXT : C_WARN, C_BG);
    tft.drawString(forty, 4, rowY + (shown - 1 - i) * 12);
  }
  if (shown == 0) {
    clearRow(rowY - 2, 14);
    tft.setTextColor(C_WARN, C_BG);
    tft.drawString("no sentences received", 6, rowY);
    shown = 1;
  }
  tft.setTextFont(2);
  rowBottom = rowY + shown * 12 + 4;
}

// Collects lines printed by the SD listing and draws them as rows afterwards.
// Bounded on purpose: a card with hundreds of logs must not exhaust RAM.
class LcdPrint : public Print {
 public:
  size_t write(uint8_t c) override {
    if (n_ >= LINES) return 1;
    if (c == '\n') { buf_[n_][len_[n_]] = 0; if (len_[n_] > 0) n_++; return 1; }
    if (len_[n_] < WIDTH - 1) buf_[n_][len_[n_]++] = (char)c;
    return 1;
  }
  void reset() { n_ = 0; for (int i = 0; i < LINES; ++i) len_[i] = 0; }
  void draw() {
    for (int i = 0; i < n_; ++i) rowTiny(buf_[i], C_DIM);
    if (n_ >= LINES) rowTiny("  ... more files on card", C_DIM);
  }
 private:
  static const int LINES = 5, WIDTH = 40;
  char buf_[LINES][WIDTH] = {};
  int len_[LINES] = {};
  int n_ = 0;
};

// ------------------------------------------------------------------ Log view
// Reads only: the card is written by the loop, not by the UI. The point of the
// tab is to answer "is it recording, where, and is it keeping up" without a
// laptop, because this build has no USB console.
static void drawLog() {
  const LogStatus& L = sdLog.st();
  char v[64], w[64];
  rowReset();
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("CARD LOG", 6, rowY);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(!L.busReady ? C_DIM : !L.cardMounted ? C_BAD : L.logging ? C_OK : C_WARN, C_BG);
  tft.drawString(!L.busReady ? "not in this build"
                 : !L.cardMounted ? "NO CARD"
                 : L.logging ? "LOGGING" : "waiting", SCREEN_W - 6, rowY);
  tft.setTextDatum(TL_DATUM);
  rowY += 22;
  tft.drawFastHLine(6, rowY - 5, SCREEN_W - 12, C_DIM);

  rowKV("File", L.file[0] ? L.file : "none yet");
  if (L.cardMounted)
    snprintf(v, sizeof(v), "type %u, %lu MB @ %lu MHz", L.cardType,
             (unsigned long)L.cardSizeMB, (unsigned long)(L.mountFreq / 1000));
  else snprintf(v, sizeof(v), "retrying every 5 s");
  rowKV("Card", v, L.cardMounted ? C_TEXT : C_DIM);

  snprintf(v, sizeof(v), "%lu", (unsigned long)L.linesWritten);
  snprintf(w, sizeof(w), "%lu B", (unsigned long)L.bytesWritten);
  rowKV2("Lines", v, "Bytes", w);

  snprintf(v, sizeof(v), "%lu", (unsigned long)L.flushes);
  snprintf(w, sizeof(w), "%lu", (unsigned long)L.busSwitches);
  rowKV2("Flushes", v, "Bus xfer", w);

  snprintf(v, sizeof(v), "%lu B", (unsigned long)L.pendingBytes);
  snprintf(w, sizeof(w), "%lu", (unsigned long)L.droppedLines);
  rowKV2("In RAM", v, "Dropped", w, C_TEXT, L.droppedLines ? C_BAD : C_TEXT);

  snprintf(v, sizeof(v), "%u", (unsigned)L.skippedShort);
  rowKV2("Short lines", v, "Errors", L.writeErrors ? "yes" : "0",
         C_DIM, L.writeErrors ? C_BAD : C_TEXT);

  rowNote(L.lastError[0] ? L.lastError : "no errors", L.lastError[0] ? C_BAD : C_OK);
  rowNote(L.clockText[0] ? L.clockText : "waiting for a UTC fix to name the file", C_DIM);
  rowTiny("raw NMEA only - PAUSE holds the pixels, NOT the log", C_ACCENT);
  rowTiny("card and touch share VSPI: 2 pin re-routes per flush", C_DIM);

  // Directory listing, so it is obvious the data survives a reboot. Cached for
  // 5 s: the Log tab repaints about twice a second (linesTotal keeps moving), and
  // a directory read costs two pin re-routes plus card time on every repaint.
  static LcdPrint lp;
  static uint32_t lastList = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastList > 5000) {
    lastList = nowMs;
    lp.reset();
    sdLog.listFiles(lp);
  }
  lp.draw();
  rowBottom = rowY;
}

// ------------------------------------------------------------------- marking
// One press, one mark. Every rule about whether a position is worth saving lives
// in WaypointStore::add(); this only hands it the current fix and then gets the
// row onto the card. A press always produces a visible answer, including when the
// answer is "no".
static void doMark() {
  const GnssSnapshot& s = gnss.snap();

  // Continue the card's numbering instead of starting a second #1. Done once, on
  // the first press, because it reads the file.
  if (!wptIdInit) {
    wpts.setNextId(sdLog.st().cardMounted ? (uint16_t)(sdLog.countWaypoints() + 1) : 1);
    wptIdInit = true;
  }

  WptFix f = {};
  f.posValid = s.latLonValid;
  f.fixValid = s.fixValid && s.state == ST_FIX;   // valid AND current, else refused
  f.stale = (s.state == ST_STALE);
  f.lat = s.lat;
  f.lon = s.lon;
  f.altValid = s.altValid;   f.altM = s.altM;
  f.satsUsed = s.satsUsed;
  f.hdopValid = s.hdopValid; f.hdop = s.hdop;
  f.clockValid = s.timeValid && s.dateValid;
  f.year = s.year; f.month = s.month; f.day = s.day;
  f.hour = s.hour; f.minute = s.minute; f.second = s.second;

  char row[96];
  const uint32_t nowMs = millis();
  const WptResult r = wpts.add(f, nowMs, row, sizeof(row));

  if (r != WPT_OK) {
    // Refusing is a result, not a silent failure: name the reason and keep count.
    snprintf(markResult, sizeof(markResult), "refused: %s (%lu so far)",
             wptResultText(r), (unsigned long)wpts.refused());
    markResultLevel = 2;
  } else if (sdLog.appendWaypoint(row, nowMs)) {
    char ev[52];
    snprintf(ev, sizeof(ev), "mark #%u %s", (unsigned)wpts.lastId(), wpts.at(0).utc);
    sdLog.event(ev);                        // breadcrumb in the NMEA log as well
    snprintf(markResult, sizeof(markResult), "marked #%u", (unsigned)wpts.lastId());
    markResultLevel = 0;
  } else {
    // Held, but only in RAM. Say so now, rather than after a reboot loses it.
    snprintf(markResult, sizeof(markResult), "marked #%u - RAM only",
             (unsigned)wpts.lastId());
    markResultLevel = 1;
    if (wpts.total() > (uint32_t)WPT_MAX) {
      snprintf(markResult, sizeof(markResult), "marked #%u - RAM full, oldest gone",
               (unsigned)wpts.lastId());
    }
  }

  markBannerUntil = nowMs + 2500;
  needFullRedraw = true;      // put the banner up at once
  Serial.printf("[mark] %s | %s\n", markResult, r == WPT_OK ? row : "no row written");
}

// ------------------------------------------------------------------ Marks tab
// What has been marked, and how far away each one is from where you are now.
// Distance and bearing are shown only when there is a CURRENT fix: a stale
// distance looks live and would send you the wrong way, a dash cannot.
static void drawMarks() {
  const LogStatus& L = sdLog.st();
  const GnssSnapshot& s = gnss.snap();
  const bool haveFix = s.latLonValid && s.state == ST_FIX;
  char v[80];

  rowReset();
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_ACCENT, C_BG);
  snprintf(v, sizeof(v), "MARKS %lu", (unsigned long)wpts.total());
  tft.drawString(v, 6, rowY);

  tft.setTextDatum(TR_DATUM);
  if (!L.busReady) {
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString("no card in this build", SCREEN_W - 6, rowY);
  } else if (!L.cardMounted) {
    tft.setTextColor(C_WARN, C_BG);
    tft.drawString("RAM only, no card", SCREEN_W - 6, rowY);
  } else {
    snprintf(v, sizeof(v), "card %u", (unsigned)L.wptOnCard);
    tft.setTextColor(C_OK, C_BG);
    tft.drawString(v, SCREEN_W - 6, rowY);
  }
  tft.setTextDatum(TL_DATUM);
  rowY += 20;
  tft.drawFastHLine(6, rowY - 5, SCREEN_W - 12, C_DIM);

  // The bottom block is always the newest mark in full plus the last press's
  // outcome, so the answer to "did that work?" is on the screen you are looking at.
  const int detailY = CONTENT_BOTTOM - 48;

  if (wpts.count() == 0) {
    rowY += 4;
    rowNote("no marks yet", C_ACCENT);
    rowTiny("wait for FIX, then press MARK", C_TEXT);
    rowTiny("distances appear here once you have a fix", C_DIM);
  } else {
    tft.setTextFont(1);
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString("#    time (UTC)   DIST      DIR", 6, rowY);
    rowY += 12;

    int rows = (detailY - 2 - rowY) / 17;
    if (rows < 1) rows = 1;
    for (int i = 0; i < wpts.count() && i < rows; ++i) {
      const Waypoint& m = wpts.at(i);
      clearRow(rowY - 2, 17);
      tft.setTextFont(2);
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(i == 0 ? C_OK : C_TEXT, C_BG);
      snprintf(v, sizeof(v), "%u", (unsigned)m.id);
      tft.drawString(v, 6, rowY);
      const char* t = m.utc;
      if (strlen(t) >= 19) t += 11;          // 2026-10-26T09:45:12Z -> 09:45:12Z
      snprintf(v, sizeof(v), "%.9s", t);
      tft.drawString(v, 36, rowY);

      tft.setTextDatum(TR_DATUM);
      if (!haveFix) {
        snprintf(v, sizeof(v), "--");
      } else {
        const double d = wptDistanceM(s.lat, s.lon, m.lat, m.lon);
        if (d < 1000.0) snprintf(v, sizeof(v), "%.0fm", d);
        else snprintf(v, sizeof(v), "%.2fkm", d / 1000.0);
      }
      tft.drawString(v, 190, rowY);

      if (!haveFix) snprintf(v, sizeof(v), "--");
      else wptCompass(v, sizeof(v), wptBearingDeg(s.lat, s.lon, m.lat, m.lon));
      tft.setTextColor(i == 0 ? C_OK : C_DIM, C_BG);
      tft.drawString(v, SCREEN_W - 6, rowY);

      rowY += 17;
    }
    if (rows < wpts.count()) {
      tft.setTextFont(1);
      tft.setTextColor(C_DIM, C_BG);
      snprintf(v, sizeof(v), "showing the newest %d of %d", rows, wpts.count());
      tft.drawString(v, 6, rowY);
    }
  }

  clearRow(detailY - 3, CONTENT_BOTTOM - (detailY - 3));
  tft.setTextFont(1);
  tft.setTextDatum(TL_DATUM);
  if (wpts.count() > 0) {
    const Waypoint& n = wpts.at(0);
    char alt[12], hd[12];
    tft.setTextColor(C_ACCENT, C_BG);
    snprintf(v, sizeof(v), "newest #%u  %s", (unsigned)n.id, n.utc);
    tft.drawString(v, 6, detailY);
    if (n.altValid) snprintf(alt, sizeof(alt), "%.1fm", n.altM);
    else snprintf(alt, sizeof(alt), "-");
    if (n.hdopValid) snprintf(hd, sizeof(hd), "%.2f", n.hdop);
    else snprintf(hd, sizeof(hd), "-");
    snprintf(v, sizeof(v), "%.6f %.6f  alt %s  %usat hdop %s",
             n.lat, n.lon, alt, (unsigned)n.sats, hd);
    tft.setTextColor(C_TEXT, C_BG);
    tft.drawString(v, 6, detailY + 12);
    tft.setTextColor(C_DIM, C_BG);
    snprintf(v, sizeof(v), "alt is the weak axis (m, not cm) - RAM window %d of %lu",
             wpts.count(), (unsigned long)wpts.total());
    tft.drawString(v, 6, detailY + 24);
  } else {
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString("altitude is the weak axis: marks are horizontal points", 6, detailY + 12);
  }
  tft.setTextColor(markResultLevel == 2 ? C_BAD : (markResultLevel == 1 ? C_WARN : C_OK), C_BG);
  tft.drawString(markResult[0] ? markResult : "press MARK to save this position",
                 6, detailY + 36);

  rowBottom = CONTENT_BOTTOM;
}

// Tiny Print sink for the directory listing: buffers what listFiles() writes and
// paints it as rows, so the SD code needs no knowledge of the display.
// force = a tab switch or the first paint: clear the whole content area, because
// the outgoing view's pixels must go. Otherwise each view clears only the bands it
// draws, and whatever is left below the last painted row is cleared here in one
// shot - which is what keeps a periodic refresh from strobing the panel.
static void drawContent(bool force) {
  static int lastBottom = CONTENT_BOTTOM;
  if (force) { clearContent(); lastBottom = CONTENT_BOTTOM; }
  switch (activeTab) {
    case 0: drawOverview(); rowBottom = rowY; break;
    case 1: drawSats(); break;      // sets rowBottom
    case 2: drawSky(); break;       // sets rowBottom
    case 3: drawRaw(); break;       // sets rowBottom
    case 4: drawLog(); break;       // sets rowBottom
    default: drawMarks(); break;    // sets rowBottom
  }
  if (rowBottom < lastBottom) clearRow(rowBottom, lastBottom - rowBottom);
  lastBottom = rowBottom;

  // The mark banner is painted here, after the view, so a periodic refresh cannot
  // rub it out mid-message. Short lived: loop() forces one repaint when it expires.
  markBannerLive = false;
  if (markResult[0] && (int32_t)(markBannerUntil - millis()) > 0) {
    markBannerLive = true;
    const int y = CONTENT_BOTTOM - 22;
    const uint16_t bg = (markResultLevel == 2) ? C_BAD : (markResultLevel == 1 ? C_WARN : C_TAB_ON);
    const uint16_t fg = (markResultLevel == 1) ? C_BG : TFT_WHITE;
    tft.fillRect(0, y, SCREEN_W, 20, bg);
    tft.setTextFont(2);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(fg, bg);
    tft.drawString(markResult, SCREEN_W / 2, y + 10);
    tft.setTextDatum(TL_DATUM);
  }
}

// -------------------------------------------------------------------- touch
static void readTouch() {
  if (!(ts.tirqTouched() && ts.touched())) return;
  TS_Point p = ts.getPoint();
  // first transaction after idle can be garbage
  if (p.x == 0 && p.y == 0) return;

  uint32_t now = millis();
  if (now - lastTouchMs < 250) return;   // one event per physical press
  lastTouchMs = now;
  tapCount++;

  lastRawX = p.x;
  lastRawY = p.y;
  int x = map(p.x, RAW_X_MIN, RAW_X_MAX, 0, SCREEN_W - 1);
  int y = map(p.y, RAW_Y_MIN, RAW_Y_MAX, 0, SCREEN_H - 1);
  x = constrain(x, 0, SCREEN_W - 1);
  y = constrain(y, 0, SCREEN_H - 1);
  lastMappedX = x;
  lastMappedY = y;

  if (Serial) Serial.printf("[touch] raw %d,%d -> screen %d,%d\n", p.x, p.y, x, y);

  if (y < TAB_H) {
    int tab = x / TAB_W;
    if (tab >= 0 && tab < TAB_COUNT && tab != activeTab) {
      activeTab = tab;
      drawTabBar();
      drawContent(true);   // tab switch: the old tab's pixels must go
      drawFooter();        // the Raw filter chips come and go with the tab
    }
  } else if (y >= CONTENT_BOTTOM) {
    if (x >= PAUSE_X0 && x <= PAUSE_X1) {
      paused = !paused;
      if (!paused) needFullRedraw = true;   // come straight back to live data
      drawFooter();
    } else if (x >= MARK_X0 && x <= MARK_X1) {
      doMark();
    } else if (activeTab == 3 && x >= CHIP_X0) {
      int i = (x - CHIP_X0) / (CHIP_W + CHIP_GAP);
      if (i >= 0 && i < CHIP_N && i != rawFilter) {
        rawFilter = i;
        drawFooter();
        drawContent(true);  // a filter change is a deliberate repaint, not a tick
      }
    }
  }
}

// ---------------------------------------------------------------- lifecycle
#ifdef GNSS_PROBE_PINS
// Bring-up diagnostic: which GPIO is the receiver's TX actually on?
// Sweeps every candidate pin, attaching UART1's RX to each in turn and
// counting bytes. A powered receiver streaming NMEA produces bytes at ANY
// baud setting, so this identifies the pin without knowing the baud rate.
// Excludes every pin owned by the display (12/13/14/15/2/21), touch
// (25/32/39/33/36) and microSD (5/18/19/23).
//
// Ask the pads what is actually driving them. A UART pin that is idle reads
// HIGH, so "reads high" alone proves nothing; toggling the internal pull (~45k)
// is what separates a real driver from a floating pin:
//   down=1           -> an external push-pull source holds the pad HIGH
//   down=0 and up=1   -> nothing is driving it (floating, free to use)
// GPIO1 is measured twice: once while UART0 drives it as the log output, once
// after Serial.end() releases the pad - the only state in which it could ever
// be used as an input, and the state HaleHound's rx:1 config implies.
static void padLevels() {
  const int pins[] = {3, 1, 22, 27};
  const int n = 4;
  int free_[4], down[4], up[4];

  for (int i = 0; i < n; ++i) {
    pinMode(pins[i], INPUT);
    delay(5);
    free_[i] = digitalRead(pins[i]);
    pinMode(pins[i], INPUT_PULLDOWN);
    delay(5);
    down[i] = digitalRead(pins[i]);
    pinMode(pins[i], INPUT_PULLUP);
    delay(5);
    up[i] = digitalRead(pins[i]);
    pinMode(pins[i], INPUT);
  }

  // GPIO1 with UART0 no longer holding it up.
  Serial.end();
  delay(40);
  pinMode(1, INPUT_PULLDOWN);
  delay(10);
  int g1_released = digitalRead(1);
  pinMode(1, INPUT);
  Serial.begin(PROBE_LOG_BAUD, SERIAL_8N1, -1, GPS_CONSOLE_TX);
  delay(80);

  tft.fillScreen(C_BG);
  tft.setTextFont(2);
  tft.setTextColor(C_TEXT, C_BG);
  tft.drawString("pad levels", 6, 6);
  tft.setTextFont(1);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("down=1 -> an external driver holds it HIGH", 6, 26);
  tft.drawString("down=0 + up=1 -> nothing drives it (free)", 6, 38);
  tft.setTextFont(2);

  for (int i = 0; i < n; ++i) {
    char line[48];
    snprintf(line, sizeof(line), "gpio%-2d free=%d down=%d up=%d",
             pins[i], free_[i], down[i], up[i]);
    tft.setTextColor(down[i] ? C_WARN : C_OK, C_BG);
    tft.drawString(line, 6, 60 + i * 18);
    Serial.printf("[pad] gpio%-2d free=%d down=%d up=%d\n",
                  pins[i], free_[i], down[i], up[i]);
  }

  char line[48];
  snprintf(line, sizeof(line), "gpio1 released, down=%d", g1_released);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.drawString(line, 6, 60 + n * 18 + 10);
  Serial.printf("[pad] gpio1 with UART0 released: down=%d\n", g1_released);
  // One self-contained line: the first printf after Serial.begin() gets eaten
  // by the re-init, and a missing row is exactly how you misread a diagnostic.
  Serial.printf("[pad] SUM gpio3 f%d d%d u%d | gpio1 f%d d%d u%d | "
                "gpio1-released d%d | gpio22 f%d d%d u%d | gpio27 f%d d%d u%d\n",
                free_[0], down[0], up[0], free_[1], down[1], up[1],
                g1_released, free_[2], down[2], up[2], free_[3], down[3], up[3]);
  Serial.println("[pad] ---- next: byte sweep ----");
  delay(6000);
}

// GPIO1 IS probed here, and it costs us the USB log: attaching UART1's RX to
// GPIO1 reclaims the pad UART0 uses to drive the CH340, so everything printed
// after that point arrives as a flood of 0x00 bytes. That is exactly why every
// result is also drawn on the LCD - the screen is the readout, the log is a
// bonus. GPIO35 (on-board LDR) and GPIO34 are no longer listed: 35 reads noise
// on any board and only muddies the table.
static void probeRxPins() {
  const int pins[] = {3, 1, 22, 27, 34, 16, 17};
  const int n = sizeof(pins) / sizeof(pins[0]);
  int bestPin = -1;
  uint32_t bestBytes = 0;
  static uint32_t counts[8];

  Serial.println("[probe] sweeping candidate RX pins, 2500ms each");
  tft.fillScreen(C_BG);
  tft.setTextFont(2);
  tft.setTextColor(C_TEXT, C_BG);
  tft.drawString("probing RX pins...", 6, 8);
  tft.setTextFont(1);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("2.5s per pin at 9600. A streaming receiver", 6, 26);
  tft.drawString("yields bytes at ANY baud, so baud is not the issue.", 6, 38);
  tft.drawString("gpio3 = header RX, gpio1 = header TX", 6, 50);
  tft.setTextFont(2);

  for (int i = 0; i < n; ++i) {
    Serial1.end();
    delay(20);
    Serial1.begin(9600, SERIAL_8N1, pins[i], -1);
    delay(30);
    uint32_t count = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 2500) {   // >=2 NMEA cycles even at 1Hz update
      if (Serial1.available()) { Serial1.read(); count++; }
      delay(2);
    }
    counts[i] = count;
    Serial.printf("[probe] gpio%-2d bytes=%lu\n", pins[i], (unsigned long)count);
    char line[40];
    snprintf(line, sizeof(line), "gpio%-2d : %lu bytes", pins[i], (unsigned long)count);
    tft.setTextColor(count ? C_OK : C_DIM, C_BG);
    tft.drawString(line, 6, 70 + i * 18);
    if (count > bestBytes) { bestBytes = count; bestPin = pins[i]; }
  }
  Serial1.end();

  // Hand the USB log back. Probing GPIO1 took UART0's TX pad away from the
  // CH340, so everything printed since then arrived as 0x00 bytes; re-pointing
  // UART0's TX at GPIO1 restores it and lets the whole table be re-printed over
  // serial as well as staying on the screen.
  Serial.end();
  delay(30);
  Serial.begin(115200, SERIAL_8N1, -1, 1);
  delay(60);
  Serial.println("[probe] --- results (log restored) ---");
  for (int i = 0; i < n; ++i) {
    Serial.printf("[probe] gpio%-2d bytes=%lu\n", pins[i], (unsigned long)counts[i]);
  }

  Serial.printf("[probe] BEST gpio%d with %lu bytes\n", bestPin, (unsigned long)bestBytes);
  if (bestPin < 0) {
    Serial.println("[probe] NO pin produced bytes -> receiver unpowered or unwired");
  } else {
    Serial.printf("[probe] build with -DGPS_RX_PIN=%d\n", bestPin);
  }
  tft.setTextColor(bestBytes ? C_OK : C_BAD, C_BG);
  char sum[48];
  snprintf(sum, sizeof(sum), bestBytes ? "best: gpio%d (%lu bytes)" : "no pin active",
           bestPin, (unsigned long)bestBytes);
  tft.drawString(sum, 6, 70 + n * 18 + 6);
  delay(4000);
}
#endif

#ifdef PANEL_PROBE
// Display bring-up diagnostic for a blank panel. Prints everything measurable
// over USB so the panel can be identified without eyes on the screen, and walks
// the panel through labelled visual steps so the user can report which one
// produced light.
static void panelProbe() {
  Serial.println("[panel] ===== panel probe start =====");

#ifdef TFT_BL
  // 1. Backlight, driven explicitly rather than trusting driver init.
  Serial.println("[panel] BL LOW  -> panel should go dark");
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
  delay(1500);
  Serial.println("[panel] BL HIGH -> panel should light up");
  digitalWrite(TFT_BL, HIGH);
  delay(1500);
#endif

  // 2. Controller identity over SPI (needs TFT_MISO / GPIO12 wired, which the
  //    CYD has). ILI9341 answers 0xD3 with 00 93 41; ST7789 answers 0x04 with
  //    85 85 52 and does not implement 0xD3.
  uint8_t d3[4], id4[3];
  for (int i = 0; i < 4; ++i) d3[i] = tft.readcommand8(0xD3, i);
  Serial.printf("[panel] RDID4 0xD3 : %02X %02X %02X %02X\n", d3[0], d3[1], d3[2], d3[3]);
  for (int i = 0; i < 3; ++i) id4[i] = tft.readcommand8(0x04, i);
  Serial.printf("[panel] RDDID 0x04 : %02X %02X %02X\n", id4[0], id4[1], id4[2]);
  uint8_t madctl = tft.readcommand8(0x0B);
  Serial.printf("[panel] MADCTL 0x0B: %02X (BGR bit=%d)\n", madctl, (madctl >> 3) & 1);

  // 3. Solid colours, one at a time, each announced on serial.
  struct Step { const char* name; uint16_t col; };
  const Step steps[] = {
      {"STEP-RED", TFT_RED},   {"STEP-GREEN", TFT_GREEN}, {"STEP-BLUE", TFT_BLUE},
      {"STEP-WHITE", TFT_WHITE}, {"STEP-BLACK", TFT_BLACK},
  };
  for (const Step& s : steps) {
    Serial.printf("[panel] %s\n", s.name);
    tft.fillScreen(s.col);
    delay(2500);
  }

  // 4. Inversion, in case the panel is inverted relative to driver default
  //    (ESP32-DIV/HaleHound set TFT_INVERSION_ON for this board family).
  Serial.println("[panel] STEP-INVERT-ON  (red, inverted)");
  tft.invertDisplay(true);
  tft.fillScreen(TFT_RED);
  delay(2500);
  Serial.println("[panel] STEP-INVERT-OFF (red, normal)");
  tft.invertDisplay(false);
  tft.fillScreen(TFT_RED);
  delay(2000);

  // 5. Real UI render: text + tab bar proves fonts and geometry too.
  Serial.println("[panel] STEP-UI (tab bar + text)");
  drawTabBar();
  drawContent(true);
  delay(3000);
  Serial.println("[panel] ===== probe done, normal UI resumes =====");
}
#endif

#ifdef BL_SWEEP
// Draw a labelled full-white screen and log which pin/level is being driven.
static void blShow(const char* big, const char* small, uint32_t ms) {
  tft.fillScreen(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(6);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.drawString(big, SCREEN_W / 2, 140);
  tft.setTextFont(4);
  tft.drawString(small, SCREEN_W / 2, 200);
  tft.setTextFont(2);
  tft.drawString("if lit, read this out", SCREEN_W / 2, 250);
  tft.setTextDatum(TL_DATUM);
  Serial.printf("[bl] %s %s\n", big, small);
  delay(ms);
}

// Backlight hunt, in three passes so the answer is unambiguous:
//   A. every candidate HIGH at once  -> is the light source one of these at all?
//   B. one candidate HIGH, rest LOW  -> which pin, active-high
//   C. one candidate LOW, rest HIGH  -> which pin, active-low
// Whichever step lights the panel, the user reads its own label off the screen.
// The whole sequence loops, so a missed step comes round again.
//
// Excludes display SPI (12/13/14/15/2), touch VSPI (25/32/33/36/39), UART0
// (1/3), and the LDR (34/35, input-only).
static void blSweep() {
  const int pins[] = {21, 27, 4, 5, 16, 17, 18, 19, 22, 23};
  const int n = sizeof(pins) / sizeof(pins[0]);
  char big[24];

  Serial.println("[bl] ===== backlight hunt (looping) =====");
  while (true) {
    // Pass A
    for (int i = 0; i < n; ++i) {
      pinMode(pins[i], OUTPUT);
      digitalWrite(pins[i], HIGH);
    }
    blShow("ALL HIGH", "(21 27 4 5 16 17 18 19 22 23)", 6000);

    // Pass B: exactly one HIGH
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        pinMode(pins[j], OUTPUT);
        digitalWrite(pins[j], (j == i) ? HIGH : LOW);
      }
      snprintf(big, sizeof(big), "ONLY gpio %d", pins[i]);
      blShow(big, "HIGH", 4000);
    }

    // Pass C: exactly one LOW (catches an active-low backlight)
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        pinMode(pins[j], OUTPUT);
        digitalWrite(pins[j], (j == i) ? LOW : HIGH);
      }
      snprintf(big, sizeof(big), "gpio %d LOW", pins[i]);
      blShow(big, "all others HIGH", 4000);
    }

    for (int i = 0; i < n; ++i) pinMode(pins[i], INPUT);
    Serial.println("[bl] cycle complete, repeating");
    delay(500);
  }
}
#endif

#ifdef SD_PROBE
// ------------------------------------------------------------ SD+touch probe
// Two things can only be settled on the board, and both fail silently:
//   1. does the microSD card mount on THIS unit (slot populated, card FAT32);
//   2. does the touch panel still answer after the shared VSPI bus has been
//      re-routed to the card and back? The failure looks like a dead touchscreen
//      with a perfectly healthy log, so it must be measured, not assumed.
static int probeY = 6;

static void probeLine(const char* text, uint16_t col = C_TEXT) {
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(col, C_BG);
  tft.drawString(text, 4, probeY);
  probeY += 16;
  if (Serial) Serial.printf("[sdprobe] %s\n", text);
}

// The two control experiments that lived here (plain init on the global SPI
// object; matrix re-route onto a touch-owned host) have served their purpose and
// are removed on purpose: both call SPI.end(), which frees the shared VSPI host
// and stales the handle the app's own SPI object is holding, so leaving them in
// makes every later mount fail and reads as a card fault.
//
// What they established: the card cold-inits at 400 kHz, and re-pointing the bus
// works by re-routing SCK/MOSI/MISO through the GPIO matrix while the SPI host
// stays up.

// Which pin is the receiver's TX actually on? P1's RX pad (GPIO3) is held high by
// the CH340 so it can never work; GPIO1 works but costs the USB console; GPIO22 and
// GPIO27 are the console-friendly alternatives. Rather than trust a note from an
// earlier rewiring, listen on the two console-friendly candidates with UART2 and
// report bytes + '$' starts. Silence on both means the receiver is on GPIO1 (or
// wired to nothing), which the console build cannot hear without eating its own log.
static void gpsPinSniff() {
  // GPIO1 last: the console's own TX idles high there, so only a real '$' count
  // means NMEA; plain bytes with zero '$' are just our log being read back.
  static const int cand[] = {22, 27, 1};
  for (unsigned i = 0; i < sizeof(cand) / sizeof(cand[0]); i++) {
    int pin = cand[i];
    Serial2.begin(9600, SERIAL_8N1, pin, -1);
    delay(30);
    uint32_t bytes = 0, dollars = 0, t0 = millis();
    while (millis() - t0 < 3000) {
      while (Serial2.available()) {
        int c = Serial2.read();
        bytes++;
        if (c == '$') dollars++;
      }
      delay(4);
    }
    Serial2.end();
    Serial.printf("[sniff] gpio%d bytes=%lu '$'=%lu%s\n", pin, (unsigned long)bytes,
                  (unsigned long)dollars, dollars > 1 ? "  <-- NMEA HERE" : "");
  }
}

static void sdProbe() {
  gpsPinSniff();
  char b[80];
  tft.fillScreen(C_BG);
  probeY = 6;
  probeLine("SD + touch probe", C_ACCENT);
  probeLine("--- shared-bus: touch + SD on VSPI ---", C_ACCENT);

  sdLog.begin(&vspi, SD_SCLK, SD_MISO, SD_MOSI, SD_CS,
              TOUCH_SCLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  const LogStatus& L = sdLog.st();
  snprintf(b, sizeof(b), "mount: %s  type %u  %lu MB  %lu kHz",
           L.cardMounted ? "OK" : "FAIL", L.cardType, (unsigned long)L.cardSizeMB,
           (unsigned long)(L.mountFreq / 1000));
  probeLine(b, L.cardMounted ? C_OK : C_BAD);
  if (!L.cardMounted) {
    probeLine(L.lastError, C_BAD);
    probeLine("no card, or not FAT32, or not seated.", C_WARN);
    probeLine("retrying every 5 s - slide it in and watch.", C_ACCENT);
  } else {
    uint32_t now = millis();
    sdLog.openBootFile(now);
    sdLog.event("probe: first write");
    sdLog.flushNow(now);                 // force the RAM buffer onto the card
    snprintf(b, sizeof(b), "wrote %lu B in %lu line(s), err %u",
             (unsigned long)L.bytesWritten, (unsigned long)L.linesWritten,
             (unsigned)L.writeErrors);
    probeLine(b, L.writeErrors ? C_BAD : C_OK);
    probeLine("files in /gps:", C_ACCENT);
    LcdPrint lp;
    sdLog.listFiles(lp);
    lp.draw();
    probeY += 4;
    // the same listing to the console, because an empty panel listing is
    // ambiguous: no directory, no file, or no iteration?
    Serial.printf("[probe] dir /gps exists=%d\n", (int)SD.exists("/gps"));
    File dir = SD.open("/gps");
    int listed = 0;
    if (dir) {
      File f = dir.openNextFile();
      while (f) {
        Serial.printf("[probe]   %s  %lu B\n", f.name(), (unsigned long)f.size());
        f = dir.openNextFile();
        listed++;
      }
      dir.close();
    } else {
      Serial.println("[probe]   cannot open /gps as a directory");
    }
    char fp[64];
    snprintf(fp, sizeof(fp), "/gps/%s", sdLog.st().file);
    Serial.printf("[probe] listed %d, expecting %s, exists=%d\n", listed, fp,
                  (int)SD.exists(fp));
    File chk = SD.open(fp, FILE_READ);
    Serial.printf("[probe] reopen %s -> size %lu\n", fp,
                  chk ? (unsigned long)chk.size() : 0UL);
    if (chk) chk.close();
  }

  probeLine("--- live: 1 card line/s, taps counted ---", C_ACCENT);
  int yLive = probeY;
  uint32_t taps = 0, lastTap = 0, lines = 0;
  uint32_t nextWrite = millis() + 1000;
  while (true) {
    if (ts.tirqTouched() && ts.touched()) {
      TS_Point pt = ts.getPoint();
      if (pt.x || pt.y) { taps++; lastTap = millis(); }
    }
    uint32_t now = millis();
    if (now >= nextWrite) {
      nextWrite = now + 1000;
      char line[64];
      snprintf(line, sizeof(line), "# probe tick %lu", (unsigned long)(now / 1000));
      sdLog.event(line);
      sdLog.tick(now);                   // honours the 5 s mount retry, flushes at 1 s
      lines++;
    }
    // repaint the counters in place: a full clear would strobe the panel
    snprintf(b, sizeof(b), "taps %lu  last %lums ago   ", (unsigned long)taps,
             lastTap ? (unsigned long)(millis() - lastTap) : 0UL);
    tft.setTextFont(2);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(lastTap && millis() - lastTap < 3000 ? C_OK : C_DIM, C_BG);
    tft.drawString(b, 4, yLive);
    snprintf(b, sizeof(b), "card lines %lu  err %u   ", (unsigned long)lines,
             (unsigned)L.writeErrors);
    tft.setTextColor(L.writeErrors ? C_BAD : C_TEXT, C_BG);
    tft.drawString(b, 4, yLive + 16);
    tft.setTextColor(C_ACCENT, C_BG);
    tft.drawString("tap the tabs - they must still switch", 4, yLive + 32);
    delay(5);
  }
}
#endif

void setup() {
#if ENABLE_USB_SERIAL
  // PROBE_LOG_BAUD exists so a diagnostic build can log at the same rate its
  // UART listens at: one USB-serial port cannot read at 115200 and write at
  // 9600 simultaneously, and that mismatch silently blinds the capture.
#ifdef PROBE_LOG_BAUD
  Serial.begin(PROBE_LOG_BAUD, SERIAL_8N1, -1, GPS_CONSOLE_TX);
#else
  Serial.begin(115200, SERIAL_8N1, -1, GPS_CONSOLE_TX);
#endif
  delay(150);
  Serial.println();
  Serial.println("[boot] GNSS Explorer stage 3 (CYD 2432S028 + ATGM336H)");
  Serial.printf("[boot] display HSPI 12/13/14 cs15 dc2 bl21, touch VSPI 25/32/39 cs33 irq36\n");
  Serial.printf("[boot] gps passive rx=%d tx=%d baud=%d\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
  Serial.printf("[boot] receiver found on gpio%d\n", gpsActivePin);
#else
  // The receiver's TX is on GPIO1 (P1's TX pin), so the console must not drive
  // that pad. Serial.end() also releases the routing the ROM bootloader left on
  // UART0's TX - without that, a check that only skips Serial.begin() still
  // fails, because the pad is driven before setup() ever runs.
  Serial.end();
#endif

  tft.init();
  tft.setRotation(0);          // portrait 240x320, matches OneBoard convention
  tft.fillScreen(C_BG);
  tft.setTextColor(C_TEXT, C_BG);

  vspi.begin(TOUCH_SCLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  ts.begin(vspi);
  ts.setRotation(0);

#if ENABLE_SD_LOG
  // Card after the panel. Order is no longer load-bearing (switchToSd() does the
  // real end()/begin()), and this way a missing card cannot hold up the screen.
  sdLog.begin(&vspi, SD_SCLK, SD_MISO, SD_MOSI, SD_CS,
              TOUCH_SCLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
#endif

#if ENABLE_USB_SERIAL && ENABLE_SD_LOG
  // Self-test in the REAL app path, not in a probe build: one file written,
  // closed, reopened, read back, and removed. Each step switches the bus there
  // and back, so this proves repeated switches work — not just the first mount.
  if (sdLog.st().cardMounted) {
    const char* tmpPath = "/gps/_selftest.tmp";
    sdLog.switchToSdForTest();
    bool wrote = false, readback = false, removed = false;
    size_t written = 0;
    {
      File f = SD.open(tmpPath, FILE_WRITE);
      if (f) {
        written = f.print("# selftest\r\n");   // never hand-count a literal
        f.close();
        wrote = (written > 0);
      }
    }
    { File f = SD.open(tmpPath, FILE_READ);
      if (f) { readback = (f.size() == written); f.close(); } }
    removed = SD.remove(tmpPath);
    File dir = SD.open("/gps");
    int n = 0;
    if (dir) { File e = dir.openNextFile(); while (e) { n++; e = dir.openNextFile(); } dir.close(); }
    sdLog.switchToTouchForTest();
    Serial.printf("[sd] mounted %lu kHz, type %u, %lu MB | selftest write=%d(%u B) "
                  "readback=%d removed=%d files_in_gps=%d\n",
                  (unsigned long)(sdLog.st().mountFreq / 1000), sdLog.st().cardType,
                  (unsigned long)sdLog.st().cardSizeMB, (int)wrote, (unsigned)written,
                  (int)readback, (int)removed, n);
  }
#endif

#ifdef SD_PROBE
  sdProbe();     // diagnostic: never returns
#endif

  // Backlight. TFT_eSPI 2.5.43 only drives it from init() when BOTH TFT_BL and
  // TFT_BACKLIGHT_ON are defined (see TFT_eSPI.cpp init()); without the second
  // macro the panel receives correct pixels but stays dark. Drive it here too so
  // this never depends on library internals again.
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

#ifdef GNSS_PROBE_PINS
  // Diagnostic only: keep sweeping so the on-screen table stays up and readable.
  // (Probing GPIO1 breaks USB logging, so the LCD is the readout here.)
  while (true) {
    padLevels();
    probeRxPins();
    Serial.println("[probe] --- next sweep ---");
    delay(500);
  }
#endif

#ifdef PANEL_PROBE
  panelProbe();      // one-shot display diagnostic before normal operation
#endif

#ifdef BL_SWEEP
  blSweep();         // never returns: backlight pin hunt
#endif

  gnss.begin();

  tft.setTextFont(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("GNSS", SCREEN_W / 2, 140);
  tft.drawString("EXPLORER", SCREEN_W / 2, 180);
  tft.setTextFont(2);
#if ENABLE_SD_LOG
  tft.drawString("stage 5: log + marks", SCREEN_W / 2, 210);
#elif defined(SD_PROBE)
  tft.drawString("SD + touch probe", SCREEN_W / 2, 210);
#else
  tft.drawString("stage 3: pause + filters", SCREEN_W / 2, 210);
#endif
  tft.setTextDatum(TL_DATUM);
  delay(900);

  drawTabBar();
  drawContent(true);
  drawFooter();
  needFullRedraw = false;
}

void loop() {
  readTouch();
  gnss.poll();

#if ENABLE_SD_LOG
  // Every complete sentence reaches the logger exactly once. linesTotal only
  // grows, and the ring is newest-first, so the newly arrived sentences are the
  // first `fresh` entries - written oldest-first so the file keeps time order.
  static uint32_t fedLines = 0;
  {
    const GnssSnapshot& sg = gnss.snap();
    if (sg.linesTotal < fedLines) fedLines = 0;             // counter reset / re-begin
    uint32_t fresh = sg.linesTotal - fedLines;
    if (fresh > (uint32_t)GNSS_RING) fresh = GNSS_RING;     // older lines already scrolled out
    for (uint32_t i = fresh; i > 0; --i) sdLog.noteLine(gnss.line((int)(i - 1)).text);
    fedLines = sg.linesTotal;
    if (sg.dateValid && sg.timeValid)
      sdLog.setClock(sg.year, sg.month, sg.day, sg.hour, sg.minute, sg.second);
    sdLog.tick(millis());
  }
#endif

  // Refresh the visible tab about twice a second. Never redraw every
  // iteration — a 5ms redraw strobes badly on this panel.
  static uint32_t lastDraw = 0;
  static uint32_t lastSig = 0xFFFFFFFFu;
  uint32_t now = millis();
  const GnssSnapshot& sn = gnss.snap();
  // Anything worth repainting moves one of these. The throttle stays, but a frame
  // that would draw identical pixels is skipped: it costs SPI time and shimmers.
  uint32_t sig = (uint32_t)sn.state * 7919u + (uint32_t)sn.satsTracked * 613u +
                 (uint32_t)sn.satsUsed * 71u + (uint32_t)sn.bestSnr +
                 (uint32_t)(sn.linesTotal & 0xFFFFu) * 31u +
                 (uint32_t)sn.fixValid * 13u + (uint32_t)activeTab +
                 (uint32_t)wpts.total() * 997u + (uint32_t)markResultLevel * 4093u;

  // The banner is short lived: one forced repaint takes it back off the panel,
  // instead of leaving a stale "marked #7" on a view that does not paint there.
  if (markBannerLive && (int32_t)(now - markBannerUntil) >= 0) needFullRedraw = true;

  if (needFullRedraw || now - lastDraw > 500) {
    if (paused) {
      // The whole point of pause: hold the pixels still. The parser above keeps
      // running, so nothing is lost and resuming shows current data immediately.
      lastDraw = now;
    } else if (needFullRedraw || sig != lastSig) {
      lastSig = sig;
      lastDraw = now;
      bool force = needFullRedraw;
      needFullRedraw = false;
      drawContent(force);
    } else {
      lastDraw = now;
    }
  }

#if ENABLE_USB_SERIAL
  // periodic link summary so the stream can be verified off-board
  static uint32_t lastReport = 0;
  static uint32_t lastLines = 0;
  if (now - lastReport > 5000) {
    const GnssSnapshot& s = gnss.snap();
    lastReport = now;
    Serial.printf("[gnss] state=%s bytes=%lu lines=%lu (+%lu) talkers=[%s] sats=%u/%u fix=%d\n",
                  s.stateText, (unsigned long)s.bytesTotal, (unsigned long)s.linesTotal,
                  (unsigned long)(s.linesTotal - lastLines),
                  s.talkerCount ? s.talkers : "-", s.satsUsed, s.satsInView,
                  (int)s.fixValid);
    lastLines = s.linesTotal;
  }
#endif

  delay(5);
}
