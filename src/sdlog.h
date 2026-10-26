// sdlog.h — NMEA logging to the CYD's microSD card.
//
// Why this is not just "SD.begin()": the microSD slot and the touch controller
// are on the SAME VSPI peripheral with DIFFERENT pin sets.
//
//     touch : SCK 25, MISO 39, MOSI 32, CS 33   (+ IRQ 36)
//     microSD: SCK 18, MISO 19, MOSI 23, CS 5
//
// The ESP32 routes the SPI peripheral to whichever GPIOs were named in the last
// begin(), so only one device can be addressed at a time. Every SD access
// therefore re-points the shared bus at the card, then re-points it at the touch
// panel before returning — otherwise the next touch read silently reads the SD
// pins instead and the tabs stop responding. Writes are batched (2 KB or 1 s)
// precisely because each flush costs two pin re-routes.
//
// The card holds the RAW NMEA stream, complete sentences only, exactly as the
// receiver sent them — the ground truth for anything done with the log later
// (tools/topo_map.py parses these files directly). App events are written as
// lines starting with '#' so a NMEA parser skips them for free.
#pragma once

#include <Arduino.h>
#include <FS.h>

// GPIO-matrix signal routing for the VSPI bus, exported for bench tests.
void sdRouteSpi(int sck, int miso, int mosi);
void sdUnrouteSpi(int sck, int miso, int mosi);
#include <SPI.h>

#define SD_LOG_BUF 4096
#define SD_LOG_NAME_MAX 40

struct LogStatus {
  bool busReady = false;
  bool mountTried = false;
  bool cardMounted = false;
  bool logging = false;
  uint8_t cardType = 0;
  uint32_t cardSizeMB = 0;
  char file[SD_LOG_NAME_MAX] = {};      // basename only, "" until a file exists
  char lastError[48] = {};
  uint32_t linesBuffered = 0;           // in RAM, not yet on the card
  uint32_t linesWritten = 0;
  uint32_t bytesWritten = 0;
  uint32_t flushes = 0;
  uint32_t writeErrors = 0;
  uint32_t droppedLines = 0;            // buffer was full, line thrown away
  uint32_t skippedShort = 0;            // not a complete sentence
  uint32_t busSwitches = 0;             // cost meter: 2 per flush
  uint32_t lastFlushMs = 0;
  uint32_t lastFsyncMs = 0;
  uint32_t pendingBytes = 0;
  uint32_t mountFreq = 0;               // SPI clock the card finally answered at
  uint32_t nextMountTryMs = 0;
  uint32_t lastStatusMs = 0;
  bool clockValid = false;              // a UTC date+time has been seen
  char clockText[24] = {};              // when the file name was decided
  // Waypoints (/gps/waypoints.csv). A separate file from the NMEA log on purpose:
  // the log stays raw sentences, the marks are our own records, and a parser that
  // reads one is never confused by the other.
  uint32_t wptLines = 0;                // marks written to the card this session
  uint32_t wptErrors = 0;
  uint16_t wptOnCard = 0;               // marks in the file (counted, then incremented)
  bool wptCounted = false;
};

class SdLogger {
 public:
  // Both pin sets, because the bus is shared and must be handed back.
  void begin(SPIClass* bus,
             int sdSck, int sdMiso, int sdMosi, int sdCs,
             int tpSck, int tpMiso, int tpMosi, int tpCs);

  // One complete NMEA sentence, no CR/LF. Short/garbage input is counted, not written.
  void noteLine(const char* text);

  // App event, written as "# <what>" so a NMEA parser ignores it.
  void event(const char* what);

  // Call every loop. Creates the file once a UTC clock exists, then flushes on
  // size or time. Cheap when there is nothing to do.
  void tick(uint32_t nowMs);

  // UTC clock from a valid fix; decides the log file name (YYYYMMDD_HHMMSS).
  void setClock(uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour, uint8_t minute, uint8_t second);

  // Probe/testing: open a boot-relative file immediately instead of waiting for
  // a UTC clock out of a fix, so a card can be proven before the antenna is up.
  bool openBootFile(uint32_t nowMs);

  void switchToSdForTest() { switchToSd(); }
  void switchToTouchForTest() { switchToTouch(); }
  bool flushNow(uint32_t nowMs);        // force the RAM buffer onto the card
  bool listFiles(Print& out);           // directory listing to any Print (Serial or LCD)
  void busToTouch();                    // hand the shared bus back to the panel

  // One waypoint CSV row (no trailing newline). Called from the UI on a button
  // press, never from the periodic path: it opens, writes and closes the file,
  // which is two bus re-routes, and that is only affordable at human rates.
  bool appendWaypoint(const char* row, uint32_t nowMs);

  // Data lines already in waypoints.csv, so mark ids keep counting across reboots
  // instead of starting a second #1. Reads the file: call it once, on the first
  // mark, not per frame.
  uint16_t countWaypoints();

  const LogStatus& st() const { return st_; }

 private:
  bool switchToSd();
  bool switchToTouch();
  bool tryMount();
  bool openFile(uint32_t nowMs);
  bool openWithName(const char* name, uint32_t nowMs);
  bool flush(uint32_t nowMs, bool force);

  SPIClass* bus_ = nullptr;
  File file_;

  int sd_[4] = {18, 19, 23, 5};
  int tp_[4] = {25, 39, 32, 33};

  char buf_[SD_LOG_BUF] = {};
  uint32_t bufLen_ = 0;
  bool busOnSd_ = false;

  LogStatus st_;
};

extern SdLogger sdLog;
