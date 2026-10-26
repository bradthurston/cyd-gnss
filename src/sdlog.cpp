// sdlog.cpp — see sdlog.h for why the bus is switched rather than shared.
#include "sdlog.h"

#include <SD.h>

#include "gnss.h"      // gpsActivePin, so the log names the pin it heard
#include "waypoints.h"  // WPT_CSV_FILE / WPT_CSV_HEADER: the marks' own file

#include "esp32-hal-matrix.h"      // pinMatrix{Out,In}Attach/Detach
#include "soc/gpio_sig_map.h"      // VSPICLK_OUT_IDX / VSPIQ_IN_IDX / VSPID_OUT_IDX

SdLogger sdLog;

// ---------------------------------------------------------------- bus handing
//
// The touch panel and the microSD card are both on VSPI (SPI3) but on DIFFERENT
// pins, so one bus has to be re-pointed between them. Two ways to do that, and
// only one of them works here:
//
//   bus_->end(); bus_->begin(otherPins);      // NO
//
// end() calls spiStopBus() -> removeApbChangeCallback() + spiInitBus(), which
// tears the host down underneath the SD driver. The driver keeps its own handle
// (ardu_sdcard_t::spi) and every transfer after that returns
// "sdSelectCard(): Select Failed" — the card answers for the first few accesses
// and then never again. Verified on the bench: mount OK, first write OK, and the
// directory read straight afterwards dead.
//
// Instead, leave the SPI host running and re-route only the three signals through
// the GPIO matrix, exactly as arduino's own spiAttachSCK() does with
// pinMatrixOutAttach(pin, SPI_CLK_IDX(VSPI), ...). SPI_CLK_IDX(3) resolves to
// VSPICLK_OUT_IDX on ESP32. The card's CS is a plain GPIO driven by
// digitalWrite() in sd_diskio, so it stays high and deselected while the panel is
// being read — the card never sees the touch traffic.
void sdRouteSpi(int sck, int miso, int mosi) {
  pinMode(sck, OUTPUT);
  pinMatrixOutAttach(sck, VSPICLK_OUT_IDX, false, false);
  pinMode(mosi, OUTPUT);
  pinMatrixOutAttach(mosi, VSPID_OUT_IDX, false, false);
  pinMode(miso, INPUT);
  pinMatrixInAttach(miso, VSPIQ_IN_IDX, false);
}

// Hand the pins back as plain inputs so the idle device is not clocked.
void sdUnrouteSpi(int sck, int miso, int mosi) {
  pinMatrixOutDetach(sck, false, false);
  pinMatrixOutDetach(mosi, false, false);
  pinMatrixInDetach(VSPIQ_IN_IDX, false, false);
  pinMode(sck, INPUT);
  pinMode(mosi, INPUT);
  pinMode(miso, INPUT);
}

bool SdLogger::switchToSd() {
  if (!bus_) return false;
  if (busOnSd_) return true;
  sdUnrouteSpi(tp_[0], tp_[1], tp_[2]);
  sdRouteSpi(sd_[0], sd_[1], sd_[2]);
  busOnSd_ = true;
  st_.busSwitches++;
  return true;
}

bool SdLogger::switchToTouch() {
  if (!bus_) return false;
  if (!busOnSd_) return true;
  sdUnrouteSpi(sd_[0], sd_[1], sd_[2]);
  sdRouteSpi(tp_[0], tp_[1], tp_[2]);
  busOnSd_ = false;
  st_.busSwitches++;
  return true;
}// --------------------------------------------------------------------- begin
void SdLogger::begin(SPIClass* bus,
                     int sdSck, int sdMiso, int sdMosi, int sdCs,
                     int tpSck, int tpMiso, int tpMosi, int tpCs) {
  bus_ = bus;
  sd_[0] = sdSck; sd_[1] = sdMiso; sd_[2] = sdMosi; sd_[3] = sdCs;
  tp_[0] = tpSck; tp_[1] = tpMiso; tp_[2] = tpMosi; tp_[3] = tpCs;
  st_.busReady = true;
  st_.mountTried = true;
  tryMount();

  if (!st_.cardMounted) {
    // A card inserted later still gets picked up, on a timer, so nobody has to
    // reboot a board that is already up a pole or in a field.
    st_.nextMountTryMs = millis() + 5000;
  }
}

// The card is the slow device and the panel must stay responsive between
// flushes, so mount conservatively and hand the bus straight back.
//
// Clock ladder, not one attempt: "sdSelectCard(): Select Failed" /
// "f_mount failed: (3) The physical drive cannot work" is what this board prints
// for BOTH an absent card and a card that simply will not answer at 20 MHz.
// Stepping down finds the working clock instead of declaring the slot dead.
// ORDER MATTERS: lowest first. Measured on this board's 16 GB SDHC card — it
// answers "sdSelectCard(): Select Failed" (f_mount error 3, FR_NOT_READY) at
// every clock from 1 MHz up on a COLD init, and mounts every time at 400 kHz.
// Once a 400 kHz init has brought the card up it will talk at 20 MHz, so the
// slow clock is only the doorway, not the operating speed. The first version of
// this ladder stepped DOWN from 20 MHz and stopped at 1 MHz, never trying the
// one speed that works.
static const uint32_t MOUNT_FREQ[] = {400000, 1000000, 4000000, 20000000};
static const int MOUNT_FREQ_N = sizeof(MOUNT_FREQ) / sizeof(MOUNT_FREQ[0]);

bool SdLogger::tryMount() {
  if (!bus_) return false;
  for (int i = 0; i < MOUNT_FREQ_N; ++i) {
    switchToSd();
    delay(2);
    if (SD.begin(sd_[3], *bus_, MOUNT_FREQ[i])) {
      st_.cardMounted = true;
      st_.mountFreq = MOUNT_FREQ[i];
      st_.cardType = SD.cardType();
      st_.cardSizeMB = (uint32_t)(SD.cardSize() / (1024ULL * 1024ULL));
      if (!SD.exists("/gps")) SD.mkdir("/gps");
      switchToTouch();
      snprintf(st_.lastError, sizeof(st_.lastError), "mounted at %lu kHz",
               (unsigned long)(MOUNT_FREQ[i] / 1000));
      return true;
    }
    SD.end();
  }
  st_.cardMounted = false;
  snprintf(st_.lastError, sizeof(st_.lastError),
           "no card / not FAT32 (tried 400k/1M/4M/20M)");
  switchToTouch();
  return false;
}

// ------------------------------------------------------------------ filename
void SdLogger::setClock(uint16_t year, uint8_t month, uint8_t day,
                        uint8_t hour, uint8_t minute, uint8_t second) {
  if (st_.clockValid || year < 2020) return;
  st_.clockValid = true;
  snprintf(st_.clockText, sizeof(st_.clockText), "%04u-%02u-%02u %02u:%02u:%02uZ",
           year, month, day, hour, minute, second);
  snprintf(st_.file, sizeof(st_.file), "%04u%02u%02u_%02u%02u%02u.nmea",
           year, month, day, hour, minute, second);
}

bool SdLogger::openWithName(const char* name, uint32_t nowMs) {
  if (st_.logging || !st_.cardMounted) return st_.logging;
  snprintf(st_.file, sizeof(st_.file), "%s", name);
  char path[64];
  snprintf(path, sizeof(path), "/gps/%s", st_.file);

  switchToSd();
  file_ = SD.open(path, FILE_APPEND);
  bool ok = (bool)file_;
  if (!ok) {
    snprintf(st_.lastError, sizeof(st_.lastError), "open failed: %s", path);
    st_.writeErrors++;
  } else {
    st_.logging = true;
    st_.lastFlushMs = nowMs;
    st_.lastFsyncMs = nowMs;
  }
  switchToTouch();
  if (ok) event(st_.clockValid ? "log opened" : "log opened (boot time)");
  return ok;
}

bool SdLogger::openBootFile(uint32_t nowMs) {
  if (st_.clockValid) return openWithName(st_.file, nowMs);
  char name[SD_LOG_NAME_MAX];
  snprintf(name, sizeof(name), "boot-%lums.nmea", (unsigned long)nowMs);
  snprintf(st_.clockText, sizeof(st_.clockText), "no fix: boot-relative name");
  return openWithName(name, nowMs);
}

bool SdLogger::openFile(uint32_t nowMs) {
  if (st_.logging || !st_.cardMounted) return st_.logging;
  if (!st_.clockValid) {
    // No fix yet, so no UTC clock. The receiver usually delivers GGA/RMC within
    // a minute; after five minutes of silence a boot-relative name beats losing
    // the data, because a card full of unnamed logs is useless.
    if (nowMs < 300000UL) return false;
    return openBootFile(nowMs);
  }
  return openWithName(st_.file, nowMs);
}

// ------------------------------------------------------------------- writing
void SdLogger::noteLine(const char* text) {
  if (!text || text[0] != '$' || strlen(text) < 8) {
    st_.skippedShort++;
    return;
  }
  size_t n = strlen(text);
  if (bufLen_ + n + 2 >= SD_LOG_BUF) {
    // Full-length sentences are ~3x what this path used to see (an 85 B GSV vs a
    // 19 B fragment), so the buffer can genuinely fill between ticks. Put the
    // pending bytes on the card rather than dropping the sentence that tripped it.
    flush(millis(), true);
  }
  if (bufLen_ + n + 2 >= SD_LOG_BUF) {
    st_.droppedLines++;
    return;
  }
  memcpy(buf_ + bufLen_, text, n);
  bufLen_ += n;
  buf_[bufLen_++] = '\r';
  buf_[bufLen_++] = '\n';
  st_.linesBuffered++;
  st_.pendingBytes = bufLen_;
}

void SdLogger::event(const char* what) {
  if (!what || bufLen_ + strlen(what) + 8 >= SD_LOG_BUF) return;
  char line[128];
  snprintf(line, sizeof(line), "# %s", what);
  size_t n = strlen(line);
  memcpy(buf_ + bufLen_, line, n);
  bufLen_ += n;
  buf_[bufLen_++] = '\r';
  buf_[bufLen_++] = '\n';
  st_.pendingBytes = bufLen_;
}

bool SdLogger::flush(uint32_t nowMs, bool force) {
  if (!st_.logging || bufLen_ == 0) return true;
  if (!force && bufLen_ < 2048 && nowMs - st_.lastFlushMs < 1000) return true;

  switchToSd();
  size_t n = file_.write((const uint8_t*)buf_, bufLen_);
  bool ok = (n == bufLen_);
  if (ok) {
    st_.bytesWritten += bufLen_;
    st_.linesWritten += st_.linesBuffered;
    st_.linesBuffered = 0;
    st_.flushes++;
    // fsync every 5 s: a card that is not flushed loses the tail on power loss,
    // but flushing every second wears the card and costs bus time.
    if (force || nowMs - st_.lastFsyncMs > 5000) {
      file_.flush();
      st_.lastFsyncMs = nowMs;
    }
  } else {
    st_.writeErrors++;
    snprintf(st_.lastError, sizeof(st_.lastError), "short write %u of %lu",
             (unsigned)n, (unsigned long)bufLen_);
  }
  bufLen_ = 0;
  st_.pendingBytes = 0;
  st_.lastFlushMs = nowMs;
  switchToTouch();
  return ok;
}

bool SdLogger::flushNow(uint32_t nowMs) { return flush(nowMs, true); }

// A status line every 30 s, written INTO the log. The field build has no USB
// console (the receiver owns GPIO1), so the only way to prove from a desk that
// sentences are being received and written is to have the card say so itself.
// '#' lines are ignored by every NMEA parser, so this costs nothing downstream.
static void statusLine(char* out, size_t n, const LogStatus& L) {
  snprintf(out, n, "status lines=%lu bytes=%lu flushes=%lu drop=%lu err=%lu pending=%lu bus=%lu gpsrx=%d",
           (unsigned long)L.linesWritten, (unsigned long)L.bytesWritten,
           (unsigned long)L.flushes, (unsigned long)L.droppedLines,
           (unsigned long)L.writeErrors, (unsigned long)L.pendingBytes,
           (unsigned long)L.busSwitches, gpsActivePin);
}

void SdLogger::tick(uint32_t nowMs) {
  if (!st_.cardMounted) {
    // Card slid in after boot? Pick it up without a power cycle.
    if (nowMs >= st_.nextMountTryMs) {
      st_.nextMountTryMs = nowMs + 5000;
      tryMount();
    }
    if (!st_.cardMounted) return;
  }
  if (!st_.logging) openFile(nowMs);
  if (!st_.logging) return;
  if (nowMs - st_.lastStatusMs > 30000) {
    st_.lastStatusMs = nowMs;
    char line[160];
    statusLine(line, sizeof(line), st_);
    event(line);
  }
  flush(nowMs, false);
}

// -------------------------------------------------------------------- listing
bool SdLogger::listFiles(Print& out) {
  if (!st_.cardMounted) return false;
  switchToSd();
  File dir = SD.open("/gps");
  bool ok = (bool)dir;
  if (ok) {
    File f = dir.openNextFile();
    while (f) {
      out.printf("  %-22s %8lu\n", f.name(), (unsigned long)f.size());
      f = dir.openNextFile();
    }
    dir.close();
  }
  switchToTouch();
  return ok;
}

// ------------------------------------------------------------------ waypoints
// Marks live in /gps/waypoints.csv, NOT in the NMEA log: that file is raw
// sentences only, and mixing our own rows into it would make every downstream
// parser guess which lines are ground truth. The '#' app-event line already
// written when a mark is taken is enough of a breadcrumb in the log itself.
static void wptPath(char* out, size_t n) {
  snprintf(out, n, "/gps/%s", WPT_CSV_FILE);
}

uint16_t SdLogger::countWaypoints() {
  if (!st_.cardMounted) return 0;          // do not cache a zero from an absent card
  char path[64];
  wptPath(path, sizeof(path));
  uint16_t n = 0;
  switchToSd();
  File f = SD.open(path, FILE_READ);
  if (f) {
    char buf[160];
    while (f.available()) {
      int len = f.readBytesUntil('\n', buf, sizeof(buf) - 1);
      if (len < 0) break;
      buf[len] = '\0';
      while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n')) buf[--len] = '\0';
      if (len == 0) continue;
      // The header ("id,utc,...") is not a mark. Match on the literal prefix
      // because a data row always starts with a digit.
      if (strncmp(buf, WPT_CSV_HEADER, 3) != 0 && buf[0] != '#') n++;
    }
    f.close();
  }
  switchToTouch();
  st_.wptOnCard = n;
  st_.wptCounted = true;
  return n;
}

bool SdLogger::appendWaypoint(const char* row, uint32_t nowMs) {
  (void)nowMs;
  if (!row || !row[0]) return false;
  if (!st_.cardMounted) {
    st_.wptErrors++;
    snprintf(st_.lastError, sizeof(st_.lastError), "mark: no card");
    return false;
  }
  if (!st_.wptCounted) countWaypoints();   // first mark: learn where the ids are

  char path[64];
  wptPath(path, sizeof(path));
  bool ok = false;
  switchToSd();
  const bool exists = SD.exists(path);
  File f = SD.open(path, FILE_APPEND);
  if (f) {
    // Header once, on a new file, so the CSV reads as a CSV and not as a column
    // of bare numbers.
    if (!exists || f.size() == 0) {
      f.print(WPT_CSV_HEADER);
      f.print("\r\n");
    }
    const size_t want = strlen(row);
    const size_t got = f.print(row);
    f.print("\r\n");
    f.flush();
    f.close();
    ok = (got == want);
  }
  switchToTouch();

  if (ok) {
    st_.wptLines++;
    uint16_t id = (uint16_t)atoi(row);     // row starts with the mark id
    if (id > st_.wptOnCard) st_.wptOnCard = id;
  } else {
    st_.wptErrors++;
    snprintf(st_.lastError, sizeof(st_.lastError), "mark write failed");
  }
  return ok;
}
