#include "gnss.h"

#ifndef GPS_RX_PIN
#define GPS_RX_PIN 3
#endif
#ifndef GPS_TX_PIN
#define GPS_TX_PIN -1
#endif
#ifndef GPS_BAUD
#define GPS_BAUD 9600
#endif

// ---------------------------------------------------------------- utilities

// Split a comma separated NMEA body in place. Returns field count.
static int splitFields(char* s, char* out[], int maxFields) {
  int n = 0;
  out[n++] = s;
  for (char* p = s; *p && n < maxFields; ++p) {
    if (*p == ',') {
      *p = '\0';
      out[n++] = p + 1;
    }
  }
  return n;
}

// ddmm.mmmm / dddmm.mmmm -> decimal degrees. Returns false if unusable.
static bool nmeaToDeg(const char* v, char hemi, double* out) {
  if (!v || !*v) return false;
  double raw = atof(v);
  if (raw == 0.0) return false;
  int deg = (int)(raw / 100.0);
  double min = raw - (deg * 100.0);
  double d = deg + (min / 60.0);
  if (hemi == 'S' || hemi == 'W') d = -d;
  *out = d;
  return true;
}

static const char* fixQualityName(uint8_t q) {
  switch (q) {
    case 0: return "none";
    case 1: return "GPS fix";
    case 2: return "DGPS";
    case 3: return "PPS";
    case 4: return "RTK fixed";
    case 5: return "RTK float";
    case 6: return "dead reck";
    default: return "unknown";
  }
}

// Two ASCII digits at p -> value. Fixed width matters: atoi("230394") is a
// single number, so the day/month must be read two characters at a time.
static bool twoDigits(const char* p, int* out) {
  char a = p[0], b = p[1];
  if (a < '0' || a > '9' || b < '0' || b > '9') return false;
  *out = (a - '0') * 10 + (b - '0');
  return true;
}

static bool addUnique(char* list, uint8_t* count, const char* tok) {
  if (*count >= 8) return false;
  size_t tokLen = strlen(tok);
  if (tokLen == 0) return false;
  if (*list) {
    // already present?
    const char* p = list;
    while (*p) {
      const char* sp = strchr(p, ' ');
      size_t len = sp ? (size_t)(sp - p) : strlen(p);
      if (len == tokLen && strncmp(p, tok, len) == 0) return false;
      if (!sp) break;
      p = sp + 1;
    }
    if (strlen(list) + 1 + tokLen + 1 > 39) return false;
    strcat(list, " ");
    strncat(list, tok, tokLen);
  } else {
    strncat(list, tok, tokLen);
  }
  (*count)++;
  return true;
}

// ------------------------------------------------------------------ reader

// Which pin the receiver was actually found on (auto-detect builds); GPS_RX_PIN
// otherwise. Shown on the Raw tab and written into the log's status line so a card
// file proves the wiring from off-site.
int gpsActivePin = GPS_RX_PIN;

void GnssReader::begin() {
  memset(&s_, 0, sizeof(s_));
  // The satellite table lives outside the snapshot, so clear it explicitly:
  // otherwise satellites tracked before a re-begin would still be counted.
  memset(sats_, 0, sizeof(sats_));
  satCount_ = 0;
  memset(curTalker_, 0, sizeof(curTalker_));
  s_.state = ST_NO_DATA;
  strcpy(s_.stateText, "NO DATA");
  s_.msSinceBoot = millis();
  secWindowStart_ = millis();
#ifdef GPS_AUTO_DETECT
  // Wiring-agnostic startup. This board's receiver has been rewired more than once
  // (P1's RX pad is held high by the CH340, so GPIO3 can never work), and a silent
  // logger on the bench looks exactly like a logging bug. So listen on every pin it
  // could plausibly be on, keep the first one that actually produces validated
  // sentences, and record it. GPIO1 costs the USB console - harmless here, the field
  // build has console off - GPIO22/27 are the console-friendly alternatives.
  // Scan order is a build flag so a console build can drop the pin its own console
  // has been moved onto.
#ifndef GPS_SCAN_PINS
#define GPS_SCAN_PINS 1, 22, 27
#endif
  static const int cands[] = {GPS_SCAN_PINS};
  int picked = 0;
  for (unsigned i = 0; i < sizeof(cands) / sizeof(cands[0]) && !picked; i++) {
    Serial1.end();
    Serial1.begin(GPS_BAUD, SERIAL_8N1, cands[i], -1);
    uint32_t before = s_.ggaSentences + s_.rmcSentences + s_.gsvSentences;
    uint32_t t0 = millis();
    while (millis() - t0 < 2000) {
      while (Serial1.available()) feed(Serial1.read());
      if (s_.ggaSentences + s_.rmcSentences + s_.gsvSentences - before >= 2) {
        picked = cands[i];
        break;
      }
      delay(2);
    }
  }
  gpsActivePin = picked ? picked : GPS_RX_PIN;
  Serial1.end();
  Serial1.begin(GPS_BAUD, SERIAL_8N1, gpsActivePin, -1);
  s_.msSinceBoot = millis();
#else
  gpsActivePin = GPS_RX_PIN;
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif
}



void GnssReader::clearLines() {
  ringCount_ = 0;
  ringHead_ = 0;
  memset(ring_, 0, sizeof(ring_));
}

void GnssReader::poll() {
  s_.msSinceBoot = millis();

  int avail = Serial1.available();
  // Drain in bounded chunks so the UI never stalls on a chatty receiver.
  for (int i = 0; i < avail && i < 512; ++i) {
    int c = Serial1.read();
    if (c < 0) break;
    feed((uint8_t)c);
  }

  uint32_t now = millis();
  if (s_.firstByteSeen) s_.msSinceByte = now - lastByteMs_;
  if (lastFixMs_) s_.msSinceFix = now - lastFixMs_;

  // lines/sec, updated once a second
  if (now - secWindowStart_ >= 1000) {
    uint32_t elapsed = now - secWindowStart_;
    s_.linesPerSec = (uint32_t)((secWindowLines_ * 1000ULL) / (elapsed ? elapsed : 1));
    secWindowLines_ = 0;
    secWindowStart_ = now;
  }
  classify();
  recountSats();   // let tracked-satellite counts decay with time, not only
                   // when a GSV happens to arrive
}

void GnssReader::feed(uint8_t b) {
  if (!s_.firstByteSeen) {
    s_.firstByteSeen = true;
    if (Serial) Serial.printf("[gnss] first byte on rx=%d baud=%d\n", GPS_RX_PIN, GPS_BAUD);
  }
  s_.bytesTotal++;
  lastByteMs_ = millis();

  if (b == '\n') {
    if (!dropUntilNewline_ && bufLen_ > 0) {
      buf_[bufLen_] = '\0';
      handleLine(buf_);
    }
    bufLen_ = 0;
    dropUntilNewline_ = false;
    return;
  }
  if (b == '\r') return;

  if (dropUntilNewline_) return;
  if (bufLen_ >= GNSS_LINE_MAX - 1) {   // overlong line: discard, resync on \n
    dropUntilNewline_ = true;
    bufLen_ = 0;
    return;
  }
  buf_[bufLen_++] = (char)b;
}

void GnssReader::pushLine(const char* text, uint8_t talker, const char* type, bool ok) {
  NmeaLine& l = ring_[ringHead_];
  strncpy(l.text, text, GNSS_LINE_MAX - 1);
  l.text[GNSS_LINE_MAX - 1] = '\0';
  l.atMs = millis();
  l.talker = talker;
  strncpy(l.type, type, 3);
  l.type[3] = '\0';
  l.checksumOk = ok;
  ringHead_ = (ringHead_ + 1) % GNSS_RING;
  if (ringCount_ < GNSS_RING) ringCount_++;
}

const NmeaLine& GnssReader::line(int newestIndex) const {
  static NmeaLine empty = {};
  if (newestIndex < 0 || newestIndex >= ringCount_) return empty;
  int idx = (ringHead_ - 1 - newestIndex + GNSS_RING * 2) % GNSS_RING;
  return ring_[idx];
}

void GnssReader::noteTalkerType(const char* line) {
  // line looks like "$GPGGA,..." — talker = chars 1..2 (possibly single char), type = last 3 before ',' or '*'
  if (line[0] != '$') return;
  char talker[3] = {0, 0, 0};
  talker[0] = line[1];
  talker[1] = line[2];
  char type[4] = {0, 0, 0, 0};
  const char* comma = strchr(line, ',');
  const char* star = strchr(line, '*');
  const char* end = comma ? comma : (star ? star : line + strlen(line));
  if (end - line >= 6) {
    type[0] = end[-3];
    type[1] = end[-2];
    type[2] = end[-1];
  }
  curTalker_[0] = talker[0];
  curTalker_[1] = talker[1];
  curTalker_[2] = '\0';
  addUnique(talkerList_, &s_.talkerCount, talker);
  addUnique(typeList_, &s_.typeCount, type);
  strncpy(s_.talkers, talkerList_, sizeof(s_.talkers) - 1);
  strncpy(s_.typesSeen, typeList_, sizeof(s_.typesSeen) - 1);
}

void GnssReader::injectLine(const char* sentence) {
  char buf[GNSS_LINE_MAX];
  strncpy(buf, sentence, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  if (!s_.firstByteSeen) s_.firstByteSeen = true;
  lastByteMs_ = millis();
  handleLine(buf);
}

void GnssReader::handleLine(char* line) {
  s_.linesTotal++;
  secWindowLines_++;

  // checksum: everything between $ and * XORed
  bool ok = false;
  char* star = strchr(line, '*');
  if (star && line[0] == '$') {
    uint8_t sum = 0;
    for (char* p = line + 1; p < star; ++p) sum ^= (uint8_t)*p;
    uint8_t given = (uint8_t)strtol(star + 1, nullptr, 16);
    ok = (sum == given);
  }
  if (!ok) s_.badChecksums++;

  noteTalkerType(line);
  char type[4] = {0, 0, 0, 0};
  char* comma = strchr(line, ',');
  const char* tail = comma ? comma : (star ? star : line + strlen(line));
  if (tail - line >= 6) {
    type[0] = tail[-3]; type[1] = tail[-2]; type[2] = tail[-1];
  }

  // Never parse a sentence whose checksum failed — a truncated or corrupted
  // line produces plausible-looking garbage coordinates.
  if (ok && comma) {
    // Parse a COPY. splitFields() NULs the commas in place and the star is
    // stripped, and this same sentence is handed to pushLine() -> the Raw tab
    // and the SD card. Parsing the live buffer mutated what got logged: every
    // line on the card was a prefix of the real sentence (GGA/RMC/GSV/GSA cut
    // at their 2nd field) and every checksum was missing, so an outdoor log
    // had zero usable coordinates while the on-screen parser looked perfect.
    char work[GNSS_LINE_MAX];
    strncpy(work, line, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';
    char* wcomma = strchr(work, ',');
    char* wstar = strchr(work, '*');
    if (wstar) *wstar = '\0';
    if (wcomma) {
      char* body = wcomma + 1;
      if (strcmp(type, "GGA") == 0) handleGga(body);
      else if (strcmp(type, "RMC") == 0) handleRmc(body);
      else if (strcmp(type, "GSA") == 0) handleGsa(body);
      else if (strcmp(type, "GSV") == 0) handleGsv(body);
    }
  }

  pushLine(line, 0, type, ok);
  if (Serial && !ok) Serial.printf("[gnss] bad checksum: %s\n", line);
}

// ------------------------------------------------- satellite table (Stage 2)
// Satellites are keyed by (talker, prn): GPS PRN 5 and BeiDou PRN 5 are
// different satellites and must not overwrite each other.
void GnssReader::mergeGsvSat(int prn, int elev, int az, int snr) {
  if (prn <= 0) return;

  GnssSat* slot = nullptr;
  for (int i = 0; i < GNSS_MAX_SATS; ++i) {
    if (sats_[i].prn == (uint8_t)prn && strncmp(sats_[i].talker, curTalker_, 2) == 0) {
      slot = &sats_[i];
      break;
    }
  }
  if (!slot) {
    for (int i = 0; i < GNSS_MAX_SATS; ++i) {
      if (sats_[i].prn == 0) { slot = &sats_[i]; break; }
    }
    if (!slot) return;                       // table full, drop this one
    slot->prn = (uint8_t)prn;
    strncpy(slot->talker, curTalker_, 2);
    slot->talker[2] = '\0';
    slot->elevDeg = -1;
    slot->azDeg = -1;
    slot->snr = -1;
    slot->used = false;
  }
  // A receiver may omit elevation/azimuth on a later cycle; keep the last
  // known value rather than blanking a field it simply didn't repeat.
  if (elev >= 0) slot->elevDeg = (int8_t)(elev > 90 ? 90 : elev);
  if (az >= 0) slot->azDeg = (int16_t)(az % 360);
  if (snr >= 0) slot->snr = (int8_t)(snr > 99 ? 99 : snr);
  slot->heardMs = millis();
}

// GSA lists the satellites actually used in the solution for one constellation.
// NMEA 4.11 puts a systemID after VDOP (1=GPS 2=GLONASS 3=Galileo 4=BeiDou
// 5=QZSS). It matters here: this receiver emits $GNGSA twice, once per
// constellation, so without it the second sentence clears the first one's flags
// and a PRN shared by two constellations lands on the wrong one.
void GnssReader::applyGsaSatUse(char* f[], int n) {
  char target[3] = {curTalker_[0], curTalker_[1], '\0'};
  bool combined = (strncmp(curTalker_, "GN", 2) == 0);
  if (combined && n > 17 && f[17][0]) {
    switch (atoi(f[17])) {
      case 1: strcpy(target, "GP"); break;
      case 2: strcpy(target, "GL"); break;
      case 3: strcpy(target, "GA"); break;
      case 4: strcpy(target, "BD"); break;
      case 5: strcpy(target, "GQ"); break;
      default: break;               // unknown systemID: mark by PRN only
    }
  }
  bool specific = (strncmp(target, "GN", 2) != 0);

  for (int i = 0; i < GNSS_MAX_SATS; ++i) {
    if (sats_[i].prn == 0) continue;
    if (!specific || strncmp(sats_[i].talker, target, 2) == 0) sats_[i].used = false;
  }
  for (int k = 2; k <= 13 && k < n; ++k) {
    if (!f[k][0]) continue;
    int prn = atoi(f[k]);
    if (prn <= 0) continue;
    for (int i = 0; i < GNSS_MAX_SATS; ++i) {
      if (sats_[i].prn != (uint8_t)prn) continue;
      if (specific && strncmp(sats_[i].talker, target, 2) != 0) continue;
      sats_[i].used = true;
    }
  }
}

// A satellite not mentioned for this long is no longer being tracked; the
// receiver has dropped it from its sky view.
void GnssReader::recountSats() {
  const uint32_t SAT_STALE_MS = 15000;
  uint8_t count = 0, best = 0;
  for (int i = 0; i < GNSS_MAX_SATS; ++i) {
    if (sats_[i].prn == 0) continue;
    if (millis() - sats_[i].heardMs > SAT_STALE_MS) continue;
    count++;
    if (sats_[i].snr > 0 && (uint8_t)sats_[i].snr > best) best = (uint8_t)sats_[i].snr;
  }
  satCount_ = count;
  s_.satsTracked = count;
  s_.bestSnr = best;
}

// $xxGGA,utc,lat,N,lon,E,fixQuality,satsUsed,hdop,alt,M,geoid,M,age,station
// The body excludes talker+type, so f[0] IS the UTC field: every index here is
// the raw NMEA field order. (An earlier revision was shifted by one and read
// the hemisphere letter as the latitude.)
void GnssReader::handleGga(char* body) {
  char* f[20];
  int n = splitFields(body, f, 20);
  s_.ggaSentences++;

  if (n > 0 && f[0][0]) {
    int hh = 0, mm = 0; double ss = 0;
    if (sscanf(f[0], "%2d%2d%lf", &hh, &mm, &ss) == 3) {
      s_.hour = hh; s_.minute = mm; s_.second = (uint8_t)ss;
      s_.timeValid = true;
    }
  }
  double d, lon;
  char ns = (n > 2 && f[2][0]) ? f[2][0] : 'N';
  char ew = (n > 4 && f[4][0]) ? f[4][0] : 'E';
  if (n > 1 && nmeaToDeg(f[1], ns, &d)) {
    if (n > 3 && nmeaToDeg(f[3], ew, &lon)) {
      s_.lat = d; s_.lon = lon; s_.latLonValid = true;
    }
  }
  if (n > 5) {
    s_.fixQuality = (uint8_t)atoi(f[5]);
    strncpy(s_.fixQualityText, fixQualityName(s_.fixQuality), sizeof(s_.fixQualityText) - 1);
  }
  if (n > 6 && f[6][0]) s_.satsUsed = (uint8_t)atoi(f[6]);
  if (n > 7 && f[7][0]) { s_.hdop = atof(f[7]); s_.hdopValid = true; }
  if (n > 8 && f[8][0]) { s_.altM = atof(f[8]); s_.altValid = true; }
  if (n > 10 && f[10][0]) { s_.geoidSepM = atof(f[10]); s_.geoidValid = true; }

  // GGA is the authoritative "do we have a fix right now" sentence.
  s_.fixValid = (s_.fixQuality > 0);
  if (s_.fixValid) lastFixMs_ = millis();
}

// $xxRMC,utc,status,lat,N,lon,E,speedKts,course,date,magvar,E/W,mode
void GnssReader::handleRmc(char* body) {
  char* f[20];
  int n = splitFields(body, f, 20);
  s_.rmcSentences++;

  if (n > 0 && f[0][0]) {
    int hh = 0, mm = 0; double ss = 0;
    if (sscanf(f[0], "%2d%2d%lf", &hh, &mm, &ss) == 3) {
      s_.hour = hh; s_.minute = mm; s_.second = (uint8_t)ss;
      s_.timeValid = true;
    }
  }
  bool active = (n > 1 && f[1][0] == 'A');
  double d, lon;
  char ns = (n > 3 && f[3][0]) ? f[3][0] : 'N';
  char ew = (n > 5 && f[5][0]) ? f[5][0] : 'E';
  if (n > 2 && nmeaToDeg(f[2], ns, &d)) {
    if (n > 4 && nmeaToDeg(f[4], ew, &lon)) {
      s_.lat = d; s_.lon = lon; s_.latLonValid = true;
    }
  }
  if (n > 6 && f[6][0]) {
    s_.speedKts = atof(f[6]);
    s_.speedKph = s_.speedKts * 1.852f;
    s_.speedValid = true;
  }
  if (n > 7 && f[7][0]) { s_.courseDeg = atof(f[7]); s_.courseValid = true; }
  if (n > 8 && strlen(f[8]) >= 6) {
    int dd = 0, mo = 0, yy2 = 0;
    if (twoDigits(f[8], &dd) && twoDigits(f[8] + 2, &mo) && twoDigits(f[8] + 4, &yy2)) {
      s_.day = (uint8_t)dd;
      s_.month = (uint8_t)mo;
      // NMEA two-digit year: 00-79 is 2000-2079, 80-99 is 1980-1999.
      s_.year = (yy2 < 80) ? (uint16_t)(2000 + yy2) : (uint16_t)(1900 + yy2);
      s_.dateValid = true;
    }
  }
  if (active && s_.fixQuality == 0) {
    // RMC can report active before GGA updates; treat as fix evidence.
    s_.fixValid = true;
    lastFixMs_ = millis();
  }
}

// $xxGSA,mode,fixType,sv1..sv12,pdop,hdop,vdop
void GnssReader::handleGsa(char* body) {
  char* f[24];
  int n = splitFields(body, f, 24);
  if (n > 16 && f[16][0]) { s_.vdop = atof(f[16]); s_.vdopValid = true; }
  if (n > 15 && f[15][0]) { s_.hdop = atof(f[15]); s_.hdopValid = true; }
  if (n > 14 && f[14][0]) { s_.pdop = atof(f[14]); s_.pdopValid = true; }
  if (n > 1 && f[1][0]) {
    int ft = atoi(f[1]);              // 1 = no fix, 2 = 2D, 3 = 3D
    if (ft == 0 || ft == 1) s_.fixValid = false;
    else { s_.fixValid = true; lastFixMs_ = millis(); }
  }
  applyGsaSatUse(f, n);
}

// $xxGSV,totalMsgs,msgNum,satsInView,[prn,elev,az,snr] x up to 4
void GnssReader::handleGsv(char* body) {
  char* f[24];
  int n = splitFields(body, f, 24);
  s_.gsvSentences++;

  if (n > 1) {
    int total = atoi(f[0]);
    int part = atoi(f[1]);
    if (total > 0 && part > s_.gsvPartsThisCycle) s_.gsvPartsThisCycle = (uint8_t)part;
    if (total > 0 && part >= total) s_.gsvPartsThisCycle = 0;  // cycle complete
  }
  if (n > 2 && f[2][0]) {
    int inView = atoi(f[2]);
    // Each constellation advertises its own count; keep the largest seen.
    if (inView > s_.satsInView) s_.satsInView = (uint8_t)inView;
  }

  // Satellite groups start at f[3] and are four fields wide. Partial groups
  // (receiver omitting trailing blank fields) are handled by the empties.
  for (int base = 3; base + 3 < n; base += 4) {
    if (!f[base][0] && !f[base + 1][0] && !f[base + 2][0] && !f[base + 3][0]) continue;
    int prn = f[base][0] ? atoi(f[base]) : 0;
    if (prn <= 0) continue;
    int elev = f[base + 1][0] ? atoi(f[base + 1]) : -1;
    int az = f[base + 2][0] ? atoi(f[base + 2]) : -1;
    int snr = f[base + 3][0] ? atoi(f[base + 3]) : -1;
    mergeGsvSat(prn, elev, az, snr);
  }
  recountSats();
}

// Decide which of the four states we are in. Order matters: no data beats
// everything, then staleness, then fix validity.
void GnssReader::classify() {
  const uint32_t DATA_TIMEOUT_MS = 5000;   // nothing for this long = no data
  const uint32_t FIX_STALE_MS = 3000;      // fix stopped refreshing

  if (!s_.firstByteSeen || (millis() - lastByteMs_) > DATA_TIMEOUT_MS) {
    s_.state = ST_NO_DATA;
    strcpy(s_.stateText, "NO DATA");
    s_.fixValid = false;
  } else if (s_.fixValid) {
    if (lastFixMs_ && (millis() - lastFixMs_) > FIX_STALE_MS) {
      s_.state = ST_STALE;
      strcpy(s_.stateText, "STALE");
    } else {
      s_.state = ST_FIX;
      strcpy(s_.stateText, "FIX");
    }
  } else {
    s_.state = ST_DATA_NO_FIX;
    strcpy(s_.stateText, "DATA / NO FIX");
  }
}
