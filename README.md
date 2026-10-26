# GNSS Explorer — standalone GPS/GNSS data explorer for the CYD

A self-contained GNSS data explorer: an ESP32 Cheap Yellow Display plus an
ATGM336H GPS/BeiDou receiver, driven entirely by the touchscreen. No phone, no
internet, no rotary encoder. The point is to look at everything the receiver
reports, not to navigate.

    Board   : ESP32-2432S028R "CYD" (2.8" ILI9341 240x320, XPT2046 touch)
    Receiver: ATGM336H (GPS + BeiDou), 9600 baud NMEA 0183, UART
    Power   : USB (the battery arrangement does not power the receiver yet)

## Views

| Tab | Content |
|---|---|
| Info | fix state, quality, coordinates, altitude, speed, course, UTC date/time, sats used/in view, HDOP/VDOP/PDOP, link diagnostics |
| Sats | per-satellite ID, constellation, signal bars, elevation, azimuth, used-in-fix |
| Sky | north-up polar plot of satellites with readable IDs |
| Raw | scrolling NMEA tail, pause/resume, message filter, serial diagnostics |
| Log | card state, file name, lines written, mount/frequency ladder, directory listing |
| Marks | every place marked, newest first, with distance and compass bearing from where you are now |

## Reception states

Four states, deliberately distinct, because "blank screen" and "no fix" are
different problems:

    NO DATA       nothing has ever arrived, or the stream stopped (>5s silent)
    DATA/NO FIX   sentences arriving, receiver says no fix yet
    FIX           valid fix, refreshing
    STALE         had a fix, updates stopped (>3s) while sentences still flow

## Wiring (as built, measured on the board)

    GPS VCC  -> 5V or 3.3V   (the board's LDO accepts 2.0-6.5V in, see below)
    GPS GND  -> GND
    GPS TX   -> GPIO 1       (P1's pin labelled TX — see below)
    GPS RX   -> not connected (this firmware never transmits)

That reads as TX-to-TX, and it is. On this board the connector labels describe
the **ESP32's** direction, not what the wire carries: P1 is
`VIN - TX(GPIO1) - RX(GPIO3) - GND`, and both of those pins belong to the
on-board CH340 USB bridge. GPIO1 feeds the CH340's *input* (harmless — an input
never argues with a driver). GPIO3 is driven by the CH340's *output*, which is
push-pull and idles high.

Measured with the internal ~45k pull-down deliberately fighting each pad:

    gpio3    HIGH under pull-down   -> a real driver holds it: unusable as an input
    gpio22   LOW  under pull-down   -> free
    gpio27   LOW  under pull-down   -> free

So a receiver's TX can never drive GPIO3 no matter how it is wired, and GPIO1 is
the pin that receives. That is exactly what HaleHound's own config declares —
`"GPS_bus": { "rx": 1, "tx": 3 }` — the board **receives** on GPIO1 and
**transmits** on GPIO3. Reading those keys as the module's pins instead is the
mistake that cost this project a day.

The cost: GPIO1 is also UART0's TX, i.e. the USB console. A build that receives
on GPIO1 must give the console up (`esp32-2432s028-gps1`, `ENABLE_USB_SERIAL=0`;
the code calls `Serial.end()` to release the pad the ROM bootloader leaves
routed to UART0 TX). To **keep** the console, put the receiver's TX on GPIO22 and
build `esp32-2432s028-gps22` — measured free, no contention. That is the better
long-term wiring; GPIO1 is the one that works with the wire where it already is.

Flashing works with the receiver on GPIO1 (tested), but the NMEA stream contends
with the bootloader's handshake. If an upload ever fails, unplug the receiver's
TX lead, flash, reconnect.

Module power: the receiver's own VCC limit is 2.7-3.6V, but this carrier board
has an on-board 3.3V LDO whose top marking is `DE=A1D` (CST6211CS5-33, input
2.0-6.5V), so 5V into VCC is safe. Do not exceed ~6.5V.

## Reading the receiver with no firmware in the path

    tools/passive_nmea_read.py 20 /dev/ttyUSB0 [save.txt]

Holds the ESP32 in reset (RTS = EN low) and reads 9600-baud NMEA straight off the
USB port: with nothing inside the chip driving GPIO1, the CH340's input sees the
receiver's own output. Use it to prove the module talks before blaming firmware,
and to capture real sentences for the host tests. With a silent build flashed
(`gps1`) the same command taps the live stream while the app runs — 6.5 kB and
120 sentences in 10 seconds, zero non-NMEA lines, is what a clean receive path
looks like.

## Stage 3: pause, filtering, and the footer

- **PAUSE / RESUME** in a footer strip that sits outside the content area, so a
  content refresh can never erase it. Pausing freezes the pixels, not the parser:
  the receiver keeps filling the ring, and resuming shows current data rather
  than replaying stale frames.
- **Raw filter chips: ALL / FIX / SAT / TXT**, matched on the parsed sentence
  type (talker-agnostic). The history ring is 96 sentences (~8s at this
  receiver's 12 sentences/second) so a filter still fills the screen.
- **Info is two pairs per line** and fits: the previous single-column layout
  needed 22 rows for a 254px area, so speed, course, UTC time, UTC date and fix
  quality were drawn off the bottom of the screen. Rows are now measured against
  TFT_eSPI's font table (font 2 is proportional — see the note in the wiki).
- **"n/a" instead of a dash** wherever the receiver reports nothing, with a legend
  on the Sats view, and Info naming what this module cannot do at all
  (SBAS/DGPS/RTK, GLONASS, Galileo, QZSS). Listing the sentence set the receiver
  emits is how the saved configuration is reported — without ever writing to it.

## Stage 5: marking places

A **MARK** button in the footer, on every tab (a mark is taken while you are
looking at something else). One press writes the current position to
`/gps/waypoints.csv` on the card and adds it to the Marks view, which shows every
mark with its distance and compass bearing from where you are standing now.

```
id,utc,lat,lon,alt_msl,sats,hdop
1,2026-10-26T09:45:12Z,35.028848,-87.192328,215.1,7,1.70
2,2026-10-26T09:46:40Z,35.028910,-87.192100,-,8,1.40
```

- **A mark is refused unless the fix is valid AND current.** Pressing MARK in
  NO DATA, DATA/NO FIX or STALE writes nothing and says which state refused it.
  This is the whole point: a coordinate with no position behind it cannot be told
  apart from a good one once it is on the card, so a bad mark is worse than no
  mark. The refusal count is shown on screen.
- Fields the receiver did not report are written as `-`, never as 0 — a mark at
  altitude 0 m in Tennessee would be a lie. `utc` falls back to `boot+<n>s` for a
  mark taken before the first fix names the clock.
- **The card keeps every mark; RAM keeps the newest 32.** No card (or a build
  without logging) is not fatal: marks are held in RAM and the view says
  `RAM only` so it is clear they will not survive a reboot. When the RAM window
  is full the oldest is dropped and the banner says so.
- Ids continue from the card (`countWaypoints()` on the first press), so mark #12
  after a reboot is still #12 in the file instead of a second #1.
- Each mark also drops a `# mark #7 2026-10-26T09:45:12Z` app-event line into the
  NMEA log, so the mark sits in the timeline of the raw stream. The log file
  itself still contains raw sentences only, and the marks have their own file.
- Distance is haversine on a 6371 km sphere: ~0.3% (about 3 m per km) against the
  ellipsoid, which is well inside the receiver's own horizontal error. **Altitude
  is the weak axis** (~8 m wander) — marks are horizontal points, and the view
  says so rather than implying centimetres.
- `tools/topo_map.py` reads `waypoints.csv` unchanged (`--csv`), so the marks
  land on the same 3DEP hillshade map as the track.

## Surveying what the receiver actually says

    tools/nmea_survey.py 60 /dev/ttyUSB0 [--no-reset]

Inventories the stream instead of guessing at it: sentence types and talkers with
their rates, a per-constellation satellite/SNR table, used-in-fix sets, DOPs,
position scatter, the module's own TXT health lines, and anything on the line that
is **not** NMEA text (binary/proprietary frames, garbage, a bootloader). Add
`--no-reset` to watch the line while the app runs.

Measured on this module: 12 sentences per 1 Hz cycle, talkers GN/GP/BD only (GPS +
BeiDou, no GLONASS/Galileo/QZSS), 720/720 checksums clean, zero non-NMEA bytes in a
minute, HDOP 1.0-1.2, and stationary scatter of ±8 m horizontal against 57 m
peak-to-peak vertically — the altitude is not worth trusting.

That vertical figure is now explained rather than just observed: `tools/dem_check.py`
compares every fix against USGS 3DEP terrain and reports the error next to the
geometry that produced it. Indoors, VDOP runs 4.3 against HDOP 1.4 (vertical
geometry 3.1x worse, as a blocked sky view implies), the implied ranging error is
7.8 m against the 1.5-3 m an open-sky unit manages, and `corr(|error|, VDOP)` is
+0.86. Median error was -33.6 m and the reported altitude wandered 37.8 m while the
antenna sat still. It matches neither the orthometric nor the ellipsoidal height, so
there is no constant to subtract — the claim to test is that open sky collapses it.

## Parser rules worth knowing

Talker IDs are whatever the receiver actually emits, and this one mixes three:
`$GNGGA`/`$GNRMC`/`$GNGSA` are **combined** (GN), while `$GPGSV` and `$BDGSV` are
per-constellation. Satellites are keyed by (talker, prn), so a `$GNGSA` names
satellites that GSV reports under GP/BD — the link is the NMEA 4.11 **systemID**
after VDOP (1=GPS, 4=BeiDou). Without mapping it, this receiver's second GSA
wipes the first one's used-in-fix flags. `tools/host_tests/` covers it against
230 sentences captured off the board.

## Passive by design

The firmware **never writes a byte to the receiver**. No $PCAS commands, no
baud changes, no constellation configuration — the module's saved
configuration is left exactly as it is. Talker IDs are whatever the receiver
actually emits (GP*, BD*, GL*, GN*), and multi-part GSV/GSA sentences are all
accepted. Fields the receiver does not report are shown as `--`.

## Build

    pio run -e esp32-2432s028                 # firmware.elf/.bin in .pio/build/
    pio run -e esp32-2432s028 -t clean        # full rebuild

Toolchain: PlatformIO Core 6.2.0, platform espressif32, framework arduino,
board esp32dev, 4MB flash. Display/touch build flags are copied from the
hardware-verified OneBoard profile (ILI9341 on HSPI 12/13/14/15/2 with
`USE_HSPI_PORT`; XPT2046 on its own VSPI bus 25/32/39/33/36).

### Flashable single image

    PY=~/.local/share/pipx/venvs/platformio/bin/python
    ESP=~/.platformio/packages/tool-esptoolpy/esptool.py
    B=.pio/build/esp32-2432s028
    $PY $ESP --chip esp32 merge_bin -o dist/gnss-explorer-stage1-merged.bin \
      --flash_mode dio --flash_freq 40m --flash_size 4MB \
      0x1000 $B/bootloader.bin 0x8000 $B/partitions.bin \
      0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
      0x10000 $B/firmware.bin

## Flashing (Linux)

    lsusb | grep -iE '1a86|10c4'          # CH340 = 1a86:7523 -> /dev/ttyUSB0
    pio run -t upload --upload-port /dev/ttyUSB0

Or write a merged image yourself:

    $PY $ESP --chip esp32 --port /dev/ttyUSB0 --baud 921600 \
      write_flash 0x0 dist/gnss-explorer-gnssauto-log-merged.bin

Images (rebuilt 2026-10-26, all include Stage 5 marks):

    dist/gnss-explorer-gnssauto-log-merged.bin    FIELD: SD logging, no console,
                                                  receiver pin auto-detected
    dist/gnss-explorer-gps1-log-merged.bin   logs to the microSD card, rx=1 fixed
    dist/gnss-explorer-gps3-log-merged.bin   SD logging with the console left on
    dist/gnss-explorer-sdprobe-merged.bin    SD mount + touch-coexistence probe
    dist/gnss-explorer-gnssauto-probe-merged.bin  auto-detect + console (scans 22/27)
    dist/gnss-explorer-gps1-merged.bin       stage 3, no card logging

**Unplug the receiver's TX lead before flashing.** The GPS streams NMEA into
GPIO3 continuously, which is the same line the flasher uses to send upload
commands, and the handshake can fail or produce a corrupt image. Power the
module only after the upload finishes. (If it still fails: the CH340 needs a
hard reset, and on Ubuntu `brltty` can hold the port — stop it.)

## Watching it from the PC

The firmware mirrors diagnostics to the CH340 at 115200, so the link can be
verified without reading the screen:

    PY=~/.local/share/pipx/venvs/platformio/bin/python
    $PY -c "
    import serial, time
    s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    s.setDTR(False); s.setRTS(True); time.sleep(0.15); s.setRTS(False)
    time.sleep(0.2); s.reset_input_buffer()
    end = time.time() + 15
    while time.time() < end:
        d = s.read(512)
        if d: print(d.decode('utf-8','replace'), end='')
    "

Expect `[boot]`, then `[gnss] state=... bytes=... talkers=[...]` every 5s.
`bytes=0` after 15s means nothing is arriving on GPIO3 — try the alternate env.

## Touch calibration

The raw->screen map is currently the documented proportional fallback
(X 200..3700, Y 240..3800). That is good enough to tap the tab bar, but the
tab bar is a stacked layout, so measure before trusting it: the Raw tab prints
`raw x,y -> screen x,y` live. Tap the four screen corners, note the raw values,
then replace `RAW_X_MIN/MAX` and `RAW_Y_MIN/MAX` in `src/main.cpp` with the
measured ones (and set the axis-swap if x and y come out exchanged).

## Status

- Stage 1: display + touch tabs + passive GPS reception, link diagnostics,
  four-state machine, raw NMEA tail. **Verified on the bench**: panel confirmed
  by controller ID read (`RDID4 0xD3 -> 41 00 93 41`), backlight driving fixed,
  tabs switching on tap, GPS link silent pending a wiring correction (see above).
- Stage 2: **done** — per-satellite parsing (GSV per constellation: prn,
  elevation, azimuth, SNR; used-in-fix from GSA), signal bars, north-up sky plot.
- Stage 3: pause/resume + filtering on Raw, saved-config awareness, field-level
  "unavailable" polish.
- Stage 4: **done** — raw-NMEA logging to the microSD card with the shared VSPI
  bus re-routed per access, mount-frequency ladder, file named from the UTC clock.
- Stage 5: **done, bench-untested** — MARK on every tab writes
  `/gps/waypoints.csv`, Marks tab with distance/bearing, marks refused unless the
  fix is valid and current. Host tests cover the rules, the CSV row and the maths;
  the button itself still needs a press on the glass.

## Tests

The parser is testable without the receiver, because NMEA is text and the
expected values are arithmetic:

    ./tools/host_tests/run.sh

Two binaries. The first compiles the real `src/gnss.cpp` on the PC against a small
Arduino shim and feeds it real sentences (74 checks: field offsets for
GGA/RMC/GSA/GSV, multi-part GSV assembly, per-constellation PRN keys, GSA
used-set, corrupt checksums, the no-data state, and satellite staleness). Run it
after touching the parser — it has already caught an off-by-one on every sentence
field and a date field read with the wrong width.

The second compiles the real `src/waypoints.cpp` (no shim needed — it touches
nothing but stdio and math) and checks the things that fail *silently* on the
board: the CSV row byte-for-byte, the refusal rules and their counters, id
continuation, ring rotation, and distance/bearing/compass against computed
offsets (48 checks).

## Files

    platformio.ini      build config, both GPS pin wiring options
    src/main.cpp        UI: tabs, touch, drawing, state rendering
    src/gnss.h/.cpp     passive NMEA reader + fix state machine
    src/waypoints.h/.cpp marked places: refusal rules, RAM ring, CSV row, geo maths
    src/sdlog.h/.cpp    raw-NMEA logging to the microSD card (shared VSPI bus)
    tools/host_tests/   PC-side tests: parser (gnss.cpp + shim) and marks
    tools/serial_capture.py  listen-only USB serial capture
    tools/topo_map.py   USGS 3DEP topo map + hillshade + GNSS track overlay
    tools/dem_check.py  per-fix altitude vs terrain, with the geometry behind it

## Repository layout

    dist/               merged single-image builds, ready to flash (above)
    src/  tools/        firmware and the PC-side tooling
    LICENSE             MIT

Two things are deliberately not in the repo, both for the same reason — they
record where somebody stood:

- `logs/` — raw NMEA pulled off the card. A capture from a walk is a position
  history, the same family of data as the marks in `/gps/waypoints.csv`. Your own
  captures stay local; the repo carries `tools/host_tests/sample_nmea.txt`
  instead, a short capture that drives the same parser paths in the tests.
- `tools/.cache/` — downloaded USGS 3DEP DEM tiles, whose filenames encode a
  bounding box. `dem_check.py` and `topo_map.py` refetch them on demand.

Two more things that only fail silently, so they get their own build:

- MicroSD and touch share **one VSPI peripheral on different pins** (SD 18/19/23/5
  vs touch 25/39/32/33), so the bus is re-pointed per card access and writes are
  batched at 2 KB or 1 s — two pin re-routes per flush, counted on the Log tab.
- `esp32-2432s028-sdprobe` exists because a card that will not mount and a touch
  panel that goes quiet after the first bus re-route both look like nothing at all.
  It writes while counting taps on the glass.
