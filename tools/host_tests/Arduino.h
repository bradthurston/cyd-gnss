// Host-side Arduino shim.
//
// Just enough of the Arduino API to compile the GNSS parser off-target, so its
// field handling can be tested against real sentences on a PC instead of by
// watching a display on a bench. Only what gnss.cpp actually touches is here.
#pragma once

#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Test-controlled clock: the tests move it forward to exercise the staleness
// and "no data" paths without waiting in real time.
extern uint32_t g_hostMillis;
inline uint32_t millis() { return g_hostMillis; }

#ifndef SERIAL_8N1
#define SERIAL_8N1 0x800001c
#endif

struct HostSerial {
  operator bool() const { return true; }
  void begin(unsigned long, uint32_t = SERIAL_8N1, int = -1, int = -1) {}
  void begin(unsigned long) {}
  void end() {}
  int available() { return 0; }
  int read() { return -1; }
  int printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vprintf(fmt, ap);
    va_end(ap);
    return n;
  }
  void println(const char* s) { printf("%s\n", s); }
};

extern HostSerial Serial;
extern HostSerial Serial1;
