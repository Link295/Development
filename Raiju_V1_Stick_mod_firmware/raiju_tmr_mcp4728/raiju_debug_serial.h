#pragma once

#include <Arduino.h>

// Serial backpressure guarding helpers.
// Kept in a header to avoid Arduino .ino auto-prototype generation breaking
// template declarations.

#if DEBUG_LEVEL >= 2

template <typename S>
static inline auto serialBoolImpl(S& s, int) -> decltype((bool)s) {
  return (bool)s;
}

template <typename S>
static inline bool serialBoolImpl(S&, ...) {
  // Some cores don't implement operator bool on Serial; assume usable.
  return true;
}

template <typename S>
static inline auto serialAvailableForWriteImpl(S& s, int) -> decltype(s.availableForWrite()) {
  return s.availableForWrite();
}

template <typename S>
static inline int serialAvailableForWriteImpl(S&, ...) {
  // Not supported on this core.
  return -1;
}

static inline bool debugSerialCanWrite(int minBytes) {
  if (!serialBoolImpl(Serial, 0)) return false;
  const int avail = serialAvailableForWriteImpl(Serial, 0);
  if (avail < 0) {
    // If we can't query backpressure, allow prints (throttled) rather than
    // disabling debug output entirely.
    return true;
  }
  return avail > minBytes;
}

// Shared across PERF + periodic debug prints.
static uint32_t g_serialBackpressureCount = 0;

#else

static inline bool debugSerialCanWrite(int) {
  return false;
}

#endif
