#pragma once

#include <Arduino.h>
#include <stddef.h>

// Types are separated into a header because Arduino's .ino preprocessor injects
// function prototypes above the sketch body. Any function that uses a custom
// type (enum/struct) in its signature can fail to compile unless the type is
// known before those injected prototypes.

enum class CalStatus : uint8_t {
  None = 0,
  Valid = 1,
  FallbackUsed = 2,
};

struct AxisConfig {
  uint8_t pin;
  bool invert;

  // Usable ADC range per axis; defaults to 0..4095.
  uint16_t inMin;
  uint16_t inMax;

  uint16_t deadzone;
  float alpha;
};

struct AxisState {
  uint16_t center;
  float filteredNorm; // filtered normalized value in [-1..1]
};

// Byte-optimized calibration data.
// Target: <=48 bytes total, <=128 bytes per slot.
struct alignas(4) CalibrationData {
  uint32_t magic;      // 4
  uint32_t crc;        // 4
  uint16_t version;    // 2
  uint16_t seq;        // 2
  uint16_t min[4];     // 8
  uint16_t max[4];     // 8
  uint16_t center[4];  // 8
  uint16_t reserved;   // 2
};

static_assert(sizeof(CalibrationData) <= 48, "Calibration struct too large");
static_assert(sizeof(CalibrationData) <= 128, "Calibration struct too large");
static_assert((sizeof(CalibrationData) % 4) == 0, "Calibration struct should be 4-byte aligned in size");
