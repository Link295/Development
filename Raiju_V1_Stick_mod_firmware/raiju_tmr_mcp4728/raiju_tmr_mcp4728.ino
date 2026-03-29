#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <math.h>

#include "raiju_tmr_mcp4728_types.h"

// Optional persistence (depends on RP2040 core).
// - Arduino-mbed RP2040: use Mbed KVStore (kvstore_global_api)
// - Earle Philhower RP2040: use EEPROM emulation (EEPROM.begin/commit)
// If neither is available, we compile without persistence and use defaults each boot.
#ifndef __has_include
#define __has_include(x) 0
#endif
#if __has_include(<EEPROM.h>)
#include <EEPROM.h>
#define HAS_EEPROM 1
#else
#define HAS_EEPROM 0
#endif

#if __has_include(<kvstore_global_api/kvstore_global_api.h>) && (defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_MBED_RP2040))
#include <kvstore_global_api/kvstore_global_api.h>
#define HAS_MBED_KVSTORE 1
#else
#define HAS_MBED_KVSTORE 0
#endif

// Philhower core typically defines ARDUINO_ARCH_RP2040, but not ARDUINO_ARCH_MBED.
#if HAS_EEPROM && defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_MBED_RP2040)
#define HAS_PHILHOWER_EEPROM 1
#else
#define HAS_PHILHOWER_EEPROM 0
#endif

#if HAS_MBED_KVSTORE
// Keep keys very short to reduce overhead and avoid KVStore issues.
static constexpr const char *KV_KEY_SLOT0 = "/raiju0";
static constexpr const char *KV_KEY_SLOT1 = "/raiju1";
#endif

// -----------------------------------------------------------------------------
// Razer Raiju V1: TMR sticks -> RP2040 ADC -> MCP4728 DAC -> controller VRX/VRY
//
// Pins (per your spec):
//   Left  X -> A0, Left  Y -> A1
//   Right X -> A2, Right Y -> A3
//
// DAC channels:
//   A -> Left  X (VRX)
//   B -> Left  Y (VRY)
//   C -> Right X (VRX)
//   D -> Right Y (VRY)
//
// Requires Arduino library: "Adafruit MCP4728".
// -----------------------------------------------------------------------------

// Update rate: 1 kHz (1000 us). You can raise this (e.g., 500 us) if stable.
static constexpr uint32_t LOOP_PERIOD_US = 1000;

// I2C speed: 400 kHz is usually safe for MCP4728 modules.
static constexpr uint32_t I2C_CLOCK_HZ = 400000;

// ADC/DAC are both 12-bit.
static constexpr uint16_t ADC_MAX = 4095;
static constexpr uint16_t DAC_MAX = 4095;
static constexpr uint16_t DAC_CENTER = 2048;

// Deadzone around center (in ADC counts). 40 ~= ~1% of full scale.
static constexpr uint16_t DEFAULT_DEADZONE = 40;

// Low-pass smoothing factor (IIR):
//   filtered = filtered + alpha * (new - filtered)
// Larger alpha = less smoothing (lower latency). Typical: 0.15 .. 0.35
// You can set these differently for movement (left stick) vs aim (right stick).
static constexpr float DEFAULT_ALPHA_MOVE = 0.30f;
static constexpr float DEFAULT_ALPHA_AIM  = 0.20f;

// Boot-time center calibration duration.
static constexpr uint32_t CENTER_CAL_MS = 800;

// -----------------------------------------------------------------------------
// Min/Max learn calibration
// -----------------------------------------------------------------------------
// Duration for capturing min/max while user moves sticks to full extents.
static constexpr uint32_t MINMAX_CAL_MS = 2000;

// Optional margin applied after capture (counts in ADC domain).
// This helps avoid hard-clipping due to tiny overshoot/noise.
static constexpr uint16_t MINMAX_MARGIN = 20;

// Safety: ignore learned ranges with too-small usable span.
static constexpr uint16_t MIN_VALID_SPAN = 600;

// Trigger option A: a button held at boot.
// Set to a valid XIAO pin (e.g., D0/D1/...) if you wire a button to GND.
// Leave as -1 to disable this trigger.
static constexpr int CALIB_BUTTON_PIN = -1;
static constexpr bool CALIB_BUTTON_ACTIVE_LOW = true;

// Trigger option B: within the first 2 seconds after boot, if a large stick
// movement is detected (user wiggles sticks), enter calibration.
static constexpr bool ENABLE_MOVEMENT_TRIGGER = true;
static constexpr uint32_t MOVEMENT_TRIGGER_WINDOW_MS = 2000;
static constexpr uint16_t MOVEMENT_TRIGGER_THRESHOLD = 700; // ADC counts from center

// Output range limiting to avoid hitting the controller's rails (clipping).
// Default request: 200..3895 (still centers to ~2048).
static constexpr uint16_t OUTPUT_MIN = 200;
static constexpr uint16_t OUTPUT_MAX = 3895;
static_assert(OUTPUT_MIN < OUTPUT_MAX, "OUTPUT_MIN must be < OUTPUT_MAX");
static constexpr uint16_t OUTPUT_CENTER = (uint16_t)((OUTPUT_MIN + OUTPUT_MAX + 1) / 2);

// Optional serial debugging.
// Compile-time debug level:
//   0 = OFF (default)
//   1 = INFO (boot-only diagnostics)
//   2 = VERBOSE (extra boot details + periodic runtime prints)
//
// Set here or via build flags (e.g. -DDEBUG_LEVEL=1).
// #define DEBUG_LEVEL 1
// Backwards-compat: older builds used DEBUG_SERIAL.

#ifndef DEBUG_LEVEL
  #ifdef DEBUG_SERIAL
    #define DEBUG_LEVEL 1
  #else
    #define DEBUG_LEVEL 0
  #endif
#endif

#if DEBUG_LEVEL >= 1
  #define DEBUG_INFO(code) do { code } while (0)
#else
  #define DEBUG_INFO(code) do { } while (0)
#endif

#if DEBUG_LEVEL >= 2
  #define DEBUG_VERBOSE(code) do { code } while (0)
#else
  #define DEBUG_VERBOSE(code) do { } while (0)
#endif

#include "raiju_debug_serial.h"

// VERBOSE runtime debug throttle mode (compile-time):
//   0 = CATCH-UP SAFE (set timestamp to now)
//   1 = DRIFTLESS (+= interval)
// If Serial backpressure is detected, we automatically behave as SAFE for that tick.
#ifndef DEBUG_THROTTLE_MODE
  #define DEBUG_THROTTLE_MODE 1
#endif

Adafruit_MCP4728 mcp;

// -----------------------------------------------------------------------------
// Status LED (optional)
// -----------------------------------------------------------------------------
#ifdef LED_BUILTIN
static constexpr int STATUS_LED_PIN = LED_BUILTIN;
#else
static constexpr int STATUS_LED_PIN = -1;
#endif
static constexpr bool STATUS_LED_ACTIVE_LOW = false;

static inline void statusLedInit() {
  if (STATUS_LED_PIN < 0) return;
  pinMode((uint8_t)STATUS_LED_PIN, OUTPUT);
  digitalWrite((uint8_t)STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? HIGH : LOW);
}

static inline void statusLedSet(bool on) {
  if (STATUS_LED_PIN < 0) return;
  digitalWrite((uint8_t)STATUS_LED_PIN,
               on ? (STATUS_LED_ACTIVE_LOW ? LOW : HIGH) : (STATUS_LED_ACTIVE_LOW ? HIGH : LOW));
}

static CalStatus g_calStatus = CalStatus::None;

static inline void statusLedPulse(uint16_t onMs, uint16_t offMs = 0) {
  statusLedSet(true);
  delay(onMs);
  statusLedSet(false);
  if (offMs) delay(offMs);
}

static void statusLedBootIndicate(CalStatus st) {
  // Boot-only indications; uses short delays during setup.
  if (STATUS_LED_PIN < 0) return;
  switch (st) {
    case CalStatus::Valid:
      statusLedPulse(60);
      break;
    case CalStatus::FallbackUsed:
      statusLedPulse(60, 60);
      statusLedPulse(60);
      break;
    case CalStatus::None:
    default:
      // no pulse here; runtime will slow-blink
      break;
  }
}

static void statusLedUpdateRuntime() {
  // Non-blocking LED status patterns for normal runtime.
  static uint32_t nextMs = 0;
  static bool on = false;
  static uint8_t phase = 0;

  if (STATUS_LED_PIN < 0) return;
  const uint32_t now = millis();
  if ((int32_t)(now - nextMs) < 0) return;

  switch (g_calStatus) {
    case CalStatus::None:
      // Slow blink forever (1 Hz)
      on = !on;
      statusLedSet(on);
      nextMs = now + 500;
      break;

    case CalStatus::FallbackUsed:
      // Double-blink pattern every ~2s
      // phase: 0 ON, 1 OFF, 2 ON, 3 OFF (long)
      if (phase == 0) {
        statusLedSet(true);
        nextMs = now + 80;
      } else if (phase == 1) {
        statusLedSet(false);
        nextMs = now + 80;
      } else if (phase == 2) {
        statusLedSet(true);
        nextMs = now + 80;
      } else {
        statusLedSet(false);
        nextMs = now + 1760;
      }
      phase = (uint8_t)((phase + 1) & 3);
      break;

    case CalStatus::Valid:
    default:
      // LED off
      statusLedSet(false);
      nextMs = now + 1000;
      break;
  }
}

static AxisConfig axisCfg[4] = {
  // pin, invert, inMin, inMax, deadzone, alpha
  {A0, false, 0, ADC_MAX, DEFAULT_DEADZONE, DEFAULT_ALPHA_MOVE}, // Lx
  {A1, false, 0, ADC_MAX, DEFAULT_DEADZONE, DEFAULT_ALPHA_MOVE}, // Ly
  {A2, false, 0, ADC_MAX, DEFAULT_DEADZONE, DEFAULT_ALPHA_AIM }, // Rx
  {A3, false, 0, ADC_MAX, DEFAULT_DEADZONE, DEFAULT_ALPHA_AIM }, // Ry
};

static AxisState axisState[4] = {
  {DAC_CENTER, 0.0f},
  {DAC_CENTER, 0.0f},
  {DAC_CENTER, 0.0f},
  {DAC_CENTER, 0.0f},
};

static inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float clampF(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static void writeDacAll(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
  // Fast 4-channel write (most efficient).
  // If your Adafruit_MCP4728 version lacks fastWrite(), replace with setChannelValue().
  mcp.fastWrite(a, b, c, d);
}

// -----------------------------------------------------------------------------
// Persistent storage (dual-slot, seq + CRC32)
// -----------------------------------------------------------------------------

static constexpr uint32_t CAL_MAGIC = 0x524A4343u; // 'RJCC'
static constexpr uint16_t CAL_VERSION = 1;

// Byte-optimized calibration data.
// Target: <=48 bytes total, <=128 bytes per slot.
static_assert(sizeof(CalibrationData) <= 48, "Calibration struct too large");
static_assert(sizeof(CalibrationData) <= 128, "Calibration struct too large");
static_assert((sizeof(CalibrationData) % 4) == 0, "Calibration struct should be 4-byte aligned in size");

static uint32_t crc32_compute(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

static uint32_t calDataCrc(const CalibrationData &d) {
  // CRC is computed over everything except the crc field itself.
  // We start at version to exclude magic+crc and keep upgrades simple.
  // Note: CRC covers any trailing padding bytes too; writes are deterministic
  // because we always value-initialize (zero) the struct before filling fields.
  const size_t off = offsetof(CalibrationData, version);
  return crc32_compute((const uint8_t *)&d + off, sizeof(CalibrationData) - off);
}

static bool calDataValid(const CalibrationData &d) {
  if (d.magic != CAL_MAGIC) return false;
  if (d.version != CAL_VERSION) return false;
  if (d.crc != calDataCrc(d)) return false;
  for (int i = 0; i < 4; i++) {
    if (d.min[i] > ADC_MAX || d.max[i] > ADC_MAX) return false;
    if (d.min[i] > d.max[i]) return false;
    if (d.center[i] > ADC_MAX) return false;
  }
  return true;
}

static inline bool seqNewer(uint16_t a, uint16_t b) {
  // Wrap-safe comparison for monotonically increasing 16-bit sequence.
  return (uint16_t)(a - b) < 0x8000u;
}

static CalibrationData calDataFromCurrent(uint16_t seq) {
  CalibrationData d{};
  d.magic = CAL_MAGIC;
  d.version = CAL_VERSION;
  d.seq = seq;
  for (int i = 0; i < 4; i++) {
    d.min[i] = axisCfg[i].inMin;
    d.max[i] = axisCfg[i].inMax;
    d.center[i] = axisState[i].center;
  }
  d.reserved = 0;
  d.crc = calDataCrc(d);
  return d;
}

static bool storageReadSlot(int slot, CalibrationData &out);
static bool storageWriteSlot(int slot, const CalibrationData &rec);

static void storageBegin() {
#if HAS_PHILHOWER_EEPROM
  // Keep this minimal and deterministic.
  (void)EEPROM.begin(256);
#endif
}

static void storageEraseAll() {
#if HAS_MBED_KVSTORE
  // Prefer removing keys so "no calibration" is clearly represented.
  // If removal fails, fall back to writing invalid records.
  bool ok0 = false;
  bool ok1 = false;
  #if defined(MBED_SUCCESS)
    const int r0 = kv_remove(KV_KEY_SLOT0);
    const int r1 = kv_remove(KV_KEY_SLOT1);
    ok0 = (r0 == MBED_SUCCESS)
  #if defined(MBED_ERROR_ITEM_NOT_FOUND)
          || (r0 == MBED_ERROR_ITEM_NOT_FOUND)
  #endif
      ;
    ok1 = (r1 == MBED_SUCCESS)
  #if defined(MBED_ERROR_ITEM_NOT_FOUND)
          || (r1 == MBED_ERROR_ITEM_NOT_FOUND)
  #endif
      ;
  #endif

  if (!(ok0 && ok1)) {
    CalibrationData blank{};
    (void)storageWriteSlot(0, blank);
    (void)storageWriteSlot(1, blank);
  }
#elif HAS_PHILHOWER_EEPROM
  CalibrationData blank{};
  (void)storageWriteSlot(0, blank);
  (void)storageWriteSlot(1, blank);
#else
  // no-op
#endif
}

static bool storageReadSlot(int slot, CalibrationData &out) {
  if (slot < 0 || slot > 1) return false;
#if HAS_MBED_KVSTORE
  const char *key = (slot == 0) ? KV_KEY_SLOT0 : KV_KEY_SLOT1;
  size_t actual = 0;
  const int ret = kv_get(key, &out, sizeof(out), &actual);
  if (ret != MBED_SUCCESS || actual != sizeof(out)) {
    return false;
  }
  return true;
#elif HAS_PHILHOWER_EEPROM
  const int addr = (slot == 0) ? 0 : (int)sizeof(CalibrationData);
  EEPROM.get(addr, out);
  return true;
#else
  (void)out;
  return false;
#endif
}

static bool storageWriteSlot(int slot, const CalibrationData &rec) {
  if (slot < 0 || slot > 1) return false;
#if HAS_MBED_KVSTORE
  const char *key = (slot == 0) ? KV_KEY_SLOT0 : KV_KEY_SLOT1;
  const int ret = kv_set(key, &rec, sizeof(rec), 0);
  return ret == MBED_SUCCESS;
#elif HAS_PHILHOWER_EEPROM
  const int addr = (slot == 0) ? 0 : (int)sizeof(CalibrationData);
  EEPROM.put(addr, rec);
  return EEPROM.commit();
#else
  (void)rec;
  return false;
#endif
}

static bool storageLoadCal(CalibrationData &out, uint16_t &seqOut) {
  CalibrationData d0{}, d1{};
  const bool slot0Valid = storageReadSlot(0, d0) && calDataValid(d0);
  const bool slot1Valid = storageReadSlot(1, d1) && calDataValid(d1);

  int selectedSlot = -1;
  uint16_t selectedSeq = 0;
  if (slot0Valid && slot1Valid) {
    selectedSlot = seqNewer(d1.seq, d0.seq) ? 1 : 0;
    selectedSeq = (selectedSlot == 1) ? d1.seq : d0.seq;
  } else if (slot0Valid) {
    selectedSlot = 0;
    selectedSeq = d0.seq;
  } else if (slot1Valid) {
    selectedSlot = 1;
    selectedSeq = d1.seq;
  }

  // When DEBUG_LEVEL=0 the DEBUG_INFO() block compiles out; keep builds warning-free.
  (void)selectedSeq;

  DEBUG_INFO(
    // Boot-only visibility: print once per boot, never in the fast loop.
    static bool printedOnce = false;
    if (!printedOnce) {
      printedOnce = true;
      Serial.print("Slot0: ");
      Serial.print(slot0Valid ? "OK" : "FAIL");

      Serial.print(" | Slot1: ");
      Serial.print(slot1Valid ? "OK" : "FAIL");

      Serial.print(" | Selected: ");
      if (selectedSlot == -1) {
        Serial.print("N/A");
      } else {
        Serial.print(selectedSlot);
      }

      Serial.print(" | Seq: ");
      Serial.print(selectedSeq);

      Serial.print(" | Result: ");
      Serial.println((!slot0Valid && !slot1Valid) ? "DEFAULT" : "OK");
    }
  );

  if (!slot0Valid && !slot1Valid) return false;

  const CalibrationData *best = nullptr;
  if (slot0Valid && slot1Valid) {
    best = (selectedSlot == 1) ? &d1 : &d0;
  } else {
    best = slot1Valid ? &d1 : &d0;
  }
  out = *best;
  seqOut = best->seq;
  return true;
}

static bool storageSaveCal() {
  // Only write after explicit learn-mode or explicit reset; never from the fast loop.
  CalibrationData d0{}, d1{};
  bool have0 = storageReadSlot(0, d0) && calDataValid(d0);
  bool have1 = storageReadSlot(1, d1) && calDataValid(d1);

  int newestSlot = -1;
  uint16_t newestSeq = 0;
  if (have0 && have1) {
    newestSlot = seqNewer(d1.seq, d0.seq) ? 1 : 0;
    newestSeq = (newestSlot == 1) ? d1.seq : d0.seq;
  } else if (have0) {
    newestSlot = 0;
    newestSeq = d0.seq;
  } else if (have1) {
    newestSlot = 1;
    newestSeq = d1.seq;
  }

  const int targetSlot = (newestSlot == 0) ? 1 : 0;
  const uint16_t nextSeq = (uint16_t)(newestSlot >= 0 ? (uint16_t)(newestSeq + 1u) : 1u);
  const CalibrationData out = calDataFromCurrent(nextSeq);
  return storageWriteSlot(targetSlot, out);
}

static bool validateAxisRange(uint16_t inMin, uint16_t inMax) {
  if (inMin >= inMax) return false;
  if ((uint16_t)(inMax - inMin) < MIN_VALID_SPAN) return false;
  if (inMin > ADC_MAX || inMax > ADC_MAX) return false;
  return true;
}

static bool applyRangesOrDefaultsTracked(const uint16_t mins[4], const uint16_t maxs[4]) {
  bool fallbackUsed = false;
  for (int i = 0; i < 4; i++) {
    if (validateAxisRange(mins[i], maxs[i])) {
      axisCfg[i].inMin = mins[i];
      axisCfg[i].inMax = maxs[i];
    } else {
      axisCfg[i].inMin = 0;
      axisCfg[i].inMax = ADC_MAX;
      fallbackUsed = true;
    }
  }
  return fallbackUsed;
}

static bool applyLearnedRangesKeepCurrent(const uint16_t mins[4], const uint16_t maxs[4]) {
  bool anyValid = false;
  for (int i = 0; i < 4; i++) {
    if (validateAxisRange(mins[i], maxs[i])) {
      axisCfg[i].inMin = mins[i];
      axisCfg[i].inMax = maxs[i];
      anyValid = true;
    }
  }
  return anyValid;
}

static bool isCalButtonPressed() {
  if (CALIB_BUTTON_PIN < 0) return false;
  int v = digitalRead((uint8_t)CALIB_BUTTON_PIN);
  return CALIB_BUTTON_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

static float applyDeadzone(float x, float dz) {
  // x in [-1..1], dz in [0..1]
  if (fabsf(x) <= dz) return 0.0f;
  // Re-scale so output reaches 1.0 at edge
  float sign = (x < 0.0f) ? -1.0f : 1.0f;
  float ax = fabsf(x);
  float y = (ax - dz) / (1.0f - dz);
  return sign * clampF(y, 0.0f, 1.0f);
}

static float rawToNormalized(uint16_t raw, const AxisConfig &cfg, const AxisState &st) {
  // Clamp to axis range
  raw = clampU16(raw, cfg.inMin, cfg.inMax);

  // Failsafe: if center is outside range, treat as neutral.
  if (st.center <= cfg.inMin || st.center >= cfg.inMax) {
    return 0.0f;
  }

  // Separate scaling for negative and positive side so center can be off.
  int32_t delta = (int32_t)raw - (int32_t)st.center;
  if (delta == 0) return 0.0f;

  int32_t negSpan = (int32_t)st.center - (int32_t)cfg.inMin;
  int32_t posSpan = (int32_t)cfg.inMax - (int32_t)st.center;

  if (delta < 0) {
    if (negSpan <= 0) return -1.0f;
    return clampF((float)delta / (float)negSpan, -1.0f, 0.0f);
  }

  if (posSpan <= 0) return 1.0f;
  return clampF((float)delta / (float)posSpan, 0.0f, 1.0f);
}

static uint16_t normalizedToDac(float norm) {
  norm = clampF(norm, -1.0f, 1.0f);
  float u = (norm + 1.0f) * 0.5f; // 0..1
  float span = (float)((int32_t)OUTPUT_MAX - (int32_t)OUTPUT_MIN);
  uint16_t v = (uint16_t)lroundf((float)OUTPUT_MIN + u * span);
  return clampU16(v, OUTPUT_MIN, OUTPUT_MAX);
}

static void calibrateCenters() {
  uint32_t start = millis();
  uint32_t samples = 0;
  uint32_t sum[4] = {0, 0, 0, 0};

  while ((millis() - start) < CENTER_CAL_MS) {
    for (int i = 0; i < 4; i++) {
      uint16_t raw = (uint16_t)analogRead(axisCfg[i].pin);
      if (axisCfg[i].invert) raw = (uint16_t)(ADC_MAX - raw);
      raw = clampU16(raw, 0, ADC_MAX);
      sum[i] += raw;
    }
    samples++;
    // small delay yields stable ADC sampling without adding meaningful startup latency
    delayMicroseconds(250);
  }

  if (samples == 0) samples = 1;
  for (int i = 0; i < 4; i++) {
    axisState[i].center = (uint16_t)(sum[i] / samples);
    axisState[i].filteredNorm = 0.0f;
  }

  DEBUG_INFO(
    Serial.println("Center calibration done:");
    Serial.print("Lx="); Serial.print(axisState[0].center);
    Serial.print(" Ly="); Serial.print(axisState[1].center);
    Serial.print(" Rx="); Serial.print(axisState[2].center);
    Serial.print(" Ry="); Serial.println(axisState[3].center);
  );
}

static uint16_t readAxisRawPostInvert(int axisIndex) {
  uint16_t raw = (uint16_t)analogRead(axisCfg[axisIndex].pin);
  if (axisCfg[axisIndex].invert) raw = (uint16_t)(ADC_MAX - raw);
  return clampU16(raw, 0, ADC_MAX);
}

static bool factoryResetRequestedAndHandled() {
  if (CALIB_BUTTON_PIN < 0) return false;
  if (!isCalButtonPressed()) return false;

  // Long hold at boot: >= 3s triggers factory reset.
  const uint32_t start = millis();
  uint32_t nextBlink = start;
  bool ledOn = false;

  while (isCalButtonPressed()) {
    const uint32_t now = millis();
    // Long blink while waiting: ~1 Hz
    if ((int32_t)(now - nextBlink) >= 0) {
      nextBlink += 500;
      ledOn = !ledOn;
      statusLedSet(ledOn);
    }
    // Keep outputs stable.
    writeDacAll(OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER);
    if ((uint32_t)(now - start) >= 3000) {
      break;
    }
  }

  statusLedSet(false);
  const uint32_t heldMs = (uint32_t)(millis() - start);
  if (heldMs < 3000) {
    // Not a factory reset hold.
    return false;
  }

  // Reset active: long blink already shown; perform erase.
  storageEraseAll();
  for (int i = 0; i < 4; i++) {
    axisCfg[i].inMin = 0;
    axisCfg[i].inMax = ADC_MAX;
  }
  g_calStatus = CalStatus::None;

  // Reset completed: short solid.
  statusLedPulse(180);

  // Wait for release so we don't immediately re-enter learn-mode.
  while (isCalButtonPressed()) {
    writeDacAll(OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER);
  }
  return true;
}

static void runMinMaxLearn() {
  // LED: fast blink during capture.
  uint32_t nextBlinkMs = millis();
  bool ledOn = false;
  statusLedSet(false);

  // Keep DAC stable during calibration.
  uint32_t lastDacUs = micros();

  uint16_t learnedMin[4];
  uint16_t learnedMax[4];
  for (int i = 0; i < 4; i++) {
    uint16_t r = readAxisRawPostInvert(i);
    learnedMin[i] = r;
    learnedMax[i] = r;
  }

  uint32_t start = millis();
  while ((millis() - start) < MINMAX_CAL_MS) {
    uint32_t nowMs = millis();
    if ((int32_t)(nowMs - nextBlinkMs) >= 0) {
      nextBlinkMs += 50; // 10 Hz blink (50ms half-period)
      ledOn = !ledOn;
      statusLedSet(ledOn);
    }

    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastDacUs) >= LOOP_PERIOD_US) {
      lastDacUs += LOOP_PERIOD_US;
      writeDacAll(OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER);
    }

    for (int i = 0; i < 4; i++) {
      uint16_t r = readAxisRawPostInvert(i);
      if (r < learnedMin[i]) learnedMin[i] = r;
      if (r > learnedMax[i]) learnedMax[i] = r;
    }
    // No delay needed; keep sampling fast.
  }

  statusLedSet(false);

  // Apply margin and clamp.
  for (int i = 0; i < 4; i++) {
    uint32_t mn = learnedMin[i];
    uint32_t mx = learnedMax[i];
    mn = (mn > MINMAX_MARGIN) ? (mn - MINMAX_MARGIN) : 0;
    mx = (mx + MINMAX_MARGIN <= ADC_MAX) ? (mx + MINMAX_MARGIN) : ADC_MAX;
    learnedMin[i] = (uint16_t)mn;
    learnedMax[i] = (uint16_t)mx;
  }

  // Only apply valid learned ranges; keep existing ones if an axis wasn't moved.
  bool anyValid = applyLearnedRangesKeepCurrent(learnedMin, learnedMax);
  bool saved = false;
  if (anyValid) {
    saved = storageSaveCal();
  }

  // LED feedback: solid if saved, otherwise slow blink a few times.
  if (anyValid && saved) {
    statusLedPulse(250);
  } else if (anyValid && !saved) {
    for (int i = 0; i < 3; i++) {
      statusLedSet(true);
      delay(150);
      statusLedSet(false);
      delay(150);
    }
  } else {
    // Nothing learned (no axis moved enough) - gentle indication.
    statusLedPulse(80);
  }

  DEBUG_INFO(
    Serial.println("Min/Max calibration applied:");
    for (int i = 0; i < 4; i++) {
      Serial.print(i);
      Serial.print(" min="); Serial.print(axisCfg[i].inMin);
      Serial.print(" max="); Serial.println(axisCfg[i].inMax);
    }
  );
}

static bool movementTriggerDetected() {
  if (!ENABLE_MOVEMENT_TRIGGER) return false;
  uint32_t start = millis();
  uint32_t lastSampleUs = micros();
  while ((millis() - start) < MOVEMENT_TRIGGER_WINDOW_MS) {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastSampleUs) < LOOP_PERIOD_US) {
      continue;
    }
    lastSampleUs += LOOP_PERIOD_US;
    for (int i = 0; i < 4; i++) {
      uint16_t r = readAxisRawPostInvert(i);
      uint16_t c = axisState[i].center;
      uint16_t d = (r > c) ? (uint16_t)(r - c) : (uint16_t)(c - r);
      if (d >= MOVEMENT_TRIGGER_THRESHOLD) {
        return true;
      }
    }
    // Keep controller stable during the trigger window.
    writeDacAll(OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER);
  }
  return false;
}

void setup() {
  #if DEBUG_LEVEL >= 1
    Serial.begin(115200);
    delay(50);
    Serial.println("Raiju TMR -> MCP4728 starting...");
    Serial.print("CalibrationData size: ");
    Serial.println(sizeof(CalibrationData));
  #endif

  statusLedInit();
  storageBegin();

  if (CALIB_BUTTON_PIN >= 0) {
    pinMode((uint8_t)CALIB_BUTTON_PIN, INPUT_PULLUP);
  }

  analogReadResolution(12);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  if (!mcp.begin()) {
    DEBUG_INFO(
      Serial.println("ERROR: MCP4728 not found on I2C.");
    );
    // Fail safe: do not run a loop writing unknown values.
    while (true) {
      delay(1000);
    }
  }

  // Initialize DAC at center (neutral stick) as early as possible.
  writeDacAll(OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER);

  // Factory reset: hold calibration button >=3s at boot.
  const bool didFactoryReset = factoryResetRequestedAndHandled();

  // Load previously learned min/max ranges if available.
  // If missing/invalid, defaults stay at 0..4095.
  {
    CalibrationData d{};
    uint16_t seq = 0;
    if (storageLoadCal(d, seq)) {
      const bool fallback = applyRangesOrDefaultsTracked(d.min, d.max);
      g_calStatus = fallback ? CalStatus::FallbackUsed : CalStatus::Valid;
    }
  }

  statusLedBootIndicate(g_calStatus);

  calibrateCenters();

  // Min/Max learn is explicit (not every boot):
  // - Option A: button held at boot
  // - Option B: within the first 2 seconds, detect large stick movement
  bool doLearn = (!didFactoryReset) && isCalButtonPressed();
  if (!doLearn && !didFactoryReset) {
    doLearn = movementTriggerDetected();
  }

  if (doLearn) {
    DEBUG_INFO(
      Serial.println("Entering min/max learn mode (2s)...");
    );
    runMinMaxLearn();
    // After calibration (sticks were moving), re-calibrate center so neutral is exact.
    calibrateCenters();
    writeDacAll(OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER);

    // Update calibration status after potential save.
    CalibrationData d{};
    uint16_t seq = 0;
    if (storageLoadCal(d, seq)) {
      const bool fallback = applyRangesOrDefaultsTracked(d.min, d.max);
      g_calStatus = fallback ? CalStatus::FallbackUsed : CalStatus::Valid;
    } else {
      g_calStatus = CalStatus::None;
    }
  }

  DEBUG_VERBOSE(
    Serial.println("Boot config:");
    for (int i = 0; i < 4; i++) {
      Serial.print("Axis "); Serial.print(i);
      Serial.print(" center="); Serial.print(axisState[i].center);
      Serial.print(" inMin="); Serial.print(axisCfg[i].inMin);
      Serial.print(" inMax="); Serial.print(axisCfg[i].inMax);
      Serial.print(" deadzone="); Serial.print(axisCfg[i].deadzone);
      Serial.print(" alpha="); Serial.println(axisCfg[i].alpha, 3);
    }
  );
}

void loop() {
  // Fixed-rate scheduler using micros() (no delays / non-blocking).
  // Uses signed time-delta so wraparound stays safe.
  static uint32_t nextTickUs = 0;
  uint32_t now = micros();
  if (nextTickUs == 0) {
    nextTickUs = now; // run immediately on first iteration
  }
  if ((int32_t)(now - nextTickUs) < 0) {
    return;
  }
  nextTickUs += LOOP_PERIOD_US;
  // If we fell behind significantly, resync to avoid a long catch-up backlog.
  if ((int32_t)(now - nextTickUs) > (int32_t)(LOOP_PERIOD_US * 5)) {
    nextTickUs = now + LOOP_PERIOD_US;
  }

  uint16_t dacOut[4];
  uint16_t adcRaw[4];

#if DEBUG_LEVEL >= 2
  const uint32_t tLoopStartUs = micros();
  // Scheduler lateness (how late we started vs the nominal schedule).
  // Note: nextTickUs has already been advanced by LOOP_PERIOD_US at this point.
  const uint32_t scheduledThisTickUs = nextTickUs - LOOP_PERIOD_US;
  const uint32_t lateStartUs = (uint32_t)((int32_t)(tLoopStartUs - scheduledThisTickUs) > 0 ? (tLoopStartUs - scheduledThisTickUs) : 0);
#endif

  // Runtime soft recalibration: long-press calibration button (>2s) to re-run learn-mode.
  static uint32_t btnStartMs = 0;
  static bool btnWasPressed = false;
  static bool btnFired = false;
  if (CALIB_BUTTON_PIN >= 0) {
    const bool pressed = isCalButtonPressed();
    const uint32_t ms = millis();
    if (pressed && !btnWasPressed) {
      btnWasPressed = true;
      btnFired = false;
      btnStartMs = ms;
    }
    if (!pressed) {
      btnWasPressed = false;
      btnFired = false;
    }
    if (pressed && !btnFired && (uint32_t)(ms - btnStartMs) >= 2000) {
      btnFired = true;

      DEBUG_INFO(
        Serial.println("Runtime long-press: entering min/max learn mode...");
      );

      runMinMaxLearn();
      calibrateCenters();
      writeDacAll(OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER, OUTPUT_CENTER);

      CalibrationData d{};
      uint16_t seq = 0;
      if (storageLoadCal(d, seq)) {
        const bool fallback = applyRangesOrDefaultsTracked(d.min, d.max);
        g_calStatus = fallback ? CalStatus::FallbackUsed : CalStatus::Valid;
      } else {
        g_calStatus = CalStatus::None;
      }
    }
  }

  // --- ADC read (capture all axes) ---
#if DEBUG_LEVEL >= 2
  const uint32_t tAdcStartUs = micros();
#endif
  for (int i = 0; i < 4; i++) {
    uint16_t raw = (uint16_t)analogRead(axisCfg[i].pin);
    if (axisCfg[i].invert) raw = (uint16_t)(ADC_MAX - raw);
    adcRaw[i] = clampU16(raw, 0, ADC_MAX);
  }
#if DEBUG_LEVEL >= 2
  const uint32_t tAdcEndUs = micros();
#endif

  // --- Processing (mapping / deadzone / smoothing) ---
#if DEBUG_LEVEL >= 2
  const uint32_t tProcStartUs = micros();
#endif
  for (int i = 0; i < 4; i++) {
    float norm = rawToNormalized(adcRaw[i], axisCfg[i], axisState[i]);

    // Failsafe: protect against NaN/Inf.
    if (!isfinite(norm)) {
      norm = 0.0f;
    }

    // Deadzone in normalized space
    float dz = (float)axisCfg[i].deadzone / (float)ADC_MAX;
    norm = applyDeadzone(norm, dz);

    // Center stability requirement:
    // If we're inside the deadzone, force an exact neutral output (no decay drift).
    if (norm == 0.0f) {
      axisState[i].filteredNorm = 0.0f;
    } else {
      // Low-pass smoothing
      float alpha = axisCfg[i].alpha;
      alpha = clampF(alpha, 0.0f, 1.0f);
      axisState[i].filteredNorm = axisState[i].filteredNorm + alpha * (norm - axisState[i].filteredNorm);
    }

    dacOut[i] = normalizedToDac(axisState[i].filteredNorm);
  }

#if DEBUG_LEVEL >= 2
  const uint32_t tProcEndUs = micros();
#endif

  // Channel mapping per spec:
  // A=Lx, B=Ly, C=Rx, D=Ry
#if DEBUG_LEVEL >= 2
  const uint32_t tI2cStartUs = micros();
#endif
  writeDacAll(dacOut[0], dacOut[1], dacOut[2], dacOut[3]);
#if DEBUG_LEVEL >= 2
  const uint32_t tI2cEndUs = micros();
  const uint32_t tLoopEndUs = micros();

  const uint32_t adcUs = tAdcEndUs - tAdcStartUs;
  const uint32_t procUs = tProcEndUs - tProcStartUs;
  const uint32_t i2cUs = tI2cEndUs - tI2cStartUs;
  const uint32_t totalUs = tLoopEndUs - tLoopStartUs;

  static uint32_t perfWindowStartUs = 0;
  static uint32_t perfLoops = 0;
  static uint32_t maxAdcUs = 0;
  static uint32_t maxProcUs = 0;
  static uint32_t maxI2cUs = 0;
  static uint32_t maxTotalUs = 0;
  static uint32_t maxLateUs = 0;
  static uint32_t overrunCount = 0;
  static uint32_t missDeadlineCount = 0;
  static uint32_t i2cSlowCount = 0;

  if (perfWindowStartUs == 0) perfWindowStartUs = tLoopStartUs;
  perfLoops++;
  if (adcUs > maxAdcUs) maxAdcUs = adcUs;
  if (procUs > maxProcUs) maxProcUs = procUs;
  if (i2cUs > maxI2cUs) maxI2cUs = i2cUs;
  if (totalUs > maxTotalUs) maxTotalUs = totalUs;
  if (lateStartUs > maxLateUs) maxLateUs = lateStartUs;

  if (totalUs > LOOP_PERIOD_US) {
    overrunCount++;
  }
  if (lateStartUs > LOOP_PERIOD_US) {
    missDeadlineCount++;
  }
  if (i2cUs > 600) {
    i2cSlowCount++;
  }

  // Periodic performance report (VERBOSE only).
  static constexpr uint32_t PERF_REPORT_INTERVAL_US = 1000000; // 1s
  if ((uint32_t)(tLoopEndUs - perfWindowStartUs) >= PERF_REPORT_INTERVAL_US) {
    const uint32_t windowUs = tLoopEndUs - perfWindowStartUs;
    const uint32_t hz = (windowUs > 0) ? (uint32_t)((perfLoops * 1000000ull) / windowUs) : 0;
    if (debugSerialCanWrite(32)) {
      DEBUG_VERBOSE(
        Serial.print("PERF hz="); Serial.print(hz);
        Serial.print(" adcMaxUs="); Serial.print(maxAdcUs);
        Serial.print(" procMaxUs="); Serial.print(maxProcUs);
        Serial.print(" i2cMaxUs="); Serial.print(maxI2cUs);
        Serial.print(" totalMaxUs="); Serial.print(maxTotalUs);
        Serial.print(" lateMaxUs="); Serial.print(maxLateUs);
        Serial.print(" overrun="); Serial.print(overrunCount);
        Serial.print(" miss>"); Serial.print(LOOP_PERIOD_US);
        Serial.print("us="); Serial.print(missDeadlineCount);
        Serial.print(" i2cSlow="); Serial.print(i2cSlowCount);
        Serial.print(" serialBP="); Serial.println(g_serialBackpressureCount);
      );
    } else {
      g_serialBackpressureCount++;
    }

    perfWindowStartUs = tLoopEndUs;
    perfLoops = 0;
    maxAdcUs = maxProcUs = maxI2cUs = maxTotalUs = maxLateUs = 0;
    overrunCount = missDeadlineCount = i2cSlowCount = 0;
    g_serialBackpressureCount = 0;
  }
#endif

  statusLedUpdateRuntime();

#if DEBUG_LEVEL >= 2
  // Non-blocking, micros-based throttle. No timers/branches exist when DEBUG_LEVEL < 2.
  static uint32_t lastDebugUs = 0;
  static constexpr uint32_t DEBUG_INTERVAL_US = 200000; // 200 ms
  const uint32_t nowUs = micros();
  if (lastDebugUs == 0) {
    // First run: align to now to avoid catch-up spam.
    lastDebugUs = nowUs;
  }
  if ((uint32_t)(nowUs - lastDebugUs) >= DEBUG_INTERVAL_US) {
    // Optional backpressure guard to reduce UART/USB jitter.
    const bool canWrite = debugSerialCanWrite(32);

    if (canWrite) {
      #if DEBUG_THROTTLE_MODE == 1
        // DRIFTLESS: fixed cadence when healthy; but avoid bursting after long stalls.
        if ((uint32_t)(nowUs - lastDebugUs) >= (DEBUG_INTERVAL_US * 4u)) {
          lastDebugUs = nowUs;
        } else {
          lastDebugUs += DEBUG_INTERVAL_US;
        }
      #else
        // CATCH-UP SAFE: set to now.
        lastDebugUs = nowUs;
      #endif

      DEBUG_VERBOSE(
        Serial.print("ADC: ");
        Serial.print(adcRaw[0]); Serial.print(",");
        Serial.print(adcRaw[1]); Serial.print(",");
        Serial.print(adcRaw[2]); Serial.print(",");
        Serial.print(adcRaw[3]);

        Serial.print(" | DAC: ");
        Serial.print(dacOut[0]); Serial.print(",");
        Serial.print(dacOut[1]); Serial.print(",");
        Serial.print(dacOut[2]); Serial.print(",");
        Serial.println(dacOut[3]);
      );
    } else {
      // Serial backpressure: behave as SAFE for this tick.
      lastDebugUs = nowUs;
      g_serialBackpressureCount++;
    }
  }
#endif
}
