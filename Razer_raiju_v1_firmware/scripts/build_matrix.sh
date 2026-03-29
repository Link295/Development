#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$PROJECT_DIR/.." && pwd)"

OUTDIR="$PROJECT_DIR/.build"
LOGDIR="$PROJECT_DIR/.build_logs"

mkdir -p "$OUTDIR" "$LOGDIR"

# Prefer the repo-local arduino-cli if present; otherwise use whatever is on PATH.
ARDUINO_CLI="arduino-cli"
if [[ -x "$REPO_ROOT/.tools/arduino-cli/arduino-cli" ]]; then
  ARDUINO_CLI="$REPO_ROOT/.tools/arduino-cli/arduino-cli"
fi

build() {
  local name=$1
  local fqbn=$2
  local flags=$3

  echo "Building $name..."

  "$ARDUINO_CLI" compile \
    --fqbn "$fqbn" \
    --build-path "$OUTDIR/$name" \
    --build-property "build.extra_flags=$flags" \
    "$PROJECT_DIR/raiju_tmr_mcp4728" > "$LOGDIR/$name.log" 2>&1

  echo "Done: $name"
}

# Philhower (XIAO RP2040)
FQBN_PHIL="rp2040:rp2040:seeed_xiao_rp2040"

# Arduino-mbed (generic RP2040)
FQBN_MBED="arduino:mbed_rp2040:pico"

FLAGS0="-DDEBUG_LEVEL=0"
FLAGS1="-DDEBUG_LEVEL=1"
FLAGS2S="-DDEBUG_LEVEL=2 -DDEBUG_THROTTLE_MODE=0"
FLAGS2D="-DDEBUG_LEVEL=2 -DDEBUG_THROTTLE_MODE=1"

# Philhower builds
build phil_dbg0 "$FQBN_PHIL" "$FLAGS0"
build phil_dbg1 "$FQBN_PHIL" "$FLAGS1"
build phil_dbg2_safe "$FQBN_PHIL" "$FLAGS2S"
build phil_dbg2_drift "$FQBN_PHIL" "$FLAGS2D"

# mbed builds
build mbed_dbg0 "$FQBN_MBED" "$FLAGS0"
build mbed_dbg1 "$FQBN_MBED" "$FLAGS1"
build mbed_dbg2_safe "$FQBN_MBED" "$FLAGS2S"
build mbed_dbg2_drift "$FQBN_MBED" "$FLAGS2D"

echo "All builds completed successfully."
