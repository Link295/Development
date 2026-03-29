# Raiju V1 Stick Mod Firmware

Firmware for a stick/joystick modification setup that reads 4 analog axes on a RP2040 and outputs 4 analog voltages via an MCP4728 quad DAC to a Razer Raiju V1 controller PCB.

## Hardware / Components

- Seeed Studio XIAO RP2040 (microcontroller)
- MCP4728 DAC module (4× analoge output)
- TMR joysticks (2 sticks → 4 assen)
- Razer Raiju V1 controller (doel PCB)

## Build

Use the build matrix script from the repository root:

```bash
./Raiju_V1_Stick_mod_firmware/scripts/build_matrix.sh
```

Build outputs and logs are written under:

- `Raiju_V1_Stick_mod_firmware/.build/`
- `Raiju_V1_Stick_mod_firmware/.build_logs/`

## CI

A GitHub Actions workflow is included in `.github/workflows/build.yml` to compile the same build matrix on every push/PR.
