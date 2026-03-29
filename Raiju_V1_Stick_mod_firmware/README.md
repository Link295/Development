# Raiju V1 Stick Mod Firmware

Dit project implementeert een custom firmware oplossing om TMR (magnetische) joysticks te gebruiken op een controller die normaal analoge potentiometer-input verwacht (Razer Raiju V1).

De firmware draait op een Seeed Studio XIAO RP2040 en vertaalt joystick input naar analoge spanningen via een MCP4728 quad DAC.

## 🎯 Project overview

TMR joysticks leveren analoge assen (4 totaal). De RP2040 leest die via de ADC, past filtering/kalibratie toe, en genereert vervolgens vier analoge uitgangen via de MCP4728 DAC die je kan aansluiten op de VRX/VRY inputs van de Raiju V1 PCB.

## 🧩 Hardware

- Microcontroller: Seeed Studio XIAO RP2040
- DAC: MCP4728 (quad-channel DAC)
- Input: 2× TMR joysticks (4 assen totaal)
- Doel: Razer Raiju V1 controller PCB

## ⚙️ Architectuur

TMR Joysticks → RP2040 (ADC + processing) → MCP4728 (I2C DAC) → Controller (VRX/VRY inputs)

## 🔧 Features

- 4× ADC input (12-bit)
- Deadzone + snap-to-center
- IIR smoothing (per stick tuning)
- Full-range DAC output (clamped)
- Min/max calibration (learn mode)
- Dual-slot persistent storage (CRC32, power-loss safe)
- Debug system (DEBUG_LEVEL 0/1/2)
- VERBOSE runtime debug met throttling

## 🛠️ Build & tooling

- Arduino IDE + RP2040 cores (mbed & Philhower)
- Adafruit MCP4728 library
- arduino-cli build system
- Build matrix script (8 build configs)
- GitHub Actions CI

### Build matrix (1 command)

Run vanaf de repository root:

```bash
./Raiju_V1_Stick_mod_firmware/scripts/build_matrix.sh
```

Outputs en logs komen onder:

- Raiju_V1_Stick_mod_firmware/.build/
- Raiju_V1_Stick_mod_firmware/.build_logs/

## 🚀 Usage (high-level)

1. Flash firmware naar de XIAO RP2040
2. Verbind TMR sticks met de ADC pins
3. Verbind MCP4728 met I2C
4. Verbind DAC outputs met controller VRX/VRY
5. Start calibration (learn mode)
6. Gebruik controller normaal

## ⚠️ Notes

- Firmware is ontworpen voor lage latency (≈ 1 kHz loop)
- Storage is geoptimaliseerd voor RP2040 flash constraints
- Debug kan volledig uitgeschakeld worden (zero overhead)

## 📌 Status

Stable – ready for hardware testing and tuning

##