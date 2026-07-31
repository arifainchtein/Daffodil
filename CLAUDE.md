# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Daffodil is ESP32 firmware for an agricultural IoT sensor node. It monitors water troughs, septic tanks, flow meters, and environmental sensors, transmitting data via LoRa and uploading to the Digital Stables cloud service over WiFi. The device is solar-powered and uses deep sleep to conserve energy.

Target board: **ESP32 FireBeetle32** (`esp32.esp32.firebeetle32`)

## Build & Flash

There is no CLI build system — the firmware is compiled and uploaded via **Arduino IDE**.

**Flash the LittleFS filesystem** (web app files) to the device:
```bash
./uploadData.sh
# Uses esptool, port /dev/ttyUSB0, baud 921600, flash address 0x00290000
```

**Update web app files** in the `data/` directory from the companion web project:
```bash
./updateData.sh
# Copies from /home/ari/Data/DigitalStables/DeviceWebsites/daffodilwebapp/
```

The `data/` directory is the LittleFS filesystem image — it contains the HTML/JS/CSS web app served by the device over WiFi.

## Hardware

| Pin | Function |
|-----|----------|
| 4 | RTC 1Hz clock output (interrupt) |
| 12/13/14 | SPI MISO/MOSI/SCK (LoRa) |
| 15/16/17 | LoRa CS/RESET/DI0 |
| 18 | Ultrasonic TRIGGER / Flow meter 1 interrupt — **shared pin** |
| 19 | WS2812 LED strip (15 LEDs) |
| 23 | LED power control (MOSFET gate) |
| 25/35 | TPL5010 watchdog DONE/WAKE |
| 26 | External power switch (held LOW during deep sleep) |
| 27 | DS18B20 temperature (OneWire) |
| 33 | Ultrasonic ECHO / Flow meter 2 interrupt — **shared pin** |
| 34 | Op mode switch (input only) |
| 36 | RTC coin cell voltage (input only) |

I2C devices:
- `0x41` — INA219 current sensor
- `0x48` — ADS1115 ADC
- `0x51` — PCF8563T RTC
- `0x23` — BH1750 light sensor
- `0x44` — SHT humidity/temperature sensor

LoRa: SX127x at 433MHz, SF9, BW 125kHz, TX power 5dBm.

## Architecture

All firmware logic is in the single file `Daffodil.ino`. The main loop is driven by a **1Hz interrupt from the PCF8563T RTC** (pin 4) via the `clockTick` ISR, which sets `clockTicked = true`. The loop body only does meaningful work when `clockTicked` is set.

**Custom libraries** (installed in the Arduino libraries path, not in this repo):
- `PowerManager` — solar power budget, LoRa TX safety, optimal sleep time calculation
- `SolarInfo` — solar position and irradiance for the configured lat/long/altitude
- `PCF8563TimeManager` — RTC read/write, time-in-seconds conversion
- `Esp32SecretManager` — reads device config (name, location, secrets) from NVS/flash; generates and validates TOTP codes
- `DaffodilWifiManager` — WiFi station/AP management, HTTP upload to Digital Stables
- `DataManager` — LittleFS storage and retrieval of `DigitalStablesData` records
- `WeatherForecastManager` — downloads OpenWeatherMap forecasts, feeds into SolarInfo
- `ErrorManager` — tracks I2C and LoRa errors

**Core data structure**: `DigitalStablesData` (from `DigitalStablesData.h`) is the central struct transmitted via LoRa, uploaded via WiFi, and stored to flash. All sensor readings are written into the global `digitalStablesData` instance.

**Operating mode** is determined at startup by reading an analog voltage from ADS1115 channel 2 (the config switch). The voltage maps to one of ~32 positions which select the operating mode (e.g. `DAFFODIL_WATER_TROUGH`, `DAFFODIL_SCEPTIC_TANK`, `FUN_1_FLOW`, `VOLTAGE_MONITOR`, etc.) and whether solar power management is active (`usingSolarPower`).

**Display**: 15 WS2812 LEDs cycle through 5 display states every 2 seconds (controlled by `viewTimer`): temperature → tank/trough level → internet status → LoRa TX → error status. LED power is gated by pin 23.

**Sleep**: `goToSleep()` reads sensors, stores a final record, sends a final LoRa message, shuts down WiFi/BT/peripherals, holds pin 23 and 26 LOW via `gpio_hold_en()`, then calls `esp_deep_sleep_start()`. Sleep duration is calculated by `PowerManager` based on solar conditions.

**LoRa**: Uses RSSI-based channel assessment (`performCAD()`) before transmitting. Receives `DigitalStablesData`, `RequestCommand`, and `WeatherForecastUpdate` packet types, distinguished by packet size. All packets are authenticated with a TOTP code and XOR checksum.

## Serial Commands

The device accepts commands over Serial (115200 baud) for diagnostics:

| Command | Description |
|---------|-------------|
| `Ping` | Responds `Ok-Ping` |
| `debug#1` / `debug#0` | Enable/disable debug serial output |
| `goToSleep` | Force deep sleep immediately |
| `SetTroughParameters#height#min#max` | Set ultrasonic trough thresholds |
| `printCurrentDSDData` | Print current sensor readings |
| `printCSWData` | Print config switch state and operating mode |
| `exportDSDCSV` | Dump stored records as CSV |
| `GenerateDSDReport#1` | Export CSV and optionally clear data |
| `clearAllDSDData` | Erase all stored records |
| `SetTime#...` | Set RTC time |

## Known Issues

- Pins 18 and 33 are shared between the ultrasonic sonar and flow meter interrupts. When `FUN_1_FLOW` or `FUN_2_FLOW` modes are active, sonar readings will falsely trigger the flow meter ISR.
- `sendMessage1()`, `performCAD1()`, `performCADOld()`, `goToSleepold()`, and `readRTCBattery()` are dead code — superseded functions never called.
- `debug` is a global `bool` (line 73). Setting it `false` via the serial `debug#0` command silences all diagnostic output.
