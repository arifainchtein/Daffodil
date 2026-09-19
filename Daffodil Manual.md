# Daffodil Manual

*Complete reference: configuration switch, per-device calibration, power management, and LED
status display. Supersedes "Daffodil Power Management and Configuration.md". Last updated
2026-09-03.*

---

## Table of Contents

1. [Overview](#1-overview)
2. [Configuration Switch — Function Modes](#2-configuration-switch--function-modes)
3. [Per-Device Calibration](#3-per-device-calibration)
4. [Power Management](#4-power-management)
5. [LED Status Display Guide](#5-led-status-display-guide)
6. [Serial Command Reference](#6-serial-command-reference)
7. [Troubleshooting](#7-troubleshooting)
8. [Known Limitations / Roadmap](#8-known-limitations--roadmap)

---

## 1. Overview

Daffodil is a solar/battery-powered field sensor unit built around an ESP32 (FireBeetle32) main
board ("Wally") with a sensor interface shield ("Daffodil") stacked on top. It monitors water
troughs, septic tanks, flow meters, and tank levels, reporting over WiFi to the Digital Stables
cloud, and over LoRa (433MHz) to other units on site (e.g. Annabelle for weather forecast
relay, or a LoRa base station like Paula).

Configuration is entirely physical: a **5-position DIP switch** (the "config switch" or CSW),
read at power-on as an analog voltage through a resistor ladder into the ADS1115 ADC.

- **Switches 1–4** select the device's **function** — what it measures and reports.
- **Switch 5** selects the device's **reporting mode** — see §2.2. It does not change the
  function.

No other configuration is required to get a unit reporting once WiFi/LoRa credentials are set —
but a **one-time per-device calibration** (§3) is required for the switch itself to decode
reliably, and is easy to overlook on a hand-flashed unit (the Factory tool does it
automatically — see §3.2).

## 2. Configuration Switch — Function Modes

### 2.1 Function table (switches 1–4)

| Switches 1–4 | Function | Status |
|---|---|---|
| `0000` | 1 Flow Sensor | Available |
| `1000` | 2 Flow Sensors | Available |
| `0100` | 1 Flow Sensor + 1 Tank | Available |
| `1100` | 1 Tank | Available |
| `0010` | 2 Tanks | Available |
| `1010` | Septic Tank | Available |
| `0110` | Water Trough | Available |
| `1110` | Water Trough + Tank | **Tank half only** — see note below and §8 |
| `0011` | 2 Water Troughs | **Coming soon** — sensor hardware not yet shipped |
| all other combinations | — | Reserved / not yet in use |

Firmware-internal names, for reference when reading debug output or this document's other
sections: `FUN_1_FLOW`, `FUN_2_FLOW`, `FUN_1_FLOW_1_TANK`, `FUN_1_TANK`, `FUN_2_TANK`,
`DAFFODIL_SCEPTIC_TANK`, `DAFFODIL_WATER_TROUGH`, `DAFFODIL_WATER_TROUGH_TANK1`,
`DAFFODIL_2_WATER_TROUGH`.

**Screw terminal wiring**, for the flow/tank sensor slots referenced throughout this manual:

| Terminal | GPIO | Used for |
|---|---|---|
| Sensor 1 | pin 18 | Flow meter 1 (interrupt pulse count), or Tank 1 pressure (jumpered to ADS1115) |
| Sensor 2 | pin 33 | Flow meter 2 (interrupt pulse count), or Tank 2 pressure (jumpered to ADS1115) |

`2 Water Troughs` (`0011`) has its switch position and firmware constant assigned, but sensor-
reading and LED-display code isn't implemented yet — pending the UART ultrasonic sensor
hardware.

`Water Trough + Tank` (`1110`) currently only reports the **tank** half. In this mode the sonar's
trigger/echo wiring collides with Sensor 1 (pin 18), which this mode already uses for tank
pressure — so the trough reading is pinned to a `-99` sentinel until a real single-wire ultrasonic
driver replaces the current trigger/echo one for this mode specifically. See §5.3 and §8 for what
this looks like on the LED display and the tracked fix.

### 2.2 Switch 5 — Reporting Mode

Switch 5 is **not** "solar attached or not." It's a trade-off between **how often the unit
checks in** and **how long the battery lasts**:

- **ON — Battery Saver.** The unit paces itself against its solar-charging schedule: it sleeps
  more and reports less often, trading frequency for battery life. Right setting for most
  unattended, solar-charged deployments (e.g. a trough level sensor) that don't need
  minute-by-minute updates.
- **OFF — Frequent Reports.** The unit reports more often and doesn't wait on the solar
  schedule, giving closer-to-real-time readings (e.g. active flow monitoring). Uses more
  battery — best when the unit has reliable charging, or you're accepting shorter battery life
  for more frequent data.

Either way, the unit still protects its battery from over-discharging (§4.3) — the difference
is purely about reporting frequency, not battery safety.

## 3. Per-Device Calibration

### 3.1 Why CSW calibration is necessary

The switch ladder's pull-up resistor is fed by `DEVICE_POWER` (the `V50` rail through a load
switch) — not the panel/USB input voltage the firmware measures on another ADC channel. On
battery power, `V50` comes from a boost converter fed by the battery, and its exact level varies
from unit to unit (confirmed empirically: two different battery units showed a consistent ~13%
difference in raw switch readings across every position). Since the ADS1115 measures an absolute
voltage, every raw switch reading scales with whatever `V50` actually is on that specific unit.

There's no spare ADC channel to measure `V50` directly, so the fix is a one-time **per-device
scale calibration** stored in flash, rather than one fixed threshold table for every unit.

### 3.2 Calibration procedure

Run this once per device at install time, and again any time the battery is swapped for a
substantially different one:

1. Set the DIP switches to `00000` (all off, no solar). **Double-check the switches physically
   before proceeding** — arming the calibration at the wrong position captures the wrong
   reference and silently corrupts the whole table. This is the single most common way to get
   this step wrong.
2. Power on / reset and let the device boot normally.
3. Send the serial command `CalibrateCSWReference`. This does **not** read the switch itself —
   it only arms a flag in flash. Sending it any time after the device has finished booting is
   fine; it doesn't need to be timed precisely, since the flag just waits in flash until the
   next reset.
4. **Reset the device again** (switches still at `00000`). This boot is the one that actually
   captures the reference reading, at the exact point in boot where the real switch decode
   happens (before WiFi/LoRa start) — reading it mid-runtime instead would see a different
   electrical load and give a wrong reference value.
5. **Verify with `printCSWData`.** Confirm `cswReferenceRaw` is a plausible `00000` reading
   (roughly 7000–8500 on units tested so far) — not `0` (never calibrated), and not matching
   some other switch position's raw value (a sign the switches moved before step 4). This
   command also prints `cswScaleFactor` and the resulting `cswDecodeValue`, so you can confirm
   the math end to end before trusting the unit.
6. Set the switches to the real desired function and reset again for normal operation.

**Factory tool automation.** As of 2026-09-01, the Factory device-configuration tool runs this
procedure automatically as the last step after flashing product firmware — it prompts for the
two switch/reset actions, sends `CalibrateCSWReference`, and verifies `cswReferenceRaw` itself.
Units configured through the Factory tool get this for free; it only needs doing manually for
units flashed some other way (e.g. a bench unit reflashed directly from the Arduino IDE).

**A reflash does not normally erase this.** Calibration lives in NVS (flash), a separate
partition from the sketch code — a normal Arduino "upload sketch" leaves it untouched. It's
wiped by a full chip erase (`esptool.py erase_flash`, or Arduino IDE's **Tools → Erase All
Flash Before Sketch Upload** set to *Enabled* instead of the default *Disabled*) — **or by
flashing with the wrong esptool version**, see §7, which can silently wipe NVS on a completely
ordinary flash with no erase requested at all. If a unit that was previously calibrated suddenly
isn't, check both before assuming something else is wrong — and note either cause also wipes
device name, WiFi credentials, and every other stored setting at the same time, so if those are
gone too, that confirms it's one of these two rather than something narrower.

### 3.3 How the calibration is applied

```
scaleFactor    = 8326 / cswReferenceRaw   (8326 = the original reference calibration's 00000 point)
cswDecodeValue = rawCSWValue * scaleFactor
```

`cswDecodeValue` — not the raw ADC reading — is what gets compared against the threshold table.
If a device has never been calibrated (`cswReferenceRaw == 0`), `scaleFactor` defaults to `1.0`.
An uncalibrated unit will often still decode to a *plausible-looking* function (because the
brackets are contiguous), just the wrong one — typically landing one bracket away from the
correct switch position, which is why this can look like a flaky switch reading rather than a
missing calibration. `printCSWData` (§3.2 step 5) is the only reliable way to tell the two apart.

### 3.4 Verification example

Symptom: switches set to `10001` (2 Flow Sensors, Battery Saver on), but the unit reports
`FUN_1_TANK` instead. Running `printCSWData` shows `cswReferenceRaw=0` — never calibrated — so
`cswScaleFactor` defaulted to `1.0`, and the unscaled raw reading landed one threshold bracket
below where it should have. Running the calibration procedure (§3.2) and resetting with the
switches back at the real function resolved it immediately.

### 3.5 Trough / Septic Tank height calibration

Separate from CSW calibration (§3.1–3.4). This calibrates the **ultrasonic level reading itself**
for `Water Trough` and `Septic Tank` mode — set it any time the sonar is mounted at a new height
or a tank/trough is physically changed.

```
SetTroughParameters#<sensorHeightCm>#<levelMinCm>#<levelMaxCm>#
```

- `sensorHeightCm` — the sonar's mounting height above the tank/trough's empty floor (stored as
  `maximumScepticHeight`).
- `levelMinCm` / `levelMaxCm` — the two zone-boundary offsets described in §5.3, stored as
  `troughlevelminimumcm` / `troughlevelmaximumcm`.

Persisted to NVS via `secretManager.saveTroughParameters(...)` — same durability properties as CSW
calibration (survives a normal reflash, wiped by a full chip erase or the wrong esptool version,
§7). There's no equivalent `print...` verification command for this one (unlike `printCSWData`
for the switch) — confirm it took by checking `printCurrentDSDData`'s level reading against a
known physical level, or by watching the Tank/Trough Level LED (§5.3) settle into the expected
zone.

**Also settable from the device's own web configuration UI**, not just the serial command above —
the same three values, submitted as `SetScepticRange` to `/DaffodilServlet`, land in the exact
same NVS fields. Either path works; use whichever is more convenient at install time.

**This calibration only affects `Water Trough` mode's coloring.** `Septic Tank` mode's LED zones
are computed from a fixed 90cm maximum sensor distance (`MAX_DISTANCE`) regardless of what's set
here — see §5.3. Setting trough parameters on a Septic Tank–mode unit has no visible effect on its
own display, though the values are still stored (and do apply if the same unit is later switched
to `Water Trough` mode).

## 4. Power Management

### 4.1 Two independent concepts

Battery protection and solar-aware reporting are **not** the same setting, even though earlier
firmware conflated them:

- **`hasBattery`** — computed once at boot from the battery-presence heuristic — gates *all*
  over-discharge protection: forced sleep at low voltage, LED cutoff, WiFi cutoff, and the
  critical-voltage COMMA mode (§4.3). This applies **regardless of Switch 5** — any unit with a
  real battery attached is protected.
- **`usingSolarPower`** (Switch 5) — purely the reporting-frequency trade-off described in §2.2.
  A unit with Switch 5 OFF (frequent reporting) still gets full battery protection if a battery
  is actually attached.

### 4.2 LED brightness

LED brightness is a function of **actual available power**, for every unit with a battery,
regardless of Switch 5:

1. **Darkness** — if the light sensor reads genuine night/deep-shade, brightness is capped low
   (30/255).
2. **Battery current headroom** — if the unit is net *discharging* even accounting for the
   LEDs' own draw, brightness drops low (20/255). If net *charging*, brightness scales smoothly
   up toward full in proportion to how much charging headroom exists. This is a smoothed,
   one-cycle-delayed calculation rather than an instant on/off snap, specifically to avoid
   oscillating on and off right at the charge/discharge boundary.
3. **Low battery voltage** — hard floor at low brightness if battery voltage drops below 3.28V,
   regardless of the above.

### 4.3 Battery protection thresholds

| Threshold | Value | Effect |
|---|---|---|
| Sleeping voltage | 3.12V | Force deep sleep — cliff edge for LiFePO4 |
| COMMA voltage | 2.80V | Critically low — skip all work, wait for the battery to recover before resuming |
| Minimum LED voltage | 3.18V | Turn off LEDs entirely |
| Minimum WiFi voltage | 3.28V | Turn off WiFi (preserve power for LoRa) |
| Minimum init WiFi voltage | 3.35V | Battery must sustain this for 30s before WiFi is allowed to start |

All of the above apply whenever a battery is attached, independent of Switch 5.

### 4.4 What stays solar-only

The following remain tied to Switch 5 = ON specifically, since they represent solar-specific
scheduling policy rather than battery protection in general:

- Solar-efficiency-based sleep scheduling
- Cloudy-day detection and its sleep-time extension
- The efficiency-gated decision of *whether* LEDs get any power this cycle at all (brightness
  itself, §4.2, applies either way — this is only about whether they light up)

### 4.5 Dynamic sleep

Using the onboard RTC and GPS-configured coordinates, Daffodil calculates sunrise and sunset for
its exact location and season. Right after sunset it wakes roughly every 7–8 minutes; as night
deepens the interval grows longer with no fixed cap; in the 90 minutes before dawn it wakes
every 90 seconds, ready to catch the first moments of solar charging. Below the COMMA voltage
threshold (2.80V) the device enters Coma state — sleep duration is still calculated normally,
but sensor data collection is suspended until charge recovers.

## 5. LED Status Display Guide

The 15 WS2812 LEDs are arranged in a 3-row × 5-column grid, wired in plain row-major order (not
serpentine):

```
row1:  0  1  2  3  4
row2:  5  6  7  8  9
row3: 10 11 12 13 14
```

The display cycles through several states every wake cycle — temperature, tank/trough/flow,
internet status, LoRa TX result, and error/battery status. Within the tank/trough/flow state,
two-sensor modes (2 Flow Sensors, 2 Tanks, 1 Flow Sensor + 1 Tank, Water Trough + Tank) show
sensor 1 and sensor 2 back-to-back before the display moves on, rather than alternating once per
full outer cycle, so both readings are visible in quick succession.

### 5.1 Temperature

![Temperature LEDs](manual-assets/leds/led-temperature.png)

Left columns encode the tens digit (filling top-to-bottom, inward from column 0); right columns
encode the units digit (filling top-to-bottom, inward from column 4). Green = positive
temperature, blue = negative, yellow = exactly 0°C. A sensor read failure (the firmware's `-99`
sentinel for `outdoortemperature`) shows a distinct red pattern on LEDs 1, 2, 3, 7, 12 instead of
digit fill — this is a fixed shape, unrelated to the digit-fill LEDs above.

### 5.2 Flow Sensor

![Flow Sensor LEDs](manual-assets/leds/led-flow.png)

An "F" icon (LEDs 0, 1, 2, 5, 6, 10). Flow has no fill level to show — just whether the sensor
currently detects movement: blue = flowing, red = no flow.

A small blue marker dot always accompanies the icon, at LED 4 — this isn't limited to two-sensor
modes: even standalone `1 Flow Sensor` permanently lights LED 4 alongside the F icon (the firmware
treats every flow/tank function as a "slot," whether or not it actually has a second sensor to
alternate with — see §5.3 for the full slot-assignment picture). In a two-slot mode (`2 Flow
Sensors`, `1 Flow Sensor + 1 Tank`) the display instead alternates each cycle between slot 1
(F icon or tower, LED 4 lit) and slot 2 (LED 14 lit) — see §5.3.

### 5.3 Tank / Trough Level

![Tank/Trough Level LEDs](manual-assets/leds/led-level.png)

A 3×3 "tower" of LEDs, in one of two positions depending on function:

- **Unshifted** — columns 1–3 (LEDs 1,2,3,6,7,8,11,12,13), no marker dot. Used only by
  `Septic Tank` and `Water Trough` (and the unreachable `Voltage Monitor`, §8).
- **Shifted** — columns 0–2 (LEDs 0,1,2,5,6,7,10,11,12), always paired with the same blue slot
  marker described in §5.2: LED 4 while slot 1 is on screen, LED 14 while slot 2 is. Used by every
  function in the flow/tank "slot" family — `1 Tank`, `2 Tanks`, `1 Flow Sensor + 1 Tank`, and
  `Water Trough + Tank`. Single-slot functions in this family (`1 Tank`) permanently show slot 1
  with LED 4 lit, exactly like `1 Flow Sensor` does for the F icon (§5.2); two-slot functions
  alternate slot 1/slot 2 back-to-back each display cycle.

**The fill-color scheme also differs by function** — same tower LEDs, three unrelated formulas:

- **Septic Tank** — percent-of-max-distance, 4 zones: `measuredHeight × 100 / 90cm` (90cm is a
  fixed constant, `MAX_DISTANCE`, not the per-device §3.5 calibration). Red = critical (≤25%),
  yellow = warning (26–50%), green = good (51–75%), blue = full (>75%).
- **Water Trough** — only **3** zones, no yellow warning tier, and the boundaries are the
  per-device §3.5 calibration values rather than fixed percentages: red if the sonar reading is
  at or beyond `sensorHeightCm − levelMinCm` (surface far from the sensor → low/critical), green
  between that and `sensorHeightCm − levelMaxCm`, blue below that (surface close to the sensor →
  full). If a Water Trough unit's display never leaves red or never leaves blue, check the §3.5
  calibration before assuming a sensor fault.
- **`1 Tank`, `2 Tanks`, `1 Flow Sensor + 1 Tank`, and the tank slot (slot 1) of
  `Water Trough + Tank`** — a different 4-zone percent scheme, `tankPercentFull()`: percent-full
  from the tank's pressure-sensor reading and its configured height, bucketed at the same
  25/50/75% breakpoints as Septic Tank above. This is unrelated to §3.5's `SetTroughParameters` —
  per-device tank height calibration is still TBA, see §8.
- **The trough slot (slot 2) of `Water Trough + Tank`** — uses the *same* 3-zone, §3.5-calibrated
  formula as standalone Water Trough above, but is not yet functional (§2.1, §8): with
  `measuredHeight` pinned at the `-99` sentinel, the comparison always falls through to the last
  zone, so **this slot always displays blue** — don't read that as an actual level. Once the
  underlying sensor is wired up, this slot will start reflecting the same §3.5 calibration
  already stored for the unit.

### 5.4 Internet Status

![Internet Status LEDs](manual-assets/leds/led-wifi.png)

Antenna-shaped pattern (LEDs 1, 2, 3, 5, 9, 11, 12, 13). Green = AP (setup) mode. Blue = connected
via WiFi station mode; in that mode the centre LED (LED 7) shows internet reachability
specifically — blue if reachable, red if WiFi is connected but there's no path to the internet.
All red = WiFi is being skipped this cycle (either genuinely off, or — on a solar-mode unit —
solar efficiency is below the minimum required to justify running WiFi at all).

**LED 7 briefly means something else during an actual cloud upload.** When a scheduled upload to
Digital Stables fires on this same display cycle, LED 7 is overwritten with the upload result
instead of reachability: blue = success (HTTP 200), magenta = server error (HTTP 500), red = any
other failure. It reverts to a plain reachability indicator on the next cycle.

### 5.5 LoRa Status

![LoRa Status LEDs](manual-assets/leds/led-lora.png)

Four LEDs (1, 6, 11, 12) in a small column, showing the result of the last LoRa transmission
attempt: green = TX OK, red = TX failed. All LEDs briefly go dark during every actual transmission
(to reduce voltage sag on the 5V rail), so a short blackout accompanies every send — that's
expected, not a fault.

The underlying `drawLora()` function also defines a yellow state, but no current code path ever
calls it with that argument — only TX OK / TX failed are reachable today.

### 5.6 Error

![Error LEDs](manual-assets/leds/led-error.png)

An "E" shape (LEDs 0, 1, 2, 5, 6, 10, 11, 12), always red in current firmware, with a blue code
dot identifying which error: the dot at LED 4 means the ADS1115 sensor wasn't found at boot; the
dot at LED 9 means onboard storage is nearly full. `drawError()` also defines a yellow/blue
variant of the E-shape and a third dot position (LED 14), but no current error path invokes them —
both are currently reachable only as red, two-dot states.

### 5.7 Battery Voltage

![Battery Voltage LEDs](manual-assets/leds/led-battery.png)

A "B"-shaped group (LEDs 1, 6, 7, 11, 12) showing the LiFePO4 voltage zone: blue ≥3.28V
(`minimumWifiVoltage`), green 3.18–3.28V (down to `minimumLEDVoltage`), amber 3.10–3.18V warning,
red 1.0–3.10V critical, and — below 1.0V — a fifth, visually distinct magenta zone meaning "no
battery detected" (the same <1.0V heuristic as `noBatteryDetected`, §3.3/§6): either nothing is
plugged in, or a battery is so deeply dead it reads electrically the same as absent. Don't confuse
this with the red critical zone just above it — magenta specifically means "the firmware doesn't
think there's a battery here at all," not merely "very low."

LED 4 (top-right) is the power-source indicator, based on live INA219 current with a ±1.0mA
deadband: green = charging (current < −1.0mA), red = discharging (current > +1.0mA), blue =
within the deadband (near-zero/transition). This is a separate, instantaneous calculation from the
smoothed charge/discharge headroom used to set LED brightness (§4.2) — the two can briefly
disagree around the charge/discharge boundary.

LED 9 shows operating mode: green = Full Mode, amber = Cloudy Mode (the whole group also runs at
50% brightness in Cloudy Mode, to signal reduced solar availability at a glance). LED 14 shows
forecast freshness: green = current data received from Annabelle within the last ~31 minutes, red
= never received or stale.

## 6. Serial Command Reference

| Command | Description |
|---|---|
| `CalibrateCSWReference` | Arms per-device CSW calibration; requires a reset to actually capture it (§3.2) |
| `printCSWData` | Prints `lastResetReason`, `rawCSWValue`, `cswReferenceRaw`, `cswScaleFactor`, `cswDecodeValue`, `noBatteryDetected`, and the decoded function — **the** command to run when the switch seems to be decoding wrong |
| `printCurrentDSDData` | Prints the full current sensor/runtime snapshot, including `lastResetReason` and the CSW decode — does **not** include calibration internals, use `printCSWData` for that |
| `SetDeviceName#<name>` | Sets and persists the device's full name |
| `SetDeviceShortName#<name>` | Sets and persists the device's short name |
| `SetDeviceSensorConfig#...` | Sets and persists device name, short name, sensor names, timezone, and location together |
| `SetTroughParameters#<sensorHeightCm>#<levelMinCm>#<levelMaxCm>#` | Sets and persists the ultrasonic level-calibration values used by `Water Trough`/`Septic Tank` mode (§3.5) |
| `SetTimezone#<tz>` | Sets the device's timezone string |
| `SetGroupId#<id>` | Sets the device's group identifier |
| `GetSerialNumber` | Prints the device's serial number (derived from the onboard temperature sensor's hardware address) |
| `GetProductDefinition` | Prints a full product/config summary — name, power source, firmware, WiFi SSID, device name, serial number, etc. — useful for confirming whether a unit's stored configuration is intact after a reflash |
| `ConfigWifiSTA#ssid#password#hostname` | Configures WiFi station mode |
| `ConfigWifiAP#ssid#password#hostname` | Configures WiFi access-point mode |
| `clearAllCommaRecords` | Clears stored COMMA-mode event records |
| `exportDSDCSV` | Exports stored sensor data as CSV |
| `GenerateDSDReport` | Prints a device time / CSW / sensor summary report |

## 7. Troubleshooting

**Device reports the wrong function even though the switches are set correctly.**
Run `printCSWData` and check `cswReferenceRaw`. If it's `0`, the device has never been
calibrated — run the procedure in §3.2. If it's a plausible-but-wrong value, the switches
likely moved between arming the calibration and the capture reset (§3.2 step 1's warning) —
redo the calibration with extra care to leave the switches untouched at `00000` throughout.

**Device previously worked, then suddenly loses calibration / device name / WiFi settings
after a reflash.**
Confirmed root cause (reproduced twice on a bench unit): the Arduino IDE's **Tools → Erase All
Flash Before Sketch Upload** setting was set to *Enabled*. NVS — where calibration, device
name/short name, group ID, sensor names, and WiFi credentials all live — is a separate flash
partition from the sketch code, so a normal upload leaves it completely untouched. With that
setting on, however, *every* upload does a full chip erase first, silently wiping NVS along with
the old firmware. This is what makes the symptom look like it "just happened" on a plain reset
or power cycle — the actual trigger was the reflash immediately before it.

Before concluding it's this, rule out the other candidates in this order (all three were checked
and ruled out during the confirmed incident): a firmware-side self-wipe (grep for any live
`preferences.clear()`/`nvs_flash_erase()` call — none exist outside commented-out code); the
Factory tool having been re-run against the unit (it re-provisions on purpose, so check whether
anyone used it recently); and a partition-table mismatch (`partitions.csv` — NVS/otadata/app0/
app1/spiffs/coredump should be cleanly non-overlapping).

To confirm it's the erase-all-flash setting specifically, run `printCSWData` or
`printCurrentDSDData` and check `lastResetReason` — a plain reset or brownout is not what
triggered the wipe; an `SW (software reset, e.g. esptool/upload)` result right after config went
missing points at the reflash. This diagnostic was added to the firmware specifically so a future
occurrence doesn't require reconstructing the timeline from memory.

Fix: set the Tools-menu option to *Disabled*. Note the *next* upload still wipes everything, since
the erase happens as part of that upload — after it, re-provision the device name (via
`SetDeviceSensorConfig#...` or `SetDeviceName#`/`SetDeviceShortName#`) and redo the CSW
calibration (§3.2) once more. Both will then persist through future uploads normally.

**Config wipes on every flash even with Erase All Flash correctly set to Disabled.**
Confirmed root cause, bench-tested 2026-09-03: it's the **esptool version**, not the Arduino IDE
setting. `esptool` 4.7.0 (both an apt-installed and a freshly pip-installed copy) silently wipes
the entire NVS partition on every flash — even though the flash only writes four fixed regions
(bootloader at 0x1000, partition table at 0x8000, app at 0x10000, `boot_app0` at 0xe000), nowhere
near NVS at 0x9000. Controlled test: calibrate + name a unit, reflash with 4.7.0, both come back
blank. `esptool` 3.0.0 — the version already hardcoded into every Factory tool flashing handler,
at `esptool_py/3.0.0/esptool.py` under the Arduino15 packages directory — does **not** have this
problem; the identical test with 3.0.0 leaves both intact. The exact internal reason inside
esptool's v3-vs-v4 changes wasn't identified, but the practical rule is: **always flash with
esptool 3.0.0 specifically**, never "whatever's newest" or whatever a package manager happens to
have installed. The Factory tool's "Create Deploy Package" flow now bundles that exact 3.0.0
`esptool.py` into the downloadable package for this reason, rather than trusting the target
machine's own install.

A related, now-mostly-moot gotcha from the same investigation: `--no-stub` (a flag needed only as
a fallback when a system esptool install is missing its stub_flasher data — a separate Debian
packaging issue, unrelated to the NVS-wipe cause above) makes this board's auto-program circuit
unreliable about exiting download mode after `--after hard_reset`, leaving it looking dead (no
boot banner at any baud rate) until a full physical power cycle (unplug USB *and* battery, wait,
reconnect) — normal stub mode doesn't have this problem. Bundling esptool 3.0.0 removes the need
for `--no-stub` in the Create Deploy Package flow, so this shouldn't come up in practice anymore —
but it's worth knowing if `--no-stub` is ever needed again for some other reason (e.g. a different
esptool version substituted in later) and a freshly flashed board looks unresponsive.

**Don't confuse `printCSWData` and `printCurrentDSDData` while diagnosing any of the above.**
Only `printCSWData` prints `cswReferenceRaw`, `cswScaleFactor`, `cswDecodeValue`, and
`noBatteryDetected` (§6) — `printCurrentDSDData` does not include any of them, even though both
print `lastResetReason` and look similar at a glance. Checking the wrong one mid-investigation
reads as "calibration is fine" when it's actually just not being shown.

**A flow sensor on one screw terminal never registers pulses, but works fine on the other
terminal (with the same physical sensor).**
This points to that terminal's GPIO not being correctly configured as an input at boot,
typically because it's shared with another peripheral (e.g. the ultrasonic trigger pin) that
claims it as an output during initialization. If you have firmware source access, check that the
sensor's GPIO is explicitly set to `INPUT` mode early in `setup()`, after any other library or
global object that might also touch that same pin. This was diagnosed and fixed on `Daffodil.ino`
for the Sensor 1 (pin 18) terminal on 2026-09-02 — devices built from a firmware version at or
after that date should not see this issue.

**Annabelle or a LoRa base station isn't receiving a Daffodil unit's transmissions.**
Confirm the unit is actually transmitting (§5.5's LoRa Status LED should show TX OK). If it is,
but the receiver shows nothing at all, check radio settings match (frequency, spreading factor,
bandwidth, sync word) on both ends. If the receiver's serial debug shows a "rejected" message
for the packet, the packet arrived but failed validation — check that the device's serial number
(from a properly-detected onboard temperature sensor) and checksum are valid, via
`GetSerialNumber` or `printCurrentDSDData` on the transmitting unit.

## 8. Known Limitations / Roadmap

- `Water Trough + Tank` (`1110`) only reports its tank sensor. The sonar's trigger/echo lines are
  hardwired to pins 18/33, which this mode already dedicates to tank1 pressure sensing — so the
  trough half is stuck at the firmware's `-99` "not read" sentinel until a real single-wire
  ultrasonic driver (distinct from the current two-pin sonar library) replaces it for this mode.
  On the LED display (§5.3) this shows up as the trough slot always displaying blue, regardless of
  actual level — don't mistake that for a working reading.
- `Septic Tank` mode's LED fill percentage is computed against a fixed 90cm maximum sensor
  distance, not the §3.5 `SetTroughParameters` calibration — that calibration currently only
  changes `Water Trough` mode's coloring. Not a bug, but easy to assume otherwise if you've just
  calibrated a unit and don't see its Septic Tank display change.
- **Per-device tank pressure height (`tank1HeightMeters`/`tank2HeightMeters`) and flow-sensor
  calibration factor (`qfactor1`/`qfactor2`) — TBA.** Both feed real, load-bearing calculations
  (§5.3's `tankPercentFull()` and the flow-rate math respectively), but the configurator UI that's
  meant to set them per device is still being built out (a separate, in-progress piece of work).
  This section will be filled in with the actual procedure once that lands.
- The non-solar half of the switch space (Switch 5 = OFF) hasn't been swept through the
  calibrated decode table position-by-position the way the Battery Saver half has — the mapping
  should be identical, but hasn't been bench-verified every position yet.
- A hardware-level backfeed exists from `V50` into the battery-sense node when no battery is
  attached, which is why battery-detection needs a delta-based/settling-aware check rather than
  a simple voltage threshold. Not an issue for real deployed units (which always have a
  battery), but relevant if bench-testing without one.
- The Voltage Monitor, Temperature+Soil Moisture, and Light Detector functions exist as firmware
  constants but aren't reachable via any current switch position — no wiring or assigned slot in
  the current scheme.
- `2 Water Troughs` (`0011`) has its switch position and firmware constant assigned, but no
  sensor-reading or LED-display code yet — pending the UART ultrasonic sensor hardware.
