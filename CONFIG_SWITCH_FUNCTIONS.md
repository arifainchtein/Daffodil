# Daffodil Config Switch — Function Mapping

Current, as-implemented mapping in `Daffodil.ino` (`setup()`, DIP switch decode). Source of truth
for the Digital Stables website's device function table.

The switch has 5 positions. The first 4 select the function; the 5th selects solar power mode
(does not change which function is selected).

| Bits (1234) | Function |
|---|---|
| `0000` | FUN_1_FLOW |
| `1000` | FUN_2_FLOW |
| `0100` | FUN_1_FLOW_1_TANK |
| `1100` | FUN_1_TANK |
| `0010` | FUN_2_TANK |
| `1010` | DAFFODIL_SCEPTIC_TANK |
| `0110` | DAFFODIL_WATER_TROUGH |
| `1110` | DAFFODIL_WATER_TROUGH_TANK1 |
| `0011` | DAFFODIL_2_WATER_TROUGH |
| `0001` | unassigned (reserved) |
| `1001` | unassigned (reserved) |
| `0101` | unassigned (reserved) |
| `1101` | unassigned (reserved) |
| `1011` | unassigned (reserved) |
| `0111` | unassigned (reserved) |
| `1111` | unassigned (reserved) |

5th bit (`usingSolarPower`): the function selected by bits 1-4 is identical either way — e.g.
`00000` and `00001` are both FUN_1_FLOW. As of 2026-09-01 this bit means a sensor-reading-frequency
vs. battery-duration tradeoff, not literally "does this unit have solar": `1` defers to the
solar-efficiency-based sleep schedule (sleeps more, samples less — for unattended solar-charged
deployments that don't need frequent readings); `0` samples more often and doesn't defer to that
schedule (e.g. continuous flow monitoring). Battery over-discharge protection (COMMA mode,
low-voltage cutoffs) applies either way whenever a battery is actually attached — it no longer
depends on this bit (previously it only applied when this bit was `1`, which meant a unit
configured for frequent sampling had no battery protection at all — see the firmware's
`hasBattery` vs `usingSolarPower` split).

## Notes

- `DAFFODIL_WATER_TROUGH` (`0110`) and `DAFFODIL_2_WATER_TROUGH` (`0011`, 2 independent troughs
  via UART ultrasonic sensors — sensor-reading code not yet implemented, pending hardware) are
  separate functions. `0011` used to be a duplicate `DAFFODIL_WATER_TROUGH` slot; reclaimed
  2026-09-01. `1110` used to be a third `DAFFODIL_WATER_TROUGH` slot but was fixed (2026-08-27) to
  `DAFFODIL_WATER_TROUGH_TANK1`, which had a function constant and full sensor/display support in
  firmware but was never actually wired to a switch position until then.
- 6 of the 16 four-bit patterns are still unassigned/reserved — no function, device just sets
  `usingSolarPower` and otherwise does nothing for those.
- `VOLTAGE_MONITOR`, `DAFFODIL_TEMP_SOILMOISTURE`, and `DAFFODIL_LIGHT_DETECTOR` exist as function
  constants but are **not currently reachable** via any switch position in this table — no
  hardware wiring, or no assigned slot in the current 5-bit scheme.
- Per-device CSW calibration (`CalibrateCSWReference` serial command, run with switches at
  `00000`) is required for reliable decoding — see "Daffodil Power Management and Configuration"
  for the full procedure and why it's needed.
