# Briefing for the "Daffodil Manual" agent

You're picking up ownership of **"Daffodil Manual.md"** in this directory. A separate Claude
Code session in `~/Data/DigitalStables/webapp_claude` is actively working on the Daffodil
firmware and the website (`src/main/webapp/daffodil.html`) at the same time — that's deliberate,
Ari split the work into two chats so the software chat can "still needs polishing" without the
manual work interrupting it. This doc is your cold-start context so you don't have to re-derive
any of it.

## Your job

Write and maintain a complete, polished **Daffodil Manual** — eventually published in the
website's download section (not wired up yet, that's a later step). It needs to cover: the
config switch / function modes, per-device calibration, power management, and the LED status
display, with real diagrams. Ari wants to keep iterating on it here in this project directory
first, before it goes anywhere near the website.

## Where everything is

- **`Daffodil Manual.md`** (this directory) — the manual itself, already has a full first draft
  (see "Current state" below). This replaces/renames the old
  `Daffodil Power Management and Configuration.md` (already `git rm`'d, staged).
- **`manual-assets/leds/*.png`** — 7 rendered LED-state diagrams (temperature, flow, tank/trough
  level, internet, LoRa, error, battery), referenced from the manual via relative
  `![...](manual-assets/leds/...)` image links.
- **`manual-assets/render_led_diagrams.py`** — the script that generated those PNGs. It hand-
  duplicates the `LED_GROUPS` color/layout data from `daffodil.html` (see below) using Pillow —
  there's a comment at the top of the script saying to keep it in sync. If the website's LED
  colors/states ever change, update this script's `LED_GROUPS` list to match and rerun it
  (`python3 manual-assets/render_led_diagrams.py`) rather than hand-editing the PNGs.
- **`CONFIG_SWITCH_FUNCTIONS.md`** — a shorter, separate reference doc with just the switch→
  function bit table and firmware constant names. Still exists, untouched. The manual's function
  table should stay consistent with it (both ultimately derive from the same `Daffodil.ino`
  CSW-decode table).
- **git status right now**: `Daffodil Manual.md` and the 7 PNGs are `git add`ed (staged, not
  committed — nothing in this repo has been committed by Claude this session, only staged).
  `manual-assets/render_led_diagrams.py` is untracked. `Daffodil.ino` shows as modified (the
  other session's in-progress firmware fixes — see below). Don't commit anything unless Ari
  explicitly asks.
- **The old PDF**, `Daffodil Power Management and Configuration.pdf`, is still sitting on disk,
  orphaned (its .md source was renamed out from under it). Ari hasn't said what to do with it yet
  — asked once, got interrupted before answering. Don't delete it without asking again.

## Source-of-truth files — re-check these, don't trust this briefing's paraphrase of them

The other session is actively editing firmware and the website concurrently. Before writing or
asserting anything technical, read the current state of:

- **`Daffodil.ino`** (this directory) — the firmware itself. As of this briefing it has three
  uncommitted changes from tonight's debugging (details below, but verify against the live file):
  a `pinMode(SENSOR_INPUT_1, INPUT)` / `pinMode(SENSOR_INPUT_2, INPUT)` fix early in `setup()`,
  new `saveDeviceName`/`saveDeviceShortName` calls in the `SetDeviceName`/`SetDeviceShortName`
  serial command handlers, and a new `lastResetReason` diagnostic (`esp_reset_reason()`) printed
  in both `printCurrentDSDData` and `printCSWData`.
- **`~/Data/DigitalStables/ProjectsSupport/Esp32ArduinoLib/libraries/DigitalStablesEsp32Lib/Esp32SecretManager.h`
  and `.cpp`** — the NVS persistence layer (calibration, device name/short name, etc.). Also has
  uncommitted changes tonight (the new `saveDeviceName`/`saveDeviceShortName` methods).
- **`~/Data/DigitalStables/webapp_claude/src/main/webapp/daffodil.html`** — the public website
  page. It has its own copies of: the function table (`CONFIG_POSITIONS` JS array), the Switch 5
  plain-language explanation, and the LED groups (`LED_GROUPS` JS array, source of the colors
  used in `render_led_diagrams.py`). Keep the manual's wording/labels consistent with this file's
  *plain-English* names (e.g. "1 Flow Sensor", not `FUN_1_FLOW`, in body copy — reserve firmware
  constant names for reference tables, same pattern the manual already uses in §2.1/§6).

Don't assume anything in this briefing about those files' *current* content is still accurate by
the time you read it — re-grep/re-read before writing anything that depends on their specifics.

## Current state of the manual (as of this briefing)

Full draft exists with 8 sections:

1. Overview
2. Configuration Switch — Function Modes (§2.1 function table with plain-English names + status
   column flagging `2 Water Troughs` as "Coming soon"; §2.2 the Switch 5 "Battery Saver vs.
   Frequent Reports" explanation, matching the website's framing)
3. Per-Device CSW Calibration (full procedure, §3.4 has a worked example from tonight's bench
   session — see incident below)
4. Power Management (battery protection thresholds, LED brightness logic, dynamic sleep)
5. LED Status Display Guide — all 7 groups, each with an embedded PNG diagram and a description
6. Serial Command Reference (table of all the relevant commands)
7. Troubleshooting
8. Known Limitations / Roadmap

A quick PDF preview was rendered tonight (python-markdown + weasyprint, see "PDF preview recipe"
below) to sanity-check layout — it read cleanly, tables and diagrams both came through fine.

## Immediate next steps (in rough priority order)

1. **Fold tonight's "everything wiped" incident into §7 Troubleshooting as a *confirmed* root
   cause**, not a hypothesis. The current draft (written mid-incident, before the cause was
   confirmed) has an entry along the lines of "check Arduino's Erase All Flash setting" framed as
   one possible cause among others. It needs rewriting as the primary, confirmed entry — Ari
   explicitly confirmed tonight that the bench machine's Arduino IDE had **Tools → Erase All
   Flash Before Sketch Upload** set to **Enabled**, which was the actual cause. See "Tonight's
   incident, in full" below for the complete narrative — worth preserving most of these details
   since they're expensive to re-derive and make the troubleshooting section genuinely useful
   rather than generic.
2. Mention the new `lastResetReason` diagnostic (added to firmware tonight, see source-of-truth
   note above) in §6 Serial Command Reference and/or §7 Troubleshooting, once you've confirmed
   against the live `Daffodil.ino` that it's actually there and what exactly it prints.
3. Ask Ari (he hasn't answered yet, got interrupted):
   - What to do with the orphaned old PDF (delete now / leave until a replacement exists).
   - Whether the manual's current technical depth (firmware constant names, NVS/flash internals,
     exact voltage thresholds) is right for a public download-section manual, or should be
     simplified for a customer audience with the technical version kept separate/internal. My
     own read, for what it's worth: the depth is probably *right* for an installer/technician
     audience (which is who actually needs to do calibration, wiring, DIP-switch configuration —
     not a casual end customer), but it's Ari's call.
4. Eventually: an actual PDF-generation pipeline for the website download section. Not built yet.
   See the recipe below — it's a viable starting point (weasyprint is already installed on this
   machine), but hasn't been productionized (no image-path handling beyond the base-url flag,
   no consistent typography/branding pass, TOC anchors don't currently resolve in the weasyprint
   render — cosmetic only, didn't affect content, not investigated further).

## Tonight's incident, in full (for §7 Troubleshooting)

Bench-testing a unit called "BenchTest" (serial `28 C3 E7 31 12 26 3 18`), configured for 2 Flow
Sensors. Over the course of the evening it lost its device name, short name, group identifier,
sensor names, *and* CSW calibration reference — twice. Symptoms both times: `printCurrentDSDData`
showed all name/identifier fields blank; the switch decoded to the wrong function (once landing
on `FUN_1_TANK` instead of the physically-set `FUN_2_FLOW`/`FUN_1_FLOW` position).

Investigation ruled out, in order: a firmware-side self-inflicted wipe (grepped for any
`preferences.clear()`/`nvs_flash_erase()` call site — none live, only commented-out code in the
library); the Factory tool's `ConfigureDeviceProcessingHandler` (Ari confirmed he hadn't used the
Factory tool recently on this unit — only once, in the past, to originally provision it); and a
partition-table mismatch (checked the actual generated `partitions.csv` — NVS/otadata/app0/app1/
spiffs/coredump partitions are cleanly non-overlapping, ruled that out too).

Root cause, confirmed by Ari: the bench machine's **Arduino IDE had Tools → Erase All Flash
Before Sketch Upload set to Enabled**. NVS lives in its own flash partition, separate from the
sketch code — a normal "upload sketch" leaves it completely untouched, but with that setting on,
every single upload does a full chip erase first, wiping calibration, device name/short name,
WiFi credentials, and everything else in NVS along with the old firmware. This explains both
occurrences and why it looked like it "just happened" from a plain reset — the actual trigger was
the reflash immediately before, which the setting silently turned into a full wipe.

Fix: set that Tools-menu option to Disabled, reflash once more (that last upload still wipes
everything, since the erase happens as part of *that* upload), then re-provision the device name
(`SetDeviceSensorConfig#...` or `SetDeviceName#`/`SetDeviceShortName#`) and redo the CSW
calibration procedure (§3) one final time. After that, both should persist through future uploads
and resets.

Along the way, a genuinely useful diagnostic (`lastResetReason`, `esp_reset_reason()`) was added
to the firmware specifically to make a *future* recurrence of "something wiped my config"
immediately diagnosable — distinguishes `POWERON`/`EXT`/`SW` (software/upload)/`BROWNOUT`/
watchdog resets, so the next time this happens, nobody has to reconstruct the sequence of events
from memory the way tonight required.

One byproduct, not part of the fix: while investigating, a real validation bug was found in
`ConfigureDeviceProcessingHandler.java` (the Factory tool) — its hostname field check
(`hostaname==null || hostaname.length()>11`) doesn't reject an *empty* string, unlike the
shortname field right below it which correctly requires 1–4 characters. Confirmed unrelated to
tonight's incident. Ari deferred fixing it — not in scope for the manual, just worth knowing it
exists if it ever comes up.

## PDF preview recipe (weasyprint, already installed on this machine)

No pandoc on this machine, but `weasyprint` (HTML→PDF) and Python's `markdown` module are both
available. Rough recipe used tonight — refine as needed, this was just a sanity-check render, not
a production pipeline:

```python
import markdown
src = open("Daffodil Manual.md", encoding="utf-8").read()
html_body = markdown.markdown(src, extensions=["tables", "toc", "fenced_code"])
# wrap html_body in a <html><head><style>...</style></head><body>...</body></html> shell
```

Then, critically, pass `-u "file:///path/to/Daffodil/"` (`--base-url`) to the `weasyprint` CLI so
relative image paths (`manual-assets/leds/...`) actually resolve — without it every image 404s
silently and weasyprint prints an `ERROR: Failed to load image` per missing image but still
produces a PDF (easy to miss). To eyeball pages afterward, either `pdftoppm`/`pdftocairo` (both
installed) or Claude's own `Read` tool on the PDF directly with a `pages` range both work.

## Scope boundaries

- Don't edit `Daffodil.ino`, `daffodil.html`, or anything under `webapp_claude` — that's the
  other session's territory. Read them for reference, don't write to them.
- Don't touch `Esp32SecretManager.h`/`.cpp` either, same reason.
- Your write scope is `Daffodil Manual.md`, `manual-assets/*`, and (if it comes up)
  `CONFIG_SWITCH_FUNCTIONS.md` if it genuinely needs a correction — but check with Ari first
  since that file is treated as a source-of-truth reference by other docs too.
