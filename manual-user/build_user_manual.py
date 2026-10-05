#!/usr/bin/env python3
"""Builds the user-facing Daffodil manual: manual-user/DaffodilManual.html -> DaffodilManual.pdf.

The technical reference is "Daffodil Manual.md" (installer/firmware depth). This one is the
plain-language user's edition. Switch/terminal/jumper data lives in FUNCTIONS below — keep it in
sync with the CSW decode table in Daffodil.ino and CONFIG_SWITCH_FUNCTIONS.md.

Usage: python3 manual-user/build_user_manual.py   (needs weasyprint)
"""
import datetime
import html
import pathlib
import subprocess

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
HTML_OUT = HERE / "DaffodilManual.html"
PDF_OUT = ROOT / "DaffodilManual.pdf"
LED_DIR = "../manual-assets/leds"

DIGITAL, ANALOG, NONE = "digital", "analog", "none"

# (switches 1-4, name, what it's for, terminal 18 sensor, 18 jumper, terminal 33 sensor, 33 jumper, note)
FUNCTIONS = [
    ("0000", "1 Flow Sensor", "Measures water flow through one pipe.",
     "Flow meter", DIGITAL, None, NONE, ""),
    ("1000", "2 Flow Sensors", "Measures flow through two pipes.",
     "Flow meter 1", DIGITAL, "Flow meter 2", DIGITAL, ""),
    ("0100", "1 Flow Sensor + 1 Tank", "One pipe's flow plus one tank's level.",
     "Flow meter", DIGITAL, "Tank pressure sensor", ANALOG, ""),
    ("1100", "1 Tank", "Level of one tank, using a pressure sensor at the bottom.",
     "Tank pressure sensor", ANALOG, None, NONE, ""),
    ("0010", "2 Tanks", "Levels of two tanks.",
     "Tank 1 pressure sensor", ANALOG, "Tank 2 pressure sensor", ANALOG, ""),
    ("1010", "Septic Tank", "How full a septic tank is, measured from above.",
     "Ultrasonic sensor", DIGITAL, None, NONE, ""),
    ("0110", "Water Trough", "Water level in a trough, measured from above.",
     "Ultrasonic sensor", DIGITAL, None, NONE, ""),
    ("1110", "Water Trough + Tank", "A trough level plus one tank level.",
     "Tank pressure sensor", ANALOG, "Ultrasonic sensor", DIGITAL,
     "The ultrasonic goes on terminal 33 in this setting."),
    ("0011", "2 Water Troughs", "Water levels in two troughs.",
     "Ultrasonic sensor 1", DIGITAL, "Ultrasonic sensor 2", DIGITAL,
     "Both levels are reported, but the lights don't show them yet."),
    ("0001", "Water Trough + Water Temperature", "A trough level plus the water temperature.",
     "Ultrasonic sensor", DIGITAL, "Water temperature probe", DIGITAL, ""),
]


def dip_svg(bits, show5=True, scale=1.0):
    """DIP switch drawing. bits = '0110' for switches 1-4; switch 5 drawn as 'your choice'."""
    n = 5 if show5 else 4
    w, h, gap = 14, 26, 4
    W = n * (w + gap) + gap
    H = h + 22
    out = [f'<svg class="dip" width="{W*scale:.0f}" height="{H*scale:.0f}" viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg">',
           f'<rect x="0.5" y="0.5" width="{W-1}" height="{h+7}" rx="3" fill="#c0392b" stroke="#922b21"/>']
    for i in range(n):
        x = gap + i * (w + gap)
        out.append(f'<rect x="{x}" y="4" width="{w}" height="{h}" rx="2" fill="#f4f4f4"/>')
        if i < 4:
            on = bits[i] == "1"
            ky = 6 if on else 4 + h / 2
            out.append(f'<rect x="{x+2}" y="{ky}" width="{w-4}" height="{h/2-2}" rx="1.5" fill="#222"/>')
        else:
            out.append(f'<rect x="{x+2}" y="6" width="{w-4}" height="{h-4}" rx="1.5" fill="#bbb"/>'
                       f'<text x="{x+w/2}" y="{4+h/2+4}" font-size="11" text-anchor="middle" fill="#fff" font-family="Roboto" font-weight="700">?</text>')
        out.append(f'<text x="{x+w/2}" y="{H-2}" font-size="10" text-anchor="middle" fill="#555" font-family="Roboto">{i+1}</text>')
    out.append("</svg>")
    return "".join(out)


def jumper_svg(state):
    """The two 2-pin headers (Digital | Analog) for one terminal, with the cap where it belongs."""
    out = ['<svg class="jmp" width="44" height="35" viewBox="0 0 58 46" xmlns="http://www.w3.org/2000/svg">']
    for col, label, key in ((0, "D", DIGITAL), (1, "A", ANALOG)):
        x = 8 + col * 28
        if state == key:
            out.append(f'<rect x="{x-6}" y="2" width="16" height="30" rx="3" fill="#1f6feb"/>')
        for y in (10, 24):
            out.append(f'<rect x="{x-2}" y="{y-4}" width="8" height="8" fill="{"#fff" if state == key else "#999"}"/>')
        out.append(f'<text x="{x+2}" y="44" font-size="10" text-anchor="middle" font-family="Roboto" fill="#444">{label}</text>')
    out.append("</svg>")
    return "".join(out)


def pill(state):
    return {DIGITAL: '<span class="pill d">Digital</span>',
            ANALOG: '<span class="pill a">Analog</span>',
            NONE: '<span class="pill n">No cap</span>'}[state]


def board_svg():
    """Simplified view of the board's bottom edge: jumpers, terminals, config switch."""
    s = ['<svg class="board" viewBox="0 0 640 300" xmlns="http://www.w3.org/2000/svg" font-family="Roboto">',
         '<rect x="4" y="4" width="632" height="292" rx="10" fill="#1d5e3a" stroke="#123d26" stroke-width="2"/>']

    def header_pair(x, y, title, cap):
        s.append(f'<text x="{x+38}" y="{y-10}" font-size="15" fill="#fff" text-anchor="middle" font-weight="700">{title}</text>')
        for i, lab in enumerate(("Digital", "Analog")):
            hx = x + i * 50
            if cap == i:
                s.append(f'<rect x="{hx-4}" y="{y-2}" width="30" height="58" rx="4" fill="#1f6feb" stroke="#fff"/>')
            for py in (y + 6, y + 30):
                s.append(f'<rect x="{hx+3}" y="{py}" width="16" height="16" fill="#d4af37" stroke="#8a6d1a"/>')
            s.append(f'<text x="{hx+11}" y="{y+76}" font-size="13" fill="#fff" text-anchor="middle">{lab}</text>')

    def terminal(x, y, labels):
        s.append(f'<rect x="{x}" y="{y}" width="150" height="70" rx="4" fill="#2e86de" stroke="#1b4f72" stroke-width="2"/>')
        for i, lab in enumerate(labels):
            cx = x + 40 + i * 70
            s.append(f'<circle cx="{cx}" cy="{y+35}" r="18" fill="#bfc9ca" stroke="#555"/>'
                     f'<line x1="{cx-11}" y1="{y+35}" x2="{cx+11}" y2="{y+35}" stroke="#555" stroke-width="4"/>')
            s.append(f'<text x="{cx}" y="{y-8}" font-size="16" fill="#fff" text-anchor="middle" font-weight="700">{lab}</text>')

    header_pair(40, 40, "18", 0)
    header_pair(210, 40, "33", 0)
    terminal(20, 185, ("18", "33"))
    terminal(190, 185, ("V50", "GND"))
    terminal(360, 185, ("SDA", "SCL"))
    s.append('<text x="95" y="280" font-size="12" fill="#cfe" text-anchor="middle">Sensor signals</text>'
             '<text x="265" y="280" font-size="12" fill="#cfe" text-anchor="middle">Sensor power</text>'
             '<text x="435" y="280" font-size="12" fill="#cfe" text-anchor="middle">Not used</text>')
    # config switch
    sx, sy = 535, 150
    s.append(f'<text x="{sx+48}" y="{sy-30}" font-size="15" fill="#fff" text-anchor="middle" font-weight="700">Config switch</text>'
             f'<text x="{sx-12}" y="{sy+12}" font-size="12" fill="#fff" text-anchor="end">ON</text>'
             f'<text x="{sx-12}" y="{sy+62}" font-size="12" fill="#fff" text-anchor="end">OFF</text>'
             f'<rect x="{sx-4}" y="{sy-10}" width="104" height="90" rx="4" fill="#c0392b"/>')
    for i in range(5):
        x = sx + 2 + i * 20
        s.append(f'<rect x="{x}" y="{sy}" width="14" height="66" rx="2" fill="#f4f4f4"/>'
                 f'<rect x="{x+2}" y="{sy+35}" width="10" height="28" rx="2" fill="#222"/>'
                 f'<text x="{x+7}" y="{sy+96}" font-size="13" fill="#fff" text-anchor="middle">{i+1}</text>')
    s.append("</svg>")
    return "".join(s)


def function_rows():
    rows = []
    for bits, name, what, s18, j18, s33, j33, note in FUNCTIONS:
        n = f'<div class="note">{html.escape(note)}</div>' if note else ""
        rows.append(f"""<tr>
<td class="sw">{dip_svg(bits, scale=0.85)}<div class="bits">{' '.join('ON' if b == '1' else 'off' for b in bits)}</div></td>
<td><b>{html.escape(name)}</b><div class="what">{html.escape(what)}</div>{n}</td>
<td>{html.escape(s18) if s18 else '<span class="muted">nothing</span>'}</td>
<td class="j">{jumper_svg(j18)}<br>{pill(j18)}</td>
<td>{html.escape(s33) if s33 else '<span class="muted">nothing</span>'}</td>
<td class="j">{jumper_svg(j33)}<br>{pill(j33)}</td>
</tr>""")
    return "\n".join(rows)


def function_cards():
    """One short per-function setup card (switches + wiring), for the setup chapter."""
    cards = []
    for bits, name, what, s18, j18, s33, j33, note in FUNCTIONS:
        def line(term, sensor, j):
            if not sensor:
                return f"<li>Terminal <b>{term}</b>: leave empty. No jumper cap needed.</li>"
            where = {"digital": "<b>Digital</b>", "analog": "<b>Analog</b>"}[j]
            return f"<li>Terminal <b>{term}</b>: {html.escape(sensor)} signal wire. Cap on the {term} {where} header.</li>"
        n = f'<p class="note">{html.escape(note)}</p>' if note else ""
        cards.append(f"""<div class="card">
<div class="cardhead">{dip_svg(bits, scale=1.15)}<div><h4>{html.escape(name)}</h4><div class="what">{html.escape(what)}</div></div></div>
<ul>{line('18', s18, j18)}{line('33', s33, j33)}<li>Every sensor's power wires: <b>V50</b> (+) and <b>GND</b> (−).</li></ul>{n}
</div>""")
    return "\n".join(cards)


CSS = """
@page { size: A4; margin: 18mm 16mm 20mm 16mm;
  @bottom-center { content: "Daffodil User Manual  ·  page " counter(page); font: 9pt Roboto; color: #888; } }
@page :first { @bottom-center { content: none; } }
* { box-sizing: border-box; }
body { font-family: Roboto, "Liberation Sans", sans-serif; font-size: 10.5pt; line-height: 1.45; color: #1f2328; }
h1 { font-size: 30pt; margin: 0 0 4mm; color: #1d5e3a; }
h2 { font-size: 17pt; color: #1d5e3a; border-bottom: 2px solid #1d5e3a; padding-bottom: 2mm; margin: 0 0 4mm; break-after: avoid; }
h3 { font-size: 12.5pt; margin: 6mm 0 2mm; break-after: avoid; }
h4 { margin: 0; font-size: 11.5pt; }
p { margin: 0 0 3mm; }
.chapter { break-before: page; }
.cover { height: 255mm; display: flex; flex-direction: column; text-align: center; }
.cover-head { border-bottom: 2px solid #1d5e3a; padding-bottom: 4mm; text-align: left; }
.cover-head img { height: 26mm; }
.cover-logo { margin: 30mm 0 8mm; }
.cover-logo img { width: 95mm; }
.cover p { max-width: 140mm; margin: 0 auto; }
.cover .sub { font-size: 16pt; color: #1d5e3a; margin-bottom: 10mm; }
.cover .meta { color: #777; font-size: 10pt; margin-top: 45mm; }
table { border-collapse: collapse; width: 100%; margin: 2mm 0 4mm; }
th, td { border: 1px solid #d0d7de; padding: 2mm; vertical-align: middle; text-align: left; }
th { background: #eef5f0; font-size: 9.5pt; }
tr { break-inside: avoid; }
table.big td { font-size: 9pt; padding: 1.2mm 1.6mm; line-height: 1.25; }
table.big .what { font-size: 8.2pt; }
table.big .note { font-size: 8pt; }
td.sw { text-align: center; width: 30mm; }
td.j { text-align: center; width: 20mm; }
.bits { font-size: 7.5pt; color: #666; margin-top: 1mm; letter-spacing: .2px; }
.what { color: #555; font-size: 9pt; }
.note { color: #9a6700; font-size: 8.8pt; margin-top: 1mm; }
.muted { color: #999; }
.pill { display: inline-block; padding: .3mm 2mm; border-radius: 3mm; font-size: 8pt; font-weight: 700; }
.pill.d { background: #ddeafe; color: #1f4fa8; }
.pill.a { background: #fdebd3; color: #9a4b00; }
.pill.n { background: #eee; color: #777; }
.callout { border-left: 4px solid #1f6feb; background: #f0f6ff; padding: 3mm 4mm; margin: 3mm 0 4mm; break-inside: avoid; }
.warn { border-left-color: #d29922; background: #fff8e6; }
.board { width: 100%; height: auto; margin: 2mm 0 3mm; }
.card { border: 1px solid #d0d7de; border-radius: 2mm; padding: 3mm 4mm; margin-bottom: 3mm; break-inside: avoid; }
.cardhead { display: flex; gap: 4mm; align-items: center; margin-bottom: 1mm; }
.card ul { margin: 1mm 0 0; padding-left: 5mm; }
.card li { margin-bottom: .6mm; }
.led { width: 100%; margin: 1mm 0 2mm; border-radius: 1.5mm; }
.led.narrow { width: 42%; }
.sensor { break-inside: avoid; }
.ledblock { break-inside: avoid; margin-bottom: 4mm; }
ol li, ul li { margin-bottom: 1mm; }
.two { display: flex; gap: 6mm; }
.two > div { flex: 1; }
"""


def build():
    today = datetime.date.today().strftime("%B %Y")
    body = f"""
<section class="cover">
  <div class="cover-head"><img src="assets/DigitalStablesLogo.png" alt="Digital Stables"></div>
  <div class="cover-logo"><img src="assets/Daffodil.svg" alt="Daffodil"></div>
  <div class="sub">User Manual: setting up and reading your sensor unit</div>
  <p>Daffodil watches water for you. It reads water troughs, tanks, septic tanks, water flow and
  water temperature, then reports to Digital Stables over WiFi and to nearby units over LoRa radio.
  It runs on solar power and a battery.</p>
  <div class="meta">Digital Stables · {today}</div>
</section>

<section class="chapter">
<h2>1. Getting to know your Daffodil</h2>
<p>Everything you need to set up is on the lower edge of the board:</p>
{board_svg()}
<table>
<tr><th style="width:32%">Part</th><th>What it does</th></tr>
<tr><td><b>18 / 33 terminal</b></td><td>The <b>signal</b> wires of your sensors go here. Terminal <b>18</b> is sensor 1 and terminal <b>33</b> is sensor 2.</td></tr>
<tr><td><b>V50 / GND terminal</b></td><td>Powers your sensors. Every sensor's <b>+</b> wire goes to <b>V50</b> and its <b>−</b> wire goes to <b>GND</b>. When two sensors are connected, both + wires share V50 and both − wires share GND.</td></tr>
<tr><td><b>18 and 33 jumpers</b><br>(marked "Digital&nbsp;Analog")</td><td>Each sensor terminal has two small 2-pin headers, one labelled <b>Digital</b> and one labelled <b>Analog</b>. Put one jumper cap on the header your sensor needs and leave the other one empty. The drawing above shows both caps on <b>Digital</b>.</td></tr>
<tr><td><b>Config switch (1–5)</b></td><td>Tells Daffodil what it is measuring (switches 1–4) and how often to report (switch 5). Up is <b>ON</b>.</td></tr>
<tr><td><b>SDA / SCL terminal</b></td><td>Not needed for any of the setups in this manual.</td></tr>
</table>
<div class="callout"><b>Which header?</b> Use <span class="pill d">Digital</span> for flow meters,
ultrasonic level sensors and the water-temperature probe. Use <span class="pill a">Analog</span>
only for tank pressure sensors. Never put a cap on both headers of the same terminal.</div>
<div class="callout warn"><b>Always switch the unit off</b> (unplug USB <i>and</i> the battery)
before changing wires, jumpers or switches. Daffodil only reads the config switch when it starts up.</div>
</section>

<section class="chapter">
<h2>2. Quick setup table</h2>
<p>Find what you want to measure, then set the switches, sensors and jumper caps exactly as shown.
In the switch pictures, a black knob at the top means <b>ON</b>. Switch 5, shown with a <b>?</b>,
is your choice (see section 3).</p>
<table class="big">
<tr><th>Switches 1–4</th><th>Setting</th><th>Terminal 18</th><th>18 jumper</th><th>Terminal 33</th><th>33 jumper</th></tr>
{function_rows()}
</table>
<p class="what">Jumper pictures: <b>D</b> = Digital header, <b>A</b> = Analog header. The blue
block shows where the cap goes. Any switch combination not in this table is unused, and the
unit won't measure anything in it.</p>
</section>

<section class="chapter">
<h2>3. Switch 5: how often to report</h2>
<div class="two">
<div class="card"><h4>Switch 5 ON: Battery Saver</h4>
<p>Daffodil plans its day around the sun. It sleeps longer and reports less often, so the battery
lasts as long as possible. <b>Best for most installations</b>, such as a trough level that only
needs checking a few times an hour.</p></div>
<div class="card"><h4>Switch 5 OFF: Frequent Reports</h4>
<p>Daffodil reports more often and doesn't wait for good sun. You get readings closer to real
time, but it uses more battery. Best when you need to watch something closely, such as water flow,
or when the unit has reliable power.</p></div>
</div>
<p>Either way, Daffodil protects its own battery. If the battery gets low it turns off its
lights first, then WiFi, then goes to sleep until the sun has recharged it.</p>

<h2 style="margin-top:10mm">4. Step-by-step setup</h2>
<ol>
<li>Unplug the battery and USB.</li>
<li>Set switches 1–4 for what you're measuring (section 2), and switch 5 for how often it reports.</li>
<li>Fit the jumper caps for terminals 18 and 33 as shown in section 2.</li>
<li>Connect each sensor: its signal wire to terminal 18 or 33, its + wire to <b>V50</b> and its − wire to <b>GND</b> (section 5).</li>
<li>Connect the battery (and USB or solar panel). The lights start cycling through their displays.</li>
<li>For trough and septic setups, enter the sensor's mounting height (section 6).</li>
<li>Watch the lights for a minute and check the level, internet and LoRa displays look right (section 7).</li>
</ol>
<p>The setups one by one:</p>
{function_cards()}
</section>

<section class="chapter">
<h2>5. Connecting the sensors</h2>
<div>
</div><div class="sensor"><h3>Ultrasonic level sensor (troughs and septic tanks)</h3>
<p>Use a <b>waterproof ultrasonic sensor with automatic serial (UART) output</b>, such as the
A02YYUW. JSN-SR04T and AJ-SR04M sensors also work if set to their automatic serial mode. The older
style of sensor with separate "trigger" and "echo" wires and open speakers does <b>not</b> work
with current Daffodils. Those sensors corroded in the field.</p>
<table>
<tr><th style="width:35%">Sensor wire</th><th>Connect to</th></tr>
<tr><td>VCC / + (usually red)</td><td><b>V50</b></td></tr>
<tr><td>GND / − (usually black)</td><td><b>GND</b></td></tr>
<tr><td>TX</td><td>Terminal <b>18</b>. In <i>Water Trough + Tank</i>, terminal <b>33</b>. In <i>2 Water Troughs</i>, the second sensor goes on <b>33</b>.</td></tr>
<tr><td>RX</td><td>Not connected. Tape it off.</td></tr>
</table>
<p>Jumper cap on that terminal's <span class="pill d">Digital</span> header.</p>
<div class="callout"><b>Mounting:</b> point the sensor straight down at the water, clear of the
trough walls, float valves and pipes. Keep its face at least <b>3&nbsp;cm above the highest water
level</b>, because it can't measure anything closer than that.</div>

</div><div class="sensor"><h3>Water temperature probe</h3>
<p>A waterproof DS18B20 probe, the common stainless-steel tube on a cable. It is used only in the
<i>Water Trough + Water Temperature</i> setting.</p>
<table>
<tr><th style="width:35%">Probe wire</th><th>Connect to</th></tr>
<tr><td>Red</td><td><b>V50</b></td></tr>
<tr><td>Black</td><td><b>GND</b></td></tr>
<tr><td>Yellow (data)</td><td>Terminal <b>33</b>, jumper cap on the 33 <span class="pill d">Digital</span> header</td></tr>
</table>
<p>No extra resistor is needed, because the board already has one. Put the probe tip in the water,
away from the trough's inlet, so it measures the water the animals are actually drinking.</p>

</div><div class="sensor"><h3>Flow meter</h3>
<p>A standard 3-wire hall-effect water flow meter.</p>
<table>
<tr><th style="width:35%">Meter wire</th><th>Connect to</th></tr>
<tr><td>Red</td><td><b>V50</b></td></tr>
<tr><td>Black</td><td><b>GND</b></td></tr>
<tr><td>Yellow (pulse)</td><td>Terminal <b>18</b> (flow meter 1) or <b>33</b> (flow meter 2), cap on <span class="pill d">Digital</span></td></tr>
</table>
<p>Install the meter with the arrow on its body pointing in the direction the water flows.</p>

</div><div class="sensor"><h3>Tank pressure sensor</h3>
<p>A submersible or bottom-mounted pressure sensor with a 0.5–4.5&nbsp;V output (0–5&nbsp;psi),
placed at the bottom of the tank.</p>
<table>
<tr><th style="width:35%">Sensor wire</th><th>Connect to</th></tr>
<tr><td>+ (usually red)</td><td><b>V50</b></td></tr>
<tr><td>− (usually black)</td><td><b>GND</b></td></tr>
<tr><td>Signal</td><td>Terminal <b>18</b> (tank 1) or <b>33</b> (tank 2), cap on <span class="pill a">Analog</span></td></tr>
</table>
</div>
</section>

<section class="chapter">
<h2>6. Setting up a trough or septic tank</h2>
<p>For troughs, Daffodil needs to know how high the sensor is and where you want the "low" and
"full" points. Open the Daffodil's web page, tap the gear icon on the level card, and enter:</p>
<table>
<tr><th style="width:35%">Setting</th><th>What to enter</th></tr>
<tr><td><b>Sensor height</b></td><td>Distance in cm from the sensor's face down to the <b>empty</b> trough floor.</td></tr>
<tr><td><b>Level minimum</b></td><td>Water depth in cm below which the trough counts as <b>low</b> (red light).</td></tr>
<tr><td><b>Level maximum</b></td><td>Water depth in cm above which the trough counts as <b>full</b> (blue light).</td></tr>
</table>
<p>Example: the sensor is 60&nbsp;cm above the trough floor, you want an alert below 20&nbsp;cm of
water, and you call it full above 30&nbsp;cm. Enter 60, 20 and 30. Re-enter these whenever you
move the sensor or change the trough.</p>
<p>A septic tank needs no settings. Its lights show how full the tank is, measured from a sensor
mounted at the top.</p>
</section>

<section class="chapter">
<h2>7. Reading the lights</h2>
<p>Daffodil has 15 lights in 3 rows of 5. Every few seconds the display moves on to show the next
piece of information, in this order.</p>

<div class="ledblock"><h3>Temperature</h3>
<img class="led" src="{LED_DIR}/led-temperature.png">
<p>The air temperature. The left lights count the tens and the right lights count the units.
Green means above zero, blue means below zero, and yellow means exactly 0&nbsp;°C. A red pattern
means the temperature sensor couldn't be read.</p></div>

<div class="ledblock"><h3>Water level (troughs, tanks, septic)</h3>
<img class="led" src="{LED_DIR}/led-level.png">
<p><b>Red</b> = low or critical, <b>yellow</b> = getting low (tanks and septic only), <b>green</b> = good,
<b>blue</b> = full. For troughs, red, green and blue follow the levels you set in section 6.
With two sensors, the block shifts left and a small blue dot shows which one you're seeing:
top-right for sensor 1, bottom-right for sensor 2.</p>
<div class="callout warn">A trough that <b>always</b> shows blue may mean the sensor isn't being
heard at all. Check its wiring (section 9).</div></div>

<div class="ledblock"><h3>Water flow</h3>
<img class="led narrow" src="{LED_DIR}/led-flow.png">
<p>An "F" shape. <b>Blue</b> = water is flowing, <b>red</b> = no flow right now.</p></div>

<div class="ledblock"><h3>Internet</h3>
<img class="led" src="{LED_DIR}/led-wifi.png">
<p><b>Blue</b> = connected to your WiFi. The centre light is blue when the internet can be reached
and red when it can't. <b>Green</b> = setup mode: Daffodil is broadcasting its own WiFi network for
you to connect to. <b>All red</b> = WiFi is resting to save battery.</p></div>

<div class="ledblock"><h3>LoRa radio</h3>
<img class="led narrow" src="{LED_DIR}/led-lora.png">
<p><b>Green</b> = the last radio message went out, <b>red</b> = it failed. The lights blink off
briefly each time a message is sent. That's normal.</p></div>

<div class="ledblock"><h3>Error</h3>
<img class="led narrow" src="{LED_DIR}/led-error.png">
<p>A red "E" with a blue dot. A dot top-right means an internal sensor wasn't found. A dot in the
middle-right means the unit's memory is nearly full. Either way, contact Digital Stables.</p></div>

<div class="ledblock"><h3>Battery</h3>
<img class="led" src="{LED_DIR}/led-battery.png">
<p>The "B" shows battery health: <b>blue</b> = good, <b>green</b> = OK, <b>amber</b> = low,
<b>red</b> = very low, <b>magenta</b> = no battery detected. The top-right light is green while
charging and red while running on battery. The middle-right light is amber on cloudy days, when
the unit is saving power.</p></div>
</section>

<section class="chapter">
<h2>8. Looking after your Daffodil</h2>
<ul>
<li>Keep the solar panel clean and in full sun.</li>
<li>Every few months, check the ultrasonic sensor's face is clean and the cable glands are tight.</li>
<li>If you change the battery, the config switch may need recalibrating. Use <b>Config Switch
Calibration</b> on the web page, or ask Digital Stables.</li>
<li>If you move a sensor or change a trough, re-enter its heights (section 6).</li>
</ul>

<h2 style="margin-top:10mm">9. Troubleshooting</h2>
<table>
<tr><th style="width:38%">Problem</th><th>What to check</th></tr>
<tr><td>The level never changes, or a trough always shows blue</td><td>The ultrasonic's TX wire is on the right terminal (section 2) and its cap is on <b>Digital</b>. It has power from V50/GND, and it's an <i>automatic serial</i> sensor.</td></tr>
<tr><td>Water temperature missing</td><td>The probe's yellow wire is on terminal <b>33</b> with the cap on 33 <b>Digital</b>, and switches 1–4 are set to <i>Water Trough + Water Temperature</i>.</td></tr>
<tr><td>Flow always shows red (no flow)</td><td>The meter's yellow wire is on the right terminal with its cap on <b>Digital</b>, and the meter's arrow points the way the water flows.</td></tr>
<tr><td>Tank level wrong or stuck</td><td>The cap for that terminal is on <b>Analog</b>, not Digital.</td></tr>
<tr><td>The unit measures the wrong thing</td><td>Switches 1–4 are set correctly and the unit was restarted after changing them. If they are, the switch needs recalibrating (section 8).</td></tr>
<tr><td>Internet lights all red</td><td>Normal when the battery is low or it's very cloudy. Daffodil rests WiFi to save power and tries again later.</td></tr>
<tr><td>No lights, or very dim lights</td><td>Normal at night (the lights dim), when the battery is low, and on dull days in Battery Saver mode (the lights turn off to save power). Wait for sun, or check the battery is connected.</td></tr>
</table>
<p>Still stuck? Contact Digital Stables with the unit's name and a description of what the
lights are doing.</p>
</section>
"""
    doc = f"""<!doctype html><html><head><meta charset="utf-8"><title>Daffodil User Manual</title>
<style>{CSS}</style></head><body>{body}</body></html>"""
    HTML_OUT.write_text(doc, encoding="utf-8")
    subprocess.run(["weasyprint", "-u", HERE.as_uri() + "/", str(HTML_OUT), str(PDF_OUT)], check=True)
    print(f"wrote {HTML_OUT}\nwrote {PDF_OUT}")


if __name__ == "__main__":
    build()
