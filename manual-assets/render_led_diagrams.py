#!/usr/bin/env python3
"""Render the Daffodil LED status groups as PNG diagrams for the manual.
Layout/colors mirror src/main/webapp/daffodil.html's makeLedSvg()/LED_GROUPS exactly,
so the manual and the website never disagree about what a given LED state looks like.

If daffodil.html's LED_GROUPS array changes (new state, new color, new group), update the
LED_GROUPS list below to match, then rerun this script to regenerate the PNGs in leds/.
Requires: pip install pillow (already available in this environment)."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "leds")

def hexc(h):
    h = h.lstrip('#')
    return tuple(int(h[i:i+2], 16) for i in (0, 2, 4))

BG = hexc('0c0c1a')
OFF_FILL = hexc('1a1a2e')
OFF_STROKE = hexc('2d2d4a')

def draw_panel(on_map, scale=6):
    W, H = 310, 190
    img = Image.new('RGB', (W * scale, H * scale), BG)
    d = ImageDraw.Draw(img)
    R = 18 * scale
    for i in range(15):
        col = i % 5
        row = i // 5
        x = (31 + col * 50) * scale
        y = (35 + row * 58) * scale
        color = on_map.get(i)
        if color:
            c = hexc(color)
            def blend(c, bg, a):
                return tuple(int(c[k] * a + bg[k] * (1 - a)) for k in range(3))
            d.ellipse([x - (R + 9*scale), y - (R + 9*scale), x + (R + 9*scale), y + (R + 9*scale)],
                      fill=blend(c, BG, 0.18))
            d.ellipse([x - (R + 4*scale), y - (R + 4*scale), x + (R + 4*scale), y + (R + 4*scale)],
                      fill=blend(c, BG, 0.32))
            d.ellipse([x - R, y - R, x + R, y + R], fill=c)
        else:
            d.ellipse([x - R, y - R, x + R, y + R], fill=OFF_FILL, outline=OFF_STROKE, width=int(1.5*scale))
    return img

def try_font(size):
    for path in [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()

def render_group(states, group_label, filename):
    thumb_scale = 3
    thumbs = [draw_panel(s['on'], scale=thumb_scale) for s in states]
    tw, th = thumbs[0].size
    font = try_font(22)
    title_font = try_font(28)
    n = len(states)
    margin = 24
    caption_h = 34
    total_w = n * tw + (n + 1) * margin
    title_h = 50
    total_h = title_h + th + caption_h + margin * 2
    canvas = Image.new('RGB', (total_w, total_h), (255, 255, 255))
    d = ImageDraw.Draw(canvas)
    d.text((margin, 12), group_label, font=title_font, fill=(20, 20, 20))
    x = margin
    y = title_h
    for img, s in zip(thumbs, states):
        canvas.paste(img, (x, y))
        label = s['label']
        bbox = d.textbbox((0, 0), label, font=font)
        lw = bbox[2] - bbox[0]
        d.text((x + (tw - lw) // 2, y + th + 8), label, font=font, fill=(60, 60, 60))
        x += tw + margin
    canvas.save(os.path.join(OUT_DIR, filename))
    print("wrote", filename, canvas.size)

# Keep this in sync with LED_GROUPS in src/main/webapp/daffodil.html.
LED_GROUPS = [
    {"label": "Temperature", "file": "led-temperature.png", "states": [
        {"label": "Sensor Error", "on": {1:'#ff2020',2:'#ff2020',3:'#ff2020',7:'#ff2020',12:'#ff2020'}},
        {"label": "+7°C", "on": {2:'#00e050',3:'#00e050',4:'#00e050',8:'#00e050',9:'#00e050',13:'#00e050',14:'#00e050'}},
        {"label": "-7°C", "on": {2:'#2080ff',3:'#2080ff',4:'#2080ff',8:'#2080ff',9:'#2080ff',13:'#2080ff',14:'#2080ff'}},
        {"label": "0°C", "on": {}},
    ]},
    {"label": "Flow Sensor", "file": "led-flow.png", "states": [
        {"label": "Flowing", "on": {0:'#2080ff',1:'#2080ff',2:'#2080ff',5:'#2080ff',6:'#2080ff',10:'#2080ff'}},
        {"label": "No Flow", "on": {0:'#ff2020',1:'#ff2020',2:'#ff2020',5:'#ff2020',6:'#ff2020',10:'#ff2020'}},
    ]},
    {"label": "Tank / Trough Level", "file": "led-level.png", "states": [
        {"label": "0-25% Critical", "on": {1:'#ff2020',2:'#ff2020',3:'#ff2020',6:'#ff2020',7:'#ff2020',8:'#ff2020',11:'#ff2020',12:'#ff2020',13:'#ff2020'}},
        {"label": "26-50% Warning", "on": {1:'#ffdd00',2:'#ffdd00',3:'#ffdd00',6:'#ffdd00',7:'#ffdd00',8:'#ffdd00',11:'#ffdd00',12:'#ffdd00',13:'#ffdd00'}},
        {"label": "51-75% Good", "on": {1:'#00e050',2:'#00e050',3:'#00e050',6:'#00e050',7:'#00e050',8:'#00e050',11:'#00e050',12:'#00e050',13:'#00e050'}},
        {"label": ">75% Full", "on": {1:'#2080ff',2:'#2080ff',3:'#2080ff',6:'#2080ff',7:'#2080ff',8:'#2080ff',11:'#2080ff',12:'#2080ff',13:'#2080ff'}},
        {"label": "Slot 1 (shifted+marker)", "on": {0:'#00e050',1:'#00e050',2:'#00e050',5:'#00e050',6:'#00e050',7:'#00e050',10:'#00e050',11:'#00e050',12:'#00e050',4:'#2080ff'}},
        {"label": "Slot 2 (shifted+marker)", "on": {0:'#00e050',1:'#00e050',2:'#00e050',5:'#00e050',6:'#00e050',7:'#00e050',10:'#00e050',11:'#00e050',12:'#00e050',14:'#2080ff'}},
    ]},
    {"label": "Internet Status", "file": "led-wifi.png", "states": [
        {"label": "AP Mode", "on": {1:'#00e050',2:'#00e050',3:'#00e050',5:'#00e050',9:'#00e050',11:'#00e050',12:'#00e050',13:'#00e050'}},
        {"label": "WiFi + Internet", "on": {1:'#2080ff',2:'#2080ff',3:'#2080ff',5:'#2080ff',7:'#2080ff',9:'#2080ff',11:'#2080ff',12:'#2080ff',13:'#2080ff'}},
        {"label": "WiFi, No Internet", "on": {1:'#2080ff',2:'#2080ff',3:'#2080ff',5:'#2080ff',7:'#ff2020',9:'#2080ff',11:'#2080ff',12:'#2080ff',13:'#2080ff'}},
        {"label": "WiFi Off", "on": {1:'#ff2020',2:'#ff2020',3:'#ff2020',5:'#ff2020',9:'#ff2020',11:'#ff2020',12:'#ff2020',13:'#ff2020'}},
    ]},
    {"label": "LoRa Status", "file": "led-lora.png", "states": [
        {"label": "TX OK", "on": {1:'#00e050',6:'#00e050',11:'#00e050',12:'#00e050'}},
        {"label": "TX Failed", "on": {1:'#ff2020',6:'#ff2020',11:'#ff2020',12:'#ff2020'}},
    ]},
    {"label": "Error", "file": "led-error.png", "states": [
        {"label": "ADS1115 Not Found", "on": {0:'#ff2020',1:'#ff2020',2:'#ff2020',5:'#ff2020',6:'#ff2020',10:'#ff2020',11:'#ff2020',12:'#ff2020',4:'#2080ff'}},
        {"label": "Memory Full", "on": {0:'#ff2020',1:'#ff2020',2:'#ff2020',5:'#ff2020',6:'#ff2020',10:'#ff2020',11:'#ff2020',12:'#ff2020',9:'#2080ff'}},
    ]},
    {"label": "Battery Voltage", "file": "led-battery.png", "states": [
        {"label": "≥3.28V Charging", "on": {1:'#2060ff',6:'#2060ff',7:'#2060ff',11:'#2060ff',12:'#2060ff',4:'#00e050',9:'#00e050',14:'#00e050'}},
        {"label": "≥3.28V Discharging", "on": {1:'#2060ff',6:'#2060ff',7:'#2060ff',11:'#2060ff',12:'#2060ff',4:'#ff2020',9:'#00e050',14:'#00e050'}},
        {"label": "3.18-3.28V", "on": {1:'#00e050',6:'#00e050',7:'#00e050',11:'#00e050',12:'#00e050',4:'#ff2020',9:'#00e050',14:'#00e050'}},
        {"label": "3.10-3.18V Warning", "on": {1:'#ffc800',6:'#ffc800',7:'#ffc800',11:'#ffc800',12:'#ffc800',4:'#ff2020',9:'#00e050',14:'#00e050'}},
        {"label": "<3.10V Critical", "on": {1:'#ff2020',6:'#ff2020',7:'#ff2020',11:'#ff2020',12:'#ff2020',4:'#ff2020',9:'#00e050',14:'#00e050'}},
        {"label": "Cloudy Mode", "on": {1:'#2060ff',6:'#2060ff',7:'#2060ff',11:'#2060ff',12:'#2060ff',4:'#00e050',9:'#ffc800',14:'#00e050'}},
        {"label": "Stale Forecast", "on": {1:'#2060ff',6:'#2060ff',7:'#2060ff',11:'#2060ff',12:'#2060ff',4:'#00e050',9:'#00e050',14:'#ff2020'}},
    ]},
]

if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    for g in LED_GROUPS:
        render_group(g['states'], g['label'], g['file'])
    print("done")
