"""Erzeugt src/Sprites/OtaDigits.h - Ziffern 0-9 und % fuer das OTA-Overlay.

Warum eigene Bitmaps statt LovyanGFX-Font: Waehrend eines OTA-Uploads wird der
Flash beschrieben, und dabei ist der Flash-Cache aus. Alles, was als Bitmap im
Flash (.rodata) liegt, darf in dieser Zeit nicht gelesen werden - sonst
Panic-Reboot. Die Ziffern werden beim Oeffnen des OTA-Fensters einmal in PSRAM
kopiert und von dort gezeichnet (siehe main.cpp, otaArtInPsram()).
"""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

SCHRIFT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
HOEHE = 46          # Pixelhoehe der Ziffern
ZIEL = Path(__file__).resolve().parents[1] / "src" / "Sprites" / "OtaDigits.h"
ZEICHEN = "0123456789%"


def raster(ch: str):
    """Rendert ein Zeichen als 1-Bit-Bitmap (MSB zuerst, zeilenweise)."""
    font = ImageFont.truetype(SCHRIFT, HOEHE)
    box = font.getbbox(ch)
    w, h = int(box[2] - box[0]), int(box[3] - box[1])
    bild = Image.new("1", (w, h), 0)
    ImageDraw.Draw(bild).text((-box[0], -box[1]), ch, font=font, fill=1)
    daten = bytearray()
    for y in range(h):
        byte = 0
        bits = 0
        for x in range(w):
            byte = (byte << 1) | (1 if bild.getpixel((x, y)) else 0)
            bits += 1
            if bits == 8:
                daten.append(byte)
                byte = 0
                bits = 0
        if bits:
            daten.append(byte << (8 - bits))
    return w, h, bytes(daten)


zeilen = [
    '#pragma once',
    '#include <Arduino.h>',
    '',
    '// Automatisch erzeugt von scripts/ota_digits_generator.py - nicht von Hand editieren.',
    '// Ziffern fuer das OTA-Overlay: liegen im Flash, werden aber beim Oeffnen des',
    '// OTA-Fensters nach PSRAM kopiert (Flash-Lesen ist waehrend des Schreibens tabu).',
    '',
]
tabelle = []
for idx, ch in enumerate(ZEICHEN):
    w, h, daten = raster(ch)
    name = "OTA_G_" + ("PCT" if ch == "%" else ch)
    zeilen.append(f"static const uint8_t {name}[] = {{")
    for i in range(0, len(daten), 16):
        zeilen.append("    " + ", ".join(f"0x{b:02X}" for b in daten[i:i + 16]) + ",")
    zeilen.append("};")
    tabelle.append((name, w, h))

zeilen += ["", "struct OtaGlyph { const uint8_t *daten; uint8_t breite; uint8_t hoehe; };", "",
           "// Reihenfolge: 0 1 2 3 4 5 6 7 8 9 %", "static const OtaGlyph OTA_GLYPHEN[11] = {"]
for name, w, h in tabelle:
    zeilen.append(f"    {{ {name}, {w}, {h} }},")
zeilen.append("};")

ZIEL.write_text("\n".join(zeilen) + "\n", encoding="utf-8")
print(f"{ZIEL} geschrieben ({ZIEL.stat().st_size} Bytes), {len(tabelle)} Zeichen")
print("Groessen:", ", ".join(f"{c}={w}x{h}" for c, (_, w, h) in zip(ZEICHEN, tabelle)))
