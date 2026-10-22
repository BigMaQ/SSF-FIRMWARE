"""Erzeugt src/Sprites/LangText.h: Statusmeldungen als 1-Bit-Bitmaps.

Zwei Schriften, weil keine einzelne beide Skripte sauber kann:
  - DejaVu Sans Bold  -> Latein inkl. Akzente (FDC, SIM, NICHT, CONNECTÉ)
  - Noto Sans Thai    -> Thai (die Variante ohne Latin faellt auf Kreise zurueck)

Zusaetzlich wird eine Vorschau-PNG geschrieben, damit man das Ergebnis vor dem
Flashen anschauen kann (das Panel zeigt sonst erst der Mensch am Geraet).
"""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

LATIN = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
THAI = "/tmp/NotoSansThai.ttf"
ZIEL = Path("/mnt/interchange/Project A320/Python/firmware/source/isis/src/Sprites/LangText.h")
VORSCHAU = Path("/tmp/langtext_vorschau.png")

SPRACHEN = {
    "en": dict(fdc_main="FDC NOT CONNECTED", sim_main="SIM NOT CONNECTED",
               fdc_sub="waiting for FDC ...", sim_sub="FDC ok - start simulator",
               ota_main="FLASH MODE", ota_sub="do not switch off"),
    "de": dict(fdc_main="FDC NICHT VERBUNDEN", sim_main="SIM NICHT VERBUNDEN",
               fdc_sub="warten auf FDC ...", sim_sub="FDC ok - Simulator starten",
               ota_main="FLASH-MODUS", ota_sub="nicht ausschalten"),
    "fr": dict(fdc_main="FDC NON CONNECTÉ", sim_main="SIMU NON CONNECTÉ",
               fdc_sub="en attente du FDC ...", sim_sub="FDC ok - démarrer le simu",
               ota_main="MODE FLASH", ota_sub="ne pas couper"),
    "th": dict(fdc_main="FDC ไม่ได้เชื่อมต่อ", sim_main="SIM ไม่ได้เชื่อมต่อ",
               fdc_sub="กำลังรอ FDC ...", sim_sub="FDC ok - เริ่ม Simulator",
               ota_main="โหมดแฟลช", ota_sub="อย่าตัดไฟ"),
}
# fdc_*/sim_* = Verbindungsbanner, ota_* = Overlay waehrend eines Firmware-Updates
FELDER = ["fdc_main", "sim_main", "fdc_sub", "sim_sub", "ota_main", "ota_sub"]


def ist_thai(zeichen: str) -> bool:
    return "\u0e00" <= zeichen <= "\u0e7f"


def laeufe(text: str):
    """Text in Abschnitte gleicher Schrift zerlegen (Thai vs. Latein)."""
    out = []
    for zeichen in text:
        art = "th" if ist_thai(zeichen) else "lat"
        if out and out[-1][0] == art:
            out[-1][1] += zeichen
        else:
            out.append([art, zeichen])
    return out


def zeichne(text: str, groesse: int):
    """Rendert gemischten Text und liefert (Breite, Hoehe, 1-Bit-Bild)."""
    fl = ImageFont.truetype(LATIN, groesse)
    ft = ImageFont.truetype(THAI, groesse)
    aufstieg = max(fl.getmetrics()[0], ft.getmetrics()[0])
    abstieg = max(fl.getmetrics()[1], ft.getmetrics()[1])

    breite = 0
    for art, stueck in laeufe(text):
        schrift = ft if art == "th" else fl
        breite += int(schrift.getlength(stueck))

    bild = Image.new("1", (breite + 2, aufstieg + abstieg + 2), 0)
    zeichner = ImageDraw.Draw(bild)
    x = 1
    for art, stueck in laeufe(text):
        schrift = ft if art == "th" else fl
        # an der Grundlinie ausrichten, sonst tanzen Thai und Latein
        y = 1 + (aufstieg - schrift.getmetrics()[0])
        zeichner.text((x, y), stueck, font=schrift, fill=1)
        x += int(schrift.getlength(stueck))
    return bild.width, bild.height, bild


def als_bytes(bild: Image.Image):
    breite, hoehe = bild.size
    zeilen = (breite + 7) // 8
    pxl = bild.load()
    daten = []
    for y in range(hoehe):
        for xb in range(zeilen):
            byte = 0
            for bit in range(8):
                x = xb * 8 + bit
                if x < breite and pxl[x, y]:
                    byte |= 0x80 >> bit
            daten.append(byte)
    return daten


def main() -> None:
    zeilen = [
        "#pragma once",
        "// Automatisch erzeugt (scripts/langtext_generator.py). Nicht von Hand editieren.",
        "// Statusmeldungen als 1-Bit-Bitmaps. Zwei Schriften noetig: die eingebauten",
        "// GFX-Fonts koennen kein Thai, die Thai-Schrift kein Latein.",
        "",
    ]
    vorschau_bilder = []

    for sprache, texte in SPRACHEN.items():
        for feld in FELDER:
            groesse = 26 if "main" in feld else 16
            breite, hoehe, bild = zeichne(texte[feld], groesse)
            name = f"LANG_{sprache.upper()}_{feld.upper()}"
            zeilen.append(f'// "{texte[feld]}"  ({breite}x{hoehe})')
            zeilen.append(f"static const uint8_t {name}[] = {{")
            daten = als_bytes(bild)
            for i in range(0, len(daten), 16):
                zeilen.append("    " + ",".join(f"0x{b:02X}" for b in daten[i:i + 16]) + ",")
            zeilen.append("};")
            zeilen.append(f"static const int {name}_W = {breite}, {name}_H = {hoehe};")
            zeilen.append("")
            vorschau_bilder.append((f"{sprache} {feld}", bild))

    zeilen.append("struct LangBanner {")
    for feld in FELDER:
        zeilen.append(f"    const uint8_t *{feld}; int {feld}_w, {feld}_h;")
    zeilen.append("};")
    zeilen.append("")
    for sprache in SPRACHEN:
        zeilen.append(f"static const LangBanner LANG_BANNER_{sprache.upper()} = {{")
        for feld in FELDER:
            n = f"LANG_{sprache.upper()}_{feld.upper()}"
            zeilen.append(f"    {n}, {n}_W, {n}_H,")
        zeilen.append("};")
        zeilen.append("")
    zeilen.append("""// Sprachkuerzel -> Texttabelle. Unbekanntes Kuerzel faellt auf Englisch zurueck.
inline const LangBanner *langBanner(const char *code)
{
    if (code) {
        if (strcmp(code, "de") == 0) return &LANG_BANNER_DE;
        if (strcmp(code, "fr") == 0) return &LANG_BANNER_FR;
        if (strcmp(code, "th") == 0) return &LANG_BANNER_TH;
    }
    return &LANG_BANNER_EN;
}""")

    ZIEL.write_text("\n".join(zeilen) + "\n", encoding="utf-8")

    # Vorschau: alles untereinander, 2x vergroessert
    rand, hoehe_gesamt = 8, 8
    breite_gesamt = max(b.size[0] for _, b in vorschau_bilder) * 2 + 2 * rand
    for _, b in vorschau_bilder:
        hoehe_gesamt += b.size[1] * 2 + 6
    blatt = Image.new("1", (breite_gesamt, hoehe_gesamt), 0)
    y = rand
    for _, b in vorschau_bilder:
        blatt.paste(b.resize((b.size[0] * 2, b.size[1] * 2), Image.Resampling.NEAREST), (rand, y))
        y += b.size[1] * 2 + 6
    blatt.save(VORSCHAU)
    print(f"{ZIEL} geschrieben ({ZIEL.stat().st_size} Bytes)")
    print(f"Vorschau: {VORSCHAU}")


if __name__ == "__main__":
    main()
