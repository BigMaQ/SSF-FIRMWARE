#!/usr/bin/env python3
"""render.py — Desktop-Renderer fuer das A320-ISIS: Zustaende rendern, PNG erzeugen.

Fuehrt den unveraenderten CC_ISIS-Grafikcode (LovyanGFX, ESP32-S3, 480x480) auf
dem PC aus. Werte kommen aus scenes.json (oder direkt per Kommandozeile an
build/isis_render, siehe --help dort).

Aufrufe:
    python3 render.py                      # bauen + alle Zustaende aus scenes.json
    python3 render.py --no-build           # nur rendern (Binary existiert schon)
    python3 render.py --scene 02_kurve     # nur einen Zustand
    python3 render.py --scenes meine.json --out out2
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

import ppm2png

ROOT = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(ROOT, "build", "isis_render")

# --- Mini-Bitmap-Font (3x5) nur fuer die Nummerierung der Montage -------------
DIGITS3X5 = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "111", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "010", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
}


def draw_text(buf, img_w, x, y, text, scale, color, bg=(0, 0, 0)):
    for ch in text:
        glyph = DIGITS3X5.get(ch)
        if glyph is None:
            x += 4 * scale
            continue
        for gy, row in enumerate(glyph):
            for gx, c in enumerate(row):
                if c != "1":
                    continue
                for dy in range(scale):
                    for dx in range(scale):
                        px = x + gx * scale + dx
                        py = y + gy * scale + dy
                        o = (py * img_w + px) * 3
                        if 0 <= o < len(buf) - 2:
                            buf[o] = color[0]
                            buf[o + 1] = color[1]
                            buf[o + 2] = color[2]
        x += 4 * scale


def make_montage(tiles, cell=480, cols=3, border=8, bg=(20, 20, 20)):
    rows = (len(tiles) + cols - 1) // cols
    W = cols * cell + (cols + 1) * border
    H = rows * cell + (rows + 1) * border
    buf = bytearray(bytes(bg) * (W * H))
    for i, (w, h, rgb) in enumerate(tiles):
        cx = border + (i % cols) * (cell + border)
        cy = border + (i // cols) * (cell + border)
        for y in range(min(h, cell)):
            src = rgb[y * w * 3:(y + 1) * w * 3]
            o = ((cy + y) * W + cx) * 3
            buf[o:o + len(src)] = src
        draw_text(buf, W, cx + 10, cy + 10, str(i + 1), 4, (255, 230, 0))
    return W, H, bytes(buf)


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default=os.path.join(ROOT, "scenes.json"))
    ap.add_argument("--out", default=os.path.join(ROOT, "out"))
    ap.add_argument("--scene", action="append", default=None, help="nur diese ID(s)")
    ap.add_argument("--frames", type=int, default=None)
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--keep-ppm", action="store_true", help="PPM-Zwischendateien behalten")
    args = ap.parse_args()

    cfg = json.load(open(args.scenes, encoding="utf-8"))
    scenes = cfg["scenes"]
    if args.scene:
        want = set(args.scene)
        scenes = [s for s in scenes if s["id"] in want]
        if not scenes:
            print("keine passende Szene gefunden", file=sys.stderr)
            return 2

    if not args.no_build:
        r = run([sys.executable, os.path.join(ROOT, "build.py"), "-j", str(min(8, os.cpu_count() or 4))])
        sys.stdout.write(r.stdout)
        if r.returncode != 0:
            sys.stderr.write(r.stderr)
            return r.returncode
    if not os.path.exists(BIN):
        print("Binary fehlt: %s (erst bauen)" % BIN, file=sys.stderr)
        return 2

    os.makedirs(args.out, exist_ok=True)
    ppm_dir = os.path.join(args.out, "ppm")
    os.makedirs(ppm_dir, exist_ok=True)

    # 0) Byte-Order-Nachweis in den Bericht aufnehmen
    st = run([BIN, "--selftest"])
    selftest_ok = (st.returncode == 0)
    print(st.stdout.strip())

    records = []
    tiles = []
    t0 = time.time()
    for s in scenes:
        ppm = os.path.join(ppm_dir, s["id"] + ".ppm")
        cmd = [BIN, "--out", ppm, "--frames", str(args.frames or s.get("frames", 400))]
        for mid, val in s.get("msgs", {}).items():
            cmd += ["--msg", "%s=%s" % (mid, val)]
        if s.get("brightness_menu") is not None:
            cmd += ["--brightness-menu", str(s["brightness_menu"])]
        r = run(cmd)
        if r.returncode != 0:
            print("Fehler bei %s:\n%s%s" % (s["id"], r.stdout, r.stderr), file=sys.stderr)
            return r.returncode
        rec = json.loads(r.stdout.strip().splitlines()[-1])
        rec["id"] = s["id"]
        rec["name"] = s["name"]
        rec["description"] = s.get("description", "")

        w, h, rgb = ppm2png.read_ppm(ppm)
        png = os.path.join(args.out, s["id"] + ".png")
        ppm2png.write_png(png, w, h, rgb)
        rec["png"] = os.path.relpath(png, ROOT)

        # Optionaler Pixelvergleich gegen einen frueheren Zustand (gleicher Hash
        # = gleiches Bild). Damit laesst sich z.B. belegen, dass die LS-/CDI-Felder
        # die Anzeige NICHT veraendern.
        if s.get("compare_with"):
            other = next((r for r in records if r["id"] == s["compare_with"]), None)
            if other is not None:
                rec["compare_with"] = s["compare_with"]
                rec["compare_equal"] = (other["fnv1a"] == rec["fnv1a"])

        records.append(rec)
        tiles.append((w, h, rgb))
        extra = ""
        if "compare_equal" in rec:
            extra = ", Pixelvergleich mit %s: %s" % (
                rec["compare_with"], "IDENTISCH" if rec["compare_equal"] else "unterschiedlich")
        print("  %-18s -> %s  (%dx%d, fnv1a %s, converged=%s%s)"
              % (s["id"], rec["png"], w, h, rec["fnv1a"], rec["converged"], extra))

    # 1) Montage aller Zustaende
    mw, mh, mrg = make_montage(tiles)
    montage = os.path.join(args.out, "uebersicht.png")
    ppm2png.write_png(montage, mw, mh, mrg)
    print("  Montage: %s (%dx%d, Felder 1..%d = obige Reihenfolge)" % (
        os.path.relpath(montage, ROOT), mw, mh, len(tiles)))

    # 2) Tabelle + Rohdaten
    with open(os.path.join(args.out, "renders.json"), "w", encoding="utf-8") as f:
        json.dump({"source": cfg.get("source", {}), "selftest_ok": selftest_ok,
                   "records": records}, f, indent=2, ensure_ascii=False)

    md = ["# ISIS-Desktop-Renderer — gerenderte Zustaende", "",
          "Quelle des Zeichencodes: %s" % cfg.get("source", {}).get("repo", "?"), "",
          "Byte-Order-Selbsttest des Framebuffers: %s" % ("bestanden" if selftest_ok else "FEHLGESCHLAGEN"),
          "", "| # | Zustand | Bild | Pitch | Bank | Fahrt | Höhe | QNH/STD | Mach | Slip | FNV-1a |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for i, rec in enumerate(records, 1):
        stt = rec["state"]
        qnh = "STD" if stt["std"] else "%d mb" % stt["qnh_mb"]
        md.append("| %d | %s | `%s` | %.1f° | %.1f° | %.0f kt | %.0f ft | %s | %.2f | %+.2f | `%s` |"
                  % (i, rec["name"], os.path.basename(rec["png"]), stt["pitch"], stt["bank"],
                     stt["airspeed"], stt["altitude"], qnh, stt["mach"], stt["slip"], rec["fnv1a"]))
    md += ["", "## Zustaende", ""]
    for i, rec in enumerate(records, 1):
        md += ["### %d. %s (`%s`)" % (i, rec["name"], rec["id"]),
               "", rec["description"], "",
               "Eingespeist (Upstream-Message-IDs): `%s`" % json.dumps(rec["msgs"], ensure_ascii=False),
               "", "Gezeichnet: `%s`" % json.dumps(rec["state"], ensure_ascii=False), ""]
    with open(os.path.join(args.out, "uebersicht.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")

    if not args.keep_ppm:
        for s in scenes:
            p = os.path.join(ppm_dir, s["id"] + ".ppm")
            if os.path.exists(p):
                os.remove(p)
        try:
            os.rmdir(ppm_dir)
        except OSError:
            pass

    print("\nfertig in %.1fs — %d Zustaende, Ausgabe in %s" % (time.time() - t0, len(records), args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
