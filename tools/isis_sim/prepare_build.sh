#!/usr/bin/env bash
# prepare_build.sh — baut build/src als Uebersetzungsverzeichnis auf.
#
# Der Upstream-Clone (third_party/CC_ISIS) wird NIE angefasst. Es werden Kopien
# nach build/src gelegt und jede Kopie wird unmittelbar danach bitweise
# verglichen (cmp), zusaetzlich wird der Git-Stand geprueft. Damit ist belegbar,
# dass der Renderer den echten Upstream-Code uebersetzt.
#
# Zwei Abweichungen sind unvermeidlich und werden hier EXAKT belegt:
#
#  A) 4inchLCDConfig_Guition.h wird NICHT kopiert, sondern durch
#     host/LGFX_Desktop.h ersetzt (Hardware-Panel -> Speicher-Framebuffer).
#     Der Upstream sieht das ueber `#ifdef USE_GUITION_SCREEN` selbst vor.
#
#  B) ISISCommon.h enthaelt zwei Includes mit Windows-Pfadtrenner:
#         #include "Sprites\\battery.h"
#         #include "Images\\PrimaSansMid32.h"
#     GCC auf Linux versteht Backslash im Header-Namen NICHT als Trennzeichen,
#     und ein Dateiname mit Backslash laesst sich auf diesem SMB-Share gar nicht
#     anlegen. In der Build-Kopie werden deshalb genau diese zwei Zeilen auf '/'
#     normalisiert. Der Skript bricht ab, wenn mehr als diese zwei Zeilen
#     abweichen — CC_ISIS.cpp und CC_ISIS.h (die komplette Zeichenlogik) sind
#     davon nicht betroffen und bleiben per sha256 identisch.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
UP="$ROOT/third_party/CC_ISIS/CC_ISIS"
HOST="$ROOT/host"
DST="$ROOT/build/src"

if [ ! -d "$UP" ]; then
    echo "Fehler: $UP fehlt. Erst 'make fetch' laufen lassen." >&2
    exit 1
fi

rm -rf "$DST"
mkdir -p "$DST"

# 1) Upstream-Quellen und -Header kopieren (ISISCommon.h s. Schritt 1b,
#    Hardware-Display-Config s. Schritt 4)
n=0
for f in "$UP"/*.h "$UP"/*.cpp; do
    b="$(basename "$f")"
    case "$b" in
        4inchLCDConfig_Guition.h|ISISCommon.h) continue ;;
    esac
    cp "$f" "$DST/$b"
    cmp -s "$f" "$DST/$b" || { echo "Fehler: Kopie weicht ab: $b" >&2; exit 1; }
    n=$((n + 1))
done

# 1b) ISISCommon.h mit der einzigen erlaubten Abweichung (2 Include-Zeilen)
python3 - "$UP/ISISCommon.h" "$DST/ISISCommon.h" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
raw = open(src, "rb").read()
lines = raw.split(b"\r\n")
changed = []
for i, ln in enumerate(lines):
    if ln.startswith(b'#include "') and b"\\" in ln:
        new = ln.replace(b"\\", b"/")
        changed.append((i, ln, new))
        lines[i] = new
open(dst, "wb").write(b"\r\n".join(lines))
if len(changed) > 2:
    print("Fehler: mehr als die erwarteten 2 Include-Zeilen geaendert", file=sys.stderr)
    sys.exit(1)
for i, o, n in changed:
    print("   normalisiert (Zeile %d): %s -> %s" % (i + 1, o.decode(), n.decode()))
# Gegenprobe: alles andere muss bitgleich sein
a = open(src, "rb").read().split(b"\r\n")
b = open(dst, "rb").read().split(b"\r\n")
if len(a) != len(b):
    print("Fehler: Zeilenzahl geaendert", file=sys.stderr); sys.exit(1)
diff = [i + 1 for i in range(len(a)) if a[i] != b[i]]
if diff != [i + 1 for i, _, _ in changed]:
    print("Fehler: unerwartete Aenderungen in Zeilen %s" % diff, file=sys.stderr); sys.exit(1)
print("   ISISCommon.h: nur %d Zeile(n) abweichend, sonst bitidentisch" % len(diff))
PY

# 2) Assets: Unterordner Sprites/ und Images/ 1:1 spiegeln
cp -r "$UP/Sprites" "$DST/Sprites"
cp -r "$UP/Images"  "$DST/Images"

# 3) Host-Stubs: Arduino/NVS/EEPROM/CmdMessenger-Ersatz fuer den PC
for f in "$HOST"/stubs/*.h; do
    cp "$f" "$DST/$(basename "$f")"
done

# 4) Desktop-Display-Config an Stelle der Guition-Config
cp "$HOST/LGFX_Desktop.h" "$DST/4inchLCDConfig_Guition.h"

# --- Nachweis --------------------------------------------------------------
echo "== Upstream-Clone =="
if [ -d "$ROOT/third_party/CC_ISIS/.git" ]; then
    echo "  HEAD: $(git -C "$ROOT/third_party/CC_ISIS" log -1 --format='%H %ad %s' --date=short)"
    dirty="$(git -C "$ROOT/third_party/CC_ISIS" status --short)"
    if [ -n "$dirty" ]; then
        echo "  ACHTUNG: Upstream-Clone hat lokale Aenderungen:" >&2
        echo "$dirty" >&2
        exit 1
    fi
    echo "  git status: sauber (Upstream unveraendert)"
fi

echo "== sha256 Abgleich (build/src <-> Upstream) =="
for f in CC_ISIS.cpp CC_ISIS.h ISISCommon.cpp MFCustomDeviceTypes.h; do
    a="$(sha256sum "$UP/$f" | cut -d' ' -f1)"
    b="$(sha256sum "$DST/$f" | cut -d' ' -f1)"
    [ "$a" = "$b" ] || { echo "  ABWEICHUNG bei $f" >&2; exit 1; }
    echo "  ok  $f  $a"
done

echo "build/src vorbereitet: $n Upstream-Dateien + Assets + Host-Stubs + Desktop-Config"
