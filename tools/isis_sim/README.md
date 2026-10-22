# ISIS-Desktop-Renderer (`isis_sim`)

Führt den **unveränderten** ISIS-Grafikcode von
[ccrawford/CC_ISIS](https://github.com/ccrawford/CC_ISIS) (MIT, Copyright © Chris Crawford)
auf dem Linux-PC aus und schreibt PNG-Bilder — 480 × 480, exakt die Pixel, die auch die
Ziel-Firmware (ESP32-S3 + 4-Zoll-ST7701-RGB-Panel) auf die Glasscheibe bringt.
Zweck: die ISIS-Grafik **ohne Hardware** bewerten (Skalen, Farben, Tapes, Lage im
später beschnittenen Displaybereich) und später als **Pixel-Referenz** für den Vergleich
mit der echten Firmware dienen (ISIS_FIRMWARE_SPEC.md, Abschnitt 10 Kriterium 1 / Abschnitt 15 Ebene 4).

Status: **funktioniert** (kein Blocker, kein SDL nötig).

---

## 1. Schnellstart

```bash
cd "/mnt/interchange/Project A320/Python/firmware/tools/isis_sim"

python3 build.py -j8          # einmal bauen (kopiert Upstream nach build/src, kompiliert)
python3 render.py             # baut bei Bedarf und rendert alle Zustände aus scenes.json
ls out/                       # PNG je Zustand + out/uebersicht.png + out/uebersicht.md
```

Nur ein einzelner Zustand:

```bash
python3 render.py --no-build --scene 02_kurve
```

Ein einzelner Zustand mit **eigenen Werten** (Message-IDs des Upstream, wie sie die
MobiFlight-Anbindung benutzt):

```bash
./build/isis_render --out /tmp/x.ppm --frames 400 \
  --msg 60=280   `# Airspeed kt` \
  --msg 102=0.78 `# Mach` \
  --msg 77=35000 `# Altitude ft` \
  --msg 100=1013 `# QNH/Druck mb` \
  --msg 101=0    `# 0 = QNH-Modus, 1 = STD` \
  --msg 80=-2.5  `# Pitch (Achtung: negativ = Nase hoch, s. Abschnitt 6)` \
  --msg 72=0     `# Bank` \
  --msg 71=0     `# Slip/Skid (ballPos)`

python3 ppm2png.py out/ppm/01_reiseflug.ppm   # PPM -> PNG (falls --keep-ppm benutzt wurde)
```

Nützliche Schalter von `isis_render`:

| Schalter | Bedeutung |
|---|---|
| `--out <datei.ppm>` | Zielbild in **Ansicht wie auf dem Display** (Default `--rotate 90`, aufrecht) |
| `--out-raw <datei.ppm>` | zusätzlich der **rohe Framebuffer** (Rotation 0) für Pixelvergleiche auf Speicherebene |
| `--rotate {0,90,180,270}` | Drehung der Ansicht (90 = so, wie der Nutzer das Panel montiert) |
| `--frames N` | Anzahl `update()`-Zyklen; die Anzeige hat geglättete/animierte Elemente, daher einschwingen lassen (Default 400) |
| `--msg ID=WERT` | Upstream-Nachricht setzen, beliebig oft |
| `--brightness-menu PCT` | Brightness-Menü-Overlay (0–100 %) über das Bild legen |
| `--selftest` | prüft die Byte-Reihenfolge des Framebuffers (siehe Abschnitt 3) |
| `--quiet` | keine JSON-Zeile auf stdout |

Jeder Lauf gibt eine JSON-Zeile aus, u. a. mit dem **FNV-1a-Hash des Framebuffers** und den
tatsächlich gezeichneten `isisState`-Werten — damit sind Läufe automatisiert vergleichbar.

---

## 2. Was hier läuft (und warum es ohne Hardware geht)

```
build/src/CC_ISIS.cpp   ─┐  Upstream, unverändert (1:1-Kopie, per cmp geprüft)
build/src/ISISCommon.cpp ┘
        │  zeichnet über lcd.* wie auf dem ESP32
        ▼
host/LGFX_Desktop.h        → ersetzt NUR 4inchLCDConfig_Guition.h (Desktop-Config)
        └── host/Panel_Memory.hpp   → lgfx::Panel_FrameBufferBase (Offscreen-Framebuffer)
```

* **Kein SDL, kein Display:** Der Renderer benutzt `Panel_FrameBufferBase` — genau die
  Basisklasse, die auch das echte Panel (`Panel_ST7701_guition_esp32_4848S040` → `Panel_RGB`)
  benutzt. Nur die Speicherbelegung macht `host/Panel_Memory.hpp` selbst. Dadurch läuft
  dieselbe Pixel-Pipeline (Sprites, Transparenz, Fonts, RGB565) wie auf der Hardware.
* **Nur die Display-Config wird ausgetauscht.** `4inchLCDConfig_Guition.h` wird **nicht**
  kopiert; `host/LGFX_Desktop.h` nimmt ihren Platz ein (Bus = Dummy). Der Upstream-Sketch
  bleibt unangetastet — `prepare_build.sh` kopiert alle `.cpp/.h` nach `build/src` und
  bricht ab, wenn eine Kopie vom Original abweicht. Einzige Ausnahme: die zwei
  `#include`-Zeilen in `ISISCommon.h` mit Windows-Backslash (`Sprites\battery.h`,
  `Images\PrimaSansMid32.h`) werden auf `/` normalisiert; das Skript prüft, dass es genau
  diese zwei Zeilen sind.
* **Hardware-Details, die nachgebildet sind:**
  * Farbtiefe ist fest **RGB565** — `setColorDepth()` ist (wie bei `Panel_RGB`) wirkungslos.
  * **Byte-Reihenfolge Big-Endian** (`rgb565_2Byte`, MSB first), so wie der RGB-Parallelbus liest.
  * `setRotation(3)` des Upstream („USB-Buchse nach unten“) → der Speicher enthält das Bild
    um 90° gedreht; `--rotate 90` (Default) dreht es für die Ansicht zurück.

Nicht nachgebildet: Panel-Init-Sequenz/Timing, Backlight-PWM, Touch (CST816), Encoder/Taster
(MobiFlight-Inputs sind im Shim als No-Op bzw. per `--msg` steuerbar), EEPROM/NVS (In-Memory).

---

## 3. Byte-Order-Nachweis (wichtig für Farburteile)

```bash
./build/isis_render --selftest
# TFT_RED    0xF800 -> Speicher F8 00  (BIG-ENDIAN: hi,lo)
# TFT_GREEN  0x07E0 -> Speicher 07 E0  ...
# Ergebnis: Framebuffer ist BIG-ENDIAN RGB565 (so liest der RGB-Bus)
```

Unabhängig bestätigt durch die Assets: die generierten Sprite-Header sind laut Kopfzeile
„byte-swapped RGB565“, und der Transparenzschlüssel des Upstream (`8184` = `0x1FF8`) ist das
bytegetauschte `TFT_MAGENTA` (`0xF81F`). Wer den Framebuffer anders interpretiert, bekommt
**vertauschte Farben (Blau ↔ Orange)** — genau der Effekt, der in einem früheren Zwischenstand
auftrat. `render.py` führt den Selbsttest bei jedem Lauf aus und schreibt das Ergebnis in
`out/renders.json` und `out/uebersicht.md`.

---

## 4. Die gerenderten Zustände (`scenes.json`)

| # | Zustand | Werte (Message-IDs) | Erwartung / Beobachtung |
|---|---|---|---|
| 1 | **Reiseflug** | 60=280, 102=0.78, 77=35000, 100=1013, 101=0, 80=−2.5, 72=0, 71=0 | Horizont waagerecht, Fahrtband 280, Höhenband 35000, QNH-Kasten blau „1013“, Mach-Kasten grün „.78“ |
| 2 | **Steile Kurve** | 72=32 (Bank), 80=4, 60=300, 77=24000, 71=0.7 | Horizont geneigt, Rollzeiger ausgelenkt, Pitch-Leiter mitgedreht, Slip-Kugel aus der Mitte |
| 3 | **STD an** | 101=1, 77=11000, 60=250 | Im Druckkasten steht statt der Zahl der Text **STD** (blau) |
| 4 | **Helligkeitsmenü offen** | wie 1 + `--brightness-menu 65` | Menü-Overlay (Rahmen, Titel „Backlight“, Prozent, Balken) |
| 5 | **LS/CDI aktiv** | 10=0, 2=0.5, 3=1, 5=0.4, 6=1, 102=0.30 | **Keine** LS-Skalen/Diamanten (s. Abschnitt 6); Mach-Kasten leer, weil Mach < 0.45 |
| 6 | **LS-Probe** | wie 1, zusätzlich CDI/GS/NAV-Felder | Pixelvergleich: FNV-Hash **identisch** zu Zustand 1 |

Die Tabelle wird bei jedem Lauf als `out/uebersicht.md` neu erzeugt (mit Hashes und allen
gezeichneten `isisState`-Werten), `out/uebersicht.png` ist die Montage aller Zustände
(3 Spalten, Felder 1..n in obiger Reihenfolge).

---

## 5. Grenzen des Verfahrens

* **BUGS-Menü gibt es im Upstream nicht.** `CC_ISIS.cpp` hat kein `createMenuItems()`; das
  einzige Menü ist `BrightnessMenu` aus `ISISCommon.h` — deshalb Zustand 4 als Ersatz.
* **LS-Skalen gibt es im Upstream nicht.** `drawLS()` ist in `CC_ISIS.h` nur *deklariert*,
  in `CC_ISIS.cpp` **nicht implementiert** und wird **nirgends aufgerufen**. Die Felder
  `cdiOffset`/`cdiValid`/`gsDeviation`/`gsValid`/`navSource` (Message 2/3/5/6/10) werden von
  keiner Zeichenroutine gelesen. Beleg: Zustand 6 ist mit gesetzten CDI/GS/NAV-Feldern
  **pixelgleich** zu Zustand 1 (FNV-1a gleich). → Falls die Ziel-Firmware LS/Diamanten
  braucht, ist das **Neuentwicklung**, nichts zum „Portieren“.
* Die Anzeige ist animiert/geglättet; einzelne Elemente brauchen ein paar hundert
  `update()`-Zyklen. Alle Szenen nutzen `--frames 400` und melden `"converged": true`
  (Hash zwischen Zyklus 399 und 400 unverändert).
* Antialiasing/Font-Rendering entspricht dem LovyanGFX-Build für den Host; ein Pixelvergleich
  gegen die echte Firmware sollte daher auf demselben LovyanGFX-Stand (hier 1.2.7) erfolgen.

---

## 6. Fachliche Auffälligkeiten (aus den Bildern gemessen)

* **Pitch-Vorzeichen:** im Code dokumentiert — *„A negative isisState.pitchAngle (nose up)
  moves the horizon down“*. `pitchAngle = −2.5` stellt also **Nase hoch** dar. Die Anbindung
  (FDC/Sim) muss das Vorzeichen entsprechend drehen.
* **Bank-Vorzeichen — gegenläufig zwischen Zeiger und Horizont:** bei `bank = +32°` wandert
  der Rollzeiger nach **rechts** (Standardkonvention „Rechtskurve“), die Horizontlinie kippt
  aber mit dem **rechten Ende nach unten**. Ein realer ADI zeigt in einer Rechtskurve das
  rechte Ende **nach oben**. Gemessen am Bild: Steigung der Horizontlinie `+0.6175` für
  `bank=+32` (⇒ Neigung `+31,7°`, Sollwert 32° — die Betragsrechnung im Renderer stimmt also
  exakt); Rollzeiger links/rechts wechselt korrekt mit dem Vorzeichen. **Vor der
  FDC-Anbindung klären, welches Vorzeichen der Sim liefert** — sonst zeigt entweder der
  Zeiger oder der Horizont verkehrt.
* Konstante aus dem Code, die ebenso für die Ziel-Firmware gilt: `ATT_HORIZON = 194`,
  Pitch-Maßstab `PITCH_SCALE = 8 px/Grad`, Horizontlinie 2 px, `CENTER_X = 160`/`CENTER_Y = 192`
  im Attitude-Sprite, Farben `HORIZON_COLOR`, `GROUND_COLOR`, `SKY_COLOR` aus `CC_ISIS.h`.

---

## 7. Pixelvergleich gegen die echte Firmware (später)

1. Auf dem Gerät denselben Zustand herstellen (Message-IDs über MobiFlight senden).
2. Display fotografieren oder den Framebuffer auslesen; auf 480 × 480 bringen.
3. Mit dem Renderer-Bild vergleichen: gleicher Zustand = gleicher FNV-1a-Hash aus der
   JSON-Zeile, sonst pixelweise Differenz bilden (Referenzbilder liegen in `out/`).
   Hinweis: Die **Ansicht wie montiert** ist `--rotate 90`; der **rohe Speicher** ist
   `--rotate 0` (`--out-raw`). Für einen Framebuffer-Dump auf dem Gerät ist `--rotate 0`
   die richtige Referenz.

---

## 8. Verzeichnisse

```
isis_sim/
├── prepare_build.sh     kopiert Upstream-Quellen nach build/src (mit cmp-Kontrolle)
├── build.py             Build (g++/gcc direkt; Makefile entfiel wegen Leerzeichen im Pfad)
├── render.py            rendert alle Zustände -> out/
├── ppm2png.py           PPM(P6) -> PNG (reines Python, ohne PIL)
├── scenes.json          Wertesätze/Beschreibung der Zustände
├── host/
│   ├── LGFX_Desktop.h   Desktop-Ersatz für 4inchLCDConfig_Guition.h
│   ├── Panel_Memory.hpp Offscreen-Panel (Panel_FrameBufferBase) + PPM-Ausgabe
│   ├── render_main.cpp  CLI-Renderer (Werte einspeisen, Bild schreiben, JSON)
│   └── stubs/           Arduino.h, Preferences.h, MFEEPROM.h, commandmessenger.h
├── third_party/CC_ISIS  Upstream-Klon (MIT) — wird nie verändert
├── third_party/LovyanGFX  LovyanGFX 1.2.7
├── build/               Objektdateien + build/isis_render
└── out/                 PNG/JSON/Markdown-Ergebnisse
```

## 9. Lizenz

Der Grafikcode bleibt MIT von Chris Crawford (`third_party/CC_ISIS/LICENSE`, Copyright-Hinweis
in den Quellen). Dieses Verzeichnis enthält **nur Host-Werkzeuge** (Shims, Desktop-Config,
Renderer, Skripte) und keine veränderten Upstream-Dateien.
