# ISIS-Desktop-Renderer — gerenderte Zustaende

Quelle des Zeichencodes: https://github.com/ccrawford/CC_ISIS

Byte-Order-Selbsttest des Framebuffers: bestanden

| # | Zustand | Bild | Pitch | Bank | Fahrt | Höhe | QNH/STD | Mach | Slip | FNV-1a |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Reiseflug (Cruise) | `01_reiseflug.png` | -2.5° | 0.0° | 280 kt | 35000 ft | 1013 mb | 0.78 | +0.00 | `34d981726b9af5ec` |
| 2 | Steile Kurve (Bank 32 Grad) | `02_kurve.png` | 4.0° | 32.0° | 300 kt | 24000 ft | 1013 mb | 0.80 | +0.70 | `ca2512977564be36` |
| 3 | STD-Modus an | `03_std.png` | 0.0° | 0.0° | 250 kt | 11000 ft | STD | 0.70 | +0.00 | `0b39199b5c067b50` |
| 4 | Helligkeitsmenue offen (Ersatz fuer BUGS-Menue) | `04_helligkeitsmenu.png` | -2.5° | 0.0° | 280 kt | 35000 ft | 1013 mb | 0.78 | +0.00 | `9c0f6a21d68fb5aa` |
| 5 | LS/CDI aktiv (NAV-Quelle, Needles valid) | `05_ls_cdi.png` | 0.0° | 0.0° | 200 kt | 5000 ft | 1002 mb | 0.30 | +0.00 | `a4176c9fcb989dea` |
| 6 | LS-Probe: wie 01, nur CDI/GS/NAV gesetzt | `06_ls_probe.png` | -2.5° | 0.0° | 280 kt | 35000 ft | 1013 mb | 0.78 | +0.00 | `34d981726b9af5ec` |

## Zustaende

### 1. Reiseflug (Cruise) (`01_reiseflug`)

Referenzzustand: Horizont waagerecht, leichte Nase-hoch-Lage. Fahrtband links mit Marke bei 280, Hoehenband rechts mit 35000, QNH-Kasten (blau) zeigt 1013, Mach-Kasten (gruen) zeigt .78. Rollzeiger in der Mitte.

Eingespeist (Upstream-Message-IDs): `[[60, "280"], [102, "0.78"], [77, "35000"], [100, "1013"], [101, "0"], [80, "-2.5"], [72, "0"], [71, "0"]]`

Gezeichnet: `{"pitch": -2.5, "bank": 0.0, "airspeed": 280.0, "altitude": 35000.0, "qnh_mb": 1013, "std": 0, "mach": 0.78, "slip": 0.0, "navSource": 1, "cdiValid": 1, "gsiValid": 1, "brightness": 100}`

### 2. Steile Kurve (Bank 32 Grad) (`02_kurve`)

Horizont um 32 Grad geneigt, Rollzeiger auf der Rollskala ausgelenkt, Pitch-Leiter mitgedreht (10/20-Grad-Linien), Slip/Skid-Kugel leicht aus der Mitte. Prueft die Rotationsmathematik der Attitude-Anzeige.

Eingespeist (Upstream-Message-IDs): `[[60, "300"], [102, "0.80"], [77, "24000"], [100, "1013"], [101, "0"], [80, "4"], [72, "32"], [71, "0.7"]]`

Gezeichnet: `{"pitch": 4.0, "bank": 32.0, "airspeed": 300.0, "altitude": 24000.0, "qnh_mb": 1013, "std": 0, "mach": 0.8, "slip": 0.7, "navSource": 1, "cdiValid": 1, "gsiValid": 1, "brightness": 100}`

### 3. STD-Modus an (`03_std`)

Baro-Modus 101 = 1: Im Druckkasten unten rechts muss statt der Zahl der Text STD stehen (Schriftgroesse 1.2). Sonst identisch zum Reiseflug, damit nur dieser Unterschied auffaellt.

Eingespeist (Upstream-Message-IDs): `[[60, "250"], [102, "0.70"], [77, "11000"], [100, "1013"], [101, "1"], [80, "0"], [72, "0"], [71, "0"]]`

Gezeichnet: `{"pitch": 0.0, "bank": 0.0, "airspeed": 250.0, "altitude": 11000.0, "qnh_mb": 1013, "std": 1, "mach": 0.7, "slip": 0.0, "navSource": 1, "cdiValid": 1, "gsiValid": 1, "brightness": 100}`

### 4. Helligkeitsmenue offen (Ersatz fuer BUGS-Menue) (`04_helligkeitsmenu`)

Das Upstream-ISIS hat KEIN BUGS-Menue (kein createMenuItems() in CC_ISIS.cpp). Einzig vorhanden ist BrightnessMenu aus ISISCommon.h — hier als Overlay ueber den Reiseflug gelegt (65 %). Zeigt die Menue-Grafik (Rahmen, Titel Backlight, Prozentwert, Balken) korrekt an oder nicht.

Eingespeist (Upstream-Message-IDs): `[[60, "280"], [102, "0.78"], [77, "35000"], [100, "1013"], [101, "0"], [80, "-2.5"], [72, "0"], [71, "0"]]`

Gezeichnet: `{"pitch": -2.5, "bank": 0.0, "airspeed": 280.0, "altitude": 35000.0, "qnh_mb": 1013, "std": 0, "mach": 0.78, "slip": 0.0, "navSource": 1, "cdiValid": 1, "gsiValid": 1, "brightness": 65}`

### 5. LS/CDI aktiv (NAV-Quelle, Needles valid) (`05_ls_cdi`)

Erwartung: KEINE LS-Skalen und KEINE Deviations-Diamanten. drawLS() ist in CC_ISIS.h nur deklariert, in CC_ISIS.cpp nicht implementiert und wird nirgends aufgerufen; die CDI-/GS-Felder (Message 2/3/5/6/10) werden von keiner Zeichenroutine gelesen. Hier zusaetzlich Mach 0.30 (< 0.45), wodurch drawMach() den Mach-Kasten leert.

Eingespeist (Upstream-Message-IDs): `[[60, "200"], [102, "0.30"], [77, "5000"], [100, "1002"], [101, "0"], [80, "0"], [72, "0"], [71, "0"], [10, "0"], [2, "0.5"], [3, "1"], [5, "0.4"], [6, "1"]]`

Gezeichnet: `{"pitch": 0.0, "bank": 0.0, "airspeed": 200.0, "altitude": 5000.0, "qnh_mb": 1002, "std": 0, "mach": 0.3, "slip": 0.0, "navSource": 0, "cdiValid": 1, "gsiValid": 1, "brightness": 100}`

### 6. LS-Probe: wie 01, nur CDI/GS/NAV gesetzt (`06_ls_probe`)

Pixelbeweis zur LS-Frage: exakt die Werte von 01_reiseflug (gleiche Mach 0.78, gleiche Hoehe/Fahrt), zusaetzlich NAV-Quelle 10=0, CDI-Offset 2=0.5, CDI-valid 3=1, GS-Deviation 5=0.4, GS-valid 6=1. Bleibt der FNV-Hash gleich dem von 01_reiseflug, veraendern diese Felder das Bild nicht — es gibt also keine LS-Skalen/Diamanten im Upstream.

Eingespeist (Upstream-Message-IDs): `[[60, "280"], [102, "0.78"], [77, "35000"], [100, "1013"], [101, "0"], [80, "-2.5"], [72, "0"], [71, "0"], [10, "0"], [2, "0.5"], [3, "1"], [5, "0.4"], [6, "1"]]`

Gezeichnet: `{"pitch": -2.5, "bank": 0.0, "airspeed": 280.0, "altitude": 35000.0, "qnh_mb": 1013, "std": 0, "mach": 0.78, "slip": 0.0, "navSource": 0, "cdiValid": 1, "gsiValid": 1, "brightness": 100}`

