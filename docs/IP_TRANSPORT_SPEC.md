# FDC IP-Transport — Implementierungsauftrag

**Status:** Vorbereitung, noch nicht implementiert
**Basis:** aktueller FDC-Stand (refactort, funktionsfähig)
**Ziel-Ordner:** `Python/FDC_IP/` = **Kopie** des aktuellen FDC. Das laufende `Python/FDC/` bleibt unverändert und funktionsfähig.

---

## 0. Ziel

Panels über **IP (UDP)** anbinden — parallel zum bestehenden Serial-Pfad, nicht als Ersatz.
Mehrere IP-Panels gleichzeitig (ISIS, PFD, ND, ECAM, VOR, IAS, …).
**Gleiche Zuordnungslogik wie heute**: `FXCEO.INI` hält die LVAR↔HW-Zuordnung, Knöpfe im Format `IN<n>-b<m>`, Displaywerte als `DSP<n>`, Display-Helligkeit als `disp_bl` (→ `DISP_BL:`).
Die Darstellung/„Show" passiert **in der Panel-Firmware** — der FDC liefert nur Werte.

Panels sollen **ohne Konfigurationsorgie** gefunden werden: kein Scan über 254 Adressen, kein manuelles Eintragen von IPs als Normalfall.

---

## 1. Ausgangslage (darf nicht kaputtgehen)

- **Serial:** `infra/multi_serialmgmt.py` (`MultiSerialManager`) — ein COM-Port pro Panel, Textprotokoll `KEY:VALUE;`
- **Panel-Identität:** läuft über die **USB-Seriennummer**, nicht über die COM-Nummer → `infra/inimgmt.py` (`last_com`, SN-Backfill, COM-Wechsel-Erkennung, COM-Konflikt-Auflösung, Auto-Assign)
- **INI-Ebenen:**
  - `INI/FXCEO.INI` — pro Panel-Sektion: `in<n>-b<m> = L:…`, `dsp<n> = L:…`, `led_* = L:…`, `bl`/`an` + `.modifier`, `in_bit_order`, `com`, `hubhop`
  - `INI/panels_default.ini` / `panels_user.ini` — Panel-Parameter (`bl`, `an`, `disp_bl`, `assigned`, …)
  - `INI/hardware.ini` — `[PANEL-HW]` VID/PID→Board, `[PANELS]` COM→Slug, `[PANELS_HW]`, `[PANELS_HWREV]`
- **Protokoll-Kommandos:** `BL:`, `AN:`, `DISP_BL:`, `DSP<n>:`, `LED…`, `IN<n>-b<m>`, `VER:`, `IDENT:`, `RESET`, `REQ`
- **Wichtig:** Serial bleibt **unverändert** nutzbar. IP ist ein zusätzlicher Transport. Bestehende Tests müssen grün bleiben.

---

## 2. Neue Datei & Struktur

- **`infra/ip_transport.py`** — eigener IP-Panel-Manager (UDP-Socket, Discovery, Routing, Heartbeat). **Nicht** in `multi_serialmgmt.py` hineinbauen.
- **Transport-Abstraktion** (dünn halten):
  ```python
  class PanelTransport(Protocol):
      def send(self, target: str, payload: str) -> None: ...
      def is_online(self, target: str) -> bool: ...
  ```
  `target` ist immer der **Panel-Slug** (nicht COM, nicht IP) → Auflösung über die Panel-Registry.
  Implementierungen: `SerialTransport` (Wrapper um den bestehenden Manager) und `IpTransport`.
  Auswahl pro Panel über INI: `transport = serial|ip`.

---

## 3. Protokoll (UDP, Text — konsistent zum Serial-Protokoll)

- **Ports:** 5005/UDP Daten & Steuerung · 5006/UDP Discovery-Broadcast (beide konfigurierbar)
- **Downlink (FDC→Panel):** gebatcht wie heute, `;`-terminiert:
  `SEQ42|DSP3:35000;DSP6:-2.5;DSP7:12.5;DISP_BL:180;`
- **Uplink (Panel→FDC):** `IN1-B5:1;` (Knopf), `STATUS:1;`, `FW:1.0.3;`, `RSSI:-62;`
- **Sequenznummer** im Kopf; Duplikate verwerfen. Prüfsumme optional (8 bit), da UDP verlieren/duplizieren kann.
- **Raten:** Displays 10–20 Hz · PFD/ND 20–30 Hz · Heartbeat 1 Hz · Buttons/Events sofort (kein Polling nötig — anders als bei RS485!)
- **Kein TCP für die Nutzdaten.** TCP nur für Konfiguration/Firmware-Update (OTA).

---

## 4. Discovery / Ident-Verfahren (Kernanforderung)

**Stufe 1 — Beacon (Primär, passiv, kein Scan):**
1. Panel sendet beim Boot und danach alle 5 s einen UDP-Broadcast an `255.255.255.255:5006`:
   `IDENT:ISIS;MAC:AA:BB:CC:DD:EE:FF;IP:192.168.20.71;FW:1.0.3;HW:ESP32S3;`
2. Der FDC **lauscht permanent** auf 5006 und baut eine Tabelle `MAC → (IP, Slug, last_seen)`.
3. Zuordnung **MAC → Panel-Slug** in `INI/hardware.ini` — exakt analog zur heutigen USB-SN-Logik:
   ```ini
   [PANELS_IP]
   mac_aabbccddeeff = ISIS
   ```
4. Nach dem ersten Kontakt sendet der FDC ein `IDENT-ACK:ISIS;` → Panel verlängert das Beacon-Intervall (z. B. 60 s) oder stoppt.
5. Ist der MAC gemappt, ist **keine IP-Konfiguration** nötig (`ip = AUTO`). Die aktuelle IP wird wie heute `last_com` mitgeführt (IP-Wechsel unproblematisch, z. B. per DHCP).

**Stufe 2 — Scan-Menü (Fallback, nur auf Knopfdruck):**
Menü mit **Von/Bis**-Range, Timeout, parallelem Probe (`IDENT?` an jede Adresse), Ergebnisliste mit MAC/FW, Zuordnung per Klick. Nie automatisch im Hintergrund.

**Stufe 3 — feste IP (Notnagel):** `ip = 192.168.20.71` direkt in der Panel-Sektion.

**mDNS/Bonjour:** nicht nötig (unter Windows unzuverlässig) — optional später ergänzbar.

---

## 5. INI-Erweiterungen

Pro Panel-Sektion in `FXCEO.INI`:

```ini
transport = ip
ip = AUTO                        ; AUTO | 192.168.20.71
port = 5005
mac = AA:BB:CC:DD:EE:FF          ; optionale harte Bindung
disp_bl = L:…                    ; Display-Helligkeit als OUTPUT-Kanal
disp_bl.modifier = *254
```

Heutiges Verhalten: unbekannte Keys landen im Parser als `MappingType.MISC` (`domain/lvar_mapping.py`, Fallback am Ende von `parse_mapping`) → **kein Crash**, aber sie tauchen als MISC auf. Der Parser muss daher **lernen**: `transport`, `ip`, `port`, `mac`, `disp_bl`, `disp_bl.modifier`.

Die ISIS-Sektion ist in `INI/FXCEO.INI` bereits angelegt (19 `dsp`-Kanäle, 8 Knöpfe, `disp_bl`) — LVARs sind **Platzhalter** (`L:ISIS_*`) und müssen noch mit echten Fenix-Quellen belegt werden.

---

## 6. Display-Brightness (`disp_bl`) — die konkreten Stellen

1. `_extract_panel_output_config` (in `ui/simconnect.py`): `disp_bl` mitlesen wie `bl`/`an`/`blg`
2. Feld-Durchreichung bis in die Output-Sammlung
3. `output_lvars`-Liste erweitern → **sonst kein Abo** und wegen `ON_SET + CHANGED` (`infra/wasm_interface.py:277`) **nie ein Wert**
4. **Clamp pro Panel-Typ:** MAX7219 nur **0–15**, LCD-Devices 0–255 → Panel-Parameter `disp_bl.range` (Default `0-255`)
5. Sende-Block mit Change-Detection: `DISP_BL:<level>;`
6. `INI/panels_default.ini`: `disp_bl` für RMP, ATC, OHP_PRI ergänzen (MIP/ACP haben es bereits)
7. **Firmware ist vorbereitet:** RMP, ATC, MIP, MIP_v2, OHP_PRI, OHP_SEC, RUD und ACP akzeptieren `DISP_BL:` **und** `DSP_BL:` (Toleranz-Patch 10/2026)

---

## 7. SimVar-Support — Blocker für ISIS/PFD/ND

`_normalize_lvar_name` (`infra/wasm_interface.py:434`) und `_normalize_lvar` (`infra/simconnect_bridge.py:107`) hängen **immer** `L:` an, wenn das Präfix fehlt:

```python
return lvar_name if str(lvar_name).upper().startswith("L:") else f"L:{lvar_name}"
```

→ `A:PLANE PITCH DEGREES` wird zu `L:A:PLANE PITCH DEGREES` = **kaputt**.

**Fix:** Präfix-Whitelist (`L:`, `A:`, `S:`, `E:`, `K:`) unverändert durchreichen; nur bei fehlendem Präfix `L:` ergänzen. Zusätzlich Read-Pfad für SimVars (`MF.SimVars.Add.(A:AIRSPEED INDICATED, knots)`).

**Warum zwingend:** Pitch, Bank, Altitude, Airspeed, Mach und QNH sind **flugzeugübergreifende SimVars** — für PFD/ND/ISIS unverzichtbar und nicht sinnvoll als LVAR nachzubauen.

---

## 8. Options-Menü (FDC)

Neue Seite „IP Settings":
- Netzwerk-Interfaces/Adresse des Rechners, UDP-Port
- Liste bekannter IP-Panels: Slug, IP, MAC, FW, online/offline, letzter Kontakt
- „Scan" mit Von/Bis-Range + Ergebnisliste → Panel zuordnen
- Pro Panel: feste IP erzwingen, MAC-Bindung, Beacon zurücksetzen

---

## 9. ISIS-Zielbelegung (Kanal-Mapping)

Quelle: `CC_ISIS/Community/devices/CC_ISIS.device.json` + `ISISCommon.h` (MIT-Lizenz).
Wertebereiche und Empfehlungen für Fenix:

| INI | Bedeutung | Message-ID | Empfohlene Quelle |
|---|---|---|---|
| `dsp1` | Airspeed (kt) | 60 | `A:AIRSPEED INDICATED` |
| `dsp2` | Mach | 102 | `A:AIRSPEED MACH` |
| `dsp3` | Altitude (ft) | 77 | `A:INDICATED ALTITUDE` |
| `dsp4` | QNH (mb) | 100 | `A:KOHLSMAN SETTING MB` |
| `dsp5` | Baro Mode STD (0/1) | 101 | Fenix-LVAR (Platzhalter) |
| `dsp6` | Pitch (°) | 80 | `A:PLANE PITCH DEGREES` |
| `dsp7` | Bank (°) | 72 | `A:PLANE BANK DEGREES` |
| `dsp8` | Slip/Skid (Ball) | 71 | `A:INCIDENCE BETA` (zu verifizieren) |
| `dsp9` | Heading | common 9 | `A:PLANE HEADING DEGREES MAGNETIC` |
| `dsp10` | Ground Speed | common 7 | `A:GPS GROUND SPEED` |
| `dsp11` | Ground Track | common 8 | `A:GPS GROUND MAGNETIC TRACK` |
| `dsp12` | AP Heading Bug | common 0 | `A:AUTOPILOT HEADING LOCK DIR` |
| `dsp13` | Nav Source | common 10 | Flugzeug-spezifisch (Platzhalter) |
| `dsp14` | CDI Lateral Deviation | common 2 | Flugzeug-spezifisch (Platzhalter) |
| `dsp15` | CDI Needle Valid | common 3 | Flugzeug-spezifisch (Platzhalter) |
| `dsp16` | CDI To/From | common 4 | Flugzeug-spezifisch (Platzhalter) |
| `dsp17` | GS Deviation | common 5 | Flugzeug-spezifisch (Platzhalter) |
| `dsp18` | GS Needle Valid | common 6 | Flugzeug-spezifisch (Platzhalter) |
| `dsp19` | Power On/Off (0/1) | common 13 | FDC-intern (Power-Status) |
| `disp_bl` | Display-Helligkeit | common 12 | `L:A_DISPLAY_BRIGHTNESS_*` (Fenix) |

**Knöpfe** (`in1-b1…b8`, `in_bit_order = LSB`) — Belegung nach dem echten ISIS-Panel:
`b1` BARO −, `b2` BARO +, `b3` BARO Push (STD), `b4` RST, `b5` BUGS, `b6` LS, `b7` BRT −, `b8` BRT +
(Die Panel-Beschriftung liegt auf dem Panel: „+"/„−" links, „BUGS"/„LS" oben, „RST" unten, „BARO"-Drehknopf rechts unten mit Push.)

### Knöpfe: Mechanik & Anbindung

**Faktenlage (AviationHunt + FlyByWire-Doku, 10/2026):**
- **`+` / `−`** = Display-Helligkeit **und** Box-Navigation im BUGS-Menü (doppelt belegt!). Im Original regelt zusätzlich eine **Photosensor-Zelle** die Helligkeit automatisch, +/- korrigiert manuell.
- **`LS`** = LS-Skalen ein/aus
- **`BUGS`** = Bug-Menü (SPD BUG: 4 Werte in kt · ALT BUG: 2 Werte in ft); Werte werden gespeichert beim Verlassen oder nach 15 s Inaktivität
- **`RST`** (ATT) = Attitude-Reset, min. 2 s drücken, Flugzeug muss waagerecht stehen, „ATT 10 s"; zusätzlich alle 350 h Betriebsstunden fällig
- **BARO-Drehknopf** = QNH **und** Bug-Wert-Einstellung; **BARO-Push** = STD bzw. Bug-Wert deaktivieren („OFF")

**Wichtig für die Architektur:** Fast alle diese Funktionen sind **ISIS-intern** — Helligkeit, Menü,
LS-Skalen, Attitude-Reset und das Bug-Menü laufen komplett im Gerät. Nur die **QNH-Änderung** muss zum
Sim (gemeinsame Baro-Referenz), und die Bugs nur dann, wenn der Sim sie kennen soll.
→ Das passt zu „die Show passiert in der Firmware": Die IN-Kanäle der ISIS-Sektion sind die **Option B**
(FDC-zentrale Auswertung), nicht der Zwang.

**Empfohlen: PCF8574 (I2C, 8 Eingänge) am vorhandenen Touch-I2C-Bus des Guition-Boards** — genau die
7 Knöpfe, kein eigenes Arduino-PCB, keine Firmware-Duplikation, minimale Latenz.
- Adressen kollidieren nicht (GT911: 0x5D/0x14 · PCF8574: 0x20–0x27), Pull-ups sind am Bus vorhanden.
- QNH-Änderung meldet die Firmware als **Uplink-Frame** `QNHSET:1013;` an den FDC → der ruft den
  Sim-Baro-Event auf. Analog optional `BUGSET:…;`.
- Alternative (nur falls FDC-zentrale Logik gewünscht): separater AVR-Input-Knoten wie die
  bestehenden Panels → dann greifen die `IN1-B1…b8` der ISIS-Sektion direkt.

**Knopf-Hinterleuchtung:** Im Original **nicht belegt** — weder AviationHunt noch die FlyByWire-Doku
noch das Panel-Rendering zeigen hinterleuchtete Knöpfe; die weißen Beschriftungen stehen im Kontrast
zum selbstleuchtenden Display. Empfehlung: **keine LED-Kanäle** vorsehen (spart Kanäle, das Display
dominiert bei Nacht). Nötigenfalls später `led_*`-Kanäle nachrüsten.

---

## 10. Akzeptanzkriterien

- **Serial-Regression:** bestehende Panels laufen unverändert, pytest-Suite grün
- **Discovery:** ein IP-Panel wird **ohne IP-Eingabe** erkannt (Beacon) und im UI als online geführt
- **Multi-Panel:** mindestens zwei IP-Panels gleichzeitig, getrennte Datenströme, keine Kreuzkopplung
- **`disp_bl`:** kommt als Output am Panel an (im Log/Test nachweisbar), clamped korrekt (0–15 vs. 0–255)
- **SimVar:** ein `A:…`-Eintrag in einer Panel-INI liefert Werte (Log-Nachweis)
- **Qualität:** neue Tests in `tests/`, `ruff` clean, kein Secret im Diff (CI prüft pytest/ruff/Secret-Scan)
- **Kein Eingriff** in `Python/FDC/` — alles im neuen Ordner

---

## 11. Offene Punkte (mit Markus klären)

1. **Ordner-Name** (`FDC_IP`?) und ob eigenes Git-Repo oder Branch im bestehenden Repo
2. **Welcher „refact"-Stand** genau die Basis sein soll (Hinweis: `_archive/background_service_refac.py`, `_archive/simconnect_refac.py` existieren als archivierte Refactor-Versionen)
3. **ISIS-Knöpfe:** Anbindungsweg (PCF8574 / separater AVR-Knoten / Touch) — siehe 9
4. **Datenrate** PFD/ND (20 oder 30 Hz) und ob Konfiguration über TCP laufen soll
5. **Firmware-Seite:** Wer implementiert den UDP-Stack im ESP32 (ISIS-Sketch)? Gehört zum ISIS-Umbau, nicht zum FDC-Auftrag.

---

## 12. ISIS-Hardware-Seite (ESP32-S3 Guition 4848S040) — Stand 13.10.2026

### Pin-Situation (aus `CC_ISIS/4inchLCDConfig_Guition.h`)
**Belegt:** GPIO 0, 3–21 (RGB-Daten D0–D15 + DE/VSYNC/HSYNC/PCLK), **38** (Backlight-PWM),
39/47/48 (SPI für Panel-Init), 19/45 (Touch-I2C GT911 @ 400 kHz), GPIO 26–37 (Octal-PSRAM + Flash —
PSRAM ist Pflicht für die Sprite-Layer).
→ **Sicher frei: GPIO 1 und 2.** Zusätzlich 40/41/42 (JTAG — freigebbar, wenn kein JTAG-Debug nötig).
→ **Der ESP32-S3 hat zwei I2C-Controller:** Touch bleibt auf `I2C_NUM_1` (19/45, 400 kHz), die Knöpfe
  kommen an **`I2C_NUM_0` auf GPIO 1/2 mit 100 kHz**. Damit stören sie den Touch nicht.
→ GPIO 19/20 sind die nativen USB-Pins (20 liegt im RGB-Bus) → USB-OTG nicht nutzbar; der COM-Port
  zum Flashen kommt vom onboard USB-Serial-Chip.

### Knopf-Elektronik — kein TTL/CMOS-Board nötig
- **PCF8574** (8 Eingänge) oder PCF8575 (16, Reserve) an **3,3 V** — passive Taster brauchen **keine
  Pegelwandlung**, weil der ESP32-S3 ebenfalls 3,3 V fährt. Die „Logik" ist ein Pull-up + RC.
- **Externe 10 kΩ Pull-ups nach 3,3 V** sind Pflicht (die internen 100 µA des PCF8574 reichen nicht),
  **100 nF pro Taster** gegen Prellen.
- **INT-Pin des PCF8574 nutzen** (active low bei Pegeländerung) → auf einen freien GPIO (z. B. 40) →
  Interrupt statt Polling, kein Tastendruck geht verloren.
- Adressen kollidieren nicht: GT911 (0x5D/0x14) vs. PCF8574 (0x20–0x27).
- **Flachband-Abgriff:** SDA/SCL **je mit GND verdrillt** führen, Kabel < 30 cm halten (Bus-Kapazität
  < 400 pF), 100 kHz. Der Touch-Bus mit 400 kHz ist für kurze Leiterbahnen ausgelegt → nicht mitbenutzen.

### Display-Helligkeit
`pin_bl = GPIO 38` → **PWM**. `DISP_BL:0-255` mappt auf `ledcWrite(GPIO38)`. Keine LED-Kanäle nötig —
Helligkeit, Menü, LS und Attitude-Reset laufen komplett im Gerät.

### Grafik-Schrumpfung (INI-Parameter, Markus' Vorgabe)
Neue Parameter in der ISIS-Sektion, vom FDC beim Start/Änderung als `CFG:`-Frame gesendet:
`scale_w`, `scale_h` (Ziel-Renderbereich in px, 480 = unbeschnitten), `offset_x`, `offset_y`.
Firmware rendert in einen Offscreen-Sprite und gibt skaliert aus (LovyanGFX `pushImageRotateZoom`).
⚠️ Unter ~80 % leiden die Bitmap-Fonts durch Interpolation. Wird nur ein **Rand** verdeckt, ist
**Offset** die bessere Wahl als Skalierung.

### Ausschnitt & Montage (Plexi-Blende)
Bezugsgröße ist die **Baugruppe**, nicht das Display (siehe `isis-integration.md`).
Empfohlener Display-Ausschnitt: **80 × 90 mm** (Display 76,92 × 78,55 mm → 1,5 mm bzw. 5,7 mm Luft).
Ein Ausschnitt **100 × 80 mm** (B × H) ist in der **Höhe** zu knapp (0,7 mm je Seite) — bei 100 mm Höhe
ist die Breite das knappe Maß. Knopföffnungen (2 × +/−, BUGS, LS, RST, BARO-Drehknopf) gehören als
**eigene Bohrungen** ins Plexi, nicht in den großen Ausschnitt; die Taster sitzen auf dem PCB-Frame
dahinter und ragen durch die Plexi-Öffnungen.

### Alternative „auseinanderbauen"
LCD per FPC von der Trägerplatine trennen (nur 76,92 mm muss durchs Loch, die 86,5-mm-Platine
verschwindet). ⚠️ Der RGB-Bus läuft mit 10 MHz über den FPC — eine Verlängerung jenseits ~5–10 cm
riskieren Bildstörungen. Vor dem Einbau testen, nicht danach.


---

## 13. FCC-Erweiterung: IP-Panels updaten (späterer Schritt)

Der FCC (`Python/cockpit_frontend/FCC/utils_firmware.py`) flasht heute AVR-Panels per **avrdude**
über COM aus `.hex`-Dateien (GitHub-Releases + lokal gemergt, `trigger_bootloader` per 1200-Baud-Touch).

**Ziel:** Auch IP-Panels (ISIS, später PFD/ND) aktualisieren — **ohne Ausbau**, also OTA.
Details auf der Firmware-Seite: `ISIS_FIRMWARE_SPEC.md`, Abschnitt 13 (OTA + Partitionstabelle).

- **Discovery liefert die Ziel-Liste:** Panel-Slug, IP, MAC und **FW-Version** (Beacon: `FW:x.y.z`).
- **Versionsvergleich** gegen die lokal/remote verfügbaren `.bin` — die Mechanik
  (`scan_panels`, `parse_fw_filename`, `merge_local_and_remote`) ist vorhanden und arbeitet aktuell
  mit `.hex`; `.bin` kommt dazu.
- **Ablauf:** Update anbieten → `CFG:ota=1;` → `.bin` per ArduinoOTA (UDP 3232) senden → Fortschritt
  (Panel meldet `OTA:progress=…`) → Reboot → Beacon bestätigt die neue Version.
- **Wichtig:** Der `fwbuild.ps1`-Weg muss IP-Panels mitbauen (PlatformIO statt Arduino CLI, siehe
  `ISIS_FIRMWARE_SPEC.md`, Abschnitt 2).
