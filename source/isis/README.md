# A320 ISIS — eigene Firmware (ohne MobiFlight)

Standby-Instrument für das A320-Cockpit auf **Guition ESP32-4848S040** (ESP32-S3,
16 MB Flash, 8 MB Octal-PSRAM, 480×480 ST7701, GT911-Touch, Backlight GPIO 38).

Die **Anzeige-/Grafiklogik ist unverändert** aus
[CC_ISIS](https://github.com/ccrawford/CC_ISIS) übernommen (MIT, Copyright © 2024
Chris Crawford / elral — siehe `LICENSE_CC_ISIS.txt`). Entfernt wurde ausschließlich
das MobiFlight-Framework; Transport (UDP) und Knopf-Anbindung (PCF8574) sind eigene
Module.

Vollständige Spezifikation: `../../docs/ISIS_FIRMWARE_SPEC.md`
Transport/Discovery: `../../docs/IP_TRANSPORT_SPEC.md`

---

## Dateien

| Datei | Herkunft | Inhalt |
|---|---|---|
| `platformio.ini` | neu | ESP32-S3, N16R8, `qio_opi`, 240 MHz, LovyanGFX 1.2.7, WiFiManager |
| `partitions.csv` | neu | **zwei** 5-MB-App-Slots → OTA ab dem ersten Flash |
| `src/main.cpp` | neu | `setup()`/`loop()`, WiFi, Display-Init, PSRAM-Canvas, Watchdog |
| `src/proto.cpp/.h` | neu | Parser `SEQ..|DSP..;CFG:..;`, Uplink-/Beacon-Frames, QNH-Settle |
| `src/net.cpp/.h` | neu | UDP 5005/5006, Beacon, Uplink, Captive Portal, OTA |
| `src/buttons.cpp/.h` | neu | PCF8574 (I2C_NUM_0, GPIO 1/2), Entprellung, RST-Hold |
| `src/displaycfg.cpp/.h` | neu | `DISP_BL`-PWM, `CFG:scale_*`/`offset_*`, NVS-Persistenz |
| `src/displaycanvas.h` | neu | 480×480-Composition-Sprite (nötig für die Skalierung) |
| `src/isisMessages.h` | übernommen | Message-ID-Konstanten (aus dem MF-Adapter-Header) |
| `src/CC_ISIS.cpp/.h` | **1:1 übernommen** | Grafik, `set()`, Sprites |
| `src/4inchLCDConfig_Guition.h` | **1:1 übernommen** | RGB-Bus, ST7701, Light_PWM GPIO 38 |
| `src/ISISCommon.cpp/.h` | übernommen, entkoppelt | Logik + Menüklassen; nur Settings/Uplink/Includes geändert |
| `src/Sprites/`, `src/Images/` | **1:1 übernommen** | Bitmap- und Font-Assets (~1,5 MB) |
| `test/` | neu | Host-Unit-Tests (PlatformIO `native`, Unity) |

---

## Bauen

PlatformIO ist nicht systemweit installiert — es gibt ein eigenes venv:

```bash
# einmalig
python3.11 -m venv ~/.venvs/pio
~/.venvs/pio/bin/pip install platformio
```

```bash
cd source/isis

# Firmware bauen  -> .pio/build/esp32s3/firmware.bin
~/.venvs/pio/bin/pio run -e esp32s3

# Host-Tests (läuft ohne Hardware)
~/.venvs/pio/bin/pio test -e native
```

Auf Windows reicht im Projektordner `pio run -e esp32s3`, wenn PlatformIO
installiert ist. `fwbuild.ps1` im Firmware-Repo baut das ISIS-Panel ebenfalls mit
(`.\fwbuild.ps1 -Build -Panel ISIS`).

### Flashen (USB, nur beim ersten Mal nötig)

```bash
pio run -e esp32s3 -t upload          # COM-Port wird automatisch gesucht
# falls nicht gefunden:
pio run -e esp32s3 -t upload --upload-port COM5
```

Alternativ die fertige Binärdatei (`isis/isis_v<version>.bin` im Repo):

```bash
esptool.py --chip esp32s3 --port COM5 --baud 921600 \
  write_flash 0x0     bootloader.bin \
             0x8000  partitions.bin \
             0xe000  boot_app0.bin \
             0x10000 isis_v0.1.0.bin
```
Alle vier Dateien entstehen im Build-Ordner `.pio/build/esp32s3/`.

> **Board wird nicht gefunden?** USB-**Daten**kabel verwenden (kein Ladekabel),
> Desktop-USB-3.0-Port (das Display zieht kurzzeitig viel Strom), Treiber für den
> USB-Serial-Chip (CH340 oder CP210x) installieren. Der ESP32-S3 hat keinen
> eigenen USB-Port frei — GPIO 19/20 liegen im RGB-Bus.

### OTA (ohne Ausbau)

Der FDC gibt den Update-Port frei: `CFG:ota=1;` → das Panel öffnet ArduinoOTA
(UDP 3232) für 10 Minuten, meldet `OTA:progress=…;` per Uplink und schließt danach
wieder. Im Normalbetrieb ist **kein** Update-Port offen.

---

## Netzwerk

| Zweck | Port | Richtung |
|---|---|---|
| Daten/Steuerung | 5005/UDP | FDC → Panel (`SEQ…|DSP…;`) und Panel → FDC (`QNHSET:…;`, `BTN:…;`) |
| Discovery | 5006/UDP | Beacon `IDENT:ISIS;MAC:…;IP:…;FW:…;HW:ESP32S3;` an 255.255.255.255 |

* Beacon alle **5 s**, nach dem ersten `IDENT-ACK` alle **60 s**.
* `IDENT?` (Scan-Menü des FDC) wird sofort unicast beantwortet.
* Doppelte Sequenznummern (UDP-Duplikate) werden verworfen.
* Fehlen Daten **> 5 s**, geht die Anzeige in einen definierten Zustand.
* **Keine ausgehenden Verbindungen** (kein NTP, keine Cloud, keine Update-Abfragen).
* `CFG:net=static_nogw;` → statische IP mit Gateway `0.0.0.0` (keine Route ins
  Internet), Werte aus der NVS.

### Erstkonfiguration (WLAN) — kein Passwort im Code

Auslöser für den Setup-Modus, erster Treffer gewinnt:

1. keine WLAN-Daten in der NVS (Erststart),
2. 10 s kein Connect,
3. **BRT +** und **BRT −** beim Einschalten gleichzeitig halten (Notfallweg).

Ablauf: SoftAP `ISIS-<letzte 4 MAC-Zeichen>`, IP `192.168.4.1`, Captive Portal;
das Display zeigt AP-Name, URL, FW-Version und MAC. Timeout 10 min.
Boarding-Seite: WLAN-Scan + Passwort, `scale_w`/`scale_h`/`offset_x`/`offset_y`,
Info, Speichern.
Zusatzaktionen per URL: `http://192.168.4.1/erasewifi` (WLAN-Daten löschen),
`http://192.168.4.1/factoryreset` (WLAN-Daten + Anzeige-Config löschen).

> Hinweis: Im Upstream-CC_ISIS ist **kein** Touch-Menü verdrahtet (`createMenuItems()`
> ist rein virtuell und wird von `CC_ISIS` nicht implementiert). Der in der Spec
> genannte Trigger „Touch-Menüpunkt Netzwerk-Setup" ist deshalb als Tastenkombination
> (Punkt 3) umgesetzt; ein Menüsystem wäre Phase-2-Arbeit.

---

## Knöpfe (PCF8574)

`I2C_NUM_0` auf **GPIO 1 (SDA) / GPIO 2 (SCL)**, 100 kHz, INT auf **GPIO 40**
(der Touch bleibt auf `I2C_NUM_1` mit 19/45 @ 400 kHz). Adresse `0x20`.
Externe 10 kΩ Pull-ups nach 3,3 V, 100 nF je Taster.

| Bit | Taste | Aktion | Ziel |
|---|---|---|---|
| b1 | BARO − | QNH − (1 mb, schnell 10 mb) | State + `QNHSET:<mb>;` |
| b2 | BARO + | QNH + | State + `QNHSET:<mb>;` |
| b3 | BARO Push | STD ein/aus | State + `BTN:BAROPUSH:…;` |
| b4 | RST | Attitude-Reset, 2 s halten | lokal |
| b5 | BUGS | Bug-Menü ein/aus | lokal |
| b6 | LS | LS-Skalen ein/aus | lokal |
| b7 | BRT − | Helligkeit − | lokal (PWM) |
| b8 | BRT + | Helligkeit + | lokal (PWM) |

* Entprellung 20 ms Software-Debounce + Hardware-RC.
* BARO: nach einer lokalen Änderung werden eingehende `dsp4`-Werte **1 s ignoriert**
  (Settle-Time gegen den Regelkreis), ein Uplink-Frame **pro Rastschritt**.
* Helligkeit wird **nicht** an den FDC gemeldet (der FDC überschreibt ohnehin).
* Fehlt der PCF8574, läuft das Display normal weiter (Lediglich keine Tasten).

---

## Anzeige-Konfiguration (`CFG:`)

```
CFG:scale_w=400;CFG:scale_h=400;CFG:off_x=40;CFG:off_y=30;
CFG:disp_bl=180;CFG:ota=1;CFG:net=static_nogw;
```

Priorität **FDC (CFG:) > NVS (Boarding-Seite) > Firmware-Default**.
`scale` wird auf 240…480 begrenzt. Ohne Skalierung (480/480, Offset 0) wird das
Bild 1:1 auf das Panel kopiert; sonst über LovyanGFX `pushImageRotateZoom`
skaliert. Letzte gültige Werte landen in der NVS, damit das Panel auch ohne FDC
richtig startet. `disp_bl` (0–255) setzt den Basishelligkeitswert
(`brightnessGamma()` wie im Original); BRT ± ändert ihn lokal in 5 %-Schritten.

---

## Bewusste Abweichungen / Phase 2

**Nicht** Teil dieser Firmware (so in der Spec, Abschnitt 9):

* `drawLS()` — im Upstream deklariert, aber nicht implementiert; damit fehlen
  LS-Skalen und die G/S-/LOC-Diamonds. Die Assets liegen bereit.
* Heading, Ground Speed, Ground Track, Heading-Bug, Nav-Source, CDI-/GS-Deviation.
* Power-Management/Batteriebetrieb (`PowerState` bleibt auf `ALWAYS_ON`).

Kleine, dokumentierte Abweichungen vom Upstream:

* **Framegrenze 512 statt 96 Byte.** Die Spec (Abschnitt 3) sagt „maximal 96 Byte
  je Frame" — das war das Erbe des MobiFlight-Serienprotokolls. Der FDC sendet
  real **155 Byte** (`SEQ1|DSP1:…;CFG:offset_y=0;`), der Schnitt lag mitten im
  `CFG`-Block: Skalierung, Offsets und `CFG:ota=1` kamen nie an. Über UDP ist das
  unkritisch (MTU ~1500), die Parse-Regeln sind unverändert (nur vollständige
  `key:value;`-Paare, unvollständiger Rest wird verworfen). Der echte Frame liegt
  als Regressionstest `test_real_fdc_frame_from_capture` in der Suite.
  → **Die Spec sollte hier nachgezogen werden.**
* `lcd` ist nicht mehr das Panel selbst, sondern ein 480×480-Composition-Sprite
  (`displaycanvas.h`). Nur so lässt sich die Szene ohne eine einzige Änderung an
  `CC_ISIS.cpp` skalieren. Ohne Skalierung ist der zusätzliche Kopierschritt
  byte-genau.
* `ISISCommon.h/.cpp`: MobiFlight-Includes entfernt, Settings von MF-EEPROM auf
  `Preferences` (Namespace `isis`) umgestellt, `sendButton()`/`sendEncoder()` auf
  UDP-Uplink umgebogen, Backslash-Includes (`"Sprites\battery.h"`) auf
  Vorwärts-Schrägstriche korrigiert (GCC).
* `CC_ISIS.cpp`, `CC_ISIS.h`, `4inchLCDConfig_Guition.h`, `Sprites/`, `Images/`
  sind **byte-identisch** zum Upstream.

### Zuordnungs-Modus (flüchtig, nie im NVS)

Der FDC schickt beim Zuordnen zwei flüchtige Keys; das Panel zeigt damit an, welches
Bedienelement gerade dran ist. Beide werden **nie** persistiert
(`displaycfgPersistMask`).

| Key | Wirkung |
| --- | --- |
| `CFG:assign=<zone>` | Wert an dieser Anzeigeposition blinken lassen. `off`/`none`/`0` beendet. |
| `CFG:assign_label=<Text>` | Klartext (max. 15 Zeichen) fürs Display — **auch ohne Zone** (Taster haben keine Anzeigeposition). |

Zonen: `baro`, `alt`, `spd`, `mach`, `att`, `bank`, `slip`, `all`.
`assign_label` bereinigt die Eingabe (Kleinbuchstaben → Groß, unlesbare Zeichen weg,
zu lang = abschneiden) und ist nie ein Fehler — wie alle `CFG:`-Keys.

---

## Tests

```bash
pio test -e native
```

61 Testfälle in 5 Suiten, ohne Hardware lauffähig: Parser (inkl. 512-Byte-Grenze,
unbekannte Keys, fehlendes `;`, Duplikat-Erkennung, **echter 155-Byte-FDC-Frame als
Regression**), `CFG:`-Handling und Priorität, QNH-Settle-Logik, Beacon-/Uplink-
Frame-Aufbau, Knopf-Bitmapping und Entprellung, Zuordnungs-Modus (`CFG:assign` /
`CFG:assign_label` inkl. Bereinigung und 15-Zeichen-Grenze). Die Grafik selbst (Font-Interpolation,
RGB-Bus) lässt sich damit nicht prüfen — dafür ist die echte Hardware nötig.

---

## Lizenz

Anzeige-/Grafikcode: MIT, Copyright © 2024 Chris Crawford / elral
(`LICENSE_CC_ISIS.txt`). Eigene Module: siehe Projektlizenz des Firmware-Repos.
