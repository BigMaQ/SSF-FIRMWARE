# ISIS-Firmware — Umbau auf eigene Firmware ohne MobiFlight

**Ziel:** Identische Funktion (Grafik + Anzeige-Logik) wie die laufende CC_ISIS-Version, aber ohne
MobiFlight-Framework, mit eigenem Transport (UDP) und eigener Knopf-Anbindung.
**Basis:** `github.com/ccrawford/CC_ISIS` @ main (MIT-Lizenz, Copyright Chris Crawford mitführen).
**Hardware:** Guition ESP32-4848S040 (ESP32-S3 N16R8, 480×480 ST7701, GT911-Touch, Backlight GPIO38).
**FDC-Seite:** siehe `IP_TRANSPORT_SPEC.md`. Diese Spec betrifft ausschließlich die Panel-Firmware.

---

## 1. Ist-Analyse — was hängt wirklich an MobiFlight?

CC_ISIS ist **kein Sketch**, sondern eine **MobiFlight-Community-Device-Bibliothek**, die vom
MobiFlight-Firmware-Core eingebunden wird (es gibt keine `.ino`; `setup()`/`loop()` liefert der Core).

**Vermessene Dateien:**

| Datei | Größe | MobiFlight-Kopplung | Aktion |
|---|---|---|---|
| `CC_ISIS/CC_ISIS.cpp` | 28,4 KB | **keine** | **bleibt 1:1** |
| `CC_ISIS/CC_ISIS.h` | 590 B | keine | bleibt |
| `CC_ISIS/ISISCommon.cpp` | 15,7 KB | 3 Zeilen (`MFeeprom`), Preferences-Ansatz vorhanden | fast 1:1, Settings umbiegen |
| `CC_ISIS/ISISCommon.h` | 23,9 KB | 3 Includes + `extern MFEEPROM` | **anpassen** |
| `CC_ISIS/MFCustomDevice.cpp` | 6,4 KB | Framework-Adapter | **fällt weg** |
| `CC_ISIS/MFCustomDevice.h` | 514 B | Framework-Adapter | **fällt weg** |
| `CC_ISIS/MFCustomDevicesConfig.h` | 219 B | Device-Registrierung (`17.CC_ISIS…`) | **fällt weg** |
| `CC_ISIS/MFCustomDeviceTypes.h` | 368 B | Konstanten | Konstanten ggf. übernehmen |
| `CC_ISIS/4inchLCDConfig_Guition.h` | 3,2 KB | keine | **bleibt 1:1** |
| `Sprites/`, `Images/` (Assets) | ~1,5 MB | keine | **bleiben 1:1** |

**Ergebnis: rund 90 % des Codes ist framework-frei und bleibt unverändert.**
Die gesamte MobiFlight-Berührung besteht aus:
1. `#include "MFEEPROM.h"`, `#include "commandmessenger.h"`, `#include "MFCustomDeviceTypes.h"`
2. `extern MFEEPROM MFeeprom;` → in `loadSettings()` / `saveSettings()`
3. Der Adapterklasse `MFCustomDevice`, die `set()` weiterreicht

---

## 2. Ablage & Struktur (entschieden 13.10.2026)

**Ablage: `Python/firmware/source/isis/`** — dort liegen die anderen Panels (RMP, ATC, MIP …),
das Verzeichnis `Python/firmware/` ist ein eigenes Git-Repo mit `fwbuild.ps1` als Build-Manager.

⚠️ **Der ISIS braucht einen anderen Build-Weg als die AVR-Panels.** Diese werden von `fwbuild.ps1`
per **Arduino CLI** zu `.hex` kompiliert (und der FCC flasht sie mit **avrdude** über COM). Der ISIS
braucht dagegen **PlatformIO** (Octal-PSRAM-Flags, LovyanGFX, eigene Partitionstabelle, ESP32-S3-SDK).
→ `fwbuild.ps1` bekommt eine Regel: **Ordner mit `platformio.ini` → `pio run`**, Ergebnis-`.bin`
versioniert ablegen (`isis/isis_v<version>.bin`, analog zu `rmp/rmp_v<version>.hex`).
`-AddPanel` erkennt das Projekt am `platformio.ini` statt an der `.ino`.

**Alle Dateien bleiben im Firmware-Repo — kein separates Projekt außerhalb der Struktur.**

```
firmware/source/isis/
  platformio.ini              ; ESP32-S3, N16R8, PSRAM Octal, 240 MHz, LovyanGFX
  partitions.csv              ; ← OTA-fähig! siehe Abschnitt 13
  src/main.cpp  net.cpp/.h  proto.cpp/.h  buttons.cpp/.h  displaycfg.cpp/.h
  src/CC_ISIS.cpp/.h  ISISCommon.cpp/.h  4inchLCDConfig_Guition.h
  src/Sprites/  src/Images/   ; 1,5 MB Assets, unverändert übernommen
firmware/isis/                ; Build-Ausgabe: isis_v0.1.0.bin  (Gegenstück zu rmp/*.hex)
```

**Übernommener Code (aus dem CC_ISIS-Repo) — die vollständige Struktur:**

```
isis-fw/src/
  main.cpp                  ; NEU: setup() + loop() (ersetzt MobiFlight-Core)

```
isis-fw/
  platformio.ini              ; ESP32-S3, N16R8, PSRAM Octal, huge_app, LovyanGFX
  src/
    main.cpp                  ; NEU: setup() + loop() (ersetzt MobiFlight-Core)
    net.cpp/.h                ; NEU: WiFi + UDP + Discovery/Beacon + Frames
    proto.cpp/.h              ; NEU: Parser/Sender für "KEY:VALUE;"
    buttons.cpp/.h            ; NEU: PCF8574 (I2C_NUM_0) inkl. INT
    displaycfg.cpp/.h         ; NEU: Skalierung/Offset + DISP_BL-PWM
    CC_ISIS.cpp/.h            ; ÜBERNOMMEN (unverändert)
    ISISCommon.cpp/.h         ; ÜBERNOMMEN (angepasst, s. u.)
    4inchLCDConfig_Guition.h  ; ÜBERNOMMEN
    Sprites/ Images/          ; ÜBERNOMMEN
```

**Anpassungen in `ISISCommon.h` (die einzigen im übernommenen Code):**
- `#include "MFEEPROM.h"` → entfernen, `extern MFEEPROM MFeeprom;` → `extern Preferences isisPrefs;`
- `#include "commandmessenger.h"` → entfernen (wird nirgends sonst referenziert)
- `#include "MFCustomDeviceTypes.h"` → entfernen oder Konstanten blockweise übernehmen

**Anpassungen in `ISISCommon.cpp`:**
- `loadSettings()`: `MFeeprom.read_block(CC_ISIS_SETTINGS_OFFSET, isisSettings)`
  → `isisPrefs.getBytes("settings", &isisSettings, sizeof(isisSettings))`
- `saveSettings()` analog mit `putBytes`
- Der Kommentar „their own device-specific fields in a second Preferences session" zeigt: der
  Preferences-Weg ist im Original bereits vorgesehen. `#include <Preferences.h>` ist schon drin.
- **`sendButton()` / `sendEncoder()`** (Basisklasse) → schreiben künftig einen **Uplink-Frame** über
  UDP statt über den MobiFlight-Connector (siehe 5.)

---

## 3. Dateneingang — vollständige Liste (Stand der Original-Firmware)

`CC_ISIS::set(int16_t messageID, char *setPoint)` verarbeitet **genau acht** IDs:

| Message-ID | Feld in `isisState` | Konvertierung | INI-Kanal |
|---|---|---|---|
| 60 | `rawAirspeed` | `atof` | `dsp1` |
| 102 | `machSpeed` | `atof` | `dsp2` |
| 77 | `rawAltitude` | `atof` | `dsp3` |
| 100 | `mbPressure` | `atoi` | `dsp4` |
| 101 | `isStdPressure` | `atoi` | `dsp5` |
| 80 | `rawPitchAngle` | `atof` | `dsp6` |
| 72 | `rawBankAngle` | `atof` | `dsp7` |
| 71 | `ballPos` | `atof` | `dsp8` |

→ Das deckt **Speed, Mach, Altitude, QNH, STD-Modus, Pitch, Bank, Slip/Skid** ab. Das ist genau
Markus' Beobachtung: „Vom Sim kommt schon einiges."

**Noch NICHT in der Firmware vorhanden** (deshalb Phase 2, siehe Abschnitt 9):
Heading, Ground Speed, Ground Track, Heading-Bug, Nav-Source, CDI-Deviation/Valid/To-From,
GS-Deviation/Valid, Screen-Brightness als Datenwert, Power-Status — und die **LS-Skalen**
(`drawLS()` ist in `CC_ISIS.h` deklariert, aber **nicht implementiert**) samt den
**G/S- und LOC-Diamonds**.

### Wire-Format
Statt Message-IDs nutzt die eigene Firmware das FDC-Textprotokoll. Die Zuordnung ist **fest kodiert
und identisch zur INI-Reihenfolge**:

```
SEQ42|DSP1:280;DSP2:0.78;DSP3:35000;DSP4:1013;DSP5:0;DSP6:-3.2;DSP7:12.5;DSP8:0.1;DISP_BL:180;
```
- `DSP<n>:` → Feld gemäß obiger Tabelle, `atof`/`atoi` wie im Original (kein Wertebereich-Clipping —
  das Original rechnet intern, nicht die Firmware)
- `DISP_BL:<0-255>` → Backlight-PWM (Abschnitt 6)
- `CFG:<key>=<value>;` → Laufzeit-Konfiguration (Abschnitt 7)
- **Robustheit:** unbekannte Keys ignorieren (nicht abbrechen), Framegrenze `;`, maximal 96 Byte je
  Frame verarbeiten, Werte ohne `;` verwerfen.

---

## 4. Netzwerk (UDP)

- **Datenport 5005/UDP**, **Discovery 5006/UDP** (identisch zur FDC-Spec)
- **Beacon** beim Boot und danach alle 5 s:
  `IDENT:ISIS;MAC:<mac>;IP:<ip>;FW:<version>;HW:ESP32S3;` an `255.255.255.255:5006`
  → nach dem ersten `IDENT-ACK` Intervall auf 60 s verlängern.
- **Empfang:** eingehende Frames sofort parsen, danach `isisState` neu zeichnen (nicht pro Feld
  zeichnen — `draw()` arbeitet mit Sprites und ist teuer).
- **Heartbeat/Keepalive:** 1 Hz; fehlt der FDC > 5 s, geht die Anzeige auf einen definierten
  Zustand (Original: `PowerState`-Logik) — nicht auf Müll aus alten Werten.

### 4.1 WLAN-Konfiguration (entschieden 13.10.2026)

**Harte Regel: Es steht kein WLAN-Credential im Quellcode, im Repo oder im Hex.**
Markus' Firmware ist zum Teilen gedacht — SSID/Passwort gehören ausschließlich in die **NVS** des
Geräts. Auch nicht als Default, nicht als Kommentar, nicht in `platformio.ini`.

**Weg: SoftAP + Captive Portal (Boarding-Website), wie bei IoT-Geräten üblich.**

Auslöser für den Setup-Modus, erster Treffer gewinnt:
1. **Keine WLAN-Daten in der NVS** (Erststart) — der Regelfall.
2. **10 s kein Connect** nach dem Boot (Router neu, Netz umbenannt, Passwort geändert).
3. **Touch-Menüpunkt „Netzwerk-Setup"** — kein Tastatur-Widget, nur ein Menüeintrag, der den
   Setup-Modus startet und neu bootet. Damit kommt Markus auch an das eingebaute Panel heran, ohne
   es ausbauen zu müssen.
4. **Tastenkombination beim Einschalten** (`BRT +` und `BRT −` gleichzeitig halten) — Notfallweg,
   falls das Display/der Touch nicht mitspielt.

**Ablauf im Setup-Modus:**
- SoftAP-Name `ISIS-<letzte 4 MAC-Zeichen>`, IP `192.168.4.1`, **Captive Portal** via DNS-Catch-All
  (Android/iOS zeigen die Seite dann automatisch an, kein URL-Tippen).
- Das **Display zeigt eine Info-Seite**: AP-Name, `http://192.168.4.1`, FW-Version, MAC. So sieht
  Markus am Panel, dass es im Setup hängt.
- **Boarding-Seite (bewusst rudimentär):**
  - WLAN-Scan mit Liste + RSSI, SSID-Auswahl, Passwortfeld
  - `scale_w`, `scale_h`, `offset_x`, `offset_y` (Display-Justage ohne FDC)
  - Info: FW-Version, MAC, IP, Verbindungsstatus, RSSI
  - Buttons: Speichern + Reboot · WLAN-Daten löschen · Factory Reset
- **Timeout:** kein Zugriff/keine Speicherung innerhalb von 10 min → AP schließen und normal booten
  (sonst hängt das Panel dauerhaft im Setup-Modus).
- Nach dem Speichern: Reboot in den STA-Modus, Beacon läuft an, FDC findet das Panel.

**Parameter-Priorität (Markus' Wunsch):** `FDC (CFG: aus der INI)` > `NVS (Web-Config)` >
`Firmware-Default`. Der FDC gewinnt im Betrieb, die Web-Werte greifen, wenn kein FDC sendet.

**Implementierung:** `WiFiManager` (tzapu) mit Custom-Parametern — bewährt, Captive Portal und
Parameter-Formular sind eingebaut, ~20 Zeilen Aufwand. Alternative wäre eine eigene Minimal-Lösung
(`SoftAP` + `DNSServer` + `WebServer`, ~150 Zeilen) — nur falls WiFiManager mit dem ESP32-S3 oder
der Grafik-Performance kollidiert. **Risikoarm heißt hier: WiFiManager nehmen.**

**Kein Webserver im Normalbetrieb (Phase 1).** Im STA-Modus läuft nur UDP. Begründung: Der FDC kann
alles, was die Seite kann (`CFG:`-Werte live), und ein offener Webserver im Heimnetz ist unnötiges
Angriffsfläche. Falls die Justage ohne FDC nötig wird → Setup-Modus über den Touch-Menüpunkt.
- **Sequenznummer** aus dem Frame-Kopf verwerfen, wenn identisch zur letzten (UDP-Duplikate).

---

## 5. Knöpfe

**Hardware:** PCF8574 (8 Eingänge) an **`I2C_NUM_0` auf GPIO 1/2, 100 kHz** (der Touch bleibt auf
`I2C_NUM_1` mit 19/45 @ 400 kHz). Externe 10 kΩ Pull-ups nach 3,3 V, 100 nF je Taster,
**INT-Pin** des PCF8574 auf einen freien GPIO (z. B. 40) → Interrupt statt Polling.

**Belegung** (identisch zur ISIS-INISEktion):

| Bit | Taste | Funktion | Ziel |
|---|---|---|---|
| b1 | BARO − | QNH verringern | State + `QNHSET:` Uplink |
| b2 | BARO + | QNH erhöhen | State + `QNHSET:` Uplink |
| b3 | BARO Push | STD-Toggle / Bug-Wert deselektieren | State + `QNHSET:` Uplink |
| b4 | RST | Attitude-Reset (2 s halten, „ATT 10 s") | lokal |
| b5 | BUGS | Bug-Menü ein/aus | lokal |
| b6 | LS | LS-Skalen ein/aus | lokal |
| b7 | BRT − | Helligkeit −− | lokal (PWM) |
| b8 | BRT + | Helligkeit ++ | lokal (PWM) |

- **Alles Lokale läuft in der Firmware** (Helligkeit, Menü, LS, Attitude-Reset). Keine FDC-Kanäle,
  keine `IN`-Werte nötig — die `in1-b1…b8` der INI bleiben Reserve.
- **`sendButton()` / `sendEncoder()`** (Basisklasse) werden umgebogen: statt MobiFlight-Connector
  senden sie einen **Uplink-Frame** — `BTN:BUGS:1;` / `BTN:BUGS:0;` und `QNHSET:1013;`.
- **BARO drehen — bidirektional (entschieden 13.10.2026).** Drehrichtung + Beschleunigung
  (langsam = 1 mb, schnell = 10 mb). Der gedrehte Wert geht als `QNHSET:<mb>;` per Uplink an den FDC,
  der ihn in den Sim schreibt; der Sim liefert ihn über `dsp4` zurück. `QNHSET` wird **immer in mb
  (hPa)** gesendet — die Anzeige-Einheit (inHg/kPa/mmHg aus den Settings) ist reine Darstellung.
  - ⚠️ **Regelkreis-Falle:** Ohne Gegenmaßnahme kämpfen der lokal gedrehte Wert und der eingehende
    `dsp4`-Wert gegeneinander, und die Anzeige zappelt. **Lösung:** Nach einer lokalen BARO-Änderung
    eingehende `dsp4`-Werte für **~1 s ignorieren** (Settle-Time), danach den Sim-Wert wieder
    übernehmen. Während des Drehens führt die Firmware den Wert selbst.
  - Gleiches Muster gilt später für die Bug-Werte (`BUGSET:`), sobald der Sim sie kennt.
  - Ein Uplink-Frame **pro Rastschritt**, nicht pro Loop-Durchlauf.
- **Entprellung:** 20 ms Software-Debounce + Hardware-RC; RST braucht 2 s Haltezeit.

---

## 6. Display-Helligkeit

- `pin_bl = GPIO 38` → PWM über `ledcWrite`. Ziel-Auflösung 8 Bit (0–255), Frequenz ~5 kHz.
- `DISP_BL:<0-255>` vom FDC setzt den **Basiswert**.
- `brightnessGamma()` (im Original vorhanden) weiterverwenden, damit die Regelung linear wirkt.
- BRT +/− verändert den Basiswert lokal in Stufen (z. B. 5 %) und meldet ihn **nicht** an den FDC
  (der FDC überschreibt beim nächsten Update — bewusst so, sonst Regelkreis).
- Original hat zusätzlich einen **Photosensor**; das Board hat keinen → entfällt, `DISP_BL` ersetzt ihn.

---

## 7. Skalierung/Offset (Markus' „Schrumpfung")

Vom FDC gesendete Parameter (INI: `scale_w`, `scale_h`, `offset_x`, `offset_y`):

```
CFG:scale_w=400;CFG:scale_h=400;CFG:off_x=40;CFG:off_y=30;
```

- Die Szene wird wie bisher in die bestehenden Sprites gerendert (480×480-Koordinatensystem,
  `TFT_MAIN_TRANSPARENT = TFT_PINK` bleibt die Transparenzfarbe).
- Ausgabe dann **skaliert** via `pushImageRotateZoom` (oder äquivalent) in den Zielbereich
  `scale_*` an Position `offset_*`.
- **Wertebereich:** `scale` 240…480 px. Unter ~80 % (384 px) leiden die Bitmap-Fonts durch
  Interpolation — das ist eine bewusste Entscheidung, kein Fehler.
- Wird nur ein **Rand** verdeckt, ist `offset_*` bei `scale=480` die bessere Wahl (Schrift bleibt scharf).
- Werte kommen aus der INI und werden beim Start **und** bei Änderung gesendet → Justage im
  eingebauten Zustand ohne Neu-Flashen.
- Persistenz: letzte gültige Werte in NVS, damit das Gerät auch ohne FDC korrekt startet.

---

## 8. Settings / Persistenz

- `CC_ISIS_Settings` (Version 7, `SETTINGS_VERSION`) enthält Einheiten: `baroUnit`
  (inHg/kPa/mmHg), `speedUnits` (kt/mph/kph), `distanceUnits`, `tempUnits` + eigene Felder.
- **Ablösung von `MFEEPROM`:** `Preferences`-Namespace `"isis"`, `getBytes`/`putBytes`.
  `CC_ISIS_SETTINGS_OFFSET` (2048) entfällt — NVS braucht kein Offset.
- **Migration:** keine. Bei `version`-Mismatch → Defaults laden (wie im Original, das den
  EEPROM-Read-Fehler genauso behandelt: „EEPROM read failed, defaults already set").
- Das **Einheiten-Menü wird per Touch** bedient — Touch bleibt voll funktionsfähig (GT911 auf
  `I2C_NUM_1`), also geht das Menü unverändert weiter.

---

## 9. Nicht in Phase 1 (bewusst später)

1. **LS-Skalen + G/S- und LOC-Diamonds** — `drawLS()` fehlt in der Firmware, die Diamond-Assets
   liegen aber bereits im Repo. Erfordert zusätzliche State-Felder (CDI/GS-Deviation + Valid).
2. **Heading, Ground Speed, Ground Track, Heading-Bug, Nav-Source** — INI-Kanäle `dsp9`–`dsp13`
   sind schon angelegt, Firmware-Felder fehlen.
3. **Power-Management** (`PowerState`, `BATTERY_POWERED`, `drawBattery`, `drawShutdown`) — im
   Sim-Pit ohne Batteriebetrieb verzichtbar; nur `POWER_ON`-Pfad nötig, Rest kann stumm bleiben.
4. Weitere Display-Varianten (`4inchLCDConfig.h`, andere Panels) — nur Guition in Phase 1.

**Wichtig:** Phase 1 = **Funktionsgleichheit mit dem Original**, nicht mehr. Die Lücken bewusst
offen lassen, nicht „nebenbei" mitfixen.

---

## 10. Akzeptanzkriterien

1. **Funktionsgleichheit (Kernkriterium):** Gleicher Wertesatz eingespeist → **pixelgleiches Bild**
   wie die Original-Firmware (Vergleich per Foto/Screenshot). Durchführung: Original-Firmware mit
   MobiFlight-Connector füttern, Bild abfotografieren; eigene Firmware mit demselben Wertesatz
   (`dsp1..dsp8`) füttern, Bild vergleichen. Für mehrere Zustände: Reiseflug, Kurve, STD-Modus,
   Bug-Menü offen, LS an.
2. **Kein MobiFlight-Artefakt** im Build: kein `MFEEPROM`, `commandmessenger`, `MFCustomDevice*`
   im Quellbaum, keine MobiFlight-Lib in `platformio.ini`.
3. **Discovery:** Panel meldet sich ohne Konfiguration per Beacon, wird vom FDC als online geführt.
4. **Datenpfad:** `dsp1..dsp8` ändern die Anzeige in ≤ 100 ms.
5. **Helligkeit:** `DISP_BL` wirkt sichtbar stufenlos; BRT-Tasten verändern lokal.
6. **Skalierung:** `CFG:scale_*` verkleinert die Ausgabe sichtbar, ohne dass Sprites kaputtgehen;
   ein Neustart mit gespeicherten Werten zeigt dasselbe Bild.
7. **Knöpfe:** alle 7 Tasten lösen die richtige Aktion aus, RST nur nach 2 s Haltezeit;
   QNH-Änderung führt zu genau einem Uplink-Frame pro Schritt.
8. **Stabilität:** WLAN-Abriss > 30 s führt nicht zum Absturz; nach Rückkehr läuft die Anzeige
   wieder (Watchdog aktiv, `esp_task_wdt`).

---

## 11. Risiken

| Risiko | Wirkung | Gegenmaßnahme |
|---|---|---|
| PSRAM-Config (Octal) falsch → Sprites fehlen | Schwarzbild | `qio_opi`, `huge_app.csv`, 240 MHz exakt wie Original |
| I2C über Flachband > 30 cm | Knöpfe/Touch zicken | 100 kHz, SDA/SCL mit GND verdrillt, kurzes Kabel |
| GPU/CPU-Last bei Skalierung + 20 Hz Frames | Tearing/Ruckeln | `draw()` nur bei Änderung, Doppelbuffer, `pushImage…` statt Neuaufbau |
| Settings-Migration (Version-Bump 7) | verlorene Einheiten | Defaults + Log-Ausgabe beim ersten Start |
| FPC-Verlängerung beim Auseinanderbauen | Bildstörungen | max. 5–10 cm, vor dem Einbau testen |

---

## 12. Entscheidungen & Umsetzung (13.10.2026)

1. **Umsetzung: Harness** (Coding-Agent), **nicht** Markus — er ist Einsteiger. Konsequenz für den
   Auftrag: vollständiges PlatformIO-Projekt liefern, jede Datei, `platformio.ini` inklusive,
   keine „hier fehlt noch"-Lücken. Zusätzlich einen **Flash-Weg für Nicht-Entwickler** beschreiben
   (`pio run -t upload`, COM-Port, was bei „Board nicht gefunden" zu tun ist) und die fertige
   `.bin` samt `esptool`-Kommandozeile mitliefern.
2. **WLAN:** SoftAP + Captive Portal im Setup-Modus (Abschnitt 4.1). Keine Credentials im Code.
3. **QNH bidirektional** mit Settle-Time (Abschnitt 5).
4. **Touch-Menü bleibt** (Einheiten = Original-Funktion) und bekommt zusätzlich den Eintrag
   „Netzwerk-Setup".

**Noch offen:** Wo liegt der neue Sketch (eigenes Repo oder Unterordner im CC_ISIS-Fork)? Und wer
testet die Funktionsgleichheit — Markus per Foto-Vergleich (Abschnitt 10, Kriterium 1)?


---

## 13. Firmware-Update (OTA) — Vorbereitung für die FCC-Erweiterung

Ziel: Der FCC (heute `cockpit_frontend/FCC/utils_firmware.py`, flasht AVR-Panels per **avrdude/COM**
aus `.hex`-Dateien) soll später auch **IP-Panels wie das ISIS** aktualisieren können. Das Panel ist
im MIP eingebaut — USB-Kabel ziehen ist keine Option, also: **Over the Air.**

### ⚠️ Vor dem ERSTEN Flash entscheiden: Partitionstabelle
Die CC_ISIS-Referenz nutzt **`huge_app.csv`** — die hat **nur einen App-Slot** (`app0`, 3 MB) und
damit **kein OTA**. Wird das später umgestellt, muss das Gerät komplett neu geflasht werden
(USB, im eingebauten Zustand).
→ **Von Anfang an eine OTA-fähige Tabelle** verwenden. Bei 16 MB Flash reicht das bequem:

```
# partitions.csv — zwei App-Slots für OTA
nvs,      data, nvs,      0x9000,   0x5000,
otadata,  data, ota,      0xe000,   0x2000,
app0,     app,  ota_0,    0x10000,  0x500000,   # 5 MB
app1,     app,  ota_1,    0x510000, 0x500000,   # 5 MB
spiffs,   data, spiffs,   0xA10000, 0x5E0000,   # ~5,9 MB Reserve (NVS-Snapshots, künftige Assets)
```
Die App inkl. der 1,5 MB Bitmap-Assets liegt bei ~2,5–3 MB → passt zweimal.

### Empfohlener Update-Weg: ArduinoOTA (UDP 3232), nur auf Freigabe
- **Nicht** über einen dauerhaften Webserver (widerspricht der Härtung in Abschnitt 14).
- `ArduinoOTA` ist ein kleiner UDP-Listener und wird mit **Passwort** betrieben.
- **Standardmäßig aus.** Der FDC gibt ihn frei: `CFG:ota=1;` → Panel öffnet den OTA-Port für
  z. B. 10 min, danach automatisch wieder zu. Damit ist im Normalbetrieb **kein** Update-Port offen.
- Fortschritt/Status meldet das Panel per Uplink (`OTA:progress=42;`, `OTA:done;`), damit der FCC
  einen Fortschrittsbalken zeigen kann (wie heute beim avrdude-Flash).
- Fallback: `esptool` über USB, falls das Panel nie ins Netz kommt (Erstinbetriebnahme).

### FCC-Seite (später, wenn alles läuft)
1. Panel ist per Discovery ohnehin bekannt (IP, MAC, **FW-Version aus dem Beacon**).
2. Version gegen die verfügbaren `.bin`-Versionen vergleichen (die vorhandene Mechanik
   `scan_panels` / `parse_fw_filename` / `merge_local_and_remote` / `load_local_fw_dir` ist bereits
   da und arbeitet mit `.hex` — für IP-Panels kommt `.bin` dazu; das Datenmodell sieht das Feld
   `filename … .bin` schon vor).
3. Ablauf: Update anbieten → `CFG:ota=1;` → `.bin` per ArduinoOTA senden → Fortschritt →
   Reboot → Beacon zeigt die neue Version → Bestätigung im FCC.

---

## 14. Netzwerk-Härtung („keine IoT-Bombe")

Grundsatz: **Das Panel ist ein reiner UDP-Endpunkt und spricht nicht nach draußen.**

| Maßnahme | Umsetzung |
|---|---|
| **No-Gateway-Option** | Statische IP mit **Gateway `0.0.0.0`** → das Panel erreicht nur Geräte im eigenen Subnetz, es existiert keine Route ins Internet. Feld in der Boarding-Seite (Abschnitt 4.1) und als `CFG:net=static_nogw;` steuerbar. Default-Vorschlag für den Pit-Betrieb: **an**. |
| **Keine ausgehenden Verbindungen** | Kein NTP, keine Cloud, keine Telemetrie, keine Update-Abfragen. Die Firmware öffnet **ausschließlich** UDP-Sockets zum FDC (5005) und für den Beacon (5006). |
| **Kein Webserver im Normalbetrieb** | Webserver existiert **nur** im Setup-Modus (Abschnitt 4.1). |
| **Update-Port nur auf Freigabe** | `ArduinoOTA` startet erst nach `CFG:ota=1;` und schließt nach 10 min (Abschnitt 13). |
| **Keine Credentials im Code** | SSID/Passwort nur in der NVS (Abschnitt 4.1). |
| **Zweite Linie: Netztrennung** | Das Panel gehört in ein IoT-VLAN ohne Internet-Route (bei Markus: `Q-VLAN_HA`/`Q-VLAN_INet`-Muster). Firmware-Härtung ersetzt das nicht — beides zusammen ist der Punkt. |

**Bewusst NICHT umgesetzt:** Secure Boot / Flash-Verschlüsselung. Für ein Panel, das nur Werte
empfängt und nichts Speichert-Brisantes hält, ist der Aufwand (Schlüsselverwaltung, Neu-Flash-Zwang)
nicht verhältnismäßig. Wenn das Panel je ins öffentliche Netz müsste, neu bewerten.


---

## 15. Testen ohne Hardware (vier Ebenen)

Die Hardware ist bestellt, aber noch nicht da. Diese Reihenfolge holt das Maximum heraus:

### Ebene 1 — Kompilieren (`pio run`)
Kein Gerät nötig. Fängt Syntaxfehler, falsche Lib-Versionen, die PSRAM-Config
(`qio_opi`, Octal!) und — wichtig — **die App-Größe gegen den 5-MB-OTA-Slot**.
Wenn die App zu groß ist, muss die Partitionstabelle angepasst werden, **bevor** geflasht wird.
Gehört in die Abnahme: Build muss ohne Warnung durchlaufen, Größe dokumentiert.

### Ebene 2 — Host-Unit-Tests (PlatformIO-Env `native`)
Parser, `CFG:`-Handling, **QNH-Settle-Logik**, Beacon-Frame-Aufbau, Knopf-Bitmapping und die
`dsp`→`isisState`-Zuordnung sind reine Logik ohne Hardware und damit auf dem PC testbar.
**Architektur-Vorgabe dafür:** Transport, Display und Tasten hinter schmale Interfaces legen
(`ITransport`, `IDisplay`, `IButtons`), damit die Logik ohne ESP32-SDK kompiliert. Das kostet den
Harness wenig und macht die Firmware testbar.
⚠️ Die Interpolation der Fonts (Abschnitt 7) und der RGB-Bus lassen sich so **nicht** prüfen —
dafür braucht es Ebene 4 oder die echte Hardware.

### Ebene 3 — ISIS-Fake (sofort nutzbar, bereits verifiziert)
`FDC_IP/tools/isis_fake.py` spielt ein ISIS-Panel im Netz:
- sendet den Discovery-Beacon an `255.255.255.255:5006`
- lauscht auf `5005`, parst `SEQ<n>|DSP…;DISP_BL…;CFG…;` und zeigt den State als Konsole
- verwirft UDP-Duplikate (gleiche Sequenznummer)
- sendet Uplinks (`QNHSET:`, `BTN:BUGS:1;`, …) interaktiv: `+` `-` `p` `b` `l` `r`
- `--selftest` prüft Parser und Kanäle 1–19 ohne Netz

**Damit ist die FDC-Seite heute testbar:** Discovery, Routing, `disp_bl`-Durchreichung und die
`CFG:`-Skalierung lassen sich prüfen, ohne dass ein ESP32 existiert.

### Ebene 4 — Desktop-Grafik-Renderer (empfohlen, Aufwand mittel)
LovyanGFX kann auf **SDL2** rendern, also auf dem PC. Damit läuft die **echte** Zeichenlogik
(`CC_ISIS.cpp` unverändert!) mit einem festen Wertesatz und schreibt PNG-Snapshots — genau der
**Pixelvergleich** aus Abschnitt 10, Kriterium 1. Es ist der einzige Weg, die Grafik **vor** der
Hardware zu bewerten (und der Punkt, an dem es bisher immer scheiterte).
Umsetzung: eigener PlatformIO-Env mit `LGFX_USE_SDL`, SDL-Panel statt `Panel_ST7701_guition…`,
gleiche Sprites/Fonts. Einschränkung: Farb- und Timing-Details des echten Panels können minimal
abweichen.

### Nicht empfehlenswert: Wokwi
Der Online-Simulator kennt ESP32-S3, aber **kein RGB-Panel/ST7701** und keine PSRAM-Sprites →
für die Grafik wertlos. Für die Logik ist Ebene 2 besser.

### Wenn die Hardware da ist — Reihenfolge
1. **Zuerst die Original-Firmware flashen** (Browser-Installer) und **Referenzbilder** für die fünf
   Zustände machen (Reiseflug, Kurve, STD, Bug-Menü, LS). Diese Bilder sind der Vergleichsmaßstab
   und lassen sich später nicht mehr nachholen, sobald die eigene Firmware drauf ist.
2. Eigene Firmware flashen, gleicher Wertesatz (über den Fake oder den FDC), Bilder vergleichen.
3. Erst danach Phase-2-Themen (LS/Diamonds) anfassen.
