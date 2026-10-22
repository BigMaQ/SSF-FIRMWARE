# ISIS-Panel flashen — eigene Firmware (v0.1.0)

Stand: 19.10.2026. Diese Datei ist die Flash-Anleitung für **beide** Wege
(Einzeldatei und PlatformIO). Voraussetzung: USB-**Daten**kabel am PC,
Display stromversorgt über einen USB-3.0-Port am Mainboard (nicht am Hub).

## Was geflasht wird

| Datei | Wohin | Größe |
|---|---|---|
| `isis_v0.1.0_merged.bin` | Offset **0x0** — alles in einer Datei | 1.204.704 B |

md5 `dbae50206e1503a017fb7c067112bf27` — **am 21.10.2026 auf Hardware verifiziert**
(ESP32-S3, 16 MB Flash, 8 MB PSRAM; bootet, zeigt `FW 0.1.0`, geht in den Setup-AP).
⚠️ Das Vorgänger-Artefakt vom 14.10. hatte dieselbe Größe, aber einen Boot-Loop —
Groesse ist kein Beweis. Immer den md5 vergleichen.

Die Einzeldatei enthält Bootloader (0x0), Partitionstabelle (0x8000),
`boot_app0` (0xE000) und die App (0x10000). Nachweisbar zusammengesetzt aus
`.pio/build/esp32s3/` — Byte-Vergleich aller vier Teile identisch.

Partitionstabelle (aus `partitions.bin` dekodiert, nicht aus der CSV):

```
nvs       0x009000  0x005000
otadata   0x00e000  0x002000
app0      0x010000  0x500000   <- läuft
app1      0x510000  0x500000   <- OTA-Ziel
spiffs    0xa10000  0x5f0000
```

App: 1.139.168 B = 1,09 MB von 5 MB je Slot (22 %). Passt zweimal → OTA ab Werk.

## Weg A — fertige Datei (empfohlen, kein Compiler nötig)

Auf dem Windows-PC, an dem das Display hängt:

```powershell
py -m pip install --user esptool
py -m esptool --chip esp32s3 erase_flash
py -m esptool --chip esp32s3 --port COM5 --baud 921600 write_flash 0x0 "I:\Project A320\Python\firmware\isis\isis_v0.1.0_merged.bin"
```

* `COM5` durch den echten Port ersetzen (`mode` in der Eingabeaufforderung
  listet ihn, oder Geräte-Manager → Anschlüsse).
* Ohne `--port` sucht esptool selbst — beim ESP32-S3 oft erfolgreich.
* `erase_flash` ist beim **Umstieg von der Fremd-Firmware** Absicht: es löscht
  das alte NVS-Image und die alte OTA-Verteilung. Der Panel-Start danach ist
  reproduzierbar (Setup-AP startet, weil keine WLAN-Daten da sind).
* **Kein Download-Modus?** Der ESP32-S3-Devkit schaltet selbst um. Wenn nicht:
  BOOT-Taste halten, USB einstecken, loslassen — dann erscheint der Port.

Danach steht in der Ausgabe `Hash of data verified.` und `Hard resetting via
RTS pin...`.

## Weg B — PlatformIO (wenn am PC installiert)

Im Projektordner `firmware/source/isis/`:

```powershell
pio run -e esp32s3 -t upload
pio run -e esp32s3 -t upload --upload-port COM5    # falls Port nicht gefunden
```

PlatformIO kennt alle Offsets selbst — der Weg ist der bequemste, wenn das
Projekt auf dem Rechner liegt und gebaut werden soll.

## Nach dem ersten Start (Pflichtschritt)

1. Das Display zeigt eine Setup-Seite: SoftAP **`ISIS-<letzte 4 MAC-Zeichen>`**,
   IP `192.168.4.1`, FW-Version, MAC. Timeout 10 Minuten.
   Auslöser erneut: **BRT +** und **BRT −** gleichzeitig beim Einschalten halten,
   oder 10 s ohne WLAN-Verbindung.
2. Mit Handy/PC in den AP einloggen → Captive Portal → Haus-WLAN (VLAN 40)
   auswählen, Passwort eintippen, speichern.
3. Panel bootet neu, holt sich eine IP per DHCP und schickt alle **5 s** einen
   Beacon `IDENT:ISIS;MAC:…;IP:…;FW:0.1.0;HW:ESP32S3;` an UDP **5006**.
4. Im FDC: **Scan** im IP-Panel-Dialog → das Panel antwortet sofort auf
   `IDENT?` und erscheint in der Liste. Danach Sektion `[ISIS]` in `FXCEO.INI`
   mit der gelernten IP (oder `ip = AUTO`).

## Verifikation

* **Netz:** `py -m esptool --chip esp32s3 chip_id` vor dem Flash = Beweis, dass
  der PC den Chip sieht. Nach dem Flash zählt der Beacon: im FDC-Log
  `Beacon von ISIS (192.168.x.y)`.
* **Anzeige:** Werte aus dem Sim müssen innerhalb von 2 s erscheinen
  (Keepalive des FDC). Bleibt das Bild schwarz → Panel einmal stromlos machen
  (Backlight-PWM startet erst nach dem Display-Init).
* **Frame-Größe:** Der FDC sendet ≤ 96 B je Werteframe und den `CFG`-Block
  einmalig getrennt. Die Firmware akzeptiert bis 512 B, ist also nicht die
  Grenze.

## Rückweg (falls nötig)

Die Fremd-Firmware liegt im Repo:
`firmware/tools/isis_sim/third_party/CC_ISIS/web-installer/firmware-merged.bin`
→ mit `write_flash 0x0` zurückspielen. Das stellt auch deren Partitionierung
wieder her; das Panel ist also nicht „verheiratet".

## Grenzen dieser Version (bewusst offen, Phase 2)

* **`drawLS()` fehlt** → LS-Skalen und die G/S-/LOC-Diamonds werden nicht
  gezeichnet (Assets liegen bereit).
* Heading, Ground Speed, Track, Heading-Bug, Nav-Source, CDI/GS-Deviation
  kamen im Original aus dem MobiFlight-Core und fehlen hier; die INI-Kanäle
  `dsp9`–`dsp19` existieren aber schon.
* Kein Power-Management (`PowerState`, `drawBattery`) — im Sim-Pit verzichtbar.
* Touch-Menü: nur der Netzwerk-Notfalltrigger, kein volles Menüsystem.


## Wenn nichts bootet: Serial statt Rätselraten

Der Verdacht „Firmware kaputt" ist meist ein **veraltetes Artefakt** oder eine
Boot-Konfiguration. Am Panel hängt ein USB-Serial-Brücke (CH340), also gibt es
eine Konsole — die drei Zeilen ROM-Ausgabe beantworten fast alles:

```
rst:0x7 (TG0WDT_SYS_RST) ...   <- Task-Watchdog: die App haengt (nicht: falscher Flash)
rst:0x1 (POWERON)              <- normaler Kaltstart
mode:DIO / mode:QIO            <- Flash-Modus aus dem Bootloader-Header
```

* **TWDT in Endlosschleife + keine App-Ausgabe** → App haengt früh. Erst prüfen,
  ob das geflashte `.bin` wirklich zum Quellstand passt (neu bauen, md5), dann
  Boot-Trace: `PLATFORMIO_BUILD_FLAGS="-DISIS_BOOT_TRACE=1" pio run -e esp32s3`
  (Marker in `src/main.cpp`, nur mit Trace-Flag aktiv).
* **Konsolen-Mitlesen** mit dem MobiFlight-Python (Embeddable, kein pip):
  `python.exe C:\HermesProbe\serial_watch.py COM4 20` — pulst RTS (Reset) und
  liest 115200 Baud mit. `esptool` liegt daneben als reines Python in
  `C:\HermesProbe\esptool-tool` (siehe `esptool_run.py`), weil die Embeddable-
  Distribution `PYTHONPATH` ignoriert.
* **Gegentest** mit der Fremd-Firmware (`oss_merged.bin` bzw.
  `tools/isis_sim/third_party/CC_ISIS/web-installer/firmware-merged.bin`):
  bootet die, ist die Hardware in Ordnung und der Fehler sitzt im eigenen Build.
* Beide Achsen wurden gemessen: **octal PSRAM (qio_opi) und Flash-Modus
  QIO funktionieren** auf diesem Gerät (Probe mit vier Varianten). Ein
  Boot-Hänger kam **nicht** von der Speicherkonfiguration.
