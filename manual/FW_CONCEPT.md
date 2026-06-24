# SSF Firmware-Konzept / Firmware Architecture / สถาปัตยกรรมเฟิร์มแวร์

## Überblick / Overview

Das SSF-Firmware-Ökosystem folgt einem einheitlichen Konzept über alle Panels hinweg. Jedes Panel ist ein eigenständiger Arduino-basierter Mikrocontroller, der via USB-Serial (115200 Baud) mit dem Simulator-PC kommuniziert. Die Firmware ist in reinem C/C++ geschrieben — ohne dynamische Speicherallokation und ohne die Arduino-`String`-Klasse — um Speicherfragmentierung auf den ressourcenarmen ATmega-Chips zu vermeiden.

---

## 1. Serielles Protokoll / Serial Protocol

### Grundstruktur

```
KEY:VALUE;KEY2:VALUE2;KEY3:VALUE3;
```

- **Trennzeichen:** Semikolon `;`
- **Key-Value-Trenner:** Doppelpunkt `:`
- **Zeilenende:** CR, LF oder `;` werden gleichermaßen akzeptiert
- **Richtung:** Bidirektional — Host (Sim) ↔ Panel

### Token-Typen

| Typ | Format | Beispiel | Richtung |
|---|---|---|---|
| **Key:Value** | `KEY:WERT;` | `LED1:10101010;` | Host → Panel |
| **Single-Word** | `WORT;` | `VER;` `REQ;` `DIAG;` | Host → Panel |
| **Status-Frame** | `FELD:WERT;FELD:WERT;...` | `IN1:01000000;POT1:0512;` | Panel → Host |
| **IDENT-Frame** | `IDENT:...;STATE:...;` | `IDENT:ECAM, v1.2 MAQ, SN:ECAM-ABCD1234;STATE:RUNNING;HWREV:1;` | Panel → Host |

### Single-Word-Befehle (alle Panels)

| Befehl | Bedeutung |
|---|---|
| `VER;` / `VERSION;` | IDENT-Antwort anfordern |
| `REQ;` | Status sofort senden (force send) |
| `DIAG;` | DIAG-Routine starten |
| `RESET;` | Watchdog-Reset (Software-Neustart) |

---

## 2. Namenskonventionen / Naming Conventions

### Eingaben (Panel → Host)

| Präfix | Bedeutung | Format | Beispiel |
|---|---|---|---|
| **IN1–INn** | Digitale Eingänge (Taster/Schalter) | 8-Bit Binärstring | `IN1:01000001;` |
| **POT1–POTn** | Potentiometer (analog) | 4-stellig, 0–1023 | `POT1:0512;` |
| **MUX0–MUX15** | Analog-Multiplexer-Kanäle | 4-stellig, 0–1023 | `MUX3:1023;` |
| **A0–An** | Direkte Analog-Pins | 4-stellig, 0–1023 | `A7:0512;` |
| **RT1–RTn** | Rotary Encoder (Delta) | Integer (positiv/negativ) | `RT1:1;` `RT2:-3;` |

> **Binärformat:** `0` = nicht gedrückt (LOW), `1` = gedrückt (HIGH). MSB zuerst (Bit 7 … Bit 0).

### Ausgaben (Host → Panel)

| Präfix | Bedeutung | Format | Beispiel |
|---|---|---|---|
| **LED1–LEDn** | LED-Schieberegister / GPIO-Bänke | 8-Bit Binärstring | `LED1:11110000;` |
| **BL** | Backlight-Helligkeit (PWM) | 0–255 dezimal | `BL:200;` |
| **AN** | Annunciator-Helligkeit (PWM) | 0–255 dezimal | `AN:150;` |
| **DISP_BL** | Display-Helligkeit (MAX7219) | 0–15 dezimal | `DISP_BL:10;` |
| **DSP1–DSPn** | Display-Inhalte | Text/String | `DSP1:123.450;` |

### Spezielle Display-Befehle

| Panel | Befehl | Bedeutung |
|---|---|---|
| RMP | `DSP1`, `DSP2` | Linkes/rechtes Frequenzdisplay |
| ATC | `DSP1`, `DSP2`, `DSP3` | Squawk-Code, Ziffernzahl, Standby |
| MIP | `CHR`, `UTC`, `ET` | Chrono, UTC-Zeit, Elapsed Time |
| MIP | `DSP:CHR:`, `DSP:UTC:`, `DSP:ET:` | Gezielte Display-Adressierung |
| RUD | `DSP1` | Rudder-Trim-Wert (Float → `A 0.4`) |

---

## 3. LED-System / LED Architecture

Jedes Panel hat bis zu 57 LEDs (MIP), organisiert in Schichten:

| Schicht | Treiber | Typische Verwendung |
|---|---|---|
| **LED1** | 74HC595 Schieberegister Kette, Byte 1 | Backlight-Taster |
| **LED2** | 74HC595, Byte 2 | Annunciator-LEDs |
| **LED3–LED4** | 74HC595, Byte 3–4 | Zusätzliche LEDs (ECAM, RUD) |
| **LED5–LED8** | 74HC595, Byte 5–8 | MIP-spezifisch (BRK, Backlight) |
| **Direkte GPIO** | `digitalWrite()` | Einzelne Status-LEDs (DOOR_FLT, ATC FAIL, GS_FO) |

**Prinzip:** Ein `LED1:10101010;`-Befehl setzt 8 LEDs auf einmal. Bit 0 = LED 1 (LSB), Bit 7 = LED 8 (MSB). Die Hardware-Verdrahtung bestimmt, welche physische LED welchem Bit entspricht.

---

## 4. Eingabe-System / Input Architecture

| Typ | Hardware | Panels |
|---|---|---|
| **Schieberegister** | 74HC165 (8 Bit pro IC) | ACP, ATC, ECAM, MIP, RUD |
| **Direkte GPIO-Pins** | `digitalRead()` mit `INPUT_PULLUP` | ATC (7 Taster), MIP (28 Taster), DOOR (6 Taster) |
| **Analog-MUX** | 4067 16:1 Multiplexer | ACP (16 Potis) |
| **Direkte Analog-Pins** | `analogRead()` | ECAM (2), DOOR (3), RUD (1), MIP (6) |
| **Hall-Sensor** | Analog-Pin A7 | TILLER |
| **Rotary Encoder** | Polling via digitalRead | RMP (2), ATC (DIAG-Navigation) |

**Entprellung:** Alle digitalen Eingänge werden per Software entprellt (typisch 12 ms `DEBOUNCE_MS`). Änderungen werden nur gesendet, wenn sich der Zustand nach der Entprellzeit stabilisiert hat.

**Glättung (Analog/Potis):** Gleitender Mittelwert über konfigurierbare Sample-Anzahl (`SMO_SAM`, default 4). Änderungen unterhalb der Totzone (`POT_DB` / `SMO_THR`) werden unterdrückt.

---

## 5. EEPROM-Layout v3 (alle Panels)

Jedes Panel speichert seine Konfiguration im internen EEPROM:

| Adresse | Größe | Feld | Beschreibung |
|---|---|---|---|
| 0 | 1 | HW-Revision | Nur ACP/ECAM (separat vom Config-Block) |
| 10–11 | 2 | Magic Number | `0xA55A` (Little-Endian) |
| 12 | 1 | Format-Version | `3` |
| 13–20 | 8 | A/C-Registrierung | z. B. `D-A320`, Leerzeichen-gepadded |
| 21–28 | 8 | PCB-Version | z. B. `PCB 1.0` |
| 29–37 | 9 | Seriennummer | 8 Hex-Zeichen + Null-Terminator |
| 38 | 1 | Checksumme | Summe über Bytes 12–37, Low-Byte |

**Seriennummern-Format:** `PANEL_PREFIX-XXXXXXXX` (z. B. `ECAM-4F9A12C3`, `RMP-A1B2C3D4`)

Wird beim ersten Boot automatisch generiert, wenn keine gültige Konfiguration gefunden wird.

---

## 6. SET-Befehl-System / Settings

Geschützte Einstellungen über den Serial-Port (PIN-geschützt):

```
SET ENA:0815;          ← Einstellungsmodus aktivieren
SET FW:1.5;            ← PCB-Version setzen
SET ACID:D-A320;       ← A/C-Registrierung setzen
SET SN:4F9A12C3;       ← Seriennummer setzen (8-stellig Hex)
SET WRI:YES;           ← In EEPROM schreiben + Neustart
SET WRITE;             ← Alternative: Schreiben wenn freigeschaltet
SET EXIT;              ← Einstellungsmodus verlassen
```

**Workflow:**
1. `SET ENA:0815;` → Panel antwortet `SET ENA> ;`
2. Einstellungen vornehmen (`SET FW:`, `SET ACID:`, `SET SN:`)
3. `SET WRI:YES;` → Speichert alles ins EEPROM und löst Watchdog-Reset aus

---

## 7. IDENT-System / Panel-Erkennung

Jedes Panel sendet beim Boot und auf `VER;`/`IDENT;`:

```
IDENT:PANEL, vX.Y MAQ[, SN:PREFIX-XXXXXXXX];STATE:RUNNING;[HWREV:n;][REG:D-A320;]
```

| Feld | Beispiel | Bedeutung |
|---|---|---|
| `PANEL` | `ECAM`, `RMP`, `ACP 1 CPT` | Panel-Typ |
| `vX.Y MAQ` | `v1.2 MAQ` | Firmware-Version + Herstellerkürzel |
| `SN:PREFIX-XXXXXXXX` | `SN:ECOM-4F9A12C3` | Eindeutige Seriennummer (optional) |
| `STATE:RUNNING` | `RUNNING` | Panel-Status |
| `HWREV:n` | `HWREV:1` | Hardware-Revision (ACP, ECAM) |
| `REG:D-A320` | `D-A320` | A/C-Registrierung |

Der Host (FDC/FCC) nutzt die IDENT-Antwort zur automatischen Panel-Erkennung und COM-Port-Zuordnung.

---

## 8. DIAG-System / Hardware-Test

Jedes Panel hat eine integrierte Diagnose-Funktion:

| Panel | Zugang | Umfang |
|---|---|---|
| **RMP** | `DIAG;` oder Tastenkombi | Menügeführt: LED-Test, Display-Test, Taster-Test, Info |
| **ACP** | Tastenkombi | 15 s LED-Dauertest, automatischer Reset |
| **ATC** | `DIAG;` (30 s nach Boot) | Menügeführt mit Taster-Navigation |
| **ECAM** | `DIAG;` oder Tastenkombi | LED-Walk (32 LEDs) + Muster-Test, 15 s Auto-Reset |
| **MIP** | `DIAG:RUN;` | ~23 s: Number-Walk, Segment-Test, LED-Walk (57 LEDs) |
| **RUD** | `DIAG;` | ~15 s: LED-Walk, Display-Walk, Segment-Test |
| **TILLER** | `DIAG;` | Zyklische Helligkeitsänderung |
| **HYDPRS** | `DIAG;` `TEST;` | Servo-Sweep + LED-Chase |
| **DOOR** | Boot-Sequenz | Kein dedizierter DIAG-Befehl |

**DIAG-Combo-Prinzip:** Panels mit Taster-Matrix (ACP, ECAM) erkennen eine spezifische Tastenkombination als DIAG-Trigger. Nach Ablauf eines Timeouts (15 s) wird der DIAG-Modus automatisch beendet, damit keine LEDs dauerhaft blockiert bleiben.

---

## 9. Boot-Sequenz / Startup

### Non-Blocking Boot (ECAM, DOOR)

```cpp
void loop() {
    if (!bootComplete) {
        if (now - bootPhaseStart >= BOOT_PHASE_MS) {
            switch (bootPhase) {
                case 0: /* Muster 1 */ break;
                case 1: /* Muster 2 */ break;
                // ...
                case N: bootComplete = true; break;
            }
            bootPhase++;
        }
        processSerialInput();  // Serial NICHT blockieren
        return;
    }
    // Normaler Betrieb
}
```

**Vorteil:** Serial-Kommunikation bleibt während der Boot-Animation empfänglich — das Panel ist sofort für IDENT-Scans bereit.

### Blocking Boot (RMP, ACP, ATC)

Kurze `delay()`-basierte LED-Sequenz in `setup()`, dann IDENT. Einfacher, aber blockiert Serial für ~600 ms.

---

## 10. Spezielle Panel-Features

| Panel | Besonderheit |
|---|---|
| **RMP** | DIAG-Menü mit Rotary-Navigation, CFG-Kommando, DSP-Formatierung (Kurs/Frequenz) |
| **ACP** | HW-Revision 1/2 (16/24 LEDs), 4067 MUX für 16 Potis, HWREVSET-Befehl |
| **ATC** | XPDR Multi-Source Display-Routing, Scroll-Funktion für Text > 4 Zeichen, Offline-"noFS"-Meldung |
| **ECAM** | Optimierte String-Verarbeitung (char-Buffer statt Arduino String), 4 LED/Input-Bytes für max. 32 LEDs/Taster |
| **MIP** | Interne UTC-Uhr, Chrono (Stoppuhr), 57 LEDs, ACK-Echo-Modus, Analog-Reporting zuschaltbar |
| **RUD** | Trim-Wert-Formatierung (Float → `A 0.4`/`L12.5`), Boot-Sequenz mit Versionsinfo auf Display |
| **TILLER** | Hall-Sensor-Glättung, Event-gesteuerte Übertragung (nur bei Änderung), Invertierte Helligkeit |
| **HYDPRS** | 3 Servo-Motoren, Race-Safe-Architektur (interne vs. Benutzer-Änderungen), Sanfte Servo-Übergänge |
| **DOOR** | Non-blocking Boot, 6 direkte Taster + 3 Potis, 2 direkte Status-LEDs |

---

## 11. Controller-Übersicht / Controller Matrix

| Controller | Panels | Flash-Methode | Bootloader |
|---|---|---|---|
| **ATmega32U4** (Pro Micro) | RMP, ACP, ATC, ECAM, RUD, HYDPRS, DOOR | avr109, 57600 Baud | 8 s, Double-Reset |
| **ATmega2560** (Mega) | MIP | wiring, 115200 Baud | Standard |
| **ATmega328P** (Nano) | TILLER | arduino, 115200 Baud | Standard |

---

## 12. Repository-Struktur

```
SSF-FIRMWARE/
├── source/           ← Alle .INO-Quelldateien
│   ├── RMP.ino
│   ├── ACP.ino
│   ├── ATC.ino
│   ├── ECAM/ECAM.ino
│   ├── MIP.ino
│   ├── RUD.ino
│   ├── Tiller_CPT.ino
│   ├── accu_press.ino   (HYDPRS)
│   └── CPT_DOOR.ino
├── _build/           ← Kompilierte .hex-Dateien (pro Panel)
│   ├── RMP/RMP.ino.hex
│   ├── ACP/ACP.ino.hex
│   └── ...
├── manual/           ← Panel-Manuals + Flash-Anleitung (DE/EN/TH)
└── tools/            ← Build-Skripte (fwbuild.ps1)
```

---

## English Version

# SSF Firmware Architecture – Concept Reference

## Overview

The SSF firmware ecosystem follows a unified concept across all panels. Each panel is a standalone Arduino-based microcontroller communicating with the simulator PC via USB-Serial (115200 baud). The firmware is written in pure C/C++ — without dynamic memory allocation and without the Arduino `String` class — to avoid memory fragmentation on resource-constrained ATmega chips.

---

## 1. Serial Protocol

### Basic Structure

```
KEY:VALUE;KEY2:VALUE2;KEY3:VALUE3;
```

- **Delimiter:** Semicolon `;`
- **Key-Value separator:** Colon `:`
- **Line ending:** CR, LF or `;` are all accepted
- **Direction:** Bidirectional — Host (Sim) ↔ Panel

### Token Types

| Type | Format | Example | Direction |
|---|---|---|---|
| **Key:Value** | `KEY:VALUE;` | `LED1:10101010;` | Host → Panel |
| **Single-Word** | `WORD;` | `VER;` `REQ;` `DIAG;` | Host → Panel |
| **Status Frame** | `FIELD:VALUE;FIELD:VALUE;...` | `IN1:01000000;POT1:0512;` | Panel → Host |
| **IDENT Frame** | `IDENT:...;STATE:...;` | `IDENT:ECAM, v1.2 MAQ, SN:ECAM-ABCD1234;STATE:RUNNING;HWREV:1;` | Panel → Host |

---

## 2. Naming Conventions

### Inputs (Panel → Host)

| Prefix | Meaning | Format | Example |
|---|---|---|---|
| **IN1–INn** | Digital inputs (buttons/switches) | 8-bit binary string | `IN1:01000001;` |
| **POT1–POTn** | Potentiometers (analog) | 4-digit, 0–1023 | `POT1:0512;` |
| **MUX0–MUX15** | Analog multiplexer channels | 4-digit, 0–1023 | `MUX3:1023;` |
| **A0–An** | Direct analog pins | 4-digit, 0–1023 | `A7:0512;` |
| **RT1–RTn** | Rotary encoders (delta) | Integer (positive/negative) | `RT1:1;` `RT2:-3;` |

> **Binary format:** `0` = not pressed (LOW), `1` = pressed (HIGH). MSB first (bit 7 … bit 0).

### Outputs (Host → Panel)

| Prefix | Meaning | Format | Example |
|---|---|---|---|
| **LED1–LEDn** | LED shift registers / GPIO banks | 8-bit binary string | `LED1:11110000;` |
| **BL** | Backlight brightness (PWM) | 0–255 decimal | `BL:200;` |
| **AN** | Annunciator brightness (PWM) | 0–255 decimal | `AN:150;` |
| **DISP_BL** | Display brightness (MAX7219) | 0–15 decimal | `DISP_BL:10;` |
| **DSP1–DSPn** | Display contents | Text/String | `DSP1:123.450;` |

---

## 3. LED Architecture

Each panel has up to 57 LEDs (MIP), organized in layers:

| Layer | Driver | Typical Use |
|---|---|---|
| **LED1** | 74HC595 shift register chain, byte 1 | Backlight buttons |
| **LED2** | 74HC595, byte 2 | Annunciator LEDs |
| **LED3–LED4** | 74HC595, bytes 3–4 | Additional LEDs (ECAM, RUD) |
| **LED5–LED8** | 74HC595, bytes 5–8 | MIP-specific (BRK, Backlight) |
| **Direct GPIO** | `digitalWrite()` | Individual status LEDs |

**Principle:** A `LED1:10101010;` command sets 8 LEDs at once. Bit 0 = LED 1 (LSB), Bit 7 = LED 8 (MSB).

---

## 4. Input Architecture

| Type | Hardware | Panels |
|---|---|---|
| **Shift Registers** | 74HC165 (8 bits per IC) | ACP, ATC, ECAM, MIP, RUD |
| **Direct GPIO Pins** | `digitalRead()` with `INPUT_PULLUP` | ATC, MIP, DOOR |
| **Analog MUX** | 4067 16:1 multiplexer | ACP |
| **Direct Analog Pins** | `analogRead()` | ECAM, DOOR, RUD, MIP |
| **Hall Sensor** | Analog pin A7 | TILLER |
| **Rotary Encoders** | Polling via digitalRead | RMP, ATC (DIAG nav) |

**Debouncing:** All digital inputs are software-debounced (typically 12 ms `DEBOUNCE_MS`). Changes are only sent after the state has stabilized.

**Smoothing (Analog/Pots):** Moving average over configurable sample count (`SMO_SAM`, default 4). Changes below the deadband (`POT_DB` / `SMO_THR`) are suppressed.

---

## 5. EEPROM Layout v3 (All Panels)

| Address | Size | Field | Description |
|---|---|---|---|
| 0 | 1 | HW Revision | ACP/ECAM only (separate from config block) |
| 10–11 | 2 | Magic Number | `0xA55A` (Little-Endian) |
| 12 | 1 | Format Version | `3` |
| 13–20 | 8 | A/C Registration | e.g. `D-A320`, space-padded |
| 21–28 | 8 | PCB Version | e.g. `PCB 1.0` |
| 29–37 | 9 | Serial Number | 8 hex chars + null terminator |
| 38 | 1 | Checksum | Sum over bytes 12–37, low byte |

**Serial Number Format:** `PANEL_PREFIX-XXXXXXXX` (e.g. `ECAM-4F9A12C3`)

Auto-generated on first boot if no valid config is found.

---

## 6. SET Command System

Protected settings via serial port (PIN-protected):

```
SET ENA:0815;          ← Activate settings mode
SET FW:1.5;            ← Set PCB version
SET ACID:D-A320;       ← Set A/C registration
SET SN:4F9A12C3;       ← Set serial number (8 hex digits)
SET WRI:YES;           ← Write to EEPROM + restart
SET WRITE;             ← Alternative: write if unlocked
SET EXIT;              ← Exit settings mode
```

---

## 7. IDENT System

Each panel sends on boot and on `VER;`/`IDENT;`:

```
IDENT:PANEL, vX.Y MAQ[, SN:PREFIX-XXXXXXXX];STATE:RUNNING;[HWREV:n;][REG:D-A320;]
```

The host (FDC/FCC) uses the IDENT response for automatic panel detection and COM port assignment.

---

## 8. DIAG System

| Panel | Access | Scope |
|---|---|---|
| **RMP** | `DIAG;` or button combo | Menu-driven: LED test, Display test, Button test, Info |
| **ACP** | Button combo | 15 s continuous LED test, auto-reset |
| **ATC** | `DIAG;` (within 30 s of boot) | Menu-driven with button navigation |
| **ECAM** | `DIAG;` or button combo | LED walk (32 LEDs) + pattern test, 15 s auto-reset |
| **MIP** | `DIAG:RUN;` | ~23 s: Number walk, Segment test, LED walk (57 LEDs) |
| **RUD** | `DIAG;` | ~15 s: LED walk, Display walk, Segment test |
| **TILLER** | `DIAG;` | Cyclic brightness change |
| **HYDPRS** | `DIAG;` `TEST;` | Servo sweep + LED chase |
| **DOOR** | Boot sequence | No dedicated DIAG command |

---

## 9. Boot Sequence

### Non-Blocking Boot (ECAM, DOOR)

State machine in `loop()` — serial stays responsive during boot animation. Panel is ready for IDENT scans immediately.

### Blocking Boot (RMP, ACP, ATC)

Short `delay()`-based LED sequence in `setup()`, then IDENT. Simpler but blocks serial for ~600 ms.

---

## 10. Controller Overview

| Controller | Panels | Flash Method | Bootloader |
|---|---|---|---|
| **ATmega32U4** (Pro Micro) | RMP, ACP, ATC, ECAM, RUD, HYDPRS, DOOR | avr109, 57600 baud | 8 s, double-reset |
| **ATmega2560** (Mega) | MIP | wiring, 115200 baud | Standard |
| **ATmega328P** (Nano) | TILLER | arduino, 115200 baud | Standard |


---

## ฉบับภาษาไทย

# สถาปัตยกรรมเฟิร์มแวร์ SSF – เอกสารอ้างอิงแนวคิด

## ภาพรวม

ระบบนิเวศเฟิร์มแวร์ SSF ใช้แนวคิดเดียวกันในทุกแผง แต่ละแผงเป็นไมโครคอนโทรลเลอร์ Arduino แบบสแตนด์อโลนที่สื่อสารกับพีซีซิมูเลเตอร์ผ่าน USB-Serial (115200 บอด) เฟิร์มแวร์เขียนด้วย C/C++ แท้ — ไม่มีการจัดสรรหน่วยความจำแบบไดนามิกและไม่ใช้คลาส `String` ของ Arduino

---

## 1. โปรโตคอล Serial

### โครงสร้างพื้นฐาน

```
KEY:VALUE;KEY2:VALUE2;KEY3:VALUE3;
```

- **ตัวคั่น:** เซมิโคลอน `;`
- **ตัวคั่น Key-Value:** ทวิภาค `:`
- **ทิศทาง:** สองทิศทาง — โฮสต์ ↔ แผง

---

## 2. ข้อตกลงการตั้งชื่อ

### อินพุต (แผง → โฮสต์)

| คำนำหน้า | ความหมาย | ตัวอย่าง |
|---|---|---|
| **IN1–INn** | อินพุตดิจิทัล (ปุ่ม/สวิตช์) | `IN1:01000001;` |
| **POT1–POTn** | โพเทนชิโอมิเตอร์ | `POT1:0512;` |
| **MUX0–MUX15** | ช่อง Analog MUX | `MUX3:1023;` |
| **A0–An** | ขาอนาล็อกโดยตรง | `A7:0512;` |
| **RT1–RTn** | Rotary Encoder (เดลต้า) | `RT1:1;` |

### เอาต์พุต (โฮสต์ → แผง)

| คำนำหน้า | ความหมาย | ตัวอย่าง |
|---|---|---|
| **LED1–LEDn** | LED ชิฟต์รีจิสเตอร์ | `LED1:11110000;` |
| **BL** | ความสว่างพื้นหลัง (PWM) | `BL:200;` |
| **AN** | ความสว่างตัวประกาศ (PWM) | `AN:150;` |
| **DSP1–DSPn** | เนื้อหาจอแสดงผล | `DSP1:123.450;` |

---

## 3. ระบบ LED

| ชั้น | ไดรเวอร์ | การใช้งาน |
|---|---|---|
| **LED1** | 74HC595 ไบต์ 1 | ปุ่มพื้นหลัง |
| **LED2** | 74HC595 ไบต์ 2 | LED ตัวประกาศ |
| **LED3–LED8** | 74HC595 ไบต์ 3–8 | LED เพิ่มเติม |

---

## 4. ระบบอินพุต

| ประเภท | ฮาร์ดแวร์ | แผง |
|---|---|---|
| **ชิฟต์รีจิสเตอร์** | 74HC165 | ACP, ATC, ECAM, MIP, RUD |
| **ขา GPIO โดยตรง** | `digitalRead()` | ATC, MIP, DOOR |
| **Analog MUX** | 4067 16:1 | ACP |
| **Hall Sensor** | A7 | TILLER |
| **Rotary Encoder** | Polling | RMP, ATC |

---

## 5. ผัง EEPROM v3

| ที่อยู่ | ขนาด | ฟิลด์ |
|---|---|---|
| 10–11 | 2 | Magic Number `0xA55A` |
| 12 | 1 | เวอร์ชันรูปแบบ `3` |
| 13–20 | 8 | ทะเบียน A/C |
| 21–28 | 8 | เวอร์ชัน PCB |
| 29–37 | 9 | หมายเลขซีเรียล |
| 38 | 1 | เช็คซัม |

---

## 6. ระบบคำสั่ง SET

```
SET ENA:0815;    ← เปิดโหมดตั้งค่า
SET FW:1.5;      ← ตั้งค่าเวอร์ชัน PCB
SET ACID:D-A320; ← ตั้งค่าทะเบียน A/C
SET SN:XXXXXXXX; ← ตั้งค่าหมายเลขซีเรียล
SET WRI:YES;     ← บันทึก + เริ่มใหม่
```

---

## 7. ระบบ IDENT

```
IDENT:PANEL, vX.Y MAQ, SN:PREFIX-XXXXXXXX;STATE:RUNNING;HWREV:n;
```

---

## 8. ภาพรวมคอนโทรลเลอร์

| คอนโทรลเลอร์ | แผง | วิธีแฟลช |
|---|---|---|
| **ATmega32U4** (Pro Micro) | RMP, ACP, ATC, ECAM, RUD, HYDPRS, DOOR | avr109, 57600 |
| **ATmega2560** (Mega) | MIP | wiring, 115200 |
| **ATmega328P** (Nano) | TILLER | arduino, 115200 |
