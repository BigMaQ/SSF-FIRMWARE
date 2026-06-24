# MIP – Main Instrument Panel (Chrono + Brake + Backlight)

## Übersicht

Das **SSF MIP** ist das Main Instrument Panel im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es vereint Chronometer, Bremsdruck-Anzeigen und Backlight-Steuerung auf einem Arduino Mega und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- 2× MAX7219 7-Segment-Displays (14 Digits: UTC 6-stellig, CHR 4-stellig, ET 4-stellig)
- 57 LEDs (16 BRK + 24 Backlight + 16 GPIO + 1 GS_FO)
- 44 Taster (16 Schieberegister + 28 direkte Pins)
- 6 analoge Eingänge (A0–A5)
- Interne UTC-Uhr (unabhängig vom Simulator)
- Chrono (Stoppuhr) mit Start/Stop/Reset
- Umfangreiches DIAG-Menü mit Segment-Tests und LED-Walk
- ACK-Echo-Modus für Befehlsbestätigung
- Analog-Reporting zuschaltbar (standardmäßig AUS)
- EEPROM-Konfiguration (A/C-Registrierung, PCB-Version, Seriennummer)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → MIP)

#### LED-Befehle

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED1** | `LED1:11111111;` | Direkte GPIO-Bank 1 (8 Bits, Pins: 5,7,12,16,17,20,21,24). |
| **LED2** | `LED2:00110011;` | Direkte GPIO-Bank 2 (8 Bits, Pins: 25,26,27,30,32,44,46,60). |
| **LED8** | `LED8:00000001;` | Direkte GPIO-Bank 3 (Bit 0 = GS_FO auf A7). |
| **LED3** | `LED3:11111111;` | BRK-Treiber High-Byte (74HC595). |
| **LED4** | `LED4:00001111;` | BRK-Treiber Low-Byte. |
| **LED5** | `LED5:11111111;` | Backlight-Kette Byte 2 (MSB). |
| **LED6** | `LED6:00110011;` | Backlight-Kette Byte 1. |
| **LED7** | `LED7:01010101;` | Backlight-Kette Byte 0 (LSB). |

#### Display-Befehle

| Befehl | Format | Beschreibung |
|---|---|---|
| **CHR** | `CHR:12.34;` | Chrono-Display (MM.SS). |
| **UTC** | `UTC:14.30.00;` | UTC-Zeit (HH.MM.SS). |
| **ET** | `ET:05.30;` | Elapsed Time (MM.SS). |
| **DSP1** | `DSP1:1234;` | RMP-kompatibel → CHR. |
| **DSP2** | `DSP2:143000;` | RMP-kompatibel → UTC. |
| **DSP3** | `DSP3:0530;` | RMP-kompatibel → ET. |
| **DSP:CHR:** | `DSP:CHR:12.34;` | Gezielte Display-Aktualisierung. |
| **DSP:UTC:** | `DSP:UTC:14.30.00;` | Gezielte Display-Aktualisierung. |
| **DSP:ET:** | `DSP:ET:05.30;` | Gezielte Display-Aktualisierung. |

#### Helligkeit

| Befehl | Format | Beschreibung |
|---|---|---|
| **BL** | `BL:200;` | PWM Backlight-Helligkeit (0–255). |
| **AN** | `AN:150;` | PWM Annunciator-Helligkeit (0–255). |
| **DISP_BL** | `DISP_BL:10;` | MAX7219 Intensität (0–15). |

#### Uhr & Chrono

| Befehl | Format | Beschreibung |
|---|---|---|
| **CLOCK** | `CLOCK:ON;` / `CLOCK:OFF;` | UTC-Uhr ein-/ausschalten. |
| **CLOCK_SET** | `CLOCK_SET:14.30.00;` | Uhrzeit setzen (HH.MM.SS). |
| **CHR_START** | `CHR_START;` | Chrono starten. |
| **CHR_STOP** | `CHR_STOP;` | Chrono stoppen. |
| **CHR_RESET** | `CHR_RESET;` | Chrono zurücksetzen. |

#### Analog & Setup

| Befehl | Format | Beschreibung |
|---|---|---|
| **ANALOG_EN** | `ANALOG_EN:1;` / `ANALOG_EN:0;` | Analog-Reporting ein/aus. |
| **ANALOG_DB** | `ANALOG_DB:8;` | Analog-Totzone (1–100). |
| **SETUP** | `SETUP;` / `SETUP:BRIGHT:AN=,BL=,DISP=;` | Setup-Menü / Helligkeit konfigurieren. |

#### DIAG & Debug

| Befehl | Format | Beschreibung |
|---|---|---|
| **DIAG** | `DIAG;` | DIAG-Report senden. |
| **DIAG:RUN** | `DIAG:RUN;` | DIAG-Sequenz starten (~23 s). |
| **DIAG:STOP** | `DIAG:STOP;` | DIAG abbrechen. |
| **DEBUG_ACK** | `DEBUG_ACK:1;` / `DEBUG_ACK:0;` | ACK-Echo ein/aus. |
| **DEBUG_SR** | `DEBUG_SR;` | Raw Schieberegister-Debug. |
| **DEBUG_PIN** | `DEBUG_PIN:13;` | Pin-Status debuggen. |
| **DEBUG_MAX** | `DEBUG_MAX;` | Alle Segmente AN (Test). |

#### Allgemein

| Befehl | Format | Beschreibung |
|---|---|---|
| **REQ** | `REQ;` | Status sofort anfordern. |
| **VER** | `VER;` | IDENT-Antwort anfordern. |
| **RESET** | `RESET;` | Software-Reset via Watchdog. |

### Serial Output (MIP → Sim)

Bei Änderungen oder auf `REQ;`:

```
IN1:01000000;IN2:00000001;IN3:00101101;IN4:00010010;IN5:00000000;IN6:0000;
```

Mit Analog-Reporting (`ANALOG_EN:1`):
```
...;A0:0512;A1:0768;A2:1023;A3:0200;A4:0800;A5:0500;
```

| Feld | Beschreibung |
|---|---|
| **IN1–IN2:** | Schieberegister-Eingänge (2× 8 Bits). |
| **IN3–IN6:** | Direkte GPIO-Eingänge in Bytes gepackt (active-low → invertiert). |
| **A0–A5:** | Analogwerte (nur wenn ANALOG_EN:1). |

Beim Start:

```
IDENT:MIP, v1.2 MAQ, SN:MIP-XXXXXXXX;STATE:RUNNING;ANALOG_DB:8;
```

---

## DIAG (Hardware-Test)

### Zugang

- **Serial:** `DIAG:RUN;` senden

### Ablauf (~23 Sekunden)

1. **Number Walk:** Ziffern 0–9 auf allen Displays
2. **Segment Test:** Alle 7 Segmente + DP einzeln
3. **Segment Sweep:** Wandernde Segmente
4. **LED Walk:** Alle 57 LEDs nacheinander, mit LED-Nummer und Hex-Wert auf Display

Abbruch mit `DIAG:STOP;`.

---

## Hardware

- **Controller:** Arduino Mega (ATmega2560)
- **Displays:** 2× MAX7219 in Serie (Device 0: UTC 6-stellig, Device 1: CHR 4-stellig + ET 4-stellig)
  - DIN=A8(62), CLK=A9(63), CS=A10(64)
- **BRK LED-Treiber:** 2× 74HC595 (Latch=2, Clock=3, Data=4) — 16 LEDs
- **Backlight LED-Treiber:** 3× 74HC595 (Latch=51, Clock=52, Data=53) — 24 LEDs
- **Direkte GPIO-LEDs:** Bank 1 (8 Pins), Bank 2 (8 Pins), Bank 3 (A7, Bit 0 = GS_FO)
- **Eingänge (Schieberegister):** 2× HC165 (Data=A13, Clock=A12, Latch=A11)
- **Eingänge (direkt):** 28 Pins
- **PWM:** Annunciator (Pin 11), Backlight (Pin 10), Display-Dim (Pin 9)
- **Analog:** A0–A5
- **Panel-ID:** `MIP, v1.2 MAQ` | Firmware: `1.2`

---

## EEPROM-Layout (v3)

Standard v3-Layout ab EEPROM_BASE_ADDR=0.

---

## Settings (Serial)

| Befehl | Wirkung |
|---|---|
| `SET ENA:0815;` | Einstellungsmodus aktivieren |
| `SET FW:1.5;` | PCB-Version setzen |
| `SET ACID:D-AIDA;` | A/C-Registrierung setzen |
| `SET SN:XXXXXXXX;` | Seriennummer setzen (8-stellig Hex) |
| `SET WRITE;` / `SET WRI:YES;` | Konfiguration speichern + Neustart |
| `SET EXIT;` | Einstellungsmodus deaktivieren |


---

## English Version

# MIP – Main Instrument Panel (Chrono + Brake + Backlight)

## Overview

The **SSF MIP** is the Main Instrument Panel in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It combines chronometer, brake pressure indicators, and backlight control on an Arduino Mega, communicating bidirectionally with the simulator via USB-Serial.

**Features:**
- 2× MAX7219 7-segment displays (14 digits: UTC 6-digit, CHR 4-digit, ET 4-digit)
- 57 LEDs (16 BRK + 24 Backlight + 16 GPIO + 1 GS_FO)
- 44 buttons (16 shift register + 28 direct pins)
- 6 analog inputs (A0–A5)
- Internal UTC clock (independent of simulator)
- Chrono (stopwatch) with Start/Stop/Reset
- Extensive DIAG menu with segment tests and LED walk
- ACK echo mode for command confirmation
- Analog reporting switchable (default OFF)
- EEPROM configuration (A/C registration, PCB version, serial number)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`).

### Serial Input (Sim → MIP)

#### LED Commands

| Command | Format | Description |
|---|---|---|
| **LED1** | `LED1:11111111;` | Direct GPIO bank 1 (8 bits). |
| **LED2** | `LED2:00110011;` | Direct GPIO bank 2 (8 bits). |
| **LED8** | `LED8:00000001;` | Direct GPIO bank 3 (bit 0 = GS_FO). |
| **LED3** | `LED3:11111111;` | BRK driver high byte (74HC595). |
| **LED4** | `LED4:00001111;` | BRK driver low byte. |
| **LED5** | `LED5:11111111;` | Backlight chain byte 2 (MSB). |
| **LED6** | `LED6:00110011;` | Backlight chain byte 1. |
| **LED7** | `LED7:01010101;` | Backlight chain byte 0 (LSB). |

#### Display Commands

| Command | Format | Description |
|---|---|---|
| **CHR** | `CHR:12.34;` | Chrono display (MM.SS). |
| **UTC** | `UTC:14.30.00;` | UTC time (HH.MM.SS). |
| **ET** | `ET:05.30;` | Elapsed time (MM.SS). |
| **DSP1** | `DSP1:1234;` | RMP-compatible → CHR. |
| **DSP2** | `DSP2:143000;` | RMP-compatible → UTC. |
| **DSP3** | `DSP3:0530;` | RMP-compatible → ET. |

#### Brightness

| Command | Format | Description |
|---|---|---|
| **BL** | `BL:200;` | PWM backlight (0–255). |
| **AN** | `AN:150;` | PWM annunciator (0–255). |
| **DISP_BL** | `DISP_BL:10;` | MAX7219 intensity (0–15). |

#### Clock & Chrono

| Command | Format | Description |
|---|---|---|
| **CLOCK** | `CLOCK:ON;` / `CLOCK:OFF;` | Enable/disable UTC clock. |
| **CLOCK_SET** | `CLOCK_SET:14.30.00;` | Set clock time. |
| **CHR_START** | `CHR_START;` | Start chrono. |
| **CHR_STOP** | `CHR_STOP;` | Stop chrono. |
| **CHR_RESET** | `CHR_RESET;` | Reset chrono. |

#### Analog & Setup

| Command | Format | Description |
|---|---|---|
| **ANALOG_EN** | `ANALOG_EN:1;` | Enable/disable analog reporting. |
| **ANALOG_DB** | `ANALOG_DB:8;` | Analog deadband (1–100). |
| **SETUP** | `SETUP:BRIGHT:AN=,BL=,DISP=;` | Brightness configuration. |

#### DIAG & Debug

| Command | Format | Description |
|---|---|---|
| **DIAG:RUN** | `DIAG:RUN;` | Start full DIAG sequence (~23 s). |
| **DIAG:STOP** | `DIAG:STOP;` | Abort DIAG. |
| **DIAG** | `DIAG;` | Send DIAG report. |
| **DEBUG_ACK** | `DEBUG_ACK:1;` | Enable/disable ACK echo. |
| **DEBUG_MAX** | `DEBUG_MAX;` | All segments ON test. |

#### General

| Command | Format | Description |
|---|---|---|
| **REQ** | `REQ;` | Request immediate status. |
| **VER** | `VER;` | Request IDENT. |
| **RESET** | `RESET;` | Software reset. |

### Serial Output (MIP → Sim)

On change or `REQ;`:

```
IN1:01000000;IN2:00000001;IN3:00101101;IN4:00010010;IN5:00000000;IN6:0000;
```

With analog (`ANALOG_EN:1`): `...;A0:0512;A1:0768;...A5:0500;`

| Field | Description |
|---|---|
| **IN1–IN2:** | Shift register inputs (2× 8 bits). |
| **IN3–IN6:** | Direct GPIO inputs packed into bytes. |
| **A0–A5:** | Analog values (only if ANALOG_EN:1). |

On startup: `IDENT:MIP, v1.2 MAQ, SN:MIP-XXXXXXXX;STATE:RUNNING;ANALOG_DB:8;`

---

## DIAG (Hardware Test)

`DIAG:RUN;` starts a ~23 second sequence:
1. Number walk on all displays
2. Segment test (all 7 segments + DP individually)
3. Segment sweep
4. LED walk across all 57 LEDs with LED number on display

Stop with `DIAG:STOP;`.

---

## Hardware

- **Controller:** Arduino Mega (ATmega2560)
- **Displays:** 2× MAX7219 in series (14 digits total)
- **BRK LEDs:** 2× 74HC595 (16 LEDs)
- **Backlight LEDs:** 3× 74HC595 (24 LEDs)
- **Direct GPIO LEDs:** 17 LEDs across 3 banks
- **Inputs:** 2× HC165 + 28 direct pins = 44 buttons
- **Analog:** A0–A5
- **Panel ID:** `MIP, v1.2 MAQ` | Firmware: `1.2`

---

## Settings (Serial)

| Command | Effect |
|---|---|
| `SET ENA:0815;` | Activate settings mode |
| `SET FW:1.5;` | Set PCB version |
| `SET ACID:D-AIDA;` | Set A/C registration |
| `SET SN:XXXXXXXX;` | Set serial number |
| `SET WRI:YES;` | Save + restart |
| `SET EXIT;` | Deactivate settings mode |


---

## ฉบับภาษาไทย

# MIP – แผงเครื่องมือหลัก (Chrono + Brake + Backlight)

## ภาพรวม

**SSF MIP** เป็นแผงเครื่องมือหลักในดีไซน์ Airbus A320 สำหรับ MSFS รวมนาฬิกาจับเวลา ตัวแสดงแรงดันเบรก และการควบคุมแสงพื้นหลังบน Arduino Mega

**คุณสมบัติ:**
- จอ MAX7219 2 ชุด (14 หลัก: UTC 6 หลัก, CHR 4 หลัก, ET 4 หลัก)
- LED 57 ดวง
- ปุ่ม 44 ปุ่ม
- อินพุตอนาล็อก 6 ช่อง
- นาฬิกา UTC ภายใน
- นาฬิกาจับเวลา (Chrono)
- เมนู DIAG ที่ครอบคลุม

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1

### อินพุต Serial (คำสั่งหลัก)

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **LED1–LED8** | `LED1:11111111;` | LED 57 ดวง (8 ไบต์) |
| **CHR / UTC / ET** | `CHR:12.34;` | จอ Chrono / UTC / ET |
| **BL / AN** | `BL:200;` | ความสว่าง (0–255) |
| **CLOCK** | `CLOCK:ON;` | เปิด/ปิดนาฬิกา UTC |
| **CHR_START** | `CHR_START;` | เริ่มจับเวลา |
| **DIAG:RUN** | `DIAG:RUN;` | เริ่มลำดับ DIAG (~23 วินาที) |

### เอาต์พุต Serial

```
IN1:01000000;IN2:00000001;IN3:...;IN6:0000;[A0:0512;...]
```

ข้อมูลระบุ: `IDENT:MIP, v1.2 MAQ, SN:MIP-XXXXXXXX;STATE:RUNNING;ANALOG_DB:8;`

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Mega (ATmega2560)
- **จอ:** MAX7219 2 ชุด (14 หลัก)
- **LED:** 57 ดวง (BRK 16 + Backlight 24 + GPIO 17)
- **ปุ่ม:** 44 ปุ่ม
- **รหัสแผง:** `MIP, v1.2 MAQ` | เฟิร์มแวร์: `1.2`
