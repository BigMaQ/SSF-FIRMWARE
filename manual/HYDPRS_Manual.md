# HYDPRS – Hydraulic Pressure / Brake Indicator Module (Airbus A320 Style)

## Übersicht

Das **SSF HYDPRS** (Hydraulic Pressure / Brake Indicator) ist ein Servo-basiertes Anzeigemodul im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es steuert drei Servo-Motoren für ACCU-HYD-Druck und Bremstemperatur-Anzeigen und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- 3× Servo-Motoren (ACCU HYD PRS, BRK L, BRK R)
- 4 LEDs (unteres Nibble eines 8-Bit-Treibers)
- Race-Safe-Architektur (interne vs. Benutzer-Änderungen getrennt)
- Sanfte Servo-Bewegung (konfigurierbare Schrittgröße und Verzögerung)
- Safe-Fade (Servos werden bei PWM-Übergängen abgekoppelt)
- Automatischer Helligkeitspuls beim Einschalten von LEDs
- Startup-Routine mit Servo-Übung und LED-Fade
- EEPROM-Konfiguration (Seriennummer)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → HYDPRS)

| Befehl | Format | Beschreibung |
|---|---|---|
| **ACCU** | `ACCU 2;` | ACCU HYD Servo-Stufe (0–4, je 30°). |
| **BRKL** | `BRKL 1;` | Brake Left Servo-Stufe (0–3, je 30°). |
| **BRKR** | `BRKR 3;` | Brake Right Servo-Stufe (0–3, je 30°). |
| **S1** | `S1 90;` | ACCU Servo Rohwinkel (0–180°). |
| **S2** | `S2 45;` | BRK L Servo Rohwinkel (0–180°). |
| **S3** | `S3 135;` | BRK R Servo Rohwinkel (0–180°). |
| **LED** | `LED 15;` | LED-Maske (0–255, untere 4 Bits verwendet). Dezimal oder Hex. |
| **LEDCH** | `LEDCH 2 1;` | Einzelnen LED-Kanal setzen (Kanal 0–7, Wert 0/1). |
| **BR** | `BR 200;` | Helligkeit setzen (0–255, mit Fade-Übergang). |
| **VER** | `VER;` | Identifikation anfordern. |
| **DIAG** | `DIAG;` | Servo + LED Diagnose. |
| **TEST** | `TEST;` | Servo-Sweep + LED-Chase Test. |
| **OFF** | `OFF;` | Alle LEDs aus. |
| **RESET** | `RESET;` | Software-Reset via Watchdog. |

### Serial Output (HYDPRS → Sim)

- Keine periodische Status-Ausgabe
- Nur Befehlsantworten und Diagnose-Ausgaben
- Race-Safe: Interne Änderungen werden getrennt protokolliert

Beim Start oder auf `VER;`:

```
IDENT: MIP HYD-PRS, v1.1 MAQ, SN:HYDPRS-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

---

## DIAG / TEST

- **DIAG:** Fährt alle 3 Servos auf Maximum und zurück, mit Helligkeits-Fade-Test
- **TEST:** Servo-Sweep 0–120° + LED-Chase (0–7) + Alle-AN-Blinken
- **Startup:** Non-blocking Servo-Übung + LED-Fade (Race-Safe)

Race-Protection: Wenn der Host während DIAG/Startup LEDs/Helligkeit ändert, werden diese Änderungen bewahrt (nicht überschrieben).

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **Servos:** ACCU HYD PRS (D6), BRK L (D3), BRK R (D5)
- **LED-Treiber:** 8-Bit Senke (Latch=14, Clock=15, Data=16), nur Bits 0–3 verwendet
- **PWM:** Helligkeit Pin 9
- **Servo-Offsets:** `servoOffsetAccuHyd`, `servoOffsetBrkL`, `servoOffsetBrkR` (derzeit 0)
- **Panel-ID:** `MIP HYD-PRS, v1.1 MAQ` | Firmware: `1.1`

---

## LED-Mapping

| Kanal | Bit | Beschreibung |
|---|---|---|
| 0 | Bit 0 | LED 0 |
| 1 | Bit 1 | LED 1 |
| 2 | Bit 2 | LED 2 |
| 3 | Bit 3 | LED 3 |
| 4–7 | Bits 4–7 | (ungenutzt) |

---

## Settings (Serial)

| Befehl | Wirkung |
|---|---|
| `SET ENA:0815;` | Einstellungsmodus aktivieren |
| `SET SN:XXXXXXXX;` | Seriennummer setzen (8-stellig Hex) |
| `SET EXIT;` | Einstellungsmodus deaktivieren |
| `SET WRI:YES;` / `SET WRITE;` | Konfiguration speichern + Neustart |


---

## English Version

# HYDPRS – Hydraulic Pressure / Brake Indicator Module (Airbus A320 Style)

## Overview

The **SSF HYDPRS** (Hydraulic Pressure / Brake Indicator) is a servo-based indicator module in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It controls three servo motors for ACCU HYD pressure and brake temperature indicators, communicating bidirectionally with the simulator via USB-Serial.

**Features:**
- 3× servo motors (ACCU HYD PRS, BRK L, BRK R)
- 4 LEDs (lower nibble of 8-bit driver)
- Race-safe architecture (separate internal vs. user change tracking)
- Smooth servo movement (configurable step size and delay)
- Safe fade (servos detached during PWM transitions)
- Automatic brightness pulse when enabling LEDs
- Startup routine with servo exercise and LED fade
- EEPROM configuration (serial number)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`).

### Serial Input (Sim → HYDPRS)

| Command | Format | Description |
|---|---|---|
| **ACCU** | `ACCU 2;` | ACCU HYD servo stage (0–4, 30° each). |
| **BRKL** | `BRKL 1;` | Brake Left servo stage (0–3, 30° each). |
| **BRKR** | `BRKR 3;` | Brake Right servo stage (0–3, 30° each). |
| **S1** | `S1 90;` | ACCU servo raw angle (0–180°). |
| **S2** | `S2 45;` | BRK L servo raw angle (0–180°). |
| **S3** | `S3 135;` | BRK R servo raw angle (0–180°). |
| **LED** | `LED 15;` | LED mask (0–255, lower 4 bits used). Decimal or hex. |
| **LEDCH** | `LEDCH 2 1;` | Set individual LED channel (channel 0–7, value 0/1). |
| **BR** | `BR 200;` | Set brightness (0–255, with fade transition). |
| **VER** | `VER;` | Request identification. |
| **DIAG** | `DIAG;` | Servo + LED diagnostic. |
| **TEST** | `TEST;` | Servo sweep + LED chase test. |
| **OFF** | `OFF;` | All LEDs off. |
| **RESET** | `RESET;` | Software reset via watchdog. |

### Serial Output (HYDPRS → Sim)

- No periodic status output
- Command responses and diagnostic output only
- Race-safe: internal changes logged separately

On startup: `IDENT: MIP HYD-PRS, v1.1 MAQ, SN:HYDPRS-XXXXXXXX;STATE:RUNNING;REG:D-A320;`

---

## DIAG / TEST

- **DIAG:** Moves all 3 servos to max and back, with brightness fade test
- **TEST:** Servo sweep 0–120° + LED chase (0–7) + all-ON flash
- **Startup:** Non-blocking servo exercise + LED fade (race-safe)

Race protection: If host changes LEDs/brightness during DIAG/startup, changes are preserved (not overwritten).

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **Servos:** ACCU HYD PRS (D6), BRK L (D3), BRK R (D5)
- **LED driver:** 8-bit sink (Latch=14, Clock=15, Data=16), lower nibble only
- **PWM:** Brightness Pin 9
- **Panel ID:** `MIP HYD-PRS, v1.1 MAQ` | Firmware: `1.1`

---

## LED Mapping

| Channel | Bit | Description |
|---|---|---|
| 0 | Bit 0 | LED 0 |
| 1 | Bit 1 | LED 1 |
| 2 | Bit 2 | LED 2 |
| 3 | Bit 3 | LED 3 |
| 4–7 | Bits 4–7 | (unused) |

---

## Settings (Serial)

| Command | Effect |
|---|---|
| `SET ENA:0815;` | Activate settings mode |
| `SET SN:XXXXXXXX;` | Set serial number (8 hex digits) |
| `SET EXIT;` | Deactivate settings mode |
| `SET WRI:YES;` / `SET WRITE;` | Save + restart |


---

## ฉบับภาษาไทย

# HYDPRS – โมดูลแสดงแรงดันไฮดรอลิก / เบรก (สไตล์ Airbus A320)

## ภาพรวม

**SSF HYDPRS** เป็นโมดูลแสดงผลที่ใช้เซอร์โวมอเตอร์ในดีไซน์ Airbus A320 สำหรับ MSFS ควบคุมเซอร์โว 3 ตัวสำหรับ ACCU HYD และตัวแสดงอุณหภูมิเบรก

**คุณสมบัติ:**
- เซอร์โวมอเตอร์ 3 ตัว (ACCU HYD PRS, BRK L, BRK R)
- LED 4 ดวง
- สถาปัตยกรรม Race-Safe
- การเคลื่อนที่เซอร์โวอย่างนุ่มนวล

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1

### อินพุต Serial

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **ACCU** | `ACCU 2;` | ขั้นเซอร์โว ACCU HYD (0–4) |
| **BRKL** | `BRKL 1;` | ขั้นเซอร์โวเบรกซ้าย (0–3) |
| **BRKR** | `BRKR 3;` | ขั้นเซอร์โวเบรกขวา (0–3) |
| **S1–S3** | `S1 90;` | มุมเซอร์โวดิบ (0–180°) |
| **LED** | `LED 15;` | หน้ากาก LED (0–255) |
| **BR** | `BR 200;` | ความสว่าง (0–255) |
| **DIAG** | `DIAG;` | วินิจฉัยเซอร์โว + LED |
| **TEST** | `TEST;` | ทดสอบกวาดเซอร์โว + LED |

ข้อมูลระบุ: `IDENT: MIP HYD-PRS, v1.1 MAQ, SN:HYDPRS-XXXXXXXX;STATE:RUNNING;REG:D-A320;`

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Pro Micro (ATmega32U4)
- **เซอร์โว:** D6 (ACCU), D3 (BRK L), D5 (BRK R)
- **LED:** 4 ดวง (บิต 0–3)
- **รหัสแผง:** `MIP HYD-PRS, v1.1 MAQ` | เฟิร์มแวร์: `1.1`
