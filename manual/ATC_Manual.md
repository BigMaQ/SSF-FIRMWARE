# ATC – ATC/Transponder Panel (Airbus A320 Style)

## Übersicht

Das **SSF ATC** ist ein originalgetreues ATC/Transponder Panel im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es zeigt Squawk-Codes auf einem 4-stelligen MAX7219-Display an und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- MAX7219 7-Segment-Display (4 Digits) für Squawk-Codes
- 9 hintergrundbeleuchtete Taster (Mode-Tasten + numerische Tasten 0–7)
- 7 direkte Funktionstaster (ON, SYS2, SYS1, IDENT, AUTO, CLR, STBY)
- ATC FAIL LED (direkt angesteuert)
- Integriertes DIAG-Menü (innerhalb 30 s nach Boot)
- Offline-Erkennung mit Display-Meldung "noFS"
- Scroll-Funktion für Texte > 4 Zeichen
- EEPROM-Konfiguration (A/C-Registrierung, PCB-Version, Seriennummer)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → ATC)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED1** | `LED1:11111111;` | Backlight-LEDs (74HC595). 8-Bit Binärstring. |
| **LED2** | `LED2:00000001;` | Bit 0 = ATC FAIL LED. |
| **BL** | `BL:200;` | PWM-Helligkeit (0–255). |
| **DISP_BL** | `DISP_BL:10;` | Helligkeit des MAX7219-Displays (0–15). |
| **DSP1** | `DSP1:1234;` | Squawk-Code (4-stellig, XPDR aktiv). |
| **DSP2** | `DSP2:2;` | Anzahl der eingegebenen Squawk-Ziffern (0–4). |
| **DSP3** | `DSP3:5678;` | Standby/partieller Squawk-Code. |
| **STATE** | `STATE:01;` | `01`/`1` = Host online, `00`/`0` = Host offline → "noFS" auf Display. |
| **REQ** | `REQ;` | Status sofort anfordern. |
| **VER** | `VER;` | IDENT-Antwort anfordern. |
| **DIAG** | `DIAG;` | DIAG-Menü starten (nur innerhalb 30 s nach Boot). |
| **CFG** | `CFG:DEB12;LED1200;DSP1200;REG:D-A320;SN:XXXXXXXX;` | Konfiguration in einem Befehl. |

### Serial Output (ATC → Sim)

Das Panel sendet bei jeder Änderung sowie auf `REQ;`:

```
IN1:01000000;IN2:00000001;IN3:00000011;
```

| Feld | Beschreibung |
|---|---|
| **IN1:** | 8-Bit Binärstatus (HC165 Register 1) — Mode-Tasten: ALL, ABV, BLW, ON, TARA, TA, STBY, THRT. |
| **IN2:** | 8-Bit Binärstatus (HC165 Register 2) — Zifferntasten 0–7. |
| **IN3:** | 7-Bit Binärstatus — ON, SYS2, SYS1, IDENT, AUTO, CLR, STBY. |

Beim Start oder auf `VER;`:

```
IDENT:ATC, v1.2 MAQ, SN:ATC-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

---

## DIAG-Menü (Hardware-Test)

### Zugang

- **Serial:** `DIAG;` senden (innerhalb 30 s nach Boot)
- **Tastenkombi am Panel:** Button-Combo (CLR + numerische Tasten)

### Menüführung

- **CLR** = Enter/Auswahl
- **Taste 2** = Nächster/Up
- **Taste 0** = Zurück/Down
- Auto-Exit nach 15 s Inaktivität

Tests: LED-Test, Taster-Test, Info-Anzeige, Segment-Test mit Animation.

---

## Hardware

- **Controller:** Arduino Mega (ATmega2560)
- **Display:** 1× MAX7219 (4 Digits)
- **LED-Treiber:** 1× 74HC595 Schieberegister (8 Backlight-LEDs)
- **Eingänge:** 2× 74HC165 Schieberegister (16 Taster) + 7 direkte Pins
- **Direkte LEDs:** ATC FAIL (Pin 7, active LOW)
- **PWM:** Helligkeit (Pin 3)
- **Panel-ID:** `ATC, v1.2 MAQ` | Firmware: `1.2`

---

## Taster-Mapping

| Register | Bit | Taste | LED |
|---|---|---|---|
| **IN1** | 0 | ALL | Backlight Bit 0 |
| | 1 | ABV | Backlight Bit 1 |
| | 2 | BLW | Backlight Bit 2 |
| | 3 | ON | Backlight Bit 3 |
| | 4 | TARA | Backlight Bit 4 |
| | 5 | TA | Backlight Bit 5 |
| | 6 | STBY | Backlight Bit 6 |
| | 7 | THRT | Backlight Bit 7 |
| **IN2** | 0–7 | Ziffern 0–7 | — |
| **IN3 (direkt)** | 0 | ON (Pin 8) | — |
| | 1 | SYS2 (Pin 9) | — |
| | 2 | SYS1 (Pin 10) | — |
| | 3 | IDENT (Pin 14) | ATC FAIL LED |
| | 4 | AUTO (Pin 15) | — |
| | 5 | CLR (Pin 16) | — |
| | 6 | STBY (Pin A0) | — |

---

## EEPROM-Layout (v3)

Standard v3-Layout ab EEPROM_BASE_ADDR=0.

---

## Settings (Serial)

| Befehl | Wirkung |
|---|---|
| `SET ENA:0815;` | Einstellungsmodus aktivieren (PIN = 0815) |
| `SET FW:1.5;` | PCB-Version setzen |
| `SET ACID:D-AIDA;` | A/C-Registrierung setzen |
| `SET SN:XXXXXXXX;` | Seriennummer setzen (8-stellig Hex) |
| `SET WRITE;` / `SET WRI:YES;` | Konfiguration speichern + Neustart |
| `SET EXIT;` | Einstellungsmodus deaktivieren |


---

## English Version

# ATC – ATC/Transponder Panel (Airbus A320 Style)

## Overview

The **SSF ATC** is a faithful ATC/Transponder Panel in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It displays squawk codes on a 4-digit MAX7219 display and communicates bidirectionally with the simulator via USB-Serial.

**Features:**
- MAX7219 7-segment display (4 digits) for squawk codes
- 9 backlit buttons (mode keys + numeric keys 0–7)
- 7 direct function buttons (ON, SYS2, SYS1, IDENT, AUTO, CLR, STBY)
- ATC FAIL LED (direct driven)
- Integrated DIAG menu (within 30 s after boot)
- Offline detection with "noFS" display message
- Scroll function for text > 4 characters
- EEPROM configuration (A/C registration, PCB version, serial number)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`), CR/LF accepted as delimiter.

### Serial Input (Sim → ATC)

| Command | Format | Description |
|---|---|---|
| **LED1** | `LED1:11111111;` | Backlight LEDs (74HC595). 8-bit binary string. |
| **LED2** | `LED2:00000001;` | Bit 0 = ATC FAIL LED. |
| **BL** | `BL:200;` | PWM brightness (0–255). |
| **DISP_BL** | `DISP_BL:10;` | MAX7219 display brightness (0–15). |
| **DSP1** | `DSP1:1234;` | Squawk code (4 digits, XPDR active). |
| **DSP2** | `DSP2:2;` | Number of entered squawk digits (0–4). |
| **DSP3** | `DSP3:5678;` | Standby/partial squawk code. |
| **STATE** | `STATE:01;` | `01`/`1` = Host online, `00`/`0` = Host offline → "noFS" on display. |
| **REQ** | `REQ;` | Request immediate status. |
| **VER** | `VER;` | Request IDENT response. |
| **DIAG** | `DIAG;` | Start DIAG menu (only within 30 s after boot). |
| **CFG** | `CFG:DEB12;LED1200;DSP1200;REG:D-A320;SN:XXXXXXXX;` | Configuration in one command. |

### Serial Output (ATC → Sim)

The panel sends on every change as well as on `REQ;`:

```
IN1:01000000;IN2:00000001;IN3:00000011;
```

| Field | Description |
|---|---|
| **IN1:** | 8-bit binary status (HC165 register 1) — Mode keys: ALL, ABV, BLW, ON, TARA, TA, STBY, THRT. |
| **IN2:** | 8-bit binary status (HC165 register 2) — Numeric keys 0–7. |
| **IN3:** | 7-bit binary status — ON, SYS2, SYS1, IDENT, AUTO, CLR, STBY. |

On startup or on `VER;`:

```
IDENT:ATC, v1.2 MAQ, SN:ATC-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

---

## DIAG Menu (Hardware Test)

### Access

- **Serial:** Send `DIAG;` (within 30 s after boot)
- **Button combination:** Button combo (CLR + numeric keys)

### Menu Navigation

- **CLR** = Enter/Select
- **Key 2** = Next/Up
- **Key 0** = Previous/Down
- Auto-exit after 15 s inactivity

Tests: LED test, Button test, Info display, Segment test with animation.

---

## Hardware

- **Controller:** Arduino Mega (ATmega2560)
- **Display:** 1× MAX7219 (4 digits)
- **LED driver:** 1× 74HC595 shift register (8 backlight LEDs)
- **Inputs:** 2× 74HC165 shift registers (16 buttons) + 7 direct pins
- **Direct LEDs:** ATC FAIL (Pin 7, active LOW)
- **PWM:** Brightness (Pin 3)
- **Panel ID:** `ATC, v1.2 MAQ` | Firmware: `1.2`

---

## Button Mapping

| Register | Bit | Button | LED |
|---|---|---|---|
| **IN1** | 0 | ALL | Backlight bit 0 |
| | 1 | ABV | Backlight bit 1 |
| | 2 | BLW | Backlight bit 2 |
| | 3 | ON | Backlight bit 3 |
| | 4 | TARA | Backlight bit 4 |
| | 5 | TA | Backlight bit 5 |
| | 6 | STBY | Backlight bit 6 |
| | 7 | THRT | Backlight bit 7 |
| **IN2** | 0–7 | Digits 0–7 | — |
| **IN3 (direct)** | 0 | ON (Pin 8) | — |
| | 1 | SYS2 (Pin 9) | — |
| | 2 | SYS1 (Pin 10) | — |
| | 3 | IDENT (Pin 14) | ATC FAIL LED |
| | 4 | AUTO (Pin 15) | — |
| | 5 | CLR (Pin 16) | — |
| | 6 | STBY (Pin A0) | — |

---

## EEPROM Layout (v3)

Standard v3 layout from EEPROM_BASE_ADDR=0.

---

## Settings (Serial)

| Command | Effect |
|---|---|
| `SET ENA:0815;` | Activate settings mode (PIN = 0815) |
| `SET FW:1.5;` | Set PCB version |
| `SET ACID:D-AIDA;` | Set A/C registration |
| `SET SN:XXXXXXXX;` | Set serial number (8 hex digits) |
| `SET WRITE;` / `SET WRI:YES;` | Save configuration + restart |
| `SET EXIT;` | Deactivate settings mode |


---

## ฉบับภาษาไทย

# ATC – แผง ATC/Transponder (สไตล์ Airbus A320)

## ภาพรวม

**SSF ATC** เป็นแผง ATC/Transponder ที่จำลองเหมือนจริงในดีไซน์ Airbus A320 สำหรับ Microsoft Flight Simulator (MSFS2020 / MSFS2024) แสดงรหัสสควอคบนจอ MAX7219 4 หลัก และสื่อสารสองทิศทางกับซิมูเลเตอร์ผ่าน USB-Serial

**คุณสมบัติ:**
- จอแสดงผล 7 ส่วน MAX7219 (4 หลัก) สำหรับรหัสสควอค
- ปุ่มกดมีไฟหลัง 9 ปุ่ม (ปุ่มโหมด + ปุ่มตัวเลข 0–7)
- ปุ่มฟังก์ชันโดยตรง 7 ปุ่ม (ON, SYS2, SYS1, IDENT, AUTO, CLR, STBY)
- ไฟ LED ATC FAIL (ขับโดยตรง)
- เมนู DIAG ในตัว (ภายใน 30 วินาทีหลังเริ่มต้น)
- การตรวจจับออฟไลน์พร้อมข้อความ "noFS" บนจอ
- ฟังก์ชันเลื่อนสำหรับข้อความ > 4 ตัวอักษร
- การกำหนดค่า EEPROM

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1  
**โปรโตคอล:** แบบบรรทัด, คั่นด้วยเซมิโคลอน (`;`)

### อินพุต Serial (Sim → ATC)

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **LED1** | `LED1:11111111;` | LED แสงพื้นหลัง (74HC595) สตริงไบนารี 8 บิต |
| **LED2** | `LED2:00000001;` | บิต 0 = ไฟ LED ATC FAIL |
| **BL** | `BL:200;` | ความสว่าง PWM (0–255) |
| **DISP_BL** | `DISP_BL:10;` | ความสว่างจอ MAX7219 (0–15) |
| **DSP1** | `DSP1:1234;` | รหัสสควอค (4 หลัก, XPDR ทำงาน) |
| **DSP2** | `DSP2:2;` | จำนวนหลักสควอคที่ป้อน (0–4) |
| **DSP3** | `DSP3:5678;` | รหัสสควอคสำรอง/บางส่วน |
| **STATE** | `STATE:01;` | `01`/`1` = โฮสต์ออนไลน์, `00`/`0` = โฮสต์ออฟไลน์ → "noFS" บนจอ |
| **REQ** | `REQ;` | ขอสถานะทันที |
| **VER** | `VER;` | ขอการตอบกลับ IDENT |
| **DIAG** | `DIAG;` | เริ่มเมนู DIAG (ภายใน 30 วินาทีหลังเริ่มต้น) |

### เอาต์พุต Serial (ATC → Sim)

```
IN1:01000000;IN2:00000001;IN3:00000011;
```

| ฟิลด์ | คำอธิบาย |
|---|---|
| **IN1:** | ปุ่มโหมด: ALL, ABV, BLW, ON, TARA, TA, STBY, THRT |
| **IN2:** | ปุ่มตัวเลข 0–7 |
| **IN3:** | ปุ่มโดยตรง: ON, SYS2, SYS1, IDENT, AUTO, CLR, STBY |

ข้อมูลระบุ: `IDENT:ATC, v1.2 MAQ, SN:ATC-XXXXXXXX;STATE:RUNNING;REG:D-A320;`

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Mega (ATmega2560)
- **จอแสดงผล:** MAX7219 1 ชุด (4 หลัก)
- **ไดรเวอร์ LED:** 74HC595 1 ชุด (LED แสงพื้นหลัง 8 ดวง)
- **อินพุต:** 74HC165 2 ชุด (16 ปุ่ม) + ขาโดยตรง 7 ขา
- **LED โดยตรง:** ATC FAIL (ขา 7, active LOW)
- **รหัสแผง:** `ATC, v1.2 MAQ` | เฟิร์มแวร์: `1.2`

---

## การตั้งค่า (Serial)

| คำสั่ง | ผลลัพธ์ |
|---|---|
| `SET ENA:0815;` | เปิดใช้งานโหมดตั้งค่า |
| `SET FW:1.5;` | ตั้งค่าเวอร์ชัน PCB |
| `SET ACID:D-AIDA;` | ตั้งค่าทะเบียน A/C |
| `SET SN:XXXXXXXX;` | ตั้งค่าหมายเลขซีเรียล (8 หลักฐานสิบหก) |
| `SET WRI:YES;` | บันทึก + เริ่มใหม่ |
| `SET EXIT;` | ปิดโหมดตั้งค่า |
