# ACP – Audio Control Panel (Airbus A320 Style)

## Übersicht

Das **SSF ACP** ist ein originalgetreues Audio Control Panel im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es steuert die Audio-Funktionen (VHF, HF, PA, etc.) und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- 16–24 hintergrundbeleuchtete Taster mit LED-Rückmeldung (je nach HW-Revision)
- 16 analoge Potentiometer über 4067-Multiplexer (Lautstärkeregler)
- Integrierte DIAG-Kombination (Hardware-Selbsttest)
- EEPROM-Konfiguration (A/C-Registrierung, PCB-Version, Seriennummer)
- HW-Revision-Erkennung (Rev 1: 16 LEDs, Rev 2: 24 LEDs)
- Speicherschonende C-Implementierung (kein `String`-Typ, keine dynamische Allokation)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → ACP)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED1** | `LED1:01010101;` | Backlight-LEDs (74HC595 Register 1). 8-Bit Binärstring. |
| **LED2** | `LED2:11001100;` | Annunciator-LEDs (74HC595 Register 2). 8-Bit Binärstring. |
| **LED3** | `LED3:00110011;` | Zusätzliche LEDs (74HC595 Register 3, nur HW Rev 2). 8-Bit Binärstring. |
| **BL** | `BL:200;` | PWM-Helligkeit Backlight (0–255). |
| **AN** | `AN:150;` | PWM-Helligkeit Annunciator (0–255). |
| **DISP_BL** | `DISP_BL:10;` | Display-Helligkeit (reserviert, kein Display). |
| **SMO_THR** | `SMO_THR:10;` | Glättungs-Schwellwert für MUX-Analogwerte (1–100). |
| **SMO_SAM** | `SMO_SAM:4;` | Anzahl Samples für MUX-Glättung (1–8). |
| **SMO_DLY** | `SMO_DLY:300;` | Verzögerung zwischen MUX-Kanalwechsel in µs (50–1000). |
| **HWREVSET** | `HWREVSET:2,ACP-HW-SET;` | HW-Revision setzen (1 oder 2, mit Passwort). |
| **VER / VERSION** | `VER;` | IDENT-Antwort anfordern. |
| **REQ** | `REQ;` | Status sofort anfordern (MUX/IN1/IN2). |
| **HWREV** | `HWREV;` | HW-Revision abfragen. |
| **RESET** | `RESET;` | Software-Reset via Watchdog. |

### Serial Output (ACP → Sim)

Das Panel sendet bei jeder Änderung (Taster oder Poti) sowie auf `REQ;`:

```
MUX0:0512;MUX1:1023;...MUX15:0000;IN1:01000001;IN2:00001000;
```

| Feld | Beschreibung |
|---|---|
| **MUX0–MUX15:** | 16 Analogwerte (4-stellig, 0–1023) vom 4067-Multiplexer. |
| **IN1:** | 8-Bit Binärstatus der Eingänge (HC165 Register 1). |
| **IN2:** | 8-Bit Binärstatus (HC165 Register 2). |

Beim Start oder auf `VER;`/`IDENT;`:

```
IDENT:ACP 1 CPT, v2.1 MAQ, SN:ACP-XXXXXXXX;STATE:RUNNING;HWREV:1;
```

---

## DIAG (Hardware-Test)

### Zugang

- **Tastenkombi am Panel:** Gleichzeitig IN1 Bit 0 + Bit 6 + IN2 Bit 3 drücken.
- Die DIAG aktiviert automatisch alle LEDs für 15 Sekunden und schaltet dann ab.

### Startup-Boot-Sequenz

Beim Einschalten durchläuft das Panel eine kurze LED-Sequenz:
0xAA → 0x55 → 0xFF → 0x00 (je 150 ms).

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **LED-Treiber:** 2–3× 74HC595 Schieberegister (je nach HW-Rev)
- **Eingänge:** 2× 74HC165 Schieberegister (16 Taster)
- **Analog-MUX:** 1× 4067 16:1 Multiplexer (S0–S3 = A0–A3, Output = A6)
- **PWM:** Backlight (Pin 3), Annunciator (Pin 6)
- **Panel-ID:** `ACP 1 CPT, v2.1 MAQ` | Firmware: `2.1`

---

## EEPROM-Layout (v3)

| Adresse | Inhalt |
|---|---|
| 0 | HW-Revision (separat vom Config-Block) |
| 10–11 | Magic Number (`0xA55A`) |
| 12 | Format-Version (`3`) |
| 13–20 | A/C-Registrierung (8 Zeichen, Leerzeichen-gepadded) |
| 21–28 | PCB-Version (8 Zeichen) |
| 29–37 | Seriennummer (9 Zeichen) |
| 38 | Checksumme |

---

## Settings (Serial)

| Befehl | Wirkung |
|---|---|
| `SET ENA:0815;` | Einstellungsmodus aktivieren (PIN = 0815) |
| `SET FW:1.5;` | PCB-Version setzen (Format: Hauptversion.Nebenversion) |
| `SET ACID:D-AIDA;` | A/C-Registrierung setzen (Format: X-XXXX, max 8 Zeichen) |
| `SET SN:XXXXXXXX;` | Seriennummer setzen (8-stellig Hex) |
| `SET WRITE;` / `SET WRI:YES;` | Konfiguration in EEPROM speichern + Neustart |
| `SET EXIT;` | Einstellungsmodus deaktivieren |


---

## English Version

# ACP – Audio Control Panel (Airbus A320 Style)

## Overview

The **SSF ACP** is a faithful Audio Control Panel in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It controls audio functions (VHF, HF, PA, etc.) and communicates bidirectionally with the simulator via USB-Serial.

**Features:**
- 16–24 backlit buttons with LED feedback (depending on HW revision)
- 16 analog potentiometers via 4067 multiplexer (volume controls)
- Integrated DIAG combination (hardware self-test)
- EEPROM configuration (A/C registration, PCB version, serial number)
- HW revision detection (Rev 1: 16 LEDs, Rev 2: 24 LEDs)
- Memory-efficient C implementation (no `String` type, no dynamic allocation)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`), CR/LF accepted as delimiter.

### Serial Input (Sim → ACP)

| Command | Format | Description |
|---|---|---|
| **LED1** | `LED1:01010101;` | Backlight LEDs (74HC595 register 1). 8-bit binary string. |
| **LED2** | `LED2:11001100;` | Annunciator LEDs (74HC595 register 2). 8-bit binary string. |
| **LED3** | `LED3:00110011;` | Extra LEDs (74HC595 register 3, HW Rev 2 only). 8-bit binary string. |
| **BL** | `BL:200;` | PWM backlight brightness (0–255). |
| **AN** | `AN:150;` | PWM annunciator brightness (0–255). |
| **DISP_BL** | `DISP_BL:10;` | Display brightness (reserved, no display). |
| **SMO_THR** | `SMO_THR:10;` | Smoothing threshold for MUX analog values (1–100). |
| **SMO_SAM** | `SMO_SAM:4;` | Number of samples for MUX smoothing (1–8). |
| **SMO_DLY** | `SMO_DLY:300;` | Delay between MUX channel switches in µs (50–1000). |
| **HWREVSET** | `HWREVSET:2,ACP-HW-SET;` | Set HW revision (1 or 2, with password). |
| **VER / VERSION** | `VER;` | Request IDENT response. |
| **REQ** | `REQ;` | Request immediate status (MUX/IN1/IN2). |
| **HWREV** | `HWREV;` | Query HW revision. |
| **RESET** | `RESET;` | Software reset via watchdog. |

### Serial Output (ACP → Sim)

The panel sends on every change (button or potentiometer) as well as on `REQ;`:

```
MUX0:0512;MUX1:1023;...MUX15:0000;IN1:01000001;IN2:00001000;
```

| Field | Description |
|---|---|
| **MUX0–MUX15:** | 16 analog values (4-digit, 0–1023) from the 4067 multiplexer. |
| **IN1:** | 8-bit binary status of inputs (HC165 register 1). |
| **IN2:** | 8-bit binary status (HC165 register 2). |

On startup or on `VER;`/`IDENT;`:

```
IDENT:ACP 1 CPT, v2.1 MAQ, SN:ACP-XXXXXXXX;STATE:RUNNING;HWREV:1;
```

---

## DIAG (Hardware Test)

### Access

- **Button combination on panel:** Press IN1 bit 0 + bit 6 + IN2 bit 3 simultaneously.
- DIAG activates all LEDs at full brightness for 15 seconds, then auto-off.

### Startup Boot Sequence

On power-up, the panel runs a short LED sequence:
0xAA → 0x55 → 0xFF → 0x00 (150 ms each).

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **LED driver:** 2–3× 74HC595 shift registers (depending on HW Rev)
- **Inputs:** 2× 74HC165 shift registers (16 buttons)
- **Analog MUX:** 1× 4067 16:1 multiplexer (S0–S3 = A0–A3, Output = A6)
- **PWM:** Backlight (Pin 3), Annunciator (Pin 6)
- **Panel ID:** `ACP 1 CPT, v2.1 MAQ` | Firmware: `2.1`

---

## EEPROM Layout (v3)

| Address | Content |
|---|---|
| 0 | HW Revision (separate from config block) |
| 10–11 | Magic Number (`0xA55A`) |
| 12 | Format Version (`3`) |
| 13–20 | A/C Registration (8 characters, space-padded) |
| 21–28 | PCB Version (8 characters) |
| 29–37 | Serial Number (9 characters) |
| 38 | Checksum |

---

## Settings (Serial)

| Command | Effect |
|---|---|
| `SET ENA:0815;` | Activate settings mode (PIN = 0815) |
| `SET FW:1.5;` | Set PCB version (format: major.minor) |
| `SET ACID:D-AIDA;` | Set A/C registration (format: X-XXXX, max 8 characters) |
| `SET SN:XXXXXXXX;` | Set serial number (8 hex digits) |
| `SET WRITE;` / `SET WRI:YES;` | Save configuration to EEPROM + restart |
| `SET EXIT;` | Deactivate settings mode |


---

## ฉบับภาษาไทย

# ACP – แผงควบคุมเสียง (สไตล์ Airbus A320)

## ภาพรวม

**SSF ACP** เป็นแผงควบคุมเสียงที่จำลองเหมือนจริงในดีไซน์ Airbus A320 สำหรับ Microsoft Flight Simulator (MSFS2020 / MSFS2024) ควบคุมฟังก์ชันเสียง (VHF, HF, PA ฯลฯ) และสื่อสารสองทิศทางกับซิมูเลเตอร์ผ่าน USB-Serial

**คุณสมบัติ:**
- ปุ่มกดมีไฟหลัง 16–24 ปุ่ม พร้อมการตอบสนองด้วย LED (ขึ้นอยู่กับรุ่น HW)
- โพเทนชิโอมิเตอร์อนาล็อก 16 ตัว ผ่านมัลติเพล็กเซอร์ 4067 (ตัวควบคุมระดับเสียง)
- การรวม DIAG ในตัว (ทดสอบฮาร์ดแวร์ด้วยตนเอง)
- การกำหนดค่า EEPROM (ทะเบียน A/C, เวอร์ชัน PCB, หมายเลขซีเรียล)
- การตรวจจับรุ่น HW (รุ่น 1: LED 16 ดวง, รุ่น 2: LED 24 ดวง)
- การเขียนโปรแกรมภาษา C ที่ประหยัดหน่วยความจำ (ไม่มีชนิด `String`, ไม่มีการจัดสรรแบบไดนามิก)

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1  
**โปรโตคอล:** แบบบรรทัด, คั่นด้วยเซมิโคลอน (`;`), ยอมรับ CR/LF เป็นตัวคั่น

### อินพุต Serial (Sim → ACP)

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **LED1** | `LED1:01010101;` | LED แสงพื้นหลัง (74HC595 รีจิสเตอร์ 1) สตริงไบนารี 8 บิต |
| **LED2** | `LED2:11001100;` | LED ตัวประกาศ (74HC595 รีจิสเตอร์ 2) สตริงไบนารี 8 บิต |
| **LED3** | `LED3:00110011;` | LED เพิ่มเติม (74HC595 รีจิสเตอร์ 3, เฉพาะ HW รุ่น 2) |
| **BL** | `BL:200;` | ความสว่าง PWM แสงพื้นหลัง (0–255) |
| **AN** | `AN:150;` | ความสว่าง PWM ตัวประกาศ (0–255) |
| **DISP_BL** | `DISP_BL:10;` | ความสว่างจอแสดงผล (สำรอง, ไม่มีจอ) |
| **SMO_THR** | `SMO_THR:10;` | เกณฑ์การปรับเรียบสำหรับค่า MUX (1–100) |
| **SMO_SAM** | `SMO_SAM:4;` | จำนวนตัวอย่างสำหรับการปรับเรียบ MUX (1–8) |
| **SMO_DLY** | `SMO_DLY:300;` | หน่วงระหว่างสลับช่อง MUX ในหน่วย µs (50–1000) |
| **HWREVSET** | `HWREVSET:2,ACP-HW-SET;` | ตั้งค่ารุ่น HW (1 หรือ 2, พร้อมรหัสผ่าน) |
| **VER / VERSION** | `VER;` | ขอการตอบกลับ IDENT |
| **REQ** | `REQ;` | ขอสถานะทันที (MUX/IN1/IN2) |
| **HWREV** | `HWREV;` | สอบถามรุ่น HW |
| **RESET** | `RESET;` | รีเซ็ตซอฟต์แวร์ผ่าน Watchdog |

### เอาต์พุต Serial (ACP → Sim)

แผงจะส่งทุกครั้งที่มีการเปลี่ยนแปลง (ปุ่มกดหรือโพเทนชิโอมิเตอร์) และเมื่อได้รับ `REQ;`:

```
MUX0:0512;MUX1:1023;...MUX15:0000;IN1:01000001;IN2:00001000;
```

| ฟิลด์ | คำอธิบาย |
|---|---|
| **MUX0–MUX15:** | ค่าอนาล็อก 16 ค่า (4 หลัก, 0–1023) จากมัลติเพล็กเซอร์ 4067 |
| **IN1:** | สถานะไบนารี 8 บิตของอินพุต (HC165 รีจิสเตอร์ 1) |
| **IN2:** | สถานะไบนารี 8 บิต (HC165 รีจิสเตอร์ 2) |

เมื่อเริ่มต้นหรือเมื่อได้รับ `VER;`/`IDENT;`:

```
IDENT:ACP 1 CPT, v2.1 MAQ, SN:ACP-XXXXXXXX;STATE:RUNNING;HWREV:1;
```

---

## DIAG (ทดสอบฮาร์ดแวร์)

### การเข้าใช้งาน

- **การกดปุ่มบนแผง:** กด IN1 บิต 0 + บิต 6 + IN2 บิต 3 พร้อมกัน
- DIAG จะเปิด LED ทั้งหมดที่ความสว่างเต็มที่เป็นเวลา 15 วินาที แล้วปิดอัตโนมัติ

### ลำดับการเริ่มต้น

เมื่อเปิดเครื่อง แผงจะแสดงลำดับ LED สั้นๆ:
0xAA → 0x55 → 0xFF → 0x00 (ครั้งละ 150 มิลลิวินาที)

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Pro Micro (ATmega32U4)
- **ไดรเวอร์ LED:** 74HC595 ชิฟต์รีจิสเตอร์ 2–3 ชุด (ขึ้นอยู่กับรุ่น HW)
- **อินพุต:** 74HC165 ชิฟต์รีจิสเตอร์ 2 ชุด (16 ปุ่ม)
- **MUX อนาล็อก:** 4067 มัลติเพล็กเซอร์ 16:1 จำนวน 1 ชุด (S0–S3 = A0–A3, เอาต์พุต = A6)
- **PWM:** แสงพื้นหลัง (ขา 3), ตัวประกาศ (ขา 6)
- **รหัสแผง:** `ACP 1 CPT, v2.1 MAQ` | เฟิร์มแวร์: `2.1`

---

## ผัง EEPROM (v3)

| ที่อยู่ | เนื้อหา |
|---|---|
| 0 | รุ่น HW (แยกจากบล็อกการกำหนดค่า) |
| 10–11 | Magic Number (`0xA55A`) |
| 12 | เวอร์ชันรูปแบบ (`3`) |
| 13–20 | ทะเบียน A/C (8 ตัวอักษร, เติมช่องว่าง) |
| 21–28 | เวอร์ชัน PCB (8 ตัวอักษร) |
| 29–37 | หมายเลขซีเรียล (9 ตัวอักษร) |
| 38 | เช็คซัม |

---

## การตั้งค่า (Serial)

| คำสั่ง | ผลลัพธ์ |
|---|---|
| `SET ENA:0815;` | เปิดใช้งานโหมดตั้งค่า (PIN = 0815) |
| `SET FW:1.5;` | ตั้งค่าเวอร์ชัน PCB (รูปแบบ: หลัก.รอง) |
| `SET ACID:D-AIDA;` | ตั้งค่าทะเบียน A/C (รูปแบบ: X-XXXX, สูงสุด 8 ตัวอักษร) |
| `SET SN:XXXXXXXX;` | ตั้งค่าหมายเลขซีเรียล (8 หลักฐานสิบหก) |
| `SET WRITE;` / `SET WRI:YES;` | บันทึกการกำหนดค่าลง EEPROM + เริ่มใหม่ |
| `SET EXIT;` | ปิดใช้งานโหมดตั้งค่า |
