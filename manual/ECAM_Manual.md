# ECAM – ECAM Control Panel (Airbus A320 Style)

## Übersicht

Das **SSF ECAM** ist ein originalgetreues ECAM Control Panel im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es steuert die ECAM-Seitenumschaltung und T/O-Konfiguration und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- Bis zu 32 hintergrundbeleuchtete Taster mit LED-Rückmeldung (4 Schieberegister-Ketten)
- 2 analoge Potentiometer (Helligkeit etc.)
- Integrierte DIAG-Routine mit LED-Walk und Muster-Test
- Non-blocking Boot-Sequenz (800 ms)
- EEPROM-Konfiguration (A/C-Registrierung, PCB-Version, Seriennummer)
- HW-Revision-Erkennung (Rev 1–2)
- Optimierte Serial-Verarbeitung (char-Puffer statt Arduino String)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → ECAM)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED1** | `LED1:11111111;` | LED-Byte 1 (Schieberegister). 8-Bit Binärstring. |
| **LED2** | `LED2:00110011;` | LED-Byte 2. 8-Bit Binärstring. |
| **LED3** | `LED3:01010101;` | LED-Byte 3. 8-Bit Binärstring. |
| **LED4** | `LED4:00001111;` | LED-Byte 4. 8-Bit Binärstring. |
| **BL** | `BL:200;` | PWM-Helligkeit Backlight (0–255). |
| **AN** | `AN:150;` | PWM-Helligkeit Annunciator (0–255). |
| **SMO_THR** | `SMO_THR:10;` | Glättungs-Schwellwert (1–100). |
| **SMO_SAM** | `SMO_SAM:4;` | Anzahl Samples für Glättung (1–8). |
| **SMO_DLY** | `SMO_DLY:300;` | Glättungsverzögerung in µs (50–1000). |
| **POT_DB** | `POT_DB:10;` | Poti-Totzone (0–200). |
| **DIAG** | `DIAG;` | DIAG-Routine starten. |
| **VER / VERSION** | `VER;` | IDENT-Antwort anfordern. |
| **REQ** | `REQ;` | Status sofort anfordern. |
| **RESET** | `RESET;` | Software-Reset via Watchdog. |

### Serial Output (ECAM → Sim)

Das Panel sendet bei jeder Änderung sowie auf `REQ;`:

```
POT1:0512;POT2:0768;IN1:11111111;IN2:00000000;IN3:00000000;IN4:00000000;
```

| Feld | Beschreibung |
|---|---|
| **POT1:** | Potentiometer 1 (4-stellig, 0–1023). |
| **POT2:** | Potentiometer 2 (4-stellig, 0–1023). |
| **IN1–IN4:** | 4× 8-Bit Binärstatus der Eingänge (HC165-Kette). |

Beim Start oder auf `VER;`:

```
IDENT:ECAM, v1.2 MAQ, SN:ECAM-XXXXXXXX;STATE:RUNNING;HWREV:1;
```

---

## DIAG (Hardware-Test)

### Zugang

- **Serial:** `DIAG;` senden
- **Tastenkombi am Panel:** IN1 = 00000000 + IN2 = 00101001 drücken

### Ablauf

1. **LED Walk:** Alle 32 LEDs werden einzeln nacheinander angesteuert (60 ms pro Schritt)
2. **Muster-Test:** 0xAA → 0x55 → 0xFF (je 200 ms, BL/AN = 200)
3. **Alle Annunciatoren AN:** Volle Helligkeit (255/255) für 300 ms
4. **Alles AUS**

Die DIAG-Kombination wird nach 15 s automatisch zurückgesetzt.

### Boot-Sequenz (non-blocking)

0xAA → 0x55 → 0xFF → 0x00 (je 200 ms, insgesamt ~800 ms). Serial wird während der Boot-Phase nicht blockiert.

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **LED-Treiber:** 4× 74HC595 in Kette (bis zu 32 LEDs)
- **Eingänge:** 4× 74HC165 in Kette (bis zu 32 Taster)
- **PWM:** Backlight (Pin 6), Annunciator (Pin 5)
- **Potentiometer:** A9, A10
- **HW-Revision:** Gespeichert in EEPROM Addr 0
- **Panel-ID:** `ECAM, v1.2 MAQ` | Firmware: `1.2`

---

## EEPROM-Layout (v3)

| Adresse | Inhalt |
|---|---|
| 0 | HW-Revision |
| 10–11 | Magic Number (`0xA55A`) |
| 12 | Format-Version (`3`) |
| 13–20 | A/C-Registrierung |
| 21–28 | PCB-Version |
| 29–37 | Seriennummer |
| 38 | Checksumme |

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

# ECAM – ECAM Control Panel (Airbus A320 Style)

## Overview

The **SSF ECAM** is a faithful ECAM Control Panel in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It controls ECAM page switching and T/O configuration, communicating bidirectionally with the simulator via USB-Serial.

**Features:**
- Up to 32 backlit buttons with LED feedback (4 shift register chains)
- 2 analog potentiometers (brightness etc.)
- Integrated DIAG routine with LED walk and pattern test
- Non-blocking boot sequence (800 ms)
- EEPROM configuration (A/C registration, PCB version, serial number)
- HW revision detection (Rev 1–2)
- Optimized serial processing (char buffer instead of Arduino String)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`), CR/LF accepted as delimiter.

### Serial Input (Sim → ECAM)

| Command | Format | Description |
|---|---|---|
| **LED1** | `LED1:11111111;` | LED byte 1 (shift register). 8-bit binary string. |
| **LED2** | `LED2:00110011;` | LED byte 2. 8-bit binary string. |
| **LED3** | `LED3:01010101;` | LED byte 3. 8-bit binary string. |
| **LED4** | `LED4:00001111;` | LED byte 4. 8-bit binary string. |
| **BL** | `BL:200;` | PWM backlight brightness (0–255). |
| **AN** | `AN:150;` | PWM annunciator brightness (0–255). |
| **SMO_THR** | `SMO_THR:10;` | Smoothing threshold (1–100). |
| **SMO_SAM** | `SMO_SAM:4;` | Smoothing sample count (1–8). |
| **SMO_DLY** | `SMO_DLY:300;` | Smoothing delay in µs (50–1000). |
| **POT_DB** | `POT_DB:10;` | Pot deadband (0–200). |
| **DIAG** | `DIAG;` | Start DIAG routine. |
| **VER / VERSION** | `VER;` | Request IDENT response. |
| **REQ** | `REQ;` | Request immediate status. |
| **RESET** | `RESET;` | Software reset via watchdog. |

### Serial Output (ECAM → Sim)

The panel sends on every change as well as on `REQ;`:

```
POT1:0512;POT2:0768;IN1:11111111;IN2:00000000;IN3:00000000;IN4:00000000;
```

| Field | Description |
|---|---|
| **POT1:** | Potentiometer 1 (4-digit, 0–1023). |
| **POT2:** | Potentiometer 2 (4-digit, 0–1023). |
| **IN1–IN4:** | 4× 8-bit binary input status (HC165 chain). |

On startup or on `VER;`:

```
IDENT:ECAM, v1.2 MAQ, SN:ECAM-XXXXXXXX;STATE:RUNNING;HWREV:1;
```

---

## DIAG (Hardware Test)

### Access

- **Serial:** Send `DIAG;`
- **Button combination:** Press IN1 = 00000000 + IN2 = 00101001

### Sequence

1. **LED Walk:** All 32 LEDs lit individually in sequence (60 ms each)
2. **Pattern Test:** 0xAA → 0x55 → 0xFF (200 ms each, BL/AN = 200)
3. **All Annunciators ON:** Full brightness (255/255) for 300 ms
4. **All OFF**

DIAG combination auto-resets after 15 seconds.

### Boot Sequence (non-blocking)

0xAA → 0x55 → 0xFF → 0x00 (200 ms each, ~800 ms total). Serial is not blocked during boot.

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **LED driver:** 4× 74HC595 chained (up to 32 LEDs)
- **Inputs:** 4× 74HC165 chained (up to 32 buttons)
- **PWM:** Backlight (Pin 6), Annunciator (Pin 5)
- **Potentiometers:** A9, A10
- **HW Revision:** Stored in EEPROM addr 0
- **Panel ID:** `ECAM, v1.2 MAQ` | Firmware: `1.2`

---

## EEPROM Layout (v3)

| Address | Content |
|---|---|
| 0 | HW Revision |
| 10–11 | Magic Number (`0xA55A`) |
| 12 | Format Version (`3`) |
| 13–20 | A/C Registration |
| 21–28 | PCB Version |
| 29–37 | Serial Number |
| 38 | Checksum |

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

# ECAM – แผงควบคุม ECAM (สไตล์ Airbus A320)

## ภาพรวม

**SSF ECAM** เป็นแผงควบคุม ECAM ที่จำลองเหมือนจริงในดีไซน์ Airbus A320 สำหรับ Microsoft Flight Simulator (MSFS2020 / MSFS2024) ควบคุมการสลับหน้า ECAM และการกำหนดค่า T/O สื่อสารสองทิศทางกับซิมูเลเตอร์ผ่าน USB-Serial

**คุณสมบัติ:**
- ปุ่มกดมีไฟหลังสูงสุด 32 ปุ่ม พร้อมการตอบสนองด้วย LED (ชิฟต์รีจิสเตอร์ 4 ชุด)
- โพเทนชิโอมิเตอร์อนาล็อก 2 ตัว
- รูทีน DIAG ในตัวพร้อม LED Walk และทดสอบรูปแบบ
- ลำดับการเริ่มต้นแบบไม่บล็อก (800 มิลลิวินาที)
- การกำหนดค่า EEPROM
- การตรวจจับรุ่น HW (รุ่น 1–2)
- การประมวลผล Serial ที่เหมาะสม (char buffer แทน Arduino String)

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1  
**โปรโตคอล:** แบบบรรทัด, คั่นด้วยเซมิโคลอน (`;`)

### อินพุต Serial (Sim → ECAM)

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **LED1–LED4** | `LED1:11111111;` | ไบต์ LED 1–4 สตริงไบนารี 8 บิต |
| **BL** | `BL:200;` | ความสว่าง PWM แสงพื้นหลัง (0–255) |
| **AN** | `AN:150;` | ความสว่าง PWM ตัวประกาศ (0–255) |
| **SMO_THR** | `SMO_THR:10;` | เกณฑ์การปรับเรียบ (1–100) |
| **POT_DB** | `POT_DB:10;` | โซนตายโพเทนชิโอมิเตอร์ (0–200) |
| **DIAG** | `DIAG;` | เริ่มรูทีน DIAG |
| **VER** | `VER;` | ขอการตอบกลับ IDENT |
| **REQ** | `REQ;` | ขอสถานะทันที |
| **RESET** | `RESET;` | รีเซ็ตซอฟต์แวร์ |

### เอาต์พุต Serial (ECAM → Sim)

```
POT1:0512;POT2:0768;IN1:11111111;IN2:00000000;IN3:00000000;IN4:00000000;
```

| ฟิลด์ | คำอธิบาย |
|---|---|
| **POT1–POT2:** | โพเทนชิโอมิเตอร์ (4 หลัก, 0–1023) |
| **IN1–IN4:** | สถานะอินพุต 4×8 บิต (HC165 chain) |

ข้อมูลระบุ: `IDENT:ECAM, v1.2 MAQ, SN:ECAM-XXXXXXXX;STATE:RUNNING;HWREV:1;`

---

## DIAG (ทดสอบฮาร์ดแวร์)

- **Serial:** ส่ง `DIAG;`
- **การกดปุ่ม:** IN1 = 00000000 + IN2 = 00101001
- **ลำดับ:** LED Walk (32 ดวง) → ทดสอบรูปแบบ → ตัวประกาศทั้งหมด → ปิดทั้งหมด
- รีเซ็ตอัตโนมัติหลังจาก 15 วินาที

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Pro Micro (ATmega32U4)
- **LED:** 74HC595 4 ชุดต่ออนุกรม (สูงสุด 32 ดวง)
- **อินพุต:** 74HC165 4 ชุดต่ออนุกรม (สูงสุด 32 ปุ่ม)
- **PWM:** แสงพื้นหลัง (ขา 6), ตัวประกาศ (ขา 5)
- **รหัสแผง:** `ECAM, v1.2 MAQ` | เฟิร์มแวร์: `1.2`

---

## การตั้งค่า (Serial)

| คำสั่ง | ผลลัพธ์ |
|---|---|
| `SET ENA:0815;` | เปิดใช้งานโหมดตั้งค่า |
| `SET SN:XXXXXXXX;` | ตั้งค่าหมายเลขซีเรียล |
| `SET WRI:YES;` | บันทึก + เริ่มใหม่ |
| `SET EXIT;` | ปิดโหมดตั้งค่า |
