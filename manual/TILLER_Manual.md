# TILLER – Tiller Captain Panel (Airbus A320 Style)

## Übersicht

Das **SSF TILLER** ist ein Tiller-Panel (CPT-Seite) im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es erfasst die Tiller-Position über einen Hall-Sensor und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- Hall-Sensor (analog) für präzise Tiller-Positionserfassung
- 8 hintergrundbeleuchtete LEDs (1× 74HC595)
- 1 physischer Taster
- Konfigurierbare Hall-Sensor-Glättung (1–60 Samples)
- Konfigurierbare Totzone (0–50)
- Event-gesteuerte Datenübertragung (sendet nur bei Änderung)
- POST (Power-On Self Test) mit LED-Blink-Sequenz
- EEPROM-Konfiguration (Seriennummer)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → TILLER)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED1** | `LED1:11111111;` | Schieberegister-LEDs. 8-Bit Binärstring oder Hex. |
| **BR** | `BR:200;` | Helligkeit PWM (0–255, invertiert: 255 = voll hell). |
| **SMO** | `SMO:60;` | Hall-Sensor Glättungs-Samples (1–60). |
| **DBD** | `DBD:5;` | Hall-Sensor Totzone (0–50). |
| **IDENT** | `IDENT;` | Identifikation anfordern. |
| **VER** | `VER;` | Firmware-Version anfordern. |
| **REQ** | `REQ;` | Aktuellen Status anfordern. |
| **DIAG** | `DIAG;` | DIAG-Modus starten. |
| **RESET** | `RESET;` | Software-Reset via Watchdog. |

### Serial Output (TILLER → Sim)

**Event-gesteuert (nur bei Änderung):**

```
IN1:00000001;A7:0512;
```

| Feld | Beschreibung |
|---|---|
| **IN1:** | Taster-Status (00000001 = gedrückt, 00000000 = losgelassen). |
| **A7:** | Hall-Sensor-Wert (4-stellig, 0–1023), totzonengefiltert. |

**READY-Signal beim Boot:**
```
READY;
```

**REQ-Antwort:**
```
LED1:11111111;BR:200;
```

---

## DIAG / POST

- **POST (Power-On Self Test):** Alle LEDs blinken 2× (~1 s): Rampe auf → Halten → Rampe ab → Halten
- **DIAG-Modus:** Aktiv über `DIAG;`-Befehl, zyklische Helligkeitsänderung

---

## Hardware

- **Controller:** Arduino Nano/Uno-kompatibel (ATmega328P)
- **LED-Treiber:** 1× 74HC595 Schieberegister (8 LEDs)
- **Taster:** Pin 4 (INPUT_PULLUP, active LOW)
- **Hall-Sensor:** A7 (analog)
- **PWM:** Helligkeit Pin 3 (invertiert: logisch 255 → PWM 0)
- **Panel-ID:** `TILLER 1 CPT, v1.2 MAQ` | Firmware: `1.2`

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

# TILLER – Tiller Captain Panel (Airbus A320 Style)

## Overview

The **SSF TILLER** is a Tiller Panel (CPT side) in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It captures tiller position via a Hall sensor and communicates bidirectionally with the simulator via USB-Serial.

**Features:**
- Hall sensor (analog) for precise tiller position capture
- 8 backlit LEDs (1× 74HC595)
- 1 physical button
- Configurable Hall sensor smoothing (1–60 samples)
- Configurable deadband (0–50)
- Event-driven data transmission (sends only on change)
- POST (Power-On Self Test) with LED blink sequence
- EEPROM configuration (serial number)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`).

### Serial Input (Sim → TILLER)

| Command | Format | Description |
|---|---|---|
| **LED1** | `LED1:11111111;` | Shift register LEDs. 8-bit binary or hex. |
| **BR** | `BR:200;` | Brightness PWM (0–255, inverted: 255 = full bright). |
| **SMO** | `SMO:60;` | Hall sensor smoothing samples (1–60). |
| **DBD** | `DBD:5;` | Hall sensor deadband (0–50). |
| **IDENT** | `IDENT;` | Request identification. |
| **VER** | `VER;` | Request firmware version. |
| **REQ** | `REQ;` | Request current status. |
| **DIAG** | `DIAG;` | Start DIAG mode. |
| **RESET** | `RESET;` | Software reset via watchdog. |

### Serial Output (TILLER → Sim)

**Event-driven (only on change):**

```
IN1:00000001;A7:0512;
```

| Field | Description |
|---|---|
| **IN1:** | Button state (00000001 = pressed, 00000000 = released). |
| **A7:** | Hall sensor value (4-digit, 0–1023), deadband filtered. |

**READY signal on boot:** `READY;`

**REQ response:** `LED1:11111111;BR:200;`

---

## DIAG / POST

- **POST:** All LEDs blink 2× (~1 s): ramp up → hold → ramp down → hold
- **DIAG mode:** Active via `DIAG;` command, cycles brightness

---

## Hardware

- **Controller:** Arduino Nano/Uno compatible (ATmega328P)
- **LED driver:** 1× 74HC595 shift register (8 LEDs)
- **Button:** Pin 4 (INPUT_PULLUP, active LOW)
- **Hall sensor:** A7 (analog)
- **PWM:** Brightness Pin 3 (inverted: logical 255 → PWM 0)
- **Panel ID:** `TILLER 1 CPT, v1.2 MAQ` | Firmware: `1.2`

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

# TILLER – แผง Tiller Captain (สไตล์ Airbus A320)

## ภาพรวม

**SSF TILLER** เป็นแผง Tiller (ฝั่ง CPT) ในดีไซน์ Airbus A320 สำหรับ MSFS จับตำแหน่ง Tiller ผ่าน Hall Sensor และสื่อสารสองทิศทางกับซิมูเลเตอร์ผ่าน USB-Serial

**คุณสมบัติ:**
- Hall Sensor สำหรับจับตำแหน่ง Tiller
- LED 8 ดวง (74HC595 1 ชุด)
- ปุ่มกด 1 ปุ่ม
- การปรับเรียบ Hall Sensor (1–60 ตัวอย่าง)
- โซนตาย (0–50)
- ส่งข้อมูลเมื่อมีการเปลี่ยนแปลงเท่านั้น

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1

### อินพุต Serial

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **LED1** | `LED1:11111111;` | LED ชิฟต์รีจิสเตอร์ |
| **BR** | `BR:200;` | ความสว่าง (0–255, กลับด้าน) |
| **SMO** | `SMO:60;` | การปรับเรียบ (1–60) |
| **DBD** | `DBD:5;` | โซนตาย (0–50) |

### เอาต์พุต Serial (เมื่อมีการเปลี่ยนแปลง)

```
IN1:00000001;A7:0512;
READY;   (เมื่อเริ่มต้น)
```

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Nano/Uno (ATmega328P)
- **LED:** 74HC595 1 ชุด (8 ดวง)
- **ปุ่ม:** ขา 4 | **Hall:** A7
- **รหัสแผง:** `TILLER 1 CPT, v1.2 MAQ` | เฟิร์มแวร์: `1.2`
