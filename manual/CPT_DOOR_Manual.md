# CPT_DOOR – Captain Door Panel (Airbus A320 Style)

## Übersicht

Das **SSF CPT_DOOR** ist ein Cockpit-Door-Panel (CPT-Seite) im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es steuert die Cockpit-Tür-Verriegelung und Beleuchtungs-Potentiometer und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- 10 LEDs (8 Backlight-Schieberegister + 2 direkte LEDs: DOOR_FLT, DOOR_OPEN)
- 6 Taster (LOCK, UNLOCK, VIDEO, DFDR_EVT, PNL_DOOR, ADIRS_PRINT)
- 3 Potentiometer (FLT_LGHT_MIP, INTEG LIGHT, FLT_LGHT_PED)
- Non-blocking Boot-Sequenz (150 ms/Phase)
- EEPROM-Konfiguration (A/C-Registrierung, PCB-Version, Seriennummer)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → DOOR)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED1** | `LED1:11111111;` | Backlight-Schieberegister (8 Bits). |
| **LED2** | `LED2:00000011;` | Direkte LEDs. Bit 0 = DOOR_FLT, Bit 1 = DOOR_OPEN. |
| **BL** | `BL:200;` | PWM-Helligkeit Backlight (0–255). |
| **SMO_THR** | `SMO_THR:10;` | Glättungs-Schwellwert für Potis (1–100). |
| **SMO_SAM** | `SMO_SAM:4;` | Anzahl Samples für Poti-Glättung (1–8). |
| **SMO_DLY** | `SMO_DLY:300;` | Glättungsverzögerung in µs (50–1000). |
| **REQ** | `REQ;` | Status sofort anfordern. |
| **VER / VERSION** | `VER;` | IDENT-Antwort anfordern. |
| **RESET** | `RESET;` | Software-Reset via Watchdog. |

### Serial Output (DOOR → Sim)

Das Panel sendet bei jeder Änderung sowie auf `REQ;`:

```
IN1:00101101;A0:0512;A1:0768;A2:1023;
```

| Feld | Beschreibung |
|---|---|
| **IN1:** | 6-Bit Taster-Status (Bits 0–5). |
| **A0:** | FLT_LGHT_MIP Poti (0–1023). |
| **A1:** | INTEG LIGHT Poti (0–1023). |
| **A2:** | FLT_LGHT_PED Poti (0–1023). |

Beim Start:

```
IDENT:CPT_DOOR, v1.0 MAQ, SN:DOOR-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

---

## Taster-Mapping

| IN1 Bit | Taster | Pin |
|---|---|---|
| 0 | LOCK | 4 |
| 1 | UNLOCK | 5 |
| 2 | VIDEO | 7 |
| 3 | DFDR_EVT | 8 |
| 4 | PNL_DOOR | 10 |
| 5 | ADIRS_PRINT | 21 |

## Potentiometer-Mapping

| Analog-Pin | Funktion |
|---|---|
| A0 | FLT_LGHT_MIP (Flight Light MIP) |
| A1 | INTEG LIGHT (Integ Light) |
| A2 | FLT_LGHT_PED (Flight Light Pedestal) |

## LED-Mapping

| LED2 Bit | LED | Pin |
|---|---|---|
| 0 | DOOR_FLT | 2 |
| 1 | DOOR_OPEN | 3 |

---

## Boot-Sequenz

Non-blocking (150 ms/Phase): 0xAA + Direkt-LEDs → 0x55 + Direkt-LEDs → 0xFF + Direkt-LEDs → Alles AUS

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **LED-Treiber:** 1× 74HC595 Schieberegister (8 Backlight-LEDs)
- **Direkte LEDs:** DOOR_FLT (Pin 2), DOOR_OPEN (Pin 3)
- **Taster:** 6 direkte Pins (INPUT_PULLUP, active LOW)
- **PWM:** Backlight (Pin 9)
- **Potentiometer:** A0, A1, A2
- **Panel-ID:** `CPT_DOOR, v1.0 MAQ` | Firmware: `1.0`

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

# CPT_DOOR – Captain Door Panel (Airbus A320 Style)

## Overview

The **SSF CPT_DOOR** is a Cockpit Door Panel (CPT side) in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It controls cockpit door locking and lighting potentiometers, communicating bidirectionally with the simulator via USB-Serial.

**Features:**
- 10 LEDs (8 backlight shift register + 2 direct LEDs: DOOR_FLT, DOOR_OPEN)
- 6 buttons (LOCK, UNLOCK, VIDEO, DFDR_EVT, PNL_DOOR, ADIRS_PRINT)
- 3 potentiometers (FLT_LGHT_MIP, INTEG LIGHT, FLT_LGHT_PED)
- Non-blocking boot sequence (150 ms/phase)
- EEPROM configuration (A/C registration, PCB version, serial number)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`).

### Serial Input (Sim → DOOR)

| Command | Format | Description |
|---|---|---|
| **LED1** | `LED1:11111111;` | Backlight shift register (8 bits). |
| **LED2** | `LED2:00000011;` | Direct LEDs. Bit 0 = DOOR_FLT, Bit 1 = DOOR_OPEN. |
| **BL** | `BL:200;` | PWM backlight brightness (0–255). |
| **SMO_THR** | `SMO_THR:10;` | Pot smoothing threshold (1–100). |
| **SMO_SAM** | `SMO_SAM:4;` | Pot smoothing samples (1–8). |
| **SMO_DLY** | `SMO_DLY:300;` | Smoothing delay in µs (50–1000). |
| **REQ** | `REQ;` | Request immediate status. |
| **VER / VERSION** | `VER;` | Request IDENT response. |
| **RESET** | `RESET;` | Software reset via watchdog. |

### Serial Output (DOOR → Sim)

```
IN1:00101101;A0:0512;A1:0768;A2:1023;
```

| Field | Description |
|---|---|
| **IN1:** | 6-bit button state (bits 0–5). |
| **A0:** | FLT_LGHT_MIP pot (0–1023). |
| **A1:** | INTEG LIGHT pot (0–1023). |
| **A2:** | FLT_LGHT_PED pot (0–1023). |

On startup: `IDENT:CPT_DOOR, v1.0 MAQ, SN:DOOR-XXXXXXXX;STATE:RUNNING;REG:D-A320;`

---

## Button Mapping

| IN1 Bit | Button | Pin |
|---|---|---|
| 0 | LOCK | 4 |
| 1 | UNLOCK | 5 |
| 2 | VIDEO | 7 |
| 3 | DFDR_EVT | 8 |
| 4 | PNL_DOOR | 10 |
| 5 | ADIRS_PRINT | 21 |

## Potentiometer Mapping

| Analog Pin | Function |
|---|---|
| A0 | FLT_LGHT_MIP |
| A1 | INTEG LIGHT |
| A2 | FLT_LGHT_PED |

## LED Mapping

| LED2 Bit | LED | Pin |
|---|---|---|
| 0 | DOOR_FLT | 2 |
| 1 | DOOR_OPEN | 3 |

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **LED driver:** 1× 74HC595 (8 backlight LEDs)
- **Direct LEDs:** DOOR_FLT (Pin 2), DOOR_OPEN (Pin 3)
- **Buttons:** 6 direct pins (INPUT_PULLUP, active LOW)
- **PWM:** Backlight (Pin 9)
- **Panel ID:** `CPT_DOOR, v1.0 MAQ` | Firmware: `1.0`

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

# CPT_DOOR – แผงประตูห้องนักบิน (สไตล์ Airbus A320)

## ภาพรวม

**SSF CPT_DOOR** เป็นแผงประตูห้องนักบิน (ฝั่ง CPT) ในดีไซน์ Airbus A320 สำหรับ MSFS ควบคุมการล็อกประตูและโพเทนชิโอมิเตอร์ไฟส่องสว่าง

**คุณสมบัติ:**
- LED 10 ดวง (8 Backlight + 2 โดยตรง)
- ปุ่ม 6 ปุ่ม
- โพเทนชิโอมิเตอร์ 3 ตัว
- ลำดับเริ่มต้นแบบไม่บล็อก

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1

### อินพุต Serial

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **LED1** | `LED1:11111111;` | Backlight (8 บิต) |
| **LED2** | `LED2:00000011;` | DOOR_FLT + DOOR_OPEN |
| **BL** | `BL:200;` | ความสว่าง (0–255) |

### เอาต์พุต Serial

```
IN1:00101101;A0:0512;A1:0768;A2:1023;
```

ข้อมูลระบุ: `IDENT:CPT_DOOR, v1.0 MAQ, SN:DOOR-XXXXXXXX;STATE:RUNNING;REG:D-A320;`

---

## การแมปปุ่ม

| IN1 บิต | ปุ่ม | ขา |
|---|---|---|
| 0 | LOCK | 4 |
| 1 | UNLOCK | 5 |
| 2 | VIDEO | 7 |
| 3 | DFDR_EVT | 8 |
| 4 | PNL_DOOR | 10 |
| 5 | ADIRS_PRINT | 21 |

## การแมป LED

| LED2 บิต | LED | ขา |
|---|---|---|
| 0 | DOOR_FLT | 2 |
| 1 | DOOR_OPEN | 3 |

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Pro Micro (ATmega32U4)
- **LED:** 74HC595 1 ชุด + LED โดยตรง 2 ดวง
- **รหัสแผง:** `CPT_DOOR, v1.0 MAQ` | เฟิร์มแวร์: `1.0`
