# RUD – Rudder Trim Panel (Airbus A320 Style)

## Übersicht

Das **SSF RUD** ist ein Rudder-Trim-Panel im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es zeigt den Rudder-Trim-Wert auf einem 4-stelligen MAX7219-Display an und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- MAX7219 7-Segment-Display (4 Digits) für Trim-Wert (z. B. `A 0.4`, `L12.5`)
- 32 hintergrundbeleuchtete LEDs (4× 74HC595 Daisy-Chain)
- 16 Taster (2× HC165 Schieberegister)
- 1 analoges Potentiometer
- Integrierte DIAG-Routine (LED-Walk, Display-Walk, Segment-Test)
- Boot-Sequenz mit Versionsanzeige auf dem Display
- EEPROM-Konfiguration (A/C-Registrierung, PCB-Version, Seriennummer)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → RUD)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED1** | `LED1:11111111;` | LED-Bits 24–31 (MSB der 32-Bit-Kette). |
| **LED2** | `LED2:00110011;` | LED-Bits 16–23. |
| **LED3** | `LED3:01010101;` | LED-Bits 8–15. |
| **LED4** | `LED4:00001111;` | LED-Bits 0–7 (LSB). |
| **BL** | `BL:200;` | PWM-Helligkeit Backlight + Annunciator (0–255, beide gleich). |
| **DISP_BL** | `DISP_BL:10;` | Helligkeit MAX7219-Display (0–15). |
| **DSP1** | `DSP1:0.400;` | Trim-Wert als Float-String (z. B. "0.400" → Anzeige "A 0.4"). |
| **STATE** | `STATE:01;` | Akzeptiert für Protokoll-Parität (keine HW-Wirkung). |
| **REQ** | `REQ;` | Status sofort anfordern. |
| **VER** | `VER;` | IDENT-Antwort anfordern. |
| **DIAG** | `DIAG;` | DIAG-Routine starten. |
| **PCB** | `PCB;` | PCB-Version abfragen. |
| **EXIT** | `EXIT;` | Einstellungsmodus beenden. |

### Serial Output (RUD → Sim)

Das Panel sendet bei jeder Änderung sowie auf `REQ;`:

```
IN1:01000000;IN2:00000001;RT1:0;RT2:0;POT:0512;
```

| Feld | Beschreibung |
|---|---|
| **IN1:** | 8-Bit Binärstatus (HC165 Register 1). |
| **IN2:** | 8-Bit Binärstatus (HC165 Register 2). |
| **RT1/RT2:** | Rotary-Encoder-Deltas (derzeit hardcodiert 0). |
| **POT:** | Analogwert von A3 (0–1023). |

Beim Start oder auf `VER;`:

```
IDENT:RUD, v1.3 MAQ, SN:RUD-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

---

## Display-Format

| Wert | Anzeige |
|---|---|
| `0.400` | `A 0.4` (A = rechts/away) |
| `-12.500` | `L12.5` (L = links) |

Der Wert wird auf 0.0–20.0 begrenzt und mit einer Nachkommastelle dargestellt.

---

## DIAG (Hardware-Test)

### Zugang

- **Serial:** `DIAG;` senden

### Ablauf

1. **LED Walk** (0–3200 ms): Alle 32 LEDs nacheinander (100 ms pro Schritt)
2. **Display Walk** (3200–8200 ms): Ziffern 0–9, DP bei geraden Zahlen
3. **Segment Walk** (8200–13200 ms): Jedes 7-Segment einzeln
4. **Alle AN** (13200–15200 ms): Alle Segmente + alle LEDs

### Boot-Sequenz

Blitz-Muster → FW-Version → "Id" → ACID (erste 2 Zeichen) → ACID (letzte 4 Zeichen)

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **Display:** 1× MAX7219 (4 Digits)
- **LED-Treiber:** 4× 74HC595 Daisy-Chain (32 LEDs)
- **Eingänge:** 2× 74HC165 Schieberegister (16 Taster)
- **PWM:** Backlight + Annunciator (Pin 9, 10)
- **Potentiometer:** A3
- **Panel-ID:** `RUD, v1.3 MAQ` | Firmware: `1.3`

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

# RUD – Rudder Trim Panel (Airbus A320 Style)

## Overview

The **SSF RUD** is a Rudder Trim Panel in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It displays the rudder trim value on a 4-digit MAX7219 display and communicates bidirectionally with the simulator via USB-Serial.

**Features:**
- MAX7219 7-segment display (4 digits) for trim value (e.g. `A 0.4`, `L12.5`)
- 32 backlit LEDs (4× 74HC595 daisy chain)
- 16 buttons (2× HC165 shift registers)
- 1 analog potentiometer
- Integrated DIAG routine (LED walk, display walk, segment test)
- Boot sequence with version display on screen
- EEPROM configuration (A/C registration, PCB version, serial number)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`).

### Serial Input (Sim → RUD)

| Command | Format | Description |
|---|---|---|
| **LED1** | `LED1:11111111;` | LED bits 24–31 (MSB of 32-bit chain). |
| **LED2** | `LED2:00110011;` | LED bits 16–23. |
| **LED3** | `LED3:01010101;` | LED bits 8–15. |
| **LED4** | `LED4:00001111;` | LED bits 0–7 (LSB). |
| **BL** | `BL:200;` | PWM brightness backlight + annunciator (0–255, both equal). |
| **DISP_BL** | `DISP_BL:10;` | MAX7219 display brightness (0–15). |
| **DSP1** | `DSP1:0.400;` | Trim value as float string (e.g. "0.400" → displays "A 0.4"). |
| **STATE** | `STATE:01;` | Accepted for protocol parity (no HW effect). |
| **REQ** | `REQ;` | Request immediate status. |
| **VER** | `VER;` | Request IDENT response. |
| **DIAG** | `DIAG;` | Start DIAG routine. |
| **PCB** | `PCB;` | Query PCB version. |
| **EXIT** | `EXIT;` | Exit settings mode. |

### Serial Output (RUD → Sim)

```
IN1:01000000;IN2:00000001;RT1:0;RT2:0;POT:0512;
```

| Field | Description |
|---|---|
| **IN1:** | 8-bit binary (HC165 register 1). |
| **IN2:** | 8-bit binary (HC165 register 2). |
| **RT1/RT2:** | Rotary encoder deltas (hardcoded 0). |
| **POT:** | Analog reading from A3 (0–1023). |

On startup: `IDENT:RUD, v1.3 MAQ, SN:RUD-XXXXXXXX;STATE:RUNNING;REG:D-A320;`

---

## Display Format

| Value | Display |
|---|---|
| `0.400` | `A 0.4` (A = away/right) |
| `-12.500` | `L12.5` (L = left) |

Value clamped to 0.0–20.0, shown with 1 decimal place.

---

## DIAG (Hardware Test)

1. **LED Walk:** All 32 LEDs sequentially (100 ms each)
2. **Display Walk:** Digits 0–9, DP on even numbers
3. **Segment Walk:** Each 7-segment individually
4. **All ON:** All segments + all LEDs

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **Display:** 1× MAX7219 (4 digits)
- **LED driver:** 4× 74HC595 daisy chain (32 LEDs)
- **Inputs:** 2× 74HC165 shift registers (16 buttons)
- **PWM:** Backlight + Annunciator (Pin 9, 10)
- **Potentiometer:** A3
- **Panel ID:** `RUD, v1.3 MAQ` | Firmware: `1.3`

---

## Settings (Serial)

| Command | Effect |
|---|---|
| `SET ENA:0815;` | Activate settings mode |
| `SET FW:1.5;` | Set PCB version |
| `SET ACID:D-AIDA;` | Set A/C registration |
| `SET SN:XXXXXXXX;` | Set serial number (8 hex digits) |
| `SET WRI:YES;` | Save + restart |
| `SET EXIT;` | Deactivate settings mode |


---

## ฉบับภาษาไทย

# RUD – แผง Rudder Trim (สไตล์ Airbus A320)

## ภาพรวม

**SSF RUD** เป็นแผง Rudder Trim ในดีไซน์ Airbus A320 สำหรับ MSFS แสดงค่า Trim บนจอ MAX7219 4 หลัก และสื่อสารสองทิศทางกับซิมูเลเตอร์ผ่าน USB-Serial

**คุณสมบัติ:**
- จอแสดงผล MAX7219 4 หลัก (เช่น `A 0.4`, `L12.5`)
- LED 32 ดวง (74HC595 4 ชุดต่ออนุกรม)
- ปุ่ม 16 ปุ่ม (HC165 2 ชุด)
- โพเทนชิโอมิเตอร์ 1 ตัว
- รูทีน DIAG ในตัว

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1

### อินพุต Serial

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **LED1–LED4** | `LED1:11111111;` | LED 32 ดวง (4 ไบต์) |
| **BL** | `BL:200;` | ความสว่าง PWM (0–255) |
| **DSP1** | `DSP1:0.400;` | ค่า Trim → แสดง `A 0.4` |
| **DIAG** | `DIAG;` | เริ่มรูทีน DIAG |
| **VER** | `VER;` | ขอ IDENT |

### เอาต์พุต Serial

```
IN1:01000000;IN2:00000001;RT1:0;RT2:0;POT:0512;
```

ข้อมูลระบุ: `IDENT:RUD, v1.3 MAQ, SN:RUD-XXXXXXXX;STATE:RUNNING;REG:D-A320;`

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Pro Micro (ATmega32U4)
- **จอ:** MAX7219 1 ชุด (4 หลัก)
- **LED:** 74HC595 4 ชุด (32 ดวง)
- **รหัสแผง:** `RUD, v1.3 MAQ` | เฟิร์มแวร์: `1.3`
