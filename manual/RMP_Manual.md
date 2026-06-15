# RMP – Radio Management Panel (Airbus A320 Style)

## Übersicht

Das **SSF RMP** ist ein originalgetreues Radio Management Panel im Airbus-A320-Design für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es simuliert vollständig die VHF-/HF-/NAV-Frequenzverwaltung eines echten RMP und kommuniziert bidirektional mit dem Simulator via USB-Serial.

**Features:**
- 2× MAX7219 6-stellige 7-Segment-Displays (VHF/NAV-Frequenzen, Kurs)
- 16 hintergrundbeleuchtete Taster mit LED-Rückmeldung
- 2 Drehgeber (Rotary Encoder) für Frequenz-/Kurswahl
- ILS/MLS-Indikator-LEDs
- Integriertes DIAG-Menü (Hardware-Selbsttest)
- Offline-Erkennung mit automatischer Meldung
- EEPROM-Konfiguration (A/C-Registrierung, PCB-Version)
- Speicherschonende C-Implementierung (kein `String`-Typ, keine dynamische Allokation)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`), CR/LF wird als Trennzeichen akzeptiert.

### Serial Input (Sim → RMP)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED1** | `LED1:01010101;` | Setzt die oberen 8 LEDs (74HC595 Register 1). 8-Bit Binärstring. |
| **LED2** | `LED2:11001100;` | Setzt die unteren 8 LEDs (74HC595 Register 2). 8-Bit Binärstring. |
| **LED3** | `LED3:01;` | Setzt ILS/MLS-Indikator-LEDs. Bit 0 = ILS, Bit 1 = MLS. |
| **BL** | `BL:200;` | PWM-Helligkeit der Taster-LEDs (0–255). |
| **DISP_BL** | `DISP_BL:10;` | Helligkeit der 7-Segment-Displays (0–15). |
| **DSP1** | `DSP1:123.450;` | Setzt das **linke Display** — siehe Display-Formate unten. |
| **DSP2** | `DSP2:118.000;` | Setzt das **rechte Display**. |
| **STATE** | `STATE:01;` | `01`/`1` = Host online, `00`/`0` = Host offline. |
| **CFG** | `CFG:ROT1010;DEB12;...` | Konfiguration (siehe CFG-Befehl unten). |
| **REQ** | `REQ;` | Status sofort anfordern (IN1/IN2/Rotary). |
| **VER / VERSION / IDENT** | `VER;` | Firmware- & PCB-Version auf Displays anzeigen (5 s), dann IDENT-Antwort. |
| **DIAG** | `DIAG;` | Wechselt in das DIAG-Menü (Hardware-Testmodus). |
| **RESET** | `RESET;` | Software-Reset via Watchdog. |
| **EXIT** | `EXIT;` | Einstellungsmodus (Settings) beenden. |
| **PCB** | `PCB;` | Gibt PCB-Version auf der seriellen Konsole zurück. |

#### Display-Formate (DSP1 / DSP2)

| Wert | Anzeige | Beschreibung |
|---|---|---|
| `-1` | **Display AUS** | Alle Segmente dunkel (keine Nullen, keine Striche). |
| `0` … `359` | `C-000` … `C-359` | Kursmodus — "C-" gefolgt von 3-stelligem Kurswert mit führenden Nullen. |
| Alles andere | z. B. `123.450` | Frequenzanzeige — der Wert wird als Text (max. 6 Zeichen) dargestellt. Ein `.` wird als Dezimalpunkt der vorherigen Stelle interpretiert. |

> **Beispiel:** `DSP1:129.550;DSP2:118.000;` zeigt links `129.55` und rechts `118.00`.

#### CFG-Befehl (Konfiguration)

```
CFG:ROT1010;DEB12;LED1200;DSP1200;REG:D-A320;
```

| Parameter | Bedeutung | Werte |
|---|---|---|
| `ROTxxxx` | Rotary-Sensitivity. xx = RT2-Konfig, xx = RT1-Konfig (je 2 Bit: 00/01/10/11) | `ROT1010` (Standard) |
| `DEBxx` | Button-Debounce in ms | 0–99 |
| `LEDxxxx` | LED-Refresh-Intervall in ms | 100–10000 |
| `DSPxxxx` | Display-Refresh-Intervall in ms | 100–10000 |
| `REG:xxxx` | A/C-Registrierung (max. 8 Zeichen) | z. B. `D-AIDA` |

---

### Serial Output (RMP → Sim)

Das Panel sendet bei jeder Änderung (Taster oder Drehgeber) sowie auf `REQ;`:

```
IN1:01000000;IN2:00000001;RT1:1;RT2:0;
```

| Feld | Beschreibung |
|---|---|
| **IN1:** | 8-Bit Binärstatus der Eingänge (HC165 Register 1). Bit 0 = XFER, Bit 1–7 = VHF1/2/3, HF1, SEL, AM, HF2. |
| **IN2:** | 8-Bit Binärstatus (HC165 Register 2). Bit 0–4 = NAV, VOR, ILS, MLS, ADF. |
| **RT1:** | Drehgeber 1 — Delta seit letzter Übertragung. Positiv = rechts, negativ = links. |
| **RT2:** | Drehgeber 2 — Delta. |

Beim Start oder auf `VER;`/`IDENT;` sendet das Panel zusätzlich:

```
IDENT:RMP, v1.2 MAQ;STATE:RUNNING;REG:D-A320;
```

---

## DIAG-Menü (Hardware-Test)

### Zugang

- **Serial:** `DIAG;` senden
- **Tastenkombi am Panel:** Gleichzeitig **NAV** (IN2 Bit 0) + **ILS** (IN2 Bit 2) + **ON/OFF** (IN2 Bit 5) drücken — entspricht `IN1:00000010;IN2:00010001`

### Menüführung

Navigation mit **Drehgeber 1** (RT1), Auswahl mit **XFER**-Taste.

| Menüpunkt | Funktion |
|---|---|
| **1. LED tESt** | Untermenü für LED/Display/Segment-Tests |
| → RUN ALL | Vollständiger Diagnosedurchlauf (~33 s) |
| → dISPly tEst | Ziffern 0–9 zählen mit wanderndem Dezimalpunkt |
| → LED wALK | Walking-Light über alle 16 LEDs |
| → SEG tESt | Einzelsegment-Test (alle 8 Segmente nacheinander) |
| → brIGht nESS | Helligkeits-Fade (LEDs + Display) |
| **2. butn tESt** | Taster-Test — gedrückte Taste wird auf Display angezeigt |
| → bUtton tESt | Direkter Tastertest mit LED-Rückmeldung |
| → ShIFt rEG | Live-Anzeige der Shift-Register-Werte (IN1/IN2) |
| **3. Info** | Zeigt PCB-Version und A/C-Registrierung |
| **4. Exit** | Zurück zum Normalbetrieb |

Im Tastertest: Doppelklick auf **XFER** zum Verlassen.

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **Displays:** 2× MAX7219 in Serie (je 6 Digits)
- **LED-Treiber:** 2× 74HC595 Schieberegister
- **Eingänge:** 2× 74HC165 Schieberegister (16 Taster)
- **Drehgeber:** 2× Rotary Encoder (Polling-Mode)
- **Direkte LEDs:** ILS SEL (Pin 14), MLS SEL (Pin 16)
- **Panel-ID:** `RMP, v1.2 MAQ` | Firmware: `1.2`

---

## EEPROM-Layout

| Adresse | Inhalt |
|---|---|
| 0–1 | Magic Number (`0xA55A`) |
| 2 | Format-Version (`2`) |
| 3–10 | A/C-Registrierung (8 Zeichen, Leerzeichen-gepadded) |
| 11–18 | PCB-Version (8 Zeichen) |
| 19 | Checksumme (Summe über Bytes 2–18, low byte) |

---

## Settings (Serial)

Geschützte Einstellungen über den Serial-Port (nur im laufenden Betrieb):

| Befehl | Wirkung |
|---|---|
| `SET ENA:0815;` | Einstellungsmodus aktivieren (PIN = 0815) |
| `SET FW:1.5;` | PCB-Version setzen (Format: Hauptversion.Nebenversion) |
| `SET ACID:D-AIDA;` | A/C-Registrierung setzen (Format: X-XXXX, max 8 Zeichen) |
| `SET WRITE;` / `SET WRI:YES;` | Konfiguration in EEPROM speichern + Neustart |
| `SET EXIT;` | Einstellungsmodus deaktivieren |


---

## English Version

# RMP – Radio Management Panel (Airbus A320 Style)

## Overview

The **SSF RMP** is a faithful Radio Management Panel in Airbus A320 design for Microsoft Flight Simulator (MSFS2020 / MSFS2024). It fully simulates the VHF/HF/NAV frequency management of a real RMP and communicates bidirectionally with the simulator via USB-Serial.

**Features:**
- 2× MAX7219 6-digit 7-segment displays (VHF/NAV frequencies, course)
- 16 backlit buttons with LED feedback
- 2 rotary encoders for frequency/course selection
- ILS/MLS indicator LEDs
- Integrated DIAG menu (hardware self-test)
- Offline detection with automatic notification
- EEPROM configuration (A/C registration, PCB version)
- Memory-efficient C implementation (no `String` type, no dynamic allocation)

---

## Serial Communication

**Baud rate:** 115200 baud, 8N1  
**Protocol:** Line-based, semicolon-separated (`;`), CR/LF accepted as delimiter.

### Serial Input (Sim → RMP)

| Command | Format | Description |
|---|---|---|
| **LED1** | `LED1:01010101;` | Sets the upper 8 LEDs (74HC595 register 1). 8-bit binary string. |
| **LED2** | `LED2:11001100;` | Sets the lower 8 LEDs (74HC595 register 2). 8-bit binary string. |
| **LED3** | `LED3:01;` | Sets ILS/MLS indicator LEDs. Bit 0 = ILS, Bit 1 = MLS. |
| **BL** | `BL:200;` | PWM brightness of button LEDs (0–255). |
| **DISP_BL** | `DISP_BL:10;` | Brightness of 7-segment displays (0–15). |
| **DSP1** | `DSP1:123.450;` | Sets the **left display** — see display formats below. |
| **DSP2** | `DSP2:118.000;` | Sets the **right display**. |
| **STATE** | `STATE:01;` | `01`/`1` = Host online, `00`/`0` = Host offline. |
| **CFG** | `CFG:ROT1010;DEB12;...` | Configuration (see CFG command below). |
| **REQ** | `REQ;` | Request immediate status (IN1/IN2/Rotary). |
| **VER / VERSION / IDENT** | `VER;` | Display firmware & PCB version on displays (5 s), then IDENT response. |
| **DIAG** | `DIAG;` | Enter DIAG menu (hardware test mode). |
| **RESET** | `RESET;` | Software reset via watchdog. |
| **EXIT** | `EXIT;` | Exit settings mode. |
| **PCB** | `PCB;` | Returns PCB version on the serial console. |

#### Display Formats (DSP1 / DSP2)

| Value | Display | Description |
|---|---|---|
| `-1` | **Display OFF** | All segments dark (no zeros, no dashes). |
| `0` … `359` | `C-000` … `C-359` | Course mode — "C-" followed by 3-digit course value with leading zeros. |
| Everything else | e.g. `123.450` | Frequency display — the value is shown as text (max. 6 characters). A `.` is interpreted as decimal point of the preceding digit. |

> **Example:** `DSP1:129.550;DSP2:118.000;` shows `129.55` on the left and `118.00` on the right.

#### CFG Command (Configuration)

```
CFG:ROT1010;DEB12;LED1200;DSP1200;REG:D-A320;
```

| Parameter | Meaning | Values |
|---|---|---|
| `ROTxxxx` | Rotary sensitivity. xx = RT2 config, xx = RT1 config (2 bits each: 00/01/10/11) | `ROT1010` (default) |
| `DEBxx` | Button debounce in ms | 0–99 |
| `LEDxxxx` | LED refresh interval in ms | 100–10000 |
| `DSPxxxx` | Display refresh interval in ms | 100–10000 |
| `REG:xxxx` | A/C registration (max. 8 characters) | e.g. `D-AIDA` |

---

### Serial Output (RMP → Sim)

The panel sends on every change (button or rotary encoder) as well as on `REQ;`:

```
IN1:01000000;IN2:00000001;RT1:1;RT2:0;
```

| Field | Description |
|---|---|
| **IN1:** | 8-bit binary status of inputs (HC165 register 1). Bit 0 = XFER, Bits 1–7 = VHF1/2/3, HF1, SEL, AM, HF2. |
| **IN2:** | 8-bit binary status (HC165 register 2). Bits 0–4 = NAV, VOR, ILS, MLS, ADF. |
| **RT1:** | Rotary encoder 1 — delta since last transmission. Positive = right, negative = left. |
| **RT2:** | Rotary encoder 2 — delta. |

On startup or on `VER;`/`IDENT;` the panel additionally sends:

```
IDENT:RMP, v1.2 MAQ;STATE:RUNNING;REG:D-A320;
```

---

## DIAG Menu (Hardware Test)

### Access

- **Serial:** Send `DIAG;`
- **Button combination on panel:** Press **NAV** (IN2 Bit 0) + **ILS** (IN2 Bit 2) + **ON/OFF** (IN2 Bit 5) simultaneously — corresponds to `IN1:00000010;IN2:00010001`

### Menu Navigation

Navigate with **Rotary Encoder 1** (RT1), select with **XFER** button.

| Menu Item | Function |
|---|---|
| **1. LED tESt** | Submenu for LED/display/segment tests |
| → RUN ALL | Complete diagnostic run (~33 s) |
| → dISPly tEst | Count digits 0–9 with wandering decimal point |
| → LED wALK | Walking light across all 16 LEDs |
| → SEG tESt | Single segment test (all 8 segments sequentially) |
| → brIGht nESS | Brightness fade (LEDs + display) |
| **2. butn tESt** | Button test — pressed button shown on display |
| → bUtton tESt | Direct button test with LED feedback |
| → ShIFt rEG | Live display of shift register values (IN1/IN2) |
| **3. Info** | Shows PCB version and A/C registration |
| **4. Exit** | Return to normal operation |

In button test: double-click **XFER** to exit.

---

## Hardware

- **Controller:** Arduino Pro Micro (ATmega32U4)
- **Displays:** 2× MAX7219 in series (6 digits each)
- **LED driver:** 2× 74HC595 shift registers
- **Inputs:** 2× 74HC165 shift registers (16 buttons)
- **Rotary encoders:** 2× rotary encoders (polling mode)
- **Direct LEDs:** ILS SEL (Pin 14), MLS SEL (Pin 16)
- **Panel ID:** `RMP, v1.2 MAQ` | Firmware: `1.2`

---

## EEPROM Layout

| Address | Content |
|---|---|
| 0–1 | Magic Number (`0xA55A`) |
| 2 | Format Version (`2`) |
| 3–10 | A/C Registration (8 characters, space-padded) |
| 11–18 | PCB Version (8 characters) |
| 19 | Checksum (sum over bytes 2–18, low byte) |

---

## Settings (Serial)

Protected settings via serial port (only during operation):

| Command | Effect |
|---|---|
| `SET ENA:0815;` | Activate settings mode (PIN = 0815) |
| `SET FW:1.5;` | Set PCB version (format: major.minor) |
| `SET ACID:D-AIDA;` | Set A/C registration (format: X-XXXX, max 8 characters) |
| `SET WRITE;` / `SET WRI:YES;` | Save configuration to EEPROM + restart |
| `SET EXIT;` | Deactivate settings mode |


---

## ฉบับภาษาไทย

# RMP – แผงจัดการวิทยุ (สไตล์ Airbus A320)

## ภาพรวม

**SSF RMP** เป็นแผงจัดการวิทยุที่จำลองเหมือนจริงในดีไซน์ Airbus A320 สำหรับ Microsoft Flight Simulator (MSFS2020 / MSFS2024) จำลองการจัดการความถี่ VHF/HF/NAV ของ RMP จริงอย่างสมบูรณ์ และสื่อสารสองทิศทางกับซิมูเลเตอร์ผ่าน USB-Serial

**คุณสมบัติ:**
- จอแสดงผล 7 ส่วน 6 หลัก MAX7219 จำนวน 2 ชุด (ความถี่ VHF/NAV, คอร์ส)
- ปุ่มกดมีไฟหลัง 16 ปุ่ม พร้อมการตอบสนองด้วย LED
- ตัวหมุน (Rotary Encoder) 2 ตัว สำหรับเลือกความถี่/คอร์ส
- ไฟแสดง ILS/MLS
- เมนู DIAG ในตัว (ทดสอบฮาร์ดแวร์ด้วยตนเอง)
- การตรวจจับออฟไลน์พร้อมการแจ้งเตือนอัตโนมัติ
- การกำหนดค่า EEPROM (ทะเบียน A/C, เวอร์ชัน PCB)
- การเขียนโปรแกรมภาษา C ที่ประหยัดหน่วยความจำ (ไม่มีชนิด `String`, ไม่มีการจัดสรรหน่วยความจำแบบไดนามิก)

---

## การสื่อสารผ่าน Serial

**อัตราบอด:** 115200 บอด, 8N1  
**โปรโตคอล:** แบบบรรทัด, คั่นด้วยเซมิโคลอน (`;`), ยอมรับ CR/LF เป็นตัวคั่น

### อินพุต Serial (Sim → RMP)

| คำสั่ง | รูปแบบ | คำอธิบาย |
|---|---|---|
| **LED1** | `LED1:01010101;` | ตั้งค่า LED 8 ตัวบน (74HC595 รีจิสเตอร์ 1) สตริงไบนารี 8 บิต |
| **LED2** | `LED2:11001100;` | ตั้งค่า LED 8 ตัวล่าง (74HC595 รีจิสเตอร์ 2) สตริงไบนารี 8 บิต |
| **LED3** | `LED3:01;` | ตั้งค่าไฟแสดง ILS/MLS บิต 0 = ILS, บิต 1 = MLS |
| **BL** | `BL:200;` | ความสว่าง PWM ของ LED ปุ่ม (0–255) |
| **DISP_BL** | `DISP_BL:10;` | ความสว่างของจอแสดงผล 7 ส่วน (0–15) |
| **DSP1** | `DSP1:123.450;` | ตั้งค่า**จอซ้าย** — ดูรูปแบบการแสดงผลด้านล่าง |
| **DSP2** | `DSP2:118.000;` | ตั้งค่า**จอขวา** |
| **STATE** | `STATE:01;` | `01`/`1` = โฮสต์ออนไลน์, `00`/`0` = โฮสต์ออฟไลน์ |
| **CFG** | `CFG:ROT1010;DEB12;...` | การกำหนดค่า (ดูคำสั่ง CFG ด้านล่าง) |
| **REQ** | `REQ;` | ขอสถานะทันที (IN1/IN2/Rotary) |
| **VER / VERSION / IDENT** | `VER;` | แสดงเวอร์ชันเฟิร์มแวร์และ PCB บนจอ (5 วินาที) จากนั้นตอบกลับ IDENT |
| **DIAG** | `DIAG;` | เข้าสู่เมนู DIAG (โหมดทดสอบฮาร์ดแวร์) |
| **RESET** | `RESET;` | รีเซ็ตซอฟต์แวร์ผ่าน Watchdog |
| **EXIT** | `EXIT;` | ออกจากโหมดการตั้งค่า |
| **PCB** | `PCB;` | ส่งคืนเวอร์ชัน PCB บนคอนโซล serial |

#### รูปแบบการแสดงผล (DSP1 / DSP2)

| ค่า | การแสดงผล | คำอธิบาย |
|---|---|---|
| `-1` | **ปิดจอ** | ทุกส่วนมืด (ไม่มีศูนย์, ไม่มีขีด) |
| `0` … `359` | `C-000` … `C-359` | โหมดคอร์ส — "C-" ตามด้วยค่าคอร์ส 3 หลักพร้อมศูนย์นำหน้า |
| อื่นๆ | เช่น `123.450` | การแสดงความถี่ — ค่าจะแสดงเป็นข้อความ (สูงสุด 6 ตัวอักษร) `.` จะถูกตีความเป็นจุดทศนิยมของหลักก่อนหน้า |

> **ตัวอย่าง:** `DSP1:129.550;DSP2:118.000;` แสดง `129.55` ทางซ้าย และ `118.00` ทางขวา

#### คำสั่ง CFG (การกำหนดค่า)

```
CFG:ROT1010;DEB12;LED1200;DSP1200;REG:D-A320;
```

| พารามิเตอร์ | ความหมาย | ค่า |
|---|---|---|
| `ROTxxxx` | ความไวของตัวหมุน xx = การตั้งค่า RT2, xx = การตั้งค่า RT1 (ชุดละ 2 บิต: 00/01/10/11) | `ROT1010` (ค่าเริ่มต้น) |
| `DEBxx` | ดีบาวซ์ปุ่มกดในหน่วย ms | 0–99 |
| `LEDxxxx` | ช่วงเวลารีเฟรช LED ในหน่วย ms | 100–10000 |
| `DSPxxxx` | ช่วงเวลารีเฟรชจอแสดงผลในหน่วย ms | 100–10000 |
| `REG:xxxx` | ทะเบียน A/C (สูงสุด 8 ตัวอักษร) | เช่น `D-AIDA` |

---

### เอาต์พุต Serial (RMP → Sim)

แผงจะส่งทุกครั้งที่มีการเปลี่ยนแปลง (ปุ่มกดหรือตัวหมุน) และเมื่อได้รับ `REQ;`:

```
IN1:01000000;IN2:00000001;RT1:1;RT2:0;
```

| ฟิลด์ | คำอธิบาย |
|---|---|
| **IN1:** | สถานะไบนารี 8 บิตของอินพุต (HC165 รีจิสเตอร์ 1) บิต 0 = XFER, บิต 1–7 = VHF1/2/3, HF1, SEL, AM, HF2 |
| **IN2:** | สถานะไบนารี 8 บิต (HC165 รีจิสเตอร์ 2) บิต 0–4 = NAV, VOR, ILS, MLS, ADF |
| **RT1:** | ตัวหมุน 1 — เดลต้าตั้งแต่การส่งครั้งล่าสุด บวก = ขวา, ลบ = ซ้าย |
| **RT2:** | ตัวหมุน 2 — เดลต้า |

เมื่อเริ่มต้นหรือเมื่อได้รับ `VER;`/`IDENT;` แผงจะส่งเพิ่มเติม:

```
IDENT:RMP, v1.2 MAQ;STATE:RUNNING;REG:D-A320;
```

---

## เมนู DIAG (ทดสอบฮาร์ดแวร์)

### การเข้าใช้งาน

- **Serial:** ส่ง `DIAG;`
- **การกดปุ่มบนแผง:** กด **NAV** (IN2 บิต 0) + **ILS** (IN2 บิต 2) + **ON/OFF** (IN2 บิต 5) พร้อมกัน — เทียบเท่า `IN1:00000010;IN2:00010001`

### การนำทางเมนู

นำทางด้วย**ตัวหมุน 1** (RT1), เลือกด้วยปุ่ม **XFER**

| รายการเมนู | ฟังก์ชัน |
|---|---|
| **1. LED tESt** | เมนูย่อยสำหรับทดสอบ LED/จอแสดงผล/เซกเมนต์ |
| → RUN ALL | การทดสอบวินิจฉัยเต็มรูปแบบ (~33 วินาที) |
| → dISPly tEst | นับเลข 0–9 พร้อมจุดทศนิยมที่เลื่อนไป |
| → LED wALK | ไฟวิ่งผ่าน LED ทั้ง 16 ดวง |
| → SEG tESt | ทดสอบแต่ละเซกเมนต์ (ทั้ง 8 เซกเมนต์ตามลำดับ) |
| → brIGht nESS | การปรับความสว่างแบบเฟด (LED + จอแสดงผล) |
| **2. butn tESt** | ทดสอบปุ่ม — ปุ่มที่กดจะแสดงบนจอ |
| → bUtton tESt | ทดสอบปุ่มโดยตรงพร้อมการตอบสนองด้วย LED |
| → ShIFt rEG | แสดงค่าชิฟต์รีจิสเตอร์แบบสด (IN1/IN2) |
| **3. Info** | แสดงเวอร์ชัน PCB และทะเบียน A/C |
| **4. Exit** | กลับสู่การทำงานปกติ |

ในการทดสอบปุ่ม: ดับเบิลคลิก **XFER** เพื่อออก

---

## ฮาร์ดแวร์

- **คอนโทรลเลอร์:** Arduino Pro Micro (ATmega32U4)
- **จอแสดงผล:** MAX7219 2 ชุดต่ออนุกรม (ชุดละ 6 หลัก)
- **ไดรเวอร์ LED:** 74HC595 ชิฟต์รีจิสเตอร์ 2 ชุด
- **อินพุต:** 74HC165 ชิฟต์รีจิสเตอร์ 2 ชุด (16 ปุ่ม)
- **ตัวหมุน:** Rotary Encoder 2 ตัว (โหมดโพลลิ่ง)
- **LED โดยตรง:** ILS SEL (ขา 14), MLS SEL (ขา 16)
- **รหัสแผง:** `RMP, v1.2 MAQ` | เฟิร์มแวร์: `1.2`

---

## ผัง EEPROM

| ที่อยู่ | เนื้อหา |
|---|---|
| 0–1 | Magic Number (`0xA55A`) |
| 2 | เวอร์ชันรูปแบบ (`2`) |
| 3–10 | ทะเบียน A/C (8 ตัวอักษร, เติมช่องว่าง) |
| 11–18 | เวอร์ชัน PCB (8 ตัวอักษร) |
| 19 | เช็คซัม (ผลรวมไบต์ 2–18, ไบต์ต่ำ) |

---

## การตั้งค่า (Serial)

การตั้งค่าที่มีการป้องกันผ่านพอร์ต serial (เฉพาะระหว่างการทำงาน):

| คำสั่ง | ผลลัพธ์ |
|---|---|
| `SET ENA:0815;` | เปิดใช้งานโหมดตั้งค่า (PIN = 0815) |
| `SET FW:1.5;` | ตั้งค่าเวอร์ชัน PCB (รูปแบบ: หลัก.รอง) |
| `SET ACID:D-AIDA;` | ตั้งค่าทะเบียน A/C (รูปแบบ: X-XXXX, สูงสุด 8 ตัวอักษร) |
| `SET WRITE;` / `SET WRI:YES;` | บันทึกการกำหนดค่าลง EEPROM + เริ่มใหม่ |
| `SET EXIT;` | ปิดใช้งานโหมดตั้งค่า |
