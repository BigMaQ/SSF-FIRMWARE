# OHP_Secondary – Overhead Panel Secondary (Arduino MEGA 2560)

## Übersicht

Das **SSF OHP Secondary Panel** ist die sekundäre Steuereinheit des Airbus-A320-Overhead-Panels für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es steuert die ADIRS-, LT- (Lighting) und CARGO-Sektionen via USB-Serial.

**Features:**
- 9× 74HC165 Input-Schieberegister (ADIRS, LT, CARGO) → IN0…IN8
- 2 Einzel-Digital-Inputs (LT_ModeSel, CDV_SW) → IN9
- 17× 74HC595 LED-Treiber (BLTDRV, BLTDRV2, ADIRS, LT, CARGO) → LED0…LED16
- 2 Einzel-LEDs (GI1_CDV, GI2_CDV) → LED17
- 4 Analog-Eingänge (CARGO_HEAT_FWD, CARGO_HEAT_AFT, LT_OVP_LT, LT_LDG_ELEV) → POT0…POT3
- Integriertes DIAG-Menü (Hardware-Selbsttest)
- EEPROM-Konfiguration
- Keine MAX7219-Displays (RES_DISP ignoriert)

---

## Pinmap – Secondary

### Input Shifter (74HC165)

| Gruppe | CLOCK | LATCH | DATA | Anzahl | IN-Bytes |
|---|---|---|---|---|---|
| ADIRS_IS | D9 | D8 | D10 | 2 | IN0, IN1 |
| LT_IS | D16 | D15 | D17 | 4 | IN2, IN3, IN4, IN5 |
| CARGO_IS | D39 | D40 | D42 | 3 | IN6, IN7, IN8 |

_Hinweis: RES_IS (CLOCK D31, LATCH D30, DATA D32) ist reserviert und wird ignoriert._

### Einzel-Inputs (→ IN9)

| Pin | Funktion |
|---|---|
| D21 | LT_ModeSel |
| D22 | CDV_SW |

### Output Shifter (74HC595)

| Gruppe | CLK | LATCH | DATA | Anzahl | LED-Bytes |
|---|---|---|---|---|---|
| BLTDRV | D6 | D5 | D7 | 4 | LED0, LED1, LED2, LED3 |
| BLTDRV2 | D3 | D2 | D4 | 4 | LED4, LED5, LED6, LED7 |
| ADIRS_LEDDRV | D12 | D11 | D14 | 2 | LED8, LED9 |
| LT_LEDDRV | D19 | D18 | D20 | 3 | LED10, LED11, LED12 |
| CARGO_LEDDRV | D44 | D41 | D43 | 4 | LED13, LED14, LED15, LED16 |

_Hinweis: RES_LEDDRV (CLK D34, LATCH D33, DATA D35) ist reserviert und wird ignoriert._

### Einzel-LEDs (→ LED17)

| Pin | LED |
|---|---|
| D46 | LED_GI1_CDV |
| D48 | LED_GI2_CDV |

### Analog-Eingänge

| Pin | POT-Index | Funktion |
|---|---|---|
| A5 | POT0 | CARGO_HEAT_FWD |
| A6 | POT1 | CARGO_HEAT_AFT |
| A7 | POT2 | LT_OVP_LT |
| A8 | POT3 | LT_LDG_ELEV |

### RES-Pins (nicht verwendet)

A0–A4, RES_IS (D30/D31/D32), RES_LEDDRV (D33/D34/D35), RES_DISP (D36/D37/D38)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`)

### Serial Input (Host → Panel)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED0…LED17** | `LED0:01010101;` | Setzt LED-Schieberegister Byte (je 8 Bit) |
| **STATE** | `STATE:01;` | `01`/`1` = Host online, `00`/`0` = offline |
| **CFG** | `CFG:DEB12;REG:D-A320;` | Konfiguration |
| **REQ** | `REQ;` | Status sofort anfordern |
| **VER / IDENT** | `VER;` | Firmware-Identifikation senden |
| **DIAG** | `DIAG;` | DIAG-Menü starten |
| **RESET** | `RESET;` | Software-Reset via Watchdog |
| **EXIT** | `EXIT;` | Settings-Modus beenden |

_Hinweis: DSPx/DISP_BL werden akzeptiert, haben aber keine Wirkung (keine Displays)._

### Serial Output (Panel → Host)

```
IN0:01010101;...;IN9:00000011;POT0:512;POT1:488;POT2:256;POT3:768;
```

| Feld | Beschreibung |
|---|---|
| **IN0…IN8** | 8-Bit Binärstatus der Schieberegister-Eingänge |
| **IN9** | 8-Bit Binärstatus der Einzel-Inputs |
| **POT0…POT3** | Gefilterte Analogwerte (0–1023) |

Beim Start:
```
IDENT:OHP_SEC, v1.0 MAQ, SN:OHS-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

---

## DIAG-Menü

### Zugang
- **Serial:** `DIAG;` senden
- **Tastenkombi:** Bits 0+1+2 von IN0 gleichzeitig drücken

### Menüpunkte
1. **LED Test** – Walking-Light über alle LED-Kanäle (LED0…LED17)
2. **Input Test** – Alle Eingangszustände über Serial ausgeben
3. **Health Report** – Uptime, freier Speicher, Version
4. **Exit** – Zurück zum Normalbetrieb

---

## Quickstart Tests

### 1. Verbindung prüfen
```
→ VER;
← IDENT:OHP_SEC, v1.0 MAQ, SN:OHS-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

### 2. LED-Test
```
→ DIAG;
← DIAG:MENU_START;
```
Erwartet: LEDs leuchten nacheinander auf (Walking-Pattern).

### 3. Input-Test
Taster auf dem Panel betätigen:
```
← IN0:00000001;  (Bit 0 = 1 wenn Taster gedrückt)
```

### 4. Einzel-LED-Test
```
→ LED17:00000011;
```
Erwartet: GI1_CDV und GI2_CDV leuchten.

### 5. Analog-Test
Potentiometer drehen:
```
← POT0:512;POT1:488;POT2:256;POT3:768;
```

### 6. Health Report
Im DIAG-Menü 2× ENTER:
```
← HEALTH:UPTIME=120s;LASTERR=0;FREEMEM=3456;VERSION=1.0;
```

---

## Build & Flash

- **Board:** Arduino MEGA 2560 (arduino:avr:mega:cpu=atmega2560)
- **Bibliotheken:** LedControl, EEPROM, Wire, SPI, avr/wdt
- **Flash via:** Arduino IDE oder avrdude

```bash
avrdude -c wiring -P COM3 -p m2560 -b 115200 -U flash:w:OHP_SEC.hex:i
```

---

## Projektstruktur

```
source/
├── OHP_PRI.INO          # Primary Sketch
├── OHP_SEC.INO          # Secondary Sketch
├── pins.h / pins.cpp    # Pin-Definitionen (beide Panels)
├── protocol.h / .cpp    # Serielles Protokoll
├── diag.h / diag.cpp    # DIAG-Menü & Tests
├── inputs.h / inputs.cpp # Input-Schieberegister
├── leds.h / leds.cpp    # LED-Treiber
├── dsp.h / dsp.cpp      # MAX7219 Displays
├── pots.h / pots.cpp    # Analog-Eingänge
└── utils.h / utils.cpp  # Hilfsfunktionen
```
