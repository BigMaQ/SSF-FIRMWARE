# OHP_Primary – Overhead Panel Primary (Arduino MEGA 2560)

## Übersicht

Das **SSF OHP Primary Panel** ist die primäre Steuereinheit des Airbus-A320-Overhead-Panels für Microsoft Flight Simulator (MSFS2020 / MSFS2024). Es steuert die HYD/ELEC-, FIRE-, EMER ELEC-, ELEC- und AC-Panel-Sektionen via USB-Serial.

**Features:**
- 11× 74HC165 Input-Schieberegister (HYD_ELEC, FIRE, EMELEC, ELEC, AC) → IN0…IN10
- 6 Einzel-Digital-Inputs (eng2_2_off, eng2_2_fault, apuagentdisch, apuagentsqd, ANNU_BRT, BACKLIGHT_BRT)
- 14× 74HC595 LED-Treiber (HYD_ELEC, FIRE, EMELEC, ELEC, AC) → LED0…LED13
- 2× MAX7219 7-Segment-Displays (BAT1 Volt, BAT2 Volt) → DSP0, DSP1
- 6 Analog-Eingänge (AC_FWD_Cabin, AC_AFT_Cabin, AC_Cockpit, FIRE_ENG1_AGENT1/2, FIRE_ENG1_TEST) → POT0…POT5
- Integriertes DIAG-Menü (Hardware-Selbsttest)
- EEPROM-Konfiguration (A/C-Registrierung, PCB-Version, Seriennummer)
- Speicherschonende C++-Implementierung (kein `String`-Typ, keine dynamische Allokation, kein Heap)

---

## Pinmap – Primary

### Input Shifter (74HC165)

| Gruppe | CLOCK | LATCH | DATA | Anzahl | IN-Bytes |
|---|---|---|---|---|---|
| HYD_ELEC_IS | D3 | D2 | D4 | 2 | IN0, IN1 |
| FIRE_IS | D19 | D18 | D14 | 1 | IN2 |
| EMELEC_IS | D25 | D24 | D26 | 4 | IN3, IN4, IN5, IN6 |
| ELEC_IS | D36 | D35 | D37 | 2 | IN7, IN8 |
| AC_IS | D44 | D41 | D43 | 2 | IN9, IN10 |

### Einzel-Inputs (→ IN11)

| Pin | Funktion |
|---|---|
| D9 | HYD_ELEC_eng2_2_off |
| D10 | HYD_ELEC_eng2_2_fault |
| D23 | FIRE_apuagentdisch |
| D20 | FIRE_apuagentsqd |
| D11 | ANNU_BRT (Annunciator Brightness) |
| D12 | BACKLIGHT_BRT |

### Output Shifter (74HC595)

| Gruppe | CLK | LATCH | DATA | Anzahl | LED-Bytes |
|---|---|---|---|---|---|
| HYD_ELEC_LEDDRV | D8 | D5 | D7 | 3 | LED0, LED1, LED2 |
| FIRE_LEDDRV | D16 | D21 | D17 | 2 | LED3, LED4 |
| EMELEC_LEDDRV | D28 | D27 | D29 | 4 | LED5, LED6, LED7, LED8 |
| ELEC_LEDDRV | D39 | D38 | D40 | 3 | LED9, LED10, LED11 |
| AC_LEDDRV | D45 | D46 | D48 | 2 | LED12, LED13 |

### Displays (MAX7219)

| Bus | CLK | DIN | LOAD | Devices | Funktion |
|---|---|---|---|---|---|
| ELEC_DISP | D33 | D32 | D34 | 2 | DSP0=BAT1 Volt, DSP1=BAT2 Volt |

### Analog-Eingänge

| Pin | POT-Index | Funktion |
|---|---|---|
| A9 | POT0 | AC_FWD_Cabin |
| A8 | POT1 | AC_AFT_Cabin |
| A10 | POT2 | AC_Cockpit |
| A13 | POT3 | FIRE_ENG1_AGENT1 |
| A14 | POT4 | FIRE_ENG1_AGENT2 |
| A15 | POT5 | FIRE_ENG1_TEST |

### RES-Pins (nicht verwendet)

A5, D… (reserviert, nicht in INx/LEDx/DSPx/POTx)

---

## Serielle Kommunikation

**Baudrate:** 115200 Baud, 8N1  
**Protokoll:** Zeilenbasiert, Semikolon-getrennt (`;`)

### Serial Input (Host → Panel)

| Befehl | Format | Beschreibung |
|---|---|---|
| **LED0…LED13** | `LED0:01010101;` | Setzt LED-Schieberegister Byte (je 8 Bit) |
| **STATE** | `STATE:01;` | `01`/`1` = Host online, `00`/`0` = offline |
| **DSP0 / DSP1** | `DSP0:285;` | Setzt Display-Wert (BAT1/BAT2 Volt ×10) |
| **DISP_BL** | `DISP_BL:10;` | Display-Helligkeit (0–15) |
| **CFG** | `CFG:DEB12;REG:D-A320;` | Konfiguration |
| **REQ** | `REQ;` | Status sofort anfordern |
| **VER / IDENT** | `VER;` | Firmware-Identifikation senden |
| **DIAG** | `DIAG;` | DIAG-Menü starten |
| **RESET** | `RESET;` | Software-Reset via Watchdog |
| **EXIT** | `EXIT;` | Settings-Modus beenden |

### Serial Output (Panel → Host)

Das Panel sendet bei Änderungen sowie auf `REQ;`:

```
IN0:01010101;IN1:11001100;...;IN11:00000110;POT0:512;POT1:488;...;POT5:256;
```

| Feld | Beschreibung |
|---|---|
| **IN0…IN10** | 8-Bit Binärstatus der Schieberegister-Eingänge |
| **IN11** | 8-Bit Binärstatus der Einzel-Inputs (Bit 0–5) |
| **POT0…POT5** | Gefilterte Analogwerte (0–1023) |

Beim Start oder auf `VER;`/`IDENT;`:

```
IDENT:OHP_PRI, v1.0 MAQ, SN:OHP-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

---

## DIAG-Menü

### Zugang
- **Serial:** `DIAG;` senden
- **Tastenkombi:** Bits 0+1+2 von IN0 gleichzeitig drücken

### Menüpunkte
1. **LED Test** – Walking-Light über alle LED-Kanäle
2. **Input Test** – Zeigt alle Eingangszustände über Serial
3. **Health Report** – Uptime, freier Speicher, Version
4. **Exit** – Zurück zum Normalbetrieb

### DIAG-Full-Sequence (automatisch)
1. Display-Count-Test (0–9 mit Dezimalpunkt-Wanderung)
2. Alle LEDs EIN + alle Segmente an
3. Input-Zustandsanzeige
4. LED-Walking-Pattern
5. Segment-Sweep
6. Helligkeits-Fade
7. DIAG PASS

---

## Quickstart Tests

### 1. Verbindung prüfen
```
→ VER;
← IDENT:OHP_PRI, v1.0 MAQ, SN:OHP-XXXXXXXX;STATE:RUNNING;REG:D-A320;
```

### 2. LED-Test
```
→ DIAG;
← DIAG:MENU_START;
```
Wählt automatisch LED-Walk. Erwartet: LEDs leuchten nacheinander auf.

### 3. Input-Test
Taster auf dem Panel betätigen. Erwartet:
```
← IN0:00000001;  (Bit 0 = 1 wenn Taster gedrückt)
```

### 4. Display-Test
```
→ DSP0:285;
→ DSP1:142;
```
Erwartet: Linkes Display zeigt `285`, rechtes `142`.

### 5. Analog-Test
Potentiometer drehen. Erwartet (Beispiel):
```
← POT0:512;POT1:488;...
```

### 6. Health Report
```
→ DIAG;
```
Im DIAG-Menü zweimal ENTER → Health Report:
```
← HEALTH:UPTIME=120s;LASTERR=0;FREEMEM=3456;VERSION=1.0;
```

---

## Build & Flash

- **Board:** Arduino MEGA 2560 (arduino:avr:mega:cpu=atmega2560)
- **Bibliotheken:** LedControl, EEPROM, Wire, SPI, avr/wdt
- **Flash via:** Arduino IDE oder avrdude

```bash
avrdude -c wiring -P COM3 -p m2560 -b 115200 -U flash:w:OHP_PRI.hex:i
```

---

## Projektstruktur

```
source/
├── OHP_PRI.INO          # Primary Sketch
├── OHP_SEC.INO          # Secondary Sketch
├── pins.h / pins.cpp    # Pin-Definitionen
├── protocol.h / .cpp    # Serielles Protokoll
├── diag.h / diag.cpp    # DIAG-Menü & Tests
├── inputs.h / inputs.cpp # Input-Schieberegister
├── leds.h / leds.cpp    # LED-Treiber
├── dsp.h / dsp.cpp      # MAX7219 Displays
├── pots.h / pots.cpp    # Analog-Eingänge
└── utils.h / utils.cpp  # Hilfsfunktionen
```
