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
