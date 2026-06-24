# SSF Firmware – Flash-Anleitung / Flashing Guide / คู่มือการแฟลช

## Bezugsquelle / Download Source

**GitHub Repository:** [https://github.com/BigMaQ/SSF-FIRMWARE](https://github.com/BigMaQ/SSF-FIRMWARE)

Im Repository findest du:
- **`source/`** – Alle `.INO`-Quellcode-Dateien zum selbst Kompilieren
- **`_build/`** – Vorkompilierte `.hex`-Dateien (via Releases oder nach eigenem Build)
- **`manual/`** – Panel-Manuals mit Serial-Protokoll-Referenz (DE/EN/TH)

---

## Panel-Übersicht / Panel Overview

| Panel | .INO Datei | Controller | .hex Datei |
|---|---|---|---|
| **RMP** | `source/RMP.ino` | Pro Micro (ATmega32U4) | `RMP.ino.hex` |
| **ACP** | `source/ACP.ino` | Pro Micro (ATmega32U4) | `ACP.ino.hex` |
| **ATC** | `source/ATC.ino` | Pro Micro (ATmega32U4) | `ATC.ino.hex` |
| **ECAM** | `source/ECAM/ECAM.ino` | Pro Micro (ATmega32U4) | `ECAM.ino.hex` |
| **MIP** | `source/MIP.ino` | Mega 2560 | `MIP.ino.hex` |
| **RUD** | `source/RUD.ino` | Pro Micro (ATmega32U4) | `RUD.ino.hex` |
| **TILLER** | `source/Tiller_CPT.ino` | Nano (ATmega328P) | `Tiller_CPT.ino.hex` |
| **HYDPRS** | `source/accu_press.ino` | Pro Micro (ATmega32U4) | `accu_press.ino.hex` |
| **CPT_DOOR** | `source/CPT_DOOR.ino` | Pro Micro (ATmega32U4) | `CPT_DOOR.ino.hex` |

---

## Methode 1: Arduino IDE (Empfohlen für Entwicklung)

### Voraussetzungen
- [Arduino IDE](https://www.arduino.cc/en/software) installiert
- USB-Treiber für den jeweiligen Controller (Pro Micro: SparkFun Treiber, Mega 2560: Standard)

### Schritt-für-Schritt

1. **Repository klonen oder .INO herunterladen:**
   ```
   git clone https://github.com/BigMaQ/SSF-FIRMWARE.git
   ```
   Oder einzelne `.INO`-Datei von GitHub herunterladen.

2. **Arduino IDE öffnen** und die `.INO`-Datei laden:
   - `Datei → Öffnen` → gewünschte `.INO`-Datei auswählen

3. **Board + Port konfigurieren:**
   - **Pro Micro:** `Werkzeuge → Board → Arduino AVR Boards → Arduino Micro`
   - **Mega 2560:** `Werkzeuge → Board → Arduino AVR Boards → Arduino Mega or Mega 2560`
   - **Nano:** `Werkzeuge → Board → Arduino AVR Boards → Arduino Nano`
   - `Werkzeuge → Port` → COM-Port des Panels auswählen

4. **Bibliotheken installieren** (falls nicht vorhanden):
   - `Werkzeuge → Bibliotheken verwalten...`
   - Benötigte Libs: **EEPROM** (integriert), **Servo** (integriert), **LedControl** (für MAX7219-Displays)

5. **Kompilieren + Hochladen:**
   - `Sketch → Hochladen` (oder `Strg+U`)
   - Bei Pro Micro: Reset-Taste doppelt drücken, wenn der Upload im "Searching for upload port..." hängt (Bootloader-Modus)

### Pro Micro Besonderheit
Der Pro Micro hat einen 8-Sekunden-Bootloader. Wenn die IDE `PORTS {} / { }` zeigt:
1. Reset-Taste **zweimal schnell** drücken
2. Der COM-Port wechselt kurz in den Bootloader-Modus
3. IDE erkennt den Port und flasht automatisch

---

## Methode 2: FCC (Firmware Config Center) – Empfohlen

Der **FCC** ist ein eigenständiges Programm zur Firmware-Verwaltung. Es wird als **FCC-Installer.exe** bereitgestellt und befindet sich im Projekt unter `cockpit_frontend\FCC\`.

### Installation & Start

1. **FCC-Installer.exe** ausführen und installieren
2. Programm starten – es erkennt automatisch alle angeschlossenen Panels (via IDENT)
3. Im **Firmware-Tab** erscheinen alle Panels mit aktueller Version und verfügbaren Updates

### Firmware-Update via FCC

1. Panel im FCC auswählen
2. FCC zeigt die installierte Version und verfügbare Updates aus dem GitHub-Repository an
3. Auf **"Update"** klicken – FCC flasht das Panel automatisch (avrdude im Hintergrund)
4. Fortschrittsbalken zeigt den Flash-Status an
5. Panel startet nach erfolgreichem Flash automatisch neu

> **Hinweis:** Der FCC lädt Firmware-Dateien automatisch aus dem GitHub-Repository (`https://github.com/BigMaQ/SSF-FIRMWARE`). Die UPDATE-SOURCE kann in `hardware.ini` konfiguriert werden.

---

## Methode 3: avrdude Kommandozeile (Fortgeschritten)

Falls FCC nicht verfügbar ist, kann `avrdude` (im Arduino-Paket enthalten) direkt via Kommandozeile verwendet werden.

### Voraussetzungen
- Arduino IDE installiert (liefert `avrdude.exe` mit)
- Panel via USB verbunden, COM-Port bekannt

### avrdude-Pfade (Standard-Installation)

```
AVRDUDE = %LOCALAPPDATA%\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe
CONFIG  = %LOCALAPPDATA%\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf
```

### Flash-Befehl (PowerShell)

**Pro Micro (ATmega32U4) – alle Panels außer MIP & TILLER:**
```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe" `
  -C "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf" `
  -v -p atmega32u4 -c avr109 -P COM3 -b 57600 `
  -D -U flash:w:"ECAM.ino.hex":i
```

**Mega 2560:**
```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe" `
  -C "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf" `
  -v -p atmega2560 -c wiring -P COM3 -b 115200 `
  -D -U flash:w:"MIP.ino.hex":i
```

**Nano (ATmega328P) – TILLER:**
```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe" `
  -C "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf" `
  -v -p atmega328p -c arduino -P COM3 -b 115200 `
  -D -U flash:w:"Tiller_CPT.ino.hex":i
```

> **Wichtig:** `COM3` durch den tatsächlichen COM-Port des Panels ersetzen! Im Geräte-Manager oder mit `Get-ComPortInfo` in PowerShell ermitteln.

### COM-Port ermitteln

**PowerShell:**
```powershell
Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Description
```

**Geräte-Manager:** `Systemsteuerung → Geräte-Manager → Anschlüsse (COM & LPT)`

### Pro Micro Bootloader-Trigger

Der Pro Micro muss für den Flash-Vorgang im Bootloader-Modus sein:
1. COM-Port notieren (z. B. `COM5`)
2. Reset-Taste am Panel **zweimal schnell** drücken
3. Der COM-Port ändert sich kurz (z. B. `COM5` → `COM6` oder bleibt gleich, je nach Treiber)
4. **Innerhalb von 8 Sekunden** den avrdude-Befehl mit dem aktuellen COM-Port ausführen
5. Nach erfolgreichem Flash startet das Panel automatisch neu

---

---

## English Version

# SSF Firmware – Flashing Guide

## Download Source

**GitHub Repository:** [https://github.com/BigMaQ/SSF-FIRMWARE](https://github.com/BigMaQ/SSF-FIRMWARE)

The repository contains:
- **`source/`** – All `.INO` source files for self-compilation
- **`_build/`** – Pre-compiled `.hex` files (via Releases or after your own build)
- **`manual/`** – Panel manuals with serial protocol reference (DE/EN/TH)

---

## Method 1: Arduino IDE (Recommended for Development)

### Prerequisites
- [Arduino IDE](https://www.arduino.cc/en/software) installed
- USB drivers for your controller (Pro Micro: SparkFun driver, Mega 2560: standard)

### Step by Step

1. **Clone the repository or download the .INO:**
   ```
   git clone https://github.com/BigMaQ/SSF-FIRMWARE.git
   ```
   Or download individual `.INO` files from GitHub.

2. **Open Arduino IDE** and load the `.INO` file:
   - `File → Open` → select the desired `.INO` file

3. **Configure Board + Port:**
   - **Pro Micro:** `Tools → Board → Arduino AVR Boards → Arduino Micro`
   - **Mega 2560:** `Tools → Board → Arduino AVR Boards → Arduino Mega or Mega 2560`
   - **Nano:** `Tools → Board → Arduino AVR Boards → Arduino Nano`
   - `Tools → Port` → select the panel's COM port

4. **Install libraries** (if missing):
   - `Tools → Manage Libraries...`
   - Required: **EEPROM** (built-in), **Servo** (built-in), **LedControl** (for MAX7219 displays)

5. **Compile + Upload:**
   - `Sketch → Upload` (or `Ctrl+U`)
   - Pro Micro: double-press the reset button if upload hangs at "Searching for upload port..." (bootloader mode)

### Pro Micro Note
The Pro Micro has an 8-second bootloader. If the IDE shows `PORTS {} / { }`:
1. Press the reset button **twice quickly**
2. The COM port briefly switches to bootloader mode
3. The IDE detects the port and flashes automatically

---

## Method 2: FCC (Firmware Config Center) – Recommended

The **FCC** is a standalone firmware management tool. It is provided as **FCC-Installer.exe** and located in the project under `cockpit_frontend\FCC\`.

### Installation & Start

1. Run **FCC-Installer.exe** and install
2. Launch the program – it automatically detects all connected panels (via IDENT)
3. In the **Firmware tab**, all panels appear with their current version and available updates

### Firmware Update via FCC

1. Select a panel in FCC
2. FCC shows the installed version and available updates from the GitHub repository
3. Click **"Update"** – FCC flashes the panel automatically (avrdude in background)
4. Progress bar shows the flash status
5. Panel restarts automatically after successful flash

> **Note:** FCC automatically downloads firmware files from the GitHub repository (`https://github.com/BigMaQ/SSF-FIRMWARE`). The UPDATE-SOURCE can be configured in `hardware.ini`.

---

## Method 3: avrdude Command Line (Advanced)

If FCC is not available, `avrdude` (included with Arduino IDE) can be used directly via command line.

### Prerequisites
- Arduino IDE installed (provides `avrdude.exe`)
- Panel connected via USB, COM port known

### avrdude Paths (default installation)

```
AVRDUDE = %LOCALAPPDATA%\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe
CONFIG  = %LOCALAPPDATA%\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf
```

### Flash Commands (PowerShell)

**Pro Micro (ATmega32U4) – all panels except MIP & TILLER:**
```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe" `
  -C "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf" `
  -v -p atmega32u4 -c avr109 -P COM3 -b 57600 `
  -D -U flash:w:"ECAM.ino.hex":i
```

**Mega 2560:**
```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe" `
  -C "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf" `
  -v -p atmega2560 -c wiring -P COM3 -b 115200 `
  -D -U flash:w:"MIP.ino.hex":i
```

**Nano (ATmega328P) – TILLER:**
```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe" `
  -C "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf" `
  -v -p atmega328p -c arduino -P COM3 -b 115200 `
  -D -U flash:w:"Tiller_CPT.ino.hex":i
```

> **Important:** Replace `COM3` with the actual COM port of your panel!

### Finding the COM Port

**PowerShell:**
```powershell
Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Description
```

**Device Manager:** `Control Panel → Device Manager → Ports (COM & LPT)`

### Pro Micro Bootloader Trigger

The Pro Micro must be in bootloader mode for flashing:
1. Note the COM port (e.g. `COM5`)
2. Press the reset button on the panel **twice quickly**
3. The COM port changes briefly (e.g. `COM5` → `COM6` or stays same, depending on driver)
4. **Within 8 seconds**, run the avrdude command with the current COM port
5. After successful flash, the panel restarts automatically

---

## Troubleshooting / Fehlerbehebung

| Problem | Lösung / Solution |
|---|---|
| **avrdude: butterfly_recv(): programmer is not responding** | Pro Micro: Reset-Taste zweimal drücken und Befehl sofort wiederholen. Mega: Baudrate prüfen (115200). |
| **avrdude: stk500_getsync(): not in sync** | Falscher Board-Typ oder falscher COM-Port. Board-Einstellungen prüfen. |
| **Error: opening port: Access is denied** | COM-Port wird von anderer Software (FCC, Serial Monitor) belegt. Schließen und erneut versuchen. |
| **Arduino IDE: PORTS {} / { }** | Pro Micro Bootloader-Modus: Reset zweimal drücken, dann sofort Upload starten. |
| **Panel antwortet nach Flash nicht** | Serial Monitor öffnen (115200 Baud), `VER;` senden. Bei `IDENT:...`-Antwort ist alles OK. |
| **.hex-Datei nicht gefunden** | In `_build/<PANEL>/` nachsehen oder selbst mit Arduino IDE kompilieren. |

---

## ฉบับภาษาไทย

# SSF Firmware – คู่มือการแฟลช

## แหล่งดาวน์โหลด

**GitHub Repository:** [https://github.com/BigMaQ/SSF-FIRMWARE](https://github.com/BigMaQ/SSF-FIRMWARE)

ใน Repository มี:
- **`source/`** – ไฟล์ `.INO` ทั้งหมดสำหรับคอมไพล์เอง
- **`_build/`** – ไฟล์ `.hex` ที่คอมไพล์แล้ว
- **`manual/`** – คู่มือแผงพร้อมข้อมูล Serial Protocol (DE/EN/TH)

---

## วิธีที่ 1: Arduino IDE (แนะนำสำหรับการพัฒนา)

### ขั้นตอน

1. **โคลน Repository หรือดาวน์โหลด .INO:**
   ```
   git clone https://github.com/BigMaQ/SSF-FIRMWARE.git
   ```

2. **เปิด Arduino IDE** และโหลดไฟล์ `.INO`:
   - `File → Open` → เลือกไฟล์ `.INO` ที่ต้องการ

3. **ตั้งค่า Board + Port:**
   - **Pro Micro:** `Tools → Board → Arduino AVR Boards → Arduino Micro`
   - **Mega 2560:** `Tools → Board → Arduino AVR Boards → Arduino Mega or Mega 2560`
   - **Nano:** `Tools → Board → Arduino AVR Boards → Arduino Nano`
   - `Tools → Port` → เลือก COM port ของแผง

4. **คอมไพล์ + อัปโหลด:**
   - `Sketch → Upload` (หรือ `Ctrl+U`)
   - Pro Micro: กดปุ่มรีเซ็ตสองครั้งถ้าอัปโหลดค้าง

---

## วิธีที่ 2: FCC (Firmware Config Center) – แนะนำ

**FCC** เป็นโปรแกรมจัดการเฟิร์มแวร์แบบสแตนด์อโลน มีให้ในรูปแบบ **FCC-Installer.exe**

### การติดตั้งและเริ่มต้น

1. รัน **FCC-Installer.exe** และติดตั้ง
2. เปิดโปรแกรม – ตรวจจับแผงที่เชื่อมต่อทั้งหมดโดยอัตโนมัติ (ผ่าน IDENT)
3. ใน**แท็บ Firmware** แผงทั้งหมดจะแสดงพร้อมเวอร์ชันปัจจุบันและอัปเดตที่มี

### อัปเดตเฟิร์มแวร์ผ่าน FCC

1. เลือกแผงใน FCC
2. FCC แสดงเวอร์ชันที่ติดตั้งและอัปเดตที่มีจาก GitHub
3. คลิก **"Update"** – FCC แฟลชแผงโดยอัตโนมัติ (avrdude ในพื้นหลัง)
4. แถบความคืบหน้าแสดงสถานะการแฟลช
5. แผงเริ่มต้นใหม่โดยอัตโนมัติหลังแฟลชสำเร็จ

---

## วิธีที่ 3: บรรทัดคำสั่ง avrdude (ขั้นสูง)

ถ้า FCC ไม่พร้อมใช้งาน สามารถใช้ `avrdude` โดยตรงผ่านบรรทัดคำสั่ง

### คำสั่งแฟลช (PowerShell)

**Pro Micro (ATmega32U4) – ทุกแผงยกเว้น MIP และ TILLER:**
```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe" `
  -C "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf" `
  -v -p atmega32u4 -c avr109 -P COM3 -b 57600 `
  -D -U flash:w:"ECAM.ino.hex":i
```

**Mega 2560:**
```powershell
... -p atmega2560 -c wiring -P COM3 -b 115200 -D -U flash:w:"MIP.ino.hex":i
```

**Nano (ATmega328P) – TILLER:**
```powershell
... -p atmega328p -c arduino -P COM3 -b 115200 -D -U flash:w:"Tiller_CPT.ino.hex":i
```

> เปลี่ยน `COM3` เป็น COM port จริงของแผง!

### การค้นหา COM Port

**PowerShell:**
```powershell
Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Description
```

### Pro Micro Bootloader

กดปุ่มรีเซ็ต**สองครั้งเร็วๆ** เพื่อเข้าโหมด Bootloader (มีเวลา 8 วินาที) แล้วรันคำสั่ง avrdude

---

## การแก้ไขปัญหา

| ปัญหา | วิธีแก้ |
|---|---|
| **avrdude: butterfly_recv(): programmer is not responding** | Pro Micro: กดรีเซ็ตสองครั้งแล้วลองใหม่ |
| **Error: opening port: Access is denied** | COM port ถูกใช้โดยโปรแกรมอื่น ปิดโปรแกรมนั้นก่อน |
| **แผงไม่ตอบสนองหลังแฟลช** | เปิด Serial Monitor (115200) ส่ง `VER;` ควรตอบกลับ `IDENT:...` |
