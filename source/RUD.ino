// RUD Control Script, (w) 2025 M. Quatember
// Derived from RMP firmware: same serial protocol and layout semantics

#include <LedControl.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================================
// PIN DEFINITIONS (RUD hardware)
// ============================================================================
// MAX7219 (single 4-digit device)
const int maxDataPin  = 6;  // DIN
const int maxClockPin = 8;  // CLK
const int maxCsPin    = 7;  // LOAD/CS

// LED sink drivers (4x 74HC595 daisy chain)
const int ledLatchPin = 3;
const int ledClockPin = 4;
const int ledDataPin  = 5;

// Input shift register (HC165)
const int inputClockPin = 14; // SRCLK
const int inputLatchPin = 15; // RCLK
const int inputDataPin  = 16; // SER

// Brightness control
const int backlightPwmPin = 9;
const int annunPwmPin     = 10;

// Analog input
const int potPin = A3;

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "RUD, v1.0 MAQ";
bool identSent = false;

// ============================================================================
// DIAG MODE STATE
// ============================================================================
enum DiagMode { DIAG_IDLE, DIAG_RUNNING };
DiagMode diagMode = DIAG_IDLE;
unsigned long diagStart = 0;

// ============================================================================
// CONFIG & EEPROM (analog to RMP)
// ============================================================================
String CFG_AIRCRAFT_REG = "D-A320";   // up to 8 chars
String CFG_PCB_VERSION = "PCB 1.0";   // up to 8 chars
bool settingsEnabled = false;
const String SETTINGS_PIN = "0815";

const int EEPROM_BASE_ADDR = 0;
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 2; // layout: magic(2), ver(1), reg(8), pcb(8), crc(1)

uint8_t calcCfgChecksum(uint8_t version, const char *reg8, const char *pcb8) {
  uint16_t sum = version;
  for (int i = 0; i < 8; i++) sum += (uint8_t)reg8[i];
  for (int i = 0; i < 8; i++) sum += (uint8_t)pcb8[i];
  return (uint8_t)(sum & 0xFF);
}

// Fast software reset via watchdog
void triggerSoftwareReset() {
  Serial.flush();
  delay(50);
  wdt_enable(WDTO_15MS);
  while (true) {}
}

void loadHWInfo() {
  uint16_t magic = (uint16_t)EEPROM.read(EEPROM_BASE_ADDR) | ((uint16_t)EEPROM.read(EEPROM_BASE_ADDR+1) << 8);
  if (magic != EEPROM_MAGIC) return;
  uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR+2);
  if (ver != EEPROM_FORMAT_VERSION) return;
  char regbuf[9];
  for (int i = 0; i < 8; i++) regbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+3+i);
  regbuf[8] = 0;
  char pcbbuf[9];
  for (int i = 0; i < 8; i++) pcbbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+11+i);
  pcbbuf[8] = 0;
  uint8_t stored = EEPROM.read(EEPROM_BASE_ADDR+19);
  if (stored != calcCfgChecksum(ver, regbuf, pcbbuf)) return;
  String regStr = "";
  for (int i = 0; i < 8; i++) if (regbuf[i] != 0 && regbuf[i] != ' ') regStr += regbuf[i];
  if (regStr.length() > 0) CFG_AIRCRAFT_REG = regStr;
  String pcbStr = "";
  for (int i = 0; i < 8; i++) pcbStr += (pcbbuf[i] ? pcbbuf[i] : ' ');
  while (pcbStr.length() > 0 && pcbStr.charAt(pcbStr.length()-1) == ' ') pcbStr.remove(pcbStr.length()-1);
  if (pcbStr.length() > 0) CFG_PCB_VERSION = pcbStr;
}

void saveHWInfo() {
  char regbuf[8];
  for (int i = 0; i < 8; i++) regbuf[i] = (i < CFG_AIRCRAFT_REG.length()) ? CFG_AIRCRAFT_REG.charAt(i) : ' ';
  char pcbbuf[8];
  for (int i = 0; i < 8; i++) pcbbuf[i] = (i < CFG_PCB_VERSION.length()) ? CFG_PCB_VERSION.charAt(i) : ' ';
  uint16_t magic = EEPROM_MAGIC;
  EEPROM.update(EEPROM_BASE_ADDR + 0, (uint8_t)(magic & 0xFF));
  EEPROM.update(EEPROM_BASE_ADDR + 1, (uint8_t)((magic >> 8) & 0xFF));
  EEPROM.update(EEPROM_BASE_ADDR + 2, EEPROM_FORMAT_VERSION);
  for (int i = 0; i < 8; i++) EEPROM.update(EEPROM_BASE_ADDR + 3 + i, (uint8_t)regbuf[i]);
  for (int i = 0; i < 8; i++) EEPROM.update(EEPROM_BASE_ADDR + 11 + i, (uint8_t)pcbbuf[i]);
  uint8_t cs = calcCfgChecksum(EEPROM_FORMAT_VERSION, regbuf, pcbbuf);
  EEPROM.update(EEPROM_BASE_ADDR + 19, cs);
  Serial.print("CFG:SAVED:REG="); Serial.print(CFG_AIRCRAFT_REG);
  Serial.print(";PCB="); Serial.println(CFG_PCB_VERSION);
}

// ============================================================================
// MAX7219 DISPLAY
// ============================================================================
LedControl lc = LedControl(maxDataPin, maxClockPin, maxCsPin, 1);
String displayMain = "    ";
uint8_t displayBrightness = 8; // 0-15

byte charTo7Seg(char c) {
  if (c == '$') return 0b00010101; // custom m
  if (c == 'o') return 0b01011100;
  if (c == 'i') return 0b00010000;
  if (c == 'm') return 0b00010101;
  if (c == 'y') return 0b00111011;
  if (c == 'u') return 0b00011100;  // lowercase u like v (c,d,e)
  if (c == 'r') return 0b00000101;
  c = toupper(c);
  switch (c) {
    case '0': return 0b01111110;
    case '1': return 0b00110000;
    case '2': return 0b01101101;
    case '3': return 0b01111001;
    case '4': return 0b00110011;
    case '5': return 0b01011011;
    case '6': return 0b01011111;
    case '7': return 0b01110000;
    case '8': return 0b01111111;
    case '9': return 0b01111011;
    case 'A': return 0b01110111;
    case 'B': return 0b00011111;
    case 'C': return 0b01001110;
    case 'D': return 0b00111101;
    case 'E': return 0b01001111;
    case 'F': return 0b01000111;
    case 'G': return 0b01011110;
    case 'H': return 0b00110111;
    case 'I': return 0b00110000;
    case 'J': return 0b00111100;
    case 'L': return 0b00001110;
    case 'M': return 0b00010101;
    case 'N': return 0b00010101;
    case 'O': return 0b01111110;
    case 'P': return 0b01100111;
    case 'R': return 0b01110111;  // R: same as A
    case 'S': return 0b01011011;
    case 'T': return 0b00001111;
    case 'U': return 0b00111110;
    case 'X': return 0b00110111;
    case 'Y': return 0b00111011;
    case '-': return 0b00000001;  // Only segment G (middle horizontal)
    case ' ': return 0;
    default:  return 0;
  }
}

void updateDisplay(const String &txt) {
  // Render up to 4 visible characters; '.' after a character sets its decimal point
  // The '.' itself does not consume a display position.
  String t = txt;
  int src = 0;
  for (int digit = 0; digit < 4; digit++) {
    // Skip any leading dots (no preceding char to attach to)
    while (src < t.length() && t.charAt(src) == '.') src++;

    char c = ' ';
    bool dp = false;
    if (src < t.length()) {
      c = t.charAt(src);
      // If next char is a dot, attach DP to current char without consuming a position
      if ((src + 1) < t.length() && t.charAt(src + 1) == '.') {
        dp = true;
      }
      // Advance past the visible character and any single trailing dot
      src++;
      if (src < t.length() && t.charAt(src) == '.') src++;
    }

    byte pattern = charTo7Seg(c);
    if (dp) pattern |= 0b10000000;  // DP bit
    lc.setRow(0, 3 - digit, pattern);
  }
  // Store last requested text (not strictly used by renderer)
  displayMain = txt;
}

void setDisplayBrightness(uint8_t b) {
  displayBrightness = constrain(b, 0, 15);
  lc.setIntensity(0, displayBrightness);
}

void initDisplay() {
  lc.shutdown(0, false);
  lc.setIntensity(0, displayBrightness);
  lc.clearDisplay(0);
}

// ============================================================================
// LED STATE (32 bits, 4 drivers)
// ============================================================================
uint32_t desiredLedState = 0;
uint32_t hwLedState = 0;
uint8_t desiredBacklight = 0;
uint8_t desiredAnnun = 0;

void shiftOutLEDs(uint32_t bits) {
  digitalWrite(ledLatchPin, LOW);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, (bits >> 24) & 0xFF);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, (bits >> 16) & 0xFF);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, (bits >> 8) & 0xFF);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, bits & 0xFF);
  digitalWrite(ledLatchPin, HIGH);
}

void applyLEDOutputs() {
  shiftOutLEDs(desiredLedState);
  analogWrite(backlightPwmPin, desiredBacklight);
  analogWrite(annunPwmPin, desiredAnnun);
  hwLedState = desiredLedState;
}

void setLEDState(uint32_t bits) {
  desiredLedState = bits;
  applyLEDOutputs();
}

// ============================================================================
// INPUT STATE (two cascaded HC165)
// ============================================================================
uint8_t inputState1 = 0;
uint8_t inputState2 = 0;
uint8_t lastInputState1 = 0;
uint8_t lastInputState2 = 0;
unsigned long lastDebounceTs = 0;
const unsigned long DEBOUNCE_MS = 12;

void readShiftRegisters(uint8_t &in1, uint8_t &in2) {
  digitalWrite(inputLatchPin, LOW);
  delayMicroseconds(5);
  digitalWrite(inputLatchPin, HIGH);
  delayMicroseconds(5);

  byte raw1 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);
  byte raw2 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);
  in1 = ~raw1;
  in2 = ~raw2;

  digitalWrite(inputLatchPin, LOW);
}

// ============================================================================
// HELPER
// ============================================================================
String bin8(byte b) {
  String s = "";
  for (int i = 7; i >= 0; i--) s += (b & (1 << i)) ? '1' : '0';
  return s;
}

uint8_t parseBin8(const String &s) {
  uint8_t v = 0;
  for (int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c == '1') v = (v << 1) | 1;
    else if (c == '0') v = (v << 1);
  }
  return v;
}

// ============================================================================
// SERIAL HANDLING
// ============================================================================
String serialAccum = "";
unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
int potValue = 0;
int lastPotValue = 0;

void sendIdentAndState() {
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
  Serial.print(";STATE:RUNNING;REG:"); Serial.print(CFG_AIRCRAFT_REG); Serial.print(";");
  Serial.println();
}

void sendStatus(bool force = false) {
  unsigned long now = millis();
  if (!force && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;

  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";");
  Serial.print("IN2:"); Serial.print(bin8(inputState2)); Serial.print(";");
  Serial.print("RT1:0;RT2:0;");
  Serial.print("POT:"); Serial.print(potValue); Serial.print(";");
  Serial.println();
  lastSendTs = now;
}

void handleLEDToken(int idx, const String &val) {
  uint8_t byteVal = parseBin8(val);
  // LED1-LED4 map to bytes in MSB-first chain:
  // LED1 -> bits 24-31, LED2 -> bits 16-23, LED3 -> bits 8-15, LED4 -> bits 0-7
  int shift = (4 - idx) * 8; // idx 1..4 -> shift 24,16,8,0
  desiredLedState &= ~(0xFFUL << shift);
  desiredLedState |= ((uint32_t)byteVal << shift);
  applyLEDOutputs();
}

void processIncomingLine(const String &line) {
  int start = 0;
  while (true) {
    int sep = line.indexOf(';', start);
    if (sep < 0) break;
    String token = line.substring(start, sep);
    start = sep + 1;
    token.trim();
    if (token.length() == 0) continue;

    if (token.indexOf(':') < 0) {
      String up = token; up.toUpperCase();
      if (up == "VER" || up == "VERSION") { sendIdentAndState(); continue; }
      if (up == "REQ") { sendStatus(true); continue; }
      if (up == "DIAG") { diagMode = DIAG_RUNNING; diagStart = millis(); continue; }
      if (up == "EXIT") { settingsEnabled = false; Serial.println("EXIT:OK ;"); continue; }
      if (up == "PCB") {
        Serial.print("PCB:");
        String pcbVer = CFG_PCB_VERSION; pcbVer.trim();
        int spacePos = pcbVer.indexOf(' ');
        if (spacePos >= 0 && spacePos < pcbVer.length() - 1) {
          String digits = pcbVer.substring(spacePos + 1);
          Serial.print("v"); Serial.print(digits);
        } else {
          Serial.print(pcbVer);
        }
        Serial.println(" ;");
        continue;
      }
      continue;
    }

    int cpos = token.indexOf(':');
    String key = token.substring(0, cpos); key.trim(); key.toUpperCase();
    String val = token.substring(cpos + 1); val.trim();

    if (key == "LED1") handleLEDToken(1, val);
    else if (key == "LED2") handleLEDToken(2, val);
    else if (key == "LED3") handleLEDToken(3, val);
    else if (key == "LED4") handleLEDToken(4, val);
    else if (key == "BL") {
      int b = constrain(val.toInt(), 0, 255);
      desiredBacklight = b;
      desiredAnnun = b;
      applyLEDOutputs();
    }
    else if (key == "DISP_BL") {
      setDisplayBrightness(val.toInt());
    }
    else if (key == "DSP1") {
      updateDisplay(val);
    }
    else if (key == "STATE") {
      // accepted for protocol parity; nothing hardware specific
    }
    else if (key == "REQ") {
      sendStatus(true);
    }
    else if (key == "VER") {
      sendIdentAndState();
    }
    else if (key.startsWith("SET ")) {
      String setCmd = token.substring(4);  // after "SET "
      setCmd.trim();
      if (setCmd.startsWith("ENA:")) {
        String pin = setCmd.substring(4); pin.trim();
        if (pin == SETTINGS_PIN) { settingsEnabled = true; Serial.println("SET ENA> ;"); }
        else { Serial.println("SET ENA:FAIL ;"); }
      }
      else if (setCmd.startsWith("FW:") && settingsEnabled) {
        String version = setCmd.substring(3); version.trim();
        int dotPos = version.indexOf('.');
        if (dotPos > 0 && dotPos < version.length() - 1) {
          String majorStr = version.substring(0, dotPos);
          String minorStr = version.substring(dotPos + 1);
          int major = majorStr.toInt(); int minor = minorStr.toInt();
          if (major >= 1 && major <= 9 && minor >= 0 && minor <= 9) {
            CFG_PCB_VERSION = String("PCb ") + major + "." + minor;
            Serial.println("SET FW:OK ;");
          } else Serial.println("SET FW:RANGE ;");
        } else Serial.println("SET FW:FORMAT ;");
      }
      else if (setCmd.startsWith("ACID:") && settingsEnabled) {
        String ident = setCmd.substring(5); ident.trim();
        if (ident.length() >= 3 && ident.length() <= 8) {
          int dashPos = ident.indexOf('-');
          bool valid = (dashPos == 1) && isAlpha(ident.charAt(0));
          if (valid) {
            for (int i = dashPos + 1; i < ident.length(); i++) if (!isAlpha(ident.charAt(i))) { valid = false; break; }
          }
          if (valid) { CFG_AIRCRAFT_REG = ident; Serial.println("SET ACID:OK ;"); }
          else Serial.println("SET ACID:FORMAT ;");
        } else Serial.println("SET ACID:FORMAT ;");
      }
      else if (setCmd.startsWith("EXIT")) {
        settingsEnabled = false; Serial.println("SET EXIT:OK ;");
      }
      else if (setCmd.startsWith("WRI:") && settingsEnabled) {
        String cmd = setCmd.substring(4); cmd.trim();
        if (cmd.equalsIgnoreCase("YES")) {
          saveHWInfo();
          settingsEnabled = false;
          Serial.println("SET WRI:OK ;");
          triggerSoftwareReset();
        }
        else Serial.println("SET WRI:FORMAT ;");
      }
      else if (setCmd.startsWith("WRITE")) {
        if (settingsEnabled) {
          char regbuf[8]; for (int i = 0; i < 8; i++) regbuf[i] = (i < CFG_AIRCRAFT_REG.length()) ? CFG_AIRCRAFT_REG.charAt(i) : ' ';
          char pcbbuf[8]; for (int i = 0; i < 8; i++) pcbbuf[i] = (i < CFG_PCB_VERSION.length()) ? CFG_PCB_VERSION.charAt(i) : ' ';
          uint8_t checksum = calcCfgChecksum(EEPROM_FORMAT_VERSION, regbuf, pcbbuf);
          for (int i = 0; i < 8; i++) EEPROM.write(EEPROM_BASE_ADDR + 3 + i, regbuf[i]);
          for (int i = 0; i < 8; i++) EEPROM.write(EEPROM_BASE_ADDR + 11 + i, pcbbuf[i]);
          EEPROM.write(EEPROM_BASE_ADDR + 19, checksum);
          settingsEnabled = false; Serial.println("WRITE:OK ;");
        } else Serial.println("WRITE:LOCKED ;");
      }
    }
  }
}

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') c = ';';
    serialAccum += c;
    int idx;
    while ((idx = serialAccum.indexOf(';')) >= 0) {
      String token = serialAccum.substring(0, idx);
      token.trim();
      serialAccum = serialAccum.substring(idx + 1);
      if (token.length() == 0) continue;
      processIncomingLine(token + ";");
    }
  }
}

// ============================================================================
// BOOT SEQUENCE (short)
// ============================================================================
unsigned long bootStart = 0;
bool bootDone = false;
enum BootState { BOOT_INIT, BOOT_INIT_SHOW, BOOT_COUNTDOWN, BOOT_COMPLETE };
BootState bootSeq = BOOT_INIT;

void runBoot() {
  unsigned long e = millis() - bootStart;
  
  // Stage 1: Flash pattern (0-240ms)
  if (e < 240) {
    if (e < 120) {
      desiredLedState = 0xAAAAAAAA; applyLEDOutputs();
    } else {
      desiredLedState = 0x55555555; applyLEDOutputs();
    }
  }
  // Stage 2: Show firmware as "u1.0" (240-740ms)
  else if (e < 740) {
    // Build digits from CFG_PCB_VERSION (e.g., "PCB 1.0" -> "1.0")
    String pcb = CFG_PCB_VERSION; pcb.trim();
    int sp = pcb.indexOf(' ');
    String digits = (sp >= 0 && sp < pcb.length()-1) ? pcb.substring(sp+1) : pcb;
    // Compact to 4 chars: 'u', digit, '.', digit
    String fwDisp = "u";
    if (digits.length() >= 1) fwDisp += String(digits.charAt(0));
    else fwDisp += "1";
    fwDisp += ".";
    if (digits.length() >= 3) fwDisp += String(digits.charAt(2));
    else fwDisp += "0";
    updateDisplay(fwDisp);
  }
  // Stage 3: Show "Id  " (740-1240ms)
  else if (e < 1240) {
    updateDisplay("Id  ");
  }
  // Stage 4: Show first 2 characters of ACID (e.g., "D-") (1240-1740ms)
  else if (e < 1740) {
    String reg = CFG_AIRCRAFT_REG;
    String first2 = "D-";  // sensible fallback
    if (reg.length() >= 2) first2 = reg.substring(0, 2);
    else if (reg.length() == 1) first2 = reg + " ";
    updateDisplay(first2);
  }
  // Stage 5: Show last 4 characters of ACID or fallback "A320" (1740-2240ms)
  else if (e < 2240) {
    String reg = CFG_AIRCRAFT_REG;
    String last4 = (reg.length() >= 4) ? reg.substring(reg.length() - 4) : "A320";
    updateDisplay(last4);
  }
  // Stage 6: Done
  else {
    desiredLedState = 0; applyLEDOutputs();
    updateDisplay("    ");
    bootDone = true;
    sendIdentAndState();
  }
}

// ============================================================================
// DIAG MODE
// ============================================================================
void runDiag() {
  unsigned long e = millis() - diagStart;
  
  // Stage 1: LED Walk (0-3200ms) - walk through all 32 LEDs
  if (e < 3200) {
    int ledIdx = (e / 100) % 32;  // 100ms per LED, 32 LEDs total
    desiredLedState = (1UL << ledIdx);
    applyLEDOutputs();
    updateDisplay("    ");
  }
  // Stage 2: Display Walk (3200-8200ms) - all 4 digits count 0-9, DP on even numbers
  else if (e < 8200) {
    unsigned long dispE = e - 3200;
    int digit = dispE / 500;  // 0..9, each 500ms
    if (digit > 9) digit = 9;
    bool showDP = (digit == 0 || digit == 2 || digit == 4 || digit == 6 || digit == 8);
    for (int pos = 0; pos < 4; pos++) {
      lc.setChar(0, 3 - pos, '0' + digit, showDP);
    }
    desiredLedState = 0; applyLEDOutputs();
  }
  // Stage 3: Segment Walk (8200-13200ms) - light each segment
  else if (e < 13200) {
    unsigned long segE = e - 8200;
    int segIdx = (segE / 600) % 8;  // 8 segments, 600ms each
    byte segPattern = 1 << segIdx;
    for (int digit = 0; digit < 4; digit++) {
      lc.setRow(0, 3 - digit, segPattern);
    }
    desiredLedState = 0; applyLEDOutputs();
  }
  // Stage 4: All segments on, all LEDs on (13200-15200ms)
  else if (e < 15200) {
    updateDisplay("8888");
    desiredLedState = 0xFFFFFFFF; applyLEDOutputs();
  }
  // Stage 5: Done
  else {
    updateDisplay("    ");
    desiredLedState = 0; applyLEDOutputs();
    diagMode = DIAG_IDLE;
    Serial.println("DIAG:COMPLETE;");
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(ledLatchPin, OUTPUT);
  pinMode(ledClockPin, OUTPUT);
  pinMode(ledDataPin, OUTPUT);

  pinMode(inputClockPin, OUTPUT);
  pinMode(inputLatchPin, OUTPUT);
  pinMode(inputDataPin, INPUT);

  pinMode(backlightPwmPin, OUTPUT);
  pinMode(annunPwmPin, OUTPUT);

  initDisplay();
  setLEDState(0);
  desiredBacklight = desiredAnnun = 0;
  analogWrite(backlightPwmPin, desiredBacklight);
  analogWrite(annunPwmPin, desiredAnnun);
  // Load EEPROM configuration
  loadHWInfo();

  bootStart = millis();
}

void loop() {
  unsigned long now = millis();
  if (!bootDone) { runBoot(); return; }

  if (diagMode == DIAG_RUNNING) { runDiag(); return; }

  processSerial();

  // Inputs with debounce
  uint8_t currentIn1, currentIn2;
  readShiftRegisters(currentIn1, currentIn2);
  bool changed = (currentIn1 != lastInputState1) || (currentIn2 != lastInputState2);
  if (changed && (now - lastDebounceTs) >= DEBOUNCE_MS) {
    lastDebounceTs = now;
    lastInputState1 = currentIn1;
    lastInputState2 = currentIn2;
    inputState1 = currentIn1;
    inputState2 = currentIn2;
    sendStatus(true);
  }

  // Potentiometer handling (deadband to reduce noise)
  potValue = analogRead(potPin);
  if (abs(potValue - lastPotValue) > 4) {
    lastPotValue = potValue;
    // Map pot to backlight by default; host can override via BL command
    desiredBacklight = map(potValue, 0, 1023, 0, 255);
    analogWrite(backlightPwmPin, desiredBacklight);
    sendStatus(true);
  }
}
