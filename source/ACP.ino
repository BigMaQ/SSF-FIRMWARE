// ACP Control Script, (w) 2025 M. Quatember
// Sauberer, human-readable Sketch mit HC165 Off-by-One Fix und Poti-Optimierung

// --- Pins ---
const int muxSelectPins[] = {A0, A1, A2, A3}; // S0..S3
const int muxOutputPin = A6;

const int inputDataPin  = 16;
const int inputClockPin = 14;
const int inputLatchPin = 15;

const int ledLatchPin = 2;
const int ledClockPin = 5;
const int ledDataPin  = 7;
const int backlightPWM = 3;
const int annunPWM     = 6;

// --- Panel identification ---
const char* PANEL_IDENT = "ACP, v1.2 MAQ";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// --- EEPROM / PCB identifier (from RMP) ---
#include <EEPROM.h>
#include <avr/wdt.h>

const int EEPROM_BASE_ADDR = 0; // start address
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 2;  // layout version

// Stored strings (editable via serial, savable to EEPROM)
String CFG_AIRCRAFT_REG = "ACP";   // default 3-8 chars
String CFG_PCB_VERSION   = "PCB 1.0"; // default
bool CFG_PCB_IS_12 = false; // helper flag

// EEPROM settings PIN to allow writes
const String SETTINGS_PIN = "0815";
bool settingsEnabled = false;

// --- LED state (desired + hardware) ---
uint8_t desiredLedBacklight = 0x00;
uint8_t desiredLedAnnun     = 0x00;
uint8_t desiredBlLevel      = 0;
uint8_t desiredAnLevel      = 0;

uint8_t hwLedBacklight = 0x00;
uint8_t hwLedAnnun     = 0x00;
uint8_t hwBlLevel      = 0;
uint8_t hwAnLevel      = 0;
uint8_t desiredLedExtra = 0x00; // third 8-bit shift-register for PCB 1.2
uint8_t hwLedExtra      = 0x00;
// (previously had LED_ALL override variables - removed)

// --- Runtime buffers / flags ---
String serialAccum = "";

// ============================================================================
// UNIVERSAL PARAMETERS (same across all firmware versions)
// ============================================================================
// These parameters should be synchronized via Python at connect time
// Commands: BL:xxx (backlight), AN:xxx (annunciator), DISP_BL:xx (display)
// Smoothing: SMO_THR (threshold), SMO_SAM (samples), SMO_DLY (delay)

// --- Smoothing Parameters (configurable via serial) ---
// SMO_THR: Minimum change threshold for analog inputs (MUX/poti)
// SMO_SAM: Number of samples for moving average (1-8)
// SMO_DLY: Delay in microseconds after analog select (50-1000)
int SMO_THRESHOLD = 10;           // Minimum change to trigger send (default: 10)
int SMO_SAMPLES = 4;              // Moving average window size (default: 4)
int SMO_DELAY_US = 300;           // Delay after MUX/analog select (microseconds, default: 300)

// --- Brightness Parameters (configurable via serial) ---
// BL:xxx    Backlight brightness (0-255, PWM)
// AN:xxx    Annunciator brightness (0-255, PWM) - optional per panel
// DISP_BL:xx Display brightness (0-15, MAX7219) - optional per panel
int BL_LEVEL = 0;                 // Backlight current level
int AN_LEVEL = 0;                 // Annunciator current level (if applicable)
int DISP_BL_LEVEL = 15;           // Display brightness current level (if applicable)

// --- Timing / throttles / debounce ---
unsigned long lastLoopTs      = 0;
const unsigned long LOOP_INTERVAL_MS      = 10;
unsigned long lastSendTs      = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
unsigned long lastDebounceTs  = 0;
const unsigned long DEBOUNCE_MS = 12;
int muxVals[16];
int lastMuxVals[16];
int muxBuffer[16][8];           // Ring buffer for smoothing (max 8 samples)
int muxBufferIdx[16];           // Index into ring buffer per channel
uint8_t inputState1 = 0xFF;
uint8_t inputState2 = 0xFF;
uint8_t lastInputState1 = 0xFF;
uint8_t lastInputState2 = 0xFF;

// --- Combo / DIAG ---
unsigned long comboStartTime = 0;
int comboStage = 0; // 0 idle, 1 running
bool forceSendNext = false;

// --- DIAG / self-test ---
void runDiagTest() {
  Serial.println("DIAG START");
  // Save current desired state
  uint8_t saveBack = desiredLedBacklight;
  uint8_t saveAnn  = desiredLedAnnun;
  uint8_t saveExtra = desiredLedExtra;
  uint8_t saveBl = desiredBlLevel;
  uint8_t saveAn = desiredAnLevel;

  // Ensure manual control during diag
  // ensure manual control during diag (no override variable used)

  int nregs = CFG_PCB_IS_12 ? 3 : 2;

  // LED walk test: single LED moving across each SR simultaneously
  for (int bit = 0; bit < 8; ++bit) {
    uint8_t pat = (1 << bit);
    desiredLedBacklight = pat;
    desiredLedAnnun     = pat;
    if (nregs == 3) desiredLedExtra = pat;
    applyLEDOutputs();
    delay(120);
  }

  // Reverse walk back
  for (int bit = 7; bit >= 0; --bit) {
    uint8_t pat = (1 << bit);
    desiredLedBacklight = pat;
    desiredLedAnnun     = pat;
    if (nregs == 3) desiredLedExtra = pat;
    applyLEDOutputs();
    delay(120);
  }

  // Driver/shift test: per-register toggles 10101010 / 01010101
  uint8_t p1 = 0b10101010;
  uint8_t p2 = 0b01010101;
  // Backlight (SR1)
  desiredLedBacklight = p1; applyLEDOutputs(); delay(180);
  desiredLedBacklight = p2; applyLEDOutputs(); delay(180);
  // Annun (SR2)
  desiredLedAnnun = p1; applyLEDOutputs(); delay(180);
  desiredLedAnnun = p2; applyLEDOutputs(); delay(180);
  // Extra (SR3) if present
  if (nregs == 3) {
    desiredLedExtra = p1; applyLEDOutputs(); delay(180);
    desiredLedExtra = p2; applyLEDOutputs(); delay(180);
  }

  // Short full-on / full-off flash to indicate end
  desiredLedBacklight = 0xFF; desiredLedAnnun = 0xFF; if (nregs==3) desiredLedExtra = 0xFF; applyLEDOutputs(); delay(180);
  desiredLedBacklight = 0x00; desiredLedAnnun = 0x00; if (nregs==3) desiredLedExtra = 0x00; applyLEDOutputs(); delay(120);

  // Restore saved state
  desiredLedBacklight = saveBack;
  desiredLedAnnun     = saveAnn;
  desiredLedExtra     = saveExtra;
  desiredBlLevel = saveBl; desiredAnLevel = saveAn;
  applyLEDOutputs();

  Serial.println("DIAG END");
}

// --- Helpers ---
String decPad4(int v) {
  char buf[8];
  v = constrain(v, 0, 1023);
  sprintf(buf, "%04d", v);
  return String(buf);
}

String bin8(byte b) {
  String s = "";
  for (int i = 7; i >= 0; i--) s += ((b & (1 << i)) ? '1' : '0');
  return s;
}

uint8_t parseBin8(const String &s) {
  uint8_t v = 0;
  for (int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c == '0' || c == '1') v = (v << 1) | (c == '1' ? 1 : 0);
  }
  return v;
}

// ---------------- EEPROM helpers (ACP adaptation from RMP) ----------------
uint8_t calcCfgChecksum(uint8_t version, const char *reg8, const char *pcb8) {
  uint16_t s = version;
  for (int i = 0; i < 8; i++) s += (uint8_t)reg8[i];
  for (int i = 0; i < 8; i++) s += (uint8_t)pcb8[i];
  return (uint8_t)(s & 0xFF);
}

void loadHWInfo() {
  uint16_t magic = EEPROM.read(EEPROM_BASE_ADDR) | (EEPROM.read(EEPROM_BASE_ADDR + 1) << 8);
  if (magic != EEPROM_MAGIC) return;
  uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR + 2);
  char reg8[9]; char pcb8[9];
  for (int i = 0; i < 8; i++) reg8[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 3 + i);
  reg8[8] = '\0';
  for (int i = 0; i < 8; i++) pcb8[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 11 + i);
  pcb8[8] = '\0';
  uint8_t storedChk = EEPROM.read(EEPROM_BASE_ADDR + 19);
  uint8_t calc = calcCfgChecksum(ver, reg8, pcb8);
  if (storedChk != calc) return;
  CFG_AIRCRAFT_REG = String(reg8);
  CFG_PCB_VERSION = String(pcb8);
  CFG_PCB_IS_12 = (CFG_PCB_VERSION.indexOf("1.2") >= 0);
}

void saveHWInfo(const String &pin) {
  if (pin != SETTINGS_PIN) {
    Serial.println("SAVE_HW:DENIED");
    return;
  }
  // Prepare 8-byte padded fields
  char reg8[8]; char pcb8[8];
  for (int i = 0; i < 8; i++) reg8[i] = ' ';
  for (int i = 0; i < 8; i++) pcb8[i] = ' ';
  for (int i = 0; i < CFG_AIRCRAFT_REG.length() && i < 8; i++) reg8[i] = CFG_AIRCRAFT_REG.charAt(i);
  for (int i = 0; i < CFG_PCB_VERSION.length() && i < 8; i++) pcb8[i] = CFG_PCB_VERSION.charAt(i);

  // Write to EEPROM
  EEPROM.write(EEPROM_BASE_ADDR + 0, (uint8_t)(EEPROM_MAGIC & 0xFF));
  EEPROM.write(EEPROM_BASE_ADDR + 1, (uint8_t)((EEPROM_MAGIC >> 8) & 0xFF));
  EEPROM.write(EEPROM_BASE_ADDR + 2, EEPROM_FORMAT_VERSION);
  for (int i = 0; i < 8; i++) EEPROM.write(EEPROM_BASE_ADDR + 3 + i, reg8[i]);
  for (int i = 0; i < 8; i++) EEPROM.write(EEPROM_BASE_ADDR + 11 + i, pcb8[i]);
  uint8_t chk = calcCfgChecksum(EEPROM_FORMAT_VERSION, reg8, pcb8);
  EEPROM.write(EEPROM_BASE_ADDR + 19, chk);
  Serial.println("SAVE_HW:OK");
}

void saveHWInfo() {
  // Write full layout including magic/version, reg and pcb, and checksum
  // Build space-padded 8-byte fields to compute checksum consistently
  char reg8[8]; char pcb8[8];
  for (int i = 0; i < 8; i++) reg8[i] = ' ';
  for (int i = 0; i < 8; i++) pcb8[i] = ' ';
  for (int i = 0; i < CFG_AIRCRAFT_REG.length() && i < 8; i++) reg8[i] = CFG_AIRCRAFT_REG.charAt(i);
  for (int i = 0; i < CFG_PCB_VERSION.length() && i < 8; i++) pcb8[i] = CFG_PCB_VERSION.charAt(i);

  uint8_t checksum = calcCfgChecksum(EEPROM_FORMAT_VERSION, reg8, pcb8);

  // Write magic and format version
  EEPROM.write(EEPROM_BASE_ADDR + 0, (uint8_t)(EEPROM_MAGIC & 0xFF));
  EEPROM.write(EEPROM_BASE_ADDR + 1, (uint8_t)((EEPROM_MAGIC >> 8) & 0xFF));
  EEPROM.write(EEPROM_BASE_ADDR + 2, EEPROM_FORMAT_VERSION);

  for (int i = 0; i < 8; i++) EEPROM.write(EEPROM_BASE_ADDR + 3 + i, reg8[i]);
  for (int i = 0; i < 8; i++) EEPROM.write(EEPROM_BASE_ADDR + 11 + i, pcb8[i]);
  EEPROM.write(EEPROM_BASE_ADDR + 19, checksum);
  Serial.println("SAVE_HW:OK");
}

void refreshPCBVersionFlags() {
  String pcbNorm = CFG_PCB_VERSION;
  pcbNorm.toUpperCase();
  pcbNorm.replace(" ", "");
  CFG_PCB_IS_12 = (pcbNorm == "PCB1.2");
}

void triggerSoftwareReset(bool showMessage = true) {
  if (showMessage) Serial.println("RESTARTING");
  Serial.flush();
  delay(50);
  wdt_enable(WDTO_15MS);
  while (true) { }
}

// Print EEPROM raw bytes and parsed HW fields for debugging
void printHWInfo() {
  Serial.println("EEPROM DUMP:");
  for (int i = 0; i <= 19; i++) {
    uint8_t b = EEPROM.read(EEPROM_BASE_ADDR + i);
    if (b < 16) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');
    if ((i % 8) == 7) Serial.println();
  }
  Serial.println();

  uint16_t magic = EEPROM.read(EEPROM_BASE_ADDR) | (EEPROM.read(EEPROM_BASE_ADDR + 1) << 8);
  uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR + 2);
  char reg8[9]; char pcb8[9];
  for (int i = 0; i < 8; i++) reg8[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 3 + i);
  reg8[8] = '\0';
  for (int i = 0; i < 8; i++) pcb8[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 11 + i);
  pcb8[8] = '\0';
  uint8_t storedChk = EEPROM.read(EEPROM_BASE_ADDR + 19);
  uint8_t calc = calcCfgChecksum(ver, reg8, pcb8);

  Serial.print("MAGIC: 0x"); Serial.println(magic, HEX);
  Serial.print("FORMAT_VER: "); Serial.println(ver);
  Serial.print("AIRCRAFT_REG (raw): '"); Serial.print(reg8); Serial.println("'");
  Serial.print("PCB_VERSION (raw): '"); Serial.print(pcb8); Serial.println("'");
  Serial.print("STORED_CHK: 0x"); Serial.println(storedChk, HEX);
  Serial.print("CALC_CHK:   0x"); Serial.println(calc, HEX);

  Serial.print("Parsed CFG_AIRCRAFT_REG: "); Serial.println(CFG_AIRCRAFT_REG);
  Serial.print("Parsed CFG_PCB_VERSION: "); Serial.println(CFG_PCB_VERSION);
  Serial.print("CFG_PCB_IS_12: "); Serial.println(CFG_PCB_IS_12 ? "YES" : "NO");
}

// --- LED HW write ---
void shiftOutLEDs(uint8_t backlightBits, uint8_t annunBits, uint8_t extraBits = 0) {
  digitalWrite(ledLatchPin, LOW);
  if (CFG_PCB_IS_12) {
    // For PCB 1.2 shift out extra register first (chain order: extra, annun, backlight)
    shiftOut(ledDataPin, ledClockPin, MSBFIRST, extraBits);
    shiftOut(ledDataPin, ledClockPin, MSBFIRST, annunBits);
    shiftOut(ledDataPin, ledClockPin, MSBFIRST, backlightBits);
  } else {
    // Legacy: two registers (annun, backlight)
    shiftOut(ledDataPin, ledClockPin, MSBFIRST, annunBits);
    shiftOut(ledDataPin, ledClockPin, MSBFIRST, backlightBits);
  }
  digitalWrite(ledLatchPin, HIGH);
}

void applyLEDOutputs() {
  shiftOutLEDs(desiredLedBacklight, desiredLedAnnun, desiredLedExtra);
  analogWrite(backlightPWM, desiredBlLevel);
  analogWrite(annunPWM, desiredAnLevel);

  hwLedBacklight = desiredLedBacklight;
  hwLedAnnun     = desiredLedAnnun;
  hwLedExtra     = desiredLedExtra;
  hwBlLevel      = desiredBlLevel;
  hwAnLevel      = desiredAnLevel;
  
  // Update universal brightness cache
  BL_LEVEL = desiredBlLevel;
  AN_LEVEL = desiredAnLevel;
}

void setLEDState(uint8_t backBits, uint8_t annBits, uint8_t bl, uint8_t an, bool allowDuringCombo = false) {
  if (comboStage != 0 && !allowDuringCombo) return;
  desiredLedBacklight = backBits;
  desiredLedAnnun     = annBits;
  desiredBlLevel      = bl;
  desiredAnLevel      = an;
  applyLEDOutputs();
}

// --- MUX read helpers ---
int readMuxChannelRaw(int idx) {
  for (int b = 0; b < 4; ++b) digitalWrite(muxSelectPins[b], (idx >> b) & 1);
  delayMicroseconds(SMO_DELAY_US);
  return constrain(analogRead(muxOutputPin), 0, 1023);
}

// --- MUX Smoothing (moving average) ---
int getMuxSmoothed(int channel, int rawValue) {
  // Add raw value to ring buffer
  muxBuffer[channel][muxBufferIdx[channel]] = rawValue;
  muxBufferIdx[channel] = (muxBufferIdx[channel] + 1) % SMO_SAMPLES;
  
  // Calculate average of all samples in buffer
  int sum = 0;
  for (int i = 0; i < SMO_SAMPLES; i++) {
    sum += muxBuffer[channel][i];
  }
  return sum / SMO_SAMPLES;
}

// --- Shift Register read (HC165) with Off-by-One Fix ---
void readShiftRegisters(uint8_t &in1, uint8_t &in2) {
    // Latch
    digitalWrite(inputLatchPin, LOW);
    delayMicroseconds(20);
    digitalWrite(inputLatchPin, HIGH);
    delayMicroseconds(20);

    // Read shift registers
    byte raw1 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);
    byte raw2 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);
    in1 = ~raw1;
    in2 = ~raw2;

    // Clock pulses to stabilize HC165 (Timing-Fix)
    digitalWrite(inputLatchPin, LOW);
    delayMicroseconds(20);
    digitalWrite(inputLatchPin, HIGH);
    delayMicroseconds(20);
    for (int i = 0; i < 16; i++) {
        digitalWrite(inputClockPin, LOW);
        delayMicroseconds(5);
        digitalWrite(inputClockPin, HIGH);
        delayMicroseconds(5);
    }
}

// --- Serial IDENT / state ---
void sendIdentAndState() {
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
  Serial.print(";STATE:RUNNING;");
  Serial.print("HW_REG:"); Serial.print(CFG_AIRCRAFT_REG);
  Serial.print(";HW_PCB:"); Serial.print(CFG_PCB_VERSION);
  Serial.println();
}



// --- Serial command parsing ---
void processIncomingLine(const String &line) {
  int start = 0;
  while (true) {
    int sep = line.indexOf(';', start);
    if (sep < 0) break;
    String token = line.substring(start, sep); start = sep + 1;
    token.trim();
    if (token.length() == 0) continue;

    if (token.indexOf(':') < 0) {
      if (token.equalsIgnoreCase("VER") || token.equalsIgnoreCase("VERSION")) { sendIdentAndState(); continue; }
      if (token.equalsIgnoreCase("REQ")) { forceSendNext = true; continue; }
      if (token.equalsIgnoreCase("RESET") || token.equalsIgnoreCase("RESTART") || token.equalsIgnoreCase("REBOOT")) { triggerSoftwareReset(); continue; }
      continue;
    }

    int colon = token.indexOf(':');
    if (colon < 0) continue;
    String key = token.substring(0, colon);
    String val = token.substring(colon + 1);
    key.trim(); val.trim();

    if (key.equalsIgnoreCase("LED1")) { desiredLedBacklight = parseBin8(val); applyLEDOutputs(); }
    else if (key.equalsIgnoreCase("LED2")) { desiredLedAnnun = parseBin8(val); applyLEDOutputs(); }
    else if (key.equalsIgnoreCase("LED3")) {
      // Only supported on PCB 1.2
      if (CFG_PCB_IS_12) { desiredLedExtra = parseBin8(val); applyLEDOutputs(); }
    }
    // LED_ALL command removed (diagnostics are used instead)
    else if (key.equalsIgnoreCase("BL")) { BL_LEVEL = constrain(val.toInt(),0,255); setLEDState(desiredLedBacklight, desiredLedAnnun, BL_LEVEL, desiredAnLevel); }
    else if (key.equalsIgnoreCase("AN")) { AN_LEVEL = constrain(val.toInt(),0,255); setLEDState(desiredLedBacklight, desiredLedAnnun, desiredBlLevel, AN_LEVEL); }
    else if (key.equalsIgnoreCase("DISP_BL")) { DISP_BL_LEVEL = constrain(val.toInt(),0,15); }
    else if (key.equalsIgnoreCase("SMO_THR")) { SMO_THRESHOLD = constrain(val.toInt(), 1, 100); }
    else if (key.equalsIgnoreCase("SMO_SAM")) { SMO_SAMPLES = constrain(val.toInt(), 1, 8); }
    else if (key.equalsIgnoreCase("SMO_DLY")) { SMO_DELAY_US = constrain(val.toInt(), 50, 1000); }
    else if (key.equalsIgnoreCase("SET_REG")) { CFG_AIRCRAFT_REG = val; }
    else if (key.equalsIgnoreCase("SET_PCB")) { CFG_PCB_VERSION = val; CFG_PCB_IS_12 = (CFG_PCB_VERSION.indexOf("1.2")>=0); }
    else if (key.equalsIgnoreCase("SAVE_HW")) { saveHWInfo(val); }
    else if (token.startsWith("SET ")) {
      String setCmd = token.substring(4);
      setCmd.trim();
      // Echo back the received SET entry for user feedback
      Serial.print("SET RECV:"); Serial.print(setCmd); Serial.println(";");

      if (setCmd.startsWith("ENA:")) {
        String pin = setCmd.substring(4);
        pin.trim();
        if (pin == SETTINGS_PIN) {
          settingsEnabled = true;
          Serial.println("SET ENA> ;");
        } else {
          Serial.println("SET ENA:FAIL ;");
        }
      }
      else if (setCmd.startsWith("FW:") && settingsEnabled) {
        String version = setCmd.substring(3);
        version.trim();
        int dotPos = version.indexOf('.');
        if (dotPos > 0 && dotPos < version.length() - 1) {
          String majorStr = version.substring(0, dotPos);
          String minorStr = version.substring(dotPos + 1);
          int major = majorStr.toInt();
          int minor = minorStr.toInt();
          if (major >= 1 && major <= 9 && minor >= 0 && minor <= 9) {
            CFG_PCB_VERSION = String("PCB ") + major + "." + minor;
            refreshPCBVersionFlags();
            Serial.println("SET FW:OK ;");
          } else {
            Serial.println("SET FW:RANGE ;");
          }
        } else {
          Serial.println("SET FW:FORMAT ;");
        }
      }
      else if (setCmd.startsWith("ACID:") && settingsEnabled) {
        String ident = setCmd.substring(5);
        ident.trim();
        if (ident.length() >= 3 && ident.length() <= 8) {
          int dashPos = ident.indexOf('-');
          if (dashPos > 0 && dashPos < ident.length() - 1) {
            bool valid = true;
            if (dashPos != 1) valid = false;
            else {
              if (!isAlpha(ident.charAt(0))) valid = false;
              for (int i = dashPos + 1; i < ident.length(); i++) {
                if (!isAlpha(ident.charAt(i))) { valid = false; break; }
              }
            }
            if (valid) { CFG_AIRCRAFT_REG = ident; Serial.println("SET ACID:OK ;"); }
            else Serial.println("SET ACID:FORMAT ;");
          } else Serial.println("SET ACID:FORMAT ;");
        } else Serial.println("SET ACID:FORMAT ;");
      }
      else if (setCmd.startsWith("EXIT")) {
        settingsEnabled = false;
        Serial.println("SET EXIT:OK ;");
      }
      else if (setCmd.startsWith("WRI:") && settingsEnabled) {
        String cmd = setCmd.substring(4);
        cmd.trim();
        if (cmd.equalsIgnoreCase("YES")) {
          saveHWInfo();
          settingsEnabled = false;
          Serial.println("SET WRI:OK ;");
          triggerSoftwareReset();
        } else {
          Serial.println("SET WRI:FORMAT ;");
        }
      }
      else if (setCmd.startsWith("WRITE")) {
        if (settingsEnabled) {
          // Use the central save function which writes magic/version/fields/checksum
          saveHWInfo();
          settingsEnabled = false;
          Serial.println("WRITE:OK ;");
          // Restart so new EEPROM values are applied
          triggerSoftwareReset();
        } else {
          Serial.println("WRITE:LOCKED ;");
        }
      }
    }
    else if (key.equalsIgnoreCase("REQ")) forceSendNext = true;
    else if (key.equalsIgnoreCase("VER")) sendIdentAndState();
  }
}

void processSerialTokensFromHost() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') c = ';';
    serialAccum += c;

    int idx;
    while ((idx = serialAccum.indexOf(';')) >= 0) {
      String token = serialAccum.substring(0, idx); token.trim();
      serialAccum = serialAccum.substring(idx + 1);
      if (token.length() == 0) continue;

      // direct single-word commands (no colon) we want to handle immediately
      String tokenUp = token;
      tokenUp.toUpperCase();

      if (tokenUp == "VER" || tokenUp == "VERSION") {
        sendIdentAndState();
        identSentOnStart = true;
        pauseUntil = millis() + 200;
        continue;
      }
      if (tokenUp == "HW" || tokenUp == "HW?") {
        printHWInfo();
        continue;
      }
      if (tokenUp == "DIAG") {
        runDiagTest();
        continue;
      }
      if (tokenUp == "RESET" || tokenUp == "RESTART" || tokenUp == "REBOOT") {
        triggerSoftwareReset();
        continue;
      }
      // LED_ALL single-word handlers removed; use DIAG for tests
      if (tokenUp == "REQ") {
        sendStatus();
        continue;
      }
      if (token.indexOf(':') >= 0) {
        processIncomingLine(token + ";");
        continue;
      }

    // otherwise ignore unknown single-word token

    }
  }
}

// --- Status ---
void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;
  for (int i=0;i<16;i++){Serial.print("MUX");Serial.print(i);Serial.print(":");Serial.print(decPad4(muxVals[i]));Serial.print(";");}
  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";IN2:"); Serial.println(bin8(inputState2));
  lastSendTs = now; forceSendNext = false;
}



// --- Setup ---
void setup() {
  Serial.begin(115200);
  for(int i=0;i<4;i++) pinMode(muxSelectPins[i], OUTPUT);
  pinMode(muxOutputPin, INPUT);
  pinMode(inputDataPin, INPUT); pinMode(inputClockPin, OUTPUT); pinMode(inputLatchPin, OUTPUT);
  pinMode(ledLatchPin, OUTPUT); pinMode(ledClockPin, OUTPUT); pinMode(ledDataPin, OUTPUT);
  pinMode(backlightPWM, OUTPUT); pinMode(annunPWM, OUTPUT);
  analogWrite(backlightPWM,0); analogWrite(annunPWM,0);
  for(int i=0;i<16;i++){muxVals[i]=0; lastMuxVals[i]=-9999; muxBufferIdx[i]=0;}
  lastInputState1=0xFF; lastInputState2=0xFF;
  // Load hardware info from EEPROM if available
  loadHWInfo();
  // Startup LED blink
  // Include the optional 3rd shift-register for PCB 1.2 during POST
  if (CFG_PCB_IS_12) desiredLedExtra = 0xAA; else desiredLedExtra = 0x00;
  setLEDState(0xAA,0xAA,200,200,true); delay(150);
  if (CFG_PCB_IS_12) desiredLedExtra = 0x55; else desiredLedExtra = 0x00;
  setLEDState(0x55,0x55,200,200,true); delay(150);
  if (CFG_PCB_IS_12) desiredLedExtra = 0xFF; else desiredLedExtra = 0x00;
  setLEDState(0xFF,0xFF,0,0,true); delay(150);
  desiredLedExtra = 0x00;
  setLEDState(0x00,0x00,0,0,true);
  delay(150);
  sendIdentAndState();
  identSentOnStart = true;
  forceSendNext = true;
}

// --- Main Loop ---
void loop() {
  unsigned long now = millis();
  if (now < pauseUntil) return;
  processSerialTokensFromHost();
  if(now - lastLoopTs < LOOP_INTERVAL_MS) return; lastLoopTs=now;

  // --- Read MUX ---
  for(int i=0;i<16;i++){
    int rawVal = readMuxChannelRaw(i);
    muxVals[i] = getMuxSmoothed(i, rawVal);
  }

  // --- Read ShiftRegisters (HC165) ---
  readShiftRegisters(inputState1,inputState2);

  // --- Button Changes ---
  if(inputState1 != lastInputState1 || inputState2 != lastInputState2){
    if(now - lastDebounceTs >= DEBOUNCE_MS){
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      sendStatus();
    }
  }

  // --- MUX changes (using SMO_THRESHOLD parameter) ---
  bool muxChanged = false;
  for(int i=0;i<16;i++){if(abs(muxVals[i]-lastMuxVals[i])>=SMO_THRESHOLD){muxChanged=true;lastMuxVals[i]=muxVals[i];}}
  if(muxChanged) sendStatus();

  // --- DIAG Combo ---
  bool comboNow = (inputState1==0b01000001)&&(inputState2==0b00001000);
  if(comboNow && comboStage==0){ comboStage=1; comboStartTime=now; Serial.println("DIAG START"); setLEDState(0xFF,0xFF,200,200,true);}
  if(comboStage==1 && (now-comboStartTime)>=15000){ comboStage=0; Serial.println("DIAG END"); setLEDState(0x00,0x00,0,0,true);}
}
