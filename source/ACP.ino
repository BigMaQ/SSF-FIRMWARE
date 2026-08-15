// ACP Control Script, (w) 2025 M. Quatember
// Sauberer, human-readable Sketch mit HC165 Off-by-One Fix und Poti-Optimierung

#include <EEPROM.h>
#include <avr/wdt.h>

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
const char* PANEL_IDENT = "ACP 1 CPT, v2.4 MAQ";
const char  FW_VERSION[] = "2.4";
const char  PANEL_SN_PREFIX[] = "ACP-";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// Serial Number
char CFG_SERIAL_NUMBER[10] = "";

// --- Config variables ---
char CFG_AIRCRAFT_REG[9] = "D-A320";
char CFG_PCB_VERSION[9] = "PCB 1.0";
bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";

// --- Hardware revision (EEPROM persistent) ---
// Rev 1: 2 LED shift-registers (16 LEDs)
// Rev 2: 3 LED shift-registers (24 LEDs)
const uint8_t HWREV_MIN = 1;
const uint8_t HWREV_MAX = 2;
const int EEPROM_ADDR_HWREV = 0;
const char* HWREV_PASSWORD = "ACP-HW-SET";
uint8_t hwRevision = 1;

// --- LED state (desired + hardware) ---
// Rev 1: LED1 bits 0-6 = backlight (7 LEDs), bit 7 = PA_SEL annunciator
// Rev 2+: LED1 bits 0-5 = backlight (6 LEDs), bits 6-7 = free
const uint8_t BACKLIGHT_MASK1_REV1 = 0b01111111;  // Rev1: bits 0-6 → backlight, bit 7 = PA_SEL
const uint8_t BACKLIGHT_MASK1_REV2 = 0b00111111;  // Rev2+: bits 0-5 → backlight

uint8_t desiredLedBacklight = 0x00;  // LED1 free bits only (non-backlight)
uint8_t desiredLedAnnun     = 0x00;
uint8_t desiredLed3         = 0x00;
uint8_t desiredBlLevel      = 0;
uint8_t desiredAnLevel      = 0;

uint8_t hwLedBacklight = 0x00;
uint8_t hwLedAnnun     = 0x00;
uint8_t hwLed3         = 0x00;
uint8_t hwBlLevel      = 0;
uint8_t hwAnLevel      = 0;

bool diagActive = false;  // Set true during DIAG to bypass backlight mask

// --- Boot state ---
enum BootState { BOOT_INIT, BOOT_SHOW, BOOT_FADE, BOOT_RUNNING };
BootState bootState = BOOT_INIT;
unsigned long bootFadeStart = 0;
const unsigned long BOOT_FADE_DURATION_MS = 1500;
unsigned long bootShowStart = 0;
const unsigned long BOOT_SHOW_STEP_MS = 50;

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

// --- LED HW write ---
void shiftOutLEDs(uint8_t backlightBits, uint8_t annunBits, uint8_t led3Bits) {
  digitalWrite(ledLatchPin, LOW);
  // Bei 3 Registern zuerst das hinterste Register schieben.
  if (hwRevision >= 2) {
    shiftOut(ledDataPin, ledClockPin, MSBFIRST, led3Bits);
  }
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, annunBits);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, backlightBits);
  digitalWrite(ledLatchPin, HIGH);
}

void applyLEDOutputs() {
  // HW-Rev-abhängige Backlight-Maske wählen
  uint8_t blMask = (hwRevision == 1) ? BACKLIGHT_MASK1_REV1 : BACKLIGHT_MASK1_REV2;

  // Combine free bits (from LED1) + backlight bits (from BL)
  // In DIAG mode, bypass mask — direct full 8-bit control
  uint8_t backlightBits = (diagActive || desiredBlLevel > 0) ? blMask : 0x00;
  uint8_t freeBits = diagActive ? desiredLedBacklight : (desiredLedBacklight & ~blMask);
  uint8_t backlightOut = freeBits | backlightBits;

  uint8_t led3Effective = (hwRevision >= 2) ? desiredLed3 : 0x00;
  shiftOutLEDs(backlightOut, desiredLedAnnun, led3Effective);
  // Inverted PWM: BL:0 = off, BL:255 = full brightness
  analogWrite(backlightPWM, 255 - desiredBlLevel);
  analogWrite(annunPWM, desiredAnLevel);

  hwLedBacklight = backlightOut;
  hwLedAnnun     = desiredLedAnnun;
  hwLed3         = led3Effective;
  hwBlLevel      = desiredBlLevel;
  hwAnLevel      = desiredAnLevel;
  
  // Update universal brightness cache
  BL_LEVEL = desiredBlLevel;
  AN_LEVEL = desiredAnLevel;
}

void setLEDState(uint8_t backBits, uint8_t annBits, uint8_t led3Bits, uint8_t bl, uint8_t an, bool allowDuringCombo = false) {
  if (comboStage != 0 && !allowDuringCombo) return;
  desiredLedBacklight = backBits;
  desiredLedAnnun     = annBits;
  desiredLed3         = led3Bits;
  desiredBlLevel      = bl;
  desiredAnLevel      = an;
  applyLEDOutputs();
}

// --- HW revision helpers ---
bool isValidHwRevision(int rev) {
  return rev >= HWREV_MIN && rev <= HWREV_MAX;
}

void loadHwRevisionFromEEPROM() {
  uint8_t rev = EEPROM.read(EEPROM_ADDR_HWREV);
  if (isValidHwRevision(rev)) {
    hwRevision = rev;
  } else {
    hwRevision = 1;
    EEPROM.update(EEPROM_ADDR_HWREV, hwRevision);
  }
}

bool saveHwRevisionToEEPROM(uint8_t rev) {
  if (!isValidHwRevision(rev)) return false;
  hwRevision = rev;
  EEPROM.update(EEPROM_ADDR_HWREV, hwRevision);
  return true;
}

// --- EEPROM config v3 infrastructure ---
const int EEPROM_BASE_ADDR = 10; // Start after HWREV byte
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 3;

void generateSerialNumber(char* out) {
  randomSeed(analogRead(A0) + analogRead(A1) + micros());
  const char hexChars[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) { out[i] = hexChars[random(0, 16)]; }
  out[8] = 0;
}

uint8_t calcCfgChecksum(uint8_t version, const char *reg8, const char *pcb8, const char *sn9) {
  uint16_t sum = version;
  for (int i = 0; i < 8; i++) sum += (uint8_t)reg8[i];
  for (int i = 0; i < 8; i++) sum += (uint8_t)pcb8[i];
  for (int i = 0; i < 9; i++) sum += (uint8_t)sn9[i];
  return (uint8_t)(sum & 0xFF);
}

void loadHWInfo() {
  uint16_t magic = (uint16_t)EEPROM.read(EEPROM_BASE_ADDR) | ((uint16_t)EEPROM.read(EEPROM_BASE_ADDR+1) << 8);
  if (magic != EEPROM_MAGIC) { generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); return; }
  uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR+2);
  char regbuf[9]; char pcbbuf[9];
  for (int i=0;i<8;i++){regbuf[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+3+i);pcbbuf[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+11+i);}
  regbuf[8]=0;pcbbuf[8]=0;
  if(ver<=2){generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();return;}
  if(ver!=EEPROM_FORMAT_VERSION){generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();return;}
  char snbuf[10];for(int i=0;i<9;i++)snbuf[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+19+i);snbuf[9]=0;
  if(EEPROM.read(EEPROM_BASE_ADDR+28)!=calcCfgChecksum(ver,regbuf,pcbbuf,snbuf)){generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();return;}
  int ri=0;for(int i=0;i<8;i++)if(regbuf[i]!=0&&regbuf[i]!=' ')CFG_AIRCRAFT_REG[ri++]=regbuf[i];CFG_AIRCRAFT_REG[ri]=0;
  int pi=0;for(int i=0;i<8;i++)if(pcbbuf[i]!=0&&pcbbuf[i]!=' ')CFG_PCB_VERSION[pi++]=pcbbuf[i];CFG_PCB_VERSION[pi]=0;
  int si=0;for(int i=0;i<8;i++)if(snbuf[i]!=0&&snbuf[i]!=' ')CFG_SERIAL_NUMBER[si++]=snbuf[i];CFG_SERIAL_NUMBER[si]=0;
  if(si==0){generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();}
}

void saveHWInfo() {
  char regbuf[8];int rl=strlen(CFG_AIRCRAFT_REG);for(int i=0;i<8;i++)regbuf[i]=(i<rl)?CFG_AIRCRAFT_REG[i]:' ';
  char pcbbuf[8];int pl=strlen(CFG_PCB_VERSION);for(int i=0;i<8;i++)pcbbuf[i]=(i<pl)?CFG_PCB_VERSION[i]:' ';
  char snbuf[9];int sl=strlen(CFG_SERIAL_NUMBER);for(int i=0;i<8;i++)snbuf[i]=(i<sl)?CFG_SERIAL_NUMBER[i]:' ';snbuf[8]=0;
  EEPROM.update(EEPROM_BASE_ADDR+0,(uint8_t)(EEPROM_MAGIC&0xFF));
  EEPROM.update(EEPROM_BASE_ADDR+1,(uint8_t)((EEPROM_MAGIC>>8)&0xFF));
  EEPROM.update(EEPROM_BASE_ADDR+2,EEPROM_FORMAT_VERSION);
  for(int i=0;i<8;i++)EEPROM.update(EEPROM_BASE_ADDR+3+i,(uint8_t)regbuf[i]);
  for(int i=0;i<8;i++)EEPROM.update(EEPROM_BASE_ADDR+11+i,(uint8_t)pcbbuf[i]);
  for(int i=0;i<9;i++)EEPROM.update(EEPROM_BASE_ADDR+19+i,(uint8_t)snbuf[i]);
  EEPROM.update(EEPROM_BASE_ADDR+28,calcCfgChecksum(EEPROM_FORMAT_VERSION,regbuf,pcbbuf,snbuf));
}

void triggerSoftwareReset() {
  Serial.flush(); delay(50);
  wdt_enable(WDTO_15MS);
  while(true){}
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
  if (CFG_SERIAL_NUMBER[0] != 0) {
    Serial.print(", SN:"); Serial.print(PANEL_SN_PREFIX); Serial.print(CFG_SERIAL_NUMBER);
  }
  Serial.print(";STATE:RUNNING;");
  Serial.print("HWREV:"); Serial.print(hwRevision); Serial.print(";");
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
      if (token.equalsIgnoreCase("VER") || token.equalsIgnoreCase("VERSION")) {
        sendIdentAndState();
        if (bootState == BOOT_RUNNING) { bootState = BOOT_FADE; bootFadeStart = millis(); }
        continue;
      }
      if (token.equalsIgnoreCase("HWREV")) { Serial.print("HWREV:"); Serial.print(hwRevision); Serial.println(";"); continue; }
      if (token.equalsIgnoreCase("REQ")) { forceSendNext = true; continue; }
      continue;
    }

    int colon = token.indexOf(':');
    if (colon < 0) continue;
    String key = token.substring(0, colon);
    String val = token.substring(colon + 1);
    key.trim(); val.trim();

    if (key.equalsIgnoreCase("LED1")) {
      uint8_t blMask = (hwRevision == 1) ? BACKLIGHT_MASK1_REV1 : BACKLIGHT_MASK1_REV2;
      setLEDState(parseBin8(val) & ~blMask, desiredLedAnnun, desiredLed3, desiredBlLevel, desiredAnLevel);
    }
    else if (key.equalsIgnoreCase("LED2")) setLEDState(desiredLedBacklight, parseBin8(val), desiredLed3, desiredBlLevel, desiredAnLevel);
    else if (key.equalsIgnoreCase("LED3")) setLEDState(desiredLedBacklight, desiredLedAnnun, parseBin8(val), desiredBlLevel, desiredAnLevel);
    else if (key.equalsIgnoreCase("BL")) { BL_LEVEL = constrain(val.toInt(),0,255); setLEDState(desiredLedBacklight, desiredLedAnnun, desiredLed3, BL_LEVEL, desiredAnLevel); }
    else if (key.equalsIgnoreCase("AN")) { AN_LEVEL = constrain(val.toInt(),0,255); setLEDState(desiredLedBacklight, desiredLedAnnun, desiredLed3, desiredBlLevel, AN_LEVEL); }
    else if (key.equalsIgnoreCase("DISP_BL")) { DISP_BL_LEVEL = constrain(val.toInt(),0,15); }
    else if (key.equalsIgnoreCase("SMO_THR")) { SMO_THRESHOLD = constrain(val.toInt(), 1, 100); }
    else if (key.equalsIgnoreCase("SMO_SAM")) { SMO_SAMPLES = constrain(val.toInt(), 1, 8); }
    else if (key.equalsIgnoreCase("SMO_DLY")) { SMO_DELAY_US = constrain(val.toInt(), 50, 1000); }
    else if (key.equalsIgnoreCase("HWREVSET")) {
      int comma = val.indexOf(',');
      if (comma > 0) {
        String revStr = val.substring(0, comma);
        String pwdStr = val.substring(comma + 1);
        revStr.trim();
        pwdStr.trim();
        int newRev = revStr.toInt();
        if (pwdStr.equals(HWREV_PASSWORD) && isValidHwRevision(newRev)) {
          saveHwRevisionToEEPROM((uint8_t)newRev);
          Serial.print("HWREVSET:OK;HWREV:"); Serial.print(hwRevision); Serial.println(";");
          applyLEDOutputs();
        } else {
          Serial.println("HWREVSET:ERR;");
        }
      } else {
        Serial.println("HWREVSET:ERR;");
      }
    }
    else if (key.equalsIgnoreCase("REQ")) forceSendNext = true;
    else if (key.equalsIgnoreCase("VER")) sendIdentAndState();
    // --- SET commands (case-insensitive key check) ---
    else if (key.equalsIgnoreCase("SET") || key.startsWith("SET ")) {
      // token = "SET SN:D27B18BB" → nach "SET " = "SN:D27B18BB"
      // Fallback: falls key nur "SET" ist (Doppelpunkt-Split-Issue), val = "SN:D27B18BB"
      String rest;
      if (key.equalsIgnoreCase("SET")) {
        rest = val;  // val enthält bereits "SN:D27B18BB"
      } else {
        rest = token.substring(token.indexOf(' ') + 1); // "SN:D27B18BB"
      }
      rest.trim();
      int colonPos = rest.indexOf(':');
      String cmd = (colonPos >= 0) ? rest.substring(0, colonPos) : rest;
      String arg = (colonPos >= 0) ? rest.substring(colonPos + 1) : "";
      cmd.toUpperCase(); arg.trim();
      
      if (cmd == "ENA") {
        if (arg == SETTINGS_PIN) { settingsEnabled = true; Serial.println("SET ENA> ;"); }
        else { Serial.println("SET ENA:FAIL ;"); }
      }
      else if (cmd == "FW" && settingsEnabled) {
        int dotPos = arg.indexOf('.');
        if (dotPos > 0 && dotPos < arg.length() - 1) {
          int major = arg.substring(0, dotPos).toInt();
          int minor = arg.substring(dotPos + 1).toInt();
          if (major >= 1 && major <= 9 && minor >= 0 && minor <= 9) {
            snprintf(CFG_PCB_VERSION, 9, "PCb %d.%d", major, minor);
            Serial.println("SET FW:OK ;");
          } else Serial.println("SET FW:RANGE ;");
        } else Serial.println("SET FW:FORMAT ;");
      }
      else if (cmd == "ACID" && settingsEnabled) {
        if (arg.length() >= 3 && arg.length() <= 8 && arg.indexOf('-') == 1 && isAlpha(arg.charAt(0))) {
          bool valid = true;
          for (int i = 2; i < arg.length(); i++) if (!isAlpha(arg.charAt(i))) { valid = false; break; }
          if (valid) { strncpy(CFG_AIRCRAFT_REG, arg.c_str(), 8); CFG_AIRCRAFT_REG[8] = 0; Serial.println("SET ACID:OK ;"); }
          else Serial.println("SET ACID:FORMAT ;");
        } else Serial.println("SET ACID:FORMAT ;");
      }
      else if (cmd == "SN" && settingsEnabled) {
        if (arg.length() == 8) {
          bool valid = true;
          for (int i = 0; i < 8 && valid; i++) {
            char c = arg.charAt(i);
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) valid = false;
          }
          if (valid) {
            arg.toUpperCase();
            for (int i = 0; i < 8; i++) CFG_SERIAL_NUMBER[i] = arg.charAt(i);
            CFG_SERIAL_NUMBER[8] = 0;
            Serial.print("SET SN:OK:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
          } else Serial.println("SET SN:FORMAT ;");
        } else Serial.println("SET SN:FORMAT ;");
      }
      else if (cmd == "EXIT") { settingsEnabled = false; Serial.println("SET EXIT:OK ;"); }
      else if (cmd == "WRI" && settingsEnabled) {
        if (arg.equalsIgnoreCase("YES")) { saveHWInfo(); settingsEnabled = false; Serial.println("SET WRI:OK ;"); triggerSoftwareReset(); }
        else Serial.println("SET WRI:FORMAT ;");
      }
      else if (cmd == "WRITE") {
        if (settingsEnabled) { saveHWInfo(); settingsEnabled = false; Serial.println("WRITE:OK ;"); }
        else Serial.println("WRITE:LOCKED ;");
      }
    }
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
        // Boot-Fade bei Host-Connect starten
        if (bootState == BOOT_RUNNING) {
          bootState = BOOT_FADE;
          bootFadeStart = millis();
        }
        pauseUntil = millis() + 200;
        continue;
      }
      if (tokenUp == "HWREV") {
        Serial.print("HWREV:"); Serial.print(hwRevision); Serial.println(";");
        continue;
      }
      if (tokenUp == "REQ") {
        // Full status on next loop — fresh MUX/IN values are read before the send.
        forceSendNext = true;
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
// Full status frame: all 16 MUX + IN1 + IN2.
// Used on startup (forceSendNext) and REQ — keeps the SIM fully in sync with the panel.
// After sending, the change-detection state is synced so no duplicate change frames follow.
void sendFullStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;
  for (int i=0;i<16;i++){
    Serial.print("MUX");Serial.print(i);Serial.print(":");Serial.print(decPad4(muxVals[i]));Serial.print(";");
    lastMuxVals[i] = muxVals[i];
  }
  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";IN2:"); Serial.println(bin8(inputState2));
  lastInputState1 = inputState1; lastInputState2 = inputState2;
  lastSendTs = now; forceSendNext = false;
}

// Change-only frame: sends only the MUX channel(s) that moved >= SMO_THRESHOLD.
// Potentiometer rotation now pushes only the changed channel — no full-frame chatter.
// lastMuxVals is updated only when the frame is actually sent (throttle-safe: no lost changes).
void sendMuxChanges() {
  unsigned long now = millis();
  if ((now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;
  bool any = false;
  for (int i=0;i<16;i++){
    if(abs(muxVals[i]-lastMuxVals[i])>=SMO_THRESHOLD){
      Serial.print("MUX");Serial.print(i);Serial.print(":");Serial.print(decPad4(muxVals[i]));Serial.print(";");
      lastMuxVals[i] = muxVals[i];
      any = true;
    }
  }
  if(any){ Serial.println(); lastSendTs = now; }
}

// Change-only frame for buttons: IN1+IN2 are ALWAYS sent together
// (the Python parser only emits inputs_update when both registers arrive in the same frame).
// Returns true when actually sent — caller updates its "last" state only then,
// so a throttled button press is re-sent on the next loop instead of being lost.
bool sendInputChanges() {
  unsigned long now = millis();
  if ((now - lastSendTs) < SEND_MIN_INTERVAL_MS) return false;
  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";IN2:"); Serial.println(bin8(inputState2));
  lastSendTs = now;
  return true;
}



// --- Boot lightshow (non-blocking) ---
// Runs as a state machine inside loop(): a single LED chases through the free
// registers (LED2 + LED3) while the backlight group breathes with BL.
// Serial stays fully live during the show — the SIM can connect right away.
void bootShowUpdate() {
  unsigned long elapsed = millis() - bootShowStart;
  const int chaserBits = (hwRevision >= 2) ? 16 : 8;      // LED2 (+LED3 on Rev2)
  const unsigned long phaseLen = (unsigned long)chaserBits * BOOT_SHOW_STEP_MS;
  const unsigned long blinkOn  = 2 * phaseLen;            // all on (short)
  const unsigned long blinkOff = blinkOn + 120;            // all off (short)
  const unsigned long totalMs  = blinkOff + 120;

  uint8_t led2 = 0;
  uint8_t led3 = 0;
  uint8_t bl   = 0;

  if (elapsed < phaseLen) {
    // Chaser forward: bit wanders 0..n-1
    int pos = elapsed / BOOT_SHOW_STEP_MS;
    if (pos < 8) led2 = (uint8_t)(1 << (7 - pos));         // MSB-first like bin8()
    else         led3 = (uint8_t)(1 << (15 - pos));
    bl = (uint8_t)(40 + (elapsed % BOOT_SHOW_STEP_MS) * 200 / BOOT_SHOW_STEP_MS);  // breathe up 40..240
  } else if (elapsed < 2 * phaseLen) {
    // Chaser backward
    unsigned long t = elapsed - phaseLen;
    int pos = chaserBits - 1 - (int)(t / BOOT_SHOW_STEP_MS);
    if (pos < 8) led2 = (uint8_t)(1 << (7 - pos));
    else         led3 = (uint8_t)(1 << (15 - pos));
    bl = (uint8_t)(240 - (t % BOOT_SHOW_STEP_MS) * 200 / BOOT_SHOW_STEP_MS);        // breathe down 240..40
  } else if (elapsed < blinkOn + 120) {
    // All on (short blink)
    led2 = 0xFF; led3 = 0xFF; bl = 200;
  } else if (elapsed < totalMs) {
    // All off
    led2 = 0x00; led3 = 0x00; bl = 0;
  } else {
    // Show finished → normal operation
    bootState = BOOT_RUNNING;
  }

  desiredLedBacklight = 0x00;   // LED1 free bits (PA_SEL) stay off during show
  desiredLedAnnun = led2;
  desiredLed3 = led3;
  desiredBlLevel = bl;
  desiredAnLevel = 0;
  applyLEDOutputs();
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  loadHwRevisionFromEEPROM();
  loadHWInfo();
  for(int i=0;i<4;i++) pinMode(muxSelectPins[i], OUTPUT);
  pinMode(muxOutputPin, INPUT);
  pinMode(inputDataPin, INPUT); pinMode(inputClockPin, OUTPUT); pinMode(inputLatchPin, OUTPUT);
  pinMode(ledLatchPin, OUTPUT); pinMode(ledClockPin, OUTPUT); pinMode(ledDataPin, OUTPUT);
  pinMode(backlightPWM, OUTPUT); pinMode(annunPWM, OUTPUT);
  analogWrite(backlightPWM,255); analogWrite(annunPWM,0);  // Invertierte PWM: 255=AUS (kein Backlight beim Boot)
  for(int i=0;i<16;i++){muxVals[i]=0; lastMuxVals[i]=-9999; muxBufferIdx[i]=0;}
  lastInputState1=0xFF; lastInputState2=0xFF;
  // Boot-Lightshow beim Power-On (non-blocking, läuft im loop())
  bootState = BOOT_SHOW;
  bootShowStart = millis();
  desiredBlLevel = 0;
  applyLEDOutputs();
  sendIdentAndState();
  identSentOnStart = true;
  forceSendNext = true;
}

// --- Main Loop ---
void loop() {
  unsigned long now = millis();
  if (now < pauseUntil) return;

  // ── Boot lightshow (non-blocking; serial & status stay LIVE) ──
  if (bootState == BOOT_SHOW) {
    bootShowUpdate();
  }

  // ── Boot backlight welcome pulse (fade-IN + fade-OUT, non-blocking) ──
  if (bootState == BOOT_FADE) {
    unsigned long elapsed = now - bootFadeStart;
    unsigned long totalMs = BOOT_FADE_DURATION_MS * 2;  // in + out
    if (elapsed >= totalMs) {
      desiredBlLevel = 0;
      applyLEDOutputs();
      bootState = BOOT_RUNNING;
    } else if (elapsed < BOOT_FADE_DURATION_MS) {
      // Phase 1: fade IN  0 → 255
      desiredBlLevel = (uint8_t)((unsigned long)255 * elapsed / BOOT_FADE_DURATION_MS);
      applyLEDOutputs();
    } else {
      // Phase 2: fade OUT  255 → 0
      unsigned long outElapsed = elapsed - BOOT_FADE_DURATION_MS;
      desiredBlLevel = 255 - (uint8_t)((unsigned long)255 * outElapsed / BOOT_FADE_DURATION_MS);
      applyLEDOutputs();
    }
    return;  // Skip main loop during fade
  }

  processSerialTokensFromHost();
  if(now - lastLoopTs < LOOP_INTERVAL_MS) return; lastLoopTs=now;

  // --- Read MUX ---
  for(int i=0;i<16;i++){
    int rawVal = readMuxChannelRaw(i);
    muxVals[i] = getMuxSmoothed(i, rawVal);
  }

  // --- Read ShiftRegisters (HC165) ---
  readShiftRegisters(inputState1,inputState2);

  // --- Full status on request (REQ / startup sync) ---
  // Sends ALL 16 MUX + IN1 + IN2 so the SIM is fully in sync with the panel.
  if(forceSendNext){
    sendFullStatus();
  }

  // --- Button Changes (change-only frame: IN1+IN2 together) ---
  if(inputState1 != lastInputState1 || inputState2 != lastInputState2){
    if(now - lastDebounceTs >= DEBOUNCE_MS){
      lastDebounceTs = now;
      // Only accept the change as "sent" when the frame actually went out —
      // otherwise a throttled press is retried on the next loop, not lost.
      if(sendInputChanges()){
        lastInputState1 = inputState1;
        lastInputState2 = inputState2;
      }
    }
  }

  // --- MUX changes (change-only frame, using SMO_THRESHOLD parameter) ---
  sendMuxChanges();

  // --- DIAG Combo ---
  bool comboNow = (inputState1==0b01000001)&&(inputState2==0b00001000);
  if(comboNow && comboStage==0){ comboStage=1; comboStartTime=now; diagActive=true; Serial.println("DIAG START"); setLEDState(0xFF,0xFF,0xFF,200,200,true);}
  if(comboStage==1 && (now-comboStartTime)>=15000){ comboStage=0; diagActive=false; Serial.println("DIAG END"); setLEDState(0x00,0x00,0x00,0,0,true);}
}
