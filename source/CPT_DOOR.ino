// CPT_DOOR Control Script, (w) 2026 M. Quatember
// Captain Door Panel – LED Driver Backlight, direct LEDs, 6 switches, 3 potentiometers

#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Direct LEDs (active HIGH)
const int PIN_DOOR_FLT = 2;   // DOOR_FLT LED
const int PIN_DOOR_OPEN = 3;  // DOOR OPEN LED

// Switches (INPUT_PULLUP, active LOW)
const int PIN_LOCK       = 4;   // LOCK Switch
const int PIN_UNLOCK     = 5;   // Unlock Switch
const int PIN_VIDEO      = 7;   // Video Switch
const int PIN_DFDR_EVT   = 8;   // DFDR_EVT Switch
const int PIN_PNL_DOOR   = 10;  // PNL-Door Switch
const int PIN_ADIRS_PRINT = 21; // ADIRS_PRINT Switch

// PWM Backlight
const int PIN_BACKLIGHT_PWM = 9;

// LED Driver (5916 Konstantstromquelle) – Backlight LEDs
const int PIN_LED_LATCH = 14;
const int PIN_LED_CLOCK = 15;
const int PIN_LED_DATA  = 16;

// Potentiometers (analog)
const int PIN_POT_MIP = A0;  // FLT_LGHT_MIP
const int PIN_POT_INT = A1;  // INTEG LIGHT
const int PIN_POT_PED = A2;  // FLT_LGHT_PED

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "CPT_DOOR, v1.1 MAQ";
const char  FW_VERSION[] = "1.1";
const char  PANEL_SN_PREFIX[] = "DOOR-";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// Serial Number
char CFG_SERIAL_NUMBER[10] = "";

// Config variables
char CFG_AIRCRAFT_REG[9] = "D-A320";
char CFG_PCB_VERSION[9] = "PCB 1.0";
bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";

// Boot state
enum BootState { BOOT_INIT, BOOT_FADE, BOOT_RUNNING };
BootState bootState = BOOT_INIT;
unsigned long bootFadeStart = 0;
const unsigned long BOOT_FADE_DURATION_MS = 1500;

// ============================================================================
// LED STATE
// ============================================================================
// Backlight mask: which LED1 bits are controlled by BL/BLT instead of LED1 command
// CPT_DOOR: bits 0-5 = backlight (6 LEDs), bits 6-7 = free (direct LED1 control)
const uint8_t BACKLIGHT_MASK = 0b00111111;  // bits 0-5 → backlight

uint8_t desiredLed1 = 0x00;   // LED1 free bits only (non-backlight, via shift register)
uint8_t desiredLed2 = 0x00;   // LED2 = Direct LEDs (bit0=DOOR_FLT, bit1=DOOR_OPEN)
uint8_t hwLed1 = 0x00;
uint8_t hwLed2 = 0x00;
uint8_t desiredBlLevel = 0;
uint8_t hwBlLevel = 0;

bool diagActive = false;  // Set true during DIAG to bypass backlight mask

// ============================================================================
// INPUT / POTI STATE
// ============================================================================
uint8_t inputState1 = 0x00;
uint8_t lastInputState1 = 0x00;
int potMipVal = 0;
int potIntVal = 0;
int potPedVal = 0;
int lastPotMipVal = 0;
int lastPotIntVal = 0;
int lastPotPedVal = 0;

// Smoothing parameters
int SMO_THRESHOLD = 10;
int SMO_SAMPLES = 4;
int SMO_DELAY_US = 300;

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;
unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
unsigned long lastDebounceTs = 0;
const unsigned long DEBOUNCE_MS = 12;
bool forceSendNext = false;

// ============================================================================
// SERIAL BUFFER
// ============================================================================
String serialAccum = "";

// ============================================================================
// HELPERS
// ============================================================================
String bin8(uint8_t b) {
  String s = "";
  for (int i = 7; i >= 0; i--) s += ((b & (1 << i)) ? '1' : '0');
  return s;
}

uint8_t parseBin8(const String &s) {
  uint8_t v = 0;
  for (int i = 0; i < s.length(); i++) {
    if (s.charAt(i) == '0' || s.charAt(i) == '1') v = (v << 1) | (s.charAt(i) == '1' ? 1 : 0);
  }
  return v;
}

// ============================================================================
// LED DRIVER (74HC595)
// ============================================================================
void shiftOutLEDs(uint8_t bits) {
  digitalWrite(PIN_LED_LATCH, LOW);
  shiftOut(PIN_LED_DATA, PIN_LED_CLOCK, MSBFIRST, bits);
  digitalWrite(PIN_LED_LATCH, HIGH);
}

void applyLEDOutputs() {
  // Combine free bits (from LED1) + backlight bits (from BL)
  // In DIAG mode, bypass mask — direct full 8-bit control
  uint8_t backlightBits = (diagActive || desiredBlLevel > 0) ? BACKLIGHT_MASK : 0x00;
  uint8_t freeBits = diagActive ? desiredLed1 : (desiredLed1 & ~BACKLIGHT_MASK);
  uint8_t output = freeBits | backlightBits;

  shiftOutLEDs(output);
  digitalWrite(PIN_DOOR_FLT, (desiredLed2 & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_DOOR_OPEN, (desiredLed2 & 0x02) ? HIGH : LOW);
  // Inverted PWM: BL:0 = off, BL:255 = full brightness
  analogWrite(PIN_BACKLIGHT_PWM, 255 - desiredBlLevel);
  hwLed1 = output;
  hwLed2 = desiredLed2;
  hwBlLevel = desiredBlLevel;
}

// ============================================================================
// INPUT READING
// ============================================================================
void readInputs() {
  inputState1 = 0x00;
  if (!digitalRead(PIN_LOCK))       inputState1 |= (1 << 0);
  if (!digitalRead(PIN_UNLOCK))     inputState1 |= (1 << 1);
  if (!digitalRead(PIN_VIDEO))      inputState1 |= (1 << 2);
  if (!digitalRead(PIN_DFDR_EVT))   inputState1 |= (1 << 3);
  if (!digitalRead(PIN_PNL_DOOR))   inputState1 |= (1 << 4);
  if (!digitalRead(PIN_ADIRS_PRINT)) inputState1 |= (1 << 5);
}

void readPotis() {
  potMipVal = analogRead(PIN_POT_MIP);
  potIntVal = analogRead(PIN_POT_INT);
  potPedVal = analogRead(PIN_POT_PED);
}

// ============================================================================
// EEPROM config v3
// ============================================================================
const int EEPROM_BASE_ADDR = 0;
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 3;

void generateSerialNumber(char* out) {
  randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + micros());
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
  for(int i=0;i<8;i++){regbuf[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+3+i);pcbbuf[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+11+i);}
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

// ============================================================================
// SERIAL IDENT / STATUS
// ============================================================================
void sendIdentAndState() {
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
  if (CFG_SERIAL_NUMBER[0] != 0) {
    Serial.print(", SN:"); Serial.print(PANEL_SN_PREFIX); Serial.print(CFG_SERIAL_NUMBER);
  }
  Serial.print(";STATE:RUNNING;");
  Serial.print("REG:"); Serial.print(CFG_AIRCRAFT_REG); Serial.print(";");
  Serial.println();
}

void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;

  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";");
  Serial.print("A0:"); Serial.print(potMipVal); Serial.print(";");
  Serial.print("A1:"); Serial.print(potIntVal); Serial.print(";");
  Serial.print("A2:"); Serial.print(potPedVal); Serial.print(";");
  Serial.println();

  lastSendTs = now;
  forceSendNext = false;
}

// ============================================================================
// SERIAL COMMAND PARSING
// ============================================================================
void processIncomingLine(const String &line) {
  int start = 0;
  while (true) {
    int sep = line.indexOf(';', start);
    if (sep < 0) break;
    String token = line.substring(start, sep); start = sep + 1;
    token.trim();
    if (token.length() == 0) continue;

    if (token.indexOf(':') < 0) {
      String up = token; up.toUpperCase();
      if (up == "VER" || up == "VERSION") { sendIdentAndState(); continue; }
      if (up == "REQ") { forceSendNext = true; continue; }
      if (up == "RESET") { Serial.println("RESET:OK ;"); triggerSoftwareReset(); continue; }
      continue;
    }

    int colon = token.indexOf(':');
    String key = token.substring(0, colon); key.trim();
    String val = token.substring(colon + 1); val.trim();

    if (key.equalsIgnoreCase("LED1")) {
      // Only apply free (non-backlight) bits — backlight bits come from BL
      desiredLed1 = parseBin8(val) & ~BACKLIGHT_MASK;
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("LED2")) {
      desiredLed2 = parseBin8(val);
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("BL")) {
      desiredBlLevel = constrain(val.toInt(), 0, 255);
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("SMO_THR")) { SMO_THRESHOLD = constrain(val.toInt(), 1, 100); }
    else if (key.equalsIgnoreCase("SMO_SAM")) { SMO_SAMPLES = constrain(val.toInt(), 1, 8); }
    else if (key.equalsIgnoreCase("SMO_DLY")) { SMO_DELAY_US = constrain(val.toInt(), 50, 1000); }
    else if (key.equalsIgnoreCase("REQ")) { forceSendNext = true; }
    else if (key.equalsIgnoreCase("VER")) { sendIdentAndState(); }
    // --- SET commands (case-insensitive) ---
    else if (key.equalsIgnoreCase("SET") || key.startsWith("SET ")) {
      String rest;
      if (key.equalsIgnoreCase("SET")) { rest = val; }
      else { rest = token.substring(token.indexOf(' ') + 1); }
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

      String tokenUp = token; tokenUp.toUpperCase();

      if (tokenUp == "VER" || tokenUp == "VERSION") {
        sendIdentAndState();
        identSentOnStart = true;
        pauseUntil = millis() + 200;
        continue;
      }
      if (tokenUp == "REQ") { sendStatus(); continue; }
      if (tokenUp == "RESET") {
        Serial.println("RESET:OK ;");
        triggerSoftwareReset();
        continue;
      }
      if (token.indexOf(':') >= 0) {
        processIncomingLine(token + ";");
        continue;
      }
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  wdt_disable();
  Serial.begin(115200);

  // LED driver
  pinMode(PIN_LED_LATCH, OUTPUT);
  pinMode(PIN_LED_CLOCK, OUTPUT);
  pinMode(PIN_LED_DATA, OUTPUT);

  // Direct LEDs
  pinMode(PIN_DOOR_FLT, OUTPUT);
  pinMode(PIN_DOOR_OPEN, OUTPUT);

  // PWM
  pinMode(PIN_BACKLIGHT_PWM, OUTPUT);

  // Switches
  pinMode(PIN_LOCK, INPUT_PULLUP);
  pinMode(PIN_UNLOCK, INPUT_PULLUP);
  pinMode(PIN_VIDEO, INPUT_PULLUP);
  pinMode(PIN_DFDR_EVT, INPUT_PULLUP);
  pinMode(PIN_PNL_DOOR, INPUT_PULLUP);
  pinMode(PIN_ADIRS_PRINT, INPUT_PULLUP);

  // Initialize outputs (all off)
  applyLEDOutputs();

  // Load config
  loadHWInfo();

  // Boot fade-in (non-blocking backlight)
  bootState = BOOT_FADE;
  bootFadeStart = millis();
  desiredBlLevel = 0;
  applyLEDOutputs();

  // IDENT sofort senden – kein delay, Panel muss ab Boot für Scans bereit sein
  sendIdentAndState();
  identSentOnStart = true;
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

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

  if (now < pauseUntil) return;
  if (now - lastLoopTs < LOOP_INTERVAL_MS) return;
  lastLoopTs = now;

  // Serial processing
  processSerialTokensFromHost();

  // Read inputs
  readInputs();
  readPotis();

  // Check for input changes (debounced)
  if (inputState1 != lastInputState1) {
    if (now - lastDebounceTs >= DEBOUNCE_MS) {
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      sendStatus();
    }
  }

  // Check for potentiometer changes
  bool potChanged = false;
  if (abs(potMipVal - lastPotMipVal) >= SMO_THRESHOLD) { lastPotMipVal = potMipVal; potChanged = true; }
  if (abs(potIntVal - lastPotIntVal) >= SMO_THRESHOLD) { lastPotIntVal = potIntVal; potChanged = true; }
  if (abs(potPedVal - lastPotPedVal) >= SMO_THRESHOLD) { lastPotPedVal = potPedVal; potChanged = true; }

  if (potChanged || forceSendNext) {
    sendStatus();
  }
}
