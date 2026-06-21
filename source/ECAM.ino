// ECAM Control Script, (w) 2026 M. Quatember
// Angelehnt an ACP-Firmware: Glättung, Entprellung, EEPROM/HWREV, DIAG, Boot-Verhalten
// Optimierte Serial-String-Verarbeitung (char-Puffer statt Arduino String)

#include <EEPROM.h>
#include <avr/wdt.h>

// --- Pins (ECAM) ---
const int ledLatchPin   = 2;
const int ledClockPin   = 3;
const int ledDataPin    = 4;

const int annunPWM      = 5;   // 0-255
const int backlightPWM  = 6;   // 0-255

const int inputClockPin = 15;
const int inputLatchPin = 14;
const int inputDataPin  = 16;

const int pot1Pin       = A9;
const int pot2Pin       = A10;

// --- Panel identification ---
const char* PANEL_IDENT = "ECAM, v1.1 MAQ";
const char  FW_VERSION[] = "1.1";
const char  PANEL_SN_PREFIX[] = "ECAM-";
bool identSentOnStart   = false;
unsigned long pauseUntil = 0;

// Serial Number
char CFG_SERIAL_NUMBER[10] = "";

// Config variables
char CFG_AIRCRAFT_REG[9] = "D-A320";
char CFG_PCB_VERSION[9] = "PCB 1.0";
bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";

// --- Hardware revision ---
const uint8_t HWREV_MIN = 1;
const uint8_t HWREV_MAX = 2;
const int EEPROM_ADDR_HWREV = 0;
const char* HWREV_PASSWORD = "ACP-HW-SET";
uint8_t hwRevision = 1;

// --- LED state ---
uint8_t desiredLed1 = 0x00;
uint8_t desiredLed2 = 0x00;
uint8_t desiredLed3 = 0x00;
uint8_t desiredLed4 = 0x00;
uint8_t desiredBlLevel = 0;
uint8_t desiredAnLevel = 0;

// --- Universal Parameters ---
int SMO_THRESHOLD = 10;
int SMO_SAMPLES   = 4;
int SMO_DELAY_US  = 300;

int BL_LEVEL      = 0;
int AN_LEVEL      = 0;
int DISP_BL_LEVEL = 15;

int POT_DEADBAND  = 10;

// --- Timing ---
unsigned long lastLoopTs = 0;
unsigned long lastSendTs = 0;
unsigned long lastDebounceTs = 0;

const unsigned long LOOP_INTERVAL_MS = 10;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
const unsigned long DEBOUNCE_MS = 12;

// --- Pots ---
int pot1Buffer[8];
int pot2Buffer[8];
int pot1Idx = 0;
int pot2Idx = 0;
int pot1Val = 0;
int pot2Val = 0;
int lastPot1Val = -9999;
int lastPot2Val = -9999;

// --- Inputs ---
uint8_t inputState1 = 0xFF;
uint8_t inputState2 = 0xFF;
uint8_t inputState3 = 0xFF;
uint8_t inputState4 = 0xFF;

uint8_t lastInputState1 = 0xFF;
uint8_t lastInputState2 = 0xFF;
uint8_t lastInputState3 = 0xFF;
uint8_t lastInputState4 = 0xFF;

// --- DIAG ---
unsigned long comboStartTime = 0;
int comboStage = 0;
bool forceSendNext = false;

// --- Serial Parser ---
const int SERIAL_BUF_SIZE = 128;
char serialBuf[SERIAL_BUF_SIZE];
int serialBufPos = 0;

// ============================================================================
// Helper Functions
// ============================================================================

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

uint8_t parseBin8(const char* s) {
  uint8_t v = 0;
  for (int i = 0; s[i] != '\0'; i++) {
    char c = s[i];
    if (c == '0' || c == '1') v = (v << 1) | (c == '1' ? 1 : 0);
  }
  return v;
}

// ============================================================================
// LED Output
// ============================================================================

void shiftOutLEDs(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4) {
  digitalWrite(ledLatchPin, LOW);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, l4);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, l3);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, l2);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, l1);
  digitalWrite(ledLatchPin, HIGH);
}

void applyLEDOutputs() {
  shiftOutLEDs(desiredLed1, desiredLed2, desiredLed3, desiredLed4);
  analogWrite(backlightPWM, desiredBlLevel);
  analogWrite(annunPWM, desiredAnLevel);
}

void setLEDState(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4,
                 uint8_t bl, uint8_t an, bool allowDuringCombo = false)
{
  if (comboStage != 0 && !allowDuringCombo) return;

  desiredLed1 = l1;
  desiredLed2 = l2;
  desiredLed3 = l3;
  desiredLed4 = l4;
  desiredBlLevel = bl;
  desiredAnLevel = an;

  applyLEDOutputs();
}

// ============================================================================
// EEPROM
// ============================================================================

bool isValidHwRevision(int rev) {
  return rev >= HWREV_MIN && rev <= HWREV_MAX;
}

void loadHwRevisionFromEEPROM() {
  uint8_t rev = EEPROM.read(EEPROM_ADDR_HWREV);
  if (isValidHwRevision(rev)) hwRevision = rev;
  else {
    hwRevision = 1;
    EEPROM.update(EEPROM_ADDR_HWREV, hwRevision);
  }
}

// --- EEPROM config v3 ---
const int EEPROM_BASE_ADDR = 10;
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 3;

void generateSerialNumber(char* out) {
  randomSeed(analogRead(A9) + analogRead(A10) + micros());
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
// Shift Register Input
// ============================================================================

void readShiftRegisters(uint8_t &in1, uint8_t &in2, uint8_t &in3, uint8_t &in4) {
  digitalWrite(inputLatchPin, LOW);
  delayMicroseconds(20);
  digitalWrite(inputLatchPin, HIGH);
  delayMicroseconds(20);

  in1 = ~shiftIn(inputDataPin, inputClockPin, MSBFIRST);
  in2 = ~shiftIn(inputDataPin, inputClockPin, MSBFIRST);
  in3 = ~shiftIn(inputDataPin, inputClockPin, MSBFIRST);
  in4 = ~shiftIn(inputDataPin, inputClockPin, MSBFIRST);

  digitalWrite(inputLatchPin, LOW);
  delayMicroseconds(20);
  digitalWrite(inputLatchPin, HIGH);
  delayMicroseconds(20);

  for (int i = 0; i < 32; i++) {
    digitalWrite(inputClockPin, LOW);
    delayMicroseconds(5);
    digitalWrite(inputClockPin, HIGH);
    delayMicroseconds(5);
  }
}

// ============================================================================
// Serial IDENT
// ============================================================================

void sendIdentAndState() {
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
  if (CFG_SERIAL_NUMBER[0] != 0) {
    Serial.print(", SN:"); Serial.print(PANEL_SN_PREFIX); Serial.print(CFG_SERIAL_NUMBER);
  }
  Serial.print(";STATE:RUNNING;");
  Serial.print("HWREV:"); Serial.print(hwRevision); Serial.print(";");
  Serial.println();
}

// ============================================================================
// DIAG ROUTINE (Extended)
// ============================================================================

void runDiag() {
  Serial.println("DIAG START");

  // 1) LED WALK (32 LEDs)
  for (int i = 0; i < 32; i++) {
    uint8_t b1 = (i < 8)  ? (1 << (i)) : 0;
    uint8_t b2 = (i >= 8  && i < 16) ? (1 << (i - 8)) : 0;
    uint8_t b3 = (i >= 16 && i < 24) ? (1 << (i - 16)) : 0;
    uint8_t b4 = (i >= 24 && i < 32) ? (1 << (i - 24)) : 0;

    setLEDState(b1, b2, b3, b4, 50, 50, true);
    delay(60);
  }

  // 2) Muster wie beim Start
  setLEDState(0xAA, 0xAA, 0xAA, 0xAA, 200, 200, true); delay(200);
  setLEDState(0x55, 0x55, 0x55, 0x55, 200, 200, true); delay(200);
  setLEDState(0xFF, 0xFF, 0xFF, 0xFF, 200, 200, true); delay(200);

  // 3) Alle AN
  setLEDState(0xFF, 0xFF, 0xFF, 0xFF, 255, 255, true); delay(300);

  // 4) Alles aus
  setLEDState(0x00, 0x00, 0x00, 0x00, 0, 0, true);

  Serial.println("DIAG END");
}

// ============================================================================
// Serial Command Parser
// ============================================================================

void handleKeyValueToken(const char* key, const char* val) {
  if (strcasecmp(key, "LED1") == 0) setLEDState(parseBin8(val), desiredLed2, desiredLed3, desiredLed4, desiredBlLevel, desiredAnLevel);
  else if (strcasecmp(key, "LED2") == 0) setLEDState(desiredLed1, parseBin8(val), desiredLed3, desiredLed4, desiredBlLevel, desiredAnLevel);
  else if (strcasecmp(key, "LED3") == 0) setLEDState(desiredLed1, desiredLed2, parseBin8(val), desiredLed4, desiredBlLevel, desiredAnLevel);
  else if (strcasecmp(key, "LED4") == 0) setLEDState(desiredLed1, desiredLed2, desiredLed3, parseBin8(val), desiredBlLevel, desiredAnLevel);

  else if (strcasecmp(key, "BL") == 0) {
    BL_LEVEL = constrain(atoi(val), 0, 255);
    setLEDState(desiredLed1, desiredLed2, desiredLed3, desiredLed4, BL_LEVEL, desiredAnLevel);
  }
  else if (strcasecmp(key, "AN") == 0) {
    AN_LEVEL = constrain(atoi(val), 0, 255);
    setLEDState(desiredLed1, desiredLed2, desiredLed3, desiredLed4, desiredBlLevel, AN_LEVEL);
  }

  else if (strcasecmp(key, "SMO_THR") == 0) SMO_THRESHOLD = constrain(atoi(val), 1, 100);
  else if (strcasecmp(key, "SMO_SAM") == 0) SMO_SAMPLES = constrain(atoi(val), 1, 8);
  else if (strcasecmp(key, "SMO_DLY") == 0) SMO_DELAY_US = constrain(atoi(val), 50, 1000);
  else if (strcasecmp(key, "POT_DB") == 0) POT_DEADBAND = constrain(atoi(val), 0, 200);

  else if (strcasecmp(key, "DIAG") == 0) runDiag();
  // --- SET commands (char* based) ---
  else if (strncasecmp(key, "SET ", 4) == 0) {
    const char* setCmd = key + 4; while (*setCmd == ' ') setCmd++;
    if (strncasecmp(setCmd, "ENA", 3) == 0) {
      if (strcmp(val, SETTINGS_PIN) == 0) { settingsEnabled = true; Serial.println("SET ENA> ;"); }
      else { Serial.println("SET ENA:FAIL ;"); }
    }
    else if (strncasecmp(setCmd, "FW", 2) == 0 && settingsEnabled) {
      const char* dot = strchr(val, '.');
      if (dot && dot > val && dot < val + strlen(val) - 1) {
        int major = atoi(val); int minor = atoi(dot + 1);
        if (major >= 1 && major <= 9 && minor >= 0 && minor <= 9) {
          snprintf(CFG_PCB_VERSION, 9, "PCb %d.%d", major, minor);
          Serial.println("SET FW:OK ;");
        } else Serial.println("SET FW:RANGE ;");
      } else Serial.println("SET FW:FORMAT ;");
    }
    else if (strncasecmp(setCmd, "ACID", 4) == 0 && settingsEnabled) {
      int slen = strlen(val);
      if (slen >= 3 && slen <= 8 && val[1] == '-' && isAlpha(val[0])) {
        bool valid = true;
        for (int i = 2; i < slen; i++) if (!isAlpha(val[i])) { valid = false; break; }
        if (valid) { strncpy(CFG_AIRCRAFT_REG, val, 8); CFG_AIRCRAFT_REG[8] = 0; Serial.println("SET ACID:OK ;"); }
        else Serial.println("SET ACID:FORMAT ;");
      } else Serial.println("SET ACID:FORMAT ;");
    }
    else if (strncasecmp(setCmd, "SN", 2) == 0 && settingsEnabled) {
      int slen = strlen(val);
      if (slen == 8) {
        bool valid = true;
        for (int i = 0; i < 8 && valid; i++) {
          char c = val[i];
          if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) valid = false;
        }
        if (valid) {
          for (int i = 0; i < 8; i++) CFG_SERIAL_NUMBER[i] = (val[i] >= 'a' && val[i] <= 'f') ? val[i] - 32 : val[i];
          CFG_SERIAL_NUMBER[8] = 0;
          Serial.print("SET SN:OK:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
        } else Serial.println("SET SN:FORMAT ;");
      } else Serial.println("SET SN:FORMAT ;");
    }
    else if (strncasecmp(setCmd, "EXIT", 4) == 0) { settingsEnabled = false; Serial.println("SET EXIT:OK ;"); }
    else if (strncasecmp(setCmd, "WRI", 3) == 0 && settingsEnabled) {
      if (strcasecmp(val, "YES") == 0) { saveHWInfo(); settingsEnabled = false; Serial.println("SET WRI:OK ;"); triggerSoftwareReset(); }
      else Serial.println("SET WRI:FORMAT ;");
    }
    else if (strncasecmp(setCmd, "WRITE", 5) == 0) {
      if (settingsEnabled) { saveHWInfo(); settingsEnabled = false; Serial.println("WRITE:OK ;"); }
      else Serial.println("WRITE:LOCKED ;");
    }
  }

void handleSingleWordToken(const char* token) {
  if (strcasecmp(token, "VER") == 0 || strcasecmp(token, "VERSION") == 0) sendIdentAndState();
  else if (strcasecmp(token, "REQ") == 0) forceSendNext = true;
  else if (strcasecmp(token, "DIAG") == 0) runDiag();
}

void processSerialToken(char* token) {
  while (*token == ' ' || *token == '\t') token++;
  int len = strlen(token);
  while (len > 0 && (token[len - 1] == ' ' || token[len - 1] == '\t')) token[--len] = '\0';
  if (len == 0) return;

  char* colon = strchr(token, ':');
  if (!colon) { handleSingleWordToken(token); return; }

  *colon = '\0';
  char* key = token;
  char* val = colon + 1;

  handleKeyValueToken(key, val);
}

void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') c = ';';

    if (serialBufPos < SERIAL_BUF_SIZE - 1) {
      serialBuf[serialBufPos++] = c;
      serialBuf[serialBufPos] = '\0';
    }

    char* semicolon;
    while ((semicolon = strchr(serialBuf, ';')) != NULL) {
      *semicolon = '\0';
      char token[SERIAL_BUF_SIZE];
      strncpy(token, serialBuf, sizeof(token) - 1);
      token[sizeof(token) - 1] = '\0';

      int remaining = serialBufPos - (semicolon - serialBuf) - 1;
      memmove(serialBuf, semicolon + 1, remaining);
      serialBuf[remaining] = '\0';
      serialBufPos = remaining;

      if (strlen(token) > 0) processSerialToken(token);
    }
  }
}

// ============================================================================
// Status
// ============================================================================

void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;

  Serial.print("POT1:"); Serial.print(decPad4(pot1Val)); Serial.print(";");
  Serial.print("POT2:"); Serial.print(decPad4(pot2Val)); Serial.print(";");

  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";");
  Serial.print("IN2:"); Serial.print(bin8(inputState2)); Serial.print(";");
  Serial.print("IN3:"); Serial.print(bin8(inputState3)); Serial.print(";");
  Serial.print("IN4:"); Serial.println(bin8(inputState4));

  lastSendTs = now;
  forceSendNext = false;
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  loadHwRevisionFromEEPROM();
  loadHWInfo();

  pinMode(ledLatchPin, OUTPUT);
  pinMode(ledClockPin, OUTPUT);
  pinMode(ledDataPin, OUTPUT);

  pinMode(backlightPWM, OUTPUT);
  pinMode(annunPWM, OUTPUT);

  pinMode(inputDataPin, INPUT);
  pinMode(inputClockPin, OUTPUT);
  pinMode(inputLatchPin, OUTPUT);

  analogWrite(backlightPWM, 0);
  analogWrite(annunPWM, 0);

  for (int i = 0; i < 8; i++) {
    pot1Buffer[i] = 0;
    pot2Buffer[i] = 0;
  }

  // Startup Diagnose
  setLEDState(0xAA, 0xAA, 0xAA, 0xAA, 200, 200, true); delay(150);
  setLEDState(0x55, 0x55, 0x55, 0x55, 200, 200, true); delay(150);
  setLEDState(0xFF, 0xFF, 0xFF, 0xFF, 0,   0,   true); delay(150);
  setLEDState(0x00, 0x00, 0x00, 0x00, 0,   0,   true); delay(150);

  sendIdentAndState();
  identSentOnStart = true;
  forceSendNext = true;

  serialBufPos = 0;
  serialBuf[0] = '\0';
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  unsigned long now = millis();
  if (now < pauseUntil) return;

  processSerialInput();

  if (now - lastLoopTs < LOOP_INTERVAL_MS) return;
  lastLoopTs = now;

  // Pots
  pot1Val = addAndSmooth(pot1Buffer, pot1Idx, SMO_SAMPLES, analogRead(pot1Pin));
  pot2Val = addAndSmooth(pot2Buffer, pot2Idx, SMO_SAMPLES, analogRead(pot2Pin));

  // Inputs
  readShiftRegisters(inputState1, inputState2, inputState3, inputState4);

  bool changedInputs =
    (inputState1 != lastInputState1) ||
    (inputState2 != lastInputState2) ||
    (inputState3 != lastInputState3) ||
    (inputState4 != lastInputState4);

  if (changedInputs && (now - lastDebounceTs >= DEBOUNCE_MS)) {
    lastDebounceTs = now;
    lastInputState1 = inputState1;
    lastInputState2 = inputState2;
    lastInputState3 = inputState3;
    lastInputState4 = inputState4;
    sendStatus();
  }

  // Pot change detection
  bool potChanged = false;
  int diff1 = abs(pot1Val - lastPot1Val);
  int diff2 = abs(pot2Val - lastPot2Val);

    if (diff1 >= POT_DEADBAND && diff1 >= SMO_THRESHOLD) {
    lastPot1Val = pot1Val;
    potChanged = true;
  }

  if (diff2 >= POT_DEADBAND && diff2 >= SMO_THRESHOLD) {
    lastPot2Val = pot2Val;
    potChanged = true;
  }

  if (potChanged) {
    sendStatus();
  }

  // --- DIAG Combo (deine Variante) ---
  bool comboNow = (inputState1 == 0b00000000) && (inputState2 == 0b00101001);

  if (comboNow && comboStage == 0) {
    comboStage = 1;
    comboStartTime = now;
    runDiag();   // komplette DIAG-Routine
  }

  // Ende loop()
}
int addAndSmooth(int* buf, int& idx, int samples, int raw) {
  buf[idx] = raw;
  idx = (idx + 1) % samples;

  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += buf[i];
  }
  return (int)(sum / samples);
}
