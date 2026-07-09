// WXR Weather Radar Panel Firmware, (c) 2025-2026 M. Quatember
// Based on RMP architecture — direct digital/analog I/O, no displays, no rotary encoders
// LED driver: single 74HC595 (8-bit), Backlight: PWM on Pin 3

#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Digital Inputs (active-low with internal pullup)
const int PIN_MULTISCAN_MAN  = 2;
const int PIN_MULTISCAN_AUTO = 7;
const int PIN_MAP            = 10;
const int PIN_TURB           = 16;  // A2 on Pro Micro (also usable as digital)
const int PIN_WXT            = 14;  // MISO — WX+T
const int PIN_WX             = 15;  // SCK
const int PIN_SYS1           = 18;  // A0
const int PIN_SYS2           = 19;  // A1
const int PIN_PWS_AUTO       = 20;  // A2 — wait, user assigned 16=TURB and also 20=PWS_AUTO
                                    // Pin 20 = PF5? Actually on Pro Micro:
                                    // Pin 18 = A0, Pin 19 = A1, Pin 20 = A2, Pin 21 = A3
                                    // But Pin 16 = PB2 (MOSI). These are distinct pins.
const int PIN_GCS_AUTO       = 21;  // A3

// Analog Inputs (POTs)
const int PIN_TILT_POT = A8;  // Pin 8  = ADC8
const int PIN_GAIN_POT = A9;  // Pin 9  = ADC9

// LED Driver (TLC5916 constant-current sink, single 8-bit)
// TLC5916 latches on FALLING edge of LE (inverted vs 74HC595)
// OE (pin 13) must be tied LOW for outputs to be active
const int PIN_LED_LATCH = 4;
const int PIN_LED_CLK   = 5;
const int PIN_LED_DATA  = 6;

// Backlight PWM
const int PIN_BACKLIGHT = 3;

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "WXR, v1.1 MAQ";
const char  FW_VERSION[] = "1.1";
const char  PANEL_SN_PREFIX[] = "WXR-";
bool identSentOnStart = false;

// Serial Number – generated on first boot, stored in EEPROM
char CFG_SERIAL_NUMBER[10] = "";  // 8 hex chars + null

// ============================================================================
// LED / BACKLIGHT STATE
// ============================================================================
// Backlight mask: which LED1 bits are controlled by BL/BLT instead of LED1 command
// WXR: bits 0-5 = backlight (6 LEDs), bits 6-7 = free (direct LED1 control)
// Other panels can override this mask (e.g. RMP might use fewer bits)
const uint8_t BACKLIGHT_MASK = 0b00111111;  // bits 0-5 → backlight

uint8_t desiredLedState = 0x00;     // Free bits only (non-backlight), set via LED1
uint8_t hwLedState      = 0x00;     // Last written to shift register
uint8_t desiredBacklight = 0;       // 0=off, 1-255=PWM brightness (inverted on OE)
uint8_t hwBacklight = 0;

// ============================================================================
// INPUT STATE
// ============================================================================
uint8_t inputState1 = 0x00;       // IN1 bits: MULTISCAN_MAN, MULTISCAN_AUTO, MAP, TURB, WX+T, WX, SYS1, SYS2
uint8_t inputState2 = 0x00;       // IN2 bits: PWS_AUTO, GCS_AUTO, 0,0,0,0,0,0
uint8_t lastInputState1 = 0x00;
uint8_t lastInputState2 = 0x00;
uint16_t potTiltRaw = 0;
uint16_t potGainRaw = 0;
uint16_t lastPotTiltRaw = 0;
uint16_t lastPotGainRaw = 0;

// ============================================================================
// CONFIGURATION
// ============================================================================
uint8_t CFG_BUTTON_DEBOUNCE = 12;
char CFG_AIRCRAFT_REG[9] = "D-A320";
char CFG_PCB_VERSION[9] = "PCB 1.0";

// EEPROM storage layout (same as RMP v3)
const int EEPROM_BASE_ADDR = 0;
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 3;
// Layout:
// 0-1: magic (uint16)
// 2: version (uint8)
// 3-10: aircraft_reg[8]
// 11-18: pcb_version[8]
// 19-27: serial_number[9] (8 hex chars + null)
// 28: checksum (uint8)

// Settings PIN
bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";

// ============================================================================
// TIMING / THROTTLES
// ============================================================================
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;

unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;

unsigned long lastDebounceTs = 0;

// ============================================================================
// BOOT / CONNECTION STATE
// ============================================================================
enum BootState { BOOT_INIT, BOOT_RUNNING, BOOT_DIAG };
BootState bootState = BOOT_INIT;
unsigned long bootSequenceStart = 0;
bool bootMessageShown = false;
unsigned long bootMessageStart = 0;
const unsigned long BOOT_MESSAGE_DURATION_MS = 5000;

// Host online state
bool hostOnline = false;

// DIAG state
unsigned long diagStartTime = 0;
int diagStage = 0;
bool diagActive = false;  // Set true during DIAG to bypass backlight mask

// ============================================================================
// RUNTIME BUFFERS
// ============================================================================
char serialAccum[64] = "";
bool forceSendNext = false;

// ============================================================================
// HELPER FUNCTIONS — STRING / BINARY
// ============================================================================
void bin8(char* out, byte b) {
  for (int i = 0; i < 8; i++) out[i] = ((b >> (7 - i)) & 1) ? '1' : '0';
  out[8] = 0;
}

uint8_t parseBin8(const char* s) {
  uint8_t v = 0;
  for (int i = 0; s[i]; i++) {
    if (s[i] == '0' || s[i] == '1') v = (v << 1) | (s[i] == '1' ? 1 : 0);
  }
  return v;
}

static bool strieq(const char* a, const char* b) {
  while (*a && *b) {
    char ca = (*a >= 'a' && *a <= 'z') ? *a - 32 : *a;
    char cb = (*b >= 'a' && *b <= 'z') ? *b - 32 : *b;
    if (ca != cb) return false;
    a++; b++;
  }
  return *a == *b;
}

static bool stripre(const char* a, const char* prefix) {
  while (*prefix) {
    char ca = (*a >= 'a' && *a <= 'z') ? *a - 32 : *a;
    char cb = (*prefix >= 'a' && *prefix <= 'z') ? *prefix - 32 : *prefix;
    if (ca != cb) return false;
    a++; prefix++;
  }
  return true;
}

static int atoi_s(const char* s) {
  int v = 0;
  int sign = 1;
  while (*s == ' ') s++;
  if (*s == '-') { sign = -1; s++; }
  while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
  return v * sign;
}

// ============================================================================
// LED DRIVER (TLC5916 constant-current sink via 74HC595-compatible protocol)
// ============================================================================
void shiftOutLEDs(uint8_t ledBits) {
  digitalWrite(PIN_LED_LATCH, LOW);
  shiftOut(PIN_LED_DATA, PIN_LED_CLK, MSBFIRST, ledBits);
  digitalWrite(PIN_LED_LATCH, HIGH);
}

void applyLEDOutputs() {
  // Combine free bits (from LED1) + backlight bits (from BL/BLT)
  // In DIAG mode, bypass mask — direct full 8-bit control
  uint8_t backlightBits = (diagActive || desiredBacklight > 0) ? BACKLIGHT_MASK : 0x00;
  uint8_t freeBits = diagActive ? desiredLedState : (desiredLedState & ~BACKLIGHT_MASK);
  uint8_t output = freeBits | backlightBits;

  // Always shift out unconditionally (same as RMP/ACP)
  shiftOutLEDs(output);
  hwLedState = output;

  // Backlight PWM on Pin 3 → TLC5916 OE (active LOW)
  // Inverted: BL:0  = PWM 255 = OE HIGH = outputs disabled = backlight OFF
  //           BL:255 = PWM 0   = OE LOW  = outputs enabled  = backlight FULL
  uint8_t pwm = 255 - desiredBacklight;
  analogWrite(PIN_BACKLIGHT, pwm);
  hwBacklight = desiredBacklight;
}

// ============================================================================
// INPUT READING
// ============================================================================
void readAllInputs() {
  // Digital inputs (active-low with pullup → invert so HIGH=pressed)
  inputState1 = 0;
  if (!digitalRead(PIN_MULTISCAN_MAN))  inputState1 |= (1 << 0);
  if (!digitalRead(PIN_MULTISCAN_AUTO)) inputState1 |= (1 << 1);
  if (!digitalRead(PIN_MAP))            inputState1 |= (1 << 2);
  if (!digitalRead(PIN_TURB))           inputState1 |= (1 << 3);
  if (!digitalRead(PIN_WXT))            inputState1 |= (1 << 4);
  if (!digitalRead(PIN_WX))             inputState1 |= (1 << 5);
  if (!digitalRead(PIN_SYS1))           inputState1 |= (1 << 6);
  if (!digitalRead(PIN_SYS2))           inputState1 |= (1 << 7);

  inputState2 = 0;
  if (!digitalRead(PIN_PWS_AUTO))       inputState2 |= (1 << 0);
  if (!digitalRead(PIN_GCS_AUTO))       inputState2 |= (1 << 1);

  // Analog inputs
  potTiltRaw = analogRead(PIN_TILT_POT);
  potGainRaw = analogRead(PIN_GAIN_POT);
}

// ============================================================================
// SOFTWARE RESET
// ============================================================================
void triggerSoftwareReset() {
  Serial.flush();
  delay(50);
  wdt_enable(WDTO_15MS);
  while (true) { /* wait for watchdog */ }
}

// ============================================================================
// SERIAL IDENT / STATE
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

void maybeSendIdentStartup() {
  if (!identSentOnStart) {
    delay(150);
    sendIdentAndState();
    identSentOnStart = true;
  }
}

// ============================================================================
// STATUS SENDING (matches RMP format)
// ============================================================================
void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;

  char b8[9];
  bin8(b8, inputState1); Serial.print("IN1:"); Serial.print(b8); Serial.print(";");
  bin8(b8, inputState2); Serial.print("IN2:"); Serial.print(b8); Serial.print(";");

  if (potTiltRaw != lastPotTiltRaw || potGainRaw != lastPotGainRaw || forceSendNext) {
    Serial.print("POT1:"); Serial.print(potTiltRaw); Serial.print(";");
    Serial.print("POT2:"); Serial.print(potGainRaw); Serial.print(";");
    lastPotTiltRaw = potTiltRaw;
    lastPotGainRaw = potGainRaw;
  }

  Serial.println();
  lastSendTs = now;
  forceSendNext = false;
}

void sendStatusImmediate() {
  // Always read fresh inputs before sending (for REQ etc.)
  readAllInputs();

  char b8[9];
  bin8(b8, inputState1); Serial.print("IN1:"); Serial.print(b8); Serial.print(";");
  bin8(b8, inputState2); Serial.print("IN2:"); Serial.print(b8); Serial.print(";");
  Serial.print("POT1:"); Serial.print(potTiltRaw); Serial.print(";");
  Serial.print("POT2:"); Serial.print(potGainRaw); Serial.print(";");
  Serial.println();

  lastPotTiltRaw = potTiltRaw;
  lastPotGainRaw = potGainRaw;
  lastSendTs = millis();
}

// ============================================================================
// SERIAL COMMAND PARSING (matching RMP protocol)
// ============================================================================
void processIncomingLine(const char* line) {
  char token[32];
  int tlen = 0;
  for (int i = 0; ; i++) {
    char c = line[i];
    if (c == ';' || c == 0) {
      if (tlen == 0) { if (c == 0) break; else continue; }
      token[tlen] = 0;
      tlen = 0;

      // Trim
      int end = strlen(token) - 1;
      while (end >= 0 && token[end] == ' ') token[end--] = 0;
      char* tok = token;
      while (*tok == ' ') tok++;
      if (*tok == 0) { if (c == 0) break; else continue; }

      // Find colon
      char* colon = strchr(tok, ':');
      if (!colon) {
        if (strieq(tok, "VER") || strieq(tok, "VERSION") || strieq(tok, "IDENT")) {
          sendIdentAndState();
        } else if (strieq(tok, "REQ")) {
          sendStatusImmediate();
        } else if (strieq(tok, "DIAG")) {
          if (bootState == BOOT_RUNNING) {
            bootState = BOOT_DIAG;
            diagStartTime = millis();
            diagStage = 0;
            Serial.println("DIAG:START;");
          }
        } else if (stripre(tok, "SET ")) {
          char* sc = tok + 4;
          while (*sc == ' ') sc++;
          if (stripre(sc, "EXIT")) {
            settingsEnabled = false;
            Serial.println("SET EXIT:OK ;");
          } else if (stripre(sc, "WRITE")) {
            if (settingsEnabled) {
              saveHWInfo();
              settingsEnabled = false;
              Serial.println("WRITE:OK ;");
              triggerSoftwareReset();
            } else {
              Serial.println("WRITE:LOCKED ;");
            }
          }
        }
        if (c == 0) break; else continue;
      }

      *colon = 0;
      char* key = tok;
      char* val = colon + 1;
      // Trim key
      int kend = strlen(key) - 1;
      while (kend >= 0 && key[kend] == ' ') key[kend--] = 0;
      char* kp = key;
      while (*kp == ' ') kp++;
      // Trim val
      while (*val == ' ') val++;
      int vend = strlen(val) - 1;
      while (vend >= 0 && val[vend] == ' ') val[vend--] = 0;

      if (strieq(kp, "LED1")) {
        // Only apply free (non-backlight) bits — backlight bits come from BL/BLT
        desiredLedState = parseBin8(val) & ~BACKLIGHT_MASK;
        applyLEDOutputs();
      }
      else if (strieq(kp, "BL") || strieq(kp, "BLT")) {
        // Backlight brightness: 0=off, 1-255=on with PWM dimming
        // Backlight bits in shift register are set/cleared automatically
        desiredBacklight = constrain(atoi_s(val), 0, 255);
        applyLEDOutputs();
      }
      else if (strieq(kp, "STATE")) {
        hostOnline = (strcmp(val, "01") == 0 || strcmp(val, "1") == 0);
      }
      else if (strieq(kp, "REQ")) {
        forceSendNext = true;
      }
      else if (strieq(kp, "VER")) {
        sendIdentAndState();
      }
      else if (strieq(kp, "CFG")) {
        char* cfgPtr = val;
        while (*cfgPtr) {
          while (*cfgPtr == ' ' || *cfgPtr == ';') cfgPtr++;
          if (!*cfgPtr) break;
          char* semi = strchr(cfgPtr, ';');
          if (semi) *semi = 0;

          if (stripre(cfgPtr, "DEB")) {
            CFG_BUTTON_DEBOUNCE = constrain(atoi_s(cfgPtr + 3), 0, 99);
          }
          else if (stripre(cfgPtr, "REG:")) {
            strncpy(CFG_AIRCRAFT_REG, cfgPtr + 4, 8);
            CFG_AIRCRAFT_REG[8] = 0;
          }
          else if (stripre(cfgPtr, "SN:")) {
            strncpy(CFG_SERIAL_NUMBER, cfgPtr + 3, 8);
            CFG_SERIAL_NUMBER[8] = 0;
          }

          if (semi) *semi = ';';
          cfgPtr = semi ? semi + 1 : cfgPtr + strlen(cfgPtr);
        }
      }
      else if (stripre(tok, "SET ")) {
        char* setCmd = tok + 4;
        while (*setCmd == ' ') setCmd++;

        if (stripre(setCmd, "ENA")) {
          if (strcmp(val, SETTINGS_PIN) == 0) {
            settingsEnabled = true;
            Serial.println("SET ENA> ;");
          } else {
            Serial.println("SET ENA:FAIL ;");
          }
        }
        else if (stripre(setCmd, "FW") && settingsEnabled) {
          char* verStr = val;
          while (*verStr == ' ') verStr++;
          char* dot = strchr(verStr, '.');
          if (dot && dot > verStr && dot < verStr + strlen(verStr) - 1) {
            *dot = 0;
            int major = atoi_s(verStr);
            int minor = atoi_s(dot + 1);
            *dot = '.';
            if (major >= 1 && major <= 9 && minor >= 0 && minor <= 9) {
              snprintf(CFG_PCB_VERSION, 9, "PCb %d.%d", major, minor);
              Serial.println("SET FW:OK ;");
            } else {
              Serial.println("SET FW:RANGE ;");
            }
          } else {
            Serial.println("SET FW:FORMAT ;");
          }
        }
        else if (stripre(setCmd, "ACID") && settingsEnabled) {
          char* ident = val;
          while (*ident == ' ') ident++;
          int ilen = strlen(ident);
          if (ilen >= 3 && ilen <= 8) {
            char* dash = strchr(ident, '-');
            if (dash && dash == ident + 1) {
              bool valid = true;
              if (!isAlpha(ident[0])) valid = false;
              for (int i = 2; i < ilen && valid; i++) {
                if (!isAlpha(ident[i])) valid = false;
              }
              if (valid) {
                strncpy(CFG_AIRCRAFT_REG, ident, 8);
                CFG_AIRCRAFT_REG[8] = 0;
                Serial.println("SET ACID:OK ;");
              } else {
                Serial.println("SET ACID:FORMAT ;");
              }
            } else {
              Serial.println("SET ACID:FORMAT ;");
            }
          } else {
            Serial.println("SET ACID:FORMAT ;");
          }
        }
        else if (stripre(setCmd, "EXIT")) {
          settingsEnabled = false;
          Serial.println("SET EXIT:OK ;");
        }
        else if (stripre(setCmd, "WRI") && settingsEnabled) {
          if (strieq(val, "YES")) {
            saveHWInfo();
            settingsEnabled = false;
            Serial.println("SET WRI:OK ;");
            triggerSoftwareReset();
          } else {
            Serial.println("SET WRI:FORMAT ;");
          }
        }
        else if (stripre(setCmd, "WRITE")) {
          if (settingsEnabled) {
            saveHWInfo();
            settingsEnabled = false;
            Serial.println("WRITE:OK ;");
            triggerSoftwareReset();
          } else {
            Serial.println("WRITE:LOCKED ;");
          }
        }
        else if (stripre(setCmd, "SN") && settingsEnabled) {
          char* snVal = val;
          while (*snVal == ' ') snVal++;
          int slen = strlen(snVal);
          bool valid = (slen == 8);
          for (int i = 0; i < slen && valid; i++) {
            char c = snVal[i];
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) valid = false;
          }
          if (valid) {
            for (int i = 0; i < 8; i++) {
              CFG_SERIAL_NUMBER[i] = (snVal[i] >= 'a' && snVal[i] <= 'f') ? snVal[i] - 32 : snVal[i];
            }
            CFG_SERIAL_NUMBER[8] = 0;
            Serial.print("SET SN:OK:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
          } else {
            Serial.println("SET SN:FORMAT ;");
          }
        }
      }

      if (c == 0) break;
    } else {
      if (tlen < 31) token[tlen++] = c;
    }
  }
}

void processSerialTokensFromHost() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') c = ';';
    int alen = strlen(serialAccum);
    if (alen < 63) { serialAccum[alen] = c; serialAccum[alen + 1] = 0; }

    char* semi;
    while ((semi = strchr(serialAccum, ';')) != NULL) {
      *semi = 0;
      char token[32];
      char* tp = serialAccum;
      while (*tp == ' ') tp++;
      int ti;
      for (ti = 0; tp[ti] && ti < 31; ti++) token[ti] = tp[ti];
      token[ti] = 0;
      int tend = ti - 1;
      while (tend >= 0 && token[tend] == ' ') token[tend--] = 0;

      int remaining = strlen(semi + 1);
      memmove(serialAccum, semi + 1, remaining + 1);

      if (token[0] == 0) continue;

      // To uppercase for comparison
      char tokenUp[32];
      for (ti = 0; token[ti] && ti < 31; ti++) {
        tokenUp[ti] = (token[ti] >= 'a' && token[ti] <= 'z') ? token[ti] - 32 : token[ti];
      }
      tokenUp[ti] = 0;

      if (strcmp(tokenUp, "VER") == 0 || strcmp(tokenUp, "VERSION") == 0 || strcmp(tokenUp, "IDENT") == 0) {
        sendIdentAndState();
        identSentOnStart = true;
        bootMessageShown = true;
        bootMessageStart = millis();
        continue;
      }
      if (strcmp(tokenUp, "EXIT") == 0) {
        settingsEnabled = false;
        Serial.println("EXIT:OK ;");
        continue;
      }
      if (strcmp(tokenUp, "PCB") == 0) {
        Serial.print("PCB:");
        char* sp = strchr(CFG_PCB_VERSION, ' ');
        if (sp && *(sp + 1)) {
          Serial.print("v"); Serial.print(sp + 1);
        } else {
          Serial.print(CFG_PCB_VERSION);
        }
        Serial.println(" ;");
        continue;
      }
      if (strcmp(tokenUp, "RESET") == 0) {
        Serial.println("RESET:OK ;");
        triggerSoftwareReset();
        continue;
      }
      if (strcmp(tokenUp, "REQ") == 0) {
        sendStatusImmediate();
        continue;
      }
      if (strcmp(tokenUp, "DIAG") == 0) {
        if (bootState == BOOT_RUNNING) {
          bootState = BOOT_DIAG;
          diagStartTime = millis();
          diagStage = 0;
          Serial.println("DIAG:START;");
        }
        continue;
      }
      if (strchr(token, ':') != NULL) {
        char lineBuf[64];
        snprintf(lineBuf, sizeof(lineBuf), "%s;", token);
        processIncomingLine(lineBuf);
        continue;
      }
    }
  }
}

// ============================================================================
// DIAG MODE — simple LED1 blink
// ============================================================================
void runDiagSequence() {
  unsigned long elapsed = millis() - diagStartTime;

  // DIAG bypasses backlight mask for full 8-bit LED control
  if (!diagActive) {
    diagActive = true;
    desiredBacklight = 0;  // Disable backlight PWM during DIAG
  }

  // Total: ~7 seconds
  // Stage 0 (0-1s): LED1 all on, backlight full
  if (elapsed < 1000) {
    desiredLedState = 0xFF;
    desiredBacklight = 255;
    applyLEDOutputs();
    if (diagStage == 0) {
      Serial.println("DIAG:LED1_ALL_ON;");
      diagStage = 1;
    }
  }
  // Stage 1 (1-5s): LED1 walking pattern
  else if (elapsed < 5000) {
    int ledIndex = ((elapsed - 1000) / 250) % 8;
    desiredLedState = (1 << ledIndex);
    applyLEDOutputs();
    if (diagStage == 1) {
      Serial.println("DIAG:LED1_WALK;");
      diagStage = 2;
    }
  }
  // Stage 2 (5-7s): LED1 all off, backlight fade down
  else if (elapsed < 7000) {
    desiredLedState = 0x00;
    desiredBacklight = 255 - ((elapsed - 5000) * 255 / 2000);
    applyLEDOutputs();
    if (diagStage == 2) {
      Serial.println("DIAG:LED1_OFF;");
      diagStage = 3;
    }
  }
  // Done
  else {
    desiredLedState = 0x00;
    desiredBacklight = 0;
    applyLEDOutputs();
    diagActive = false;
    bootState = BOOT_RUNNING;
    Serial.println("DIAG:COMPLETE;");
  }
}

// ============================================================================
// EEPROM — SERIAL NUMBER HANDLING (identical to RMP with WXR- prefix)
// ============================================================================
void generateSerialNumber(char* out) {
  randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3) + micros());
  const char hexChars[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) {
    out[i] = hexChars[random(0, 16)];
  }
  out[8] = 0;
}

uint8_t calcCfgChecksum(uint8_t version, const char* reg8, const char* pcb8, const char* sn9) {
  uint16_t sum = version;
  for (int i = 0; i < 8; i++) sum += (uint8_t)reg8[i];
  for (int i = 0; i < 8; i++) sum += (uint8_t)pcb8[i];
  for (int i = 0; i < 9; i++) sum += (uint8_t)sn9[i];
  return (uint8_t)(sum & 0xFF);
}

void loadHWInfo() {
  uint16_t magic = (uint16_t)EEPROM.read(EEPROM_BASE_ADDR) | ((uint16_t)EEPROM.read(EEPROM_BASE_ADDR + 1) << 8);

  // Case 1: No magic → empty EEPROM, generate new SN
  if (magic != EEPROM_MAGIC) {
    Serial.println("CFG:EEPROM:MAGIC_MISSING;");
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    Serial.print("CFG:SN:NEW:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    return;
  }

  uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR + 2);

  char regbuf[9];
  char pcbbuf[9];
  for (int i = 0; i < 8; i++) regbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 3 + i);
  for (int i = 0; i < 8; i++) pcbbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 11 + i);
  regbuf[8] = 0;
  pcbbuf[8] = 0;

  // Case 2: v1/v2 EEPROM → upgrade to v3 with new SN
  if (ver <= 2) {
    uint8_t storedCs = EEPROM.read(EEPROM_BASE_ADDR + 19);
    uint8_t cs = calcCfgChecksum(ver, regbuf, pcbbuf, "         ");
    if (cs != storedCs) {
      Serial.println("CFG:EEPROM:CRC_FAIL_v1v2;");
    }
    if (cs == storedCs) {
      int ri = 0;
      for (int i = 0; i < 8; i++) if (regbuf[i] != 0 && regbuf[i] != ' ') CFG_AIRCRAFT_REG[ri++] = regbuf[i];
      if (ri > 0) CFG_AIRCRAFT_REG[ri] = 0;
      int pi = 0;
      for (int i = 0; i < 8; i++) {
        if (pcbbuf[i] == 0 || pcbbuf[i] == ' ') { int j = i; while (j < 8 && (pcbbuf[j] == 0 || pcbbuf[j] == ' ')) j++; if (j >= 8) break; }
        CFG_PCB_VERSION[pi++] = pcbbuf[i];
      }
      while (pi > 0 && CFG_PCB_VERSION[pi - 1] == ' ') pi--;
      if (pi > 0) CFG_PCB_VERSION[pi] = 0;
    }
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    Serial.print("CFG:SN:UPGRADE_v1v2:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    Serial.print("CFG:LOAD:REG="); Serial.print(CFG_AIRCRAFT_REG);
    Serial.print(";PCB="); Serial.print(CFG_PCB_VERSION);
    Serial.print(";SN="); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    return;
  }

  // Case 3: Version mismatch
  if (ver != EEPROM_FORMAT_VERSION) {
    Serial.print("CFG:EEPROM:VERSION_MISMATCH:"); Serial.print(ver);
    Serial.print("!="); Serial.println(EEPROM_FORMAT_VERSION);
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    Serial.print("CFG:SN:NEW_VER_MISMATCH:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    return;
  }

  // Case 4: v3 with correct version → load SN
  char snbuf[10];
  for (int i = 0; i < 9; i++) snbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 19 + i);
  snbuf[9] = 0;

  uint8_t storedCs = EEPROM.read(EEPROM_BASE_ADDR + 28);
  uint8_t cs = calcCfgChecksum(ver, regbuf, pcbbuf, snbuf);

  if (cs != storedCs) {
    Serial.println("CFG:EEPROM:CRC_FAIL_v3;");
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    Serial.print("CFG:SN:NEW_CRC:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    return;
  }

  // CRC OK → load REG, PCB, SN
  int ri = 0;
  for (int i = 0; i < 8; i++) if (regbuf[i] != 0 && regbuf[i] != ' ') CFG_AIRCRAFT_REG[ri++] = regbuf[i];
  if (ri > 0) CFG_AIRCRAFT_REG[ri] = 0;

  int pi = 0;
  for (int i = 0; i < 8; i++) {
    if (pcbbuf[i] == 0 || pcbbuf[i] == ' ') { int j = i; while (j < 8 && (pcbbuf[j] == 0 || pcbbuf[j] == ' ')) j++; if (j >= 8) break; }
    CFG_PCB_VERSION[pi++] = pcbbuf[i];
  }
  while (pi > 0 && CFG_PCB_VERSION[pi - 1] == ' ') pi--;
  if (pi > 0) CFG_PCB_VERSION[pi] = 0;

  int si = 0;
  for (int i = 0; i < 8; i++) if (snbuf[i] != 0 && snbuf[i] != ' ') CFG_SERIAL_NUMBER[si++] = snbuf[i];
  CFG_SERIAL_NUMBER[si] = 0;

  if (si == 0) {
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    Serial.print("CFG:SN:NEW_EMPTY:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
  } else {
    Serial.print("CFG:SN:LOADED:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
  }

  Serial.print("CFG:LOAD:REG="); Serial.print(CFG_AIRCRAFT_REG);
  Serial.print(";PCB="); Serial.print(CFG_PCB_VERSION);
  Serial.print(";SN="); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
}

void saveHWInfo() {
  char regbuf[8];
  int rlen = strlen(CFG_AIRCRAFT_REG);
  for (int i = 0; i < 8; i++) {
    if (i < rlen) regbuf[i] = CFG_AIRCRAFT_REG[i];
    else regbuf[i] = ' ';
  }

  char pcbbuf[8];
  int plen = strlen(CFG_PCB_VERSION);
  for (int i = 0; i < 8; i++) {
    if (i < plen) pcbbuf[i] = CFG_PCB_VERSION[i];
    else pcbbuf[i] = ' ';
  }

  char snbuf[9];
  int slen = strlen(CFG_SERIAL_NUMBER);
  for (int i = 0; i < 8; i++) {
    if (i < slen) snbuf[i] = CFG_SERIAL_NUMBER[i];
    else snbuf[i] = ' ';
  }
  snbuf[8] = 0;

  uint16_t magic = EEPROM_MAGIC;
  EEPROM.update(EEPROM_BASE_ADDR + 0, (uint8_t)(magic & 0xFF));
  EEPROM.update(EEPROM_BASE_ADDR + 1, (uint8_t)((magic >> 8) & 0xFF));
  EEPROM.update(EEPROM_BASE_ADDR + 2, EEPROM_FORMAT_VERSION);

  for (int i = 0; i < 8; i++) EEPROM.update(EEPROM_BASE_ADDR + 3 + i, (uint8_t)regbuf[i]);
  for (int i = 0; i < 8; i++) EEPROM.update(EEPROM_BASE_ADDR + 11 + i, (uint8_t)pcbbuf[i]);
  for (int i = 0; i < 9; i++) EEPROM.update(EEPROM_BASE_ADDR + 19 + i, (uint8_t)snbuf[i]);

  uint8_t cs = calcCfgChecksum(EEPROM_FORMAT_VERSION, regbuf, pcbbuf, snbuf);
  EEPROM.update(EEPROM_BASE_ADDR + 28, cs);

  Serial.print("CFG:SAVED:REG="); Serial.print(CFG_AIRCRAFT_REG);
  Serial.print(";PCB="); Serial.print(CFG_PCB_VERSION);
  Serial.print(";SN="); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  wdt_disable();
  Serial.begin(115200);

  // Pin modes — digital inputs with pullups
  pinMode(PIN_MULTISCAN_MAN,  INPUT_PULLUP);
  pinMode(PIN_MULTISCAN_AUTO, INPUT_PULLUP);
  pinMode(PIN_MAP,            INPUT_PULLUP);
  pinMode(PIN_TURB,           INPUT_PULLUP);
  pinMode(PIN_WXT,            INPUT_PULLUP);
  pinMode(PIN_WX,             INPUT_PULLUP);
  pinMode(PIN_SYS1,           INPUT_PULLUP);
  pinMode(PIN_SYS2,           INPUT_PULLUP);
  pinMode(PIN_PWS_AUTO,       INPUT_PULLUP);
  pinMode(PIN_GCS_AUTO,       INPUT_PULLUP);

  // Analog inputs (default is INPUT, no pullup)
  pinMode(PIN_TILT_POT, INPUT);
  pinMode(PIN_GAIN_POT, INPUT);

  // LED driver outputs
  pinMode(PIN_LED_LATCH, OUTPUT);
  pinMode(PIN_LED_CLK,   OUTPUT);
  pinMode(PIN_LED_DATA,  OUTPUT);

  // Backlight PWM
  pinMode(PIN_BACKLIGHT, OUTPUT);

  // Initialize outputs off
  shiftOutLEDs(0x00);
  analogWrite(PIN_BACKLIGHT, 0);
  hwLedState = 0x00;
  hwBacklight = 0;

  // Load persisted configuration from EEPROM
  loadHWInfo();

  // Print configuration
  Serial.print("SETUP:Debounce="); Serial.print(CFG_BUTTON_DEBOUNCE);
  Serial.print("ms, REG="); Serial.print(CFG_AIRCRAFT_REG);
  Serial.print(", SN="); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
  Serial.print("SETUP:FW="); Serial.print(FW_VERSION);
  Serial.print(", PCB="); Serial.println(CFG_PCB_VERSION);

  // Boot message
  bootMessageShown = true;
  bootMessageStart = millis();

  // Go straight to running
  bootState = BOOT_RUNNING;
  forceSendNext = true;
  maybeSendIdentStartup();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  // Handle DIAG sequence
  if (bootState == BOOT_DIAG) {
    runDiagSequence();
    return;
  }

  processSerialTokensFromHost();

  if (now - lastLoopTs < LOOP_INTERVAL_MS) return;
  lastLoopTs = now;

  // Read all inputs
  readAllInputs();

  // Detect changes (debounced)
  bool inputChanged = false;
  if (inputState1 != lastInputState1 || inputState2 != lastInputState2) {
    if (now - lastDebounceTs >= CFG_BUTTON_DEBOUNCE) {
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      inputChanged = true;
    }
  }

  // Detect POT changes
  bool potChanged = (abs((int)potTiltRaw - (int)lastPotTiltRaw) > 2 ||
                     abs((int)potGainRaw - (int)lastPotGainRaw) > 2);

  // Send status on change
  if (inputChanged || potChanged) {
    sendStatusImmediate();
  }
}
