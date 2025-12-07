// RMP Control Script, (w) 2025 M. Quatember
// Based on ACP architecture with MAX7219 displays, rotary encoders, and LED sink drivers

#include <LedControl.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// MAX7219 7-segment displays (2x in series)
const int max7219DataPin  = A0;  // DIN
const int max7219CSPin    = 15;  // CS
const int max7219ClkPin   = 2;   // CLK

// LED Sink Drivers (74HC595, 2x in series)
const int ledLatchPin = 4;
const int ledClockPin = 5;
const int ledDataPin  = 6;

// Input Shift Registers (HC165)
// Note: A1, A2, A3 are analog pins but used as digital on Pro Micro
const int inputDataPin  = A1;  // Serial data in
const int inputClockPin = A2;  // Clock
const int inputLatchPin = A3;  // Latch (parallel load)

// Direct-wired LEDs
const int ledIlsSel = 14;  // ILS SEL indicator
const int ledMlsSel = 16;  // MLS SEL indicator

// Rotary encoders
const int rotary1PinA = 7;   // RT1 left
const int rotary1PinB = 8;   // RT1 right
const int rotary2PinA = 9;   // RT2 left
const int rotary2PinB = 10;  // RT2 right

// PWM brightness control
const int pwmBrightness = 3;

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "RMP1 CPT, v1.0 MAQ";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// ============================================================================
// MAX7219 DISPLAY SETUP
// ============================================================================
// LedControl(dataPin, clkPin, csPin, numDevices)
LedControl lc = LedControl(max7219DataPin, max7219ClkPin, max7219CSPin, 2);

// Display addresses: 0 = left display, 1 = right display
const int DISP_LEFT  = 0;
const int DISP_RIGHT = 1;

// ============================================================================
// LED STATE (desired + hardware cache)
// ============================================================================
uint16_t desiredLedState = 0x0000;  // 16 bits for 2x 8-bit shift registers
uint16_t hwLedState      = 0x0000;

bool desiredIlsLed = false;
bool desiredMlsLed = false;
bool hwIlsLed = false;
bool hwMlsLed = false;

uint8_t desiredBrightness = 0;
uint8_t hwBrightness = 0;

// ============================================================================
// DISPLAY STATE
// ============================================================================
String displayLeft  = "      ";  // 6 chars for left display
String displayRight = "      ";  // 6 chars for right display
uint8_t displayBrightness = 15;  // MAX7219 brightness (0-15), default full

// ============================================================================
// INPUT STATE (shift registers)
// ============================================================================
uint8_t inputState1 = 0x00;
uint8_t inputState2 = 0x00;
uint8_t lastInputState1 = 0x00;
uint8_t lastInputState2 = 0x00;
uint8_t prevInputState1 = 0x00;  // Track previous state to detect actual changes
uint8_t prevInputState2 = 0x00;

// ============================================================================
// ROTARY ENCODER STATE
// ============================================================================
volatile int rotary1Counter = 0;
volatile int rotary2Counter = 0;
int lastRotary1Counter = 0;
int lastRotary2Counter = 0;

volatile uint8_t rotary1LastEncoded = 0;
volatile uint8_t rotary2LastEncoded = 0;

// Rotary speed detection (fast = 2 steps, slow = 1 step)
volatile unsigned long rotary1LastChangeTime = 0;
volatile unsigned long rotary2LastChangeTime = 0;
const unsigned long ROTARY_FAST_THRESHOLD_MS = 50;  // < 50ms between steps = fast turn

// ============================================================================
// TIMING / THROTTLES / DEBOUNCE
// ============================================================================
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;

unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;

unsigned long lastDebounceTs = 0;
const unsigned long DEBOUNCE_MS = 12;

// ============================================================================
// BOOT / CONNECTION STATE
// ============================================================================
enum BootState { BOOT_INIT, BOOT_RUNNING, BOOT_DIAG };
BootState bootState = BOOT_INIT;
unsigned long bootSequenceStart = 0;

// External state control (from host via STATE:00/01)
bool hostOnline = false;  // false = offline, true = online
String offlineMessage1 = "no FS ";
String offlineMessage2 = "onLinE";
unsigned long offlineDisplayStart = 0;  // When offline message was shown
const unsigned long OFFLINE_DISPLAY_DURATION_MS = 10000;  // Show for 10 seconds
bool offlineMessageShown = false;  // Track if message is currently displayed

// DIAG mode state
unsigned long diagStartTime = 0;
int diagStage = 0;
bool diagComboWasActive = false;  // Track if combo was pressed (prevent re-trigger while held)

// ============================================================================
// RUNTIME BUFFERS / FLAGS
// ============================================================================
String serialAccum = "";
bool forceSendNext = false;

// ============================================================================
// HELPER FUNCTIONS - STRING FORMATTING
// ============================================================================
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

uint16_t parseBin16(const String &s) {
  uint16_t v = 0;
  for (int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c == '0' || c == '1') v = (v << 1) | (c == '1' ? 1 : 0);
  }
  return v;
}

// ============================================================================
// MAX7219 DISPLAY FUNCTIONS
// ============================================================================
void initDisplays() {
  // Wake up both MAX7219 devices
  for (int dev = 0; dev < 2; dev++) {
    lc.shutdown(dev, false);       // Wake up
    lc.setIntensity(dev, displayBrightness);  // Set brightness
    lc.clearDisplay(dev);          // Clear display
  }
}

void setDisplayBrightness(uint8_t brightness) {
  displayBrightness = constrain(brightness, 0, 15);
  for (int dev = 0; dev < 2; dev++) {
    lc.setIntensity(dev, displayBrightness);
  }
}

// Map characters to 7-segment patterns (simple A-Z, 0-9, space, dash)
byte charTo7Seg(char c) {
  // Check for lowercase characters before converting to uppercase
  if (c == 'o') return 0b00011101;  // Lowercase o (bottom segments only)
  if (c == 'i') return 0b00010000;  // Lowercase i (just vertical segments)
  if (c == 'm') return 0b00010101;  // Lowercase m (simplified)
  
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
    case 'N': return 0b00010101;
    case 'O': return 0b01111110;
    case 'P': return 0b01100111;
    case 'R': return 0b00000101;
    case 'S': return 0b01011011;
    case 'T': return 0b00001111;
    case 'U': return 0b00111110;
    case '-': return 0b00000001;
    case ' ': return 0b00000000;
    default:  return 0b00000000;
  }
}

void updateDisplay(int device, const String &text) {
  // text is up to 6 characters; device 0 or 1
  // MAX7219: digit 0 is rightmost, digit 7 is leftmost (we use 0-5 for 6 digits)
  int textPos = 0;
  for (int digitPos = 0; digitPos < 6; digitPos++) {
    if (textPos >= text.length()) {
      lc.setRow(device, 5 - digitPos, 0b00000000);  // Blank remaining digits
      continue;
    }
    
    char c = text.charAt(textPos);
    
    // Skip decimal points - they're not characters, they're modifiers
    if (c == '.') {
      textPos++;
      if (textPos >= text.length()) {
        lc.setRow(device, 5 - digitPos, 0b00000000);
        continue;
      }
      c = text.charAt(textPos);
    }
    
    byte pattern = charTo7Seg(c);
    
    // Check if next char is a decimal point (applies to current digit)
    if (textPos + 1 < text.length() && text.charAt(textPos + 1) == '.') {
      pattern |= 0b10000000;  // Set DP bit
    }
    
    lc.setRow(device, 5 - digitPos, pattern);
    textPos++;
  }
}

void displayText(const String &left, const String &right) {
  // Remove decimal points from length calculation (they're merged with prev digit)
  String leftClean = left;
  String rightClean = right;
  leftClean.replace(".", "");
  rightClean.replace(".", "");
  
  updateDisplay(DISP_LEFT, left);
  updateDisplay(DISP_RIGHT, right);
  
  displayLeft = left;
  displayRight = right;
}

// ============================================================================
// LED SINK DRIVER FUNCTIONS (74HC595)
// ============================================================================
void shiftOutLEDs(uint16_t ledBits) {
  digitalWrite(ledLatchPin, LOW);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, (ledBits >> 8) & 0xFF);  // High byte first
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, ledBits & 0xFF);          // Low byte second
  digitalWrite(ledLatchPin, HIGH);
}

void applyLEDOutputs() {
  shiftOutLEDs(desiredLedState);
  digitalWrite(ledIlsSel, desiredIlsLed ? HIGH : LOW);
  digitalWrite(ledMlsSel, desiredMlsLed ? HIGH : LOW);
  analogWrite(pwmBrightness, desiredBrightness);
  
  hwLedState = desiredLedState;
  hwIlsLed = desiredIlsLed;
  hwMlsLed = desiredMlsLed;
  hwBrightness = desiredBrightness;
}

void setLEDState(uint16_t ledBits, bool ils, bool mls, uint8_t brightness) {
  desiredLedState = ledBits;
  desiredIlsLed = ils;
  desiredMlsLed = mls;
  desiredBrightness = brightness;
  applyLEDOutputs();
}

// ============================================================================
// SHIFT REGISTER INPUT (HC165)
// ============================================================================
void readShiftRegisters(uint8_t &in1, uint8_t &in2) {
  // Latch
  digitalWrite(inputLatchPin, LOW);
  delayMicroseconds(20);
  digitalWrite(inputLatchPin, HIGH);
  delayMicroseconds(20);
  
  // Read shift registers
  byte raw1 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);
  byte raw2 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);
  in1 = ~raw1;  // Invert if buttons are active-low
  in2 = ~raw2;
  
  // Timing stabilization (like ACP)
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

// ============================================================================
// ROTARY ENCODER INTERRUPTS
// ============================================================================
void updateRotary1() {
  uint8_t MSB = digitalRead(rotary1PinA);
  uint8_t LSB = digitalRead(rotary1PinB);
  uint8_t encoded = (MSB << 1) | LSB;
  uint8_t sum = (rotary1LastEncoded << 2) | encoded;
  
  unsigned long now = millis();
  bool isFast = (now - rotary1LastChangeTime) < ROTARY_FAST_THRESHOLD_MS;
  int step = isFast ? 2 : 1;
  
  // Only process valid state transitions (ignore bounces)
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    // Debounce: only count if enough time passed since last change (2ms minimum)
    if ((now - rotary1LastChangeTime) >= 2 || rotary1LastChangeTime == 0) {
      rotary1Counter += step;
      rotary1LastChangeTime = now;
    }
  }
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    if ((now - rotary1LastChangeTime) >= 2 || rotary1LastChangeTime == 0) {
      rotary1Counter -= step;
      rotary1LastChangeTime = now;
    }
  }
  
  rotary1LastEncoded = encoded;
}

void updateRotary2() {
  uint8_t MSB = digitalRead(rotary2PinA);
  uint8_t LSB = digitalRead(rotary2PinB);
  uint8_t encoded = (MSB << 1) | LSB;
  uint8_t sum = (rotary2LastEncoded << 2) | encoded;
  
  unsigned long now = millis();
  bool isFast = (now - rotary2LastChangeTime) < ROTARY_FAST_THRESHOLD_MS;
  int step = isFast ? 2 : 1;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    rotary2Counter += step;
    rotary2LastChangeTime = now;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    rotary2Counter -= step;
    rotary2LastChangeTime = now;
  }
  
  rotary2LastEncoded = encoded;
}

// ============================================================================
// SERIAL IDENT / STATE
// ============================================================================
void sendIdentAndState() {
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
  Serial.print(";STATE:RUNNING;");
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
// SERIAL COMMAND PARSING
// ============================================================================
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
      if (token.equalsIgnoreCase("VER") || token.equalsIgnoreCase("VERSION")) {
        sendIdentAndState();
        continue;
      }
      if (token.equalsIgnoreCase("REQ")) {
        sendStatusImmediate();
        continue;
      }
      if (token.equalsIgnoreCase("DIAG")) {
        if (bootState == BOOT_RUNNING) {
          bootState = BOOT_DIAG;
          diagStartTime = millis();
          diagStage = 0;
          Serial.println("DIAG:START;");
        }
        continue;
      }
      continue;
    }
    
    int colon = token.indexOf(':');
    if (colon < 0) continue;
    String key = token.substring(0, colon);
    String val = token.substring(colon + 1);
    key.trim();
    val.trim();
    
    if (key.equalsIgnoreCase("LED1")) {
      desiredLedState = (desiredLedState & 0x00FF) | (parseBin8(val) << 8);
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("LED2")) {
      desiredLedState = (desiredLedState & 0xFF00) | parseBin8(val);
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("LED3")) {
      // Format: "LED3:01" where bit 0 = ILS, bit 1 = MLS
      uint8_t bits = parseBin8(val);
      desiredIlsLed = bits & 0b01;
      desiredMlsLed = bits & 0b10;
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("BL")) {
      desiredBrightness = constrain(val.toInt(), 0, 255);
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("DISP_BL")) {
      setDisplayBrightness(val.toInt());
    }
    else if (key.equalsIgnoreCase("DSP1")) {
      displayLeft = val;
      updateDisplay(DISP_LEFT, displayLeft);
    }
    else if (key.equalsIgnoreCase("DSP2")) {
      displayRight = val;
      updateDisplay(DISP_RIGHT, displayRight);
    }
    else if (key.equalsIgnoreCase("STATE")) {
      // STATE:00 = offline, STATE:01 = online
      hostOnline = (val == "01" || val == "1");
      // If going online, clear offline message
      if (hostOnline && displayLeft == offlineMessage1 && displayRight == offlineMessage2) {
        displayText("      ", "      ");
      }
    }
    else if (key.equalsIgnoreCase("REQ")) {
      forceSendNext = true;
    }
    else if (key.equalsIgnoreCase("VER")) {
      sendIdentAndState();
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
      String token = serialAccum.substring(0, idx);
      token.trim();
      serialAccum = serialAccum.substring(idx + 1);
      if (token.length() == 0) continue;
      
      String tokenUp = token;
      tokenUp.toUpperCase();
      
      if (tokenUp == "VER" || tokenUp == "VERSION") {
        // Show "  UEr" on left display, "rEQ  " on right display (inner digits)
        displayText("  UEr", "rEQ  ");
        delay(500);  // Show for 500ms
        displayText("      ", "      ");  // Clear
        sendIdentAndState();
        identSentOnStart = true;
        pauseUntil = millis() + 200;
        continue;
      }
      if (tokenUp == "REQ") {
        sendStatusImmediate();
        continue;
      }
      if (tokenUp == "DIAG") {
        if (bootState == BOOT_RUNNING) {
          bootState = BOOT_DIAG;
          diagStartTime = millis();
          diagStage = 0;
          Serial.println("DIAG:START;");
        }
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
// STATUS SENDING
// ============================================================================
void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;
  
  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";");
  Serial.print("IN2:"); Serial.print(bin8(inputState2)); Serial.print(";");
  
  // Send rotary encoder deltas (reset after sending)
  int rt1Delta = rotary1Counter - lastRotary1Counter;
  int rt2Delta = rotary2Counter - lastRotary2Counter;
  
  if (rt1Delta != 0 || rt2Delta != 0 || forceSendNext) {
    Serial.print("RT1:"); Serial.print(rt1Delta); Serial.print(";");
    Serial.print("RT2:"); Serial.print(rt2Delta); Serial.print(";");
    lastRotary1Counter = rotary1Counter;
    lastRotary2Counter = rotary2Counter;
  }
  
  Serial.println();
  lastSendTs = now;
  forceSendNext = false;
}

void sendStatusImmediate() {
  // Force send immediately, bypassing throttle and always including rotary data
  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";");
  Serial.print("IN2:"); Serial.print(bin8(inputState2)); Serial.print(";");
  
  // Always send rotary encoder deltas
  int rt1Delta = rotary1Counter - lastRotary1Counter;
  int rt2Delta = rotary2Counter - lastRotary2Counter;
  Serial.print("RT1:"); Serial.print(rt1Delta); Serial.print(";");
  Serial.print("RT2:"); Serial.print(rt2Delta); Serial.print(";");
  
  Serial.println();
  
  // Reset deltas after sending
  lastRotary1Counter = rotary1Counter;
  lastRotary2Counter = rotary2Counter;
  lastSendTs = millis();
}

// ============================================================================
// BOOT SEQUENCE
// ============================================================================
void runBootSequence() {
  unsigned long elapsed = millis() - bootSequenceStart;
  
  // Stage 1: LED pattern flash (0-300ms)
  if (elapsed < 150) {
    setLEDState(0xAAAA, true, false, 200);
    displayText("      ", "      ");
  }
  else if (elapsed < 300) {
    setLEDState(0x5555, false, true, 200);
    displayText("      ", "      ");
  }
  // Stage 2: All LEDs on, display "--xx--" counter (left) and "  init" (right)
  else if (elapsed < 450) {
    setLEDState(0xFFFF, true, true, 200);
    displayText("--00--", "  init");
  }
  else if (elapsed < 600) displayText("--01--", "  init");
  else if (elapsed < 750) displayText("--02--", "  init");
  else if (elapsed < 900) displayText("--03--", "  init");
  else if (elapsed < 1050) displayText("--04--", "  init");
  else if (elapsed < 1200) displayText("--05--", "  init");
  else if (elapsed < 1350) displayText("--06--", "  init");
  else if (elapsed < 1500) displayText("--07--", "  init");
  else if (elapsed < 1650) displayText("--08--", "  init");
  else if (elapsed < 1800) displayText("--09--", "  init");
  else if (elapsed < 1950) displayText("--10--", "  init");
  // Stage 3: All off, sequence complete (1950ms+)
  else {
    setLEDState(0x0000, false, false, 0);
    displayText("      ", "      ");
    bootState = BOOT_RUNNING;
    forceSendNext = true;
    maybeSendIdentStartup();
  }
}

// ============================================================================
// OFFLINE DISPLAY (non-blocking)
// ============================================================================
void updateOfflineDisplay() {
  unsigned long now = millis();
  
  // If host is offline
  if (!hostOnline) {
    // First time offline and display is blank - show message
    if (!offlineMessageShown && displayLeft == "      " && displayRight == "      ") {
      displayLeft = offlineMessage1;
      displayRight = offlineMessage2;
      updateDisplay(DISP_LEFT, displayLeft);
      updateDisplay(DISP_RIGHT, displayRight);
      offlineDisplayStart = now;
      offlineMessageShown = true;
      Serial.println("OFFLINE:MESSAGE_SHOWN;");
    }
    // If message is shown and timeout expired, clear it and STAY cleared
    else if (offlineMessageShown && (now - offlineDisplayStart) >= OFFLINE_DISPLAY_DURATION_MS) {
      // Clear display but DON'T reset offlineMessageShown - keep it true to prevent re-showing
      // Only reactivateOfflineMessage() can show it again
      if (displayLeft == offlineMessage1 && displayRight == offlineMessage2) {
        displayLeft = "      ";
        displayRight = "      ";
        updateDisplay(DISP_LEFT, displayLeft);
        updateDisplay(DISP_RIGHT, displayRight);
        Serial.println("OFFLINE:MESSAGE_TIMEOUT;");
        // NOTE: offlineMessageShown stays TRUE to prevent automatic re-show
      }
    }
  } else {
    // Online - reset flags
    if (offlineMessageShown) {
      Serial.println("OFFLINE:MESSAGE_CLEARED_ONLINE;");
    }
    offlineMessageShown = false;
  }
}

// Reactivate offline message on button press
void reactivateOfflineMessage() {
  if (!hostOnline && !offlineMessageShown) {
    if (displayLeft == "      " && displayRight == "      ") {
      displayLeft = offlineMessage1;
      displayRight = offlineMessage2;
      updateDisplay(DISP_LEFT, displayLeft);
      updateDisplay(DISP_RIGHT, displayRight);
      offlineDisplayStart = millis();
      offlineMessageShown = true;
      Serial.println("OFFLINE:MESSAGE_REACTIVATED;");
    }
  }
}

// ============================================================================
// DIAG MODE
// ============================================================================
void runDiagSequence() {
  unsigned long elapsed = millis() - diagStartTime;
  
  // Stage 0: Show "DIAG START" message (0-2000ms)
  if (elapsed < 2000) {
    setLEDState(0x0000, false, false, 0);
    displayText("dIAG  ", "StArt ");
  }
  // Stage 1: Display count test - All LEDs OFF, digits count 0-9 with DP sweep (2000-7000ms)
  else if (elapsed < 7000) {
    int digit = ((elapsed - 2000) / 500) % 10;
    int dpPos = ((elapsed - 2000) / 500) % 6;
    
    // Keep ALL LEDs off during counting sequence
    setLEDState(0x0000, false, false, 0);
    
    String leftDisplay = "";
    String rightDisplay = "";
    
    for (int i = 0; i < 6; i++) {
      leftDisplay += String(digit);
      rightDisplay += String(digit);
      if (i == dpPos) {
        leftDisplay += ".";
        rightDisplay += ".";
      }
    }
    
    displayText(leftDisplay, rightDisplay);
  }
  // Stage 2: Full LED test - ALL LEDs ON (7000-10000ms)
  else if (elapsed < 10000) {
    setLEDState(0xFFFF, true, true, 255);
    displayText("012345", "6789Ab");
  }
  // Stage 3: Shift register input test - show raw input states (10000-15000ms)
  else if (elapsed < 15000) {
    setLEDState(0x0000, false, false, 128);
    
    // Read current input states
    uint8_t in1, in2;
    readShiftRegisters(in1, in2);
    
    // Display input states in binary on both displays
    String leftBin = "";
    String rightBin = "";
    
    // Show IN1 on left display (6 bits: bits 7-2)
    for (int i = 7; i >= 2; i--) {
      leftBin += (in1 & (1 << i)) ? "1" : "0";
    }
    
    // Show IN2 on right display (6 bits: bits 7-2)
    for (int i = 7; i >= 2; i--) {
      rightBin += (in2 & (1 << i)) ? "1" : "0";
    }
    
    displayText(leftBin, rightBin);
    Serial.print("DIAG:IN1="); Serial.print(bin8(in1));
    Serial.print(";IN2="); Serial.println(bin8(in2));
  }
  // Stage 4: LED driver test - walking LED pattern (15000-20000ms)
  else if (elapsed < 20000) {
    displayText("LEd   ", "tESt  ");
    
    // Walking bit pattern through all 16 LEDs
    int ledIndex = ((elapsed - 15000) / 200) % 16;
    uint16_t pattern = 1 << ledIndex;
    setLEDState(pattern, ledIndex == 0, ledIndex == 1, 200);
  }
  // Stage 5: MAX7219 segment test - light up each segment individually (20000-25000ms)
  else if (elapsed < 25000) {
    setLEDState(0x0000, false, false, 128);
    
    // Show message for first 2 seconds, then clear and run actual test
    if (elapsed < 22000) {
      displayText("SEG   ", "tESt  ");
    } else {
      // Clear displays and run actual segment test
      // Cycle through segments a-g plus DP (8 segments)
      int segIndex = ((elapsed - 22000) / 300) % 8;
      byte segPattern = 1 << segIndex;
      
      // Show same segment pattern on all digits of both displays
      for (int dev = 0; dev < 2; dev++) {
        for (int digit = 0; digit < 6; digit++) {
          lc.setRow(dev, digit, segPattern);
        }
      }
    }
  }
  // Stage 6: Brightness test (25000-30000ms)
  else if (elapsed < 30000) {
    displayText("brIGht", "tESt  ");
    setLEDState(0xFFFF, true, true, 255);
    
    // Fade brightness up and down
    int fadePos = (elapsed - 25000) % 2000;
    int brightness = (fadePos < 1000) ? (fadePos / 4) : (255 - ((fadePos - 1000) / 4));
    
    analogWrite(pwmBrightness, brightness);
    for (int dev = 0; dev < 2; dev++) {
      lc.setIntensity(dev, brightness / 17);  // 0-15 range
    }
  }
  // Stage 7: DIAG complete message (30000-33000ms)
  else if (elapsed < 33000) {
    setLEDState(0x0000, false, false, 128);
    displayText("dIAG  ", "PASs  ");
  }
  // Stage 8: Return to running
  else {
    // Restore defaults
    setDisplayBrightness(displayBrightness);  // Restore user brightness setting
    setLEDState(0x0000, false, false, 0);
    displayText("      ", "      ");
    bootState = BOOT_RUNNING;
    diagComboWasActive = false;
    // Reset offline message state so it shows fresh after DIAG
    offlineMessageShown = false;
    Serial.println("DIAG:COMPLETE;");
  }
}

// Check for DIAG combo: IN1:00000010;IN2:00010001
void checkDiagCombo() {
  bool comboActive = (inputState1 == 0b00000010 && inputState2 == 0b00010001);
  
  // Trigger DIAG only on rising edge (combo just pressed, not held)
  if (bootState == BOOT_RUNNING && comboActive && !diagComboWasActive) {
    bootState = BOOT_DIAG;
    diagStartTime = millis();
    diagStage = 0;
    Serial.println("DIAG:START;");
    diagComboWasActive = true;  // Set flag immediately
  }
  else if (!comboActive) {
    diagComboWasActive = false;  // Reset flag when combo released
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Pin modes
  pinMode(ledLatchPin, OUTPUT);
  pinMode(ledClockPin, OUTPUT);
  pinMode(ledDataPin, OUTPUT);
  pinMode(ledIlsSel, OUTPUT);
  pinMode(ledMlsSel, OUTPUT);
  pinMode(pwmBrightness, OUTPUT);
  
  pinMode(inputDataPin, INPUT);
  pinMode(inputClockPin, OUTPUT);
  pinMode(inputLatchPin, OUTPUT);
  
  pinMode(rotary1PinA, INPUT_PULLUP);
  pinMode(rotary1PinB, INPUT_PULLUP);
  pinMode(rotary2PinA, INPUT_PULLUP);
  pinMode(rotary2PinB, INPUT_PULLUP);
  
  // Initialize displays
  initDisplays();
  
  // Initialize LEDs off (including direct LEDs)
  digitalWrite(ledIlsSel, LOW);
  digitalWrite(ledMlsSel, LOW);
  setLEDState(0x0000, false, false, 0);
  
  // Attach rotary encoder interrupts (both pins for better reliability)
  attachInterrupt(digitalPinToInterrupt(rotary1PinA), updateRotary1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotary1PinB), updateRotary1, CHANGE);
  
  // Pro Micro: pins 9 and 10 do NOT support hardware interrupts!
  // Only pins 0,1,2,3,7 support interrupts on ATmega32U4
  // We need to poll rotary 2 in the main loop instead
  // attachInterrupt(digitalPinToInterrupt(rotary2PinA), updateRotary2, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(rotary2PinB), updateRotary2, CHANGE);
  
  Serial.println("SETUP:Pin 9/10 no interrupts, using polling for RT2;");
  
  // Start boot sequence
  bootState = BOOT_INIT;
  bootSequenceStart = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  
  // Handle boot sequence
  if (bootState == BOOT_INIT) {
    runBootSequence();
    return;
  }
  
  // Handle DIAG sequence
  if (bootState == BOOT_DIAG) {
    runDiagSequence();
    return;
  }
  
  if (now < pauseUntil) return;
  
  processSerialTokensFromHost();
  
  if (now - lastLoopTs < LOOP_INTERVAL_MS) return;
  lastLoopTs = now;
  
  // Read shift registers (do this first to get current button state)
  readShiftRegisters(inputState1, inputState2);
  
  // Detect button changes (debounced)
  bool inputChanged = false;
  if (inputState1 != lastInputState1 || inputState2 != lastInputState2) {
    if (now - lastDebounceTs >= DEBOUNCE_MS) {
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      inputChanged = true;
      
      // Reactivate offline message only if inputs actually changed from previous stable state
      if (inputState1 != prevInputState1 || inputState2 != prevInputState2) {
        reactivateOfflineMessage();
        prevInputState1 = inputState1;
        prevInputState2 = inputState2;
      }
      
      sendStatusImmediate();
    }
  }
  
  // Check for DIAG combo (after debounce)
  checkDiagCombo();
  
  // Poll rotary 2 manually (pins 9/10 don't support interrupts on Pro Micro)
  updateRotary2();
  
  // Send status periodically or if rotary changed
  if (rotary1Counter != lastRotary1Counter || rotary2Counter != lastRotary2Counter) {
    sendStatus();
  }
  
  // Update offline display if needed (non-blocking)
  updateOfflineDisplay();
}
