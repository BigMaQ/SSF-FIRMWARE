// ATC Control Script, (w) 2025 M. Quatember
// Based on RMP architecture with MAX7219 display, shift register inputs, and LED sink drivers

#include <LedControl.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// MAX7219 7-segment display (1x, 4 digits)
const int max7219DataPin  = 1;   // DIN
const int max7219CSPin    = 0;   // CS
const int max7219ClkPin   = 2;   // CLK

// Backlight Driver (74HC595, 1x)
const int blLatchPin = 4;
const int blClockPin = 5;
const int blDataPin  = 6;

// PWM brightness control
const int pwmBrightness = 3;

// Input Shift Registers (HC165, 1x)
const int inputDataPin  = A1;  // Serial data in
const int inputClockPin = A2;  // Clock
const int inputLatchPin = A3;  // Latch (parallel load)

// Direct-wired LEDs
const int ledAtcFail = 7;   // ATC FAIL indicator

// Direct-wired Inputs (mapped as IN3)
// IN3 bit mapping (active-low with INPUT_PULLUP):
//   bit 0 = ON     (Pin 8)
//   bit 1 = SYS2   (Pin 9)
//   bit 2 = SYS1   (Pin 10)
//   bit 3 = IDENT  (Pin 14)
//   bit 4 = AUTO   (Pin 15)
//   bit 5 = CLR    (Pin 16)
//   bit 6 = STBY   (Pin A0)
const int pinOn        = 8;   // ON     → IN3 bit 0
const int pinSys2      = 9;   // SYS2   → IN3 bit 1
const int pinSys1      = 10;  // SYS1   → IN3 bit 2
const int pinIdent     = 14;  // IDENT  → IN3 bit 3
const int pinAuto      = 15;  // AUTO   → IN3 bit 4
const int pinClr       = 16;  // CLR    → IN3 bit 5
const int pinStby      = A0;  // STBY   → IN3 bit 6

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "ATC, v1.2 MAQ";
const char  FW_VERSION[] = "1.2";
const char  PANEL_SN_PREFIX[] = "ATC-";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// Serial Number – eindeutige Panel-ID
char CFG_SERIAL_NUMBER[10] = "";  // 8 hex chars + null

// ============================================================================
// MAX7219 DISPLAY SETUP
// ============================================================================
// LedControl(dataPin, clkPin, csPin, numDevices) — only 1 device, 4 digits
LedControl lc = LedControl(max7219DataPin, max7219ClkPin, max7219CSPin, 1);

const int DISP_DEVICE = 0;

// ============================================================================
// LED STATE (desired + hardware cache)
// ============================================================================
uint8_t desiredBacklight = 0x00;   // 8 bits for backlight shift register
uint8_t hwBacklight      = 0x00;

bool desiredAtcFailLed = true;   // true = ATC FAIL active (LED ON until host clears it via STATE:01 or LED2:00000000)
bool hwAtcFailLed      = true;
bool ledTestActive     = false; // true during boot/DIAG LED tests (disables offline-force for ATC FAIL)

uint8_t desiredBrightness = 0;
uint8_t hwBrightness = 0;

// ============================================================================
// DISPLAY STATE
// ============================================================================
String displayText4 = "    ";  // 4 chars for the single 4-digit display
uint8_t displayBrightness = 15;  // MAX7219 brightness (0-15), default full

// XPDR multi-source display routing
//   dsp1 = complete squawk   (L:N_FREQ_XPDR_SELECTED)
//   dsp2 = char count 0..4   (L:N_PED_XPDR_CHAR_DISPLAYED)
//   dsp3 = partial/standby   (L:N_FREQ_STANDBY_XPDR_SELECTED)
String dsp1Value = "";      // complete squawk code
int    dsp2Value = -1;      // character count (-1 = never received, 0..4)
String dsp3Value = "";      // standby / partial entry
bool   dsp2Active = false;  // true once first dsp2 received (enables routing)

// Scrolling support for the 4-digit display
String currentFull = "";
int scrollOffset = 0;
unsigned long lastScrollTs = 0;
const unsigned long SCROLL_INTERVAL_MS = 300;
bool scrollEnabled = false;

// ============================================================================
// INPUT STATE (2x daisy-chained HC165 + direct pins)
// ============================================================================
uint8_t  inputState1 = 0x00;     // 8 bits from 1st HC165 (IN1)
uint8_t  inputState2 = 0x00;     // 8 bits from 2nd HC165 (IN2)
uint8_t  inputState3 = 0x00;     // 7 bits from direct pins (IN3, bit 7 = 0)
uint8_t  lastInputState1 = 0x00;
uint8_t  lastInputState2 = 0x00;
uint8_t  lastInputState3 = 0x00;
uint8_t  prevInputState1 = 0x00;
uint8_t  prevInputState2 = 0x00;
uint8_t  prevInputState3 = 0x00;
uint8_t  pendingChanged1 = 0x00;
uint8_t  pendingChanged2 = 0x00;
uint8_t  pendingChanged3 = 0x00;

// ============================================================================
// TIMING / THROTTLES / DEBOUNCE
// ============================================================================
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;

unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;

unsigned long lastDebounceTs = 0;

// ============================================================================
// CONFIGURATION PARAMETERS (before first use in functions)
// ============================================================================
uint8_t  CFG_BUTTON_DEBOUNCE = 25;     // Button debounce time in ms (00-99), default 25ms
uint16_t CFG_LED_REFRESH = 1200;      // LED refresh interval in ms, default 1200ms
uint16_t CFG_DISPLAY_REFRESH = 1200;  // Display refresh interval in ms, default 1200ms
String   CFG_AIRCRAFT_REG = "D-A320"; // Aircraft registration, default D-A320, max 8 chars
bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";
char CFG_PCB_VERSION[9] = "PCB 1.0";

// ============================================================================
// BUTTON & LED DEFINITIONS FOR BUTTON TEST
// ============================================================================

// Button definitions for IN1 (register/chip 1) — 8 bits from 1x HC165
struct ButtonDef {
  const char* name;     // Display name (max 4 chars for ATC)
  uint8_t ledRegister;  // 1 = backlight shift register, 2 = direct LED (ATC FAIL), 0 = no LED
  uint8_t ledBit;       // 0-7 (which bit in that register), or 0 for direct
};

// IN1 button definitions (1st Shifter, 8 total) — Mode/Data keys
const ButtonDef buttons_IN1[8] = {
  {"ALL ", 1, 0},  // Bit 0: ALL
  {"AbU ", 1, 1},  // Bit 1: ABV
  {"bLuU", 1, 2},  // Bit 2: BLW
  {"On  ", 1, 3},  // Bit 3: ON
  {"tArA", 1, 4},  // Bit 4: TARA
  {"tA  ", 1, 5},  // Bit 5: TA
  {"Stby", 1, 6},  // Bit 6: STBY
  {"tHrt", 1, 7},  // Bit 7: THRT
};

// IN2 button definitions (2nd Shifter, 8 total) — Numeric keypad
const ButtonDef buttons_IN2[8] = {
  {"0   ", 0, 0},  // Bit 0: 0
  {"1   ", 0, 0},  // Bit 1: 1
  {"2   ", 0, 0},  // Bit 2: 2
  {"3   ", 0, 0},  // Bit 3: 3
  {"4   ", 0, 0},  // Bit 4: 4
  {"5   ", 0, 0},  // Bit 5: 5
  {"6   ", 0, 0},  // Bit 6: 6
  {"7   ", 0, 0},  // Bit 7: 7
};

// Button definitions for IN3 (direct pins)
const ButtonDef buttons_IN3[8] = {
  {"On  ", 0, 0},  // Bit 0: ON
  {"SyS2", 0, 0},  // Bit 1: SYS2
  {"SyS1", 0, 0},  // Bit 2: SYS1
  {"IdEnt", 2, 0},  // Bit 3: IDENT — direct LED (ATC FAIL)
  {"AUtO", 0, 0},  // Bit 4: AUTO
  {"CLr ", 0, 0},  // Bit 5: CLR
  {"Stby", 0, 0},  // Bit 6: STBY
  {"UnUSEd",0, 0}, // Bit 7: free
};

const ButtonDef* getButtonDef(uint8_t bitIndex, uint8_t registerNum) {
  switch (registerNum) {
    case 1:  return &buttons_IN1[bitIndex];
    case 2:  return &buttons_IN2[bitIndex];
    default: return &buttons_IN3[bitIndex];
  }
}

void displayButtonPressed(uint8_t bitIndex, uint8_t registerNum) {
  const ButtonDef *btn = getButtonDef(bitIndex, registerNum);
  displayText4 = btn->name;
  updateDisplay4(displayText4);
}

void displayButtonReleased(uint8_t bitIndex, uint8_t registerNum) {
  displayText4 = "rEL ";
  updateDisplay4(displayText4);
}

void controlButtonLED(uint8_t bitIndex, uint8_t registerNum, bool pressed) {
  const ButtonDef *btn = getButtonDef(bitIndex, registerNum);
  
  // Check for direct LED pin (ATC FAIL) — ledRegister == 2
  if (btn->ledRegister == 2) {
    desiredAtcFailLed = pressed;
    applyLEDOutputs();
    return;
  }
  
  // Shift register backlight LEDs: ledRegister == 1
  if (btn->ledRegister == 1) {
    uint8_t ledMask = 1 << btn->ledBit;
    if (pressed) {
      desiredBacklight |= ledMask;
    } else {
      desiredBacklight &= ~ledMask;
    }
    applyLEDOutputs();
    return;
  }
  
  // ledRegister == 0: no LED
}

// ============================================================================
// BOOT / CONNECTION STATE
// ============================================================================
enum BootState { BOOT_INIT, BOOT_RUNNING, BOOT_DIAG_MENU, BOOT_DIAG_TEST };
BootState bootState = BOOT_INIT;
unsigned long bootSequenceStart = 0;

// External state control (from host via STATE:00/01)
bool hostOnline = false;  // false = offline, true = online
String offlineMessage = "noFS";       // 4 chars for offline indicator
String offlineMessageClear = "    ";  // 4 spaces for clear
unsigned long offlineDisplayStart = 0;
const unsigned long OFFLINE_DISPLAY_DURATION_MS = 10000;
bool offlineMessageShown = false;

// DIAG mode state (menu & test)
unsigned long diagStartTime = 0;
int diagStage = 0;
bool diagComboWasActive = false;

// Boot complete timestamp for DIAG timeout (30s after boot)
unsigned long bootCompleteTime = 0;
const unsigned long DIAG_AVAILABLE_MS = 30000;  // 30 seconds

// Menu related state
bool menuInitialized = false;
int menuIndex = 0;        // 0..2 main menu (no encoder, navigate via buttons)
int menuMode = 0;         // 0 = main menu, 1 = LED test, 2 = button test, 3 = show info, 4 = segment test
unsigned long lastMenuActivityTs = 0;
const unsigned long MENU_INACTIVITY_TIMEOUT_MS = 15000;

// DIAG navigation: use CLR button as "Enter", IN2 bit 2 ("2") as Next/Up, IN2 bit 0 ("0") as Prev/Down
bool lastClrPressed = false;
bool lastUpPressed = false;
bool lastDownPressed = false;
unsigned long lastClrPressTime = 0;

// If a full diag test was requested from the menu, return to menu afterwards
bool diagReturnToMenu = false;

// ============================================================================
// RUNTIME BUFFERS / FLAGS
// ============================================================================
String serialAccum = "";
bool forceSendNext = false;

// ============================================================================
// HELPER FUNCTIONS - STRING FORMATTING
// ============================================================================
String bin16(uint16_t w) {
  String s = "";
  for (int i = 15; i >= 0; i--) s += ((w & (1 << i)) ? '1' : '0');
  return s;
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

uint16_t parseBin16(const String &s) {
  uint16_t v = 0;
  for (int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c == '0' || c == '1') v = (v << 1) | (c == '1' ? 1 : 0);
  }
  return v;
}

// Perform a fast software reset using the watchdog
void triggerSoftwareReset(bool showMessage = true) {
  if (showMessage) {
    displayTextFull("rESE", "t");
    delay(1000);
  }
  Serial.flush();
  delay(50);
  wdt_enable(WDTO_15MS);
  while (true) {
    // Wait for watchdog to trigger
  }
}

// ============================================================================
// MAX7219 DISPLAY FUNCTIONS (4-digit, 1 device)
// ============================================================================
void initDisplays() {
  lc.shutdown(DISP_DEVICE, false);       // Wake up
  lc.setIntensity(DISP_DEVICE, displayBrightness);
  lc.clearDisplay(DISP_DEVICE);
}

void setDisplayBrightness(uint8_t brightness) {
  displayBrightness = constrain(brightness, 0, 15);
  lc.setIntensity(DISP_DEVICE, displayBrightness);
}

// Map characters to 7-segment patterns (identical to RMP)
//     a
//    ---
//  f| g |b
//    ---
//  e|   |c
//    ---
//     d   (dp)
// Bit pattern: 0bDPgfedcba
byte charTo7Seg(char c) {
  // Special characters for better display
  if (c == '$') return 0b00000100;  // Custom "m": segments c,e

  // Lowercase special forms
  if (c == 'o') return 0b00011101;
  if (c == 'i') return 0b00010000;
  if (c == 'm') return 0b00010101;
  if (c == 'y') return 0b00111011;
  
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
    case 'R': return 0b00000101;
    case 'S': return 0b01011011;
    case 'T': return 0b00001111;
    case 'U': return 0b00111110;
    case 'X': return 0b00110111;
    case 'Y': return 0b00111011;
    case '-': return 0b00000001;
    case '_': return 0b00001000;
    case '=': return 0b00001001;
    case ' ': return 0b00000000;
    default:  return 0b00000000;
  }
}

void updateDisplay4(const String &text) {
  // text is up to 4 characters; device 0
  // MAX7219: digit 0 is rightmost, digit 3 is leftmost (for 4-digit display)
  int textPos = 0;
  for (int digitPos = 0; digitPos < 4; digitPos++) {
    if (textPos >= text.length()) {
      lc.setRow(DISP_DEVICE, 3 - digitPos, 0b00000000);  // Blank remaining digits
      continue;
    }
    
    char c = text.charAt(textPos);
    
    // Skip decimal points — they're modifiers, not characters
    if (c == '.') {
      textPos++;
      if (textPos >= text.length()) {
        lc.setRow(DISP_DEVICE, 3 - digitPos, 0b00000000);
        continue;
      }
      c = text.charAt(textPos);
    }
    
    byte pattern = charTo7Seg(c);
    
    // Check if next char is a decimal point (applies to current digit)
    if (textPos + 1 < text.length() && text.charAt(textPos + 1) == '.') {
      pattern |= 0b10000000;  // Set DP bit
    }
    
    lc.setRow(DISP_DEVICE, 3 - digitPos, pattern);
    textPos++;
  }
}

// Left-to-right variant used only for XPDR squawk display
// MAX7219: row 0 = rightmost, row 3 = leftmost → 3-digitPos maps first char to left
void updateDisplay4Left(const String &text) {
  int textPos = 0;
  for (int digitPos = 0; digitPos < 4; digitPos++) {
    if (textPos >= text.length()) {
      lc.setRow(DISP_DEVICE, 3 - digitPos, 0b00000000);
      continue;
    }
    char c = text.charAt(textPos);
    if (c == '.') {
      textPos++;
      if (textPos >= text.length()) {
        lc.setRow(DISP_DEVICE, 3 - digitPos, 0b00000000);
        continue;
      }
      c = text.charAt(textPos);
    }
    byte pattern = charTo7Seg(c);
    lc.setRow(DISP_DEVICE, 3 - digitPos, pattern);
    textPos++;
  }
}

void displayTextFull(const String &text) {
  currentFull = text;
  // Determine if scrolling is needed (count effective chars excluding '.')
  auto effectiveLength = [](const String &s) {
    int cnt = 0;
    for (int i = 0; i < s.length(); i++) if (s.charAt(i) != '.') cnt++;
    return cnt;
  };
  scrollEnabled = (effectiveLength(currentFull) > 4);
  scrollOffset = 0;
  lastScrollTs = millis();

  // Build window of up to 4 chars
  auto buildWindow = [](const String &full) {
    String out = "";
    int placed = 0;
    for (int i = 0; i < full.length() && placed < 4; i++) {
      char c = full.charAt(i);
      out += c;
      if (c != '.') placed++;
    }
    while (placed < 4) { out += ' '; placed++; }
    return out;
  };
  String window = buildWindow(currentFull);
  updateDisplay4(window);
  displayText4 = window;
}

// Overload for two-string compatibility with RMP convention (show first 4 of left on 4-digit)
void displayTextFull(const String &left, const String &right) {
  // Combine left+right for the 4-digit display (show left first, then right)
  displayTextFull(left + right);
}

// Advance scroll offset if needed and refresh display
void scrollTick(unsigned long now) {
  if (!scrollEnabled) return;
  if ((now - lastScrollTs) < SCROLL_INTERVAL_MS) return;
  lastScrollTs = now;
  
  auto effectiveLength = [](const String &s) {
    int cnt = 0;
    for (int i = 0; i < s.length(); i++) if (s.charAt(i) != '.') cnt++;
    return cnt;
  };
  
  auto buildWindow = [](const String &full, int offsetChars) {
    String out = "";
    int placed = 0;
    int skipped = 0;
    for (int i = 0; i < full.length() && placed < 4; i++) {
      char c = full.charAt(i);
      if (c != '.') {
        if (skipped < offsetChars) { skipped++; continue; }
        placed++;
      }
      out += c;
    }
    while (placed < 4) { out += ' '; placed++; }
    return out;
  };

  int elen = effectiveLength(currentFull);
  int maxOffset = max(0, elen - 4);
  scrollOffset++;
  if (scrollOffset > maxOffset) scrollOffset = 0;
  String window = buildWindow(currentFull, scrollOffset);
  updateDisplay4(window);
  displayText4 = window;
}

// ============================================================================
// BACKLIGHT DRIVER FUNCTIONS (74HC595)
// ============================================================================
void shiftOutBacklight(uint8_t blBits) {
  digitalWrite(blLatchPin, LOW);
  shiftOut(blDataPin, blClockPin, MSBFIRST, blBits);
  digitalWrite(blLatchPin, HIGH);
}

void applyLEDOutputs() {
  shiftOutBacklight(desiredBacklight);
  // ATC FAIL LED is active-low: LOW (0) = LED ON, HIGH (1) = LED OFF
  // When host is offline and no LED cmd received yet → LED forced ON (simulates ATC FAIL)
  // Once host sends any LED/DSP/BL cmd or STATE:01 → LED controlled by LED2
  bool atcFailOn;
  if (ledTestActive) {
    atcFailOn = desiredAtcFailLed;
  } else {
    atcFailOn = desiredAtcFailLed || !hostOnline;
  }
  digitalWrite(ledAtcFail, atcFailOn ? LOW : HIGH);
  analogWrite(pwmBrightness, desiredBrightness);
  
  hwBacklight = desiredBacklight;
  hwAtcFailLed = desiredAtcFailLed;
  hwBrightness = desiredBrightness;
}

void setLEDState(uint8_t backlight, bool atcFail, uint8_t brightness) {
  desiredBacklight = backlight;
  desiredAtcFailLed = atcFail;
  desiredBrightness = brightness;
  applyLEDOutputs();
}

// ============================================================================
// SHIFT REGISTER INPUT (HC165, 1x)
// ============================================================================
void readShiftRegisters(uint8_t &in1, uint8_t &in2, uint8_t &in3) {
  // Latch
  digitalWrite(inputLatchPin, LOW);
  delayMicroseconds(20);
  digitalWrite(inputLatchPin, HIGH);
  delayMicroseconds(20);
  
  // Read 2 daisy-chained shift registers (16 bits)
  byte raw1 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);  // 1st shifter
  byte raw2 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);  // 2nd shifter
  in1 = ~raw1;  // Invert if buttons are active-low
  in2 = ~raw2;
  
  // Takt-Stabilisierung (wie RMP)
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
  
  // Read direct input pins into IN3 (buttons active-low with INPUT_PULLUP → invert)
  in3 = 0x00;
  if (!digitalRead(pinOn))    in3 |= (1 << 0);
  if (!digitalRead(pinSys2))  in3 |= (1 << 1);
  if (!digitalRead(pinSys1))  in3 |= (1 << 2);
  if (!digitalRead(pinIdent)) in3 |= (1 << 3);
  if (!digitalRead(pinAuto))  in3 |= (1 << 4);
  if (!digitalRead(pinClr))   in3 |= (1 << 5);
  if (!digitalRead(pinStby))  in3 |= (1 << 6);
  // Bit 7 remains 0
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
// XPDR DISPLAY ROUTING — selects dsp1 (complete) or dsp3 (partial) via dsp2
// ============================================================================
void routeXpdrDisplay() {
  // dsp2 tells how many characters are entered (0..4)
  //   0..3 → show first N chars of dsp3 (STBY) on leftmost digits
  //   4    → show dsp1 (active XPDR) on all 4 digits
  //   never received → fall back to dsp1 (backward compatible)
  
  String source;
  int showDigits;
  
  if (dsp2Active && dsp2Value >= 0 && dsp2Value <= 3) {
    source = dsp3Value;        // partial entry
    showDigits = dsp2Value;    // show exactly this many digits from the left
  } else {
    source = dsp1Value;        // complete squawk (or fallback)
    showDigits = 4;            // all 4 digits
  }
  
  // Negative values → display OFF (XPDR inactive)
  if (source.startsWith("-")) {
    lc.shutdown(DISP_DEVICE, true);
    displayText4 = "    ";
    return;
  }
  
  lc.shutdown(DISP_DEVICE, false);
  
  // Strip decimal point — XPDR frequency is always integer
  source.replace(".", "");
  
  // Build exactly showDigits characters, left-aligned, rest blank
  String out = "";
  for (int i = 0; i < 4; i++) {
    if (i < showDigits && i < source.length()) {
      out += source.charAt(i);
    } else {
      out += ' ';
    }
  }
  
  displayText4 = out;
  updateDisplay4Left(out);
}

// ============================================================================
// SERIAL COMMAND PARSING (RMP-compatible protocol)
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
        if (bootState == BOOT_RUNNING && (millis() - bootCompleteTime) < DIAG_AVAILABLE_MS) {
          bootState = BOOT_DIAG_MENU;
          diagStartTime = millis();
          diagStage = 0;
          Serial.println("DIAG:START;");
        } else if (bootState == BOOT_RUNNING) {
          Serial.println("DIAG:LOCKED;");  // DIAG timeout expired
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
      // LED1: backlight shift register (8 bits)
      if (!hostOnline) { hostOnline = true; }  // implicit online
      desiredBacklight = parseBin8(val);
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("LED2")) {
      // LED2 bit 0 = ATC FAIL
      if (!hostOnline) { hostOnline = true; }  // implicit online
      uint8_t bits = parseBin8(val);
      desiredAtcFailLed = bits & 0b00000001;
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("BL")) {
      if (!hostOnline) { hostOnline = true; }  // implicit online
      desiredBrightness = constrain(val.toInt(), 0, 255);
      applyLEDOutputs();
    }
    else if (key.equalsIgnoreCase("DISP_BL")) {
      if (!hostOnline) { hostOnline = true; }  // implicit online
      setDisplayBrightness(val.toInt());
    }
    else if (key.equalsIgnoreCase("DSP1")) {
      if (!hostOnline) { hostOnline = true; }  // implicit online
      dsp1Value = val;
      routeXpdrDisplay();
    }
    else if (key.equalsIgnoreCase("DSP2")) {
      // Character count 0..4 from XPDR (internal routing only, no physical display)
      if (!hostOnline) { hostOnline = true; }
      dsp2Value = val.toInt();
      dsp2Active = true;
      routeXpdrDisplay();
    }
    else if (key.equalsIgnoreCase("DSP3")) {
      // Standby / partial squawk entry (shown while dsp2 = 0..3)
      if (!hostOnline) { hostOnline = true; }
      dsp3Value = val;
      routeXpdrDisplay();
    }
    else if (key.equalsIgnoreCase("STATE")) {
      // STATE:00 = offline, STATE:01 = online
      hostOnline = (val == "01" || val == "1");
      // applyLEDOutputs auto-forces ATC FAIL LED when offline
      if (hostOnline && displayText4 == offlineMessage) {
        displayTextFull("    ");
      }
    }
    else if (key.equalsIgnoreCase("REQ")) {
      forceSendNext = true;
    }
    else if (key.equalsIgnoreCase("VER")) {
      sendIdentAndState();
    }
    else if (key.equalsIgnoreCase("CFG")) {
      // Parse configuration: CFG:DEB15;LED1000;DSP800;REG:D-A320
      String cfgStr = val;
      int pos = 0;
      while (pos < cfgStr.length()) {
        int nextSep = cfgStr.indexOf(';', pos);
        if (nextSep < 0) nextSep = cfgStr.length();
        String param = cfgStr.substring(pos, nextSep);
        param.trim();
        
        if (param.startsWith("DEB")) {
          CFG_BUTTON_DEBOUNCE = constrain(param.substring(3).toInt(), 0, 99);
        }
        else if (param.startsWith("LED")) {
          CFG_LED_REFRESH = constrain(param.substring(3).toInt(), 100, 10000);
        }
        else if (param.startsWith("DSP")) {
          CFG_DISPLAY_REFRESH = constrain(param.substring(3).toInt(), 100, 10000);
        }
        else if (param.startsWith("REG:")) {
          CFG_AIRCRAFT_REG = param.substring(4);
          if (CFG_AIRCRAFT_REG.length() > 8) CFG_AIRCRAFT_REG = CFG_AIRCRAFT_REG.substring(0, 8);
        }
        else if (param.startsWith("SN:")) {
          String snVal = param.substring(3);
          if (snVal.length() == 8) { snVal.toUpperCase(); for (int i=0;i<8;i++) CFG_SERIAL_NUMBER[i]=snVal.charAt(i); CFG_SERIAL_NUMBER[8]=0; }
        }
        
        pos = nextSep + 1;
      }
    }
    // --- SET commands (case-insensitive key check) ---
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
          if (valid) { CFG_AIRCRAFT_REG = arg; Serial.println("SET ACID:OK ;"); }
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
      else if (cmd == "EXIT") {
        settingsEnabled = false; Serial.println("SET EXIT:OK ;");
      }
      else if (cmd == "WRI" && settingsEnabled) {
        if (arg.equalsIgnoreCase("YES")) {
          saveHWInfo(); settingsEnabled = false;
          Serial.println("SET WRI:OK ;"); triggerSoftwareReset();
        } else Serial.println("SET WRI:FORMAT ;");
      }
      else if (cmd == "WRITE") {
        if (settingsEnabled) { saveHWInfo(); settingsEnabled = false; Serial.println("WRITE:OK ;"); }
        else Serial.println("WRITE:LOCKED ;");
      }
    }
  }
}

// ============================================================================
// EEPROM STORAGE LAYOUT
// ============================================================================
const int EEPROM_BASE_ADDR = 0;
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 3; // v3: +serial_number (9 bytes)
// Layout:
// 0-1: magic (uint16)
// 2: version (uint8)
// 3-10: aircraft_reg[8]
// 11-18: pcb_version[8]
// 19-27: serial_number[9] (8 hex chars + null)
// 28: checksum (uint8)

// ============================================================================
// SERIAL TOKEN PROCESSING
// ============================================================================
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
        displayTextFull(" UEr");
        delay(500);
        displayTextFull("    ");
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
        if (bootState == BOOT_RUNNING && (millis() - bootCompleteTime) < DIAG_AVAILABLE_MS) {
          bootState = BOOT_DIAG_MENU;
          diagStartTime = millis();
          diagStage = 0;
          menuInitialized = false;
          Serial.println("DIAG:MENU_START;");
        } else if (bootState == BOOT_RUNNING) {
          Serial.println("DIAG:LOCKED;");  // DIAG timeout expired
        }
        continue;
      }
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
// STATUS SENDING
// ============================================================================
void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;
  
  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";");
  Serial.print("IN2:"); Serial.print(bin8(inputState2)); Serial.print(";");
  Serial.print("IN3:"); Serial.print(bin8(inputState3)); Serial.print(";");
  Serial.println();
  
  lastSendTs = now;
  forceSendNext = false;
}

void sendStatusImmediate() {
  // Force send immediately, bypassing throttle
  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";");
  Serial.print("IN2:"); Serial.print(bin8(inputState2)); Serial.print(";");
  Serial.print("IN3:"); Serial.print(bin8(inputState3)); Serial.print(";");
  Serial.println();
  
  lastSendTs = millis();
}

// ============================================================================
// BOOT SEQUENCE (non-blocking, millis-based)
// ============================================================================
void runBootSequence() {
  ledTestActive = true;  // Boot LED tests have direct ATC FAIL control
  unsigned long elapsed = millis() - bootSequenceStart;
  
  // Stage 1: Show "boot" for ~3 seconds (0-3000ms)
  if (elapsed < 3000) {
    setLEDState(0x00, false, 0);
    displayTextFull("boot");
  }
  // Stage 2: Segment Dance animation (~1000ms, 3000-4000ms)
  else if (elapsed < 4000) {
    setLEDState(0x00, false, 0);
    unsigned long danceElapsed = elapsed - 3000;
    doSegmentDanceStep(danceElapsed);
  }
  // Stage 3: LED Cycle — cycle through all backlight bits + ATC FAIL (~1000ms, 4000-5000ms)
  else if (elapsed < 5000) {
    displayTextFull("LED ");
    unsigned long ledElapsed = elapsed - 4000;
    doLedCycleStep(ledElapsed);
  }
  // Stage 4: All off, sequence complete (5000ms+)
  else {
    ledTestActive = false;
    setLEDState(0x00, false, 0);
    displayTextFull("    ");
    bootState = BOOT_RUNNING;
    bootCompleteTime = millis();     // record when boot finished for DIAG timeout
    forceSendNext = true;
    maybeSendIdentStartup();
  }
}

// Segment dance: elegant animation lighting individual segments across all 4 digits
void doSegmentDanceStep(unsigned long elapsed) {
  // Phase timing: each segment shown for ~120ms
  int segIndex = (elapsed / 120) % 8;
  byte segPattern = 1 << segIndex;
  
  // Light up the current segment on all 4 digits
  for (int d = 0; d < 4; d++) {
    lc.setRow(DISP_DEVICE, d, segPattern);
  }
}

// LED cycle: walk through all backlight LEDs + ATC FAIL LED
void doLedCycleStep(unsigned long elapsed) {
  // 150ms per LED, 9 LEDs (8 backlight + 1 ATC FAIL)
  int ledIndex = (elapsed / 150) % 9;
  
  if (ledIndex < 8) {
    // Backlight shift register LEDs
    setLEDState(1 << ledIndex, false, 128);
  } else {
    // ATC FAIL LED
    setLEDState(0x00, true, 128);
  }
}

// ============================================================================
// OFFLINE DISPLAY (non-blocking)
// ============================================================================
void updateOfflineDisplay() {
  unsigned long now = millis();
  
  if (!hostOnline) {
    if (!offlineMessageShown && displayText4 == "    ") {
      displayTextFull(offlineMessage);
      offlineDisplayStart = now;
      offlineMessageShown = true;
    }
    else if (offlineMessageShown && (now - offlineDisplayStart) >= OFFLINE_DISPLAY_DURATION_MS) {
      if (displayText4 == offlineMessage) {
        displayTextFull("    ");
      }
    }
  } else {
    offlineMessageShown = false;
  }
}

void reactivateOfflineMessage() {
  if (!hostOnline && !offlineMessageShown) {
    if (displayText4 == "    ") {
      displayTextFull(offlineMessage);
      offlineDisplayStart = millis();
      offlineMessageShown = true;
      Serial.println("OFFLINE:MESSAGE_REACTIVATED;");
    }
  }
}

// ============================================================================
// BUTTON TEST — Tight loop for instant response
// ============================================================================
void runButtonTest() {
  static bool headerShown = false;
  static unsigned long testStartTime = 0;
  static unsigned long diagBlinkStart = 0;
  static bool diagBlinkActive = false;
  static String diagAlt1 = "";
  static String diagAlt2 = "";
  static unsigned long diagBlinkDuration = 0;
  static bool lastKnownPressed = false;
  
  if (!headerShown) {
    displayTextFull("PrSS");
    testStartTime = millis();
    headerShown = true;
    readShiftRegisters(lastInputState1, lastInputState2, lastInputState3);
  }
  
  unsigned long now = millis();
  unsigned long lastButtonActivityTs = now;
  
  // Tight loop — no throttle, runs as fast as possible
  while (true) {
    now = millis();
    
    // --- Handle DIAG blink animation (non-blocking) ---
    if (diagBlinkActive) {
      if (now - diagBlinkStart >= diagBlinkDuration) {
        diagBlinkActive = false;
        displayTextFull("PrSS");
      } else {
        unsigned long phase = now - diagBlinkStart;
        // Toggle every 400ms
        if ((phase / 400) % 2 == 0) {
          displayTextFull(diagAlt1);
        } else {
          displayTextFull(diagAlt2);
        }
      }
    }
    
    // Read inputs directly — no debounce
    uint8_t currentIn1;
    uint8_t currentIn2;
    uint8_t currentIn3;
    readShiftRegisters(currentIn1, currentIn2, currentIn3);
    
    // Check for button state changes in IN1 (8 bits)
    if (currentIn1 != lastInputState1) {
      uint8_t changed = currentIn1 ^ lastInputState1;
      
      for (int i = 0; i < 8; i++) {
        if (changed & (1 << i)) {
          bool isPressed = (currentIn1 >> i) & 1;
          
          if (isPressed) {
            diagAlt1 = getButtonDef(i, 1)->name; diagAlt2 = "PrSS";
            diagBlinkStart = now; diagBlinkDuration = 1200; diagBlinkActive = true;
          } else {
            diagAlt1 = getButtonDef(i, 1)->name; diagAlt2 = "rlSt";
            diagBlinkStart = now; diagBlinkDuration = 1200; diagBlinkActive = true;
          }
          lastButtonActivityTs = now;
        }
      }
      
      lastInputState1 = currentIn1;
    }
    
    // Check for button state changes in IN2 (8 bits, 2nd shifter)
    if (currentIn2 != lastInputState2) {
      uint8_t changed = currentIn2 ^ lastInputState2;
      
      for (int i = 0; i < 8; i++) {
        if (changed & (1 << i)) {
          bool isPressed = (currentIn2 >> i) & 1;
          
          if (isPressed) {
            diagAlt1 = getButtonDef(i, 2)->name; diagAlt2 = "PrSS";
            diagBlinkStart = now; diagBlinkDuration = 1200; diagBlinkActive = true;
          } else {
            diagAlt1 = getButtonDef(i, 2)->name; diagAlt2 = "rlSt";
            diagBlinkStart = now; diagBlinkDuration = 1200; diagBlinkActive = true;
          }
          lastButtonActivityTs = now;
        }
      }
      
      lastInputState2 = currentIn2;
    }
    
    // Check for button state changes in IN3 (8 bits, direct pins)
    if (currentIn3 != lastInputState3) {
      uint8_t changed = currentIn3 ^ lastInputState3;
      
      for (int i = 0; i < 8; i++) {
        if (changed & (1 << i)) {
          bool isPressed = (currentIn3 >> i) & 1;
          
          if (isPressed) {
            diagAlt1 = getButtonDef(i, 3)->name; diagAlt2 = "PrSS";
            diagBlinkStart = now; diagBlinkDuration = 1200; diagBlinkActive = true;
          } else {
            diagAlt1 = getButtonDef(i, 3)->name; diagAlt2 = "rlSt";
            diagBlinkStart = now; diagBlinkDuration = 1200; diagBlinkActive = true;
          }
          lastButtonActivityTs = now;
        }
      }
      
      lastInputState3 = currentIn3;
    }
    
    // If all buttons released and 1 second passed, show "PrSS"
    if ((currentIn1 == 0) && (currentIn2 == 0) && (currentIn3 == 0) && (now - lastButtonActivityTs >= 1000)) {
      displayTextFull("PrSS");
      lastButtonActivityTs = now + 10000;
    }
    
    // CLR button double-click to exit (CLR = IN3 bit 5)
    static unsigned long lastClrPressTs = 0;
    static bool clrWasPressed = false;
    bool clrPressed = (currentIn3 & (1 << 5));
    
    if (clrPressed && !clrWasPressed) {
      if (now - lastClrPressTs < 500) {
        delay(200);
        ledTestActive = false;
        setLEDState(0x00, false, 0);
        displayTextFull("    ");
        diagBlinkActive = false;
        menuMode = 0;
        menuInitialized = false;
        headerShown = false;
        return;
      }
      lastClrPressTs = now;
    }
    clrWasPressed = clrPressed;
    
    delayMicroseconds(500);
  }
}

// ============================================================================
// SHIFT REGISTER TEST — Live display of IN1/IN2 states (4-digit)
// ============================================================================
void runShiftRegTest() {
  static bool headerShown = false;
  static unsigned long lastToggleTs = 0;
  static bool showIN1 = true;
  
  if (!headerShown) {
    displayTextFull("ShIF");
    delay(1000);
    headerShown = true;
    lastToggleTs = millis();
  }
  
  unsigned long now = millis();
  
  while (true) {
    now = millis();
    
    uint8_t currentIn1;
    uint8_t currentIn2;
    uint8_t currentIn3;
    readShiftRegisters(currentIn1, currentIn2, currentIn3);
    
    // Toggle between IN1, IN2, IN3 display every 1.5 seconds
    if (now - lastToggleTs >= 1500) {
      showIN1 = !showIN1;
      lastToggleTs = now;
    }
    
    if (showIN1) {
      String disp = "1:";
      disp += (currentIn1 & 0x80) ? "1" : "0";
      disp += (currentIn1 & 0x40) ? "1" : "0";
      disp += (currentIn1 & 0x20) ? "1" : "0";
      displayTextFull(disp);
    } else {
      // Alternate IN2 and IN3
      String disp = "2:";
      disp += (currentIn2 & 0x80) ? "1" : "0";
      disp += (currentIn2 & 0x40) ? "1" : "0";
      disp += (currentIn2 & 0x20) ? "1" : "0";
      displayTextFull(disp);
    }
    
    // CLR double-click to exit
    static unsigned long lastClrPressTs = 0;
    static bool clrWasPressed = false;
    bool clrPressed = (currentIn3 & (1 << 5));
    
    if (clrPressed && !clrWasPressed) {
      if (now - lastClrPressTs < 500) {
        delay(200);
        displayTextFull("    ");
        menuMode = 0;
        menuInitialized = false;
        headerShown = false;
        return;
      }
      lastClrPressTs = now;
    }
    clrWasPressed = clrPressed;
    
    delayMicroseconds(500);
  }
}

// ============================================================================
// DIAG MODE — SIMPLIFIED MENU (button-navigated, no rotary encoder)
// ============================================================================
void runDiagMenu() {
  unsigned long now = millis();
  
  // Poll inputs every iteration
  readShiftRegisters(inputState1, inputState2, inputState3);
  
  // Debounce inputs
  if (inputState1 != lastInputState1 || inputState2 != lastInputState2 || inputState3 != lastInputState3) {
    if (now - lastDebounceTs >= CFG_BUTTON_DEBOUNCE) {
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      lastInputState3 = inputState3;
      reactivateOfflineMessage();
    }
  }
  
  // Initialize menu on first entry
  if (!menuInitialized) {
    menuInitialized = true;
    menuIndex = 0;
    menuMode = 0;
    lastClrPressed = false;
    lastUpPressed = false;
    lastDownPressed = false;
    displayTextFull("dIAG");
    lastMenuActivityTs = now;
    return;
  }
  
  // Keep header visible for 2 seconds
  if ((now - diagStartTime) < 2000) return;
  
  // Auto-exit after inactivity
  if ((now - lastMenuActivityTs) >= MENU_INACTIVITY_TIMEOUT_MS) {
    ledTestActive = false;
    setLEDState(0x00, false, 0);
    displayTextFull("    ");
    bootState = BOOT_RUNNING;
    menuInitialized = false;
    return;
  }
  
  // Read navigation buttons (using debounced values)
  // CLR = IN3 bit 5 (Enter), "2" = IN2 bit 2 (Up/Next), "0" = IN2 bit 0 (Down/Prev)
  bool clrPressed = (lastInputState3 & (1 << 5)) != 0;     // CLR = Enter
  bool upPressed  = (lastInputState2 & (1 << 2)) != 0;     // "2" = Next/Up
  bool downPressed = (lastInputState2 & (1 << 0)) != 0;    // "0" = Prev/Down
  
  // ===== MAIN MENU (menuMode == 0) =====
  if (menuMode == 0) {
    // Navigate via "2" (next) and "0" (prev)
    if (upPressed && !lastUpPressed) {
      menuIndex++;
      if (menuIndex > 4) menuIndex = 0;
      lastUpPressed = true;
      lastMenuActivityTs = now;
    }
    if (!upPressed) lastUpPressed = false;
    
    if (downPressed && !lastDownPressed) {
      menuIndex--;
      if (menuIndex < 0) menuIndex = 4;
      lastDownPressed = true;
      lastMenuActivityTs = now;
    }
    if (!downPressed) lastDownPressed = false;
    
    // Display current menu item
    switch (menuIndex) {
      case 0: displayTextFull("LEd "); break;   // LED Test
      case 1: displayTextFull("bUtn"); break;   // Button Test
      case 2: displayTextFull("SEG "); break;   // Segment Test
      case 3: displayTextFull("InFo"); break;   // Info
      case 4: displayTextFull("Exit"); break;    // Exit
    }
    
    // CLR (Enter) handling
    if (clrPressed && !lastClrPressed) {
      lastClrPressed = true;
      lastClrPressTime = now;
      lastMenuActivityTs = now;
      
      switch (menuIndex) {
        case 0:
          // LED Test submenu — run LED walk
          ledTestActive = true;
          menuMode = 1;
          diagStartTime = now;
          break;
        case 1:
          // Button Test
          menuMode = 2;
          break;
        case 2:
          // Segment Test
          ledTestActive = true;
          menuMode = 4;
          diagStartTime = now;
          break;
        case 3:
          // Show Info
          menuMode = 3;
          break;
        case 4:
          // Exit
          ledTestActive = false;
          setLEDState(0x00, false, 0);
          displayTextFull("    ");
          bootState = BOOT_RUNNING;
          menuInitialized = false;
          return;
      }
      return;
    }
    if (!clrPressed) lastClrPressed = false;
    return;
  }
  
  // ===== LED TEST (menuMode == 1) =====
  if (menuMode == 1) {
    unsigned long testElapsed = now - diagStartTime;
    
    if (testElapsed < 8000) {
      do_led_walk_step(testElapsed);
      // Alternate "LEd " / "run " every 500ms
      if (((now / 500) % 2) == 0) {
        displayTextFull("LEd ");
      } else {
        displayTextFull("run ");
      }
    } else {
      ledTestActive = false;
      setLEDState(0x00, false, 0);
      menuMode = 0;
      menuInitialized = false;
    }
    return;
  }
  
  // ===== BUTTON TEST (menuMode == 2) =====
  if (menuMode == 2) {
    runButtonTest();
    return;
  }
  
  // ===== INFO DISPLAY (menuMode == 3) =====
  if (menuMode == 3) {
    static bool infoTextSet = false;
    if (!infoTextSet) {
      displayTextFull(CFG_AIRCRAFT_REG);
      infoTextSet = true;
    }
    scrollTick(now);  // advance scroll for long registrations
    
    // Exit on CLR
    if (clrPressed && !lastClrPressed) {
      lastClrPressed = true;
      infoTextSet = false;
      menuMode = 0;
      menuInitialized = false;
      return;
    }
    if (!clrPressed) lastClrPressed = false;
    return;
  }
  
  // ===== SEGMENT TEST (menuMode == 4) =====
  if (menuMode == 4) {
    unsigned long testElapsed = now - diagStartTime;
    
    if (testElapsed < 6000) {
      // Run segment sweep
      do_segment_sweep_step_4dig(testElapsed);
    } else {
      // Clear and return
      ledTestActive = false;
      lc.clearDisplay(DISP_DEVICE);
      menuMode = 0;
      menuInitialized = false;
    }
    return;
  }
  
  scrollTick(now);
}

// LED walk step (shared between menu and full diag)
void do_led_walk_step(unsigned long elapsed) {
  int ledIndex = (elapsed / 200) % 9;
  if (ledIndex < 8) {
    setLEDState(1 << ledIndex, false, 128);
  } else {
    setLEDState(0x00, true, 128);
  }
}

// Segment sweep for 4-digit display
void do_segment_sweep_step_4dig(unsigned long elapsed) {
  if (elapsed < 500) return;  // brief pause
  
  int segIndex = ((elapsed - 500) / 250) % 8;
  byte segPattern = 1 << segIndex;
  for (int digit = 0; digit < 4; digit++) {
    lc.setRow(DISP_DEVICE, digit, segPattern);
  }
}

// ============================================================================
// FULL DIAG SEQUENCE (runFullDiagSequence)
// ============================================================================
void runFullDiagSequence() {
  ledTestActive = true;  // Full DIAG LED tests have direct ATC FAIL control
  unsigned long elapsed = millis() - diagStartTime;
  
  // Stage 0: "dIAG" message (0-2000ms)
  if (elapsed < 2000) {
    setLEDState(0x00, false, 0);
    displayTextFull("dIAG");
  }
  // Stage 1: Digit count 0-9 with DP sweep (2000-6000ms)
  else if (elapsed < 6000) {
    setLEDState(0x00, false, 0);
    do_display_count_step_4dig(elapsed - 2000);
  }
  // Stage 2: Full LED test — all backlight + ATC FAIL on (6000-8000ms)
  else if (elapsed < 8000) {
    setLEDState(0xFF, true, 255);
    displayTextFull("8888");
  }
  // Stage 3: Shift register input test (8000-12000ms)
  else if (elapsed < 12000) {
    setLEDState(0x00, false, 128);
    
    uint8_t in1;
    uint8_t in2;
    uint8_t in3;
    readShiftRegisters(in1, in2, in3);
    
    // Show IN1 high nibble as binary on 4 digits
    String disp = "";
    for (int i = 7; i >= 4; i--) {
      disp += (in1 & (1 << i)) ? "1" : "0";
    }
    displayTextFull(disp);
    
    Serial.print("DIAG:IN1="); Serial.print(bin8(in1));
    Serial.print(";IN2="); Serial.print(bin8(in2));
    Serial.print(";IN3="); Serial.println(bin8(in3));
  }
  // Stage 4: LED walk pattern (12000-16000ms)
  else if (elapsed < 16000) {
    // Alternate "LEd " / "run " every 500ms
    if (((elapsed / 500) % 2) == 0) {
      displayTextFull("LEd ");
    } else {
      displayTextFull("run ");
    }
    do_led_walk_step(elapsed - 12000);
  }
  // Stage 5: Segment test (16000-20000ms)
  else if (elapsed < 20000) {
    setLEDState(0x00, false, 128);
    if (elapsed < 17000) {
      // brief pause before sweep
    } else {
      do_segment_sweep_step_4dig(elapsed - 17000 + 500);
    }
  }
  // Stage 6: Brightness fade test (20000-24000ms)
  else if (elapsed < 24000) {
    displayTextFull("brIG");
    setLEDState(0xFF, true, 255);
    do_brightness_fade_step(elapsed - 20000);
  }
  // Stage 7: "PASS" message (24000-27000ms)
  else if (elapsed < 27000) {
    setLEDState(0x00, false, 128);
    displayTextFull("PASS");
  }
  // Stage 8: Return to running
  else {
    ledTestActive = false;
    setDisplayBrightness(displayBrightness);
    setLEDState(0x00, false, 0);
    displayTextFull("    ");
    if (diagReturnToMenu) {
      setLEDState(0x00, false, 0);
      displayTextFull("    ");
      bootState = BOOT_DIAG_MENU;
      menuInitialized = false;
      diagReturnToMenu = false;
    } else {
      bootState = BOOT_RUNNING;
    }
    diagComboWasActive = false;
    offlineMessageShown = false;
    Serial.println("DIAG:COMPLETE;");
  }
}

// Display count step for 4-digit display
void do_display_count_step_4dig(unsigned long elapsed) {
  int digit = (elapsed / 400) % 10;
  int dpPos = (elapsed / 400) % 4;
  
  String disp = "";
  for (int i = 0; i < 4; i++) {
    disp += String(digit);
    if (i == dpPos) disp += ".";
  }
  displayTextFull(disp);
}

// Brightness fade step (shared)
void do_brightness_fade_step(unsigned long elapsed) {
  int fadePos = elapsed % 2000;
  int brightness = (fadePos < 1000) ? (fadePos / 4) : (255 - ((fadePos - 1000) / 4));
  
  analogWrite(pwmBrightness, brightness);
  lc.setIntensity(DISP_DEVICE, brightness / 17);
}

// Check for DIAG combo: CLR (IN3 bit 5) + "7" (IN2 bit 7) pressed simultaneously
// Only available within DIAG_AVAILABLE_MS (30s) after boot
void checkDiagCombo() {
  bool clrActive = (inputState3 & (1 << 5)) != 0;
  bool key7Active = (inputState2 & (1 << 7)) != 0;
  bool comboActive = clrActive && key7Active;
  
  if (bootState == BOOT_RUNNING && comboActive && !diagComboWasActive) {
    if ((millis() - bootCompleteTime) < DIAG_AVAILABLE_MS) {
      bootState = BOOT_DIAG_MENU;
      diagStartTime = millis();
      diagStage = 0;
      Serial.println("DIAG:START;");
      diagComboWasActive = true;
    }
    // If timeout expired, combo is ignored — 7 and CLR still output via normal sendStatusImmediate
  }
  else if (!comboActive) {
    diagComboWasActive = false;
  }
}

// ============================================================================
// SETUP
// ============================================================================

// EEPROM helpers
void generateSerialNumber(char* out) {
  randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3) + micros());
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
  for (int i = 0; i < 8; i++) { regbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+3+i); pcbbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+11+i); }
  regbuf[8] = 0; pcbbuf[8] = 0;
  
  if (ver <= 2) {
    // v1 only had reg at offset 11 (crc), no pcb. Try best-effort load.
    if (ver == 1) {
      uint8_t storedCs = EEPROM.read(EEPROM_BASE_ADDR+11);
      if (storedCs == calcCfgChecksum(ver, regbuf, "        ", "         ")) {
        String regStr = ""; for (int i = 0; i < 8; i++) if (regbuf[i]!=0 && regbuf[i]!=' ') regStr += regbuf[i];
        if (regStr.length()>0) CFG_AIRCRAFT_REG = regStr;
      }
    }
    generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); return;
  }
  if (ver != EEPROM_FORMAT_VERSION) { generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); return; }
  
  char snbuf[10]; for (int i=0;i<9;i++) snbuf[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+19+i); snbuf[9]=0;
  if (EEPROM.read(EEPROM_BASE_ADDR+28) != calcCfgChecksum(ver,regbuf,pcbbuf,snbuf)) { generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); return; }
  
  String regStr = ""; for (int i=0;i<8;i++) if (regbuf[i]!=0 && regbuf[i]!=' ') regStr += regbuf[i];
  if (regStr.length()>0) CFG_AIRCRAFT_REG = regStr;
  String pcbStr = ""; for (int i=0;i<8;i++) pcbStr += (pcbbuf[i]?pcbbuf[i]:' ');
  while (pcbStr.length()>0 && pcbStr.charAt(pcbStr.length()-1)==' ') pcbStr.remove(pcbStr.length()-1);
  if (pcbStr.length()>0) strncpy(CFG_PCB_VERSION, pcbStr.c_str(), 8);
  
  int si=0; for (int i=0;i<8;i++) if (snbuf[i]!=0 && snbuf[i]!=' ') CFG_SERIAL_NUMBER[si++]=snbuf[i];
  CFG_SERIAL_NUMBER[si]=0;
  if (si==0) { generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); }
}

void saveHWInfo() {
  char regbuf[8]; for (int i=0;i<8;i++) regbuf[i]=(i<CFG_AIRCRAFT_REG.length())?CFG_AIRCRAFT_REG.charAt(i):' ';
  char pcbbuf[8]; int plen=strlen(CFG_PCB_VERSION);
  for (int i=0;i<8;i++) pcbbuf[i]=(i<plen)?CFG_PCB_VERSION[i]:' ';
  char snbuf[9]; int slen=strlen(CFG_SERIAL_NUMBER);
  for (int i=0;i<8;i++) snbuf[i]=(i<slen)?CFG_SERIAL_NUMBER[i]:' '; snbuf[8]=0;
  
  uint16_t magic = EEPROM_MAGIC;
  EEPROM.update(EEPROM_BASE_ADDR+0,(uint8_t)(magic&0xFF));
  EEPROM.update(EEPROM_BASE_ADDR+1,(uint8_t)((magic>>8)&0xFF));
  EEPROM.update(EEPROM_BASE_ADDR+2,EEPROM_FORMAT_VERSION);
  for (int i=0;i<8;i++) EEPROM.update(EEPROM_BASE_ADDR+3+i,(uint8_t)regbuf[i]);
  for (int i=0;i<8;i++) EEPROM.update(EEPROM_BASE_ADDR+11+i,(uint8_t)pcbbuf[i]);
  for (int i=0;i<9;i++) EEPROM.update(EEPROM_BASE_ADDR+19+i,(uint8_t)snbuf[i]);
  uint8_t cs=calcCfgChecksum(EEPROM_FORMAT_VERSION,regbuf,pcbbuf,snbuf);
  EEPROM.update(EEPROM_BASE_ADDR+28,cs);
}

void setup() {
  wdt_disable();
  Serial.begin(115200);
  
  // Pin modes — Backlight driver
  pinMode(blLatchPin, OUTPUT);
  pinMode(blClockPin, OUTPUT);
  pinMode(blDataPin, OUTPUT);
  pinMode(pwmBrightness, OUTPUT);
  
  // Pin modes — Direct LED
  pinMode(ledAtcFail, OUTPUT);
  
  // Pin modes — Input shift registers
  pinMode(inputDataPin, INPUT);
  pinMode(inputClockPin, OUTPUT);
  pinMode(inputLatchPin, OUTPUT);
  
  // Pin modes — Direct inputs (INPUT_PULLUP for active-low buttons)
  pinMode(pinOn,    INPUT_PULLUP);
  pinMode(pinSys2,  INPUT_PULLUP);
  pinMode(pinSys1,  INPUT_PULLUP);
  pinMode(pinIdent, INPUT_PULLUP);
  pinMode(pinAuto,  INPUT_PULLUP);
  pinMode(pinClr,   INPUT_PULLUP);
  pinMode(pinStby,  INPUT_PULLUP);
  
  // Initialize displays
  initDisplays();
  
  // Load persisted configuration from EEPROM
  loadHWInfo();
  
  // Initialize LEDs off
  setLEDState(0x00, false, 0);
  
  // Print configuration
  Serial.print("SETUP:Debounce="); Serial.print(CFG_BUTTON_DEBOUNCE);
  Serial.print("ms, LED="); Serial.print(CFG_LED_REFRESH);
  Serial.print("ms, DSP="); Serial.print(CFG_DISPLAY_REFRESH);
  Serial.print("ms, REG="); Serial.println(CFG_AIRCRAFT_REG);
  Serial.print("IDENT:"); Serial.println(PANEL_IDENT);
  
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
  
  // Handle DIAG menu
  if (bootState == BOOT_DIAG_MENU) {
    runDiagMenu();
    return;
  }
  
  // Handle DIAG test sequence
  if (bootState == BOOT_DIAG_TEST) {
    runFullDiagSequence();
    return;
  }
  
  if (now < pauseUntil) return;
  
  processSerialTokensFromHost();
  
  if (now - lastLoopTs < LOOP_INTERVAL_MS) return;
  lastLoopTs = now;
  
  // Read shift registers + direct inputs
  readShiftRegisters(inputState1, inputState2, inputState3);
  
  // Detect button changes (debounced)
  if (inputState1 != lastInputState1 || inputState2 != lastInputState2 || inputState3 != lastInputState3) {
    if (now - lastDebounceTs >= CFG_BUTTON_DEBOUNCE) {
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      lastInputState3 = inputState3;
      
      // Reactivate offline message on input change
      if (inputState1 != prevInputState1 || inputState2 != prevInputState2 || inputState3 != prevInputState3) {
        reactivateOfflineMessage();
        prevInputState1 = inputState1;
        prevInputState2 = inputState2;
        prevInputState3 = inputState3;
      }
      
      sendStatusImmediate();
    }
  }
  
  // Check for DIAG combo (CLR + 7)
  checkDiagCombo();
  
  // Update offline display if needed (non-blocking)
  updateOfflineDisplay();
  
  // Handle scroll tick for long text
  scrollTick(now);
}
