// RMP Control Script, (w) 2025 M. Quatember
// Based on ACP architecture with MAX7219 displays, rotary encoders, and LED sink drivers

#include <LedControl.h>
#include <EEPROM.h>
#include <avr/wdt.h>

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
const char* PANEL_IDENT = "RMP, v1.0 MAQ";
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

// Scrolling support for displays: hold full strings and scroll window if >6 chars
String currentLeftFull = "";
String currentRightFull = "";
int scrollOffsetLeft = 0;
int scrollOffsetRight = 0;
unsigned long lastScrollTs = 0;
const unsigned long SCROLL_INTERVAL_MS = 300; // default scroll speed
bool scrollLeftEnabled = false;
bool scrollRightEnabled = false;

// ============================================================================
// INPUT STATE (shift registers)
// ============================================================================
uint8_t inputState1 = 0x00;
uint8_t inputState2 = 0x00;
uint8_t lastInputState1 = 0x00;
uint8_t lastInputState2 = 0x00;
uint8_t prevInputState1 = 0x00;  // Track previous state to detect actual changes
uint8_t prevInputState2 = 0x00;
uint8_t pendingChanged1 = 0x00; // changes detected at debounce time, consumed by menu tests
uint8_t pendingChanged2 = 0x00;

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
// Menu-specific guard to avoid double-counting when navigating menus
volatile unsigned long rotary1LastMenuEventTs = 0;
const unsigned long ROTARY_MENU_MIN_MS = 180; // minimum ms between accepted rotary increments while in menu (increased to reduce double-count)

// MobiFlight-style rotary encoder configuration
// Format: 0b<RT2_config><RT1_config> where each is 2 bits:
// 00 = No sensitivity (all transitions), 01 = Low (half-step), 10 = Medium (detent), 11 = High (strict)
uint8_t ROTARY_CONFIG = 0b1010;  // Both encoders: Medium sensitivity (detent mode) - Default 10

// Extract individual configs (updated dynamically)
uint8_t RT1_SENSITIVITY = (ROTARY_CONFIG >> 0) & 0b11;  // Bits 0-1
uint8_t RT2_SENSITIVITY = (ROTARY_CONFIG >> 2) & 0b11;  // Bits 2-3

// Configuration parameters (can be changed via CFG: command)
uint8_t CFG_BUTTON_DEBOUNCE = 12;     // Button debounce time in ms (00-99), default 12ms
uint16_t CFG_LED_REFRESH = 1200;      // LED refresh interval in ms, default 1200ms
uint16_t CFG_DISPLAY_REFRESH = 1200;  // Display refresh interval in ms, default 1200ms
String CFG_AIRCRAFT_REG = "D-A320";   // Aircraft registration, default D-A320, max 8 chars
String CFG_PCB_VERSION = "PCB 1.0";   // PCB version, default PCB 1.0, max 8 chars
bool CFG_PCB_IS_12 = false;            // Helper flag: true if PCB version is 1.2

// EEPROM storage layout for configuration
const int EEPROM_BASE_ADDR = 0; // start address
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 2;  // Incremented for new layout
// Layout (offsets): 
// 0-1: magic (uint16)
// 2: version (uint8)
// 3-10: aircraft_reg[8]
// 11-18: pcb_version[8]
// 19: checksum (uint8)

// Editing buffer for Settings->HW ID
char editReg[9] = {0}; // 8 chars + null
int editPos = 0;

// PIN & Settings state (Serial-based, not menu-based)
bool settingsEnabled = false;
const String SETTINGS_PIN = "0815";

// ============================================================================
// TIMING / THROTTLES / DEBOUNCE
// ============================================================================
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;

unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;

unsigned long lastDebounceTs = 0;
// DEBOUNCE_MS now uses CFG_BUTTON_DEBOUNCE (set above in config section)

// ============================================================================
// BUTTON & LED DEFINITIONS FOR BUTTON TEST
// ============================================================================

// Button names and their LED bit positions
// IN1 bits: 0=XFER, 1=VHF1, 2=VHF2, 3=VHF3, 4=HF1, 5=MODE1/SEL, 6=AM, 7=HF2
// IN2 bits: 0=NAV, 1=VOR, 2=ILS, 3=MLS, 4=ADF, 5=UNUSED, 6=UNUSED, 7=UNUSED

struct ButtonDef {
  const char* name;     // Display name (max 6 chars)
  uint8_t ledRegister;  // 1 or 2 (which 8-bit register)
  uint8_t ledBit;       // 0-7 (which bit in that register)
};

// Button definitions for IN1 (register/chip 1 = high byte)
// IN1 bits: 0=XFER, 1=VHF1, 2=VHF2, 3=VHF3, 4=HF1, 5=SEL, 6=AM, 7=HF2
const ButtonDef buttons_IN1[8] = {
  {"XFEr  ", 0, 0},  // Bit 0: XFER - no LED
  {"uhf1  ", 2, 0},  // Bit 1: VHF1 - LED on register 2 bit 0
  {"uhf2  ", 2, 1},  // Bit 2: VHF2 - LED on register 2 bit 1
  {"uhf3  ", 2, 2},  // Bit 3: VHF3 - LED on register 2 bit 2
  {"hf1   ", 2, 3},  // Bit 4: HF1 - LED on register 2 bit 3
  {"SEL   ", 0, 0},  // Bit 5: MODE1/SEL - no LED
  {"an$    ", 2, 4},  // Bit 6: AM - LED on register 2 bit 4
  {"hf2   ", 2, 5},  // Bit 7: HF2 - LED on register 2 bit 5
};

// Button definitions for IN2 (register/chip 2 = low byte)
// IN2 bits: 0=NAV, 1=VOR, 2=ILS, 3=MLS, 4=ADF, 5=ONOFF, 6=UNUSED, 7=UNUSED
const ButtonDef buttons_IN2[8] = {
  {"nAU   ", 2, 6},  // Bit 0: NAV - LED on register 2 bit 6
  {"uor   ", 2, 7},  // Bit 1: VOR - LED on register 2 bit 7
  {"ils   ", 0, 2},  // Bit 2: ILS - Special: direct LED pin (ledIlsSel)
  {"n$ls   ", 0, 3},  // Bit 3: MLS - Special: direct LED pin (ledMlsSel)
  {"adf   ", 1, 6},  // Bit 4: ADF - LED on register 1 bit 6
  {"on-off", 0, 0},  // Bit 5: ON/OFF - no LED
  {"UnUSEd", 0, 0},  // Bit 6: Unused
  {"UnUSEd", 0, 0},  // Bit 7: Unused
};

// Button LED state tracking for fade effects
struct ButtonLEDState {
  uint8_t targetBrightness;  // 0 = off, 255 = full
  uint8_t currentBrightness; // For fade animations
  unsigned long fadeStartTs;
  bool isFading;
};

// Track LED fade state for each button (8 on IN1 + 8 on IN2)
ButtonLEDState buttonLEDStates[16];

void displayButtonPressed(uint8_t bitIndex, uint8_t registerNum) {
  const ButtonDef *btn = (registerNum == 1) ? &buttons_IN1[bitIndex] : &buttons_IN2[bitIndex];
  displayText(btn->name, "PrESEd");
}

void displayButtonReleased(uint8_t bitIndex, uint8_t registerNum) {
  const ButtonDef *btn = (registerNum == 1) ? &buttons_IN1[bitIndex] : &buttons_IN2[bitIndex];
  displayText(btn->name, "rELSEd");
}

void controlButtonLED(uint8_t bitIndex, uint8_t registerNum, bool pressed) {
  const ButtonDef *btn = (registerNum == 1) ? &buttons_IN1[bitIndex] : &buttons_IN2[bitIndex];
  
  // Check for direct LED pins (ILS/MLS) - ledRegister == 0 and ledBit identifies which
  if (btn->ledRegister == 0) {
    if (btn->ledBit == 2) {  // ILS (IN2 bit 2)
      desiredIlsLed = pressed;
      applyLEDOutputs();
    } else if (btn->ledBit == 3) {  // MLS (IN2 bit 3)
      desiredMlsLed = pressed;
      applyLEDOutputs();
    }
    // Otherwise no LED (XFER, SEL, ON/OFF, UNUSED buttons)
    return;
  }
  
  // Shift register LEDs: control via 16-bit desiredLedState
  uint16_t ledMask = (btn->ledRegister == 1) 
    ? (1 << (8 + btn->ledBit))  // High byte (register 1)
    : (1 << btn->ledBit);        // Low byte (register 2)
  
  if (pressed) {
    desiredLedState |= ledMask;   // Set bit ON
  } else {
    desiredLedState &= ~ledMask;  // Clear bit OFF
  }
  applyLEDOutputs();
}

void updateButtonLEDFades() {
  // Update fade animations for button LEDs (called from main loop in menu)
  unsigned long now = millis();
  const unsigned long FADE_DURATION_MS = 300;
  
  for (int i = 0; i < 16; i++) {
    if (!buttonLEDStates[i].isFading) continue;
    
    unsigned long elapsed = now - buttonLEDStates[i].fadeStartTs;
    if (elapsed >= FADE_DURATION_MS) {
      buttonLEDStates[i].currentBrightness = buttonLEDStates[i].targetBrightness;
      buttonLEDStates[i].isFading = (buttonLEDStates[i].targetBrightness > 0);
      if (buttonLEDStates[i].targetBrightness == 0) {
        // Final fade-out: clear the LED bit
        uint8_t registerNum = (i < 8) ? 1 : 2;
        uint8_t bitIndex = i % 8;
        const ButtonDef *btn = (registerNum == 1) ? &buttons_IN1[bitIndex] : &buttons_IN2[bitIndex];
        
        if (btn->ledRegister > 0) {
          uint16_t ledMask = (btn->ledRegister == 1)
            ? (1 << (8 + btn->ledBit))
            : (1 << btn->ledBit);
          desiredLedState &= ~ledMask;
          applyLEDOutputs();
        } else if (registerNum == 2 && bitIndex == 2) {
          desiredIlsLed = false;
          applyLEDOutputs();
        } else if (registerNum == 2 && bitIndex == 3) {
          desiredMlsLed = false;
          applyLEDOutputs();
        }
      }
      continue;
    }
    
    // Interpolate brightness fade
    float progress = (float)elapsed / FADE_DURATION_MS;
    if (buttonLEDStates[i].targetBrightness == 0) {
      // Fade out (255 -> 0)
      buttonLEDStates[i].currentBrightness = (uint8_t)(255 * (1.0 - progress));
    } else {
      // Fade in (0 -> 255)
      buttonLEDStates[i].currentBrightness = (uint8_t)(255 * progress);
    }
  }
}

// ============================================================================
// BOOT / CONNECTION STATE
// ============================================================================
enum BootState { BOOT_INIT, BOOT_RUNNING, BOOT_DIAG_MENU, BOOT_DIAG_TEST };
BootState bootState = BOOT_INIT;
unsigned long bootSequenceStart = 0;

// External state control (from host via STATE:00/01)
bool hostOnline = false;  // false = offline, true = online
String offlineMessage1 = "no FS ";
String offlineMessage2 = "onLinE";
unsigned long offlineDisplayStart = 0;  // When offline message was shown
const unsigned long OFFLINE_DISPLAY_DURATION_MS = 10000;  // Show for 10 seconds
bool offlineMessageShown = false;  // Track if message is currently displayed

// DIAG mode state (menu & test)
unsigned long diagStartTime = 0;
int diagStage = 0;
bool diagComboWasActive = false;  // Track if combo was pressed (prevent re-trigger while held)

// Menu related state
bool menuInitialized = false;
int menuIndex = 0;        // 0..3 main menu
int menuSubIndex = 0;     // submenu selection
int menuMode = 0;         // 0 = main menu, 1 = submenu for option1, 2 = button test, 3 = show info, 4 = settings
int lastMenuRotary1 = 0;
int lastMenuRotary2 = 0;
bool lastXferPressed = false;
unsigned long lastXferPressTime = 0;
const uint8_t BUTTON_XFER_MASK1 = 0x01; // XFER on inputState1 bit 0 (IN1:00000001)
const uint8_t BUTTON_XFER_MASK2 = 0x00; // ignore inputState2 for XFER

// XFER handling guard to avoid double registration
unsigned long lastXferHandledTs = 0;
const unsigned long XFER_IGNORE_MS = 250; // ignore new XFER events within 250ms of handling

// Menu navigation timing / inactivity
unsigned long lastMenuActivityTs = 0;
unsigned long lastMenuNavTs = 0;
const unsigned long MENU_INACTIVITY_TIMEOUT_MS = 10000; // auto-exit menu after 10s inactivity
const unsigned long MENU_NAV_DEBOUNCE_MS = 300; // minimum ms between menu nav steps to avoid oversensitivity
unsigned long menuNavIgnoreUntil = 0; // short window after entering submenu to ignore accumulated rotary events

// If a full diag test was requested from the menu, return to menu afterwards instead of running to BOOT_RUNNING
bool diagReturnToMenu = false;

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

void refreshPCBVersionFlags() {
  String pcbNorm = CFG_PCB_VERSION;
  pcbNorm.toUpperCase();
  pcbNorm.replace(" ", "");
  CFG_PCB_IS_12 = (pcbNorm == "PCB1.2");
}

// Perform a fast software reset using the watchdog
void triggerSoftwareReset(bool showMessage = true) {
  if (showMessage) {
    displayText("rESEt", "triGGt");
    delay(1000);
  }
  Serial.flush();
  delay(50);
  wdt_enable(WDTO_15MS);
  while (true) {
    // Wait for watchdog to trigger
  }
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
// 7-segment layout:
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
  if (c == '$') return 0b00000100;  // Custom "m": segments c,e (right and bottom-left vertical)
  
  // Check for lowercase characters before converting to uppercase
  if (c == 'o') return 0b00011101;  // Lowercase o (bottom segments only)
  if (c == 'i') return 0b00010000;  // Lowercase i (just vertical segments)
  if (c == 'm') return 0b00010101;  // Lowercase m: segments c, e (looks like "ni" when paired)
  if (c == 'y') return 0b00111011;  // Lowercase y: like "4" with bottom segment (b,c,d,f,g)
  
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
    case 'M': return 0b00010101;  // Same as lowercase m
    case 'N': return 0b00010101;
    case 'O': return 0b01111110;
    case 'P': return 0b01100111;
    case 'R': return 0b00000101;
    case 'S': return 0b01011011;
    case 'T': return 0b00001111;
    case 'U': return 0b00111110;
    case 'X': return 0b00110111;  // H with middle segment (looks like +/×)
    case 'Y': return 0b00111011;  // Same as lowercase y
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
  // Set full strings and prepare scroll state
  currentLeftFull = left;
  currentRightFull = right;
  // Determine if scrolling is needed (count effective characters excluding '.' which are DP markers)
  auto effectiveLength = [](const String &s) {
    int cnt = 0;
    for (int i = 0; i < s.length(); i++) if (s.charAt(i) != '.') cnt++;
    return cnt;
  };
  scrollLeftEnabled = (effectiveLength(currentLeftFull) > 6);
  scrollRightEnabled = (effectiveLength(currentRightFull) > 6);
  scrollOffsetLeft = 0;
  scrollOffsetRight = 0;
  lastScrollTs = millis();

  // Windowing that preserves '.' so DP can be shown. We place up to 6 digits/letters,
  // but we keep any '.' that belong to those digits in the output string.
  auto buildWindow = [](const String &full) {
    String out = "";
    int placed = 0;
    for (int i = 0; i < full.length() && placed < 6; i++) {
      char c = full.charAt(i);
      out += c;              // keep char (including '.')
      if (c != '.') placed++; // count only real digits/letters
    }
    // Pad with spaces if fewer than 6 chars were placed
    while (placed < 6) { out += ' '; placed++; }
    return out;
  };
  String leftWindow = buildWindow(currentLeftFull);
  String rightWindow = buildWindow(currentRightFull);
  updateDisplay(DISP_LEFT, leftWindow);
  updateDisplay(DISP_RIGHT, rightWindow);
  displayLeft = leftWindow;
  displayRight = rightWindow;
}

// Advance scroll offsets if needed and refresh displays
void scrollTick(unsigned long now) {
  if ((now - lastScrollTs) < SCROLL_INTERVAL_MS) return;
  lastScrollTs = now;
  auto effectiveLength = [](const String &s) {
    int cnt = 0;
    for (int i = 0; i < s.length(); i++) if (s.charAt(i) != '.') cnt++;
    return cnt;
  };
  // Windowing for scrolling that preserves '.' so DP renders
  auto buildWindow = [](const String &full, int offsetChars) {
    String out = "";
    int placed = 0;
    int skipped = 0;
    for (int i = 0; i < full.length() && placed < 6; i++) {
      char c = full.charAt(i);
      if (c != '.') {
        if (skipped < offsetChars) { skipped++; continue; }
        placed++;
      }
      out += c; // keep character, including '.'
    }
    while (placed < 6) { out += ' '; placed++; }
    return out;
  };

  if (scrollLeftEnabled) {
    int elen = effectiveLength(currentLeftFull);
    int maxOffset = max(0, elen - 6);
    scrollOffsetLeft++;
    if (scrollOffsetLeft > maxOffset) scrollOffsetLeft = 0;
    String leftWindow = buildWindow(currentLeftFull, scrollOffsetLeft);
    updateDisplay(DISP_LEFT, leftWindow);
    displayLeft = leftWindow;
  }
  if (scrollRightEnabled) {
    int elen = effectiveLength(currentRightFull);
    int maxOffset = max(0, elen - 6);
    scrollOffsetRight++;
    if (scrollOffsetRight > maxOffset) scrollOffsetRight = 0;
    String rightWindow = buildWindow(currentRightFull, scrollOffsetRight);
    updateDisplay(DISP_RIGHT, rightWindow);
    displayRight = rightWindow;
  }
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
  bool ilsOut = desiredIlsLed;
  bool mlsOut = desiredMlsLed;
  if (CFG_PCB_IS_12) {
    // PCB 1.2 uses inverted logic on LED3 lines
    ilsOut = !ilsOut;
    mlsOut = !mlsOut;
  }
  digitalWrite(ledIlsSel, ilsOut ? HIGH : LOW);
  digitalWrite(ledMlsSel, mlsOut ? HIGH : LOW);
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
  int step = 1;  // Always use step = 1 for Fenix compatibility
  
  bool validTransition = false;
  int direction = 0;
  
  // Apply sensitivity pattern based on RT1_SENSITIVITY
  switch (RT1_SENSITIVITY) {
    case 0b00:  // No sensitivity - all transitions
      if (sum == 0b0001 || sum == 0b0111 || sum == 0b1000 || sum == 0b1110) {
        direction = -1;
        validTransition = true;
      }
      else if (sum == 0b0010 || sum == 0b0100 || sum == 0b1011 || sum == 0b1101) {
        direction = 1;
        validTransition = true;
      }
      break;
      
    case 0b01:  // Low sensitivity - half-step
      if (sum == 0b0001 || sum == 0b1000) {
        direction = -1;
        validTransition = true;
      }
      else if (sum == 0b0010 || sum == 0b0100) {
        direction = 1;
        validTransition = true;
      }
      break;
      
    case 0b10:  // Medium sensitivity - detent (MobiFlight standard)
      if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        direction = 1;
        validTransition = true;
      }
      else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        direction = -1;
        validTransition = true;
      }
      break;
      
    case 0b11:  // High sensitivity - strict detent only
      if (sum == 0b1101 || sum == 0b1011) {
        direction = 1;
        validTransition = true;
      }
      else if (sum == 0b1110 || sum == 0b1000) {
        direction = -1;
        validTransition = true;
      }
      break;
  }
  
  if (validTransition) {
    // Debounce: only count if enough time passed since last change (2ms minimum)
    if ((now - rotary1LastChangeTime) >= 2 || rotary1LastChangeTime == 0) {
      // If we're in DIAG menu, enforce a slightly longer minimum interval to avoid double-counts
      if (bootState == BOOT_DIAG_MENU) {
        if ((now - rotary1LastMenuEventTs) >= ROTARY_MENU_MIN_MS) {
          rotary1Counter += (direction * step);
          rotary1LastChangeTime = now;
          rotary1LastMenuEventTs = now;
        }
      } else {
        rotary1Counter += (direction * step);
        rotary1LastChangeTime = now;
      }
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
  int step = 1;  // Always use step = 1 for Fenix compatibility
  
  bool validTransition = false;
  int direction = 0;
  
  // Apply sensitivity pattern based on RT2_SENSITIVITY
  switch (RT2_SENSITIVITY) {
    case 0b00:  // No sensitivity - all transitions
      if (sum == 0b0001 || sum == 0b0111 || sum == 0b1000 || sum == 0b1110) {
        direction = -1;
        validTransition = true;
      }
      else if (sum == 0b0010 || sum == 0b0100 || sum == 0b1011 || sum == 0b1101) {
        direction = 1;
        validTransition = true;
      }
      break;
      
    case 0b01:  // Low sensitivity - half-step
      if (sum == 0b0001 || sum == 0b1000) {
        direction = -1;
        validTransition = true;
      }
      else if (sum == 0b0010 || sum == 0b0100) {
        direction = 1;
        validTransition = true;
      }
      break;
      
    case 0b10:  // Medium sensitivity - detent (MobiFlight standard)
      if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        direction = 1;
        validTransition = true;
      }
      else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        direction = -1;
        validTransition = true;
      }
      break;
      
    case 0b11:  // High sensitivity - strict detent only
      if (sum == 0b1101 || sum == 0b1011) {
        direction = 1;
        validTransition = true;
      }
      else if (sum == 0b1110 || sum == 0b1000) {
        direction = -1;
        validTransition = true;
      }
      break;
  }
  
  if (validTransition) {
    // Debounce: only count if enough time passed since last change (2ms minimum)
    if ((now - rotary2LastChangeTime) >= 2 || rotary2LastChangeTime == 0) {
      rotary2Counter += (direction * step);
      rotary2LastChangeTime = now;
    }
  }
  
  rotary2LastEncoded = encoded;
}

// ============================================================================
// SERIAL IDENT / STATE
// ============================================================================
void sendIdentAndState() {
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
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
          bootState = BOOT_DIAG_MENU;
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
    else if (key.equalsIgnoreCase("CFG")) {
      // Parse configuration: CFG:ROT1010;DEB15;LED1000;DSP800;REG:D-AIDA
      String cfgStr = val;
      int pos = 0;
      while (pos < cfgStr.length()) {
        int nextSep = cfgStr.indexOf(';', pos);
        if (nextSep < 0) nextSep = cfgStr.length();
        String param = cfgStr.substring(pos, nextSep);
        param.trim();
        
        if (param.startsWith("ROT")) {
          // ROT followed by 4 binary digits: ROT1010 = 0b1010
          String rotVal = param.substring(3);
          if (rotVal.length() >= 4) {
            uint8_t rt1 = ((rotVal.charAt(2) == '1') ? 2 : 0) | ((rotVal.charAt(3) == '1') ? 1 : 0);
            uint8_t rt2 = ((rotVal.charAt(0) == '1') ? 2 : 0) | ((rotVal.charAt(1) == '1') ? 1 : 0);
            ROTARY_CONFIG = (rt2 << 2) | rt1;
            RT1_SENSITIVITY = rt1;
            RT2_SENSITIVITY = rt2;
          }
        }
        else if (param.startsWith("DEB")) {
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
        
        pos = nextSep + 1;
      }
    }
    else if (token.startsWith("SET ")) {
      // Serial-based settings: SET ENA, SET FW, SET ACID, WRITE
      String setCmd = token.substring(4);  // Remove "SET "
      setCmd.trim();
      
      if (setCmd.startsWith("ENA:")) {
        // PIN authentication: SET ENA:0815
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
        // Firmware version: SET FW:1.5
        String version = setCmd.substring(3);
        version.trim();
        
        int dotPos = version.indexOf('.');
        if (dotPos > 0 && dotPos < version.length() - 1) {
          String majorStr = version.substring(0, dotPos);
          String minorStr = version.substring(dotPos + 1);
          int major = majorStr.toInt();
          int minor = minorStr.toInt();
          
          if (major >= 1 && major <= 9 && minor >= 0 && minor <= 9) {
            CFG_PCB_VERSION = String("PCb ") + major + "." + minor;
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
        // Aircraft ident: SET ACID:D-AIDA (format: letter-letters, max 8 chars total)
        String ident = setCmd.substring(5);
        ident.trim();
        
        if (ident.length() >= 3 && ident.length() <= 8) {
          // Find dash position
          int dashPos = ident.indexOf('-');
          if (dashPos > 0 && dashPos < ident.length() - 1) {
            // Validate: before dash must be 1 letter, after dash all letters
            bool valid = true;
            if (dashPos != 1) {
              valid = false;  // Dash must be at position 1 (after first letter)
            } else {
              // Check first char is alpha
              if (!isAlpha(ident.charAt(0))) valid = false;
              // Check rest after dash are alpha
              for (int i = dashPos + 1; i < ident.length(); i++) {
                if (!isAlpha(ident.charAt(i))) {
                  valid = false;
                  break;
                }
              }
            }
            if (valid) {
              CFG_AIRCRAFT_REG = ident;
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
      else if (setCmd.startsWith("EXIT")) {
        settingsEnabled = false;
        Serial.println("SET EXIT:OK ;");
      }
      else if (setCmd.startsWith("WRI:") && settingsEnabled) {
        // Write to EEPROM: SET WRI:YES
        String cmd = setCmd.substring(4);
        cmd.trim();
        
        if (cmd.equalsIgnoreCase("YES")) {
          // Use the standard saveHWInfo function
          saveHWInfo();
          settingsEnabled = false;
          Serial.println("SET WRI:OK ;");
          triggerSoftwareReset();
        } else {
          Serial.println("SET WRI:FORMAT ;");
        }
      }
      else if (setCmd.startsWith("WRITE")) {
        // Handle both "SET WRITE" and "SET WRITE:"
        if (settingsEnabled) {
          uint8_t checksum = calcCfgChecksum(EEPROM_FORMAT_VERSION, 
                                             CFG_AIRCRAFT_REG.c_str(), 
                                             CFG_PCB_VERSION.c_str());
          
          for (int i = 0; i < 8; i++) {
            char c = (i < CFG_AIRCRAFT_REG.length()) ? CFG_AIRCRAFT_REG.charAt(i) : 0;
            EEPROM.write(EEPROM_BASE_ADDR + 3 + i, c);
          }
          
          for (int i = 0; i < 8; i++) {
            char c = (i < CFG_PCB_VERSION.length()) ? CFG_PCB_VERSION.charAt(i) : 0;
            EEPROM.write(EEPROM_BASE_ADDR + 11 + i, c);
          }
          
          EEPROM.write(EEPROM_BASE_ADDR + 19, checksum);
          
          settingsEnabled = false;
          Serial.println("WRITE:OK ;");
        } else {
          Serial.println("WRITE:LOCKED ;");
        }
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
      if (tokenUp == "EXIT") {
        settingsEnabled = false;
        Serial.println("EXIT:OK ;");
        continue;
      }
      if (tokenUp == "PCB") {
        // Query PCB version from EEPROM
        Serial.print("PCB:");
        // Extract major.minor from CFG_PCB_VERSION (e.g., "PCB 1.2" -> "v1.2")
        String pcbVer = CFG_PCB_VERSION;
        pcbVer.trim();
        // Find the space and take everything after it
        int spacePos = pcbVer.indexOf(' ');
        if (spacePos >= 0 && spacePos < pcbVer.length() - 1) {
          String digits = pcbVer.substring(spacePos + 1);
          Serial.print("v");
          Serial.print(digits);
        } else {
          Serial.print(pcbVer);
        }
        Serial.println(" ;");
        continue;
      }
      if (tokenUp == "RESET") {
        Serial.println("RESET:OK ;");
        triggerSoftwareReset();
        continue;
      }
      if (tokenUp == "REQ") {
        sendStatusImmediate();
        continue;
      }
      if (tokenUp == "DIAG") {
          if (bootState == BOOT_RUNNING) {
            bootState = BOOT_DIAG_MENU;
            diagStartTime = millis();
            diagStage = 0;
            menuInitialized = false;
            Serial.println("DIAG:MENU_START;");
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
  // Stage 3: Show PCB version (left) and A/C ident (right) (1950-2450ms)
  else if (elapsed < 2450) {
    // Left: full PCB version (trim/pad to 6 chars)
    String leftDisplay = CFG_PCB_VERSION;
    // If too long, try to remove a space to keep the numeric part
    if (leftDisplay.length() > 6) {
      int sp = leftDisplay.indexOf(' ');
      if (sp >= 0) leftDisplay.remove(sp, 1);
    }
    if (leftDisplay.length() > 6) leftDisplay = leftDisplay.substring(0, 6);
    while (leftDisplay.length() < 6) leftDisplay += " ";

    // Right: aircraft ident (trim/pad to 6 chars)
    String rightDisplay = CFG_AIRCRAFT_REG;
    if (rightDisplay.length() > 6) rightDisplay = rightDisplay.substring(0, 6);
    while (rightDisplay.length() < 6) rightDisplay += " ";

    displayText(leftDisplay, rightDisplay);
  }
  // Stage 4: All off, sequence complete (2450ms+)
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
        // NOTE: offlineMessageShown stays TRUE to prevent automatic re-show
      }
    }
  } else {
    // Online - reset flags
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
// BUTTON TEST - Tight loop for instant response
// ============================================================================
void runButtonTest() {
  // Display "PRESS BUTTON" message initially
  static bool headerShown = false;
  static unsigned long testStartTime = 0;
  
  if (!headerShown) {
    displayText("PrESS ", "bUtton");
    testStartTime = millis();
    headerShown = true;
    // Initialize input state baseline
    readShiftRegisters(lastInputState1, lastInputState2);
  }
  
  unsigned long now = millis();
  unsigned long lastButtonActivityTs = now;  // Track last button activity for timeout
  
  // Tight loop - NO throttle, runs as fast as possible
  while (true) {
    now = millis();
    
    // Read inputs directly - NO debounce
    uint8_t currentIn1, currentIn2;
    readShiftRegisters(currentIn1, currentIn2);
    
    // Check for button state changes in IN1
    if (currentIn1 != lastInputState1) {
      uint8_t changed = currentIn1 ^ lastInputState1;
      
      // Find and display the changed button
      for (int i = 0; i < 8; i++) {
        if (changed & (1 << i)) {
          bool isPressed = (currentIn1 >> i) & 1;
          
          if (isPressed) {
            displayButtonPressed(i, 1);
            controlButtonLED(i, 1, true);
          } else {
            displayButtonReleased(i, 1);
            controlButtonLED(i, 1, false);
          }
          lastButtonActivityTs = now;  // Update activity timestamp
        }
      }
      
      lastInputState1 = currentIn1;
    }
    
    // Check for button state changes in IN2
    if (currentIn2 != lastInputState2) {
      uint8_t changed = currentIn2 ^ lastInputState2;
      
      // Find and display the changed button
      for (int i = 0; i < 8; i++) {
        if (changed & (1 << i)) {
          bool isPressed = (currentIn2 >> i) & 1;
          
          if (isPressed) {
            displayButtonPressed(i, 2);
            controlButtonLED(i, 2, true);
          } else {
            displayButtonReleased(i, 2);
            controlButtonLED(i, 2, false);
          }
          lastButtonActivityTs = now;  // Update activity timestamp
        }
      }
      
      lastInputState2 = currentIn2;
    }
    
    // If all buttons released and 1 second passed, show "PRESS BUTTON"
    if ((currentIn1 == 0) && (currentIn2 == 0) && (now - lastButtonActivityTs >= 1000)) {
      displayText("PrESS ", "bUtton");
      lastButtonActivityTs = now + 10000;  // Prevent repeated updates
    }
    
    // Poll rotary encoders
    updateRotary1();
    updateRotary2();
    
    int rt1Delta = rotary1Counter - lastMenuRotary1;
    int rt2Delta = rotary2Counter - lastMenuRotary2;
    
    if (rt1Delta != 0 && (now - rotary1LastMenuEventTs) >= ROTARY_MENU_MIN_MS) {
      String direction = (rt1Delta > 0) ? "INC   " : "dEC   ";
      displayText("rot1  ", direction);
      delay(1000);  // Show for 1 second
      displayText("PrESS ", "bUtton");
      lastMenuRotary1 = rotary1Counter;
      rotary1LastMenuEventTs = now;
      lastButtonActivityTs = now + 10000;  // Prevent button timeout trigger
    }
    
    if (rt2Delta != 0) {
      String direction = (rt2Delta > 0) ? "INC   " : "dEC   ";
      displayText("rot2  ", direction);
      delay(1000);  // Show for 1 second
      displayText("PrESS ", "bUtton");
      lastMenuRotary2 = rotary2Counter;
      lastButtonActivityTs = now + 10000;  // Prevent button timeout trigger
    }
    
    // Check for XFER double-click to exit
    static unsigned long lastXferPressTs = 0;
    static bool xferWasPressed = false;
    bool xferPressed = (currentIn1 & BUTTON_XFER_MASK1);
    
    if (xferPressed && !xferWasPressed) {
      // XFER pressed
      if (now - lastXferPressTs < 500) {
        // Double-click detected (within 500ms)
        delay(200);  // Debounce
        setLEDState(0x0000, false, false, 0);
        displayText("      ", "      ");
        menuMode = 0;
        menuInitialized = false;
        headerShown = false;
        return;
      }
      lastXferPressTs = now;
    }
    xferWasPressed = xferPressed;
    
    // Small delay to prevent Arduino lock-up (1ms = 1000 loops/sec)
    delayMicroseconds(500);
  }
}

// ============================================================================
// SHIFT REGISTER TEST - Live display of IN1/IN2 states
// ============================================================================
void runShiftRegTest() {
  static bool headerShown = false;
  static unsigned long lastToggleTs = 0;
  static bool showIN1 = true;
  
  if (!headerShown) {
    displayText("ShIFt ", "rEG   ");
    delay(1000);
    headerShown = true;
    lastToggleTs = millis();
  }
  
  unsigned long now = millis();
  
  // Tight loop - NO throttle
  while (true) {
    now = millis();
    
    // Read inputs directly
    uint8_t currentIn1, currentIn2;
    readShiftRegisters(currentIn1, currentIn2);
    
    // Toggle between IN1 and IN2 display every 1 second
    if (now - lastToggleTs >= 1000) {
      showIN1 = !showIN1;
      lastToggleTs = now;
    }
    
    // Format display: "IN1:00" "000000" or "IN2:00" "000000"
    String leftDisplay, rightDisplay;
    
    if (showIN1) {
      leftDisplay = "In1 ";
      leftDisplay += ((currentIn1 >> 6) & 1) ? "1" : "0";
      leftDisplay += ((currentIn1 >> 7) & 1) ? "1" : "0";
      
      rightDisplay = "";
      for (int i = 5; i >= 0; i--) {
        rightDisplay += ((currentIn1 >> i) & 1) ? "1" : "0";
      }
    } else {
      leftDisplay = "In2 ";
      leftDisplay += ((currentIn2 >> 6) & 1) ? "1" : "0";
      leftDisplay += ((currentIn2 >> 7) & 1) ? "1" : "0";
      
      rightDisplay = "";
      for (int i = 5; i >= 0; i--) {
        rightDisplay += ((currentIn2 >> i) & 1) ? "1" : "0";
      }
    }
    
    displayText(leftDisplay, rightDisplay);
    
    // Check for XFER double-click to exit
    static unsigned long lastXferPressTs = 0;
    static bool xferWasPressed = false;
    bool xferPressed = (currentIn1 & BUTTON_XFER_MASK1);
    
    if (xferPressed && !xferWasPressed) {
      if (now - lastXferPressTs < 500) {
        // Double-click detected
        delay(200);
        displayText("      ", "      ");
        menuMode = 0;
        menuInitialized = false;
        headerShown = false;
        return;
      }
      lastXferPressTs = now;
    }
    xferWasPressed = xferPressed;
    
    // Small delay
    delayMicroseconds(500);
  }
}

// ============================================================================
// DIAG MODE - SIMPLIFIED MENU
// ============================================================================
void runDiagMenu() {
  unsigned long now = millis();
  
  // Poll inputs & encoders every iteration (NO throttle in button test for instant response)
  readShiftRegisters(inputState1, inputState2);
  updateRotary1();
  updateRotary2();

  // Debounce inputs
  if (inputState1 != lastInputState1 || inputState2 != lastInputState2) {
    if (now - lastDebounceTs >= CFG_BUTTON_DEBOUNCE) {
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      reactivateOfflineMessage();
    }
  }

  // Initialize menu on first entry
  if (!menuInitialized) {
    menuInitialized = true;
    menuIndex = 0;
    menuMode = 0;
    lastMenuRotary1 = rotary1Counter;
    lastXferPressed = false;
    displayText("dIAG  ", "n$Enu ");
    lastMenuActivityTs = now;
    lastMenuNavTs = now;
    return;
  }

  // Keep header visible for 2 seconds
  if ((now - diagStartTime) < 2000) return;

  // Auto-exit after inactivity
  if ((now - lastMenuActivityTs) >= MENU_INACTIVITY_TIMEOUT_MS) {
    // Clear display before exiting
    setLEDState(0x0000, false, false, 0);
    displayText("      ", "      ");
    updateDisplay(DISP_LEFT, "      ");
    updateDisplay(DISP_RIGHT, "      ");
    bootState = BOOT_RUNNING;
    menuInitialized = false;
    return;
  }

  // ===== MAIN MENU (menuMode == 0) =====
  if (menuMode == 0) {
    int delta = rotary1Counter - lastMenuRotary1;
    if (delta != 0 && (now - lastMenuNavTs) >= MENU_NAV_DEBOUNCE_MS) {
      // Skip if we're in ignore window
      if (now < menuNavIgnoreUntil) {
        lastMenuRotary1 = rotary1Counter;
        lastMenuNavTs = now;
        return;
      }
      
      // Navigate: 4 main items now
      if (delta > 0) menuIndex++;
      else menuIndex--;
      if (menuIndex < 0) menuIndex = 0;
      if (menuIndex > 3) menuIndex = 3;
      
      lastMenuRotary1 = rotary1Counter;
      lastMenuNavTs = now;
      menuNavIgnoreUntil = now + ROTARY_MENU_MIN_MS;
      lastMenuActivityTs = now;
    }

    // Display current menu item
    switch (menuIndex) {
      case 0: displayText("1. LED ", "tESt  "); break;
      case 1: displayText("2. butn", "tESt  "); break;
      case 2: displayText("3. Info", "      "); break;
      case 3: displayText("4. Exit", "to RUN "); break;
    }

    // XFER (Enter) handling
    bool xferPressed = ((lastInputState1 & BUTTON_XFER_MASK1) || (lastInputState2 & BUTTON_XFER_MASK2));
    if (xferPressed && !lastXferPressed && (now - lastXferHandledTs) >= XFER_IGNORE_MS) {
      lastXferPressed = true;
      lastXferPressTime = now;
      lastXferHandledTs = now;
      lastMenuActivityTs = now;

      if (menuIndex == 0) {
        // Enter LED Test submenu
        menuMode = 1;
        menuSubIndex = 0;
        menuInitialized = true;
        lastMenuRotary1 = rotary1Counter;
        menuNavIgnoreUntil = now + 250;
      } else if (menuIndex == 1) {
        // Enter Button Test submenu (2 options)
        menuMode = 4;  // Button submenu
        menuSubIndex = 0;
        menuInitialized = true;
        lastMenuRotary1 = rotary1Counter;
        menuNavIgnoreUntil = now + 250;
      } else if (menuIndex == 2) {
        // Show Info (HW ID)
        menuMode = 5;  // Info display
        diagStartTime = now;
      } else if (menuIndex == 3) {
        // Exit
        setLEDState(0x0000, false, false, 0);
        displayText("      ", "      ");
        updateDisplay(DISP_LEFT, "      ");
        updateDisplay(DISP_RIGHT, "      ");
        bootState = BOOT_RUNNING;
        menuInitialized = false;
        return;
      }
      return;
    }
    if (!xferPressed) lastXferPressed = false;
    return;
  }

  // ===== BUTTON TEST (menuMode == 2) =====
  if (menuMode == 2) {
    // Run button test in tight loop for instant response (bypasses main loop throttle)
    runButtonTest();
    return;
  }

  // ===== SHIFT REG TEST (menuMode == 3) =====
  if (menuMode == 3) {
    // Run shift register test in tight loop
    runShiftRegTest();
    return;
  }

  // ===== BUTTON SUBMENU (menuMode == 4) =====
  if (menuMode == 4) {
    int delta = rotary1Counter - lastMenuRotary1;
    if (delta != 0 && (now - lastMenuNavTs) >= MENU_NAV_DEBOUNCE_MS) {
      if (now < menuNavIgnoreUntil) {
        lastMenuRotary1 = rotary1Counter;
        lastMenuNavTs = now;
        return;
      }

      if (delta > 0) menuSubIndex++;
      else menuSubIndex--;
      if (menuSubIndex < 0) menuSubIndex = 0;
      if (menuSubIndex > 2) menuSubIndex = 2;  // 3 options: Button Test, Shift Reg, Back

      lastMenuRotary1 = rotary1Counter;
      lastMenuNavTs = now;
      menuNavIgnoreUntil = now + ROTARY_MENU_MIN_MS;
      lastMenuActivityTs = now;
    }

    // Display submenu items
    switch (menuSubIndex) {
      case 0: displayText("bUtton ", "tESt  "); break;
      case 1: displayText("ShIFt  ", "rEG   "); break;
      case 2: displayText("bACK  ", "      "); break;
    }

    // XFER to execute or go back
    bool xferPressed = ((lastInputState1 & BUTTON_XFER_MASK1) || (lastInputState2 & BUTTON_XFER_MASK2));
    if (xferPressed && !lastXferPressed && (now - lastXferHandledTs) >= XFER_IGNORE_MS) {
      lastXferPressed = true;
      lastXferHandledTs = now;
      lastMenuActivityTs = now;

      if (menuSubIndex == 0) {
        // Button Test
        menuMode = 2;
        readShiftRegisters(lastInputState1, lastInputState2);
        diagStartTime = now;
      } else if (menuSubIndex == 1) {
        // Shift Register Test
        menuMode = 3;
      } else if (menuSubIndex == 2) {
        // BACK to main menu
        menuMode = 0;
        menuInitialized = false;
      }
      return;
    }
    if (!xferPressed) lastXferPressed = false;
    return;
  }

  // ===== INFO DISPLAY (menuMode == 5) =====
  if (menuMode == 5) {
    // Left display: PCB Version (e.g., "PCb 1.5")
    // Right display: Aircraft Ident (e.g., "D-AIDA")
    
    String leftDisplay = CFG_PCB_VERSION;
    // If too long, try to remove a single space to keep version digits
    if (leftDisplay.length() > 6) {
      int sp = leftDisplay.indexOf(' ');
      if (sp >= 0) leftDisplay.remove(sp, 1);
    }
    while (leftDisplay.length() < 6) leftDisplay += " ";
    if (leftDisplay.length() > 6) leftDisplay = leftDisplay.substring(0, 6);
    
    // Build right display: Aircraft Ident
    String rightDisplay = CFG_AIRCRAFT_REG;
    while (rightDisplay.length() < 6) rightDisplay += " ";
    if (rightDisplay.length() > 6) rightDisplay = rightDisplay.substring(0, 6);
    
    // Display both
    displayText(leftDisplay, rightDisplay);
    
    // Exit on XFER
    bool xferPressed = ((lastInputState1 & BUTTON_XFER_MASK1) || (lastInputState2 & BUTTON_XFER_MASK2));
    if (xferPressed && !lastXferPressed && (now - lastXferHandledTs) >= XFER_IGNORE_MS) {
      lastXferPressed = true;
      lastXferHandledTs = now;
      menuMode = 0;
      menuInitialized = false;
      return;
    }
    if (!xferPressed) lastXferPressed = false;
    return;
  }

  if (menuMode == 1) {
    int delta = rotary1Counter - lastMenuRotary1;
    if (delta != 0 && (now - lastMenuNavTs) >= MENU_NAV_DEBOUNCE_MS) {
      if (now < menuNavIgnoreUntil) {
        lastMenuRotary1 = rotary1Counter;
        lastMenuNavTs = now;
        return;
      }

      if (delta > 0) menuSubIndex++;
      else menuSubIndex--;
      if (menuSubIndex < 0) menuSubIndex = 0;
      if (menuSubIndex > 5) menuSubIndex = 5;  // 6 options: ALL, DISP, LED, SEG, BRIGHT, BACK

      lastMenuRotary1 = rotary1Counter;
      lastMenuNavTs = now;
      menuNavIgnoreUntil = now + ROTARY_MENU_MIN_MS;
      lastMenuActivityTs = now;
    }

    // Display submenu items
    switch (menuSubIndex) {
      case 0: displayText("RUN   ", "ALL   "); break;
      case 1: displayText("dISPly", "tEst  "); break;
      case 2: displayText("LED   ", "wALK  "); break;
      case 3: displayText("SEG   ", "tESt  "); break;
      case 4: displayText("bright", "nESS  "); break;
      case 5: displayText("bACK  ", "      "); break;
    }

    // XFER to execute or go back
    bool xferPressed = ((lastInputState1 & BUTTON_XFER_MASK1) || (lastInputState2 & BUTTON_XFER_MASK2));
    if (xferPressed && !lastXferPressed && (now - lastXferHandledTs) >= XFER_IGNORE_MS) {
      lastXferPressed = true;
      lastXferHandledTs = now;
      lastMenuActivityTs = now;

      if (menuSubIndex == 0) {
        // RUN ALL - the full diag sequence
        diagReturnToMenu = true;
        bootState = BOOT_DIAG_TEST;
        diagStartTime = millis();
        Serial.println("DIAG:FULL_START;");
        return;
      } else if (menuSubIndex == 1) {
        // DISPLAY test - reuse the display count stage from runFullDiagSequence (~5 sec)
        unsigned long t0 = millis();
        setLEDState(0x0000, false, false, 0);
        while (millis() - t0 < 5000) {
          do_display_count_step(millis() - t0);
          delay(80);
        }
        displayText("      ", "      ");
        menuMode = 0;
        menuInitialized = false;
      } else if (menuSubIndex == 2) {
        // LED WALK test - reuse the LED walking pattern stage from runFullDiagSequence (~5 sec)
        unsigned long t0 = millis();
        displayText("LEd   ", "wALK  ");
        delay(1000);
        while (millis() - t0 < 5000) {
          do_led_walk_step(millis() - t0 - 1000);
          delay(100);
        }
        setLEDState(0x0000, false, false, 0);
        menuMode = 0;
        menuInitialized = false;
      } else if (menuSubIndex == 3) {
        // SEGMENT test - reuse segment sweep from runFullDiagSequence (~5 sec)
        unsigned long t0 = millis();
        displayText("SEG   ", "tESt  ");
        delay(1000);
        setLEDState(0x0000, false, false, 128);
        while (millis() - t0 < 5000) {
          do_segment_sweep_step(millis() - t0);
          delay(50);
        }
        displayText("      ", "      ");
        menuMode = 0;
        menuInitialized = false;
      } else if (menuSubIndex == 4) {
        // BRIGHTNESS test - reuse the EXACT brightness fade stage from runFullDiagSequence (~5 sec)
        unsigned long t0 = millis();
        displayText("brIGht", "tESt  ");
        setLEDState(0xFFFF, true, true, 255);
        
        while (millis() - t0 < 5000) {
          do_brightness_fade_step(millis() - t0);
          delay(40);
        }
        
        // Blank all LEDs explicitly
        setLEDState(0x0000, false, false, 0);
        Serial.println("LED1:00000000;LED2:00000000;LED3:00;");
        
        setDisplayBrightness(displayBrightness);
        menuMode = 0;
        menuInitialized = false;
      } else if (menuSubIndex == 5) {
        // BACK to main menu
        menuMode = 0;
        menuInitialized = false;
      }
      return;
    }
    if (!xferPressed) lastXferPressed = false;
    return;
  }

  scrollTick(now);
}

void runFullDiagSequence() {
  unsigned long elapsed = millis() - diagStartTime;
  
  // Stage 0: Show "DIAG START" message (0-2000ms)
  if (elapsed < 2000) {
    setLEDState(0x0000, false, false, 0);
    displayText("dIAG  ", "StArt ");
  }
  // Stage 1: Display count test - All LEDs OFF, digits count 0-9 with DP sweep (2000-7000ms)
  else if (elapsed < 7000) {
    // Keep ALL LEDs off during counting sequence
    setLEDState(0x0000, false, false, 0);
    do_display_count_step(elapsed - 2000);
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
    do_led_walk_step(elapsed - 15000);
  }
  // Stage 5: MAX7219 segment test - light up each segment individually (20000-25000ms)
  else if (elapsed < 25000) {
    setLEDState(0x0000, false, false, 128);
    
    // Show message for first 2 seconds, then run actual test
    if (elapsed < 22000) {
      displayText("SEG   ", "tESt  ");
    } else {
      do_segment_sweep_step(elapsed - 22000 + 1000);  // Offset so segment test starts after initial delay
    }
  }
  // Stage 6: Brightness test (25000-30000ms)
  else if (elapsed < 30000) {
    displayText("brIGht", "tESt  ");
    setLEDState(0xFFFF, true, true, 255);
    
    // Fade brightness up and down - reuse shared function
    do_brightness_fade_step(elapsed - 25000);
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
    if (diagReturnToMenu) {
      // return into DIAG menu instead of leaving diagnostics
      // Blank LEDs and displays before returning to menu
      setLEDState(0x0000, false, false, 0);
      displayText("      ", "      ");
      updateDisplay(DISP_LEFT, "      ");
      updateDisplay(DISP_RIGHT, "      ");
      bootState = BOOT_DIAG_MENU;
      menuInitialized = false;
      diagReturnToMenu = false;
    } else {
      bootState = BOOT_RUNNING;
    }
    diagComboWasActive = false;
    // Reset offline message state so it shows fresh after DIAG
    offlineMessageShown = false;
    Serial.println("DIAG:COMPLETE;");
  }
}

// Blocking helpers reused by menu submenu tests (short, user-invoked)
void do_display_count_blocking(unsigned long dur) {
  unsigned long t0 = millis();
  while (millis() - t0 < dur) {
    unsigned long rel = millis() - t0;
    int digit = (rel / 500) % 10;
    int dpPos = (rel / 500) % 6;
    String leftDisplay = "";
    String rightDisplay = "";
    for (int i = 0; i < 6; i++) {
      leftDisplay += String(digit);
      rightDisplay += String(digit);
      if (i == dpPos) { leftDisplay += "."; rightDisplay += "."; }
    }
    displayText(leftDisplay, rightDisplay);
    delay(80);
  }
  displayText("      ", "      ");
}

void do_led_walk_blocking(unsigned long dur) {
  unsigned long t0 = millis();
  while (millis() - t0 < dur) {
    for (int i = 0; i < 16 && (millis() - t0 < dur); i++) {
      setLEDState(1 << i, i==0, i==1, 200);
      delay(120);
    }
  }
  setLEDState(0x0000, false, false, 0);
}

void do_segment_sweep_blocking(unsigned long dur) {
  unsigned long t0 = millis();
  while (millis() - t0 < dur) {
    for (int seg = 0; seg < 8 && (millis() - t0 < dur); seg++) {
      byte p = (1 << seg);
      for (int dev = 0; dev < 2; dev++) for (int d = 0; d < 6; d++) lc.setRow(dev, d, p);
      delay(150);
    }
  }
  displayText("      ", "      ");
}

void do_brightness_blocking(unsigned long dur) {
  unsigned long t0 = millis();
  while (millis() - t0 < dur) {
    int pos = (millis() - t0) % 2000;
    int b = (pos < 1000) ? (pos / 67) : (255 - ((pos - 1000) / 67));
    analogWrite(pwmBrightness, b);
    for (int dev = 0; dev < 2; dev++) lc.setIntensity(dev, b/17);
    delay(40);
  }
  setDisplayBrightness(displayBrightness);
}

// Helper functions for test steps - shared between DIAG menu and full sequence
void do_brightness_fade_step(unsigned long elapsed) {
  int fadePos = elapsed % 2000;
  int brightness = (fadePos < 1000) ? (fadePos / 4) : (255 - ((fadePos - 1000) / 4));
  
  analogWrite(pwmBrightness, brightness);
  for (int dev = 0; dev < 2; dev++) {
    lc.setIntensity(dev, brightness / 17);  // 0-15 range
  }
}

void do_display_count_step(unsigned long elapsed) {
  int digit = (elapsed / 500) % 10;
  int dpPos = (elapsed / 500) % 6;
  
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

void do_led_walk_step(unsigned long elapsed) {
  int ledIndex = (elapsed / 200) % 16;
  uint16_t pattern = 1 << ledIndex;
  setLEDState(pattern, ledIndex == 0, ledIndex == 1, 200);
}

void do_segment_sweep_step(unsigned long elapsed) {
  // After 1000ms, start segment sweep
  if (elapsed < 1000) {
    return;
  }
  int segIndex = ((elapsed - 1000) / 300) % 8;
  byte segPattern = 1 << segIndex;
  for (int dev = 0; dev < 2; dev++) {
    for (int digit = 0; digit < 6; digit++) {
      lc.setRow(dev, digit, segPattern);
    }
  }
}

// Check for DIAG combo: IN1:00000010;IN2:00010001
void checkDiagCombo() {
  bool comboActive = (inputState1 == 0b00000010 && inputState2 == 0b00010001);
  
  // Trigger DIAG only on rising edge (combo just pressed, not held)
  if (bootState == BOOT_RUNNING && comboActive && !diagComboWasActive) {
    bootState = BOOT_DIAG_MENU;
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
// ---------------------- EEPROM helpers -------------------------------------
uint8_t calcCfgChecksum(uint8_t version, const char *reg8, const char *pcb8) {
  uint16_t sum = version;
  for (int i = 0; i < 8; i++) sum += (uint8_t)reg8[i];
  for (int i = 0; i < 8; i++) sum += (uint8_t)pcb8[i];
  return (uint8_t)(sum & 0xFF);
}

void loadHWInfo() {
  uint16_t magic = (uint16_t)EEPROM.read(EEPROM_BASE_ADDR) | ((uint16_t)EEPROM.read(EEPROM_BASE_ADDR+1) << 8);
  if (magic != EEPROM_MAGIC) {
    Serial.println("CFG:EEPROM:MAGIC_MISSING;");
    return;
  }
  uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR+2);
  if (ver != EEPROM_FORMAT_VERSION) {
    Serial.print("CFG:EEPROM:VERSION_MISMATCH:"); Serial.println(ver);
    return;
  }
  
  // Read aircraft registration
  char regbuf[9];
  for (int i = 0; i < 8; i++) regbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+3+i);
  regbuf[8] = 0;
  
  // Read PCB version
  char pcbbuf[9];
  for (int i = 0; i < 8; i++) pcbbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+11+i);
  pcbbuf[8] = 0;
  
  uint8_t storedCs = EEPROM.read(EEPROM_BASE_ADDR+19);
  uint8_t cs = calcCfgChecksum(ver, regbuf, pcbbuf);
  if (cs != storedCs) {
    Serial.println("CFG:EEPROM:CRC_FAIL;");
    return;
  }
  
  // Build Strings, trim trailing spaces/zeros
  String regStr = "";
  for (int i = 0; i < 8; i++) if (regbuf[i] != 0 && regbuf[i] != ' ') regStr += regbuf[i];
  if (regStr.length() > 0) CFG_AIRCRAFT_REG = regStr;
  
  String pcbStr = "";
  for (int i = 0; i < 8; i++) {
    if (pcbbuf[i] == 0 || pcbbuf[i] == ' ') {
      // Only trim trailing spaces
      int j = i;
      while (j < 8 && (pcbbuf[j] == 0 || pcbbuf[j] == ' ')) j++;
      if (j >= 8) break;  // Rest is all spaces, stop here
    }
    pcbStr += pcbbuf[i];
  }
  // Trim trailing spaces manually
  while (pcbStr.length() > 0 && pcbStr.charAt(pcbStr.length()-1) == ' ') {
    pcbStr = pcbStr.substring(0, pcbStr.length()-1);
  }
  if (pcbStr.length() > 0) CFG_PCB_VERSION = pcbStr;
  refreshPCBVersionFlags();
  
  Serial.print("CFG:LOAD:REG="); Serial.print(CFG_AIRCRAFT_REG);
  Serial.print(";PCB="); Serial.println(CFG_PCB_VERSION);
}

void saveHWInfo() {
  // Prepare buffers (pad with spaces)
  char regbuf[8];
  for (int i = 0; i < 8; i++) {
    if (i < CFG_AIRCRAFT_REG.length()) regbuf[i] = CFG_AIRCRAFT_REG.charAt(i);
    else regbuf[i] = ' ';
  }
  
  char pcbbuf[8];
  for (int i = 0; i < 8; i++) {
    if (i < CFG_PCB_VERSION.length()) pcbbuf[i] = CFG_PCB_VERSION.charAt(i);
    else pcbbuf[i] = ' ';
  }
  
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

void setup() {
  wdt_disable();
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

  // Initialize button LED states
  for (int i = 0; i < 16; i++) {
    buttonLEDStates[i].currentBrightness = 0;
    buttonLEDStates[i].targetBrightness = 0;
    buttonLEDStates[i].fadeStartTs = 0;
    buttonLEDStates[i].isFading = false;
  }

  // Load persisted configuration from EEPROM (if present)
  loadHWInfo();
  refreshPCBVersionFlags();
  
  // Initialize LEDs off (including direct LEDs)
  setLEDState(0x0000, false, false, 0);
  
  // Use polling for both rotary encoders (more consistent behavior)
  // Hardware interrupts on RT1 can cause excessive triggering with bouncing
  // Polling in main loop at 10ms intervals provides stable, debounced reads
  
  // Print configuration
  Serial.print("SETUP:Rotary polling mode, Config=0b");
  Serial.print(ROTARY_CONFIG, BIN);
  Serial.print(" (RT1=");
  Serial.print(RT1_SENSITIVITY, BIN);
  Serial.print(", RT2=");
  Serial.print(RT2_SENSITIVITY, BIN);
  Serial.println(");");
  Serial.print("SETUP:Debounce="); Serial.print(CFG_BUTTON_DEBOUNCE);
  Serial.print("ms, LED="); Serial.print(CFG_LED_REFRESH);
  Serial.print("ms, DSP="); Serial.print(CFG_DISPLAY_REFRESH);
  Serial.print("ms, REG="); Serial.println(CFG_AIRCRAFT_REG);
  
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
  if (bootState == BOOT_DIAG_MENU) {
    runDiagMenu();
    return;
  }

  if (bootState == BOOT_DIAG_TEST) {
    runFullDiagSequence();
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
    if (now - lastDebounceTs >= CFG_BUTTON_DEBOUNCE) {
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
  
  // Poll both rotary encoders (consistent polling instead of interrupts)
  updateRotary1();
  updateRotary2();
  
  // Only send status if there's actual activity (rotary or buttons changed)
  if (rotary1Counter != lastRotary1Counter || rotary2Counter != lastRotary2Counter) {
    sendStatusImmediate();
  }
  // No periodic sending - only send on actual changes
  
  // Update offline display if needed (non-blocking)
  updateOfflineDisplay();
}
