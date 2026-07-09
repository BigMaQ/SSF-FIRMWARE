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
const char* PANEL_IDENT = "RMP, v1.7 MAQ";
const char  FW_VERSION[] = "1.7";
const char  PANEL_SN_PREFIX[] = "RMP-";   // SN-Präfix / SN prefix / คำนำหน้า SN
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// Serial Number – eindeutige Panel-ID / unique panel ID / รหัสแผงเฉพาะ
// Wird beim ersten Start generiert und im EEPROM gespeichert
// Generated on first boot and stored in EEPROM / สร้างเมื่อเริ่มต้นครั้งแรกและเก็บใน EEPROM
char CFG_SERIAL_NUMBER[10] = "";  // 8 hex chars + null (prefix added on output / Präfix bei Ausgabe ergänzt)

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
// Backlight mask: which LED1 (high byte) bits are controlled by BL/BLT
// RMP: all 8 bits of LED1 high byte are backlight controlled
const uint8_t BACKLIGHT_MASK = 0b11111111;  // LED1 high byte → all backlight

uint16_t desiredLedState = 0x0000;  // 16 bits: LED1=high byte (free bits), LED2=low byte
uint16_t hwLedState      = 0x0000;

bool desiredIlsLed = false;
bool desiredMlsLed = false;
bool hwIlsLed = false;
bool hwMlsLed = false;

uint8_t desiredBrightness = 0;
uint8_t hwBrightness = 0;

bool diagActive = false;  // Set true during DIAG to bypass backlight mask

// ============================================================================
// DISPLAY STATE
// ============================================================================
char displayLeft[12] = "      ";  // up to 6 chars + DP markers + null
char displayRight[12] = "      ";
uint8_t displayBrightness = 15;  // MAX7219 brightness (0-15), default full

// Scrolling support for displays: hold full strings and scroll window if >6 chars
char currentLeftFull[21] = "";   // max 20 chars + null
char currentRightFull[21] = "";
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
const unsigned long ROTARY_FAST_THRESHOLD_MS = 50;

// Toggle-based double-step filter: after counting a step, skip the next
// same-direction transition.  This reliably maps 2 quadrature edges/detent
// to 1 counter increment, regardless of timing.
volatile int rotary1LastDir = 0;
volatile bool rotary1Skip = false;
volatile int rotary2LastDir = 0;
volatile bool rotary2Skip = false;
// Menu-specific guard to avoid double-counting when navigating menus
volatile unsigned long rotary1LastMenuEventTs = 0;
const unsigned long ROTARY_MENU_MIN_MS = 180; // minimum ms between accepted rotary increments while in menu (increased to reduce double-count)

// MobiFlight-style rotary encoder configuration
// Format: 0b<RT2_config><RT1_config> where each is 2 bits:
// 00 = No sensitivity (all transitions), 01 = Low (half-step), 10 = Medium (detent), 11 = High (strict)
uint8_t ROTARY_CONFIG = 0b1010;  // Both encoders: Medium sensitivity; PC-side rot_div=2 handles the rest

// Extract individual configs (updated dynamically)
uint8_t RT1_SENSITIVITY = (ROTARY_CONFIG >> 0) & 0b11;  // Bits 0-1
uint8_t RT2_SENSITIVITY = (ROTARY_CONFIG >> 2) & 0b11;  // Bits 2-3

// Configuration parameters (can be changed via CFG: command)
uint8_t CFG_BUTTON_DEBOUNCE = 12;     // Button debounce time in ms (00-99), default 12ms
uint16_t CFG_LED_REFRESH = 1200;      // LED refresh interval in ms, default 1200ms
uint16_t CFG_DISPLAY_REFRESH = 1200;  // Display refresh interval in ms, default 1200ms
char CFG_AIRCRAFT_REG[9] = "D-A320";  // Aircraft registration, default D-A320, max 8 chars + null
char CFG_PCB_VERSION[9] = "PCB 1.0";  // PCB version, default PCB 1.0, max 8 chars + null
bool CFG_PCB_IS_12 = false;            // Helper flag: true if PCB version is 1.2

// EEPROM storage layout for configuration
const int EEPROM_BASE_ADDR = 0; // start address
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 3;  // v3: +serial_number (9 bytes)
// Layout (offsets): 
// 0-1: magic (uint16)
// 2: version (uint8)
// 3-10: aircraft_reg[8]
// 11-18: pcb_version[8]
// 19-27: serial_number[9] (8 hex chars + null) / Seriennummer / หมายเลขซีเรียล
// 28: checksum (uint8)

// Editing buffer for Settings->HW ID
char editReg[9] = {0}; // 8 chars + null
int editPos = 0;

// PIN & Settings state (Serial-based, not menu-based)
bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";

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
enum BootState { BOOT_INIT, BOOT_FADE, BOOT_RUNNING, BOOT_DIAG_MENU, BOOT_DIAG_TEST };
BootState bootState = BOOT_INIT;
unsigned long bootFadeStart = 0;
const unsigned long BOOT_FADE_DURATION_MS = 1500;
unsigned long bootSequenceStart = 0;

// Boot version message (non-blocking, auto-clears after 5s)
bool bootMessageShown = false;
unsigned long bootMessageStart = 0;
const unsigned long BOOT_MESSAGE_DURATION_MS = 5000;

// External state control (from host via STATE:00/01)
bool hostOnline = false;  // false = offline, true = online
const char offlineMessage1[] = "no FS ";
const char offlineMessage2[] = "onLinE";
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

// DATA mode state (VHF3 LED triggers cross-cockpit data transfer display)
bool dataMode = false;       // DATA mode active (VHF3 LED on)
bool dataOnLeft = true;      // which display currently shows "dAtA"
char lastDsp1Val[12] = "";   // cached last DSP1 value for restore after DATA swap
char lastDsp2Val[12] = "";   // cached last DSP2 value

// ============================================================================
// RUNTIME BUFFERS / FLAGS
// ============================================================================
char serialAccum[64] = "";
bool forceSendNext = false;

// ============================================================================
// HELPER FUNCTIONS - STRING FORMATTING
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

void refreshPCBVersionFlags() {
  // Check if PCB version is "PCB1.2" (case-insensitive, ignore spaces)
  char norm[9];
  uint8_t j = 0;
  for (uint8_t i = 0; CFG_PCB_VERSION[i] && j < 8; i++) {
    char c = CFG_PCB_VERSION[i];
    if (c == ' ') continue;
    if (c >= 'a' && c <= 'z') c -= 32;
    norm[j++] = c;
  }
  norm[j] = 0;
  CFG_PCB_IS_12 = (norm[0] == 'P' && norm[1] == 'C' && norm[2] == 'B' && norm[3] == '1' && norm[4] == '.' && norm[5] == '2' && norm[6] == 0);
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

uint16_t parseBin16(const char* s) {
  uint16_t v = 0;
  for (int i = 0; s[i]; i++) {
    if (s[i] == '0' || s[i] == '1') v = (v << 1) | (s[i] == '1' ? 1 : 0);
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

void updateDisplay(int device, const char* text) {
  // text is up to 6 characters; device 0 or 1
  // MAX7219: digit 0 is rightmost, digit 7 is leftmost (we use 0-5 for 6 digits)
  int textLen = strlen(text);
  int textPos = 0;
  for (int digitPos = 0; digitPos < 6; digitPos++) {
    if (textPos >= textLen) {
      lc.setRow(device, 5 - digitPos, 0b00000000);  // Blank remaining digits
      continue;
    }
    
    char c = text[textPos];
    
    // Skip decimal points - they're not characters, they're modifiers
    if (c == '.') {
      textPos++;
      if (textPos >= textLen) {
        lc.setRow(device, 5 - digitPos, 0b00000000);
        continue;
      }
      c = text[textPos];
    }
    
    byte pattern = charTo7Seg(c);
    
    // Check if next char is a decimal point (applies to current digit)
    if (textPos + 1 < textLen && text[textPos + 1] == '.') {
      pattern |= 0b10000000;  // Set DP bit
    }
    
    lc.setRow(device, 5 - digitPos, pattern);
    textPos++;
  }
}

// Helper: count effective chars (excluding '.') in a string
static int effLen(const char* s) {
  int cnt = 0;
  for (int i = 0; s[i]; i++) if (s[i] != '.') cnt++;
  return cnt;
}

// Helper: build a 6-char window from full string, preserving '.' as DP markers
static void buildWin(char* out, const char* full, int offsetChars) {
  int placed = 0;
  int skipped = 0;
  int i = 0;
  for (; full[i] && placed < 6; i++) {
    char c = full[i];
    if (c != '.') {
      if (skipped < offsetChars) { skipped++; continue; }
      placed++;
    }
    *out++ = c;
  }
  while (placed < 6) { *out++ = ' '; placed++; }
  *out = 0;
}

void displayText(const char* left, const char* right) {
  // Copy full strings for scrolling
  strncpy(currentLeftFull, left, 20); currentLeftFull[20] = 0;
  strncpy(currentRightFull, right, 20); currentRightFull[20] = 0;
  
  scrollLeftEnabled = (effLen(currentLeftFull) > 6);
  scrollRightEnabled = (effLen(currentRightFull) > 6);
  scrollOffsetLeft = 0;
  scrollOffsetRight = 0;
  lastScrollTs = millis();

  buildWin(displayLeft, currentLeftFull, 0);
  buildWin(displayRight, currentRightFull, 0);
  updateDisplay(DISP_LEFT, displayLeft);
  updateDisplay(DISP_RIGHT, displayRight);
}

// Advance scroll offsets if needed and refresh displays
void scrollTick(unsigned long now) {
  if ((now - lastScrollTs) < SCROLL_INTERVAL_MS) return;
  lastScrollTs = now;

  if (scrollLeftEnabled) {
    int elen = effLen(currentLeftFull);
    int maxOffset = max(0, elen - 6);
    scrollOffsetLeft++;
    if (scrollOffsetLeft > maxOffset) scrollOffsetLeft = 0;
    buildWin(displayLeft, currentLeftFull, scrollOffsetLeft);
    updateDisplay(DISP_LEFT, displayLeft);
  }
  if (scrollRightEnabled) {
    int elen = effLen(currentRightFull);
    int maxOffset = max(0, elen - 6);
    scrollOffsetRight++;
    if (scrollOffsetRight > maxOffset) scrollOffsetRight = 0;
    buildWin(displayRight, currentRightFull, scrollOffsetRight);
    updateDisplay(DISP_RIGHT, displayRight);
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
  // Combine free bits + backlight bits, inverted PWM
  // In DIAG mode, bypass mask — direct full 16-bit control
  uint8_t backlightBits = (diagActive || desiredBrightness > 0) ? BACKLIGHT_MASK : 0x00;
  uint16_t output;
  if (diagActive) {
    output = desiredLedState;
  } else {
    uint8_t led1Free = (uint8_t)((desiredLedState >> 8) & ~BACKLIGHT_MASK);
    uint8_t led1Out = led1Free | backlightBits;
    output = ((uint16_t)led1Out << 8) | (desiredLedState & 0x00FF);
  }
  shiftOutLEDs(output);
  bool ilsOut = desiredIlsLed;
  bool mlsOut = desiredMlsLed;
  if (CFG_PCB_IS_12) {
    // PCB 1.2 uses inverted logic on LED3 lines
    ilsOut = !ilsOut;
    mlsOut = !mlsOut;
  }
  digitalWrite(ledIlsSel, ilsOut ? HIGH : LOW);
  digitalWrite(ledMlsSel, mlsOut ? HIGH : LOW);
  // Inverted PWM: BL:0 = off, BL:255 = full brightness
  analogWrite(pwmBrightness, 255 - desiredBrightness);
  
  hwLedState = output;
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
  in2 ^= 0b00100000;  // Bit 5 (ON/OFF) inverted due to soldering error
  
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
    // Toggle filter: count on first edge of a pair, skip the second.
    // Direction change resets the skip flag immediately.
    if (direction != rotary1LastDir) {
      rotary1Skip = false;
      rotary1LastDir = direction;
    }
    if (rotary1Skip) {
      rotary1Skip = false;
    } else if ((now - rotary1LastChangeTime) >= 2 || rotary1LastChangeTime == 0) {
      rotary1Skip = true;
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
    // Toggle filter: count on first edge of a pair, skip the second.
    if (direction != rotary2LastDir) {
      rotary2Skip = false;
      rotary2LastDir = direction;
    }
    if (rotary2Skip) {
      rotary2Skip = false;
    } else if ((now - rotary2LastChangeTime) >= 2 || rotary2LastChangeTime == 0) {
      rotary2Skip = true;
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
// SERIAL COMMAND PARSING
// ============================================================================

// Helper: case-insensitive compare of two null-terminated strings
static bool strieq(const char* a, const char* b) {
  while (*a && *b) {
    char ca = (*a >= 'a' && *a <= 'z') ? *a - 32 : *a;
    char cb = (*b >= 'a' && *b <= 'z') ? *b - 32 : *b;
    if (ca != cb) return false;
    a++; b++;
  }
  return *a == *b;
}

// Helper: check if a starts with prefix (case-insensitive)
static bool stripre(const char* a, const char* prefix) {
  while (*prefix) {
    char ca = (*a >= 'a' && *a <= 'z') ? *a - 32 : *a;
    char cb = (*prefix >= 'a' && *prefix <= 'z') ? *prefix - 32 : *prefix;
    if (ca != cb) return false;
    a++; prefix++;
  }
  return true;
}

// Simple atoi for char*
static int atoi_s(const char* s) {
  int v = 0;
  int sign = 1;
  while (*s == ' ') s++;
  if (*s == '-') { sign = -1; s++; }
  while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
  return v * sign;
}

void processIncomingLine(const char* line) {
  char token[32];
  int tlen = 0;
  for (int i = 0; ; i++) {
    char c = line[i];
    if (c == ';' || c == 0) {
      if (tlen == 0) { if (c == 0) break; else continue; }
      token[tlen] = 0;
      tlen = 0;
      
      // Trim trailing spaces
      int end = strlen(token) - 1;
      while (end >= 0 && token[end] == ' ') token[end--] = 0;
      
      // Trim leading spaces
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
            bootState = BOOT_DIAG_MENU;
            diagActive = true;
            diagStartTime = millis();
            diagStage = 0;
            Serial.println("DIAG:START;");
          }
        } else if (stripre(tok, "SET ")) {
          // SET without colon: EXIT, WRITE
          char* sc = tok + 4;
          while (*sc == ' ') sc++;
          if (stripre(sc, "EXIT")) {
            settingsEnabled = false;
            Serial.println("SET EXIT:OK ;");
          } else if (stripre(sc, "WRITE")) {
            if (settingsEnabled) {
              saveHWInfo();
              settingsEnabled = false;
              displayText("SAUE ", "SAUE ");
              delay(1000);
              Serial.println("WRITE:OK ;");
              triggerSoftwareReset(false);
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
        // Only apply free (non-backlight) bits to LED1 high byte — backlight bits come from BL
        desiredLedState = (desiredLedState & 0x00FF) | ((parseBin8(val) & ~BACKLIGHT_MASK) << 8);
        applyLEDOutputs();
      }
      else if (strieq(kp, "LED2")) {
        desiredLedState = (desiredLedState & 0xFF00) | parseBin8(val);
        // Immediate DATA mode update on VHF3 LED change
        bool vhf3Now = (desiredLedState & 0x0004) != 0;
        if (vhf3Now && !dataMode) {
          dataMode = true; dataOnLeft = true;
          strncpy(displayLeft, " dAtA ", 6); displayLeft[6] = 0;
          updateDisplay(DISP_LEFT, displayLeft);
        } else if (!vhf3Now && dataMode) {
          dataMode = false;
        }
        applyLEDOutputs();
      }
      else if (strieq(kp, "LED3")) {
        uint8_t bits = parseBin8(val);
        desiredIlsLed = bits & 0b01;
        desiredMlsLed = bits & 0b10;
        applyLEDOutputs();
      }
      else if (strieq(kp, "BL")) {
        desiredBrightness = constrain(atoi_s(val), 0, 255);
        applyLEDOutputs();
      }
      else if (strieq(kp, "DISP_BL")) {
        setDisplayBrightness(atoi_s(val));
      }
      else if (strieq(kp, "DSP1")) {
        int dspVal = atoi_s(val);
        // Always cache the raw value for DATA mode restore
        strncpy(lastDsp1Val, val, 11); lastDsp1Val[11] = 0;
        // DATA mode: if this display is the "dAtA" side, ignore incoming DSP value
        if (dataMode && dataOnLeft) {
          // dAtA blocks DSP1 – value is cached above for later restore
        } else if (dspVal < 0 || val[0] == '-') {
          // Any negative value → blank display
          for (int d = 0; d < 6; d++) lc.setRow(DISP_LEFT, d, 0);
          displayLeft[0] = 0;
        } else if (desiredLedState & 0x4000) {
          // --- ADF mode (LED1 bit 6) → kHz with one decimal, right-aligned ---
          // > 999 → /100, < 100 → *10, else as-is; always append ".0"
          int adfVal = dspVal;
          if (adfVal > 999) adfVal /= 100;
          else if (adfVal < 100) adfVal *= 10;
          char adfBuf[8];
          snprintf(adfBuf, 8, "%d.0", adfVal);
          int elen = 0; for (const char* p = adfBuf; *p; p++) if (*p != '.') elen++;
          int pad = 6 - elen;
          if (pad > 0) { memset(displayLeft, ' ', pad); strncpy(displayLeft + pad, adfBuf, 12 - pad); }
          else { strncpy(displayLeft, adfBuf, 12); }
          displayLeft[11] = 0;
          updateDisplay(DISP_LEFT, displayLeft);
        } else if (desiredLedState & 0x0040) {
          // --- NAV LED on → course mode active ---
          // With decimal point: "0.000".."0.359" → course (×1000)
          // Without decimal point: 0..359 → course, >=360 → frequency
          bool isCourse = false;
          int courseVal = 0;
          char* dot = strchr(val, '.');
          if (dot) {
            *dot = 0;
            int intPart = atoi_s(val);
            int fracPart = atoi_s(dot + 1);
            *dot = '.';
            if (intPart == 0 && fracPart >= 0 && fracPart <= 359) {
              isCourse = true;
              courseVal = fracPart;
            }
          } else {
            if (dspVal >= 0 && dspVal <= 359) {
              isCourse = true;
              courseVal = dspVal;
            }
          }

          if (isCourse) {
            displayLeft[0] = ' ';
            displayLeft[1] = 'C'; displayLeft[2] = '-';
            snprintf(displayLeft + 3, 4, "%03d", courseVal);
            displayLeft[6] = 0;
            updateDisplay(DISP_LEFT, displayLeft);
          } else {
            // Not a course value → frequency
            int elen = 0; for (const char* p = val; *p; p++) if (*p != '.') elen++;
            int pad = 6 - elen;
            if (pad > 0) { memset(displayLeft, ' ', pad); strncpy(displayLeft + pad, val, 12 - pad); }
            else { strncpy(displayLeft, val, 12); }
            displayLeft[11] = 0;
            updateDisplay(DISP_LEFT, displayLeft);
          }
        } else if (dspVal == 0) {
          // NAV LED off + value 0 → blank
          for (int d = 0; d < 6; d++) lc.setRow(DISP_LEFT, d, 0);
          displayLeft[0] = 0;
        } else {
          // NAV LED off + non-zero → frequency
          int elen = 0; for (const char* p = val; *p; p++) if (*p != '.') elen++;
          int pad = 6 - elen;
          if (pad > 0) { memset(displayLeft, ' ', pad); strncpy(displayLeft + pad, val, 12 - pad); }
          else { strncpy(displayLeft, val, 12); }
          displayLeft[11] = 0;
          updateDisplay(DISP_LEFT, displayLeft);
        }
      }
      else if (strieq(kp, "DSP2")) {
        int dspVal = atoi_s(val);
        // Always cache the raw value for DATA mode restore
        strncpy(lastDsp2Val, val, 11); lastDsp2Val[11] = 0;
        if (dataMode && !dataOnLeft) {
          // dAtA blocks DSP2 – value is cached above for later restore
        } else if (dspVal < 0 || val[0] == '-') {
          for (int d = 0; d < 6; d++) lc.setRow(DISP_RIGHT, d, 0);
          displayRight[0] = 0;
        } else if (desiredLedState & 0x4000) {
          // --- ADF mode (LED1 bit 6) → kHz with one decimal, right-aligned ---
          int adfVal = dspVal;
          if (adfVal > 999) adfVal /= 100;
          else if (adfVal < 100) adfVal *= 10;
          char adfBuf[8];
          snprintf(adfBuf, 8, "%d.0", adfVal);
          int elen = 0; for (const char* p = adfBuf; *p; p++) if (*p != '.') elen++;
          int pad = 6 - elen;
          if (pad > 0) { memset(displayRight, ' ', pad); strncpy(displayRight + pad, adfBuf, 12 - pad); }
          else { strncpy(displayRight, adfBuf, 12); }
          displayRight[11] = 0;
          updateDisplay(DISP_RIGHT, displayRight);
        } else if (desiredLedState & 0x0040) {
          // --- NAV LED on → course mode active ---
          bool isCourse = false;
          int courseVal = 0;
          char* dot = strchr(val, '.');
          if (dot) {
            *dot = 0;
            int intPart = atoi_s(val);
            int fracPart = atoi_s(dot + 1);
            *dot = '.';
            if (intPart == 0 && fracPart >= 0 && fracPart <= 359) {
              isCourse = true;
              courseVal = fracPart;
            }
          } else {
            if (dspVal >= 0 && dspVal <= 359) {
              isCourse = true;
              courseVal = dspVal;
            }
          }

          if (isCourse) {
            displayRight[0] = ' ';
            displayRight[1] = 'C'; displayRight[2] = '-';
            snprintf(displayRight + 3, 4, "%03d", courseVal);
            displayRight[6] = 0;
            updateDisplay(DISP_RIGHT, displayRight);
          } else {
            int elen = 0; for (const char* p = val; *p; p++) if (*p != '.') elen++;
            int pad = 6 - elen;
            if (pad > 0) { memset(displayRight, ' ', pad); strncpy(displayRight + pad, val, 12 - pad); }
            else { strncpy(displayRight, val, 12); }
            displayRight[11] = 0;
            updateDisplay(DISP_RIGHT, displayRight);
          }
        } else if (dspVal == 0) {
          // NAV LED off + value 0 → blank
          for (int d = 0; d < 6; d++) lc.setRow(DISP_RIGHT, d, 0);
          displayRight[0] = 0;
        } else {
          // NAV LED off + non-zero → frequency
          int elen = 0; for (const char* p = val; *p; p++) if (*p != '.') elen++;
          int pad = 6 - elen;
          if (pad > 0) { memset(displayRight, ' ', pad); strncpy(displayRight + pad, val, 12 - pad); }
          else { strncpy(displayRight, val, 12); }
          displayRight[11] = 0;
          updateDisplay(DISP_RIGHT, displayRight);
        }
      }
      else if (strieq(kp, "STATE")) {
        hostOnline = (strcmp(val, "01") == 0 || strcmp(val, "1") == 0);
        if (hostOnline && strcmp(displayLeft, offlineMessage1) == 0 && strcmp(displayRight, offlineMessage2) == 0) {
          displayText("      ", "      ");
        }
      }
      else if (strieq(kp, "REQ")) {
        forceSendNext = true;
      }
      else if (strieq(kp, "VER")) {
        sendIdentAndState();
      }
      else if (strieq(kp, "CFG")) {
        // Parse configuration: CFG:ROT1010;DEB15;LED1000;DSP800;REG:D-AIDA
        char* cfgPtr = val;
        while (*cfgPtr) {
          // Skip leading spaces/semicolons
          while (*cfgPtr == ' ' || *cfgPtr == ';') cfgPtr++;
          if (!*cfgPtr) break;
          
          // Find next semicolon
          char* semi = strchr(cfgPtr, ';');
          if (semi) *semi = 0;
          
          if (stripre(cfgPtr, "ROT") && strlen(cfgPtr) >= 7) {
            // ROT1010
            uint8_t rt1 = ((cfgPtr[5] == '1') ? 2 : 0) | ((cfgPtr[6] == '1') ? 1 : 0);
            uint8_t rt2 = ((cfgPtr[3] == '1') ? 2 : 0) | ((cfgPtr[4] == '1') ? 1 : 0);
            ROTARY_CONFIG = (rt2 << 2) | rt1;
            RT1_SENSITIVITY = rt1;
            RT2_SENSITIVITY = rt2;
          }
          else if (stripre(cfgPtr, "DEB")) {
            CFG_BUTTON_DEBOUNCE = constrain(atoi_s(cfgPtr + 3), 0, 99);
          }
          else if (stripre(cfgPtr, "LED")) {
            CFG_LED_REFRESH = constrain(atoi_s(cfgPtr + 3), 100, 10000);
          }
          else if (stripre(cfgPtr, "DSP")) {
            CFG_DISPLAY_REFRESH = constrain(atoi_s(cfgPtr + 3), 100, 10000);
          }
          else if (stripre(cfgPtr, "REG:")) {
            strncpy(CFG_AIRCRAFT_REG, cfgPtr + 4, 8);
            CFG_AIRCRAFT_REG[8] = 0;
          }
          else if (stripre(cfgPtr, "SN:")) {
            // SN:XXXXXXXX – 8-stelliger Hex-Wert / 8-digit hex value / ค่าเลขฐานสิบหก 8 หลัก
            strncpy(CFG_SERIAL_NUMBER, cfgPtr + 3, 8);
            CFG_SERIAL_NUMBER[8] = 0;
          }
          
          if (semi) *semi = ';';  // Restore
          cfgPtr = semi ? semi + 1 : cfgPtr + strlen(cfgPtr);
        }
      }
      else if (stripre(tok, "SET ")) {
        // SET commands: key is e.g. "SET ENA", val is e.g. "0815" (already split by colon)
        // Sub-commands no longer carry the colon in the key part
        char* setCmd = tok + 4;
        while (*setCmd == ' ') setCmd++;
        
        if (stripre(setCmd, "ENA")) {
          if (strcmp(val, SETTINGS_PIN) == 0) {
            settingsEnabled = true;
            displayText("CFG  ", "ENA  ");
            Serial.println("SET ENA> ;");
          } else {
            displayText("CFG  ", "FAIL ");
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
              refreshPCBVersionFlags();
              char fwDisp[8];
              snprintf(fwDisp, 8, "PCb %s", verStr);
              displayText(fwDisp, " SEt  ");
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
          char* cmd = val;
          while (*cmd == ' ') cmd++;
          if (strieq(cmd, "YES")) {
            saveHWInfo();
            settingsEnabled = false;
            displayText("SAUE ", "SAUE ");
            delay(1000);
            Serial.println("SET WRI:OK ;");
            triggerSoftwareReset(false);
          } else {
            Serial.println("SET WRI:FORMAT ;");
          }
        }
        else if (stripre(setCmd, "WRITE")) {
          if (settingsEnabled) {
            saveHWInfo();
            settingsEnabled = false;
            displayText("SAUE ", "SAUE ");
            delay(1000);
            Serial.println("WRITE:OK ;");
            triggerSoftwareReset(false);
          } else {
            Serial.println("WRITE:LOCKED ;");
          }
        }
        // SET SN:XXXXXXXX – Serial Number setzen / set serial number / ตั้งค่าหมายเลขซีเรียล
        else if (stripre(setCmd, "SN") && settingsEnabled) {
          char* snVal = val;
          while (*snVal == ' ') snVal++;
          int slen = strlen(snVal);
          // Nur 8-stellige Hex-Werte akzeptieren / Only accept 8-digit hex / ยอมรับเฉพาะค่าเลขฐานสิบหก 8 หลัก
          bool valid = (slen == 8);
          for (int i = 0; i < slen && valid; i++) {
            char c = snVal[i];
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) valid = false;
          }
          if (valid) {
            // To uppercase for storage / Großbuchstaben für Speicherung / ตัวพิมพ์ใหญ่สำหรับจัดเก็บ
            for (int i = 0; i < 8; i++) {
              CFG_SERIAL_NUMBER[i] = (snVal[i] >= 'a' && snVal[i] <= 'f') ? snVal[i] - 32 : snVal[i];
            }
            CFG_SERIAL_NUMBER[8] = 0;
            char snDisp[7];
            snprintf(snDisp, 7, "%s", CFG_SERIAL_NUMBER);
            displayText("Sn SEt", snDisp);
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
      // Copy token to local buffer BEFORE memmove shifts serialAccum
      char token[32];
      char* tp = serialAccum;
      while (*tp == ' ') tp++;
      int ti;
      for (ti = 0; tp[ti] && ti < 31; ti++) token[ti] = tp[ti];
      token[ti] = 0;
      // Trim trailing spaces
      int tend = ti - 1;
      while (tend >= 0 && token[tend] == ' ') token[tend--] = 0;
      
      // Shift remaining buffer
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
        // Send ident immediately, then show versions on display (non-blocking)
        sendIdentAndState();
        identSentOnStart = true;
        char fwDisp[8];
        snprintf(fwDisp, 8, "FRM %s", FW_VERSION);
        char* sp = strchr(CFG_PCB_VERSION, ' ');
        char pcbDisp[8];
        snprintf(pcbDisp, 8, "PCb %s", sp ? sp + 1 : CFG_PCB_VERSION);
        displayText(fwDisp, pcbDisp);
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
        // Find space in CFG_PCB_VERSION
        char* sp = strchr(CFG_PCB_VERSION, ' ');
        if (sp && *(sp + 1)) {
          Serial.print("v");
          Serial.print(sp + 1);
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
          bootState = BOOT_DIAG_MENU;
          diagActive = true;
          diagStartTime = millis();
          diagStage = 0;
          menuInitialized = false;
          Serial.println("DIAG:MENU_START;");
        }
        continue;
      }
      if (strchr(token, ':') != NULL) {
        // Re-add semicolon terminator for processIncomingLine
        char lineBuf[64];
        snprintf(lineBuf, sizeof(lineBuf), "%s;", token);
        processIncomingLine(lineBuf);
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
  
  char b8[9];
  bin8(b8, inputState1); Serial.print("IN1:"); Serial.print(b8); Serial.print(";");
  bin8(b8, inputState2); Serial.print("IN2:"); Serial.print(b8); Serial.print(";");
  
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
  char b8[9];
  bin8(b8, inputState1); Serial.print("IN1:"); Serial.print(b8); Serial.print(";");
  bin8(b8, inputState2); Serial.print("IN2:"); Serial.print(b8); Serial.print(";");
  
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
    char leftDisplay[8];
    char rightDisplay[7];
    // Left: full PCB version (trim/pad to 6 display positions, DP counts as extra char)
    int li = 0;
    for (int i = 0; CFG_PCB_VERSION[i] && li < 7; i++) {
      if (CFG_PCB_VERSION[i] == ' ' && (li > 3)) continue; // skip space if already long
      leftDisplay[li++] = CFG_PCB_VERSION[i];
    }
    while (li < 7) leftDisplay[li++] = ' ';
    leftDisplay[7] = 0;
    // Right: aircraft ident (trim/pad to 6 chars)
    int ri = 0;
    for (int i = 0; CFG_AIRCRAFT_REG[i] && ri < 6; i++) rightDisplay[ri++] = CFG_AIRCRAFT_REG[i];
    while (ri < 6) rightDisplay[ri++] = ' ';
    rightDisplay[6] = 0;
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
    if (!offlineMessageShown && strcmp(displayLeft, "      ") == 0 && strcmp(displayRight, "      ") == 0) {
      strncpy(displayLeft, offlineMessage1, 6); displayLeft[6] = 0;
      strncpy(displayRight, offlineMessage2, 6); displayRight[6] = 0;
      updateDisplay(DISP_LEFT, displayLeft);
      updateDisplay(DISP_RIGHT, displayRight);
      offlineDisplayStart = now;
      offlineMessageShown = true;
    }
    // If message is shown and timeout expired, clear it and STAY cleared
    else if (offlineMessageShown && (now - offlineDisplayStart) >= OFFLINE_DISPLAY_DURATION_MS) {
      if (strcmp(displayLeft, offlineMessage1) == 0 && strcmp(displayRight, offlineMessage2) == 0) {
        strncpy(displayLeft, "      ", 6); displayLeft[6] = 0;
        strncpy(displayRight, "      ", 6); displayRight[6] = 0;
        updateDisplay(DISP_LEFT, displayLeft);
        updateDisplay(DISP_RIGHT, displayRight);
      }
    }
  } else {
    offlineMessageShown = false;
  }
}

// Reactivate offline message on button press
void reactivateOfflineMessage() {
  if (!hostOnline && !offlineMessageShown) {
    if (strcmp(displayLeft, "      ") == 0 && strcmp(displayRight, "      ") == 0) {
      strncpy(displayLeft, offlineMessage1, 6); displayLeft[6] = 0;
      strncpy(displayRight, offlineMessage2, 6); displayRight[6] = 0;
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
      const char* direction = (rt1Delta > 0) ? "INC   " : "dEC   ";
      displayText("rot1  ", direction);
      delay(1000);  // Show for 1 second
      displayText("PrESS ", "bUtton");
      lastMenuRotary1 = rotary1Counter;
      rotary1LastMenuEventTs = now;
      lastButtonActivityTs = now + 10000;  // Prevent button timeout trigger
    }
    
    if (rt2Delta != 0) {
      const char* direction = (rt2Delta > 0) ? "INC   " : "dEC   ";
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
    char leftDisplay[7], rightDisplay[7];
    
    if (showIN1) {
      leftDisplay[0] = 'I'; leftDisplay[1] = 'n'; leftDisplay[2] = '1'; leftDisplay[3] = ' ';
      leftDisplay[4] = ((currentIn1 >> 6) & 1) ? '1' : '0';
      leftDisplay[5] = ((currentIn1 >> 7) & 1) ? '1' : '0';
      leftDisplay[6] = 0;
      for (int i = 0; i < 6; i++) {
        rightDisplay[i] = ((currentIn1 >> (5 - i)) & 1) ? '1' : '0';
      }
      rightDisplay[6] = 0;
    } else {
      leftDisplay[0] = 'I'; leftDisplay[1] = 'n'; leftDisplay[2] = '2'; leftDisplay[3] = ' ';
      leftDisplay[4] = ((currentIn2 >> 6) & 1) ? '1' : '0';
      leftDisplay[5] = ((currentIn2 >> 7) & 1) ? '1' : '0';
      leftDisplay[6] = 0;
      for (int i = 0; i < 6; i++) {
        rightDisplay[i] = ((currentIn2 >> (5 - i)) & 1) ? '1' : '0';
      }
      rightDisplay[6] = 0;
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
    diagActive = false;
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
        diagActive = false;
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
    
    char leftDisp[8], rightDisp[7];
    // Left: PCB version, trim/pad to 7 chars (6 display positions + DP)
    int li = 0;
    for (int i = 0; CFG_PCB_VERSION[i] && li < 7; i++) {
      if (CFG_PCB_VERSION[i] == ' ' && (li > 3)) continue;
      leftDisp[li++] = CFG_PCB_VERSION[i];
    }
    while (li < 7) leftDisp[li++] = ' ';
    leftDisp[7] = 0;
    // Right: aircraft ident
    int ri = 0;
    for (int i = 0; CFG_AIRCRAFT_REG[i] && ri < 6; i++) rightDisp[ri++] = CFG_AIRCRAFT_REG[i];
    while (ri < 6) rightDisp[ri++] = ' ';
    rightDisp[6] = 0;
    displayText(leftDisp, rightDisp);
    
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
    char leftBin[7], rightBin[7];
    
    // Show IN1 on left display (6 bits: bits 7-2)
    for (int i = 0; i < 6; i++) {
      leftBin[i] = (in1 & (1 << (7 - i))) ? '1' : '0';
    }
    leftBin[6] = 0;
    
    // Show IN2 on right display (6 bits: bits 7-2)
    for (int i = 0; i < 6; i++) {
      rightBin[i] = (in2 & (1 << (7 - i))) ? '1' : '0';
    }
    rightBin[6] = 0;
    
    displayText(leftBin, rightBin);
    char b8a[9]; bin8(b8a, in1);
    char b8b[9]; bin8(b8b, in2);
    Serial.print("DIAG:IN1="); Serial.print(b8a);
    Serial.print(";IN2="); Serial.println(b8b);
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
      diagActive = false;
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
    char leftDisplay[13] = {0};
    char rightDisplay[13] = {0};
    int lp = 0, rp = 0;
    for (int i = 0; i < 6; i++) {
      leftDisplay[lp++] = '0' + digit;
      rightDisplay[rp++] = '0' + digit;
      if (i == dpPos) { leftDisplay[lp++] = '.'; rightDisplay[rp++] = '.'; }
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
  
  char leftDisplay[13] = {0};
  char rightDisplay[13] = {0};
  int lp = 0, rp = 0;
  for (int i = 0; i < 6; i++) {
    leftDisplay[lp++] = '0' + digit;
    rightDisplay[rp++] = '0' + digit;
    if (i == dpPos) {
      leftDisplay[lp++] = '.';
      rightDisplay[rp++] = '.';
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
// ---------------------- Serial Number helpers ---------------------------------
// Generiert 8 zufällige Hex-Zeichen / Generates 8 random hex chars / สร้างอักขระฐานสิบหกแบบสุ่ม 8 ตัว
void generateSerialNumber(char* out) {
  // Seed from floating analog pin + micros() for better randomness
  // Seed von unbelegtem Analog-Pin + micros() für bessere Zufälligkeit
  // ใช้ขาอนาล็อกที่ไม่ได้ต่อ + micros() เพื่อความสุ่มที่ดีขึ้น
  randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3) + micros());
  const char hexChars[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) {
    out[i] = hexChars[random(0, 16)];
  }
  out[8] = 0;
}

// ---------------------- EEPROM helpers -------------------------------------
uint8_t calcCfgChecksum(uint8_t version, const char *reg8, const char *pcb8, const char *sn9) {
  uint16_t sum = version;
  for (int i = 0; i < 8; i++) sum += (uint8_t)reg8[i];
  for (int i = 0; i < 8; i++) sum += (uint8_t)pcb8[i];
  for (int i = 0; i < 9; i++) sum += (uint8_t)sn9[i];  // SN: 8 chars + null
  return (uint8_t)(sum & 0xFF);
}

void loadHWInfo() {
  // Lese Magic und Version / Read magic and version / อ่าน magic และ version
  uint16_t magic = (uint16_t)EEPROM.read(EEPROM_BASE_ADDR) | ((uint16_t)EEPROM.read(EEPROM_BASE_ADDR+1) << 8);
  
  // --- Fall 1: Kein Magic → EEPROM leer, alles neu anlegen / No magic → empty EEPROM / ไม่มี magic → EEPROM ว่าง ---
  if (magic != EEPROM_MAGIC) {
    Serial.println("CFG:EEPROM:MAGIC_MISSING;");
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();  // SOFORT ins EEPROM schreiben / write to EEPROM immediately / เขียนลง EEPROM ทันที
    Serial.print("CFG:SN:NEW:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    return;
  }
  
  uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR+2);
  
  // --- Gemeinsam: REG + PCB aus EEPROM lesen (Offset ist für v1/v2/v3 gleich) ---
  // --- Common: read REG + PCB from EEPROM (same offset for v1/v2/v3) ---
  char regbuf[9];
  char pcbbuf[9];
  for (int i = 0; i < 8; i++) regbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+3+i);
  for (int i = 0; i < 8; i++) pcbbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+11+i);
  regbuf[8] = 0;
  pcbbuf[8] = 0;
  
  // --- Fall 2: v1/v2 EEPROM → kein SN-Feld → upgraden / v1/v2 EEPROM → no SN field → upgrade ---
  if (ver <= 2) {
    uint8_t storedCs = EEPROM.read(EEPROM_BASE_ADDR+19);
    uint8_t cs = calcCfgChecksum(ver, regbuf, pcbbuf, "         ");  // leere SN für v1/v2 / empty SN
    if (cs != storedCs) {
      // CRC fehlerhaft → trotzdem upgraden mit neuer SN / CRC corrupt → upgrade with new SN
      Serial.println("CFG:EEPROM:CRC_FAIL_v1v2;");
    }
    // REG + PCB aus v1/v2 übernehmen (falls CRC ok) oder Defaults behalten
    if (cs == storedCs) {
      int ri = 0;
      for (int i = 0; i < 8; i++) if (regbuf[i] != 0 && regbuf[i] != ' ') CFG_AIRCRAFT_REG[ri++] = regbuf[i];
      if (ri > 0) CFG_AIRCRAFT_REG[ri] = 0;
      int pi = 0;
      for (int i = 0; i < 8; i++) {
        if (pcbbuf[i] == 0 || pcbbuf[i] == ' ') { int j = i; while (j < 8 && (pcbbuf[j] == 0 || pcbbuf[j] == ' ')) j++; if (j >= 8) break; }
        CFG_PCB_VERSION[pi++] = pcbbuf[i];
      }
      while (pi > 0 && CFG_PCB_VERSION[pi-1] == ' ') pi--;
      if (pi > 0) CFG_PCB_VERSION[pi] = 0;
      refreshPCBVersionFlags();
    }
    // Neue SN generieren UND als v3 speichern / Generate new SN AND save as v3
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();  // Upgrade auf v3 + SN persistieren / upgrade to v3 + persist SN
    Serial.print("CFG:SN:UPGRADE_v1v2:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    Serial.print("CFG:LOAD:REG="); Serial.print(CFG_AIRCRAFT_REG);
    Serial.print(";PCB="); Serial.print(CFG_PCB_VERSION);
    Serial.print(";SN="); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    return;
  }
  
  // --- Fall 3: v3+, aber andere Version als erwartet ---
  if (ver != EEPROM_FORMAT_VERSION) {
    Serial.print("CFG:EEPROM:VERSION_MISMATCH:"); Serial.print(ver);
    Serial.print("!="); Serial.println(EEPROM_FORMAT_VERSION);
    // Trotzdem versuchen, REG + PCB zu laden, dann neue SN speichern
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    Serial.print("CFG:SN:NEW_VER_MISMATCH:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    return;
  }
  
  // --- Fall 4: v3 mit korrekter Version → SN-Feld lesen ---
  char snbuf[10];
  for (int i = 0; i < 9; i++) snbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR+19+i);
  snbuf[9] = 0;
  
  uint8_t storedCs = EEPROM.read(EEPROM_BASE_ADDR+28);
  uint8_t cs = calcCfgChecksum(ver, regbuf, pcbbuf, snbuf);
  
  if (cs != storedCs) {
    // CRC-Fehler → REG/PCB mit Defaults, neue SN / CRC error → defaults + new SN
    Serial.println("CFG:EEPROM:CRC_FAIL_v3;");
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    Serial.print("CFG:SN:NEW_CRC:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    return;
  }
  
  // --- CRC OK → REG, PCB und SN laden ---
  int ri = 0;
  for (int i = 0; i < 8; i++) if (regbuf[i] != 0 && regbuf[i] != ' ') CFG_AIRCRAFT_REG[ri++] = regbuf[i];
  if (ri > 0) CFG_AIRCRAFT_REG[ri] = 0;
  
  int pi = 0;
  for (int i = 0; i < 8; i++) {
    if (pcbbuf[i] == 0 || pcbbuf[i] == ' ') { int j = i; while (j < 8 && (pcbbuf[j] == 0 || pcbbuf[j] == ' ')) j++; if (j >= 8) break; }
    CFG_PCB_VERSION[pi++] = pcbbuf[i];
  }
  while (pi > 0 && CFG_PCB_VERSION[pi-1] == ' ') pi--;
  if (pi > 0) CFG_PCB_VERSION[pi] = 0;
  refreshPCBVersionFlags();
  
  // SN laden / load SN
  int si = 0;
  for (int i = 0; i < 8; i++) if (snbuf[i] != 0 && snbuf[i] != ' ') CFG_SERIAL_NUMBER[si++] = snbuf[i];
  CFG_SERIAL_NUMBER[si] = 0;
  
  if (si == 0) {
    // SN-Feld leer (sollte nicht vorkommen) → neu generieren + speichern
    // Empty SN field (should not happen) → generate + save
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    Serial.print("CFG:SN:NEW_EMPTY:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
  } else {
    // SN erfolgreich aus EEPROM geladen – KEIN saveHWInfo() nötig!
    // SN loaded from EEPROM – NO saveHWInfo() needed!
    Serial.print("CFG:SN:LOADED:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
  }
  
  Serial.print("CFG:LOAD:REG="); Serial.print(CFG_AIRCRAFT_REG);
  Serial.print(";PCB="); Serial.print(CFG_PCB_VERSION);
  Serial.print(";SN="); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
}

void saveHWInfo() {
  // Prepare buffers (pad with spaces) / Puffer vorbereiten (mit Leerzeichen auffüllen) / เตรียมบัฟเฟอร์ (เติมด้วยช่องว่าง)
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
  snbuf[8] = 0;  // null terminator for checksum consistency
  
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
  Serial.print("ms, REG="); Serial.print(CFG_AIRCRAFT_REG);
  Serial.print(", SN="); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
  
  // Show firmware and PCB version at boot (non-blocking, auto-clears after 5s in loop)
  {
    char fwDisp[8];
    snprintf(fwDisp, 8, "FRM %s", FW_VERSION);
    char* sp = strchr(CFG_PCB_VERSION, ' ');
    char pcbDisp[8];
    snprintf(pcbDisp, 8, "PCb %s", sp ? sp + 1 : CFG_PCB_VERSION);
    displayText(fwDisp, pcbDisp);
    bootMessageShown = true;
    bootMessageStart = millis();
  }
  
  // Boot fade-in (non-blocking backlight)
  bootState = BOOT_FADE;
  bootFadeStart = millis();
  desiredBrightness = 0;
  applyLEDOutputs();
  forceSendNext = true;
  maybeSendIdentStartup();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  
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
  
  // Boot/version message timeout (also used by VER/IDENT) – non-blocking clear
  if (bootMessageShown && (now - bootMessageStart) >= BOOT_MESSAGE_DURATION_MS) {
    bootMessageShown = false;
    displayText("      ", "      ");
  }
  
  // ── Boot backlight welcome pulse (fade-IN + fade-OUT, non-blocking) ──
  if (bootState == BOOT_FADE) {
    unsigned long elapsed = now - bootFadeStart;
    unsigned long totalMs = BOOT_FADE_DURATION_MS * 2;  // in + out
    if (elapsed >= totalMs) {
      desiredBrightness = 0;
      applyLEDOutputs();
      bootState = BOOT_RUNNING;
    } else if (elapsed < BOOT_FADE_DURATION_MS) {
      // Phase 1: fade IN  0 → 255
      desiredBrightness = (uint8_t)((unsigned long)255 * elapsed / BOOT_FADE_DURATION_MS);
      applyLEDOutputs();
    } else {
      // Phase 2: fade OUT  255 → 0
      unsigned long outElapsed = elapsed - BOOT_FADE_DURATION_MS;
      desiredBrightness = 255 - (uint8_t)((unsigned long)255 * outElapsed / BOOT_FADE_DURATION_MS);
      applyLEDOutputs();
    }
    return;  // Skip main loop during fade
  }

  processSerialTokensFromHost();
  
  // --- DATA mode: VHF3 LED edge detection & XFER swap ---
  bool vhf3Led = (desiredLedState & 0x0004) != 0;  // VHF3 LED = LED2 bit 2
  if (vhf3Led && !dataMode) {
    // VHF3 LED just turned ON → enter DATA mode, dAtA on left
    dataMode = true;
    dataOnLeft = true;
    strncpy(displayLeft, " dAtA ", 6); displayLeft[6] = 0;
    updateDisplay(DISP_LEFT, displayLeft);
  } else if (!vhf3Led && dataMode) {
    // VHF3 LED turned OFF → exit DATA mode
    dataMode = false;
  }
  
  if (now - lastLoopTs < LOOP_INTERVAL_MS) return;
  lastLoopTs = now;
  
  // Read shift registers (do this first to get current button state)
  readShiftRegisters(inputState1, inputState2);
  
  // --- DATA mode: XFER swap ---
  if (dataMode && (inputState1 & BUTTON_XFER_MASK1) && !(lastInputState1 & BUTTON_XFER_MASK1)) {
    // XFER just pressed in DATA mode → swap dAtA side, restore cached value on old side
    dataOnLeft = !dataOnLeft;
    if (dataOnLeft) {
      // dAtA moves to left, restore cached DSP2 on right
      strncpy(displayLeft, " dAtA ", 6); displayLeft[6] = 0;
      updateDisplay(DISP_LEFT, displayLeft);
      if (lastDsp2Val[0]) {
        int elen = 0; for (const char* p = lastDsp2Val; *p; p++) if (*p != '.') elen++;
        int pad = 6 - elen;
        if (pad > 0) { memset(displayRight, ' ', pad); strncpy(displayRight + pad, lastDsp2Val, 12 - pad); }
        else { strncpy(displayRight, lastDsp2Val, 12); }
        displayRight[11] = 0;
        updateDisplay(DISP_RIGHT, displayRight);
      }
    } else {
      // dAtA moves to right, restore cached DSP1 on left
      strncpy(displayRight, " dAtA ", 6); displayRight[6] = 0;
      updateDisplay(DISP_RIGHT, displayRight);
      if (lastDsp1Val[0]) {
        int elen = 0; for (const char* p = lastDsp1Val; *p; p++) if (*p != '.') elen++;
        int pad = 6 - elen;
        if (pad > 0) { memset(displayLeft, ' ', pad); strncpy(displayLeft + pad, lastDsp1Val, 12 - pad); }
        else { strncpy(displayLeft, lastDsp1Val, 12); }
        displayLeft[11] = 0;
        updateDisplay(DISP_LEFT, displayLeft);
      }
    }
  }
  
  // Detect button changes (debounced)
  bool inputChanged = false;
  if (inputState1 != lastInputState1 || inputState2 != lastInputState2) {
    if (now - lastDebounceTs >= CFG_BUTTON_DEBOUNCE) {
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      inputChanged = true;
      
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
}
