// MIP Control Script, (w) 2025 M. Quatember / BigMaQ

// Setze diese Zeile auf 1, um Bootsequenz und alle Startup-Ausgaben IMMER zu unterdrÃƒÂ¼cken
#define SUPPRESS_BOOTSEQ 0

// Panel-Identifikation beim Start anzeigen (Sekunden)
#define IDENT_DISPLAY_MS 2000
// Protocol-compatible with RMP/ACP: VER, IDENT, RESET, REQ, LEDx, INx, BL/AN/DISP_BL
// Chrono: 2x MAX7219 in series, driving 3 logical rows (CHR/UTC/ET) via time-multiplex
// BRK Panel: 2 LED drivers in series; Backlight drivers: 3 in series

// 2026-06-06
// Added LED8 bank for GS_FO on A7.
// Because 16 LEDs were apparently not enough.

#include <LedControl.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/io.h>

// ============================================================================
// PIN DEFINITIONS (placeholders from schematic; adjust as needed)
// ============================================================================
// MAX7219 chain for Chrono (align to schematic nets)
// Arduino Mega digital equivalents for analog pins:
// A8=62, A9=63, A10=64, A11=65, A12=66, A13=67, A14=68, A15=69
const int maxDataPin = 62;  // CHR_DISP_DIN -> A8
const int maxClkPin = 63;   // CHR_DISP_CLK -> A9
const int maxCsPin = 64;    // CHR_DISP_LOAD (CS) -> A10
// Two devices in series
LedControl lc = LedControl(maxDataPin, maxClkPin, maxCsPin, 2);

// LED Drivers in series
// BRK (2x) latch/clk/data
const int brkLatchPin = 2;  // BRK_LEDDRV_LATCH
const int brkDataPin = 4;   // BRK_LEDDRV_DATA
const int brkClkPin = 3;    // BRK_LEDDRV_CLK

// Backlight drivers (3x) share same Latch/Clk/Data (using separate chain)
const int blLatchPin = 51;  // BLTDRV_LATCH
const int blClkPin = 52;    // BLTDRV_CLK
const int blDataPin = 53;   // BLTDRV_DATA

// Direct GPIO LED outputs (16 lines, two 8-bit banks, ordered by resistor pins)
const int ledGpioPins1[8] = { 5, 7, 12, 16, 17, 20, 21, 24 };
const int ledGpioPins2[8] = { 25, 26, 27, 30, 32, 44, 46, 60 };  // A6 as digital 60
const int ledGpioPins3[8] = { 61, -1, -1, -1, -1, -1, -1, -1 };  // A7 as digital 61

// Annunciator & Backlight PWM (hardware PWM pins)
const int annuPWM = 11;       // ANNU_BRT
const int backlightPWM = 10;  // CON_BRT
const int displayPWM = 9;     // CON_DIM (optional PWM dim chain)

// Input shift registers (HC165) - direct inputs grouped as INx
// CHR_IS (Input Shifter) pins per schematic
const int inDataPin = 67;   // CHR_IS_DATA -> A13
const int inClkPin = 66;    // CHR_IS_CLOCK -> A12
const int inLatchPin = 65;  // CHR_IS_LATCH -> A11

// Direct GPIO inputs on Arduino Mega (button inputs only, ordered low->high)
// Use every free GPIO that is not tied to shift/MAX/LED driver clocks/data/latch or LED outputs
const int directInPins[] = {
  6, 8, 13, 14, 15, 18, 19, 22,
  23, 28, 29, 31, 33, 34, 35, 36,
  37, 38, 39, 40, 41, 42, 43, 45,
  47, 48, 49, 50, 68, 69
};
const int NUM_DIRECT_PINS = sizeof(directInPins) / sizeof(directInPins[0]);
const int NUM_DIRECT_GROUPS = (NUM_DIRECT_PINS + 7) / 8;  // pack into IN3+ groups

// Analog inputs (Ax): forward raw via serial (do NOT include pins used for MAX/shift regs)
const int analogPins[] = { A0, A1, A2, A3, A4, A5 };

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "MIP, v1.2 MAQ";
const char FW_VERSION[] = "1.2";
const char PANEL_SN_PREFIX[] = "MIP-";
bool identSentOnStart = false;

// Serial Number
char CFG_SERIAL_NUMBER[10] = "";

// Config variables
char CFG_AIRCRAFT_REG[9] = "D-A320";
char CFG_PCB_VERSION[9] = "PCB 1.0";
bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";

// ============================================================================
// STATE: LED drivers and brightness
// ============================================================================
// BRK drivers: 2x 8-bit -> 16 bits
uint16_t desiredBrkLed = 0x0000;
uint16_t hwBrkLed = 0x0000;

// Backlight drivers: 3x 8-bit -> 24 bits (pack into uint32_t lower 24 bits)
uint32_t desiredBlLed = 0x000000;
uint32_t hwBlLed = 0x000000;

uint8_t desiredGpioLed1 = 0x00;  // direct GPIO bank 1
uint8_t desiredGpioLed2 = 0x00;  // direct GPIO bank 2
uint8_t desiredGpioLed3 = 0x00;  // direct GPIO bank 3
uint8_t hwGpioLed1 = 0x00;
uint8_t hwGpioLed2 = 0x00;

// PWM brightness
uint8_t desiredAnLevel = 0;    // Annunciator bright
uint8_t desiredBlLevel = 0;    // Backlight bright
uint8_t desiredDispLevel = 0;  // Display PWM bright (optional)

uint8_t hwAnLevel = 0, hwBlLevel = 0, hwDispLevel = 0;

// Display brightness for MAX7219 (0-15)
uint8_t DISP_BL_LEVEL = 15;

// ============================================================================
// Chrono displays: Device 0 = UTC(6), Device 1 = CHR(4)+ET(4)
// ============================================================================
String chronoRowCHR = "nnIP ";
String chronoRowUTC = " boot ";
String chronoRowET = "    ";

// ============================================================================
// INPUT STATE
// ============================================================================
uint8_t inShift1 = 0x00;
uint8_t inShift2 = 0x00;
uint8_t lastInShift1 = 0x00;
uint8_t lastInShift2 = 0x00;

// Direct GPIO groups packed into bytes (IN3, IN4, IN5 for 17 pins)
uint8_t inDirect[NUM_DIRECT_GROUPS] = { 0 };
uint8_t lastInDirect[NUM_DIRECT_GROUPS] = { 0 };

// DIAG combo masks (pressed = 1)
const uint8_t DIAG_MASK_RST = 0b00010000;   // IN1 pattern xxx1xxxx
const uint8_t DIAG_MASK_CHR = 0b00001000;   // IN1 pattern xxxx1xxx
const uint8_t DIAG_MASK_DATE = 0b00010000;  // IN6 pattern xxx1xxxx
bool diagComboLatched = false;

// DIAG sequence runtime
bool diagRunning = false;
bool diagMode = false;  // When true, disable decimal points in displays
unsigned long diagStartTs = 0;
// Analog input state
int lastSentAnalog[6] = { 0 };

// ============================================================================
// RUNTIME
// ============================================================================
char serialAccum[64] = "";
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;
unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
bool forceSendNext = false;
bool analogReportEnabled = false;  // Gate analog reporting (enable via ANALOG_EN:1)
int analogDeadband = 8;

// IDENT delay for scan clarity (non-blocking)
bool identDelayActive = false;
unsigned long identDelayStart = 0;
const unsigned long IDENT_DELAY_MS = 3000;  // Boot display timeout

bool ackEnabled = false;  // gate serial ACK echo for commands
// When true, `initDisplays()` will avoid printing its debug message (used on EXTRF skips)
// By default suppress noisy init prints on (re)connect; IDENT/STATUS still
// sent when requested by host via VER/REQ.
bool suppress_init_debug = true;

// Chrono state (seconds mode: MM:SS)
bool chronoRunning = false;
unsigned long chronoSeconds = 0;

// ============================================================================
// HELPERS
// ============================================================================
// HELPER: bin8 writes 8-char binary string into provided buffer
void bin8(char* out, byte b) {
  for (int i = 7; i >= 0; i--) out[7 - i] = ((b & (1 << i)) ? '1' : '0');
  out[8] = 0;
}
// Helper: case-insensitive string compare
bool strieq(const char* a, const char* b) {
  while (*a && *b) {
    char ca = (*a >= 'a' && *a <= 'z') ? *a - 32 : *a;
    char cb = (*b >= 'a' && *b <= 'z') ? *b - 32 : *b;
    if (ca != cb) return false;
    a++;
    b++;
  }
  return *a == *b;
}
int atoi_s(const char* s) {
  int v = 0;
  bool neg = false;
  while (*s == ' ') s++;
  if (*s == '-') {
    neg = true;
    s++;
  }
  while (*s >= '0' && *s <= '9') {
    v = v * 10 + (*s - '0');
    s++;
  }
  return neg ? -v : v;
}
uint8_t parseBin8(const char* s) {
  uint8_t v = 0;
  for (int i = 0; s[i]; i++) {
    if (s[i] == '0' || s[i] == '1') v = (v << 1) | (s[i] == '1');
  }
  return v;
}
String trimToDigits(const String& src, int maxDigits);  // forward

void shiftOut16(uint16_t bits, int latch, int clk, int data) {
  digitalWrite(latch, LOW);
  shiftOut(data, clk, MSBFIRST, (bits >> 8) & 0xFF);
  shiftOut(data, clk, MSBFIRST, bits & 0xFF);
  digitalWrite(latch, HIGH);
}

void shiftOut24(uint32_t bits24, int latch, int clk, int data) {
  digitalWrite(latch, LOW);
  shiftOut(data, clk, MSBFIRST, (bits24 >> 16) & 0xFF);
  shiftOut(data, clk, MSBFIRST, (bits24 >> 8) & 0xFF);
  shiftOut(data, clk, MSBFIRST, bits24 & 0xFF);
  digitalWrite(latch, HIGH);
}

void applyOutputs() {
  // LED drivers
  shiftOut16(desiredBrkLed, brkLatchPin, brkClkPin, brkDataPin);
  shiftOut24(desiredBlLed, blLatchPin, blClkPin, blDataPin);
  // Direct GPIO LED banks
  for (int i = 0; i < 8; i++) digitalWrite(ledGpioPins1[i], (desiredGpioLed1 & (1 << i)) ? HIGH : LOW);
  for (int i = 0; i < 8; i++) digitalWrite(ledGpioPins2[i], (desiredGpioLed2 & (1 << i)) ? HIGH : LOW);
  // BEGIN fix: applyOutputs skips unused pins (-1)
  for (int i = 0; i < 8; i++) {
    int pin = ledGpioPins3[i];
    if (pin < 0) continue;
    digitalWrite(pin, (desiredGpioLed3 & (1 << i)) ? HIGH : LOW);
  }

  // PWM brightness
  analogWrite(annuPWM, desiredAnLevel);
  analogWrite(backlightPWM, desiredBlLevel);
  analogWrite(displayPWM, desiredDispLevel);
  // cache
  hwBrkLed = desiredBrkLed;
  hwBlLed = desiredBlLed;
  hwGpioLed1 = desiredGpioLed1;
  hwGpioLed2 = desiredGpioLed2;
  hwAnLevel = desiredAnLevel;
  hwBlLevel = desiredBlLevel;
  hwDispLevel = desiredDispLevel;
}

void setLEDState(uint16_t brkBits, uint32_t blBits, uint8_t an, uint8_t bl, uint8_t dispPWM) {
  desiredBrkLed = brkBits;
  desiredBlLed = blBits;
  desiredAnLevel = an;
  desiredBlLevel = bl;
  desiredDispLevel = dispPWM;
  applyOutputs();
}

// ============================================================================
// DISPLAY
// ============================================================================
void initDisplays() {
  // Device 0: 6 digits for UTC
  lc.shutdown(0, false);  // Wake up
  delay(10);              // Wait for chip to stabilize

  // CRITICAL: Configure scan limit FIRST - tells MAX7219 how many digits to use
  lc.setScanLimit(0, 5);  // Use 6 digits (0-5)

  // Then set intensity and clear
  lc.setIntensity(0, DISP_BL_LEVEL);  // Set brightness (0-15)
  lc.clearDisplay(0);                 // Clear all data

  // Explicitly set all 6 rows to 0
  for (int d = 0; d < 6; d++) lc.setRow(0, d, 0x00);

  // Device 1: 8 digits for CHR+ET
  lc.shutdown(1, false);  // Wake up
  delay(10);              // Wait for chip to stabilize

  // CRITICAL: Configure scan limit FIRST
  lc.setScanLimit(1, 7);  // Use 8 digits (0-7)

  // Then set intensity and clear
  lc.setIntensity(1, DISP_BL_LEVEL);  // Set brightness (0-15)
  lc.clearDisplay(1);                 // Clear all data

  // Explicitly set all 8 rows to 0
  for (int d = 0; d < 8; d++) lc.setRow(1, d, 0x00);
}

byte charTo7Seg(char c) {
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
    case 'H': return 0b00110111;
    case 'U': return 0b00111110;
    case 'S': return 0b01011011;
    case 'L': return 0b00001110;
    case 'J': return 0b00111000;
    case 'I': return 0b00110000;
    case 'G': return 0b01011110;
    case 'O': return 0b01111110;
    case 'P': return 0b01100111;
    case 'Y': return 0b00111011;
    case 'N': return 0b00010101;
    case 'M': return 0b00110111;
    case 'T': return 0b00001111;
    case 'R': return 0b00000101;
    case '-': return 0b00000001;
    case '$': return 0b00000100;
    case ' ': return 0b00000000;
    default: return 0b00000000;
  }
}

void buildDigitBuffer(const String& src, byte* out, int maxDigits,
                      bool colonMode = false, int colonIdx = 2) {
  for (int i = 0; i < maxDigits; i++)
    out[i] = charTo7Seg(' ');  // Initialize with space (blank), not '0'

  int len = 0;

  for (int i = 0; i < src.length(); i++) {
    char c = src.charAt(i);

    if (c != '.')
      len++;
  }

  int startDigit = maxDigits - len;

  if (startDigit < 0)
    startDigit = 0;

  int digit = startDigit;
  bool sawDot = false;

  for (int i = 0; i < src.length() && digit < maxDigits; i++) {
    char c = src.charAt(i);

    if (c == '.') {
      sawDot = true;

      if (!colonMode && digit > startDigit)
        out[digit - 1] |= 0b10000000;

      continue;
    }

    out[digit] = charTo7Seg(c);
    digit++;
  }

  bool hasDigit = false;

  for (int i = 0; i < src.length(); i++) {
    if (isDigit(src.charAt(i))) {
      hasDigit = true;
      break;
    }
  }

  if (colonMode && hasDigit && colonIdx >= 0 && colonIdx < maxDigits) {
    out[colonIdx] |= 0b10000000;
  }
}

String trimToDigits(const String& src, int maxDigits) {
  String out = "";
  int digits = 0;

  for (int i = 0; i < src.length(); i++) {
    char c = src.charAt(i);

    if (c == '.')
      continue;  // Skip decimal points, keep collecting digits

    if (digits >= maxDigits)
      break;

    out += c;
    digits++;
  }

  return out;
}

// Helper: Pad string with leading zeros to desired length
String padWithZeros(const String& src, int targetLen) {
  String result = src;
  while (result.length() < targetLen) {
    result = "0" + result;
  }
  return result;
}

// ============================================================================
// DIAG HELPERS
// ============================================================================
void diagDisplayNumberWalk(unsigned long elapsed) {
  // elapsed in ms; advance every 300ms
  int step = (elapsed / 300) % 10;
  // Build rolling digits for UTC (6), CHR (4), ET (4)
  String utc = "";
  for (int i = 0; i < 6; i++) utc += char('0' + ((step + i) % 10));
  String chr = "";
  for (int i = 0; i < 4; i++) chr += char('0' + ((step + i + 6) % 10));
  String et = "";
  for (int i = 0; i < 4; i++) et += char('0' + ((step + i + 2) % 10));
  chronoRowUTC = utc;
  chronoRowCHR = chr;
  chronoRowET = et;
  renderChronoDisplays();
}

void diagSegmentTest(bool on) {
  byte val = on ? 0xFF : 0x00;
  for (int d = 0; d < 6; d++) lc.setRow(0, d, val);
  for (int d = 0; d < 8; d++) lc.setRow(1, d, val);
}

void diagSegmentSweep(unsigned long elapsed) {
  // Sweep through each segment bit individually
  // 7 segments per digit + DP = 8 bits, cycle every 100ms
  int segmentBit = (elapsed / 100) % 8;
  byte pattern = (1 << segmentBit);

  // Apply to all digits
  for (int d = 0; d < 6; d++) lc.setRow(0, d, pattern);
  for (int d = 0; d < 8; d++) lc.setRow(1, d, pattern);
}

const int DIAG_LED_TOTAL = 16 + 24 + 8 + 8;  // BRK + Backlight + GPIO1 + GPIO2

void diagLedWalkStep(unsigned long elapsed) {
  int step = (elapsed / 120) % DIAG_LED_TOTAL;
  desiredBrkLed = 0;
  desiredBlLed = 0;
  desiredGpioLed1 = 0;
  desiredGpioLed2 = 0;

  if (step < 8) {
    desiredGpioLed1 = (1 << (7 - step));
  } else if (step < 16) {
    desiredGpioLed2 = (1 << (15 - step));
  } else if (step < 32) {
    desiredBrkLed = (1 << (31 - step));  // walk MSB->LSB for clarity
  } else {
    int idx = step - 32;  // 0..23
    desiredBlLed = (uint32_t(1) << (23 - idx));
  }
  applyOutputs();

  // Update displays to show current LED byte and value
  // Top (CHR): LEDx where x=1..7 (active byte)
  // Middle (UTC): two-digit HEX value of the active byte
  // Lower (ET): "tE5t" (7-seg friendly for tESt)
  int activeLedByte = 0;  // 1..7
  uint8_t activeValue = 0x00;

  if (step < 8) {
    // GPIO bank 1 -> LED1
    activeLedByte = 1;
    activeValue = desiredGpioLed1;
  } else if (step < 16) {
    // GPIO bank 2 -> LED2
    activeLedByte = 2;
    activeValue = desiredGpioLed2;
  } else if (step < 32) {
    // BRK 16-bit -> LED3 (high), LED4 (low)
    uint8_t hi = (desiredBrkLed >> 8) & 0xFF;
    uint8_t lo = desiredBrkLed & 0xFF;
    if (hi) {
      activeLedByte = 3;
      activeValue = hi;
    } else {
      activeLedByte = 4;
      activeValue = lo;
    }
  } else {
    // Backlight 24-bit -> LED5 (MSB), LED6, LED7 (LSB)
    uint8_t b2 = (desiredBlLed >> 16) & 0xFF;  // LED5
    uint8_t b1 = (desiredBlLed >> 8) & 0xFF;   // LED6
    uint8_t b0 = (desiredBlLed)&0xFF;          // LED7
    if (b2) {
      activeLedByte = 5;
      activeValue = b2;
    } else if (b1) {
      activeLedByte = 6;
      activeValue = b1;
    } else {
      activeLedByte = 7;
      activeValue = b0;
    }
  }

  char topStr[5];  // "LEDx"
  snprintf(topStr, sizeof(topStr), "LED%d", activeLedByte);
  // Center HEX value in 6-digit UTC: "  XX  "
  char midStr6[7];
  char hex2[3];
  snprintf(hex2, sizeof(hex2), "%02X", activeValue);
  snprintf(midStr6, sizeof(midStr6), "  %s  ", hex2);

  chronoRowCHR = String(topStr);
  chronoRowUTC = String(midStr6);
  chronoRowET = "tE5t";  // 7-seg friendly for tESt (S ~ digit 5)
  renderChronoDisplays();
}

void startDiagSequence() {
  diagRunning = true;
  diagMode = true;  // Disable decimal points during DIAG
  diagStartTs = millis();
  Serial.println("DIAG:START");
}

// Manuelle Bootsequenz (wie bisher, aber nur auf Kommando)
void runManualBootSequence();

void stopDiagSequence() {
  diagRunning = false;
  diagMode = false;  // Re-enable decimal points
  // Clear outputs and restore baseline labels
  setLEDState(0x0000, 0x000000, 0, 0, 0);
  chronoRowCHR = "nnIP ";
  chronoRowUTC = " boot ";
  chronoRowET = "    ";
  renderChronoDisplays();
  Serial.println("DIAG:DONE");
  forceSendNext = true;
}

void runDiagSequence() {
  if (!diagRunning) return;
  unsigned long elapsed = millis() - diagStartTs;

  if (elapsed < 2000) {
    chronoRowCHR = "DIAG";
    chronoRowUTC = "START ";
    chronoRowET = "DIAG";
    renderChronoDisplays();
  } else if (elapsed < 8000) {
    diagDisplayNumberWalk(elapsed - 2000);
  } else if (elapsed < 10000) {
    // Segment test: all segments ON for 2 seconds
    diagSegmentTest(true);
  } else if (elapsed < 12000) {
    // Segment sweep: walk through each segment for 2 seconds (8 segments Ãƒâ€” 250ms)
    diagSegmentSweep(elapsed - 10000);
  } else if (elapsed < 23000) {
    diagLedWalkStep(elapsed - 12000);
  } else {
    diagSegmentTest(false);
    stopDiagSequence();
  }
}

int indexAfterDigits(const String& src, int digitBudget) {
  // Return index in src after consuming digitBudget digits (dots don't count)
  int digits = 0;
  for (int i = 0; i < src.length(); i++) {
    char c = src.charAt(i);
    if (c != '.') digits++;
    if (digits >= digitBudget) return i + 1;
  }
  return src.length();
}

void updateDisplay(int device, const String& text, int maxDigits) {
  // Update display with text, RMP-style
  // MAX7219: digit 0 is rightmost, higher digits are leftmost
  int textPos = 0;
  for (int digitPos = 0; digitPos < maxDigits; digitPos++) {
    if (textPos >= text.length()) {
      lc.setRow(device, (maxDigits - 1) - digitPos, 0b00000000);  // Blank remaining
      continue;
    }

    char c = text.charAt(textPos);
    byte pattern = charTo7Seg(c);

    lc.setRow(device, (maxDigits - 1) - digitPos, pattern);
    textPos++;
  }
}

void renderChronoDisplays() {
  // Device 0: UTC (6 digits)
  // Device 1: CHR (4 digits) on rows 0-3 (right)
  //           ET (4 digits) on rows 4-7 (left)

  byte utcBuf[6];
  buildDigitBuffer(chronoRowUTC, utcBuf, 6, !diagMode, 1);  // Use colons only outside DIAG mode
  for (int i = 0; i < 6; i++) lc.setRow(0, 5 - i, utcBuf[i]);

  // Render CHR on device 1, rows 0-3 (right, rightmost)
  byte chrBuf[4];
  buildDigitBuffer(chronoRowCHR, chrBuf, 4, !diagMode, 2);
  for (int i = 0; i < 4; i++) lc.setRow(1, 3 - i, chrBuf[i]);

  // Render ET on device 1, rows 4-7 (left, leftmost)
  byte etBuf[4];
  buildDigitBuffer(chronoRowET, etBuf, 4, !diagMode, 2);
  for (int i = 0; i < 4; i++) lc.setRow(1, 7 - i, etBuf[i]);
}

// --- Ende Patch ---

// ============================================================================
// INPUTS
// ============================================================================
void readShift(uint8_t& in1, uint8_t& in2) {
  digitalWrite(inLatchPin, LOW);
  delayMicroseconds(20);
  digitalWrite(inLatchPin, HIGH);
  delayMicroseconds(20);
  byte raw1 = shiftIn(inDataPin, inClkPin, MSBFIRST);
  byte raw2 = shiftIn(inDataPin, inClkPin, MSBFIRST);
  in1 = ~raw1;
  in2 = ~raw2;
  digitalWrite(inLatchPin, LOW);
  delayMicroseconds(20);
  digitalWrite(inLatchPin, HIGH);
  delayMicroseconds(20);
  for (int i = 0; i < 16; i++) {
    digitalWrite(inClkPin, LOW);
    delayMicroseconds(5);
    digitalWrite(inClkPin, HIGH);
    delayMicroseconds(5);
  }
}

uint8_t readDirectGroup(const int* pins, int count) {
  uint8_t v = 0x00;  // active-high reporting (pressed=1), unused bits stay 0
  for (int i = 0; i < count && i < 8; i++) {
    int r = digitalRead(pins[i]);
    if (r == LOW) v |= (1 << i);  // invert because buttons are pull-up (LOW when pressed)
  }
  return v;
}

// ============================================================================
// DIAG / SETUP HELPERS (serial-driven)
// ============================================================================
void sendDiagReport() {
  char b8[9];
  bin8(b8, inShift1);
  Serial.print("DIAG:STATE;IN1:");
  Serial.print(b8);
  bin8(b8, inShift2);
  Serial.print(";IN2:");
  Serial.print(b8);
  for (int g = 0; g < NUM_DIRECT_GROUPS; g++) {
    bin8(b8, inDirect[g]);
    Serial.print(";IN");
    Serial.print(3 + g);
    Serial.print(":");
    Serial.print(b8);
  }
  Serial.print(";LED_BRK:");
  Serial.print(hwBrkLed, HEX);
  Serial.print(";LED_BL:");
  Serial.print(hwBlLed, HEX);
  Serial.print(";LED_GPIO1:");
  Serial.print(hwGpioLed1, HEX);
  Serial.print(";LED_GPIO2:");
  Serial.print(hwGpioLed2, HEX);
  Serial.print(";BRT_AN:");
  Serial.print(hwAnLevel);
  Serial.print(";BRT_BL:");
  Serial.print(hwBlLevel);
  Serial.print(";BRT_DISP:");
  Serial.print(hwDispLevel);
  Serial.print(";ANALOG_DB:");
  Serial.print(analogDeadband);
  Serial.print(";INTENSITY:");
  Serial.print(DISP_BL_LEVEL);
  Serial.print(";CHR:");
  Serial.print(chronoRowCHR);
  Serial.print(";UTC:");
  Serial.print(chronoRowUTC);
  Serial.print(";ET:");
  Serial.print(chronoRowET);
  Serial.println(";");
}

void sendSetupMenu() {
  Serial.println(
    "SETUP:MENU;"
    "ITEMS:BRIGHT,ANALOG_EN,ANALOG_DB;"
    "CMDS:"
    "SETUP:BRIGHT:AN=0-255,BL=0-255,DISP=0-15|"
    "ANALOG_EN:0/1|"
    "ANALOG_DB:1-100;");
}

// ============================================================================
// SERIAL
// ============================================================================

// --- EEPROM config v3 ---
const int EEPROM_BASE_ADDR = 0;
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t EEPROM_FORMAT_VERSION = 3;

void generateSerialNumber(char* out) {
  randomSeed(analogRead(A0) + analogRead(A1) + micros());
  const char hexChars[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) { out[i] = hexChars[random(0, 16)]; }
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
  if (magic != EEPROM_MAGIC) {
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    return;
  }
  uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR + 2);
  char regbuf[9];
  char pcbbuf[9];
  for (int i = 0; i < 8; i++) {
    regbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 3 + i);
    pcbbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 11 + i);
  }
  regbuf[8] = 0;
  pcbbuf[8] = 0;
  if (ver <= 2) {
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    return;
  }
  if (ver != EEPROM_FORMAT_VERSION) {
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    return;
  }
  char snbuf[10];
  for (int i = 0; i < 9; i++) snbuf[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 19 + i);
  snbuf[9] = 0;
  if (EEPROM.read(EEPROM_BASE_ADDR + 28) != calcCfgChecksum(ver, regbuf, pcbbuf, snbuf)) {
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
    return;
  }
  int ri = 0;
  for (int i = 0; i < 8; i++)
    if (regbuf[i] != 0 && regbuf[i] != ' ') CFG_AIRCRAFT_REG[ri++] = regbuf[i];
  CFG_AIRCRAFT_REG[ri] = 0;
  int pi = 0;
  for (int i = 0; i < 8; i++)
    if (pcbbuf[i] != 0 && pcbbuf[i] != ' ') CFG_PCB_VERSION[pi++] = pcbbuf[i];
  CFG_PCB_VERSION[pi] = 0;
  int si = 0;
  for (int i = 0; i < 8; i++)
    if (snbuf[i] != 0 && snbuf[i] != ' ') CFG_SERIAL_NUMBER[si++] = snbuf[i];
  CFG_SERIAL_NUMBER[si] = 0;
  if (si == 0) {
    generateSerialNumber(CFG_SERIAL_NUMBER);
    saveHWInfo();
  }
}

void saveHWInfo() {
  char regbuf[8];
  int rl = strlen(CFG_AIRCRAFT_REG);
  for (int i = 0; i < 8; i++) regbuf[i] = (i < rl) ? CFG_AIRCRAFT_REG[i] : ' ';
  char pcbbuf[8];
  int pl = strlen(CFG_PCB_VERSION);
  for (int i = 0; i < 8; i++) pcbbuf[i] = (i < pl) ? CFG_PCB_VERSION[i] : ' ';
  char snbuf[9];
  int sl = strlen(CFG_SERIAL_NUMBER);
  for (int i = 0; i < 8; i++) snbuf[i] = (i < sl) ? CFG_SERIAL_NUMBER[i] : ' ';
  snbuf[8] = 0;
  EEPROM.update(EEPROM_BASE_ADDR + 0, (uint8_t)(EEPROM_MAGIC & 0xFF));
  EEPROM.update(EEPROM_BASE_ADDR + 1, (uint8_t)((EEPROM_MAGIC >> 8) & 0xFF));
  EEPROM.update(EEPROM_BASE_ADDR + 2, EEPROM_FORMAT_VERSION);
  for (int i = 0; i < 8; i++) EEPROM.update(EEPROM_BASE_ADDR + 3 + i, (uint8_t)regbuf[i]);
  for (int i = 0; i < 8; i++) EEPROM.update(EEPROM_BASE_ADDR + 11 + i, (uint8_t)pcbbuf[i]);
  for (int i = 0; i < 9; i++) EEPROM.update(EEPROM_BASE_ADDR + 19 + i, (uint8_t)snbuf[i]);
  EEPROM.update(EEPROM_BASE_ADDR + 28, calcCfgChecksum(EEPROM_FORMAT_VERSION, regbuf, pcbbuf, snbuf));
}

void triggerSoftwareReset() {
  Serial.flush();
  delay(50);
  wdt_enable(WDTO_15MS);
  while (true) {}
}

void sendIdentAndState() {
  Serial.print("IDENT:");
  Serial.print(PANEL_IDENT);
  if (CFG_SERIAL_NUMBER[0] != 0) {
    Serial.print(", SN:");
    Serial.print(PANEL_SN_PREFIX);
    Serial.print(CFG_SERIAL_NUMBER);
  }
  Serial.print(";STATE:RUNNING;");
  Serial.print("ANALOG_DB:");
  Serial.print(analogDeadband);
  Serial.print(";");
  Serial.println();
}

void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;
  char b8[9];
  bin8(b8, inShift1);
  Serial.print("IN1:");
  Serial.print(b8);
  Serial.print(";");
  bin8(b8, inShift2);
  Serial.print("IN2:");
  Serial.print(b8);
  Serial.print(";");
  for (int g = 0; g < NUM_DIRECT_GROUPS; g++) {
    Serial.print("IN");
    Serial.print(3 + g);
    Serial.print(":");
    bin8(b8, inDirect[g]);
    Serial.print(b8);
    Serial.print(";");
  }
  if (analogReportEnabled) {
    for (int i = 0; i < (int)(sizeof(analogPins) / sizeof(analogPins[0])); i++) {
      int v = analogRead(analogPins[i]);
      Serial.print("A");
      Serial.print(i);
      Serial.print(":");
      Serial.print(v);
      Serial.print(";");
    }
  }
  Serial.println();
  lastSendTs = now;
  forceSendNext = false;
}

void processIncomingLine(const char* line) {
  char token[32]; int tlen = 0;
  for (int i = 0;; i++) {
    char c = line[i];
    if (c == ';' || c == 0) {
      if (tlen == 0) { if (c == 0) break; else continue; }
      token[tlen] = 0; tlen = 0;
      int end = strlen(token) - 1; while (end >= 0 && token[end] == ' ') token[end--] = 0;
      char* tok = token; while (*tok == ' ') tok++;
      if (*tok == 0) { if (c == 0) break; else continue; }
      char* colon = strchr(tok, ':');
      if (!colon) {
        if (strieq(tok, "VER") || strieq(tok, "VERSION")) { sendIdentAndState(); identDelayActive = true; identDelayStart = millis(); }
        else if (strieq(tok, "REQ")) { forceSendNext = true; }
        else if (strieq(tok, "RESET")) { Serial.println("RESET:OK ;"); Serial.flush(); delay(50); wdt_enable(WDTO_15MS); while (true) {} }
        if (c == 0) break; else continue;
      }
      *colon = 0; char* key = tok; char* val = colon + 1;
      { int kend = (int)strlen(key) - 1; while (kend >= 0 && key[kend] == ' ') key[kend--] = 0; }
      while (*val == ' ') val++; int vend = (int)strlen(val) - 1; while (vend >= 0 && val[vend] == ' ') val[vend--] = 0;
      if (strieq(key, "LED1")) { desiredGpioLed1 = parseBin8(val); applyOutputs(); if (ackEnabled) { char b8[9]; bin8(b8, desiredGpioLed1); Serial.print("ACK:LED1:"); Serial.println(b8); } }
      else if (strieq(key, "LED2")) { desiredGpioLed2 = parseBin8(val); applyOutputs(); if (ackEnabled) { char b8[9]; bin8(b8, desiredGpioLed2); Serial.print("ACK:LED2:"); Serial.println(b8); } }
      else if (strieq(key, "LED8")) { desiredGpioLed3 = parseBin8(val); applyOutputs(); if (ackEnabled) { char b8[9]; bin8(b8, desiredGpioLed3); Serial.print("ACK:LED8:"); Serial.println(b8); } }
      else if (strieq(key, "LED3")) { uint8_t v = parseBin8(val); desiredBrkLed = (desiredBrkLed & 0x00FF) | ((uint16_t)v << 8); applyOutputs(); if (ackEnabled) { char b8[9]; bin8(b8, v); Serial.print("ACK:LED3:"); Serial.println(b8); } }
      else if (strieq(key, "LED4")) { uint8_t v = parseBin8(val); desiredBrkLed = (desiredBrkLed & 0xFF00) | (uint16_t)v; applyOutputs(); if (ackEnabled) { char b8[9]; bin8(b8, v); Serial.print("ACK:LED4:"); Serial.println(b8); } }
      else if (strieq(key, "LED5")) { uint8_t v = parseBin8(val); desiredBlLed = (desiredBlLed & 0x0000FFFF) | ((uint32_t)v << 16); applyOutputs(); if (ackEnabled) { char b8[9]; bin8(b8, v); Serial.print("ACK:LED5:"); Serial.println(b8); } }
      else if (strieq(key, "LED6")) { uint8_t v = parseBin8(val); desiredBlLed = (desiredBlLed & 0x00FF00FF) | ((uint32_t)v << 8); applyOutputs(); if (ackEnabled) { char b8[9]; bin8(b8, v); Serial.print("ACK:LED6:"); Serial.println(b8); } }
      else if (strieq(key, "LED7")) { uint8_t v = parseBin8(val); desiredBlLed = (desiredBlLed & 0x00FFFF00) | (uint32_t)v; applyOutputs(); if (ackEnabled) { char b8[9]; bin8(b8, v); Serial.print("ACK:LED7:"); Serial.println(b8); } }
      else if (strieq(key, "BL")) { desiredBlLevel = constrain(atoi_s(val), 0, 255); applyOutputs(); }
      else if (strieq(key, "AN")) { desiredAnLevel = constrain(atoi_s(val), 0, 255); applyOutputs(); }
      else if (strieq(key, "DISP_BL")) { DISP_BL_LEVEL = constrain(atoi_s(val), 0, 15); for (int dev = 0; dev < 2; dev++) lc.setIntensity(dev, DISP_BL_LEVEL); }
      else if (strieq(key, "DSP1")) { if (val[0] == '-') { chronoRowCHR = ""; } else { chronoRowCHR = padWithZeros(trimToDigits(String(val), 4), 4); } renderChronoDisplays(); if (ackEnabled) { Serial.print("ACK:DSP1:"); Serial.println(chronoRowCHR); } }
      else if (strieq(key, "DSP2")) { if (val[0] == '-') { chronoRowUTC = ""; } else { chronoRowUTC = padWithZeros(trimToDigits(String(val), 6), 6); } renderChronoDisplays(); if (ackEnabled) { Serial.print("ACK:DSP2:"); Serial.println(chronoRowUTC); } }
      else if (strieq(key, "DSP3")) { if (val[0] == '-') { chronoRowET = ""; } else { chronoRowET = padWithZeros(trimToDigits(String(val), 4), 4); } renderChronoDisplays(); if (ackEnabled) { Serial.print("ACK:DSP3:"); Serial.println(chronoRowET); } }
      else if (strieq(key, "DEBUG_ACK")) { ackEnabled = (val[0] == '1' || strieq(val, "ON")); Serial.print("ACK:DEBUG_ACK:"); Serial.println(ackEnabled ? "ON" : "OFF"); }
      else if (strieq(key, "DIAG")) { if (strieq(val, "RUN")) { startDiagSequence(); } else if (strieq(val, "STOP")) { stopDiagSequence(); } else { sendDiagReport(); } }
      else if (strieq(key, "SETUP")) {
        if (val[0] == 0) { sendSetupMenu(); }
        else { char upVal[32]; int uvi = 0; for (int vi = 0; val[vi] && uvi < 31; vi++) { upVal[uvi] = (val[vi] >= 'a' && val[vi] <= 'z') ? val[vi] - 32 : val[vi]; uvi++; } upVal[uvi] = 0;
          if (strncmp(upVal, "BRIGHT", 6) == 0) { char* anEq = strstr(upVal, "AN="); char* blEq = strstr(upVal, "BL="); char* dpEq = strstr(upVal, "DISP="); if (anEq) { desiredAnLevel = constrain(atoi_s(anEq + 3), 0, 255); } if (blEq) { desiredBlLevel = constrain(atoi_s(blEq + 3), 0, 255); } if (dpEq) { DISP_BL_LEVEL = constrain(atoi_s(dpEq + 5), 0, 15); } applyOutputs(); for (int dev = 0; dev < 2; dev++) lc.setIntensity(dev, DISP_BL_LEVEL); Serial.print("SETUP:BRIGHT:AN="); Serial.print(desiredAnLevel); Serial.print(";BL="); Serial.print(desiredBlLevel); Serial.print(";DISP="); Serial.println(DISP_BL_LEVEL); } else { sendSetupMenu(); } } }
      else if (strieq(key, "ANALOG_EN")) { analogReportEnabled = (val[0] == '1' || strieq(val, "ON")); Serial.print("ANALOG_EN:"); Serial.println(analogReportEnabled ? "ON" : "OFF"); }
      else if (strieq(key, "ANALOG_DB")) { analogDeadband = constrain(atoi_s(val), 1, 100); Serial.print("ANALOG_DB:"); Serial.println(analogDeadband); }
      else if (strieq(key, "DEBUG_SR")) { digitalWrite(inLatchPin, LOW); delayMicroseconds(20); digitalWrite(inLatchPin, HIGH); delayMicroseconds(20); byte raw1 = shiftIn(inDataPin, inClkPin, MSBFIRST); byte raw2 = shiftIn(inDataPin, inClkPin, MSBFIRST); char b8a[9]; bin8(b8a, raw1); Serial.print("DEBUG_SR:RAW1="); Serial.print(b8a); bin8(b8a, raw2); Serial.print(";RAW2="); Serial.print(b8a); bin8(b8a, ~raw1); Serial.print(";INV1="); Serial.print(b8a); bin8(b8a, ~raw2); Serial.print(";INV2="); Serial.println(b8a); }
      else if (strieq(key, "DEBUG_PIN")) { if (val[0] != 0) { int pin = atoi_s(val); pinMode(pin, INPUT_PULLUP); int state = digitalRead(pin); Serial.print("DEBUG_PIN:"); Serial.print(pin); Serial.print("="); Serial.println(state); } else { Serial.print("DEBUG_PIN:ALL="); for (int i = 0; i < NUM_DIRECT_PINS; i++) { Serial.print("P"); Serial.print(directInPins[i]); Serial.print(":"); Serial.print(digitalRead(directInPins[i])); Serial.print(";"); } Serial.println(); } }
      else if (strieq(key, "DEBUG_MAX")) { Serial.println("DEBUG_MAX: Setting all segments to ON"); for (int d = 0; d < 6; d++) lc.setRow(0, d, 0xFF); for (int d = 0; d < 8; d++) lc.setRow(1, d, 0xFF); Serial.println("DEBUG_MAX: All segments ON"); }
      else if (strieq(key, "REQ")) { forceSendNext = true; }
      else if (strieq(key, "SET") || strncmp(key, "SET ", 4) == 0) { char* rest; if (strieq(key, "SET")) { rest = val; } else { rest = strchr(token, ' '); if (rest) rest++; else rest = val; } while (*rest == ' ') rest++; char* er = rest + strlen(rest) - 1; while (er >= rest && *er == ' ') *er-- = 0; char* colonPos = strchr(rest, ':'); char* cmd = rest; char* arg = (char*)""; if (colonPos) { *colonPos = 0; arg = colonPos + 1; } for (int ci = 0; cmd[ci]; ci++) { if (cmd[ci] >= 'a' && cmd[ci] <= 'z') cmd[ci] -= 32; } while (*arg == ' ') arg++; char* ea = arg + strlen(arg) - 1; while (ea >= arg && *ea == ' ') *ea-- = 0;
        if (strcmp(cmd, "ENA") == 0) { if (strcmp(arg, SETTINGS_PIN) == 0) { settingsEnabled = true; Serial.println("SET ENA> ;"); } else { Serial.println("SET ENA:FAIL ;"); } }
        else if (strcmp(cmd, "FW") == 0 && settingsEnabled) { char* dotPos = strchr(arg, '.'); if (dotPos && dotPos > arg && dotPos[1] != 0) { *dotPos = 0; int major = atoi_s(arg); int minor = atoi_s(dotPos + 1); if (major >= 1 && major <= 9 && minor >= 0 && minor <= 9) { snprintf(CFG_PCB_VERSION, 9, "PCb %d.%d", major, minor); Serial.println("SET FW:OK ;"); } else Serial.println("SET FW:RANGE ;"); } else Serial.println("SET FW:FORMAT ;"); }
        else if (strcmp(cmd, "ACID") == 0 && settingsEnabled) { int alen = (int)strlen(arg); if (alen >= 3 && alen <= 8 && arg[1] == '-' && isAlpha(arg[0])) { bool valid = true; for (int i = 2; i < alen; i++) if (!isAlpha(arg[i])) { valid = false; break; } if (valid) { strncpy(CFG_AIRCRAFT_REG, arg, 8); CFG_AIRCRAFT_REG[8] = 0; Serial.println("SET ACID:OK ;"); } else Serial.println("SET ACID:FORMAT ;"); } else Serial.println("SET ACID:FORMAT ;"); }
        else if (strcmp(cmd, "SN") == 0 && settingsEnabled) { if (strlen(arg) == 8) { bool valid = true; for (int i = 0; i < 8 && valid; i++) { char c = arg[i]; if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) valid = false; } if (valid) { for (int i = 0; i < 8; i++) { CFG_SERIAL_NUMBER[i] = (arg[i] >= 'a' && arg[i] <= 'z') ? arg[i] - 32 : arg[i]; } CFG_SERIAL_NUMBER[8] = 0; Serial.print("SET SN:OK:"); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER); } else Serial.println("SET SN:FORMAT ;"); } else Serial.println("SET SN:FORMAT ;"); }
        else if (strcmp(cmd, "EXIT") == 0) { settingsEnabled = false; Serial.println("SET EXIT:OK ;"); }
        else if (strcmp(cmd, "WRI") == 0 && settingsEnabled) { if (strieq(arg, "YES")) { saveHWInfo(); settingsEnabled = false; Serial.println("SET WRI:OK ;"); triggerSoftwareReset(); } else Serial.println("SET WRI:FORMAT ;"); }
        else if (strcmp(cmd, "WRITE") == 0) { if (settingsEnabled) { saveHWInfo(); settingsEnabled = false; Serial.println("WRITE:OK ;"); } else Serial.println("WRITE:LOCKED ;"); }
      }
      if (c == 0) break;
    } else { if (tlen < 31) token[tlen++] = c; }
  }
}

void processSerialTokensFromHost() {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') continue;
      if (c == '\n') c = ';';
      int alen = strlen(serialAccum);
      if (alen < 63) {
        serialAccum[alen] = c;
        serialAccum[alen + 1] = 0;
      }
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
        char tokenUp[32];
        for (ti = 0; token[ti] && ti < 31; ti++) { tokenUp[ti] = (token[ti] >= 'a' && token[ti] <= 'z') ? token[ti] - 32 : token[ti]; }
        tokenUp[ti] = 0;
        if (strcmp(tokenUp, "VER") == 0 || strcmp(tokenUp, "VERSION") == 0) {
          sendIdentAndState();
          identDelayActive = true;
          identDelayStart = millis();
          continue;
        }
        if (strcmp(tokenUp, "REQ") == 0) {
          sendStatus();
          continue;
        }
        if (strcmp(tokenUp, "RESET") == 0) {
          Serial.println("RESET:OK ;");
          Serial.flush();
          delay(50);
          wdt_enable(WDTO_15MS);
          while (true) {}
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
// BOOT SEQUENCE
// ============================================================================
// Removed boot sequence

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    wdt_disable();
    Serial.begin(115200);
    loadHWInfo();
    delay(10);
    pinMode(maxDataPin, OUTPUT);
    pinMode(maxClkPin, OUTPUT);
    pinMode(maxCsPin, OUTPUT);
    pinMode(brkLatchPin, OUTPUT);
    pinMode(brkClkPin, OUTPUT);
    pinMode(brkDataPin, OUTPUT);
    pinMode(blLatchPin, OUTPUT);
    pinMode(blClkPin, OUTPUT);
    pinMode(blDataPin, OUTPUT);
    for (int i = 0; i < 8; i++) pinMode(ledGpioPins1[i], OUTPUT);
    for (int i = 0; i < 8; i++) pinMode(ledGpioPins2[i], OUTPUT);
    for (int i = 0; i < 8; i++) {
      int pin = ledGpioPins3[i];
      if (pin >= 0) pinMode(pin, OUTPUT);
    }
    pinMode(annuPWM, OUTPUT);
    pinMode(backlightPWM, OUTPUT);
    pinMode(displayPWM, OUTPUT);
    pinMode(inDataPin, INPUT);
    pinMode(inClkPin, OUTPUT);
    pinMode(inLatchPin, OUTPUT);
    for (int i = 0; i < NUM_DIRECT_PINS; i++) pinMode(directInPins[i], INPUT_PULLUP);
    for (unsigned i = 0; i < sizeof(analogPins) / sizeof(analogPins[0]); i++)
      pinMode(analogPins[i], INPUT);

    // Initiale Analogwerte erfassen
    for (int i = 0; i < 6; i++) {
      lastSentAnalog[i] = analogRead(analogPins[i]);
    }

    initDisplays();

    setLEDState(0x0000, 0x000000, 0, 0, 0);
    chronoRowCHR = "nnIP ";
    chronoRowUTC = " boot ";
    chronoRowET = "    ";
    renderChronoDisplays();
    
    // Boot timer: nach 2500ms normale Anzeige
    identDelayActive = true;
    identDelayStart = millis();
    
    sendIdentAndState();
  }

  // ============================================================================
  // LOOP
  // ============================================================================
  void loop() {
    unsigned long now = millis();

    // If DIAG sequence is running, drive it and skip normal processing
    if (diagRunning) {
      runDiagSequence();
      return;
    }

    // Always process incoming host commands
    processSerialTokensFromHost();

    // Check for IDENT delay (non-blocking pause for scan clarity)
    if (identDelayActive) {
      if (millis() - identDelayStart >= IDENT_DELAY_MS) {
        chronoRowCHR = "";
        chronoRowUTC = "";
        chronoRowET = "";
        renderChronoDisplays();
        identDelayActive = false;
      } else {
        // Skip main loop logic during delay, but serial processing continues above
        return;
      }
    }

    if (now - lastLoopTs < LOOP_INTERVAL_MS) return;
    lastLoopTs = now;
    // Inputs
    readShift(inShift1, inShift2);
    for (int g = 0; g < NUM_DIRECT_GROUPS; g++) {
      int offset = g * 8;
      int pinsInGroup = NUM_DIRECT_PINS - offset;
      if (pinsInGroup > 8) pinsInGroup = 8;
      inDirect[g] = readDirectGroup(directInPins + offset, pinsInGroup);
    }

    // --- Inputs: Button changes handled by SIM, FW only sends raw state ---
    // Buttons are read via shift registers and sent in sendStatus()
    // Date button (IN6 bit 4) disables UTC DP internally but is still reported

    // DIAG combo: RST (IN1 bit4), CHR (IN1 bit3), DATE (IN6 bit4)
    bool hasIN6 = (NUM_DIRECT_GROUPS >= 4);
    bool diagCombo = (inShift1 & DIAG_MASK_RST) && (inShift1 & DIAG_MASK_CHR) && hasIN6 && (inDirect[3] & DIAG_MASK_DATE);
    if (diagCombo && !diagComboLatched) {
      startDiagSequence();  // Start full LED walk + display test
      diagComboLatched = true;
    }
    if (!diagCombo) diagComboLatched = false;

    // Send on changes (compare all inputs against last state)
    // Send on changes (digital + analog)
    bool change = (inShift1 != lastInShift1) || (inShift2 != lastInShift2);

    for (int g = 0; g < NUM_DIRECT_GROUPS && !change; g++) {
      if (inDirect[g] != lastInDirect[g])
        change = true;
    }

    // AnalogÃƒÂ¤nderungen erkennen
    bool analogChange = false;

    if (analogReportEnabled) {
      for (int i = 0; i < 6; i++) {
        int v = analogRead(analogPins[i]);

        if (abs(v - lastSentAnalog[i]) >= analogDeadband) {
          analogChange = true;
          break;
        }
      }
    }

    if (change || analogChange) {
      lastInShift1 = inShift1;
      lastInShift2 = inShift2;

      for (int g = 0; g < NUM_DIRECT_GROUPS; g++)
        lastInDirect[g] = inDirect[g];

      // aktuelle Analogwerte als Referenz ÃƒÂ¼bernehmen
      for (int i = 0; i < 6; i++)
        lastSentAnalog[i] = analogRead(analogPins[i]);

      forceSendNext = true;  // CRITICAL: Force send buttons immediately
      sendStatus();
    }

    // Note: All display updates happen via serial commands (DSP1/DSP2/DSP3)
  }
