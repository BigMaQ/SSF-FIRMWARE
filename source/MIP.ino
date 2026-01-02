// MIP Control Script, (w) 2025 M. Quatember / BigMaQ

// Setze diese Zeile auf 1, um Bootsequenz und alle Startup-Ausgaben IMMER zu unterdrücken
#define SUPPRESS_BOOTSEQ 0

// Panel-Identifikation beim Start anzeigen (Sekunden)
#define IDENT_DISPLAY_MS 2000
// Protocol-compatible with RMP/ACP: VER, IDENT, RESET, REQ, LEDx, INx, BL/AN/DISP_BL
// Chrono: 2x MAX7219 in series, driving 3 logical rows (CHR/UTC/ET) via time-multiplex
// BRK Panel: 2 LED drivers in series; Backlight drivers: 3 in series

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
const int maxDataPin  = 62;   // CHR_DISP_DIN -> A8
const int maxClkPin   = 63;   // CHR_DISP_CLK -> A9
const int maxCsPin    = 64;   // CHR_DISP_LOAD (CS) -> A10
// Two devices in series
LedControl lc = LedControl(maxDataPin, maxClkPin, maxCsPin, 2);

// LED Drivers in series
// BRK (2x) latch/clk/data
const int brkLatchPin = 2;    // BRK_LEDDRV_LATCH
const int brkDataPin  = 4;    // BRK_LEDDRV_DATA
const int brkClkPin   = 3;    // BRK_LEDDRV_CLK

// Backlight drivers (3x) share same Latch/Clk/Data (using separate chain)
const int blLatchPin = 53;    // BLTDRV_LATCH
const int blClkPin   = 52;    // BLTDRV_CLK
const int blDataPin  = 51;    // BLTDRV_DATA

// Direct GPIO LED outputs (16 lines, two 8-bit banks, ordered by resistor pins)
const int ledGpioPins1[8] = {5,7,12,16,17,20,21,24};
const int ledGpioPins2[8] = {25,26,27,30,32,44,46,60}; // A6 as digital 60

// Annunciator & Backlight PWM (hardware PWM pins)
const int annuPWM = 11;       // ANNU_BRT
const int backlightPWM = 10;  // CON_BRT
const int displayPWM = 9;     // CON_DIM (optional PWM dim chain)

// Input shift registers (HC165) - direct inputs grouped as INx
// CHR_IS (Input Shifter) pins per schematic
const int inDataPin  = 67;    // CHR_IS_DATA -> A13
const int inClkPin   = 66;    // CHR_IS_CLOCK -> A12
const int inLatchPin = 65;    // CHR_IS_LATCH -> A11

// Direct GPIO inputs on Arduino Mega (button inputs only, ordered low->high)
// Use every free GPIO that is not tied to shift/MAX/LED driver clocks/data/latch or LED outputs
const int directInPins[] = {
  6, 8, 13, 14, 15, 18, 19, 22,
  23, 28, 29, 31, 33, 34, 35, 36,
  37, 38, 39, 40, 41, 42, 43, 45,
  47, 48, 49, 50, 68, 69
};
const int NUM_DIRECT_PINS = sizeof(directInPins) / sizeof(directInPins[0]);
const int NUM_DIRECT_GROUPS = (NUM_DIRECT_PINS + 7) / 8; // pack into IN3+ groups

// Analog inputs (Ax): forward raw via serial (do NOT include pins used for MAX/shift regs)
const int analogPins[] = {A0,A1,A2,A3,A4,A5,A6};

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "MIP, v1.0 MAQ";
bool identSentOnStart = false;

// ============================================================================
// STATE: LED drivers and brightness
// ============================================================================
// BRK drivers: 2x 8-bit -> 16 bits
uint16_t desiredBrkLed = 0x0000;
uint16_t hwBrkLed = 0x0000;

// Backlight drivers: 3x 8-bit -> 24 bits (pack into uint32_t lower 24 bits)
uint32_t desiredBlLed = 0x000000;
uint32_t hwBlLed = 0x000000;

uint8_t desiredGpioLed1 = 0x00; // direct GPIO bank 1
uint8_t desiredGpioLed2 = 0x00; // direct GPIO bank 2
uint8_t hwGpioLed1 = 0x00;
uint8_t hwGpioLed2 = 0x00;

// PWM brightness
uint8_t desiredAnLevel = 0; // Annunciator bright
uint8_t desiredBlLevel = 0; // Backlight bright
uint8_t desiredDispLevel = 0; // Display PWM bright (optional)

uint8_t hwAnLevel = 0, hwBlLevel = 0, hwDispLevel = 0;

// Display brightness for MAX7219 (0-15)
uint8_t DISP_BL_LEVEL = 15;

// ============================================================================
// Chrono displays: Device 0 = CHR(4)+ET(4), Device 1 = UTC(6)
// ============================================================================
String chronoRowCHR = "CHR ";
String chronoRowUTC = "UTC   ";
String chronoRowET  = "ET  ";

// ============================================================================
// INPUT STATE
// ============================================================================
uint8_t inShift1 = 0x00;
uint8_t inShift2 = 0x00;
uint8_t lastInShift1 = 0x00;
uint8_t lastInShift2 = 0x00;

// Direct GPIO groups packed into bytes (IN3, IN4, IN5 for 17 pins)
uint8_t inDirect[NUM_DIRECT_GROUPS] = {0};
uint8_t lastInDirect[NUM_DIRECT_GROUPS] = {0};

// DIAG combo masks (pressed = 1)
const uint8_t DIAG_MASK_RST = 0b00010000; // IN1 pattern xxx1xxxx
const uint8_t DIAG_MASK_CHR = 0b00001000; // IN1 pattern xxxx1xxx
const uint8_t DIAG_MASK_DATE = 0b00010000; // IN6 pattern xxx1xxxx
bool diagComboLatched = false;

// DIAG sequence runtime
bool diagRunning = false;
unsigned long diagStartTs = 0;

// ============================================================================
// RUNTIME
// ============================================================================
String serialAccum = "";
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;
unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
bool forceSendNext = false;
bool analogReportEnabled = false;  // Gate analog reporting (enable via ANALOG_EN:1)

// IDENT delay for scan clarity (non-blocking)
bool identDelayActive = false;
unsigned long identDelayStart = 0;
const unsigned long IDENT_DELAY_MS = 200;

// Clock mode
bool clockEnabled = false;
unsigned long lastClockUpdate = 0;
const unsigned long CLOCK_UPDATE_MS = 1000;
unsigned long clockSeconds = 0;
bool ackEnabled = false; // gate serial ACK echo for commands
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
String bin8(byte b){String s=""; for(int i=7;i>=0;i--) s+=((b&(1<<i))? '1':'0'); return s;}
uint8_t parseBin8(const String &s){uint8_t v=0; for(int i=0;i<s.length();i++){char c=s.charAt(i); if(c=='0'||c=='1') v=(v<<1)|(c=='1');} return v;}

void shiftOut16(uint16_t bits, int latch, int clk, int data){
  digitalWrite(latch, LOW);
  shiftOut(data, clk, MSBFIRST, (bits>>8)&0xFF);
  shiftOut(data, clk, MSBFIRST, bits&0xFF);
  digitalWrite(latch, HIGH);
}

void shiftOut24(uint32_t bits24, int latch, int clk, int data){
  digitalWrite(latch, LOW);
  shiftOut(data, clk, MSBFIRST, (bits24>>16)&0xFF);
  shiftOut(data, clk, MSBFIRST, (bits24>>8)&0xFF);
  shiftOut(data, clk, MSBFIRST, bits24&0xFF);
  digitalWrite(latch, HIGH);
}

void applyOutputs(){
  // LED drivers
  shiftOut16(desiredBrkLed, brkLatchPin, brkClkPin, brkDataPin);
  shiftOut24(desiredBlLed, blLatchPin, blClkPin, blDataPin);
  // Direct GPIO LED banks
  for(int i=0;i<8;i++) digitalWrite(ledGpioPins1[i], (desiredGpioLed1 & (1<<i)) ? HIGH : LOW);
  for(int i=0;i<8;i++) digitalWrite(ledGpioPins2[i], (desiredGpioLed2 & (1<<i)) ? HIGH : LOW);
  // PWM brightness
  analogWrite(annuPWM, desiredAnLevel);
  analogWrite(backlightPWM, desiredBlLevel);
  analogWrite(displayPWM, desiredDispLevel);
  // cache
  hwBrkLed = desiredBrkLed; hwBlLed = desiredBlLed;
  hwGpioLed1 = desiredGpioLed1; hwGpioLed2 = desiredGpioLed2;
  hwAnLevel = desiredAnLevel; hwBlLevel = desiredBlLevel; hwDispLevel = desiredDispLevel;
}

void setLEDState(uint16_t brkBits, uint32_t blBits, uint8_t an, uint8_t bl, uint8_t dispPWM){
  desiredBrkLed = brkBits; desiredBlLed = blBits;
  desiredAnLevel = an; desiredBlLevel = bl; desiredDispLevel = dispPWM;
  applyOutputs();
}

// ============================================================================
// DISPLAY
// ============================================================================
void initDisplays(){
  // Device 0: 6 digits for UTC
  lc.shutdown(0, false);              // Wake up
  delay(10);                          // Wait for chip to stabilize
  
  // CRITICAL: Configure scan limit FIRST - tells MAX7219 how many digits to use
  lc.setScanLimit(0, 5);              // Use 6 digits (0-5)
  
  // Then set intensity and clear
  lc.setIntensity(0, DISP_BL_LEVEL);  // Set brightness (0-15)
  lc.clearDisplay(0);                 // Clear all data
  
  // Explicitly set all 6 rows to 0
  for(int d=0; d<6; d++) lc.setRow(0, d, 0x00);
  
  // Device 1: 8 digits for CHR+ET
  lc.shutdown(1, false);              // Wake up
  delay(10);                          // Wait for chip to stabilize
  
  // CRITICAL: Configure scan limit FIRST
  lc.setScanLimit(1, 7);              // Use 8 digits (0-7)
  
  // Then set intensity and clear
  lc.setIntensity(1, DISP_BL_LEVEL);  // Set brightness (0-15)
  lc.clearDisplay(1);                 // Clear all data
  
  // Explicitly set all 8 rows to 0
  for(int d=0; d<8; d++) lc.setRow(1, d, 0x00);
}

byte charTo7Seg(char c){
  c = toupper(c);
  switch(c){
    case '0': return 0b01111110; case '1': return 0b00110000; case '2': return 0b01101101;
    case '3': return 0b01111001; case '4': return 0b00110011; case '5': return 0b01011011;
    case '6': return 0b01011111; case '7': return 0b01110000; case '8': return 0b01111111;
    case '9': return 0b01111011; case 'A': return 0b01110111; case 'B': return 0b00011111;
    case 'C': return 0b01001110; case 'D': return 0b00111101; case 'E': return 0b01001111;
    case 'F': return 0b01000111; case 'H': return 0b00110111; case 'U': return 0b00111110;
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
    case 'T': return 0b00001111; case 'R': return 0b00000101; case '-': return 0b00000001; case '$': return 0b00000100;
    case ' ': return 0b00000000; default: return 0b00000000;
  }
}

void buildDigitBuffer(const String &src, byte *out, int maxDigits, bool colonMode=false, int colonIdx=2){
  // Build digit patterns with optional colon-style DP placement
  int digit=0; bool sawDot=false;
  for(int i=0; i<src.length() && digit<maxDigits; i++){
    char c = src.charAt(i);
    if(c=='.'){
      sawDot = true; // track dot for colon placement
      if(!colonMode && digit>0) out[digit-1] |= 0b10000000; // normal DP on previous digit
      continue;
    }
    out[digit] = charTo7Seg(c);
    digit++;
  }
  for(; digit<maxDigits; digit++) out[digit]=0x00;
  if(colonMode && sawDot && colonIdx>=0 && colonIdx<maxDigits){
    out[colonIdx] |= 0b10000000; // place colon/DP at fixed digit (e.g., HH:MM)
  }
}

String trimToDigits(const String &src, int maxDigits){
  // Keep characters until maxDigits (dots don't count toward digit budget)
  String out=""; int digits=0;
  for(int i=0; i<src.length(); i++){
    char c=src.charAt(i);
    if(c=='.'){
      if(digits>0) out+=c; // allow DP for previous digit
      continue;
    }
    if(digits>=maxDigits) break;
    out+=c; digits++;
  }
  return out;
}

// ============================================================================
// DIAG HELPERS
// ============================================================================
void diagDisplayNumberWalk(unsigned long elapsed){
  // elapsed in ms; advance every 300ms
  int step = (elapsed/300) % 10;
  // Build rolling digits for UTC (6), CHR (4), ET (4)
  String utc=""; for(int i=0;i<6;i++) utc += char('0' + ((step + i)%10));
  String chr=""; for(int i=0;i<4;i++) chr += char('0' + ((step + i + 6)%10));
  String et =""; for(int i=0;i<4;i++) et  += char('0' + ((step + i + 2)%10));
  chronoRowUTC = utc; chronoRowCHR = chr; chronoRowET = et;
  renderChronoDisplays();
}

void diagSegmentTest(bool on){
  byte val = on ? 0xFF : 0x00;
  for(int d=0; d<6; d++) lc.setRow(0, d, val);
  for(int d=0; d<8; d++) lc.setRow(1, d, val);
}

void diagSegmentSweep(unsigned long elapsed){
  // Sweep through each segment bit individually
  // 7 segments per digit + DP = 8 bits, cycle every 100ms
  int segmentBit = (elapsed / 100) % 8;
  byte pattern = (1 << segmentBit);
  
  // Apply to all digits
  for(int d=0; d<6; d++) lc.setRow(0, d, pattern);
  for(int d=0; d<8; d++) lc.setRow(1, d, pattern);
}

const int DIAG_LED_TOTAL = 16 + 24 + 8 + 8; // BRK + Backlight + GPIO1 + GPIO2

void diagLedWalkStep(unsigned long elapsed){
  int step = (elapsed/120) % DIAG_LED_TOTAL;
  desiredBrkLed = 0;
  desiredBlLed = 0;
  desiredGpioLed1 = 0;
  desiredGpioLed2 = 0;

  if(step < 8){
    desiredGpioLed1 = (1 << (7 - step));
  } else if(step < 16){
    desiredGpioLed2 = (1 << (15 - step));
  } else if(step < 32){
    desiredBrkLed = (1 << (31 - step)); // walk MSB->LSB for clarity
  } else {
    int idx = step - 32; // 0..23
    desiredBlLed = (uint32_t(1) << (23 - idx));
  }
  applyOutputs();

  // Update displays to show current LED byte and value
  // Top (CHR): LEDx where x=1..7 (active byte)
  // Middle (UTC): two-digit HEX value of the active byte
  // Lower (ET): "tE5t" (7-seg friendly for tESt)
  int activeLedByte = 0; // 1..7
  uint8_t activeValue = 0x00;

  if(step < 8){
    // GPIO bank 1 -> LED1
    activeLedByte = 1; activeValue = desiredGpioLed1;
  } else if(step < 16){
    // GPIO bank 2 -> LED2
    activeLedByte = 2; activeValue = desiredGpioLed2;
  } else if(step < 32){
    // BRK 16-bit -> LED3 (high), LED4 (low)
    uint8_t hi = (desiredBrkLed >> 8) & 0xFF;
    uint8_t lo = desiredBrkLed & 0xFF;
    if(hi) { activeLedByte = 3; activeValue = hi; }
    else   { activeLedByte = 4; activeValue = lo; }
  } else {
    // Backlight 24-bit -> LED5 (MSB), LED6, LED7 (LSB)
    uint8_t b2 = (desiredBlLed >> 16) & 0xFF; // LED5
    uint8_t b1 = (desiredBlLed >> 8)  & 0xFF; // LED6
    uint8_t b0 = (desiredBlLed)       & 0xFF; // LED7
    if(b2){ activeLedByte = 5; activeValue = b2; }
    else if(b1){ activeLedByte = 6; activeValue = b1; }
    else { activeLedByte = 7; activeValue = b0; }
  }

  char topStr[5]; // "LEDx"
  snprintf(topStr, sizeof(topStr), "LED%d", activeLedByte);
  // Center HEX value in 6-digit UTC: "  XX  "
  char midStr6[7];
  char hex2[3];
  snprintf(hex2, sizeof(hex2), "%02X", activeValue);
  snprintf(midStr6, sizeof(midStr6), "  %s  ", hex2);

  chronoRowCHR = String(topStr);
  chronoRowUTC = String(midStr6);
  chronoRowET  = "tE5t"; // 7-seg friendly for tESt (S ~ digit 5)
  renderChronoDisplays();
}

void startDiagSequence(){
  diagRunning = true;
  diagStartTs = millis();
  Serial.println("DIAG:START");
}

// Manuelle Bootsequenz (wie bisher, aber nur auf Kommando)
void runManualBootSequence();

void stopDiagSequence(){
  diagRunning = false;
  // Clear outputs and restore baseline labels
  setLEDState(0x0000, 0x000000, 0, 0, 0);
  chronoRowCHR = "CHR ";
  chronoRowUTC = "UTC   ";
  chronoRowET  = "ET  ";
  renderChronoDisplays();
  Serial.println("DIAG:DONE");
  forceSendNext = true;
}

void runDiagSequence(){
  if(!diagRunning) return;
  unsigned long elapsed = millis() - diagStartTs;

  if(elapsed < 2000){
    chronoRowCHR = "DIAG"; chronoRowUTC = "START "; chronoRowET = "DIAG"; renderChronoDisplays();
  } else if(elapsed < 8000){
    diagDisplayNumberWalk(elapsed - 2000);
  } else if(elapsed < 10000){
    // Segment test: all segments ON for 2 seconds
    diagSegmentTest(true);
  } else if(elapsed < 12000){
    // Segment sweep: walk through each segment for 2 seconds (8 segments × 250ms)
    diagSegmentSweep(elapsed - 10000);
  } else if(elapsed < 23000){
    diagLedWalkStep(elapsed - 12000);
  } else {
    diagSegmentTest(false);
    stopDiagSequence();
  }
}

int indexAfterDigits(const String &src, int digitBudget){
  // Return index in src after consuming digitBudget digits (dots don't count)
  int digits=0;
  for(int i=0;i<src.length(); i++){
    char c=src.charAt(i);
    if(c!='.') digits++;
    if(digits>=digitBudget) return i+1;
  }
  return src.length();
}

void updateDisplay(int device, const String &text, int maxDigits) {
  // Update display with text, RMP-style
  // MAX7219: digit 0 is rightmost, higher digits are leftmost
  int textPos = 0;
  for (int digitPos = 0; digitPos < maxDigits; digitPos++) {
    if (textPos >= text.length()) {
      lc.setRow(device, (maxDigits-1) - digitPos, 0b00000000);  // Blank remaining
      continue;
    }
    
    char c = text.charAt(textPos);
    byte pattern = charTo7Seg(c);
    
    lc.setRow(device, (maxDigits-1) - digitPos, pattern);
    textPos++;
  }
}

void renderChronoDisplays(){
  // Device 0: UTC (6 digits)
  // Device 1: CHR (4 digits) on the right half (digits 0-3)
  //           ET (4 digits) on the left half (digits 4-7)
  // Manuelle Bootsequenz ist im globalen Scope implementiert

  // Render UTC on device 0, digits 0-5 as full HH.MM.SS (DPs from '.' in text) 
  byte utcBuf[6];
  buildDigitBuffer(chronoRowUTC, utcBuf, 6, false, 0);
  for(int i=0; i<6; i++) lc.setRow(0, 5-i, utcBuf[i]); // digit 5=leftmost, 0=rightmost
  
  // Render CHR on device 1, digits 0-3
  byte chrBuf[4]; buildDigitBuffer(chronoRowCHR, chrBuf, 4, true, 2);
  for(int i=0; i<4; i++) lc.setRow(1, 3-i, chrBuf[i]); // digit 3=leftmost of CHR, 0=rightmost
  
  // Render ET on device 1, digits 4-7
  byte etBuf[4]; buildDigitBuffer(chronoRowET, etBuf, 4, true, 2);
  for(int i=0; i<4; i++) lc.setRow(1, 7-i, etBuf[i]); // digit 7=leftmost of ET, 4=rightmost
}

// --- Ende Patch ---

// ============================================================================
// INPUTS
// ============================================================================
void readShift(uint8_t &in1, uint8_t &in2){
  digitalWrite(inLatchPin, LOW); delayMicroseconds(20);
  digitalWrite(inLatchPin, HIGH); delayMicroseconds(20);
  byte raw1 = shiftIn(inDataPin, inClkPin, MSBFIRST);
  byte raw2 = shiftIn(inDataPin, inClkPin, MSBFIRST);
  in1 = ~raw1; in2 = ~raw2;
  digitalWrite(inLatchPin, LOW); delayMicroseconds(20);
  digitalWrite(inLatchPin, HIGH); delayMicroseconds(20);
  for(int i=0;i<16;i++){ digitalWrite(inClkPin, LOW); delayMicroseconds(5); digitalWrite(inClkPin, HIGH); delayMicroseconds(5);} 
}

uint8_t readDirectGroup(const int *pins, int count){
  uint8_t v=0x00; // active-high reporting (pressed=1), unused bits stay 0
  for(int i=0;i<count && i<8;i++){
    int r=digitalRead(pins[i]);
    if(r==LOW) v|=(1<<i); // invert because buttons are pull-up (LOW when pressed)
  }
  return v;
}

// ============================================================================
// CLOCK
// ============================================================================
void updateClock() {
  if (!clockEnabled) return;
  unsigned long now = millis();
  if (now - lastClockUpdate >= CLOCK_UPDATE_MS) {
    lastClockUpdate = now;
    clockSeconds++;
    
    unsigned long hours = (clockSeconds / 3600) % 24;
    unsigned long mins = (clockSeconds / 60) % 60;
    unsigned long secs = clockSeconds % 60;
    // Render UTC as HH.MM.SS using DP as colon
    char utcStr[9];
    sprintf(utcStr, "%02lu.%02lu.%02lu", hours, mins, secs);
    chronoRowUTC = String(utcStr);
    // Update chrono in seconds mode (MM.SS) if running
    if (chronoRunning) {
      unsigned long cmins = (chronoSeconds / 60) % 100; // roll over at 99:59
      unsigned long csecs = chronoSeconds % 60;
      char chrStr[6];
      sprintf(chrStr, "%02lu.%02lu", cmins, csecs);
      chronoRowCHR = String(chrStr);
      chronoSeconds++;
    }
    renderChronoDisplays();
  }
}

// ============================================================================
// DIAG / SETUP HELPERS (serial-driven)
// ============================================================================
void sendDiagReport(){
  Serial.print("DIAG:STATE;IN1:"); Serial.print(bin8(inShift1));
  Serial.print(";IN2:"); Serial.print(bin8(inShift2));
  for(int g=0; g<NUM_DIRECT_GROUPS; g++){
    Serial.print(";IN"); Serial.print(3+g); Serial.print(":"); Serial.print(bin8(inDirect[g]));
  }
  Serial.print(";LED_BRK:"); Serial.print(hwBrkLed, HEX);
  Serial.print(";LED_BL:"); Serial.print(hwBlLed, HEX);
  Serial.print(";LED_GPIO1:"); Serial.print(hwGpioLed1, HEX);
  Serial.print(";LED_GPIO2:"); Serial.print(hwGpioLed2, HEX);
  Serial.print(";BRT_AN:"); Serial.print(hwAnLevel);
  Serial.print(";BRT_BL:"); Serial.print(hwBlLevel);
  Serial.print(";BRT_DISP:"); Serial.print(hwDispLevel);
  Serial.print(";INTENSITY:"); Serial.print(DISP_BL_LEVEL);
  Serial.print(";CLOCK:"); Serial.print(clockEnabled?"ON":"OFF");
  Serial.print(";CLK_SEC:"); Serial.print(clockSeconds);
  Serial.print(";CHR:"); Serial.print(chronoRowCHR);
  Serial.print(";UTC:"); Serial.print(chronoRowUTC);
  Serial.print(";ET:"); Serial.print(chronoRowET);
  Serial.println(";");
}

void sendSetupMenu(){
  Serial.println("SETUP:MENU;ITEMS:BRIGHT,ANALOG_EN,CLOCK;CMDS:SETUP:BRIGHT:AN=0-255,BL=0-255,DISP=0-15|ANALOG_EN:0/1|CLOCK:0/1;");
}

// ============================================================================
// SERIAL
// ============================================================================
void sendIdentAndState(){
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
  Serial.print(";STATE:RUNNING;");
  Serial.println();
}

void sendStatus(){
  unsigned long now=millis(); if(!forceSendNext && (now-lastSendTs)<SEND_MIN_INTERVAL_MS) return;
  Serial.print("IN1:"); Serial.print(bin8(inShift1)); Serial.print(";");
  Serial.print("IN2:"); Serial.print(bin8(inShift2)); Serial.print(";");
  for(int g=0; g<NUM_DIRECT_GROUPS; g++){
    Serial.print("IN"); Serial.print(3+g); Serial.print(":");
    Serial.print(bin8(inDirect[g])); Serial.print(";");
  }
  // Analog pass-through Ax (only if enabled)
  if(analogReportEnabled){
    for(int i=0;i<sizeof(analogPins)/sizeof(analogPins[0]); i++){ int v=analogRead(analogPins[i]); Serial.print("A"); Serial.print(i); Serial.print(":"); Serial.print(v); Serial.print(";"); }
  }
  Serial.println(); lastSendTs=now; forceSendNext=false;
}

void processIncomingLine(const String &line){
  int start=0; while(true){ int sep=line.indexOf(';', start); if(sep<0) break; String token=line.substring(start,sep); start=sep+1; token.trim(); if(token.length()==0) continue;
    if(token.indexOf(':')<0){
      if(token.equalsIgnoreCase("VER")||token.equalsIgnoreCase("VERSION")){ sendIdentAndState(); identDelayActive = true; identDelayStart = millis(); continue;}
      if(token.equalsIgnoreCase("REQ")){ forceSendNext=true; continue;}
      if(token.equalsIgnoreCase("RESET")){ Serial.println("RESET:OK ;"); Serial.flush(); delay(50); wdt_enable(WDTO_15MS); while(true){} }
      continue;
    }
    int colon=token.indexOf(':'); if(colon<0) continue; String key=token.substring(0,colon); String val=token.substring(colon+1); key.trim(); val.trim();
    if(key.equalsIgnoreCase("LED1")){ // Direct GPIO bank 1 (8 outputs)
      desiredGpioLed1 = parseBin8(val); applyOutputs();
      if(ackEnabled){ Serial.print("ACK:LED1:"); Serial.println(bin8(desiredGpioLed1)); }
    } else if(key.equalsIgnoreCase("LED2")){ // Direct GPIO bank 2 (8 outputs)
      desiredGpioLed2 = parseBin8(val); applyOutputs();
      if(ackEnabled){ Serial.print("ACK:LED2:"); Serial.println(bin8(desiredGpioLed2)); }
    } else if(key.equalsIgnoreCase("LED3")){ // BRK driver high byte
      uint8_t v=parseBin8(val); desiredBrkLed = (desiredBrkLed & 0x00FF) | (uint16_t(v)<<8); applyOutputs();
      if(ackEnabled){ Serial.print("ACK:LED3:"); Serial.println(bin8(v)); }
    } else if(key.equalsIgnoreCase("LED4")){ // BRK driver low byte
      uint8_t v=parseBin8(val); desiredBrkLed = (desiredBrkLed & 0xFF00) | uint16_t(v); applyOutputs();
      if(ackEnabled){ Serial.print("ACK:LED4:"); Serial.println(bin8(v)); }
    } else if(key.equalsIgnoreCase("LED5")){ // Backlight chain byte 2 (MSB)
      uint8_t v=parseBin8(val); desiredBlLed = (desiredBlLed & 0x0000FFFF) | (uint32_t(v)<<16); applyOutputs();
      if(ackEnabled){ Serial.print("ACK:LED5:"); Serial.println(bin8(v)); }
    } else if(key.equalsIgnoreCase("LED6")){ // Backlight chain byte 1
      uint8_t v=parseBin8(val); desiredBlLed = (desiredBlLed & 0x00FF00FF) | (uint32_t(v)<<8); applyOutputs();
      if(ackEnabled){ Serial.print("ACK:LED6:"); Serial.println(bin8(v)); }
    } else if(key.equalsIgnoreCase("LED7")){ // Backlight chain byte 0 (LSB)
      uint8_t v=parseBin8(val); desiredBlLed = (desiredBlLed & 0x00FFFF00) | uint32_t(v); applyOutputs();
      if(ackEnabled){ Serial.print("ACK:LED7:"); Serial.println(bin8(v)); }
    } else if(key.equalsIgnoreCase("BL")){ desiredBlLevel = constrain(val.toInt(),0,255); applyOutputs();
    } else if(key.equalsIgnoreCase("AN")){ desiredAnLevel = constrain(val.toInt(),0,255); applyOutputs();
    } else if(key.equalsIgnoreCase("DISP_BL")){ DISP_BL_LEVEL = constrain(val.toInt(),0,15); for(int dev=0;dev<2;dev++) lc.setIntensity(dev, DISP_BL_LEVEL);
    } else if(key.equalsIgnoreCase("CHR")){ chronoRowCHR = val; renderChronoDisplays();
    } else if(key.equalsIgnoreCase("UTC")){ chronoRowUTC = val; renderChronoDisplays();
    } else if(key.equalsIgnoreCase("ET")){ chronoRowET = val; renderChronoDisplays();
    } else if(key.equalsIgnoreCase("DSP1")){
      // RMP-compatible: DSP1 drives CHR (4 digits)
      chronoRowCHR = trimToDigits(val, 4); renderChronoDisplays();
      if(ackEnabled){ Serial.print("ACK:DSP1:"); Serial.println(chronoRowCHR); }
    } else if(key.equalsIgnoreCase("DSP2")){
      // RMP-compatible: DSP2 drives UTC (6 digits)
      chronoRowUTC = trimToDigits(val, 6); renderChronoDisplays();
      if(ackEnabled){ Serial.print("ACK:DSP2:"); Serial.println(chronoRowUTC); }
    } else if(key.equalsIgnoreCase("DSP3")){
      // RMP-compatible: DSP3 drives ET (4 digits)
      chronoRowET = trimToDigits(val, 4); renderChronoDisplays();
      if(ackEnabled){ Serial.print("ACK:DSP3:"); Serial.println(chronoRowET); }
    } else if(key.equalsIgnoreCase("DSP")){
      // DSP:CHR:text or DSP:UTC:text or DSP:ET:text
      int colon2 = val.indexOf(':');
      if(colon2 > 0){
        String target = val.substring(0, colon2);
        String text = val.substring(colon2+1);
        if(target.equalsIgnoreCase("CHR")) { chronoRowCHR = trimToDigits(text,4); renderChronoDisplays(); if(ackEnabled){ Serial.print("ACK:DSP:CHR:"); Serial.println(chronoRowCHR); } }
        else if(target.equalsIgnoreCase("UTC")) { chronoRowUTC = trimToDigits(text,6); renderChronoDisplays(); if(ackEnabled){ Serial.print("ACK:DSP:UTC:"); Serial.println(chronoRowUTC); } }
        else if(target.equalsIgnoreCase("ET")) { chronoRowET = trimToDigits(text,4); renderChronoDisplays(); if(ackEnabled){ Serial.print("ACK:DSP:ET:"); Serial.println(chronoRowET); } }
      }
    } else if(key.equalsIgnoreCase("DEBUG_ACK")){
      ackEnabled = (val=="1" || val.equalsIgnoreCase("ON"));
      Serial.print("ACK:DEBUG_ACK:"); Serial.println(ackEnabled?"ON":"OFF");
    } else if(key.equalsIgnoreCase("DIAG")){
        if(val.equalsIgnoreCase("RUN")){
          startDiagSequence();
        } else if(val.equalsIgnoreCase("STOP")){
          stopDiagSequence();
        } else {
          sendDiagReport(); // Default: send report (DIAG or DIAG:REPORT)
        }
    } else if(key.equalsIgnoreCase("SETUP")){
      // SETUP or SETUP:BRIGHT:AN=..,BL=..,DISP=..
      if(val.length()==0){
        sendSetupMenu();
      } else {
        String upVal = val; upVal.toUpperCase();
        if(upVal.startsWith("BRIGHT")){
          // Parse simple tokens AN=,BL=,DISP=
          int anIdx = upVal.indexOf("AN=");
          int blIdx = upVal.indexOf("BL=");
          int dpIdx = upVal.indexOf("DISP=");
          if(anIdx>=0){ int v = val.substring(anIdx+3).toInt(); desiredAnLevel = constrain(v,0,255); }
          if(blIdx>=0){ int v = val.substring(blIdx+3).toInt(); desiredBlLevel = constrain(v,0,255); }
          if(dpIdx>=0){ int v = val.substring(dpIdx+5).toInt(); DISP_BL_LEVEL = constrain(v,0,15); }
          applyOutputs(); for(int dev=0;dev<2;dev++) lc.setIntensity(dev, DISP_BL_LEVEL);
          Serial.print("SETUP:BRIGHT:AN="); Serial.print(desiredAnLevel);
          Serial.print(";BL="); Serial.print(desiredBlLevel);
          Serial.print(";DISP="); Serial.println(DISP_BL_LEVEL);
        } else {
          sendSetupMenu();
        }
      }
    } else if(key.equalsIgnoreCase("CLOCK")){
      clockEnabled = (val=="1" || val.equalsIgnoreCase("ON"));
      if(clockEnabled){
        clockSeconds = 0; // Default to 00:00:00 unless set via CLOCK:SET
        lastClockUpdate = millis();
        // Initialize CHR and ET
        chronoRowCHR = "00.00"; // Chrono MM.SS when running
        chronoRowET  = "00.00"; // Elapsed/ET placeholder
        Serial.println("CLOCK:ON");
        renderChronoDisplays();
      } else {
        Serial.println("CLOCK:OFF");
      }
    } else if(key.equalsIgnoreCase("CLOCK_SET") || key.equalsIgnoreCase("CLOCK:SET")){
      // Accept HH:MM:SS or HH.MM.SS
      String t = val;
      t.replace(':', '.');
      int p1 = t.indexOf('.');
      int p2 = (p1>=0) ? t.indexOf('.', p1+1) : -1;
      if(p1>0 && p2>p1){
        int hh = t.substring(0, p1).toInt();
        int mm = t.substring(p1+1, p2).toInt();
        int ss = t.substring(p2+1).toInt();
        hh = constrain(hh, 0, 23);
        mm = constrain(mm, 0, 59);
        ss = constrain(ss, 0, 59);
        clockSeconds = (unsigned long)hh*3600UL + (unsigned long)mm*60UL + (unsigned long)ss;
        // Immediately render updated time
        char utcStr[9];
        sprintf(utcStr, "%02d.%02d.%02d", hh, mm, ss);
        chronoRowUTC = String(utcStr);
        renderChronoDisplays();
        Serial.println("CLOCK:SET:OK");
      } else {
        Serial.println("CLOCK:SET:ERR");
      }
    } else if(key.equalsIgnoreCase("CHR_START") || key.equalsIgnoreCase("CHR:START")){
      chronoRunning = true;
      Serial.println("CHR:START");
    } else if(key.equalsIgnoreCase("CHR_STOP") || key.equalsIgnoreCase("CHR:STOP")){
      chronoRunning = false;
      Serial.println("CHR:STOP");
    } else if(key.equalsIgnoreCase("CHR_RESET") || key.equalsIgnoreCase("CHR:RESET")){
      chronoRunning = false;
      chronoSeconds = 0;
      chronoRowCHR = "00.00";
      renderChronoDisplays();
      Serial.println("CHR:RESET");
    } else if(key.equalsIgnoreCase("ANALOG_EN")){ analogReportEnabled = (val=="1" || val.equalsIgnoreCase("ON")); Serial.print("ANALOG_EN:"); Serial.println(analogReportEnabled?"ON":"OFF");
    } else if(key.equalsIgnoreCase("DEBUG_SR")){
      // Debug: read raw shift register values before inversion
      digitalWrite(inLatchPin, LOW); delayMicroseconds(20);
      digitalWrite(inLatchPin, HIGH); delayMicroseconds(20);
      byte raw1 = shiftIn(inDataPin, inClkPin, MSBFIRST);
      byte raw2 = shiftIn(inDataPin, inClkPin, MSBFIRST);
      Serial.print("DEBUG_SR:RAW1="); Serial.print(bin8(raw1));
      Serial.print(";RAW2="); Serial.print(bin8(raw2));
      Serial.print(";INV1="); Serial.print(bin8(~raw1));
      Serial.print(";INV2="); Serial.println(bin8(~raw2));
    } else if(key.equalsIgnoreCase("DEBUG_PIN")){
      // Debug: read specific pin or all direct pins
      if(val.length()>0){
        int pin=val.toInt();
        pinMode(pin, INPUT_PULLUP);
        int state=digitalRead(pin);
        Serial.print("DEBUG_PIN:"); Serial.print(pin); Serial.print("="); Serial.println(state);
      } else {
        Serial.print("DEBUG_PIN:ALL=");
        for(int i=0;i<NUM_DIRECT_PINS;i++){
          Serial.print("P"); Serial.print(directInPins[i]); Serial.print(":"); Serial.print(digitalRead(directInPins[i])); Serial.print(";");
        }
        Serial.println();
      }
    } else if(key.equalsIgnoreCase("DEBUG_MAX")){
      // Debug: test MAX7219 by setting all segments
      Serial.println("DEBUG_MAX: Setting all segments to ON");
      for(int d=0; d<6; d++) lc.setRow(0, d, 0xFF);  // Device 0: UTC (6 digits)
      for(int d=0; d<8; d++) lc.setRow(1, d, 0xFF);  // Device 1: CHR+ET (8 digits)
      Serial.println("DEBUG_MAX: All segments ON");
    } else if(key.equalsIgnoreCase("REQ")){ forceSendNext=true; }
  }
}

void processSerialTokensFromHost(){
  while(Serial.available()){
    char c=Serial.read(); if(c=='\r' || c=='\n') c=';'; serialAccum+=c;
    int idx; while((idx=serialAccum.indexOf(';'))>=0){ String token=serialAccum.substring(0,idx); token.trim(); serialAccum=serialAccum.substring(idx+1); if(token.length()==0) continue;
      String up=token; up.toUpperCase();
      if(up=="VER"||up=="VERSION"){ sendIdentAndState(); continue; }
      if(up=="REQ"){ sendStatus(); continue; }
      if(up=="RESET"){ Serial.println("RESET:OK ;"); Serial.flush(); delay(50); wdt_enable(WDTO_15MS); while(true){} }
      if(token.indexOf(':')>=0){ processIncomingLine(token+";"); continue; }
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
void setup(){
  wdt_disable();
  Serial.begin(115200);
  delay(10);
  pinMode(maxDataPin, OUTPUT);
  pinMode(maxClkPin, OUTPUT);
  pinMode(maxCsPin, OUTPUT);
  pinMode(brkLatchPin, OUTPUT); pinMode(brkClkPin, OUTPUT); pinMode(brkDataPin, OUTPUT);
  pinMode(blLatchPin, OUTPUT); pinMode(blClkPin, OUTPUT); pinMode(blDataPin, OUTPUT);
  for(int i=0;i<8;i++) pinMode(ledGpioPins1[i], OUTPUT);
  for(int i=0;i<8;i++) pinMode(ledGpioPins2[i], OUTPUT);
  pinMode(annuPWM, OUTPUT); pinMode(backlightPWM, OUTPUT); pinMode(displayPWM, OUTPUT);
  pinMode(inDataPin, INPUT); pinMode(inClkPin, OUTPUT); pinMode(inLatchPin, OUTPUT);
  for(int i=0;i<NUM_DIRECT_PINS; i++) pinMode(directInPins[i], INPUT_PULLUP);
  for(unsigned i=0;i<sizeof(analogPins)/sizeof(analogPins[0]); i++) pinMode(analogPins[i], INPUT);
  initDisplays();
  setLEDState(0x0000, 0x000000, 0, 0, 0);
  chronoRowCHR = "CHR ";
  chronoRowUTC = "UTC   ";
  chronoRowET  = "ET  ";
  renderChronoDisplays();
  sendIdentAndState();
}

// ============================================================================
// LOOP
// ============================================================================
void loop(){
  unsigned long now=millis();
  
  // If DIAG sequence is running, drive it and skip normal processing
  if(diagRunning){
    runDiagSequence();
    return;
  }
  
  // Always process incoming host commands
  processSerialTokensFromHost();

  // Check for IDENT delay (non-blocking pause for scan clarity)
  if(identDelayActive){
    if(millis() - identDelayStart >= IDENT_DELAY_MS){
      identDelayActive = false;
    } else {
      // Skip main loop logic during delay, but serial processing continues above
      return;
    }
  }

  if(now-lastLoopTs<LOOP_INTERVAL_MS) return;
  lastLoopTs=now;
  // Inputs
  readShift(inShift1, inShift2);
  for(int g=0; g<NUM_DIRECT_GROUPS; g++){
    int offset = g*8;
    int pinsInGroup = NUM_DIRECT_PINS - offset;
    if(pinsInGroup > 8) pinsInGroup = 8;
    inDirect[g] = readDirectGroup(directInPins + offset, pinsInGroup);
  }

  // --- Chrono button logic (CHR: IN1 bit 3, RST: IN1 bit 4) only in CLOCK:ON mode ---
  static bool lastChrBtn = false, lastRstBtn = false;
  bool chrBtn = (inShift1 & 0b00001000); // IN1 bit 3
  bool rstBtn = (inShift1 & 0b00010000); // IN1 bit 4

  if(clockEnabled) {
    // CHR: toggle start/stop on rising edge
    if(chrBtn && !lastChrBtn) {
      chronoRunning = !chronoRunning;
      Serial.print("CHR:"); Serial.println(chronoRunning ? "START" : "STOP");
    }
    // RST: reset chrono on rising edge
    if(rstBtn && !lastRstBtn) {
      chronoRunning = false;
      chronoSeconds = 0;
      chronoRowCHR = "00.00";
      renderChronoDisplays();
      Serial.println("CHR:RESET");
    }
  }
  lastChrBtn = chrBtn;
  lastRstBtn = rstBtn;

  // DIAG combo: RST (IN1 bit4), CHR (IN1 bit3), DATE (IN6 bit4)
  bool hasIN6 = (NUM_DIRECT_GROUPS >= 4);
  bool diagCombo = (inShift1 & DIAG_MASK_RST) && (inShift1 & DIAG_MASK_CHR) && hasIN6 && (inDirect[3] & DIAG_MASK_DATE);
  if(diagCombo && !diagComboLatched){
    startDiagSequence(); // Start full LED walk + display test
    diagComboLatched = true;
  }
  if(!diagCombo) diagComboLatched = false;

  // Send on changes (compare all inputs against last state)
  bool change = (inShift1!=lastInShift1)||(inShift2!=lastInShift2);
  for(int g=0; g<NUM_DIRECT_GROUPS && !change; g++){
    if(inDirect[g]!=lastInDirect[g]) change = true;
  }
  if(change){ 
    lastInShift1=inShift1; 
    lastInShift2=inShift2; 
    for(int g=0; g<NUM_DIRECT_GROUPS; g++) lastInDirect[g]=inDirect[g];
    sendStatus(); 
  }
  
  // Update clock if enabled
  updateClock();
  
  // Note: Display updates happen via serial commands (CHR:, UTC:, ET:, DSP:)
}
