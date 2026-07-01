// OHP_PRI.INO – OHP Primary Panel (Arduino MEGA 2560)
// (w) 2025 M. Quatember, OHP edition
// Controls HYD_ELEC, FIRE, EMELEC, ELEC, AC sections
#include <LedControl.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================================
// PIN DEFINITIONS – PRIMARY INPUT SHIFTER (74HC165)
// ============================================================================
// HYD_ELEC_IS: CLOCK D3, LATCH D2, DATA D4, AMOUNT 2
const int inClk_HE = 3; const int inLatch_HE = 2; const int inData_HE = 4; const uint8_t IN_CNT_HE = 2;
// FIRE_IS: CLOCK D19, LATCH D18, DATA D14, AMOUNT 1
const int inClk_FI = 19; const int inLatch_FI = 18; const int inData_FI = 14; const uint8_t IN_CNT_FI = 1;
// EMELEC_IS: CLOCK D25, LATCH D24, DATA D26, AMOUNT 4
const int inClk_EM = 25; const int inLatch_EM = 24; const int inData_EM = 26; const uint8_t IN_CNT_EM = 4;
// ELEC_IS: CLOCK D36, LATCH D35, DATA D37, AMOUNT 2
const int inClk_EL = 36; const int inLatch_EL = 35; const int inData_EL = 37; const uint8_t IN_CNT_EL = 2;
// AC_IS: CLOCK D42, LATCH D41, DATA D43, AMOUNT 2
const int inClk_AC = 42; const int inLatch_AC = 41; const int inData_AC = 43; const uint8_t IN_CNT_AC = 2;

#define IN_SHIFTER_BYTES 11  // 2+1+4+2+2
#define IN_SINGLE_COUNT   0   // D9/D10/D20/D23 are direct LEDs now, not inputs
#define IN_TOTAL_BYTES    (IN_SHIFTER_BYTES)  // no extra byte for singles

// ============================================================================
// PIN DEFINITIONS – PRIMARY OUTPUT SHIFTER (74HC595)
// ============================================================================
// HYD_ELEC_LEDDRV: CLK D6, LATCH D5, DATA D7, AMOUNT 3
const int ledClk_HE  = 6;  const int ledLatch_HE  = 5;  const int ledData_HE  = 7;  const uint8_t LED_CNT_HE = 3;
// FIRE_LEDDRV:     CLK D16, LATCH D21, DATA D17, AMOUNT 2
const int ledClk_FI  = 16; const int ledLatch_FI  = 21; const int ledData_FI  = 17; const uint8_t LED_CNT_FI = 2;
// EMELEC_LEDDRV:   CLK D28, LATCH D27, DATA D29, AMOUNT 4
const int ledClk_EM  = 28; const int ledLatch_EM  = 27; const int ledData_EM  = 29; const uint8_t LED_CNT_EM = 4;
// ELEC_LEDDRV:     CLK D39, LATCH D38, DATA D40, AMOUNT 3
const int ledClk_EL  = 39; const int ledLatch_EL  = 38; const int ledData_EL  = 40; const uint8_t LED_CNT_EL = 3;
// AC_LEDDRV:       CLK D45, LATCH D44, DATA D46, AMOUNT 2
const int ledClk_AC  = 45; const int ledLatch_AC  = 44; const int ledData_AC  = 46; const uint8_t LED_CNT_AC = 2;

#define LED_TOTAL_BYTES 15  // 14 shifter bytes + 1 direct LED byte (LED14)

// Direct LEDs (MEGA output pins, active-LOW = LED ON)
// LED14 bit0=D9(HYD_ELEC_eng2_2_off), bit1=D8(HYD_ELEC_eng2_2_fault), bit2=D20(FIRE_apuagentsqd), bit3=D23(FIRE_apuagentdisch)
const uint8_t DLED_PINS[4] = {9, 8, 20, 23};
#define DLED_COUNT 4
// All except D9: 0=LED ON, 1=LED OFF. D9 is hardware-different, no invert.
const bool DLED_INVERT[4] = {false, false, true, true};

// TLC5916 OE (Output Enable) – LOW=active, PWM for brightness
// D11=ANNU_BRT (Annunciator), D10=BACKLIGHT_BRT (Backlight)
const uint8_t OE_ANNU_PIN = 11;
const uint8_t OE_BACKL_PIN = 10;
uint8_t annuBrightness = 255;   // 0-255 PWM for annunciator groups
uint8_t backlBrightness = 255;  // 0-255 PWM for backlight groups

// ============================================================================
// PIN DEFINITIONS – MAX7219 DISPLAYS
// ============================================================================
// ELEC_DISP: CLK D33, DIN D32, LOAD D34, 2 devices
const int dspClkPin  = 33;
const int dspDinPin  = 32;
const int dspLoadPin = 34;
const int dspNumDev  = 1;   // single MAX7219, both displays share same controller

// ============================================================================
// PIN DEFINITIONS – ANALOG
// ============================================================================
#define ANALOG_COUNT 6
const uint8_t ANA_PINS[ANALOG_COUNT] = { A9, A8, A10, A13, A14, A15 };
// A9=AC_FWD_Cabin, A8=AC_AFT_Cabin, A10=AC_Cockpit,
// A13=FIRE_ENG1_AGENT1, A14=FIRE_ENG1_AGENT2, A15=FIRE_ENG1_TEST

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "OHP_PRI, v1.1 MAQ";
const char  FW_VERSION[] = "1.1";
const char  PANEL_SN_PREFIX[] = "OHP-";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

char CFG_SERIAL_NUMBER[10] = "";

// ============================================================================
// MAX7219 DISPLAY SETUP
// ============================================================================
LedControl lc = LedControl(dspDinPin, dspClkPin, dspLoadPin, dspNumDev);
// DSP0=BAT1 (digits 5,4,3), DSP1=BAT2 (digits 2,1,0) on single MAX7219

uint8_t displayBrightness = 15;
uint8_t showSavedDispBr = 15;

// ============================================================================
// LED STATE
// ============================================================================
uint8_t desiredLedState[LED_TOTAL_BYTES] = {0};  // LED0..LED13=shifters, LED14=direct
uint8_t hwLedState[LED_TOTAL_BYTES] = {0};

// ============================================================================
// INPUT STATE
// ============================================================================
uint8_t inputState[IN_TOTAL_BYTES] = {0};
uint8_t lastInputState[IN_TOTAL_BYTES] = {0};

// ============================================================================
// ANALOG STATE
// ============================================================================
uint16_t analogValues[ANALOG_COUNT] = {0};
uint16_t lastAnalogValues[ANALOG_COUNT] = {0};
#define ANA_FLT_SZ 4
uint16_t anaFltBuf[ANALOG_COUNT][ANA_FLT_SZ] = {{0}};
uint8_t  anaFltIdx[ANALOG_COUNT] = {0};

// ============================================================================
// CONFIGURATION
// ============================================================================
uint8_t  CFG_BUTTON_DEBOUNCE = 12;
uint16_t CFG_LED_REFRESH = 1200;
uint16_t CFG_DISPLAY_REFRESH = 1200;
uint8_t  CFG_ANA_DEADBAND = 5;    // pot deadband threshold
char CFG_AIRCRAFT_REG[9] = "D-A320";
char CFG_PCB_VERSION[9] = "PCB 1.0";
bool CFG_PCB_IS_12 = false;

const int EEPROM_BASE_ADDR = 0;
const uint16_t EEPROM_MAGIC = 0xA55A;
const uint8_t  EEPROM_FORMAT_VERSION = 3;

bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";

// ============================================================================
// BOOT / CONNECTION STATE
// ============================================================================
enum BootState { BOOT_INIT, BOOT_RUNNING, BOOT_DIAG_MENU, BOOT_DIAG_TEST };
BootState bootState = BOOT_INIT;

bool bootMessageShown = false;
unsigned long bootMessageStart = 0;
const unsigned long BOOT_MSG_DURATION_MS = 5000;

bool hostOnline = false;

// DIAG
unsigned long diagStartTime = 0;
int diagStage = 0;
bool diagComboWasActive = false;
bool diagReturnToMenu = false;
bool menuInitialized = false;
int menuIndex = 0;
unsigned long lastMenuActivityTs = 0;
// DIAG LED-Walk sub-state
int  diagWalkPhase = 0;       // 0=single-bit walk, 1=byte all-on
int  diagWalkByte = 0;        // current byte index
unsigned long diagWalkLastTs = 0;
// DIAG Show sub-state
int  showPhase = 0;
int  showStep = 0;
unsigned long showLastTs = 0;
int  showRepeat = 1;          // how many full runs (SHOW:N)
// DIAG Display sub-state
unsigned long diagDspStart = 0;   // when display diag started
const unsigned long MENU_INACTIVITY_MS = 60000;

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;
unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
unsigned long lastDebounceTs = 0;

// ============================================================================
// RUNTIME BUFFERS
// ============================================================================
char serialAccum[64] = "";
bool forceSendNext = false;

// ============================================================================
// HELPERS
// ============================================================================
void bin8(char* out, byte b) {
  for (int i = 0; i < 8; i++) out[i] = ((b >> (7 - i)) & 1) ? '1' : '0';
  out[8] = 0;
}
uint8_t parseBin8(const char* s) {
  uint8_t v = 0;
  for (int i = 0; s[i]; i++) { if (s[i]=='0'||s[i]=='1') v=(v<<1)|(s[i]=='1'?1:0); }
  return v;
}
static bool strieq(const char* a, const char* b) {
  while (*a && *b) {
    char ca = (*a>='a'&&*a<='z') ? *a-32 : *a; char cb = (*b>='a'&&*b<='z') ? *b-32 : *b;
    if (ca != cb) return false; a++; b++;
  }
  return *a == *b;
}
static bool stripre(const char* a, const char* pf) {
  while (*pf) {
    char ca = (*a>='a'&&*a<='z') ? *a-32 : *a; char cb = (*pf>='a'&&*pf<='z') ? *pf-32 : *pf;
    if (ca != cb) return false; a++; pf++;
  }
  return true;
}
static int atoi_s(const char* s) {
  int v=0, sign=1; while (*s==' ') s++; if (*s=='-') { sign=-1; s++; }
  while (*s>='0'&&*s<='9') { v=v*10+(*s-'0'); s++; } return v*sign;
}
void refreshPCBVersionFlags() {
  char norm[9]; uint8_t j=0;
  for (uint8_t i=0; CFG_PCB_VERSION[i] && j<8; i++) {
    char c=CFG_PCB_VERSION[i]; if (c==' ') continue; if (c>='a'&&c<='z') c-=32; norm[j++]=c;
  }
  norm[j]=0;
  CFG_PCB_IS_12 = (norm[0]=='P'&&norm[1]=='C'&&norm[2]=='B'&&norm[3]=='1'&&norm[4]=='.'&&norm[5]=='2'&&norm[6]==0);
}
void triggerSoftwareReset(bool showMsg=true) {
  if (showMsg) { Serial.println("RESET:OK ;"); Serial.flush(); delay(100); }
  wdt_enable(WDTO_15MS); while(true) {}
}

// ============================================================================
// MAX7219 DISPLAY FUNCTIONS
// ============================================================================
byte charTo7Seg(char c) {
  if (c=='o') return 0b00011101; if (c=='i') return 0b00010000;
  c = toupper(c);
  switch(c) {
    case '0':return 0b01111110;case '1':return 0b00110000;case '2':return 0b01101101;
    case '3':return 0b01111001;case '4':return 0b00110011;case '5':return 0b01011011;
    case '6':return 0b01011111;case '7':return 0b01110000;case '8':return 0b01111111;
    case '9':return 0b01111011;case 'A':return 0b01110111;case 'B':return 0b00011111;
    case 'C':return 0b01001110;case 'D':return 0b00111101;case 'E':return 0b01001111;
    case 'F':return 0b01000111;case 'H':return 0b00110111;case 'I':return 0b00110000;
    case 'L':return 0b00001110;case 'N':return 0b00010101;case 'O':return 0b01111110;
    case 'P':return 0b01100111;case 'R':return 0b00000101;case 'S':return 0b01011011;
    case 'T':return 0b00001111;case 'U':return 0b00111110;case 'Y':return 0b00111011;
    case '-':return 0b00000001;case ' ':return 0b00000000;default:return 0b00000000;
  }
}
void updateDisplay(int dev, const char* txt) {
  int tl=strlen(txt), tp=0;
  for (int dp=0; dp<6; dp++) {
    if (tp>=tl) { lc.setRow(dev,5-dp,0); continue; }
    char c=txt[tp]; if (c=='.') { tp++; if(tp>=tl){lc.setRow(dev,5-dp,0);continue;} c=txt[tp]; }
    byte pat=charTo7Seg(c);
    if (tp+1<tl && txt[tp+1]=='.') pat|=0b10000000;
    lc.setRow(dev,5-dp,pat); tp++;
  }
}
void dspSetVal(int dev, uint8_t digitOfs, int16_t v, bool blank0) {
  // digitOfs=0 → digits 2,1,0 (BAT2). digitOfs=3 → digits 5,4,3 (BAT1)
  // First clear all 3 digits of this display
  for(int d=0;d<3;d++) lc.setRow(dev, digitOfs+d, 0);
  if (blank0 && v==0) return;
  char buf[4]; bool neg=(v<0); uint16_t av=neg?(uint16_t)(-v):(uint16_t)v;
  int8_t p=2; buf[3]=0;
  if (av==0) buf[p--]='0';
  else while(av>0&&p>=0) { buf[p--]='0'+(av%10); av/=10; }
  if (neg&&p>=0) buf[p--]='-';
  while(p>=0) buf[p--]=' ';
  // Write 3 digits right-aligned: buf[2]→digitOfs+2, buf[1]→digitOfs+1, buf[0]→digitOfs+0
  for(int d=0;d<3;d++) lc.setRow(dev, digitOfs+(2-d), charTo7Seg(buf[d]));
}
// String-based display setter with DP support (like RMP updateDisplay).
// txt = raw string (e.g. "12.3"), digitOfs = row offset for the 3-digit group.
// DP is folded into the digit to its LEFT: "12.3" uses 3 positions: '1' '2.' '3'.
static int dspEffLen(const char* s) { int cnt=0; for(int i=0;s[i];i++) if(s[i]!='.')cnt++; return cnt; }
void dspSetValStr(int dev, uint8_t digitOfs, const char* txt) {
  // Clear all 3 digits
  for(int d=0;d<3;d++) lc.setRow(dev, digitOfs+d, 0);
  if (!txt || txt[0]==0) return;
  int elen = dspEffLen(txt);
  if (elen==0) return;
  // Build right-aligned segment patterns, DP folded into the left neighbour.
  // Walk through txt LEFT-TO-RIGHT, skipping '.' as a modifier that sets DP
  // on the digit that PRECEDES it in the string.
  uint8_t pat[3] = {0,0,0};
  int srcLen = (int)strlen(txt);
  int srcPos = 0;
  int elenRemaining = elen;
  int padLeft = (3 > elenRemaining) ? (3 - elenRemaining) : 0;  // blank positions on the left
  // Fill blank positions
  for (int d = 0; d < padLeft; d++) pat[d] = 0;
  int d = padLeft;
  while (d < 3 && srcPos < srcLen) {
    char c = txt[srcPos];
    if (c == '.') { srcPos++; continue; }  // DP handled in next iteration
    uint8_t p = charTo7Seg(c);
    // Check if '.' follows this character → set DP on this digit
    if (srcPos + 1 < srcLen && txt[srcPos + 1] == '.') p |= 0b10000000;
    pat[d] = p;
    srcPos++; d++;
  }
  for(int d=0;d<3;d++) lc.setRow(dev, digitOfs+(2-d), pat[d]);
}
void setDisplayBrightness(uint8_t b) { displayBrightness=constrain(b,0,15);
  lc.setIntensity(0, displayBrightness); }
void initDisplays() {
  // Explicitly configure MAX7219: scan limit = 7 (all 8 digits multiplexed)
  // Without this, scan limit defaults may cause missing/stuck digits and brightness imbalance
  lc.shutdown(0, true);
  lc.setScanLimit(0, 7);
  lc.setIntensity(0, displayBrightness);
  lc.clearDisplay(0);
  lc.shutdown(0, false);
  // Startup test: light all segments for 500ms so user sees displays work
  for(int i=0;i<6;i++) lc.setRow(0, i, 0b01111111);
  delay(500);
  lc.clearDisplay(0);
}

// ============================================================================
// LED DRIVER (TLC5916) – MULTI-GROUP with OE/PWM brightness
// TLC5916 protocol = same as 74HC595 (SDI/CLK/LE), but OE must be LOW
// ============================================================================
void shiftOutOne(uint8_t clk, uint8_t latch, uint8_t data, const uint8_t* bytes, uint8_t cnt) {
  digitalWrite(latch, LOW);
  for (int8_t b=cnt-1; b>=0; b--) shiftOut(data, clk, MSBFIRST, bytes[b]);
  digitalWrite(latch, HIGH);
}
void applyLEDOutputs() {
  uint8_t bi=0;
  // HYD_ELEC, FIRE, EMELEC, ELEC, AC → all on ANNU_BRT (D11)
  analogWrite(OE_ANNU_PIN, 255-annuBrightness);  // TLC5916 OE: LOW=full brightness
  analogWrite(OE_BACKL_PIN, 255-backlBrightness);
  shiftOutOne(ledClk_HE, ledLatch_HE, ledData_HE, &desiredLedState[bi], LED_CNT_HE); bi+=LED_CNT_HE;
  shiftOutOne(ledClk_FI, ledLatch_FI, ledData_FI, &desiredLedState[bi], LED_CNT_FI); bi+=LED_CNT_FI;
  shiftOutOne(ledClk_EM, ledLatch_EM, ledData_EM, &desiredLedState[bi], LED_CNT_EM); bi+=LED_CNT_EM;
  shiftOutOne(ledClk_EL, ledLatch_EL, ledData_EL, &desiredLedState[bi], LED_CNT_EL); bi+=LED_CNT_EL;
  shiftOutOne(ledClk_AC, ledLatch_AC, ledData_AC, &desiredLedState[bi], LED_CNT_AC); bi+=LED_CNT_AC;
  // Direct LEDs (LED14, active-LOW: LOW=ON, HIGH=OFF, D9 no invert)
  uint8_t dl=desiredLedState[14];
  for(uint8_t i=0;i<DLED_COUNT;i++){
    bool on=(dl>>i)&1;
    if(DLED_INVERT[i]) on=!on;  // D9: 1=OFF, 0=ON
    digitalWrite(DLED_PINS[i], on?LOW:HIGH);  // LOW=LED ON (active-low)
  }
  memcpy(hwLedState, desiredLedState, LED_TOTAL_BYTES);
}
void ledClearAll() { memset(desiredLedState, 0, LED_TOTAL_BYTES); applyLEDOutputs(); }
void ledSetAll(uint8_t v) { memset(desiredLedState, v, LED_TOTAL_BYTES); applyLEDOutputs(); }

// ============================================================================
// INPUT SHIFT REGISTERS (74HC165) – MULTI-GROUP
// ============================================================================
void readOneInGrp(uint8_t clk, uint8_t latch, uint8_t data, uint8_t* out, uint8_t cnt) {
  digitalWrite(latch, LOW); delayMicroseconds(20); digitalWrite(latch, HIGH); delayMicroseconds(20);
  for (uint8_t b=0; b<cnt; b++) out[b] = ~shiftIn(data, clk, MSBFIRST);
  digitalWrite(latch, LOW); delayMicroseconds(20); digitalWrite(latch, HIGH); delayMicroseconds(20);
  for (int i=0; i<16; i++) { digitalWrite(clk, LOW); delayMicroseconds(5); digitalWrite(clk, HIGH); delayMicroseconds(5); }
}
void readAllInputs() {
  uint8_t bi=0;
  readOneInGrp(inClk_HE, inLatch_HE, inData_HE, &inputState[bi], IN_CNT_HE); bi+=IN_CNT_HE;
  readOneInGrp(inClk_FI, inLatch_FI, inData_FI, &inputState[bi], IN_CNT_FI); bi+=IN_CNT_FI;
  readOneInGrp(inClk_EM, inLatch_EM, inData_EM, &inputState[bi], IN_CNT_EM); bi+=IN_CNT_EM;
  readOneInGrp(inClk_EL, inLatch_EL, inData_EL, &inputState[bi], IN_CNT_EL); bi+=IN_CNT_EL;
  readOneInGrp(inClk_AC, inLatch_AC, inData_AC, &inputState[bi], IN_CNT_AC); bi+=IN_CNT_AC;
}

// ============================================================================
// ANALOG INPUTS
// ============================================================================
void readAnalogFiltered() {
  for (uint8_t i=0; i<ANALOG_COUNT; i++) {
    uint16_t raw=analogRead(ANA_PINS[i]);
    anaFltBuf[i][anaFltIdx[i]]=raw; anaFltIdx[i]=(anaFltIdx[i]+1)%ANA_FLT_SZ;
    uint32_t sum=0; for(uint8_t j=0;j<ANA_FLT_SZ;j++) sum+=anaFltBuf[i][j];
    analogValues[i]=(uint16_t)(sum/ANA_FLT_SZ);
  }
}
bool analogChanged() {
  for (uint8_t i=0; i<ANALOG_COUNT; i++) {
    int16_t d=(int16_t)analogValues[i]-(int16_t)lastAnalogValues[i];
    int16_t db=(int16_t)CFG_ANA_DEADBAND;
    if (d<-db || d>db) { lastAnalogValues[i]=analogValues[i]; return true; }
  }
  return false;
}

// ============================================================================
// SERIAL IDENT / STATE
// ============================================================================
void sendIdentAndState() {
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
  if (CFG_SERIAL_NUMBER[0]!=0) { Serial.print(", SN:"); Serial.print(PANEL_SN_PREFIX); Serial.print(CFG_SERIAL_NUMBER); }
  Serial.print(";STATE:RUNNING;REG:"); Serial.print(CFG_AIRCRAFT_REG); Serial.println(";");
}
void maybeSendIdentStartup() { if (!identSentOnStart) { delay(150); sendIdentAndState(); identSentOnStart=true; } }

// ============================================================================
// STATUS SENDING
// ============================================================================
void sendStatusImmediate() {
  char b8[9];
  for (uint8_t i=0; i<IN_TOTAL_BYTES; i++) { bin8(b8,inputState[i]); Serial.print("IN");Serial.print(i+1);Serial.print(":");Serial.print(b8);Serial.print(";"); }
  for (uint8_t i=0; i<ANALOG_COUNT; i++) { Serial.print("POT");Serial.print(i+1);Serial.print(":");Serial.print(analogValues[i]);Serial.print(";"); }
  Serial.println(); lastSendTs=millis(); forceSendNext=false;
}

// ============================================================================
// SERIAL COMMAND PARSING
// ============================================================================
void processIncomingLine(const char* line) {
  char token[32]; int tlen=0;
  for (int i=0;;i++) {
    char c=line[i];
    if (c==';'||c==0) {
      if (tlen==0) { if(c==0)break; else continue; }
      token[tlen]=0; tlen=0;
      int end=strlen(token)-1; while(end>=0&&token[end]==' ') token[end--]=0;
      char* tok=token; while(*tok==' ') tok++;
      if (*tok==0) { if(c==0)break; else continue; }
      char* colon=strchr(tok,':');
      if (!colon) {
        if (strieq(tok,"VER")||strieq(tok,"VERSION")||strieq(tok,"IDENT")) { sendIdentAndState(); }
        else if (strieq(tok,"REQ")) { sendStatusImmediate(); }
        else if (strieq(tok,"DIAG")&&bootState==BOOT_RUNNING) { bootState=BOOT_DIAG_MENU; diagStartTime=millis(); diagStage=0; menuInitialized=false; Serial.println("DIAG:MENU_START;"); }
        else if (stripre(tok,"SET ")) { char* sc=tok+4; while(*sc==' ') sc++; if (stripre(sc,"EXIT")){settingsEnabled=false;Serial.println("SET EXIT:OK ;");} else if (stripre(sc,"WRITE")&&settingsEnabled){saveHWInfo();settingsEnabled=false;Serial.println("WRITE:OK ;");triggerSoftwareReset(false);} else if (stripre(sc,"WRITE")){Serial.println("WRITE:LOCKED ;");} }
        if (c==0) break; else continue;
      }
      *colon=0; char* key=tok; char* val=colon+1;
      int ke=strlen(key)-1; while(ke>=0&&key[ke]==' ') key[ke--]=0;
      char* kp=key; while(*kp==' ') kp++;
      while(*val==' ') val++; int ve=strlen(val)-1; while(ve>=0&&val[ve]==' ') val[ve--]=0;
      // LEDx (case-insensitive prefix check) – silent apply, matching RMP behavior
      if (stripre(kp,"LED")) { int idx=atoi_s(kp+3)-1; if (idx>=0 && idx<LED_TOTAL_BYTES){desiredLedState[idx]=parseBin8(val);applyLEDOutputs();} }
      // STATE – silent, matching RMP behavior
      else if (strieq(kp,"STATE")) { hostOnline=(strcmp(val,"01")==0||strcmp(val,"1")==0); }
      // DSPx: DSP1=BAT1(digits 5,4,3), DSP2=BAT2(digits 2,1,0) – silent apply
      else if (strieq(kp,"DSP1")) { int v=atoi_s(val); if(v<0||val[0]=='-')for(int d=3;d<6;d++)lc.setRow(0,d,0); else if(strchr(val,'.'))dspSetValStr(0,3,val); else dspSetVal(0,3,v,false); }
      else if (strieq(kp,"DSP2")) { int v=atoi_s(val); if(v<0||val[0]=='-')for(int d=0;d<3;d++)lc.setRow(0,d,0); else if(strchr(val,'.'))dspSetValStr(0,0,val); else dspSetVal(0,0,v,false); }
      else if (strieq(kp,"ANN")) { annuBrightness=constrain(atoi_s(val),0,255); applyLEDOutputs(); }
      else if (strieq(kp,"BL"))  { backlBrightness=constrain(atoi_s(val),0,255); applyLEDOutputs(); }
      else if (strieq(kp,"DISP_BL")) { setDisplayBrightness(atoi_s(val)); }
      else if (strieq(kp,"REQ")) { forceSendNext=true; }
      else if (strieq(kp,"VER")) { sendIdentAndState(); }
      // CFG
      else if (strieq(kp,"CFG")) { char* cp=val;
        while(*cp){while(*cp==' '||*cp==';')cp++; if(!*cp)break; char* semi=strchr(cp,';'); if(semi)*semi=0;
          if(stripre(cp,"DEB"))CFG_BUTTON_DEBOUNCE=constrain(atoi_s(cp+3),0,99);
          else if(stripre(cp,"DEAD"))CFG_ANA_DEADBAND=constrain(atoi_s(cp+4),1,100);
          else if(stripre(cp,"LED"))CFG_LED_REFRESH=constrain(atoi_s(cp+3),100,10000);
          else if(stripre(cp,"DSP"))CFG_DISPLAY_REFRESH=constrain(atoi_s(cp+3),100,10000);
          else if(stripre(cp,"REG:")){strncpy(CFG_AIRCRAFT_REG,cp+4,8);CFG_AIRCRAFT_REG[8]=0;}
          else if(stripre(cp,"SN:")){strncpy(CFG_SERIAL_NUMBER,cp+3,8);CFG_SERIAL_NUMBER[8]=0;}
          if(semi)*semi=';'; cp=semi?semi+1:cp+strlen(cp);
        }
      }
      // SET
      else if (stripre(tok,"SET ")) { char* sc=tok+4; while(*sc==' ') sc++;
        if (stripre(sc,"ENA")) { if(strcmp(val,SETTINGS_PIN)==0){settingsEnabled=true;Serial.println("SET ENA:OK ;");}else Serial.println("SET ENA:FAIL ;"); }
        else if (stripre(sc,"EXIT")) { settingsEnabled=false; Serial.println("SET EXIT:OK ;"); }
        else if ((stripre(sc,"WRI")||stripre(sc,"WRITE"))&&settingsEnabled) { saveHWInfo(); settingsEnabled=false; Serial.println("WRITE:OK ;"); triggerSoftwareReset(false); }
        else if (stripre(sc,"FW")&&settingsEnabled) { char* vs=val; while(*vs==' ') vs++; char* dot=strchr(vs,'.'); if(dot&&dot>vs){*dot=0;int ma=atoi_s(vs);int mi=atoi_s(dot+1);*dot='.';if(ma>=1&&ma<=9&&mi>=0&&mi<=9){snprintf(CFG_PCB_VERSION,9,"PCb %d.%d",ma,mi);refreshPCBVersionFlags();Serial.println("SET FW:OK ;");}else Serial.println("SET FW:RANGE ;");}else Serial.println("SET FW:FORMAT ;"); }
        else if (stripre(sc,"ACID")&&settingsEnabled) { char* idt=val; while(*idt==' ') idt++; int il=strlen(idt); if(il>=3&&il<=8){strncpy(CFG_AIRCRAFT_REG,idt,8);CFG_AIRCRAFT_REG[8]=0;Serial.println("SET ACID:OK ;");}else Serial.println("SET ACID:FORMAT ;"); }
        else if (stripre(sc,"SN")&&settingsEnabled) { char* sv=val; while(*sv==' ') sv++; int sl=strlen(sv); bool vok=(sl==8); for(int i2=0;i2<sl&&vok;i2++){char ch=sv[i2];if(!((ch>='0'&&ch<='9')||(ch>='A'&&ch<='F')||(ch>='a'&&ch<='f')))vok=false;} if(vok){for(int i2=0;i2<8;i2++)CFG_SERIAL_NUMBER[i2]=(sv[i2]>='a'&&sv[i2]<='f')?sv[i2]-32:sv[i2];CFG_SERIAL_NUMBER[8]=0;Serial.print("SET SN:OK:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);}else Serial.println("SET SN:FORMAT ;"); }
      }
      if (c==0) break;
    } else { if (tlen<31) token[tlen++]=c; }
  }
}

void processSerialTokensFromHost() {
  while (Serial.available()) {
    char c=Serial.read(); if(c=='\r')continue; if(c=='\n')c=';';
    int alen=strlen(serialAccum); if(alen<63){serialAccum[alen]=c;serialAccum[alen+1]=0;}
    char* semi;
    while ((semi=strchr(serialAccum,';'))!=NULL) {
      *semi=0; char token[32]; char* tp=serialAccum; while(*tp==' ') tp++;
      int ti; for(ti=0;tp[ti]&&ti<31;ti++)token[ti]=tp[ti];token[ti]=0;
      int tend=ti-1; while(tend>=0&&token[tend]==' ')token[tend--]=0;
      int remaining=strlen(semi+1); memmove(serialAccum,semi+1,remaining+1);
      if (token[0]==0) continue;
      char tokenUp[32]; for(ti=0;token[ti]&&ti<31;ti++)tokenUp[ti]=(token[ti]>='a'&&token[ti]<='z')?token[ti]-32:token[ti];tokenUp[ti]=0;
      if (strcmp(tokenUp,"VER")==0||strcmp(tokenUp,"VERSION")==0||strcmp(tokenUp,"IDENT")==0){sendIdentAndState();identSentOnStart=true;bootMessageShown=true;bootMessageStart=millis();continue;}
      if (strcmp(tokenUp,"EXIT")==0){if(bootState==BOOT_DIAG_MENU){bootState=BOOT_RUNNING;menuInitialized=false;diagStage=0;ledClearAll();Serial.println("DIAG:EXIT;");}else{settingsEnabled=false;Serial.println("EXIT:OK ;");}continue;}
      if (strcmp(tokenUp,"PCB")==0){Serial.print("PCB:");char* sp=strchr(CFG_PCB_VERSION,' ');if(sp&&*(sp+1)){Serial.print("v");Serial.print(sp+1);}else Serial.print(CFG_PCB_VERSION);Serial.println(" ;");continue;}
      if (strcmp(tokenUp,"RESET")==0){Serial.println("RESET:OK ;");triggerSoftwareReset();continue;}
      if (strcmp(tokenUp,"REQ")==0){sendStatusImmediate();continue;}
      if (strcmp(tokenUp,"DIAG")==0){bootState=BOOT_DIAG_MENU;diagStartTime=millis();diagStage=0;menuInitialized=false;ledClearAll();Serial.println("DIAG:MENU;");continue;}
      // ---- DIAG serial navigation (works only in DIAG mode) ----
      if (bootState==BOOT_DIAG_MENU) {
        lastMenuActivityTs=millis();
        if (strcmp(tokenUp,"NEXT")==0)  { menuIndex=(menuIndex+1)%5; Serial.print("DIAG:MENU:");Serial.println(menuIndex); if(menuIndex==0){Serial.println("DIAG:LED_WALK;");diagStage=1;diagWalkPhase=0;diagWalkByte=0;diagWalkLastTs=millis();ledClearAll();} else if(menuIndex==1){Serial.println("DIAG:INPUTS;"); diagStage=0; readAllInputs(); for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("IN");Serial.print(i);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println();} else if(menuIndex==2){Serial.println("DIAG:DSP_START;");diagStage=2;diagDspStart=millis();ledClearAll();} else if(menuIndex==3){diagStage=0; Serial.print("HEALTH:UPTIME=");Serial.print(millis()/1000);Serial.print("s;LASTERR=0;FREEMEM=");extern int __heap_start,*__brkval;int vv;Serial.print((int)&vv-(__brkval==0?(int)&__heap_start:(int)__brkval));Serial.print(";VERSION=");Serial.print(FW_VERSION);Serial.println(";");Serial.println("DIAG:ALL_OK;");} else if(menuIndex==4){bootState=BOOT_RUNNING;menuInitialized=false;diagStage=0;ledClearAll();Serial.println("DIAG:EXIT;");} continue;}
        if (strcmp(tokenUp,"LED")==0)  { menuIndex=0; Serial.println("DIAG:LED_WALK;");diagStage=1;diagWalkPhase=0;diagWalkByte=0;diagWalkLastTs=millis();ledClearAll();continue;}
        if (strcmp(tokenUp,"IN")==0)   { diagStage=0; readAllInputs(); for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("IN");Serial.print(i);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println();continue;}
        if (strcmp(tokenUp,"HEALTH")==0){ diagStage=0; Serial.print("HEALTH:UPTIME=");Serial.print(millis()/1000);Serial.print("s;LASTERR=0;FREEMEM=");extern int __heap_start,*__brkval;int vv;Serial.print((int)&vv-(__brkval==0?(int)&__heap_start:(int)__brkval));Serial.print(";VERSION=");Serial.print(FW_VERSION);Serial.println(";");continue;}
        if (strcmp(tokenUp,"DSP")==0)   { diagStage=2; diagDspStart=millis(); ledClearAll(); Serial.println("DIAG:DSP_START;");continue;}
        if (strcmp(tokenUp,"SHOW")==0||strncmp(tokenUp,"SHOW:",5)==0) {
          showRepeat=1;
          if(tokenUp[4]==':'){int n=atoi_s(tokenUp+5);if(n>=1&&n<=9)showRepeat=n;}
          menuIndex=5; Serial.print("DIAG:SHOW_START x");Serial.print(showRepeat);Serial.println(";");
          diagStage=3;showPhase=0;showStep=0;showLastTs=millis();ledClearAll();
          for(int d=0;d<6;d++)lc.setRow(0,d,0);showSavedDispBr=displayBrightness;setDisplayBrightness(0);
          annuBrightness=255;backlBrightness=255;continue;
        }
      }
      if (strchr(token,':')!=NULL){char lineBuf[64];snprintf(lineBuf,sizeof(lineBuf),"%s;",token);processIncomingLine(lineBuf);continue;}
    }
  }
}

// ============================================================================
// DIAG MODE – Two-phase LED walk: single-bit, then byte-all-on
// ============================================================================
void diagLedWalkStep(unsigned long now) {
  const uint16_t totalBits = (uint16_t)LED_TOTAL_BYTES * 8;
  const unsigned long SINGLE_MS = 150;
  const unsigned long BYTE_MS   = 800;
  lastMenuActivityTs=now;

  if (diagWalkPhase == 0) {
    // ---- Phase 0: Single-bit walk through ALL LEDs (one pass) ----
    if (now - diagWalkLastTs >= SINGLE_MS) {
      diagWalkLastTs = now;
      memset(desiredLedState, 0, LED_TOTAL_BYTES);  // clear buffer only, no apply
      if (diagWalkByte < totalBits) {
        uint8_t bi = diagWalkByte / 8;
        desiredLedState[bi] = 1 << (diagWalkByte % 8);
        applyLEDOutputs();  // single apply = no flicker
        Serial.print("DIAG:WALK_BIT="); Serial.print(diagWalkByte);
        Serial.print("  LED"); Serial.print(bi+1);
        Serial.print("."); Serial.println(diagWalkByte % 8);
      } else {
        applyLEDOutputs();  // apply cleared state for last step
      }
      diagWalkByte++;
      if (diagWalkByte > totalBits) {
        diagWalkPhase = 1; diagWalkByte = 0; diagWalkLastTs = now;
        Serial.println("DIAG:WALK_BYTE_ALL_ON_START;");
      }
    }
  } else {
    // ---- Phase 1: Byte-by-byte ALL-ON with brightness test ----
    if (now - diagWalkLastTs >= BYTE_MS) {
      diagWalkLastTs = now;
      memset(desiredLedState, 0, LED_TOTAL_BYTES);
      if (diagWalkByte < LED_TOTAL_BYTES) {
        desiredLedState[diagWalkByte] = 0xFF;
        // Alternate brightness: even bytes=ANNU full, odd bytes=BACKL dim
        if (diagWalkByte % 2 == 0) {
          annuBrightness = 255; backlBrightness = 64;
        } else {
          annuBrightness = 64;  backlBrightness = 255;
        }
        applyLEDOutputs();
        Serial.print("DIAG:ALL_ON LED"); Serial.print(diagWalkByte+1);
        Serial.print(" ANN="); Serial.print(annuBrightness);
        Serial.print(" BL="); Serial.println(backlBrightness);
      } else {
        applyLEDOutputs();
      }
      diagWalkByte++;
      if (diagWalkByte > LED_TOTAL_BYTES) {
        // Single run complete – restore full brightness, return to menu
        annuBrightness = 255; backlBrightness = 255;
        ledClearAll();
        diagStage = 0;
        Serial.println("DIAG:LED_WALK_DONE;");
      }
    }
  }
}

// ============================================================================
// DIAG – Display Test: counter + segment sweep (RMP-style)
// ============================================================================
void diagRunDisplay(unsigned long now) {
  unsigned long e = now - diagDspStart;
  lastMenuActivityTs = now;

  // Phase 0 (0-5000ms): counting 0-9 on all 6 display digits, DP sweeps
  if (e < 5000) {
    int dig = (e / 400) % 10;
    int dpPos = (e / 400) % 3;
    for (int g = 0; g < 2; g++) {
      uint8_t ofs = g * 3;
      for (int d = 0; d < 3; d++) {
        uint8_t pat = charTo7Seg('0' + dig);
        if (d == dpPos) pat |= 0b10000000;
        lc.setRow(0, ofs + (2 - d), pat);
      }
    }
    if (e < 200) Serial.println("DIAG:DSP_COUNT;");
  }
  // Phase 1 (5000-9000ms): segment sweep a→g→dp across all 6 digits
  else if (e < 9000) {
    int seg = ((e - 5000) / 250) % 8;
    uint8_t pat = 1 << seg;
    for (int d = 0; d < 6; d++) lc.setRow(0, d, pat);
    if (e < 5200) Serial.println("DIAG:DSP_SEGMENT;");
  }
  // Single run complete – clear displays, return to menu
  else {
    for (int d = 0; d < 6; d++) lc.setRow(0, d, 0);
    diagStage = 0;
    Serial.println("DIAG:DSP_DONE;");
  }
}

void diagRunMenu() {
  unsigned long now=millis();
  readAllInputs();
  if (!menuInitialized) { menuInitialized=true; menuIndex=0; lastMenuActivityTs=now; Serial.println("DIAG:MENU;"); ledClearAll(); return; }
  if (diagStage==0 && (now-lastMenuActivityTs)>=MENU_INACTIVITY_MS) { bootState=BOOT_RUNNING; menuInitialized=false; ledClearAll(); Serial.println("DIAG:EXIT_TIMEOUT;"); return; }
  // Check input for menu nav
  if (memcmp(inputState,lastInputState,IN_TOTAL_BYTES)!=0) {
    lastMenuActivityTs=now;
    if (inputState[0]!=lastInputState[0]) {
      uint8_t ch=inputState[0]^lastInputState[0];
      if (ch&0x01 && (inputState[0]&0x01)) {
        menuIndex=(menuIndex+1)%5; memcpy(lastInputState,inputState,IN_TOTAL_BYTES);
        Serial.print("DIAG:MENU:"); Serial.println(menuIndex);
        if (menuIndex==0) {
          Serial.println("DIAG:LED_WALK;");
          diagStage=1; diagStartTime=now;
          diagWalkPhase=0; diagWalkByte=0; diagWalkLastTs=now;
          ledClearAll();
        }
        else if (menuIndex==1) { Serial.println("DIAG:INPUTS;"); for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("IN");Serial.print(i);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println(); }
        else if (menuIndex==2) { Serial.println("DIAG:DSP_START;"); diagStage=2; diagDspStart=now; ledClearAll(); }
        else if (menuIndex==3) { Serial.print("HEALTH:UPTIME=");Serial.print(millis()/1000);Serial.print("s;LASTERR=0;FREEMEM=");extern int __heap_start,*__brkval;int vv;Serial.print((int)&vv-(__brkval==0?(int)&__heap_start:(int)__brkval));Serial.print(";VERSION=");Serial.print(FW_VERSION);Serial.println(";"); }
        else if (menuIndex==4) { bootState=BOOT_RUNNING; menuInitialized=false; ledClearAll(); Serial.println("DIAG:EXIT;"); }
      }
    }
  }
  // ---- LED Walk active ----
  if (diagStage==1) { diagLedWalkStep(now); }
  // ---- Display Test active ----
  if (diagStage==2) { diagRunDisplay(now); }
  // ---- SHOW active ----
  if (diagStage==3) { diagShowStep(now); }
}

// ============================================================================
// DIAG SHOW – Demo animation with PWM breathing + displays
// ============================================================================
void diagShowStep(unsigned long now) {
  const uint16_t totalBits = (uint16_t)LED_TOTAL_BYTES * 8;
  lastMenuActivityTs=now;

  if (showPhase == 0) {
    // ---- Phase 0: Knight Rider (left→right→left across all bits) ----
    const unsigned long KR_MS = 30;
    if (now - showLastTs >= KR_MS) {
      showLastTs = now;
      int pos = showStep;
      if (pos >= totalBits) { pos = totalBits*2 - 2 - showStep; }
      if (showStep >= totalBits*2 - 1) {
        showPhase=1; showStep=0; showLastTs=now;
        ledClearAll();
        Serial.println("DIAG:SHOW_CASCADE;");
        return;
      }
      memset(desiredLedState, 0, LED_TOTAL_BYTES);
      uint8_t bi = pos / 8;
      desiredLedState[bi] = 1 << (pos % 8);
      applyLEDOutputs();
      showStep++;
    }
  } else if (showPhase == 1) {
    // ---- Phase 1: Cascade fill ----
    const unsigned long CAS_MS = 100;
    if (now - showLastTs >= CAS_MS) {
      showLastTs = now;
      if (showStep < LED_TOTAL_BYTES) {
        desiredLedState[showStep] = 0xFF;
        applyLEDOutputs();
      } else if (showStep < LED_TOTAL_BYTES*2) {
        desiredLedState[showStep - LED_TOTAL_BYTES] = 0x00;
        applyLEDOutputs();
      } else {
        showPhase=2; showStep=0; showLastTs=now;
        ledSetAll(0xFF);
        annuBrightness=0; backlBrightness=0;
        applyLEDOutputs();
        setDisplayBrightness(0);
        for(int d=0;d<6;d++) lc.setRow(0,d,0xFF); // all segments on
        Serial.println("DIAG:SHOW_BREATHE;");
        return;
      }
      showStep++;
    }
  } else if (showPhase == 2) {
    // ---- Phase 2: Breathing – PWM fade + display content show (7 cycles) ----
    unsigned long elapsed = (now - showLastTs);
    int cycle = elapsed / 2000;
    if (cycle >= 7) {
      showPhase=3; showStep=0; showLastTs=now;
      annuBrightness=255; backlBrightness=255;
      ledSetAll(0xFF);
      setDisplayBrightness(15);
      for(int d=0;d<6;d++) lc.setChar(0,d,'8',false);
      Serial.println("DIAG:SHOW_FINALE;");
      return;
    }
    int fp = elapsed % 2000;
    uint8_t br = (fp < 1000) ? (fp / 4) : (255 - ((fp - 1000) / 4));
    annuBrightness = br;
    backlBrightness = br;
    applyLEDOutputs();
    setDisplayBrightness(br / 17);
    // Display content – changes each cycle, 6-char buffer → no overlay
    int cp = fp;
    char dspBuf[7] = "      ";
    if (cycle == 0) {
      // "A320" / " OHP " – letters appear one by one
      const char* L="A320  "; const char* R=" OHP  ";
      int vis = (cp<400)?1:(cp<800)?2:(cp<1200)?3:4;
      for(int i=0;i<3;i++) dspBuf[i]   = (i<vis && L[i]) ? L[i] : ' ';
      for(int i=0;i<3;i++) dspBuf[i+3] = (i<vis && R[i]) ? R[i] : ' ';
      updateDisplay(0, dspBuf);
    } else if (cycle == 1) {
      // Number walk 0-9 on all 6 digits
      int dig = (cp / 150) % 10;
      for(int d=0;d<6;d++) lc.setChar(0,d,'0'+dig,false);
    } else if (cycle == 2) {
      // "brIGht" / "tESt  " (RMP-style)
      for(int i=0;i<3;i++) dspBuf[i]   = "brI"[i];
      for(int i=0;i<3;i++) dspBuf[i+3] = "tES"[i];
      updateDisplay(0, dspBuf);
    } else if (cycle == 3) {
      // Segment dance – single segments sweep a→g→dp
      int seg = (cp / 180) % 8;
      uint8_t pat = 1 << seg;
      for(int d=0;d<6;d++) lc.setRow(0,d,pat);
      if(cp<100)Serial.println("DIAG:SEGMENT_DANCE;");
    } else if (cycle == 4) {
      // Cycle 4: "dIAG" / "SHOW "
      for(int i=0;i<3;i++) dspBuf[i]   = "dIA"[i];
      for(int i=0;i<3;i++) dspBuf[i+3] = "SHO"[i];
      updateDisplay(0, dspBuf);
    } else if (cycle == 5) {
      // Cycle 5: Number walk 0-9 again
      int dig = (cp / 120) % 10;
      for(int d=0;d<6;d++) lc.setChar(0,d,'0'+dig,false);
    } else {
      // Cycle 6: "A320" / "OHP " – final
      for(int i=0;i<3;i++) dspBuf[i]   = "A32"[i];
      for(int i=0;i<3;i++) dspBuf[i+3] = "OHP"[i];
      updateDisplay(0, dspBuf);
    }
  } else if (showPhase == 3) {
    // ---- Phase 3: All ON with "888" displays → hold 2.5s → done or repeat ----
    if (now - showLastTs >= 2500) {
      showRepeat--;
      if (showRepeat > 0) {
        // Restart show
        showPhase=0; showStep=0; showLastTs=now;
        ledClearAll();
        for(int d=0;d<6;d++)lc.setRow(0,d,0);
        setDisplayBrightness(0);
        annuBrightness=255; backlBrightness=255;
        Serial.print("DIAG:SHOW_REPEAT ");Serial.print(showRepeat);Serial.println(";");
      } else {
        ledClearAll();
        annuBrightness=255; backlBrightness=255;
        applyLEDOutputs();
        setDisplayBrightness(showSavedDispBr);
        for(int d=0;d<6;d++) lc.setRow(0,d,0);
        diagStage = 0;
        showPhase = 0; showStep = 0; showRepeat = 1;
        Serial.println("DIAG:SHOW_DONE;");
      }
    }
  }
}

void diagRunFull() {
  unsigned long e=millis()-diagStartTime;
  // Stage 0+1: Display count + all LEDs
  if (e<2000) { ledClearAll(); for(int d=0;d<dspNumDev;d++)lc.clearDisplay(d); if(e<500)Serial.println("DIAG:FULL_START;"); }
  else if (e<7000) { ledClearAll(); for(int d=0;d<dspNumDev;d++){int dig=((e-2000)/500)%10;int dpP=((e-2000)/500)%6;for(int i=0;i<6;i++){uint8_t p=charTo7Seg('0'+dig);if(i==dpP&&d==0)p|=0x80;lc.setRow(d,i,p);}} }
  else if (e<10000) { ledSetAll(0xFF); Serial.println("DIAG:ALL_LEDS_ON;"); }
  // Stage 2: Single-bit walk
  else if (e<18000) {
    if (e<10100) { diagWalkPhase=0; diagWalkByte=0; diagWalkLastTs=e-150; Serial.println("DIAG:WALK_BIT_START;"); }
    diagLedWalkStep(e);
  }
  // Stage 3: Inputs
  else if (e<21000) { ledClearAll(); readAllInputs(); for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("DIAG:IN");Serial.print(i);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println(); }
  // Stage 4: Segment sweep
  else if (e<26000) { ledClearAll(); if(e>=23000){int si=((e-23000)/300)%8;uint8_t sp=1<<si;for(int d=0;d<dspNumDev;d++)for(int i=0;i<6;i++)lc.setRow(d,i,sp);} }
  // Stage 5: Brightness fade
  else if (e<31000) { ledSetAll(0xFF); int fp=(e-26000)%2000;uint8_t br=(fp<1000)?(fp/4):(255-((fp-1000)/4));for(int d=0;d<dspNumDev;d++)lc.setIntensity(d,br/17); }
  else if (e<34000) { ledClearAll(); }
  else { setDisplayBrightness(displayBrightness); ledClearAll(); for(int d=0;d<dspNumDev;d++)lc.clearDisplay(d); bootState=diagReturnToMenu?BOOT_DIAG_MENU:BOOT_RUNNING; if(diagReturnToMenu){menuInitialized=false;diagReturnToMenu=false;}Serial.println("DIAG:COMPLETE;"); }
}

bool diagCheckCombo() {
  bool combo=(inputState[0]&0x07)==0x07;
  if (bootState==BOOT_RUNNING&&combo&&!diagComboWasActive) { diagComboWasActive=true; bootState=BOOT_DIAG_MENU; diagStartTime=millis(); diagStage=0; menuInitialized=false; Serial.println("DIAG:COMBO_START;"); return true; }
  else if (!combo) diagComboWasActive=false;
  return false;
}

// ============================================================================
// EEPROM
// ============================================================================
void generateSerialNumber(char* out) {
  randomSeed(analogRead(A0)+analogRead(A1)+analogRead(A2)+analogRead(A3)+micros());
  const char hx[]="0123456789ABCDEF"; for(int i=0;i<8;i++)out[i]=hx[random(0,16)]; out[8]=0;
}
uint8_t calcCfgCs(uint8_t ver, const char* r8, const char* p8, const char* s9) {
  uint16_t sum=ver; for(int i=0;i<8;i++)sum+=(uint8_t)r8[i]; for(int i=0;i<8;i++)sum+=(uint8_t)p8[i]; for(int i=0;i<9;i++)sum+=(uint8_t)s9[i]; return(uint8_t)(sum&0xFF);
}
void saveHWInfo() {
  char rb[8]; int rl=strlen(CFG_AIRCRAFT_REG); for(int i=0;i<8;i++)rb[i]=(i<rl)?CFG_AIRCRAFT_REG[i]:' ';
  char pb[8]; int pl=strlen(CFG_PCB_VERSION); for(int i=0;i<8;i++)pb[i]=(i<pl)?CFG_PCB_VERSION[i]:' ';
  char sb[9]; int sl=strlen(CFG_SERIAL_NUMBER); for(int i=0;i<8;i++)sb[i]=(i<sl)?CFG_SERIAL_NUMBER[i]:' '; sb[8]=0;
  EEPROM.update(EEPROM_BASE_ADDR+0,(uint8_t)(EEPROM_MAGIC&0xFF)); EEPROM.update(EEPROM_BASE_ADDR+1,(uint8_t)((EEPROM_MAGIC>>8)&0xFF));
  EEPROM.update(EEPROM_BASE_ADDR+2,EEPROM_FORMAT_VERSION);
  for(int i=0;i<8;i++)EEPROM.update(EEPROM_BASE_ADDR+3+i,(uint8_t)rb[i]);
  for(int i=0;i<8;i++)EEPROM.update(EEPROM_BASE_ADDR+11+i,(uint8_t)pb[i]);
  for(int i=0;i<9;i++)EEPROM.update(EEPROM_BASE_ADDR+19+i,(uint8_t)sb[i]);
  uint8_t cs=calcCfgCs(EEPROM_FORMAT_VERSION,rb,pb,sb); EEPROM.update(EEPROM_BASE_ADDR+28,cs);
  Serial.print("CFG:SAVED:REG=");Serial.print(CFG_AIRCRAFT_REG);Serial.print(";PCB=");Serial.print(CFG_PCB_VERSION);Serial.print(";SN=");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);
}
void loadHWInfo() {
  uint16_t magic=(uint16_t)EEPROM.read(EEPROM_BASE_ADDR)|((uint16_t)EEPROM.read(EEPROM_BASE_ADDR+1)<<8);
  if (magic!=EEPROM_MAGIC) { Serial.println("CFG:EEPROM:MAGIC_MISSING;"); generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); Serial.print("CFG:SN:NEW:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER); return; }
  uint8_t ver=EEPROM.read(EEPROM_BASE_ADDR+2);
  char rb[9],pb[9]; for(int i=0;i<8;i++)rb[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+3+i); for(int i=0;i<8;i++)pb[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+11+i); rb[8]=0;pb[8]=0;
  if (ver<=2) { generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); Serial.print("CFG:SN:UPGRADE:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER); return; }
  if (ver!=EEPROM_FORMAT_VERSION) { Serial.print("CFG:EEPROM:VERSION_MISMATCH:");Serial.print(ver);Serial.print("!=");Serial.println(EEPROM_FORMAT_VERSION); generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); return; }
  char sb[10]; for(int i=0;i<9;i++)sb[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+19+i); sb[9]=0;
  uint8_t scs=EEPROM.read(EEPROM_BASE_ADDR+28); uint8_t cs=calcCfgCs(ver,rb,pb,sb);
  if (cs!=scs) { Serial.println("CFG:EEPROM:CRC_FAIL;"); generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); return; }
  int ri=0; for(int i=0;i<8;i++)if(rb[i]!=0&&rb[i]!=' ')CFG_AIRCRAFT_REG[ri++]=rb[i]; if(ri>0)CFG_AIRCRAFT_REG[ri]=0;
  int pi=0; for(int i=0;i<8;i++){if(pb[i]==0||pb[i]==' ')continue;CFG_PCB_VERSION[pi++]=pb[i];}if(pi>0)CFG_PCB_VERSION[pi]=0;
  int si=0; for(int i=0;i<8;i++)if(sb[i]!=0&&sb[i]!=' ')CFG_SERIAL_NUMBER[si++]=sb[i];CFG_SERIAL_NUMBER[si]=0;
  if (si==0) { generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo(); Serial.print("CFG:SN:NEW:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER); }
  else { Serial.print("CFG:SN:LOADED:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER); }
  refreshPCBVersionFlags();
  Serial.print("CFG:LOAD:REG=");Serial.print(CFG_AIRCRAFT_REG);Serial.print(";PCB=");Serial.print(CFG_PCB_VERSION);Serial.print(";SN=");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  wdt_disable(); Serial.begin(115200); Serial.flush(); delay(50);
  // Input shifters
  pinMode(inClk_HE,OUTPUT);pinMode(inLatch_HE,OUTPUT);pinMode(inData_HE,INPUT);
  pinMode(inClk_FI,OUTPUT);pinMode(inLatch_FI,OUTPUT);pinMode(inData_FI,INPUT);
  pinMode(inClk_EM,OUTPUT);pinMode(inLatch_EM,OUTPUT);pinMode(inData_EM,INPUT);
  pinMode(inClk_EL,OUTPUT);pinMode(inLatch_EL,OUTPUT);pinMode(inData_EL,INPUT);
  pinMode(inClk_AC,OUTPUT);pinMode(inLatch_AC,OUTPUT);pinMode(inData_AC,INPUT);
  // Direct LED pins (active-low outputs, start OFF=HIGH)
  for(uint8_t i=0;i<DLED_COUNT;i++){pinMode(DLED_PINS[i],OUTPUT);digitalWrite(DLED_PINS[i],HIGH);}
  // LED shifters
  pinMode(ledClk_HE,OUTPUT);pinMode(ledLatch_HE,OUTPUT);pinMode(ledData_HE,OUTPUT);
  pinMode(ledClk_FI,OUTPUT);pinMode(ledLatch_FI,OUTPUT);pinMode(ledData_FI,OUTPUT);
  pinMode(ledClk_EM,OUTPUT);pinMode(ledLatch_EM,OUTPUT);pinMode(ledData_EM,OUTPUT);
  pinMode(ledClk_EL,OUTPUT);pinMode(ledLatch_EL,OUTPUT);pinMode(ledData_EL,OUTPUT);
  pinMode(ledClk_AC,OUTPUT);pinMode(ledLatch_AC,OUTPUT);pinMode(ledData_AC,OUTPUT);
  // TLC5916 OE pins – must be PWM outputs, start LOW (outputs enabled)
  pinMode(OE_ANNU_PIN,OUTPUT);  digitalWrite(OE_ANNU_PIN,LOW);
  pinMode(OE_BACKL_PIN,OUTPUT); digitalWrite(OE_BACKL_PIN,LOW);
  // Analog
  for (uint8_t i=0;i<ANALOG_COUNT;i++) pinMode(ANA_PINS[i],INPUT);
  initDisplays();
  loadHWInfo();
  ledClearAll(); readAllInputs(); readAnalogFiltered();
  memcpy(lastInputState,inputState,IN_TOTAL_BYTES); memcpy(lastAnalogValues,analogValues,ANALOG_COUNT*sizeof(uint16_t));
  bootState=BOOT_RUNNING; forceSendNext=true; maybeSendIdentStartup();
  Serial.print("SETUP:COMPLETE;SN=");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now=millis();
  if (bootState==BOOT_DIAG_MENU) { processSerialTokensFromHost(); diagRunMenu(); return; }
  if (bootState==BOOT_DIAG_TEST) { diagRunFull(); return; }
  if (now<pauseUntil) return;
  if (bootMessageShown&&(now-bootMessageStart)>=BOOT_MSG_DURATION_MS) { bootMessageShown=false; for(int d=0;d<dspNumDev;d++)lc.clearDisplay(d); }
  processSerialTokensFromHost();
  if (now-lastLoopTs<LOOP_INTERVAL_MS) return;
  lastLoopTs=now;
  readAllInputs(); readAnalogFiltered();
  bool inCh=memcmp(inputState,lastInputState,IN_TOTAL_BYTES)!=0;
  if (inCh&&(now-lastDebounceTs)>=CFG_BUTTON_DEBOUNCE) { lastDebounceTs=now; memcpy(lastInputState,inputState,IN_TOTAL_BYTES); sendStatusImmediate(); }
  diagCheckCombo();
  if (analogChanged()) { for(uint8_t i=0;i<ANALOG_COUNT;i++){Serial.print("POT");Serial.print(i+1);Serial.print(":");Serial.print(analogValues[i]);Serial.print(";");}Serial.println(); }
  if (forceSendNext) sendStatusImmediate();
}
