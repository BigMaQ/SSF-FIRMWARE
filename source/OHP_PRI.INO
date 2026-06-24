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
// AC_IS: CLOCK D44, LATCH D41, DATA D43, AMOUNT 2
const int inClk_AC = 44; const int inLatch_AC = 41; const int inData_AC = 43; const uint8_t IN_CNT_AC = 2;

#define IN_SHIFTER_BYTES 11  // 2+1+4+2+2
#define IN_SINGLE_COUNT   6
#define IN_TOTAL_BYTES    (IN_SHIFTER_BYTES + 1)  // +1 for single inputs

// Single inputs: D9=HYD_ELEC_eng2_2_off, D10=HYD_ELEC_eng2_2_fault,
// D23=FIRE_apuagentdisch, D20=FIRE_apuagentsqd, D11=ANNU_BRT, D12=BACKLIGHT_BRT
const uint8_t SIN_PINS[IN_SINGLE_COUNT] = {9, 10, 23, 20, 11, 12};

// ============================================================================
// PIN DEFINITIONS – PRIMARY OUTPUT SHIFTER (74HC595)
// ============================================================================
// HYD_ELEC_LEDDRV: CLK D8, LATCH D5, DATA D7, AMOUNT 3
const int ledClk_HE  = 8;  const int ledLatch_HE  = 5;  const int ledData_HE  = 7;  const uint8_t LED_CNT_HE = 3;
// FIRE_LEDDRV:     CLK D16, LATCH D21, DATA D17, AMOUNT 2
const int ledClk_FI  = 16; const int ledLatch_FI  = 21; const int ledData_FI  = 17; const uint8_t LED_CNT_FI = 2;
// EMELEC_LEDDRV:   CLK D28, LATCH D27, DATA D29, AMOUNT 4
const int ledClk_EM  = 28; const int ledLatch_EM  = 27; const int ledData_EM  = 29; const uint8_t LED_CNT_EM = 4;
// ELEC_LEDDRV:     CLK D39, LATCH D38, DATA D40, AMOUNT 3
const int ledClk_EL  = 39; const int ledLatch_EL  = 38; const int ledData_EL  = 40; const uint8_t LED_CNT_EL = 3;
// AC_LEDDRV:       CLK D45, LATCH D46, DATA D48, AMOUNT 2
const int ledClk_AC  = 45; const int ledLatch_AC  = 46; const int ledData_AC  = 48; const uint8_t LED_CNT_AC = 2;

#define LED_TOTAL_BYTES 14  // 3+2+4+3+2

// ============================================================================
// PIN DEFINITIONS – MAX7219 DISPLAYS
// ============================================================================
// ELEC_DISP: CLK D33, DIN D32, LOAD D34, 2 devices
const int dspClkPin  = 33;
const int dspDinPin  = 32;
const int dspLoadPin = 34;
const int dspNumDev  = 2;   // DSP0=BAT1 Volt, DSP1=BAT2 Volt

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
const char* PANEL_IDENT = "OHP_PRI, v1.0 MAQ";
const char  FW_VERSION[] = "1.0";
const char  PANEL_SN_PREFIX[] = "OHP-";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

char CFG_SERIAL_NUMBER[10] = "";

// ============================================================================
// MAX7219 DISPLAY SETUP
// ============================================================================
LedControl lc = LedControl(dspDinPin, dspClkPin, dspLoadPin, dspNumDev);
const int DSP_BAT1 = 0;
const int DSP_BAT2 = 1;

uint8_t displayBrightness = 15;

// ============================================================================
// LED STATE
// ============================================================================
uint8_t desiredLedState[LED_TOTAL_BYTES] = {0};
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
const unsigned long MENU_INACTIVITY_MS = 10000;

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
void dspSetVal(int dev, int16_t v, bool blank0) {
  if (blank0 && v==0) { for(int d=0;d<6;d++) lc.setRow(dev,d,0); return; }
  char buf[7]; bool neg=(v<0); uint16_t av=neg?(uint16_t)(-v):(uint16_t)v;
  int8_t p=5; buf[6]=0;
  if (av==0) buf[p--]='0';
  else while(av>0&&p>=0) { buf[p--]='0'+(av%10); av/=10; }
  if (neg&&p>=0) buf[p--]='-';
  while(p>=0) buf[p--]=' ';
  for(int d=0;d<6;d++) lc.setRow(dev,d,charTo7Seg(buf[5-d]));
}
void setDisplayBrightness(uint8_t b) { displayBrightness=constrain(b,0,15);
  for(int d=0;d<dspNumDev;d++) lc.setIntensity(d,displayBrightness); }
void initDisplays() { for(int d=0;d<dspNumDev;d++){lc.shutdown(d,false);lc.setIntensity(d,displayBrightness);lc.clearDisplay(d);} }

// ============================================================================
// LED DRIVER (74HC595) – MULTI-GROUP
// ============================================================================
void shiftOutOne(uint8_t clk, uint8_t latch, uint8_t data, const uint8_t* bytes, uint8_t cnt) {
  digitalWrite(latch, LOW);
  for (int8_t b=cnt-1; b>=0; b--) shiftOut(data, clk, MSBFIRST, bytes[b]);
  digitalWrite(latch, HIGH);
}
void applyLEDOutputs() {
  uint8_t bi=0;
  shiftOutOne(ledClk_HE, ledLatch_HE, ledData_HE, &desiredLedState[bi], LED_CNT_HE); bi+=LED_CNT_HE;
  shiftOutOne(ledClk_FI, ledLatch_FI, ledData_FI, &desiredLedState[bi], LED_CNT_FI); bi+=LED_CNT_FI;
  shiftOutOne(ledClk_EM, ledLatch_EM, ledData_EM, &desiredLedState[bi], LED_CNT_EM); bi+=LED_CNT_EM;
  shiftOutOne(ledClk_EL, ledLatch_EL, ledData_EL, &desiredLedState[bi], LED_CNT_EL); bi+=LED_CNT_EL;
  shiftOutOne(ledClk_AC, ledLatch_AC, ledData_AC, &desiredLedState[bi], LED_CNT_AC); bi+=LED_CNT_AC;
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
  // Single inputs → IN11
  uint8_t s=0;
  for (uint8_t i=0; i<IN_SINGLE_COUNT; i++) if (digitalRead(SIN_PINS[i])==LOW) s|=(1<<i);
  inputState[IN_SHIFTER_BYTES]=s;
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
    if (d<-5 || d>5) { lastAnalogValues[i]=analogValues[i]; return true; }
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
  for (uint8_t i=0; i<IN_TOTAL_BYTES; i++) { bin8(b8,inputState[i]); Serial.print("IN");Serial.print(i);Serial.print(":");Serial.print(b8);Serial.print(";"); }
  for (uint8_t i=0; i<ANALOG_COUNT; i++) { Serial.print("POT");Serial.print(i);Serial.print(":");Serial.print(analogValues[i]);Serial.print(";"); }
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
      // LEDx
      if (kp[0]=='L'&&kp[1]=='E'&&kp[2]=='D') { uint8_t idx=atoi_s(kp+3); if (idx<LED_TOTAL_BYTES){desiredLedState[idx]=parseBin8(val);applyLEDOutputs();} }
      // STATE
      else if (strieq(kp,"STATE")) { hostOnline=(strcmp(val,"01")==0||strcmp(val,"1")==0); }
      // DSPx
      else if (strieq(kp,"DSP0")) { int v=atoi_s(val); if(v<0||val[0]=='-')for(int d=0;d<6;d++)lc.setRow(DSP_BAT1,d,0); else dspSetVal(DSP_BAT1,v,false); }
      else if (strieq(kp,"DSP1")) { int v=atoi_s(val); if(v<0||val[0]=='-')for(int d=0;d<6;d++)lc.setRow(DSP_BAT2,d,0); else dspSetVal(DSP_BAT2,v,false); }
      else if (strieq(kp,"DISP_BL")) { setDisplayBrightness(atoi_s(val)); }
      else if (strieq(kp,"REQ")) { forceSendNext=true; }
      else if (strieq(kp,"VER")) { sendIdentAndState(); }
      // CFG
      else if (strieq(kp,"CFG")) { char* cp=val;
        while(*cp){while(*cp==' '||*cp==';')cp++; if(!*cp)break; char* semi=strchr(cp,';'); if(semi)*semi=0;
          if(stripre(cp,"DEB"))CFG_BUTTON_DEBOUNCE=constrain(atoi_s(cp+3),0,99);
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
      if (strcmp(tokenUp,"EXIT")==0){settingsEnabled=false;Serial.println("EXIT:OK ;");continue;}
      if (strcmp(tokenUp,"PCB")==0){Serial.print("PCB:");char* sp=strchr(CFG_PCB_VERSION,' ');if(sp&&*(sp+1)){Serial.print("v");Serial.print(sp+1);}else Serial.print(CFG_PCB_VERSION);Serial.println(" ;");continue;}
      if (strcmp(tokenUp,"RESET")==0){Serial.println("RESET:OK ;");triggerSoftwareReset();continue;}
      if (strcmp(tokenUp,"REQ")==0){sendStatusImmediate();continue;}
      if (strcmp(tokenUp,"DIAG")==0){if(bootState==BOOT_RUNNING){bootState=BOOT_DIAG_MENU;diagStartTime=millis();diagStage=0;menuInitialized=false;Serial.println("DIAG:MENU_START;");}continue;}
      if (strchr(token,':')!=NULL){char lineBuf[64];snprintf(lineBuf,sizeof(lineBuf),"%s;",token);processIncomingLine(lineBuf);continue;}
    }
  }
}

// ============================================================================
// DIAG MODE
// ============================================================================
void diagRunMenu() {
  unsigned long now=millis();
  readAllInputs();
  if (!menuInitialized) { menuInitialized=true; menuIndex=0; lastMenuActivityTs=now; Serial.println("DIAG:MENU;"); ledClearAll(); return; }
  if ((now-lastMenuActivityTs)>=MENU_INACTIVITY_MS) { bootState=BOOT_RUNNING; menuInitialized=false; ledClearAll(); Serial.println("DIAG:EXIT_TIMEOUT;"); return; }
  // Blinkenlights
  static unsigned long lbTs=0; static uint8_t bp=0;
  if (now-lbTs>=100) { lbTs=now;
    if (bp<6) { if(bp%2==0)ledSetAll(0xFF); else ledClearAll(); bp++; }
    else if (bp<12) { ledClearAll(); bp++; } else bp=0;
  }
  // Check input for menu nav
  if (memcmp(inputState,lastInputState,IN_TOTAL_BYTES)!=0) {
    lastMenuActivityTs=now;
    if (inputState[0]!=lastInputState[0]) {
      uint8_t ch=inputState[0]^lastInputState[0];
      if (ch&0x01 && (inputState[0]&0x01)) {
        menuIndex=(menuIndex+1)%4; memcpy(lastInputState,inputState,IN_TOTAL_BYTES);
        Serial.print("DIAG:MENU:"); Serial.println(menuIndex);
        if (menuIndex==0) { Serial.println("DIAG:LED_WALK;"); diagStage=1; diagStartTime=now; }
        else if (menuIndex==1) { Serial.println("DIAG:INPUTS;"); for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("IN");Serial.print(i);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println(); }
        else if (menuIndex==2) { Serial.print("HEALTH:UPTIME=");Serial.print(millis()/1000);Serial.print("s;LASTERR=0;FREEMEM=");extern int __heap_start,*__brkval;int vv;Serial.print((int)&vv-(__brkval==0?(int)&__heap_start:(int)__brkval));Serial.print(";VERSION=");Serial.print(FW_VERSION);Serial.println(";"); }
        else if (menuIndex==3) { bootState=BOOT_RUNNING; menuInitialized=false; ledClearAll(); Serial.println("DIAG:EXIT;"); }
      }
    }
  }
  // LED walk
  if (diagStage==1) { int li=((now-diagStartTime)/200)%(LED_TOTAL_BYTES*8); ledClearAll(); uint8_t bi=li/8; desiredLedState[bi]=1<<(li%8); applyLEDOutputs(); }
}

void diagRunFull() {
  unsigned long e=millis()-diagStartTime;
  if (e<2000) { ledClearAll(); for(int d=0;d<dspNumDev;d++)lc.clearDisplay(d); if(e<500)Serial.println("DIAG:START;"); }
  else if (e<7000) { ledClearAll(); for(int d=0;d<dspNumDev;d++){int dig=((e-2000)/500)%10;int dpP=((e-2000)/500)%6;for(int i=0;i<6;i++){uint8_t p=charTo7Seg('0'+dig);if(i==dpP&&d==0)p|=0x80;lc.setRow(d,i,p);}} }
  else if (e<10000) { ledSetAll(0xFF); }
  else if (e<15000) { ledClearAll(); readAllInputs(); for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("DIAG:IN");Serial.print(i);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println(); }
  else if (e<20000) { int li=((e-15000)/200)%(LED_TOTAL_BYTES*8); ledClearAll(); uint8_t bi=li/8; desiredLedState[bi]=1<<(li%8); applyLEDOutputs(); }
  else if (e<25000) { ledClearAll(); if(e>=22000){int si=((e-22000)/300)%8;uint8_t sp=1<<si;for(int d=0;d<dspNumDev;d++)for(int i=0;i<6;i++)lc.setRow(d,i,sp);} }
  else if (e<30000) { ledSetAll(0xFF); int fp=(e-25000)%2000;uint8_t br=(fp<1000)?(fp/4):(255-((fp-1000)/4));for(int d=0;d<dspNumDev;d++)lc.setIntensity(d,br/17); }
  else if (e<33000) { ledClearAll(); }
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
  wdt_disable(); Serial.begin(115200);
  // Input shifters
  pinMode(inClk_HE,OUTPUT);pinMode(inLatch_HE,OUTPUT);pinMode(inData_HE,INPUT);
  pinMode(inClk_FI,OUTPUT);pinMode(inLatch_FI,OUTPUT);pinMode(inData_FI,INPUT);
  pinMode(inClk_EM,OUTPUT);pinMode(inLatch_EM,OUTPUT);pinMode(inData_EM,INPUT);
  pinMode(inClk_EL,OUTPUT);pinMode(inLatch_EL,OUTPUT);pinMode(inData_EL,INPUT);
  pinMode(inClk_AC,OUTPUT);pinMode(inLatch_AC,OUTPUT);pinMode(inData_AC,INPUT);
  // Single inputs
  for (uint8_t i=0;i<IN_SINGLE_COUNT;i++) pinMode(SIN_PINS[i],INPUT_PULLUP);
  // LED shifters
  pinMode(ledClk_HE,OUTPUT);pinMode(ledLatch_HE,OUTPUT);pinMode(ledData_HE,OUTPUT);
  pinMode(ledClk_FI,OUTPUT);pinMode(ledLatch_FI,OUTPUT);pinMode(ledData_FI,OUTPUT);
  pinMode(ledClk_EM,OUTPUT);pinMode(ledLatch_EM,OUTPUT);pinMode(ledData_EM,OUTPUT);
  pinMode(ledClk_EL,OUTPUT);pinMode(ledLatch_EL,OUTPUT);pinMode(ledData_EL,OUTPUT);
  pinMode(ledClk_AC,OUTPUT);pinMode(ledLatch_AC,OUTPUT);pinMode(ledData_AC,OUTPUT);
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
  if (bootState==BOOT_DIAG_MENU) { diagRunMenu(); return; }
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
  if (analogChanged()) { for(uint8_t i=0;i<ANALOG_COUNT;i++){Serial.print("POT");Serial.print(i);Serial.print(":");Serial.print(analogValues[i]);Serial.print(";");}Serial.println(); }
  if (forceSendNext) sendStatusImmediate();
}
