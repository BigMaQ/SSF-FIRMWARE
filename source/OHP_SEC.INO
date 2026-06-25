// OHP_SEC.INO – OHP Secondary Panel (Arduino MEGA 2560)
// (w) 2025 M. Quatember, OHP edition
// Controls ADIRS, LT, CARGO sections with BLTDRV LED drivers
#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================================
// PIN DEFINITIONS – SECONDARY INPUT SHIFTER (74HC165)
// ============================================================================
// ADIRS_IS: CLOCK D9, LATCH D8, DATA D10, AMOUNT 2
const int inClk_AD = 9;  const int inLatch_AD = 8;  const int inData_AD = 10; const uint8_t IN_CNT_AD = 2;
// LT_IS:    CLOCK D16, LATCH D15, DATA D17, AMOUNT 4
const int inClk_LT = 16; const int inLatch_LT = 15; const int inData_LT = 17; const uint8_t IN_CNT_LT = 4;
// CARGO_IS: CLOCK D39, LATCH D40, DATA D42, AMOUNT 3
const int inClk_CA = 39; const int inLatch_CA = 40; const int inData_CA = 42; const uint8_t IN_CNT_CA = 3;
// RES_IS (D30/D31/D32) – ignored

#define IN_SHIFTER_BYTES 9   // 2+4+3
#define IN_SINGLE_COUNT  2
#define IN_TOTAL_BYTES   (IN_SHIFTER_BYTES + 1)

// Single inputs: D21=LT_ModeSel, D22=CDV_SW
const uint8_t SIN_PINS[IN_SINGLE_COUNT] = {21, 22};

// ============================================================================
// PIN DEFINITIONS – SECONDARY OUTPUT SHIFTER (74HC595)
// ============================================================================
// BLTDRV:       CLK D6, LATCH D5, DATA D7, AMOUNT 4
const int ledClk_B1  = 6;  const int ledLatch_B1  = 5;  const int ledData_B1  = 7;  const uint8_t LED_CNT_B1 = 4;
// BLTDRV2:      CLK D3, LATCH D2, DATA D4, AMOUNT 4
const int ledClk_B2  = 3;  const int ledLatch_B2  = 2;  const int ledData_B2  = 4;  const uint8_t LED_CNT_B2 = 4;
// ADIRS_LEDDRV: CLK D12, LATCH D11, DATA D14, AMOUNT 2
const int ledClk_AD  = 12; const int ledLatch_AD  = 11; const int ledData_AD  = 14; const uint8_t LED_CNT_AD = 2;
// LT_LEDDRV:    CLK D19, LATCH D18, DATA D20, AMOUNT 3
const int ledClk_LT  = 19; const int ledLatch_LT  = 18; const int ledData_LT  = 20; const uint8_t LED_CNT_LT = 3;
// CARGO_LEDDRV: CLK D44, LATCH D41, DATA D43, AMOUNT 4
const int ledClk_CA  = 44; const int ledLatch_CA  = 41; const int ledData_CA  = 43; const uint8_t LED_CNT_CA = 4;
// RES_LEDDRV (D33/D34/D35) – ignored

#define LED_TOTAL_BYTES 17  // 4+4+2+3+4

// Single LEDs: D46=LED_GI1_CDV, D48=LED_GI2_CDV
const uint8_t SLED_PINS[2] = {46, 48};
#define SLED_COUNT 2

// TLC5916 OE – controlled entirely by Primary (ANNU_BRT / BACKLIGHT_BRT)

// ============================================================================
// PIN DEFINITIONS – ANALOG
// ============================================================================
#define ANALOG_COUNT 4
const uint8_t ANA_PINS[ANALOG_COUNT] = { A5, A6, A7, A8 };
// A5=CARGO_HEAT_FWD, A6=CARGO_HEAT_AFT, A7=LT_OVP_LT, A8=LT_LDG_ELEV

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char* PANEL_IDENT = "OHP_SEC, v1.1 MAQ";
const char  FW_VERSION[] = "1.1";
const char  PANEL_SN_PREFIX[] = "OHS-";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

char CFG_SERIAL_NUMBER[10] = "";

// ============================================================================
// LED STATE (shifters + single LEDs)
// ============================================================================
uint8_t desiredLedState[LED_TOTAL_BYTES + 1] = {0};  // +1 for single LED byte
uint8_t hwLedState[LED_TOTAL_BYTES + 1] = {0};

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
uint8_t  CFG_ANA_DEADBAND = 5;
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
const unsigned long MENU_INACTIVITY_MS = 60000;
// DIAG LED-Walk sub-state
int  diagWalkPhase = 0;       // 0=single-bit walk, 1=byte all-on
int  diagWalkByte = 0;        // current byte index
unsigned long diagWalkLastTs = 0;
// DIAG Show sub-state
int  showPhase = 0;           // 0=KnightRider, 1=Cascade, 2=Breathe, 3=Finale
int  showStep = 0;
int  showDir = 1;
unsigned long showLastTs = 0;
int  showRepeat = 1;

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
  for (int i=0;i<8;i++) out[i]=((b>>(7-i))&1)?'1':'0'; out[8]=0;
}
uint8_t parseBin8(const char* s) {
  uint8_t v=0; for(int i=0;s[i];i++){if(s[i]=='0'||s[i]=='1')v=(v<<1)|(s[i]=='1'?1:0);} return v;
}
static bool strieq(const char* a, const char* b) {
  while(*a&&*b){char ca=(*a>='a'&&*a<='z')?*a-32:*a;char cb=(*b>='a'&&*b<='z')?*b-32:*b;if(ca!=cb)return false;a++;b++;}return*a==*b;
}
static bool stripre(const char* a,const char* pf) {
  while(*pf){char ca=(*a>='a'&&*a<='z')?*a-32:*a;char cb=(*pf>='a'&&*pf<='z')?*pf-32:*pf;if(ca!=cb)return false;a++;pf++;}return true;
}
static int atoi_s(const char* s) {
  int v=0,sign=1;while(*s==' ')s++;if(*s=='-'){sign=-1;s++;}while(*s>='0'&&*s<='9'){v=v*10+(*s-'0');s++;}return v*sign;
}
void refreshPCBVersionFlags() {
  char norm[9];uint8_t j=0;
  for(uint8_t i=0;CFG_PCB_VERSION[i]&&j<8;i++){char c=CFG_PCB_VERSION[i];if(c==' ')continue;if(c>='a'&&c<='z')c-=32;norm[j++]=c;}
  norm[j]=0;CFG_PCB_IS_12=(norm[0]=='P'&&norm[1]=='C'&&norm[2]=='B'&&norm[3]=='1'&&norm[4]=='.'&&norm[5]=='2'&&norm[6]==0);
}
void triggerSoftwareReset(bool showMsg=true) {
  if(showMsg){Serial.println("RESET:OK ;");Serial.flush();delay(100);}wdt_enable(WDTO_15MS);while(true){}
}

// ============================================================================
// LED DRIVER (TLC5916) – pure shiftOut, no OE/PWM (Primary does brightness)
// ============================================================================
void shiftOutOne(uint8_t clk,uint8_t latch,uint8_t data,const uint8_t* bytes,uint8_t cnt) {
  digitalWrite(latch,LOW);for(int8_t b=cnt-1;b>=0;b--)shiftOut(data,clk,MSBFIRST,bytes[b]);digitalWrite(latch,HIGH);
}
void applyLEDOutputs() {
  uint8_t bi=0;
  // All TLC5916 groups – OE held LOW by Primary, no PWM on Secondary
  shiftOutOne(ledClk_B1,ledLatch_B1,ledData_B1,&desiredLedState[bi],LED_CNT_B1);bi+=LED_CNT_B1;
  shiftOutOne(ledClk_B2,ledLatch_B2,ledData_B2,&desiredLedState[bi],LED_CNT_B2);bi+=LED_CNT_B2;
  shiftOutOne(ledClk_AD,ledLatch_AD,ledData_AD,&desiredLedState[bi],LED_CNT_AD);bi+=LED_CNT_AD;
  shiftOutOne(ledClk_LT,ledLatch_LT,ledData_LT,&desiredLedState[bi],LED_CNT_LT);bi+=LED_CNT_LT;
  shiftOutOne(ledClk_CA,ledLatch_CA,ledData_CA,&desiredLedState[bi],LED_CNT_CA);bi+=LED_CNT_CA;
  // Single LEDs (from last byte)
  uint8_t sb=desiredLedState[LED_TOTAL_BYTES];
  for(uint8_t i=0;i<SLED_COUNT;i++) digitalWrite(SLED_PINS[i],(sb&(1<<i))?HIGH:LOW);
  memcpy(hwLedState,desiredLedState,LED_TOTAL_BYTES+1);
}
void ledClearAll() { memset(desiredLedState,0,LED_TOTAL_BYTES+1); applyLEDOutputs(); }
void ledSetAll(uint8_t v) { memset(desiredLedState,v,LED_TOTAL_BYTES+1); applyLEDOutputs(); }

// ============================================================================
// INPUT SHIFT REGISTERS (74HC165) – MULTI-GROUP
// ============================================================================
void readOneInGrp(uint8_t clk,uint8_t latch,uint8_t data,uint8_t* out,uint8_t cnt) {
  digitalWrite(latch,LOW);delayMicroseconds(20);digitalWrite(latch,HIGH);delayMicroseconds(20);
  for(uint8_t b=0;b<cnt;b++)out[b]=~shiftIn(data,clk,MSBFIRST);
  digitalWrite(latch,LOW);delayMicroseconds(20);digitalWrite(latch,HIGH);delayMicroseconds(20);
  for(int i=0;i<16;i++){digitalWrite(clk,LOW);delayMicroseconds(5);digitalWrite(clk,HIGH);delayMicroseconds(5);}
}
void readAllInputs() {
  uint8_t bi=0;
  readOneInGrp(inClk_AD,inLatch_AD,inData_AD,&inputState[bi],IN_CNT_AD); bi+=IN_CNT_AD;
  readOneInGrp(inClk_LT,inLatch_LT,inData_LT,&inputState[bi],IN_CNT_LT);bi+=IN_CNT_LT;
  readOneInGrp(inClk_CA,inLatch_CA,inData_CA,&inputState[bi],IN_CNT_CA);bi+=IN_CNT_CA;
  uint8_t s=0;for(uint8_t i=0;i<IN_SINGLE_COUNT;i++)if(digitalRead(SIN_PINS[i])==LOW)s|=(1<<i);
  inputState[IN_SHIFTER_BYTES]=s;
}

// ============================================================================
// ANALOG INPUTS
// ============================================================================
void readAnalogFiltered() {
  for(uint8_t i=0;i<ANALOG_COUNT;i++){
    uint16_t raw=analogRead(ANA_PINS[i]);anaFltBuf[i][anaFltIdx[i]]=raw;anaFltIdx[i]=(anaFltIdx[i]+1)%ANA_FLT_SZ;
    uint32_t sum=0;for(uint8_t j=0;j<ANA_FLT_SZ;j++)sum+=anaFltBuf[i][j];analogValues[i]=(uint16_t)(sum/ANA_FLT_SZ);
  }
}
bool analogChanged() {
  for(uint8_t i=0;i<ANALOG_COUNT;i++){int16_t d=(int16_t)analogValues[i]-(int16_t)lastAnalogValues[i];int16_t db=(int16_t)CFG_ANA_DEADBAND;if(d<-db||d>db){lastAnalogValues[i]=analogValues[i];return true;}}return false;
}

// ============================================================================
// SERIAL IDENT / STATE
// ============================================================================
void sendIdentAndState() {
  Serial.print("IDENT:");Serial.print(PANEL_IDENT);
  if(CFG_SERIAL_NUMBER[0]!=0){Serial.print(", SN:");Serial.print(PANEL_SN_PREFIX);Serial.print(CFG_SERIAL_NUMBER);}
  Serial.print(";STATE:RUNNING;REG:");Serial.print(CFG_AIRCRAFT_REG);Serial.println(";");
}
void maybeSendIdentStartup() { if(!identSentOnStart){delay(150);sendIdentAndState();identSentOnStart=true;} }

// ============================================================================
// STATUS SENDING
// ============================================================================
void sendStatusImmediate() {
  char b8[9];
  for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){bin8(b8,inputState[i]);Serial.print("IN");Serial.print(i+1);Serial.print(":");Serial.print(b8);Serial.print(";");}
  for(uint8_t i=0;i<ANALOG_COUNT;i++){Serial.print("POT");Serial.print(i+1);Serial.print(":");Serial.print(analogValues[i]);Serial.print(";");}
  Serial.println();lastSendTs=millis();forceSendNext=false;
}

// ============================================================================
// SERIAL COMMAND PARSING
// ============================================================================
void processIncomingLine(const char* line) {
  char token[32];int tlen=0;
  for(int i=0;;i++){
    char c=line[i];
    if(c==';'||c==0){
      if(tlen==0){if(c==0)break;else continue;}
      token[tlen]=0;tlen=0;int end=strlen(token)-1;while(end>=0&&token[end]==' ')token[end--]=0;
      char* tok=token;while(*tok==' ')tok++;if(*tok==0){if(c==0)break;else continue;}
      char* colon=strchr(tok,':');
      if(!colon){
        if(strieq(tok,"VER")||strieq(tok,"VERSION")||strieq(tok,"IDENT")){sendIdentAndState();}
        else if(strieq(tok,"REQ")){sendStatusImmediate();}
        else if(strieq(tok,"DIAG")&&bootState==BOOT_RUNNING){bootState=BOOT_DIAG_MENU;diagStartTime=millis();diagStage=0;menuInitialized=false;Serial.println("DIAG:MENU_START;");}
        else if(stripre(tok,"SET ")){char* sc=tok+4;while(*sc==' ')sc++;if(stripre(sc,"EXIT")){settingsEnabled=false;Serial.println("SET EXIT:OK ;");}else if(stripre(sc,"WRITE")&&settingsEnabled){saveHWInfo();settingsEnabled=false;Serial.println("WRITE:OK ;");triggerSoftwareReset(false);}else if(stripre(sc,"WRITE")){Serial.println("WRITE:LOCKED ;");}}
        if(c==0)break;else continue;
      }
      *colon=0;char* key=tok;char* val=colon+1;
      int ke=strlen(key)-1;while(ke>=0&&key[ke]==' ')key[ke--]=0;char* kp=key;while(*kp==' ')kp++;
      while(*val==' ')val++;int ve=strlen(val)-1;while(ve>=0&&val[ve]==' ')val[ve--]=0;
      // LEDx (case-insensitive, 0..LED_TOTAL_BYTES incl single LEDs) – silent apply, matching RMP behavior
      if(stripre(kp,"LED")){int idx=atoi_s(kp+3)-1;if(idx>=0&&idx<=LED_TOTAL_BYTES){desiredLedState[idx]=parseBin8(val);applyLEDOutputs();}}
      else if(strieq(kp,"STATE")){hostOnline=(strcmp(val,"01")==0||strcmp(val,"1")==0);}
      else if(strieq(kp,"DISP_BL")){} // accepted but no-op (no displays)
      else if(strieq(kp,"REQ")){forceSendNext=true;}
      else if(strieq(kp,"VER")){sendIdentAndState();}
      else if(strieq(kp,"CFG")){char* cp=val;
        while(*cp){while(*cp==' '||*cp==';')cp++;if(!*cp)break;char* semi=strchr(cp,';');if(semi)*semi=0;
          if(stripre(cp,"DEB"))CFG_BUTTON_DEBOUNCE=constrain(atoi_s(cp+3),0,99);
          else if(stripre(cp,"DEAD"))CFG_ANA_DEADBAND=constrain(atoi_s(cp+4),1,100);
          else if(stripre(cp,"LED"))CFG_LED_REFRESH=constrain(atoi_s(cp+3),100,10000);
          else if(stripre(cp,"DSP"))CFG_DISPLAY_REFRESH=constrain(atoi_s(cp+3),100,10000);
          else if(stripre(cp,"REG:")){strncpy(CFG_AIRCRAFT_REG,cp+4,8);CFG_AIRCRAFT_REG[8]=0;}
          else if(stripre(cp,"SN:")){strncpy(CFG_SERIAL_NUMBER,cp+3,8);CFG_SERIAL_NUMBER[8]=0;}
          if(semi)*semi=';';cp=semi?semi+1:cp+strlen(cp);
        }
      }
      else if(stripre(tok,"SET ")){char* sc=tok+4;while(*sc==' ')sc++;
        if(stripre(sc,"ENA")){if(strcmp(val,SETTINGS_PIN)==0){settingsEnabled=true;Serial.println("SET ENA:OK ;");}else Serial.println("SET ENA:FAIL ;");}
        else if(stripre(sc,"EXIT")){settingsEnabled=false;Serial.println("SET EXIT:OK ;");}
        else if((stripre(sc,"WRI")||stripre(sc,"WRITE"))&&settingsEnabled){saveHWInfo();settingsEnabled=false;Serial.println("WRITE:OK ;");triggerSoftwareReset(false);}
        else if(stripre(sc,"FW")&&settingsEnabled){char* vs=val;while(*vs==' ')vs++;char* dot=strchr(vs,'.');if(dot&&dot>vs){*dot=0;int ma=atoi_s(vs);int mi=atoi_s(dot+1);*dot='.';if(ma>=1&&ma<=9&&mi>=0&&mi<=9){snprintf(CFG_PCB_VERSION,9,"PCb %d.%d",ma,mi);refreshPCBVersionFlags();Serial.println("SET FW:OK ;");}else Serial.println("SET FW:RANGE ;");}else Serial.println("SET FW:FORMAT ;");}
        else if(stripre(sc,"ACID")&&settingsEnabled){char* idt=val;while(*idt==' ')idt++;int il=strlen(idt);if(il>=3&&il<=8){strncpy(CFG_AIRCRAFT_REG,idt,8);CFG_AIRCRAFT_REG[8]=0;Serial.println("SET ACID:OK ;");}else Serial.println("SET ACID:FORMAT ;");}
        else if(stripre(sc,"SN")&&settingsEnabled){char* sv=val;while(*sv==' ')sv++;int sl=strlen(sv);bool vok=(sl==8);for(int i2=0;i2<sl&&vok;i2++){char ch=sv[i2];if(!((ch>='0'&&ch<='9')||(ch>='A'&&ch<='F')||(ch>='a'&&ch<='f')))vok=false;}if(vok){for(int i2=0;i2<8;i2++)CFG_SERIAL_NUMBER[i2]=(sv[i2]>='a'&&sv[i2]<='f')?sv[i2]-32:sv[i2];CFG_SERIAL_NUMBER[8]=0;Serial.print("SET SN:OK:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);}else Serial.println("SET SN:FORMAT ;");}
      }
      if(c==0)break;
    }else{if(tlen<31)token[tlen++]=c;}
  }
}

void processSerialTokensFromHost() {
  while(Serial.available()){
    char c=Serial.read();if(c=='\r')continue;if(c=='\n')c=';';
    int alen=strlen(serialAccum);if(alen<63){serialAccum[alen]=c;serialAccum[alen+1]=0;}
    char* semi;
    while((semi=strchr(serialAccum,';'))!=NULL){
      *semi=0;char token[32];char* tp=serialAccum;while(*tp==' ')tp++;
      int ti;for(ti=0;tp[ti]&&ti<31;ti++)token[ti]=tp[ti];token[ti]=0;
      int tend=ti-1;while(tend>=0&&token[tend]==' ')token[tend--]=0;
      int remaining=strlen(semi+1);memmove(serialAccum,semi+1,remaining+1);
      if(token[0]==0)continue;
      char tokenUp[32];for(ti=0;token[ti]&&ti<31;ti++)tokenUp[ti]=(token[ti]>='a'&&token[ti]<='z')?token[ti]-32:token[ti];tokenUp[ti]=0;
      if(strcmp(tokenUp,"VER")==0||strcmp(tokenUp,"VERSION")==0||strcmp(tokenUp,"IDENT")==0){sendIdentAndState();identSentOnStart=true;bootMessageShown=true;bootMessageStart=millis();continue;}
      if(strcmp(tokenUp,"EXIT")==0){if(bootState==BOOT_DIAG_MENU){bootState=BOOT_RUNNING;menuInitialized=false;diagStage=0;ledClearAll();Serial.println("DIAG:EXIT;");}else{settingsEnabled=false;Serial.println("EXIT:OK ;");}continue;}
      if(strcmp(tokenUp,"PCB")==0){Serial.print("PCB:");char* sp=strchr(CFG_PCB_VERSION,' ');if(sp&&*(sp+1)){Serial.print("v");Serial.print(sp+1);}else Serial.print(CFG_PCB_VERSION);Serial.println(" ;");continue;}
      if(strcmp(tokenUp,"RESET")==0){Serial.println("RESET:OK ;");triggerSoftwareReset();continue;}
      if(strcmp(tokenUp,"REQ")==0){sendStatusImmediate();continue;}
      if (strcmp(tokenUp,"DIAG")==0){bootState=BOOT_DIAG_MENU;diagStartTime=millis();diagStage=0;menuInitialized=false;ledClearAll();Serial.println("DIAG:MENU;");continue;}
      // ---- DIAG serial navigation (works only in DIAG mode) ----
      if (bootState==BOOT_DIAG_MENU) {
        lastMenuActivityTs=millis();
        if (strcmp(tokenUp,"NEXT")==0)  { menuIndex=(menuIndex+1)%4; Serial.print("DIAG:MENU:");Serial.println(menuIndex); if(menuIndex==0){Serial.println("DIAG:LED_WALK;");diagStage=1;diagWalkPhase=0;diagWalkByte=0;diagWalkLastTs=millis();ledClearAll();} else if(menuIndex==1){Serial.println("DIAG:INPUTS;"); diagStage=0; readAllInputs(); for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("IN");Serial.print(i+1);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println();} else if(menuIndex==2){diagStage=0; Serial.print("HEALTH:UPTIME=");Serial.print(millis()/1000);Serial.print("s;LASTERR=0;FREEMEM=");extern int __heap_start,*__brkval;int vv;Serial.print((int)&vv-(__brkval==0?(int)&__heap_start:(int)__brkval));Serial.print(";VERSION=");Serial.print(FW_VERSION);Serial.println(";");Serial.println("DIAG:ALL_OK;");} else if(menuIndex==3){bootState=BOOT_RUNNING;menuInitialized=false;diagStage=0;ledClearAll();Serial.println("DIAG:EXIT;");} continue;}
        if (strcmp(tokenUp,"LED")==0)  { menuIndex=0; Serial.println("DIAG:LED_WALK;");diagStage=1;diagWalkPhase=0;diagWalkByte=0;diagWalkLastTs=millis();ledClearAll();continue;}
        if (strcmp(tokenUp,"IN")==0)   { diagStage=0; readAllInputs(); for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("IN");Serial.print(i+1);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println();continue;}
        if (strcmp(tokenUp,"HEALTH")==0){ diagStage=0; Serial.print("HEALTH:UPTIME=");Serial.print(millis()/1000);Serial.print("s;LASTERR=0;FREEMEM=");extern int __heap_start,*__brkval;int vv;Serial.print((int)&vv-(__brkval==0?(int)&__heap_start:(int)__brkval));Serial.print(";VERSION=");Serial.print(FW_VERSION);Serial.println(";");continue;}
        if (strcmp(tokenUp,"SHOW")==0||strncmp(tokenUp,"SHOW:",5)==0) {
          showRepeat=1;
          if(tokenUp[4]==':'){int n=atoi_s(tokenUp+5);if(n>=1&&n<=9)showRepeat=n;}
          menuIndex=4; Serial.print("DIAG:SHOW_START x");Serial.print(showRepeat);Serial.println(";");
          diagStage=3;showPhase=0;showStep=0;showDir=1;showLastTs=millis();ledClearAll();continue;
        }
      }
      if(strchr(token,':')!=NULL){char lineBuf[64];snprintf(lineBuf,sizeof(lineBuf),"%s;",token);processIncomingLine(lineBuf);continue;}
    }
  }
}

// ============================================================================
// DIAG MODE – Two-phase LED walk: single-bit, then byte-all-on
// ============================================================================
void diagLedWalkStep(unsigned long now) {
  const uint16_t totalBits = (uint16_t)(LED_TOTAL_BYTES+1) * 8;  // +1 for single LED byte
  const unsigned long SINGLE_MS = 150;
  const unsigned long BYTE_MS   = 800;
  lastMenuActivityTs=now;

  if (diagWalkPhase == 0) {
    if (now - diagWalkLastTs >= SINGLE_MS) {
      diagWalkLastTs = now;
      memset(desiredLedState, 0, LED_TOTAL_BYTES+1);  // clear buffer only, no flicker
      if (diagWalkByte < totalBits) {
        uint8_t bi = diagWalkByte / 8;
        desiredLedState[bi] = 1 << (diagWalkByte % 8);
        applyLEDOutputs();  // single apply
        Serial.print("DIAG:WALK_BIT="); Serial.print(diagWalkByte);
        Serial.print("  LED"); Serial.print(bi+1);
        Serial.print("."); Serial.println(diagWalkByte % 8);
      } else { applyLEDOutputs(); }
      diagWalkByte++;
      if (diagWalkByte > totalBits) {
        diagWalkPhase = 1; diagWalkByte = 0; diagWalkLastTs = now;
        Serial.println("DIAG:WALK_BYTE_ALL_ON_START;");
      }
    }
  } else {
    if (now - diagWalkLastTs >= BYTE_MS) {
      diagWalkLastTs = now;
      memset(desiredLedState, 0, LED_TOTAL_BYTES+1);
      if (diagWalkByte <= LED_TOTAL_BYTES) {
        desiredLedState[diagWalkByte] = 0xFF;
        applyLEDOutputs();
        Serial.print("DIAG:ALL_ON LED"); Serial.print(diagWalkByte+1);
        Serial.println(" = 11111111");
      } else { applyLEDOutputs(); }
      diagWalkByte++;
      if (diagWalkByte > LED_TOTAL_BYTES) {
        ledClearAll();
        diagStage = 0;
        Serial.println("DIAG:LED_WALK_DONE;");
      }
    }
  }
}

void diagRunMenu() {
  unsigned long now=millis();readAllInputs();
  if(!menuInitialized){menuInitialized=true;menuIndex=0;lastMenuActivityTs=now;Serial.println("DIAG:MENU;");ledClearAll();return;}
  if(diagStage==0&&(now-lastMenuActivityTs)>=MENU_INACTIVITY_MS){bootState=BOOT_RUNNING;menuInitialized=false;ledClearAll();Serial.println("DIAG:EXIT_TIMEOUT;");return;}
  if(memcmp(inputState,lastInputState,IN_TOTAL_BYTES)!=0){lastMenuActivityTs=now;
    if(inputState[0]!=lastInputState[0]){uint8_t ch=inputState[0]^lastInputState[0];
      if(ch&0x01&&(inputState[0]&0x01)){menuIndex=(menuIndex+1)%4;memcpy(lastInputState,inputState,IN_TOTAL_BYTES);Serial.print("DIAG:MENU:");Serial.println(menuIndex);
        if(menuIndex==0){Serial.println("DIAG:LED_WALK;");diagStage=1;diagStartTime=now;diagWalkPhase=0;diagWalkByte=0;diagWalkLastTs=now;ledClearAll();}
        else if(menuIndex==1){Serial.println("DIAG:INPUTS;");for(uint8_t i=0;i<IN_TOTAL_BYTES;i++){char b8[9];bin8(b8,inputState[i]);Serial.print("IN");Serial.print(i+1);Serial.print("=");Serial.print(b8);Serial.print(";");}Serial.println();}
        else if(menuIndex==2){Serial.print("HEALTH:UPTIME=");Serial.print(millis()/1000);Serial.print("s;LASTERR=0;FREEMEM=");extern int __heap_start,*__brkval;int vv;Serial.print((int)&vv-(__brkval==0?(int)&__heap_start:(int)__brkval));Serial.print(";VERSION=");Serial.print(FW_VERSION);Serial.println(";");}
        else if(menuIndex==3){bootState=BOOT_RUNNING;menuInitialized=false;ledClearAll();Serial.println("DIAG:EXIT;");}
      }
    }
  }
  if(diagStage==1){diagLedWalkStep(now);}
  if(diagStage==3){diagShowStep(now);}
}

// ============================================================================
// DIAG SHOW – Animation sequence for demo / video
// ============================================================================
void diagShowStep(unsigned long now) {
  const uint16_t totalBits = (uint16_t)(LED_TOTAL_BYTES+1) * 8;
  lastMenuActivityTs=now;

  if (showPhase == 0) {
    // ---- Phase 0: Knight Rider sweep (left→right→left, 1 pass) ----
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
      memset(desiredLedState, 0, LED_TOTAL_BYTES+1);
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
      int totalBytes = LED_TOTAL_BYTES+1;
      if (showStep < totalBytes) {
        desiredLedState[showStep] = 0xFF;
        applyLEDOutputs();
      } else if (showStep < totalBytes*2) {
        int bi = showStep - totalBytes;
        desiredLedState[bi] = 0x00;
        applyLEDOutputs();
      } else {
        showPhase=2; showStep=0; showLastTs=now;
        ledClearAll();
        Serial.println("DIAG:SHOW_BREATHE;");
        return;
      }
      showStep++;
    }
  } else if (showPhase == 2) {
    // ---- Phase 2: "Breathing" – accelerating pulse ----
    if (now - showLastTs >= (unsigned long)(220 - showStep*2)) {
      showLastTs = now;
      if (showStep >= 100) {
        showPhase=3; showStep=0; showLastTs=now;
        ledSetAll(0xFF);
        Serial.println("DIAG:SHOW_FINALE;");
        return;
      }
      if (showStep & 1) ledClearAll(); else ledSetAll(0xFF);
      showStep++;
    }
  } else if (showPhase == 3) {
    // ---- Phase 3: All ON → hold 2.5s → done or repeat ----
    if (now - showLastTs >= 2500) {
      showRepeat--;
      if (showRepeat > 0) {
        showPhase=0; showStep=0; showLastTs=now;
        ledClearAll();
        Serial.print("DIAG:SHOW_REPEAT ");Serial.print(showRepeat);Serial.println(";");
      } else {
        ledClearAll();
        diagStage = 0;
        showPhase = 0; showStep = 0; showRepeat = 1;
        Serial.println("DIAG:SHOW_DONE;");
      }
    }
  }
}

bool diagCheckCombo() {
  bool combo=(inputState[0]&0x07)==0x07;
  if(bootState==BOOT_RUNNING&&combo&&!diagComboWasActive){diagComboWasActive=true;bootState=BOOT_DIAG_MENU;diagStartTime=millis();diagStage=0;menuInitialized=false;Serial.println("DIAG:COMBO_START;");return true;}
  else if(!combo)diagComboWasActive=false;return false;
}

// ============================================================================
// EEPROM
// ============================================================================
void generateSerialNumber(char* out) {
  randomSeed(analogRead(A0)+analogRead(A1)+analogRead(A2)+analogRead(A3)+micros());
  const char hx[]="0123456789ABCDEF";for(int i=0;i<8;i++)out[i]=hx[random(0,16)];out[8]=0;
}
uint8_t calcCfgCs(uint8_t ver,const char* r8,const char* p8,const char* s9) {
  uint16_t sum=ver;for(int i=0;i<8;i++)sum+=(uint8_t)r8[i];for(int i=0;i<8;i++)sum+=(uint8_t)p8[i];for(int i=0;i<9;i++)sum+=(uint8_t)s9[i];return(uint8_t)(sum&0xFF);
}
void saveHWInfo() {
  char rb[8];int rl=strlen(CFG_AIRCRAFT_REG);for(int i=0;i<8;i++)rb[i]=(i<rl)?CFG_AIRCRAFT_REG[i]:' ';
  char pb[8];int pl=strlen(CFG_PCB_VERSION);for(int i=0;i<8;i++)pb[i]=(i<pl)?CFG_PCB_VERSION[i]:' ';
  char sb[9];int sl=strlen(CFG_SERIAL_NUMBER);for(int i=0;i<8;i++)sb[i]=(i<sl)?CFG_SERIAL_NUMBER[i]:' ';sb[8]=0;
  EEPROM.update(EEPROM_BASE_ADDR+0,(uint8_t)(EEPROM_MAGIC&0xFF));EEPROM.update(EEPROM_BASE_ADDR+1,(uint8_t)((EEPROM_MAGIC>>8)&0xFF));
  EEPROM.update(EEPROM_BASE_ADDR+2,EEPROM_FORMAT_VERSION);
  for(int i=0;i<8;i++)EEPROM.update(EEPROM_BASE_ADDR+3+i,(uint8_t)rb[i]);
  for(int i=0;i<8;i++)EEPROM.update(EEPROM_BASE_ADDR+11+i,(uint8_t)pb[i]);
  for(int i=0;i<9;i++)EEPROM.update(EEPROM_BASE_ADDR+19+i,(uint8_t)sb[i]);
  uint8_t cs=calcCfgCs(EEPROM_FORMAT_VERSION,rb,pb,sb);EEPROM.update(EEPROM_BASE_ADDR+28,cs);
  Serial.print("CFG:SAVED:REG=");Serial.print(CFG_AIRCRAFT_REG);Serial.print(";PCB=");Serial.print(CFG_PCB_VERSION);Serial.print(";SN=");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);
}
void loadHWInfo() {
  uint16_t magic=(uint16_t)EEPROM.read(EEPROM_BASE_ADDR)|((uint16_t)EEPROM.read(EEPROM_BASE_ADDR+1)<<8);
  if(magic!=EEPROM_MAGIC){Serial.println("CFG:EEPROM:MAGIC_MISSING;");generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();Serial.print("CFG:SN:NEW:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);return;}
  uint8_t ver=EEPROM.read(EEPROM_BASE_ADDR+2);
  char rb[9],pb[9];for(int i=0;i<8;i++)rb[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+3+i);for(int i=0;i<8;i++)pb[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+11+i);rb[8]=0;pb[8]=0;
  if(ver<=2){generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();Serial.print("CFG:SN:UPGRADE:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);return;}
  if(ver!=EEPROM_FORMAT_VERSION){Serial.print("CFG:EEPROM:VERSION_MISMATCH:");Serial.print(ver);Serial.print("!=");Serial.println(EEPROM_FORMAT_VERSION);generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();return;}
  char sb[10];for(int i=0;i<9;i++)sb[i]=(char)EEPROM.read(EEPROM_BASE_ADDR+19+i);sb[9]=0;
  uint8_t scs=EEPROM.read(EEPROM_BASE_ADDR+28);uint8_t cs=calcCfgCs(ver,rb,pb,sb);
  if(cs!=scs){Serial.println("CFG:EEPROM:CRC_FAIL;");generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();return;}
  int ri=0;for(int i=0;i<8;i++)if(rb[i]!=0&&rb[i]!=' ')CFG_AIRCRAFT_REG[ri++]=rb[i];if(ri>0)CFG_AIRCRAFT_REG[ri]=0;
  int pi=0;for(int i=0;i<8;i++){if(pb[i]==0||pb[i]==' ')continue;CFG_PCB_VERSION[pi++]=pb[i];}if(pi>0)CFG_PCB_VERSION[pi]=0;
  int si=0;for(int i=0;i<8;i++)if(sb[i]!=0&&sb[i]!=' ')CFG_SERIAL_NUMBER[si++]=sb[i];CFG_SERIAL_NUMBER[si]=0;
  if(si==0){generateSerialNumber(CFG_SERIAL_NUMBER);saveHWInfo();Serial.print("CFG:SN:NEW:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);}
  else{Serial.print("CFG:SN:LOADED:");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);}
  refreshPCBVersionFlags();
  Serial.print("CFG:LOAD:REG=");Serial.print(CFG_AIRCRAFT_REG);Serial.print(";PCB=");Serial.print(CFG_PCB_VERSION);Serial.print(";SN=");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  wdt_disable();Serial.begin(115200);Serial.flush();delay(50);
  pinMode(inClk_AD,OUTPUT);pinMode(inLatch_AD,OUTPUT);pinMode(inData_AD,INPUT);
  pinMode(inClk_LT,OUTPUT);pinMode(inLatch_LT,OUTPUT);pinMode(inData_LT,INPUT);
  pinMode(inClk_CA,OUTPUT);pinMode(inLatch_CA,OUTPUT);pinMode(inData_CA,INPUT);
  for(uint8_t i=0;i<IN_SINGLE_COUNT;i++)pinMode(SIN_PINS[i],INPUT_PULLUP);
  pinMode(ledClk_B1,OUTPUT);pinMode(ledLatch_B1,OUTPUT);pinMode(ledData_B1,OUTPUT);
  pinMode(ledClk_B2,OUTPUT);pinMode(ledLatch_B2,OUTPUT);pinMode(ledData_B2,OUTPUT);
  pinMode(ledClk_AD,OUTPUT);pinMode(ledLatch_AD,OUTPUT);pinMode(ledData_AD,OUTPUT);
  pinMode(ledClk_LT,OUTPUT);pinMode(ledLatch_LT,OUTPUT);pinMode(ledData_LT,OUTPUT);
  pinMode(ledClk_CA,OUTPUT);pinMode(ledLatch_CA,OUTPUT);pinMode(ledData_CA,OUTPUT);
  for(uint8_t i=0;i<SLED_COUNT;i++)pinMode(SLED_PINS[i],OUTPUT);
  for(uint8_t i=0;i<ANALOG_COUNT;i++)pinMode(ANA_PINS[i],INPUT);
  loadHWInfo();
  ledClearAll();readAllInputs();readAnalogFiltered();
  memcpy(lastInputState,inputState,IN_TOTAL_BYTES);memcpy(lastAnalogValues,analogValues,ANALOG_COUNT*sizeof(uint16_t));
  bootState=BOOT_RUNNING;forceSendNext=true;maybeSendIdentStartup();
  Serial.print("SETUP:COMPLETE;SN=");Serial.print(PANEL_SN_PREFIX);Serial.println(CFG_SERIAL_NUMBER);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now=millis();
  if(bootState==BOOT_DIAG_MENU){processSerialTokensFromHost();diagRunMenu();return;}
  if(bootState==BOOT_DIAG_TEST){/*no full seq for secondary*/ bootState=BOOT_RUNNING;return;}
  if(now<pauseUntil)return;
  processSerialTokensFromHost();
  if(now-lastLoopTs<LOOP_INTERVAL_MS)return;lastLoopTs=now;
  readAllInputs();readAnalogFiltered();
  bool inCh=memcmp(inputState,lastInputState,IN_TOTAL_BYTES)!=0;
  if(inCh&&(now-lastDebounceTs)>=CFG_BUTTON_DEBOUNCE){lastDebounceTs=now;memcpy(lastInputState,inputState,IN_TOTAL_BYTES);sendStatusImmediate();}
  diagCheckCombo();
  if(analogChanged()){for(uint8_t i=0;i<ANALOG_COUNT;i++){Serial.print("POT");Serial.print(i+1);Serial.print(":");Serial.print(analogValues[i]);Serial.print(";");}Serial.println();}
  if(forceSendNext)sendStatusImmediate();
}
