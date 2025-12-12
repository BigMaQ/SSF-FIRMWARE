// ACP Control Script, (w) 2025 M. Quatember
// Sauberer, human-readable Sketch mit HC165 Off-by-One Fix und Poti-Optimierung

// --- Pins ---
const int muxSelectPins[] = {A0, A1, A2, A3}; // S0..S3
const int muxOutputPin = A6;

const int inputDataPin  = 16;
const int inputClockPin = 14;
const int inputLatchPin = 15;

const int ledLatchPin = 2;
const int ledClockPin = 5;
const int ledDataPin  = 7;
const int backlightPWM = 3;
const int annunPWM     = 6;

// --- Panel identification ---
const char* PANEL_IDENT = "ACP 1 CPT, v1.1 MAQ";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// --- LED state (desired + hardware) ---
uint8_t desiredLedBacklight = 0x00;
uint8_t desiredLedAnnun     = 0x00;
uint8_t desiredBlLevel      = 0;
uint8_t desiredAnLevel      = 0;

uint8_t hwLedBacklight = 0x00;
uint8_t hwLedAnnun     = 0x00;
uint8_t hwBlLevel      = 0;
uint8_t hwAnLevel      = 0;

// --- Runtime buffers / flags ---
String serialAccum = "";

// ============================================================================
// UNIVERSAL PARAMETERS (same across all firmware versions)
// ============================================================================
// These parameters should be synchronized via Python at connect time
// Commands: BL:xxx (backlight), AN:xxx (annunciator), DISP_BL:xx (display)
// Smoothing: SMO_THR (threshold), SMO_SAM (samples), SMO_DLY (delay)

// --- Smoothing Parameters (configurable via serial) ---
// SMO_THR: Minimum change threshold for analog inputs (MUX/poti)
// SMO_SAM: Number of samples for moving average (1-8)
// SMO_DLY: Delay in microseconds after analog select (50-1000)
int SMO_THRESHOLD = 10;           // Minimum change to trigger send (default: 10)
int SMO_SAMPLES = 4;              // Moving average window size (default: 4)
int SMO_DELAY_US = 300;           // Delay after MUX/analog select (microseconds, default: 300)

// --- Brightness Parameters (configurable via serial) ---
// BL:xxx    Backlight brightness (0-255, PWM)
// AN:xxx    Annunciator brightness (0-255, PWM) - optional per panel
// DISP_BL:xx Display brightness (0-15, MAX7219) - optional per panel
int BL_LEVEL = 0;                 // Backlight current level
int AN_LEVEL = 0;                 // Annunciator current level (if applicable)
int DISP_BL_LEVEL = 15;           // Display brightness current level (if applicable)

// --- Timing / throttles / debounce ---
unsigned long lastLoopTs      = 0;
const unsigned long LOOP_INTERVAL_MS      = 10;
unsigned long lastSendTs      = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
unsigned long lastDebounceTs  = 0;
const unsigned long DEBOUNCE_MS = 12;
int muxVals[16];
int lastMuxVals[16];
int muxBuffer[16][8];           // Ring buffer for smoothing (max 8 samples)
int muxBufferIdx[16];           // Index into ring buffer per channel
uint8_t inputState1 = 0xFF;
uint8_t inputState2 = 0xFF;
uint8_t lastInputState1 = 0xFF;
uint8_t lastInputState2 = 0xFF;

// --- Combo / DIAG ---
unsigned long comboStartTime = 0;
int comboStage = 0; // 0 idle, 1 running
bool forceSendNext = false;

// --- Helpers ---
String decPad4(int v) {
  char buf[8];
  v = constrain(v, 0, 1023);
  sprintf(buf, "%04d", v);
  return String(buf);
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

// --- LED HW write ---
void shiftOutLEDs(uint8_t backlightBits, uint8_t annunBits) {
  digitalWrite(ledLatchPin, LOW);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, annunBits);
  shiftOut(ledDataPin, ledClockPin, MSBFIRST, backlightBits);
  digitalWrite(ledLatchPin, HIGH);
}

void applyLEDOutputs() {
  shiftOutLEDs(desiredLedBacklight, desiredLedAnnun);
  analogWrite(backlightPWM, desiredBlLevel);
  analogWrite(annunPWM, desiredAnLevel);

  hwLedBacklight = desiredLedBacklight;
  hwLedAnnun     = desiredLedAnnun;
  hwBlLevel      = desiredBlLevel;
  hwAnLevel      = desiredAnLevel;
  
  // Update universal brightness cache
  BL_LEVEL = desiredBlLevel;
  AN_LEVEL = desiredAnLevel;
}

void setLEDState(uint8_t backBits, uint8_t annBits, uint8_t bl, uint8_t an, bool allowDuringCombo = false) {
  if (comboStage != 0 && !allowDuringCombo) return;
  desiredLedBacklight = backBits;
  desiredLedAnnun     = annBits;
  desiredBlLevel      = bl;
  desiredAnLevel      = an;
  applyLEDOutputs();
}

// --- MUX read helpers ---
int readMuxChannelRaw(int idx) {
  for (int b = 0; b < 4; ++b) digitalWrite(muxSelectPins[b], (idx >> b) & 1);
  delayMicroseconds(SMO_DELAY_US);
  return constrain(analogRead(muxOutputPin), 0, 1023);
}

// --- MUX Smoothing (moving average) ---
int getMuxSmoothed(int channel, int rawValue) {
  // Add raw value to ring buffer
  muxBuffer[channel][muxBufferIdx[channel]] = rawValue;
  muxBufferIdx[channel] = (muxBufferIdx[channel] + 1) % SMO_SAMPLES;
  
  // Calculate average of all samples in buffer
  int sum = 0;
  for (int i = 0; i < SMO_SAMPLES; i++) {
    sum += muxBuffer[channel][i];
  }
  return sum / SMO_SAMPLES;
}

// --- Shift Register read (HC165) with Off-by-One Fix ---
void readShiftRegisters(uint8_t &in1, uint8_t &in2) {
    // Latch
    digitalWrite(inputLatchPin, LOW);
    delayMicroseconds(20);
    digitalWrite(inputLatchPin, HIGH);
    delayMicroseconds(20);

    // Read shift registers
    byte raw1 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);
    byte raw2 = shiftIn(inputDataPin, inputClockPin, MSBFIRST);
    in1 = ~raw1;
    in2 = ~raw2;

    // Clock pulses to stabilize HC165 (Timing-Fix)
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

// --- Serial IDENT / state ---
void sendIdentAndState() {
  Serial.print("IDENT:"); Serial.print(PANEL_IDENT);
  Serial.print(";STATE:RUNNING;");
  Serial.println();
}



// --- Serial command parsing ---
void processIncomingLine(const String &line) {
  int start = 0;
  while (true) {
    int sep = line.indexOf(';', start);
    if (sep < 0) break;
    String token = line.substring(start, sep); start = sep + 1;
    token.trim();
    if (token.length() == 0) continue;

    if (token.indexOf(':') < 0) {
      if (token.equalsIgnoreCase("VER") || token.equalsIgnoreCase("VERSION")) { sendIdentAndState(); continue; }
      if (token.equalsIgnoreCase("REQ")) { forceSendNext = true; continue; }
      continue;
    }

    int colon = token.indexOf(':');
    if (colon < 0) continue;
    String key = token.substring(0, colon);
    String val = token.substring(colon + 1);
    key.trim(); val.trim();

    if (key.equalsIgnoreCase("LED1")) setLEDState(parseBin8(val), desiredLedAnnun, desiredBlLevel, desiredAnLevel);
    else if (key.equalsIgnoreCase("LED2")) setLEDState(desiredLedBacklight, parseBin8(val), desiredBlLevel, desiredAnLevel);
    else if (key.equalsIgnoreCase("BL")) { BL_LEVEL = constrain(val.toInt(),0,255); setLEDState(desiredLedBacklight, desiredLedAnnun, BL_LEVEL, desiredAnLevel); }
    else if (key.equalsIgnoreCase("AN")) { AN_LEVEL = constrain(val.toInt(),0,255); setLEDState(desiredLedBacklight, desiredLedAnnun, desiredBlLevel, AN_LEVEL); }
    else if (key.equalsIgnoreCase("DISP_BL")) { DISP_BL_LEVEL = constrain(val.toInt(),0,15); }
    else if (key.equalsIgnoreCase("SMO_THR")) { SMO_THRESHOLD = constrain(val.toInt(), 1, 100); }
    else if (key.equalsIgnoreCase("SMO_SAM")) { SMO_SAMPLES = constrain(val.toInt(), 1, 8); }
    else if (key.equalsIgnoreCase("SMO_DLY")) { SMO_DELAY_US = constrain(val.toInt(), 50, 1000); }
    else if (key.equalsIgnoreCase("REQ")) forceSendNext = true;
    else if (key.equalsIgnoreCase("VER")) sendIdentAndState();
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
      String token = serialAccum.substring(0, idx); token.trim();
      serialAccum = serialAccum.substring(idx + 1);
      if (token.length() == 0) continue;

      // direct single-word commands (no colon) we want to handle immediately
      String tokenUp = token;
      tokenUp.toUpperCase();

      if (tokenUp == "VER" || tokenUp == "VERSION") {
        sendIdentAndState();
        identSentOnStart = true;
        pauseUntil = millis() + 200;
        continue;
      }
      if (tokenUp == "REQ") {
        sendStatus();
        continue;
      }
      if (token.indexOf(':') >= 0) {
        processIncomingLine(token + ";");
        continue;
      }

    // otherwise ignore unknown single-word token

    }
  }
}

// --- Status ---
void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;
  for (int i=0;i<16;i++){Serial.print("MUX");Serial.print(i);Serial.print(":");Serial.print(decPad4(muxVals[i]));Serial.print(";");}
  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";IN2:"); Serial.println(bin8(inputState2));
  lastSendTs = now; forceSendNext = false;
}



// --- Setup ---
void setup() {
  Serial.begin(115200);
  for(int i=0;i<4;i++) pinMode(muxSelectPins[i], OUTPUT);
  pinMode(muxOutputPin, INPUT);
  pinMode(inputDataPin, INPUT); pinMode(inputClockPin, OUTPUT); pinMode(inputLatchPin, OUTPUT);
  pinMode(ledLatchPin, OUTPUT); pinMode(ledClockPin, OUTPUT); pinMode(ledDataPin, OUTPUT);
  pinMode(backlightPWM, OUTPUT); pinMode(annunPWM, OUTPUT);
  analogWrite(backlightPWM,0); analogWrite(annunPWM,0);
  for(int i=0;i<16;i++){muxVals[i]=0; lastMuxVals[i]=-9999; muxBufferIdx[i]=0;}
  lastInputState1=0xFF; lastInputState2=0xFF;
  // Startup LED blink
  setLEDState(0xAA,0xAA,200,200,true); delay(150);
  setLEDState(0x55,0x55,200,200,true); delay(150);
  setLEDState(0xFF,0xFF,0,0,true); delay(150);
  setLEDState(0x00,0x00,0,0,true);
  delay(150);
  sendIdentAndState();
  identSentOnStart = true;
  forceSendNext = true;
}

// --- Main Loop ---
void loop() {
  unsigned long now = millis();
  if (now < pauseUntil) return;
  processSerialTokensFromHost();
  if(now - lastLoopTs < LOOP_INTERVAL_MS) return; lastLoopTs=now;

  // --- Read MUX ---
  for(int i=0;i<16;i++){
    int rawVal = readMuxChannelRaw(i);
    muxVals[i] = getMuxSmoothed(i, rawVal);
  }

  // --- Read ShiftRegisters (HC165) ---
  readShiftRegisters(inputState1,inputState2);

  // --- Button Changes ---
  if(inputState1 != lastInputState1 || inputState2 != lastInputState2){
    if(now - lastDebounceTs >= DEBOUNCE_MS){
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      sendStatus();
    }
  }

  // --- MUX changes (using SMO_THRESHOLD parameter) ---
  bool muxChanged = false;
  for(int i=0;i<16;i++){if(abs(muxVals[i]-lastMuxVals[i])>=SMO_THRESHOLD){muxChanged=true;lastMuxVals[i]=muxVals[i];}}
  if(muxChanged) sendStatus();

  // --- DIAG Combo ---
  bool comboNow = (inputState1==0b01000001)&&(inputState2==0b00001000);
  if(comboNow && comboStage==0){ comboStage=1; comboStartTime=now; Serial.println("DIAG START"); setLEDState(0xFF,0xFF,200,200,true);}
  if(comboStage==1 && (now-comboStartTime)>=15000){ comboStage=0; Serial.println("DIAG END"); setLEDState(0x00,0x00,0,0,true);}
}
