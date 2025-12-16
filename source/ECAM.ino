// ECAM Control Script, (w) 2025 M. Quatember
// Basierend auf ACP.ino — gleicher Ein-/Ausgabe-Format

// --- Pins ---
const int ledLatchPin = 2;   // LED Driver Latch
const int ledClockPin = 3;   // LED Driver Clock
const int ledDataPin  = 4;   // LED Driver Data
const int LED_DRIVERS = 4;   // Anzahl der durchgereichten LED-Driver (Bytes)

const int annunPWM     = 5;
const int backlightPWM = 6;

// Input Shifter (HC165) - 4 bytes (IN1..IN4)
const int isLatchPin = 14;
const int isClockPin = 15;
const int isDataPin  = 16;

// Analog inputs
const int analogUpperPin = A9;
const int analogLowerPin = A10;

// --- Panel identification ---
const char* PANEL_IDENT = "ECAM, v1.0 MAQ";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// --- LED state (desired + hardware) ---
uint8_t desiredLedChain[LED_DRIVERS];
uint8_t hwLedChain[LED_DRIVERS];
uint8_t desiredAnLevel = 0;
uint8_t desiredBlLevel = 0;
uint8_t hwAnLevel = 0;
uint8_t hwBlLevel = 0;

// --- Runtime buffers / flags ---
String serialAccum = "";

// --- Smoothing / Brightness / Timing (same semantics as ACP) ---
int SMO_THRESHOLD = 10;
int SMO_SAMPLES = 4;
int SMO_DELAY_US = 300;

int BL_LEVEL = 0;
int AN_LEVEL = 0;

unsigned long lastLoopTs      = 0;
const unsigned long LOOP_INTERVAL_MS      = 10;
unsigned long lastSendTs      = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
unsigned long lastDebounceTs  = 0;
const unsigned long DEBOUNCE_MS = 12;

// Analog smoothing buffers (2 channels)
int analogVals[2];
int lastAnalogVals[2];
int analogBuffer[2][8];
int analogBufferIdx[2];

// Input shifter
uint8_t inputState[4];
uint8_t lastInputState[4];

// Combo / DIAG (kept for parity)
unsigned long comboStartTime = 0;
int comboStage = 0;
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

// --- LED HW write (4 driver chain) ---
void shiftOutLEDChain(uint8_t *chain) {
  digitalWrite(ledLatchPin, LOW);
  // shift out highest index first so chain[0] ends up closest to MCU
  for (int i = LED_DRIVERS - 1; i >= 0; --i) {
    shiftOut(ledDataPin, ledClockPin, MSBFIRST, chain[i]);
  }
  digitalWrite(ledLatchPin, HIGH);
}

void applyLEDOutputs() {
  shiftOutLEDChain(desiredLedChain);
  analogWrite(backlightPWM, desiredBlLevel);
  analogWrite(annunPWM, desiredAnLevel);

  for (int i = 0; i < LED_DRIVERS; ++i) hwLedChain[i] = desiredLedChain[i];
  hwAnLevel = desiredAnLevel;
  hwBlLevel = desiredBlLevel;

  BL_LEVEL = desiredBlLevel;
  AN_LEVEL = desiredAnLevel;
}

void setLEDState(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t bl, uint8_t an, bool allowDuringCombo = false) {
  if (comboStage != 0 && !allowDuringCombo) return;
  desiredLedChain[0] = b0;
  desiredLedChain[1] = b1;
  desiredLedChain[2] = b2;
  desiredLedChain[3] = b3;
  desiredBlLevel = bl;
  desiredAnLevel = an;
  applyLEDOutputs();
}

// --- Analog read helper ---
int readAnalogRaw(int pin) {
  delayMicroseconds(SMO_DELAY_US);
  return constrain(analogRead(pin), 0, 1023);
}

int getAnalogSmoothed(int channel, int rawValue) {
  analogBuffer[channel][analogBufferIdx[channel]] = rawValue;
  analogBufferIdx[channel] = (analogBufferIdx[channel] + 1) % SMO_SAMPLES;
  int sum = 0;
  for (int i = 0; i < SMO_SAMPLES; i++) sum += analogBuffer[channel][i];
  return sum / SMO_SAMPLES;
}

// --- Shift Register read (HC165) - 4 bytes ---
void readShiftRegisters(uint8_t out[4]) {
  // Latch
  digitalWrite(isLatchPin, LOW);
  delayMicroseconds(20);
  digitalWrite(isLatchPin, HIGH);
  delayMicroseconds(20);

  // Read 4 bytes
  uint8_t raw[4];
  for (int i = 0; i < 4; ++i) raw[i] = shiftIn(isDataPin, isClockPin, MSBFIRST);
  // invert to match button-pressed == 1 semantics
  for (int i = 0; i < 4; ++i) out[i] = ~raw[i];

  // Timing-fix pulses to stabilize HC165 (same approach as ACP)
  digitalWrite(isLatchPin, LOW);
  delayMicroseconds(20);
  digitalWrite(isLatchPin, HIGH);
  delayMicroseconds(20);
  for (int i = 0; i < 32; i++) {
    digitalWrite(isClockPin, LOW);
    delayMicroseconds(5);
    digitalWrite(isClockPin, HIGH);
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

    if (key.equalsIgnoreCase("LED1")) desiredLedChain[0] = parseBin8(val);
    else if (key.equalsIgnoreCase("LED2")) desiredLedChain[1] = parseBin8(val);
    else if (key.equalsIgnoreCase("LED3")) desiredLedChain[2] = parseBin8(val);
    else if (key.equalsIgnoreCase("LED4")) desiredLedChain[3] = parseBin8(val);
    else if (key.equalsIgnoreCase("BL")) { BL_LEVEL = constrain(val.toInt(),0,255); desiredBlLevel = BL_LEVEL; }
    else if (key.equalsIgnoreCase("AN")) { AN_LEVEL = constrain(val.toInt(),0,255); desiredAnLevel = AN_LEVEL; }
    else if (key.equalsIgnoreCase("SMO_THR")) { SMO_THRESHOLD = constrain(val.toInt(), 1, 100); }
    else if (key.equalsIgnoreCase("SMO_SAM")) { SMO_SAMPLES = constrain(val.toInt(), 1, 8); }
    else if (key.equalsIgnoreCase("SMO_DLY")) { SMO_DELAY_US = constrain(val.toInt(), 50, 1000); }
    else if (key.equalsIgnoreCase("REQ")) forceSendNext = true;
    else if (key.equalsIgnoreCase("VER")) sendIdentAndState();
  }
  // Apply LED outputs after processing tokens that may change them
  applyLEDOutputs();
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

      String tokenUp = token;
      tokenUp.toUpperCase();

      if (tokenUp == "VER" || tokenUp == "VERSION") {
        sendIdentAndState();
        identSentOnStart = true;
        pauseUntil = millis() + 200;
        continue;
      }
      if (tokenUp == "REQ") {
        // immediate status
        forceSendNext = true;
        continue;
      }
      if (token.indexOf(':') >= 0) {
        processIncomingLine(token + ";");
        continue;
      }

      // otherwise ignore
    }
  }
}

// --- Status ---
void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;
  Serial.print("UP:"); Serial.print(decPad4(analogVals[0])); Serial.print(";");
  Serial.print("LOW:"); Serial.print(decPad4(analogVals[1])); Serial.print(";");
  // Inputs IN1..IN4
  for (int i = 0; i < 4; ++i) {
    Serial.print("IN"); Serial.print(i+1); Serial.print(":"); Serial.print(bin8(inputState[i])); Serial.print(";");
  }
  Serial.println();
  lastSendTs = now; forceSendNext = false;
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  pinMode(ledLatchPin, OUTPUT); pinMode(ledClockPin, OUTPUT); pinMode(ledDataPin, OUTPUT);
  pinMode(annunPWM, OUTPUT); pinMode(backlightPWM, OUTPUT);
  pinMode(isDataPin, INPUT); pinMode(isClockPin, OUTPUT); pinMode(isLatchPin, OUTPUT);

  analogWrite(backlightPWM,0); analogWrite(annunPWM,0);

  for (int i = 0; i < LED_DRIVERS; ++i) { desiredLedChain[i] = 0x00; hwLedChain[i] = 0x00; }
  for (int i = 0; i < 2; ++i) { analogVals[i] = 0; lastAnalogVals[i] = -9999; analogBufferIdx[i] = 0; for (int j=0;j<8;j++) analogBuffer[i][j]=0; }
  for (int i = 0; i < 4; ++i) { inputState[i] = 0xFF; lastInputState[i] = 0xFF; }

  // Startup LED blink pattern across 4 drivers
  setLEDState(0xAA,0xAA,0xAA,0xAA,200,200,true); delay(120);
  setLEDState(0x55,0x55,0x55,0x55,200,200,true); delay(120);
  setLEDState(0xFF,0xFF,0xFF,0xFF,0,0,true); delay(120);
  setLEDState(0x00,0x00,0x00,0x00,0,0,true); delay(120);

  sendIdentAndState(); identSentOnStart = true; forceSendNext = true;
}

// --- Main Loop ---
void loop() {
  unsigned long now = millis();
  if (now < pauseUntil) return;
  processSerialTokensFromHost();
  if(now - lastLoopTs < LOOP_INTERVAL_MS) return; lastLoopTs=now;

  // --- Read analogs ---
  int rawUpper = readAnalogRaw(analogUpperPin);
  int rawLower = readAnalogRaw(analogLowerPin);
  analogVals[0] = getAnalogSmoothed(0, rawUpper);
  analogVals[1] = getAnalogSmoothed(1, rawLower);

  // --- Read ShiftRegisters (4 bytes) ---
  readShiftRegisters(inputState);

  // --- Input changes (debounce across any of 4 bytes) ---
  bool inputChanged = false;
  for (int i=0;i<4;i++) if (inputState[i] != lastInputState[i]) { inputChanged = true; break; }
  if (inputChanged) {
    if (now - lastDebounceTs >= DEBOUNCE_MS) {
      lastDebounceTs = now;
      for (int i=0;i<4;i++) lastInputState[i] = inputState[i];
      sendStatus();
    }
  }

  // --- Analog changes ---
  bool analogChanged = false;
  for (int i=0;i<2;i++) {
    if (abs(analogVals[i] - lastAnalogVals[i]) >= SMO_THRESHOLD) { analogChanged=true; lastAnalogVals[i]=analogVals[i]; }
  }
  if (analogChanged) sendStatus();

  // --- DIAG Combo ---
  // Same behaviour as in ACP but use the 4-byte hotkey requested by user:
  // IN1:00010000; IN2:10000000; IN3:00000001; IN4:00000000
  bool comboNow = (inputState[0] == 0b00010000) && (inputState[1] == 0b10000000)
                  && (inputState[2] == 0b00000001) && (inputState[3] == 0b00000000);
  if (comboNow && comboStage == 0) {
    comboStage = 1;
    comboStartTime = now;
    Serial.println("DIAG START");
    setLEDState(0xFF, 0xFF, 0xFF, 0xFF, 200, 200, true);
  }
  if (comboStage == 1 && (now - comboStartTime) >= 15000) {
    comboStage = 0;
    Serial.println("DIAG END");
    setLEDState(0x00, 0x00, 0x00, 0x00, 0, 0, true);
  }
}
