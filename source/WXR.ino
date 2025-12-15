// WXR Weather Radar Panel, (c) 2025 M. Quatember
// Based on ACP.ino — simplified for Weather Radar panel

// --- Identification ---
const char* PANEL_IDENT = "WXR, v1.0 MAQ";
bool identSentOnStart = false;
unsigned long pauseUntil = 0;

// --- Pins (user-specified) ---
const int pinInputMAN  = 2;   // Digital multiscan MAN
const int pinBacklightPWM = 3; // PWM backlight
const int ledLatchPin = 4;
const int ledClockPin = 5;
const int ledDataPin  = 6;
const int pinInputAUTO = 7;   // Digital multiscan AUTO
const int pinAnalogTILT = A8; // Poti TILT
const int pinAnalogGAIN = A9; // Poti GAIN
const int pinInputMAP  = 10;
const int pinInputTURB = 16;
const int pinInputWXP = 14;   // WX+T
const int pinInputWX  = 15;
const int pinInputSYS1 = 18;
const int pinInputSYS2 = 19;
const int pinInputPWS_AUTO = 20;
const int pinInputGCS_AUTO = 21;

// --- LED state (desired + hw) ---
uint8_t desiredLedBacklight = 0x00;
uint8_t desiredLedAnnun     = 0x00;
uint8_t desiredBlLevel      = 0;
uint8_t desiredAnLevel      = 0;

uint8_t hwLedBacklight = 0x00;
uint8_t hwLedAnnun     = 0x00;
uint8_t hwBlLevel      = 0;
uint8_t hwAnLevel      = 0;

// --- Smoothing / brightness params ---
int SMO_THRESHOLD = 10;
int SMO_SAMPLES = 4;
int SMO_DELAY_US = 300;

int BL_LEVEL = 0;
int AN_LEVEL = 0;
int DISP_BL_LEVEL = 15;

// --- Timing / debounce ---
unsigned long lastLoopTs = 0;
const unsigned long LOOP_INTERVAL_MS = 10;
unsigned long lastSendTs = 0;
const unsigned long SEND_MIN_INTERVAL_MS = 10;
unsigned long lastDebounceTs = 0;
const unsigned long DEBOUNCE_MS = 12;

// --- Analog buffers for TILT and GAIN ---
int analogPins[2] = {pinAnalogTILT, pinAnalogGAIN};
int analogVals[2];
int lastAnalogVals[2];
int analogBuffer[2][8];
int analogBufferIdx[2];

// --- Digital inputs packed into two bytes ---
int digitalPins[] = { pinInputMAN, pinInputAUTO, pinInputMAP, pinInputTURB, pinInputWXP, pinInputWX, pinInputSYS1, pinInputSYS2, pinInputPWS_AUTO, pinInputGCS_AUTO };
uint8_t inputState1 = 0xFF;
uint8_t inputState2 = 0xFF;
uint8_t lastInputState1 = 0xFF;
uint8_t lastInputState2 = 0xFF;

// --- Runtime ---
String serialAccum = "";
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
  analogWrite(pinBacklightPWM, desiredBlLevel);

  hwLedBacklight = desiredLedBacklight;
  hwLedAnnun     = desiredLedAnnun;
  hwBlLevel      = desiredBlLevel;
  hwAnLevel      = desiredAnLevel;

  BL_LEVEL = desiredBlLevel;
  AN_LEVEL = desiredAnLevel;
}

void setLEDState(uint8_t backBits, uint8_t annBits, uint8_t bl, uint8_t an, bool allowDuringCombo = false) {
  desiredLedBacklight = backBits;
  desiredLedAnnun     = annBits;
  desiredBlLevel      = bl;
  desiredAnLevel      = an;
  applyLEDOutputs();
}

// --- Analog read + smoothing ---
int readAnalogRaw(int pin) {
  delayMicroseconds(SMO_DELAY_US);
  return constrain(analogRead(pin), 0, 1023);
}

int getAnalogSmoothed(int idx, int rawValue) {
  analogBuffer[idx][analogBufferIdx[idx]] = rawValue;
  analogBufferIdx[idx] = (analogBufferIdx[idx] + 1) % SMO_SAMPLES;
  int sum = 0;
  for (int i = 0; i < SMO_SAMPLES; i++) sum += analogBuffer[idx][i];
  return sum / SMO_SAMPLES;
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

      String tokenUp = token;
      tokenUp.toUpperCase();

      if (tokenUp == "VER" || tokenUp == "VERSION") {
        sendIdentAndState();
        identSentOnStart = true;
        pauseUntil = millis() + 200;
        continue;
      }
      // PINTEST removed - diagnostics complete
      if (tokenUp == "REQ") {
        // immediate status send
        forceSendNext = true;
        sendStatus();
        continue;
      }
      if (token.indexOf(':') >= 0) {
        processIncomingLine(token + ";");
        continue;
      }
    }
  }
}

// --- Status ---
void sendStatus() {
  unsigned long now = millis();
  if (!forceSendNext && (now - lastSendTs) < SEND_MIN_INTERVAL_MS) return;

  // Send analog channels as MUX0 and MUX1 for compatibility
  Serial.print("MUX0:"); Serial.print(decPad4(analogVals[0])); Serial.print(";");
  Serial.print("MUX1:"); Serial.print(decPad4(analogVals[1])); Serial.print(";");

  Serial.print("IN1:"); Serial.print(bin8(inputState1)); Serial.print(";");
  Serial.print("IN2:"); Serial.println(bin8(inputState2));

  lastSendTs = now; forceSendNext = false;
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  pinMode(pinInputMAN, INPUT_PULLUP);
  pinMode(pinBacklightPWM, OUTPUT);
  pinMode(ledLatchPin, OUTPUT); pinMode(ledClockPin, OUTPUT); pinMode(ledDataPin, OUTPUT);
  pinMode(pinInputAUTO, INPUT_PULLUP);
  for (int i=0;i<10;i++) pinMode(digitalPins[i], INPUT_PULLUP);

  analogWrite(pinBacklightPWM,0);
  for (int i=0;i<2;i++){ analogVals[i]=0; lastAnalogVals[i]=-9999; analogBufferIdx[i]=0; for(int j=0;j<8;j++) analogBuffer[i][j]=0; }
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
  if (now - lastLoopTs < LOOP_INTERVAL_MS) return; lastLoopTs = now;

  // --- Read analogs ---
  for (int i=0;i<2;i++){
    int raw = readAnalogRaw(analogPins[i]);
    analogVals[i] = getAnalogSmoothed(i, raw);
  }

  // --- Read digitals and pack into two bytes (LSB first) ---
  uint16_t packed = 0;
  for (int i=0;i<10;i++){
    int v = digitalRead(digitalPins[i]);
    // assume INPUT_PULLUP = active low -> invert so '1' means active
    int bit = (v == LOW) ? 1 : 0;
    packed |= (bit << i);
  }
  // For direct GPIO inputs using INPUT_PULLUP we already map active (LOW) -> bit=1 above.
  // Do NOT invert here; keep '1' == active/on.
  inputState1 = (packed & 0xFF);
  inputState2 = ((packed >> 8) & 0xFF);

  // --- Digital changes (debounce) ---
  if (inputState1 != lastInputState1 || inputState2 != lastInputState2) {
    if (now - lastDebounceTs >= DEBOUNCE_MS) {
      lastDebounceTs = now;
      lastInputState1 = inputState1;
      lastInputState2 = inputState2;
      sendStatus();
    }
  }

  // --- Analog changes ---
  bool analogChanged = false;
  for (int i=0;i<2;i++){
    if (abs(analogVals[i] - lastAnalogVals[i]) >= SMO_THRESHOLD) { analogChanged = true; lastAnalogVals[i]=analogVals[i]; }
  }
  if (analogChanged) sendStatus();
}
