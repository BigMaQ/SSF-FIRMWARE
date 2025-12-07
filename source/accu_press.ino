// ACCU Press Modul - Race-safe LED + User Brightness + Safe Fade
// Pro Micro friendly (ATmega32U4)
// Servos: D6 (ACCU HYD PRS), D3 (BRK L), D5 (BRK R)
// Sink driver: latch=14 clock=15 data=16 (8-bit, lower nibble 0..3 used)
// PWM brightness on PIN_BRIGHTNESS = 9 (A9 on Pro Micro)

#include <Servo.h>
#if defined(__AVR__)
  #include <avr/wdt.h>
#endif

// ==== Pin mapping ====
const uint8_t PIN_SERVO_ACCU_HYD = 6; // D6
const uint8_t PIN_SERVO_BRK_L    = 3; // D3
const uint8_t PIN_SERVO_BRK_R    = 5; // D5

const uint8_t PIN_LATCH = 14; // shift latch
const uint8_t PIN_CLOCK = 15; // shift clock
const uint8_t PIN_DATA  = 16; // shift data

const uint8_t PIN_BRIGHTNESS = 9; // PWM (A9 on Pro Micro)

// ==== Globals ====
Servo servoAccuHyd;
Servo servoBrkL;
Servo servoBrkR;

int servoOffsetAccuHyd = 0;
int servoOffsetBrkL    = 0;
int servoOffsetBrkR    = 0;

uint8_t ledState8 = 0x00;      // 8-bit sink driver state (only 0..3 used)
uint8_t brightnessVal = 0;     // logical 0..255 (255 = max visible)

// Change counters for race protection
volatile uint32_t ledChangeCounter = 0;    // increments on user LED changes
volatile uint32_t brightChangeCounter = 0; // increments on user brightness changes

const unsigned long SERIAL_BAUD = 115200;
const char IDENT_STRING[] = "IDENT: MIP HYD-PRS, v1.0 MAQ";

bool RUN_STARTUP_TEST = true;
const unsigned long STARTUP_TEST_DELAY_MS = 400;

const int DEFAULT_STEP_DELAY_MS = 8;
const int DEFAULT_STEP_DEG = 1;

// ==== Low-level LED update (MSB-first for 8-bit) ====
void updateLedDriver8(uint8_t state) {
  digitalWrite(PIN_LATCH, LOW);
  for (int i = 7; i >= 0; --i) {
    bool bit = (state >> i) & 0x1;
    digitalWrite(PIN_DATA, bit ? HIGH : LOW);
    digitalWrite(PIN_CLOCK, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_CLOCK, LOW);
    delayMicroseconds(2);
  }
  digitalWrite(PIN_LATCH, HIGH);
  delayMicroseconds(4);
  digitalWrite(PIN_LATCH, LOW);
  ledState8 = state;
}

// ==== LED setters ====
void setLedMask8Internal(uint8_t mask) {
  // System/internal setter: writes mask to hardware but does NOT increment user counter
  updateLedDriver8(mask);
  Serial.print("setLedMask8Internal -> 0x"); Serial.println(mask, HEX);
}

void setLedMask8(uint8_t mask) {
  // User-facing setter: writes mask and signals a user change
  updateLedDriver8(mask);
  ledChangeCounter++; // mark user change
  Serial.print("setLedMask8 called -> 0x"); Serial.println(mask, HEX);
}

// ==== Brightness (inverted HW) - internal and user setters ====
void setBrightnessInternal(uint8_t val) {
  brightnessVal = constrain(val, 0, 255);
  uint8_t hw = 255 - brightnessVal;
  if (hw == 0) hw = 1;
  analogWrite(PIN_BRIGHTNESS, hw);
  Serial.print("setBrightnessInternal -> "); Serial.println(brightnessVal);
}

void setBrightnessUser(uint8_t val) {
  brightnessVal = constrain(val, 0, 255);
  brightChangeCounter++; // mark user change
  uint8_t hw = 255 - brightnessVal;
  if (hw == 0) hw = 1;
  analogWrite(PIN_BRIGHTNESS, hw);
  Serial.print("setBrightnessUser -> "); Serial.println(brightnessVal);
}

// ==== Servo attach/detach helpers ====
void detachAllServos() {
  Serial.println("Servos: detaching all for safe PWM.");
  servoAccuHyd.detach();
  servoBrkL.detach();
  servoBrkR.detach();
}

void attachAllServos() {
  servoAccuHyd.attach(PIN_SERVO_ACCU_HYD);
  servoBrkL.attach(PIN_SERVO_BRK_L);
  servoBrkR.attach(PIN_SERVO_BRK_R);
  Serial.println("Servos: re-attached.");
  delay(8);
}

// ==== Safe fade that detaches servos during fade (internal use) ====
void fadeBrightnessInternal(uint8_t startVal, uint8_t endVal, int steps = 30, int stepDelayMs = 20) {
  if (startVal == endVal) return;
  int absDiff = abs(endVal - startVal);
  if (absDiff == 0) return;
  if (absDiff < steps) steps = absDiff;

  detachAllServos();
  delay(6);

  for (int i = 1; i <= steps; ++i) {
    int next = startVal + ((endVal - startVal) * i) / steps;
    setBrightnessInternal(constrain(next, 0, 255));
    delay(stepDelayMs);
  }
  setBrightnessInternal(endVal);

  delay(8);
  attachAllServos();
  delay(6);
}

// ==== User fade: performs fade and finalizes as user change ====
void fadeBrightnessUser(uint8_t startVal, uint8_t endVal, int steps = 20, int stepDelayMs = 12) {
  if (startVal == endVal) {
    // Still mark as user change to prevent restores overriding
    brightChangeCounter++;
    return;
  }
  int absDiff = abs(endVal - startVal);
  if (absDiff < steps) steps = absDiff;

  detachAllServos();
  delay(6);

  for (int i = 1; i <= steps; ++i) {
    int next = startVal + ((endVal - startVal) * i) / steps;
    setBrightnessInternal(constrain(next, 0, 255)); // avoid marking interim steps as user-change
    delay(stepDelayMs);
  }
  // finalize as user change
  setBrightnessUser(endVal);

  delay(8);
  attachAllServos();
  delay(6);
}

// ==== Motion helpers ====
int applyServoOffsetAndConstrain(int angle, int offset) {
  int a = angle + offset;
  if (a < 0) a = 0;
  if (a > 180) a = 180;
  return a;
}

void smoothMove(Servo &s, int fromDeg, int toDeg, int stepDeg = DEFAULT_STEP_DEG, int stepDelayMs = DEFAULT_STEP_DELAY_MS) {
  fromDeg = constrain(fromDeg, 0, 180);
  toDeg   = constrain(toDeg,   0, 180);
  if (fromDeg == toDeg) return;
  int dir = (toDeg > fromDeg) ? 1 : -1;
  int pos = fromDeg;
  while (pos != toDeg) {
    pos += dir * stepDeg;
    if ((dir == 1 && pos > toDeg) || (dir == -1 && pos < toDeg)) pos = toDeg;
    s.write(pos);
    delay(stepDelayMs);
  }
}

// ==== Stage mappings ====
int mapAccuStageToAngle(int stage) {
  stage = constrain(stage, 0, 4);
  return applyServoOffsetAndConstrain(stage * 30, servoOffsetAccuHyd); // 0,30,60,90,120
}

int mapBrakeStageToAngle(int stage, int which) {
  stage = constrain(stage, 0, 3);
  int base = stage * 30; // 0,30,60,90
  if (which == 0) return applyServoOffsetAndConstrain(base, servoOffsetBrkL);
  return applyServoOffsetAndConstrain(base, servoOffsetBrkR);
}

// ==== Startup (race-protected, uses INTERNAL setter for temp masks & brightness) ====
void runStartupRoutine() {
  Serial.println("Startup test: servos out+back then safe fade (race-protected).");
  uint8_t savedOldMask = ledState8;
  uint8_t savedOldBright = brightnessVal;
  uint32_t startLedCounter = ledChangeCounter;
  uint32_t startBrightCounter = brightChangeCounter;

  int outA = mapAccuStageToAngle(4);
  int outL = mapBrakeStageToAngle(3, 0);
  int outR = mapBrakeStageToAngle(3, 1);

  int curA = servoAccuHyd.read();
  int curL = servoBrkL.read();
  int curR = servoBrkR.read();

  const int steps = 12;

  // show static backlight nibble during motion (no heavy PWM)
  setBrightnessInternal(min(200, brightnessVal + 80));
  setLedMask8Internal((ledState8 & 0xF0) | 0x0F);

  for (int step = 1; step <= steps; ++step) {
    int aPos = curA + ((outA - curA) * step) / steps;
    int lPos = curL + ((outL - curL) * step) / steps;
    int rPos = curR + ((outR - curR) * step) / steps;
    servoAccuHyd.write(aPos);
    servoBrkL.write(lPos);
    servoBrkR.write(rPos);

    uint8_t bl = (15 * step) / steps;
    setLedMask8Internal((ledState8 & 0xF0) | bl);

    delay(DEFAULT_STEP_DELAY_MS * 8);
  }

  delay(220);

  for (int step = 1; step <= steps; ++step) {
    int aPos = outA + ((curA - outA) * step) / steps;
    int lPos = outL + ((curL - outL) * step) / steps;
    int rPos = outR + ((curR - outR) * step) / steps;
    servoAccuHyd.write(aPos);
    servoBrkL.write(lPos);
    servoBrkR.write(rPos);

    uint8_t bl = (15 * (steps - step)) / steps;
    setLedMask8Internal((ledState8 & 0xF0) | bl);

    delay(DEFAULT_STEP_DELAY_MS * 8);
  }

  // After motion: safe internal fades
  fadeBrightnessInternal(brightnessVal, 255, 30, 12);
  delay(120);
  fadeBrightnessInternal(255, brightnessVal, 30, 12);

  // restore only if no user changes happened during startup
  delay(10);
  if (ledChangeCounter == startLedCounter) {
    setLedMask8Internal(savedOldMask);
    Serial.println("Startup: restored previous LED mask");
  } else {
    Serial.println("Startup: LED mask changed during Startup; leaving user changes in place");
  }

  if (brightChangeCounter == startBrightCounter) {
    setBrightnessInternal(savedOldBright);
    Serial.println("Startup: restored previous brightness");
  } else {
    Serial.println("Startup: brightness changed during Startup; leaving user changes in place");
  }

  Serial.println("Startup completed.");
}

// ==== DIAG (race-protected) ====
void runDiagRoutine() {
  Serial.println("DIAG: Starting diagnostic routine (race-protected).");
  uint8_t savedOldMask = ledState8;
  uint8_t savedOldBright = brightnessVal;
  uint32_t startLedCounter = ledChangeCounter;
  uint32_t startBrightCounter = brightChangeCounter;

  // show static backlight nibble during motion (no heavy PWM)
  setLedMask8Internal((ledState8 & 0xF0) | 0x0F);
  setBrightnessInternal(min(200, brightnessVal + 80));
  delay(80);

  int outA = mapAccuStageToAngle(4);
  int outL = mapBrakeStageToAngle(3, 0);
  int outR = mapBrakeStageToAngle(3, 1);

  int curA = servoAccuHyd.read();
  int curL = servoBrkL.read();
  int curR = servoBrkR.read();

  // move out
  smoothMove(servoAccuHyd, curA, outA);
  smoothMove(servoBrkL, curL, outL);
  smoothMove(servoBrkR, curR, outR);

  delay(200);

  // move back
  smoothMove(servoAccuHyd, servoAccuHyd.read(), curA);
  smoothMove(servoBrkL, servoBrkL.read(), curL);
  smoothMove(servoBrkR, servoBrkR.read(), curR);

  // After motion: safe internal fades
  fadeBrightnessInternal(brightnessVal, 255, 30, 12);
  delay(120);
  fadeBrightnessInternal(255, brightnessVal, 30, 12);

  // restore only if no user changes happened during DIAG
  delay(10);
  if (ledChangeCounter == startLedCounter) {
    setLedMask8Internal(savedOldMask);
    Serial.println("DIAG: restored previous LED mask");
  } else {
    Serial.println("DIAG: LED mask changed during DIAG; leaving user changes in place");
  }

  if (brightChangeCounter == startBrightCounter) {
    setBrightnessInternal(savedOldBright);
    Serial.println("DIAG: restored previous brightness");
  } else {
    Serial.println("DIAG: brightness changed during DIAG; leaving user changes in place");
  }

  Serial.println("DIAG finished.");
}

// ==== Soft reset ====
void performReset() {
  Serial.println("RESET: performing soft reset...");
  Serial.flush();
  delay(50);
  #if defined(__AVR__)
    wdt_enable(WDTO_15MS);
    while (1) {}
  #else
    while (1) {}
  #endif
}

// ==== Serial command parser ====
void handleSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  int sp = line.indexOf(' ');
  String cmd = (sp == -1) ? line : line.substring(0, sp);
  cmd.toUpperCase();

  Serial.print("CMD recv: "); Serial.println(cmd);

  if (cmd == "VER") {
    Serial.println(IDENT_STRING);
    delay(50);
    return;
  }

  if (cmd == "DIAG") {
    runDiagRoutine();
    return;
  }

  if (cmd == "RESET") {
    performReset();
    return;
  }

  if (cmd == "ACCU") {
    String arg = (sp == -1) ? "" : line.substring(sp + 1);
    int stage = constrain(arg.toInt(), 0, 4);
    int angle = mapAccuStageToAngle(stage);
    Serial.print("ACCU stage "); Serial.print(stage); Serial.print(" -> "); Serial.println(angle);
    smoothMove(servoAccuHyd, servoAccuHyd.read(), angle);
    return;
  }

  if (cmd == "BRKL" || cmd == "BRKR") {
    String arg = (sp == -1) ? "" : line.substring(sp + 1);
    int stage = constrain(arg.toInt(), 0, 3);
    int angle = mapBrakeStageToAngle(stage, (cmd == "BRKL") ? 0 : 1); // fixed mapping
    Serial.print(cmd); Serial.print(" stage "); Serial.print(stage); Serial.print(" -> "); Serial.println(angle);
    if (cmd == "BRKL") smoothMove(servoBrkL, servoBrkL.read(), angle);
    else smoothMove(servoBrkR, servoBrkR.read(), angle);
    return;
  }

  if (cmd == "S1" || cmd == "S2" || cmd == "S3") {
    String arg = (sp == -1) ? "" : line.substring(sp + 1);
    int deg = constrain(arg.toInt(), 0, 180);
    if (cmd == "S1") smoothMove(servoAccuHyd, servoAccuHyd.read(), applyServoOffsetAndConstrain(deg, servoOffsetAccuHyd));
    if (cmd == "S2") smoothMove(servoBrkL, servoBrkL.read(), applyServoOffsetAndConstrain(deg, servoOffsetBrkL));
    if (cmd == "S3") smoothMove(servoBrkR, servoBrkR.read(), applyServoOffsetAndConstrain(deg, servoOffsetBrkR));
    Serial.print(cmd); Serial.print(" -> "); Serial.println(deg);
    return;
  }

  if (cmd == "LED") {
    String arg = (sp == -1) ? "" : line.substring(sp + 1);
    unsigned long m = strtoul(arg.c_str(), nullptr, 0);
    uint8_t mask = (uint8_t)(m & 0xFF);
    setLedMask8(mask);
    Serial.print("LED mask set to 0x"); Serial.println(mask, HEX);
    return;
  }

  // LEDCH: user-facing channel set: LEDCH <channel> <0|1>
  if (cmd == "LEDCH") {
    String rest = (sp == -1) ? "" : line.substring(sp + 1);
    int sp2 = rest.indexOf(' ');
    if (sp2 == -1) {
      Serial.println("LEDCH requires two args: LEDCH <channel> <0|1>");
      return;
    }
    int ch = rest.substring(0, sp2).toInt();
    int val = rest.substring(sp2 + 1).toInt();
    if (ch < 0 || ch > 7) {
      Serial.println("Channel out of range 0..7");
      return;
    }
    bool on = (val != 0);
    if (on) ledState8 |= (1u << ch);
    else    ledState8 &= ~(1u << ch);

    // user setter -> increments counter
    setLedMask8(ledState8);

    // If turning on and global brightness is zero, do a visible pulse
    if (on && brightnessVal == 0) {
      // quick pulse without permanently changing brightnessVal
      uint8_t prev = brightnessVal;
      fadeBrightnessUser(prev, 200, 12, 12); // user-fade will finalize as user change
      fadeBrightnessUser(200, prev, 12, 12);
    } else {
      setBrightnessInternal(brightnessVal); // ensure OE driven
    }

    Serial.print("LEDCH "); Serial.print(ch); Serial.print(" -> "); Serial.println(on ? "ON" : "OFF");
    Serial.print(" -> mask now 0x"); Serial.println(ledState8, HEX);
    Serial.print(" -> brightness logical "); Serial.println(brightnessVal);
    return;
  }

  if (cmd == "BR") {
    String arg = (sp == -1) ? "" : line.substring(sp + 1);
    int v = constrain(arg.toInt(), 0, 255);
    fadeBrightnessUser(brightnessVal, v, 20, 12);
    Serial.print("BR set to logical "); Serial.println(brightnessVal);
    return;
  }

  if (cmd == "TEST") {
    Serial.println("Running TEST: servo sweep + LED chase");
    for (int a = 0; a <= 120; a += 10) smoothMove(servoAccuHyd, servoAccuHyd.read(), a);
    for (int i = 0; i < 8; i++) { setLedMask8Internal(1u << i); delay(80); }
    setLedMask8Internal(0xFF); delay(200); setLedMask8Internal(0x00);
    Serial.println("TEST done");
    return;
  }

  if (cmd == "OFF") {
    setLedMask8(0x00);
    Serial.println("All LEDs off.");
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
}

// ==== setup + loop ====
String serialBuffer = "";

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) { delay(2); }
  Serial.println();
  Serial.println("ACCU Press Modul (Race-safe) initialising...");

  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_LATCH, LOW);
  digitalWrite(PIN_CLOCK, LOW);
  digitalWrite(PIN_DATA, LOW);

  pinMode(PIN_BRIGHTNESS, OUTPUT);
  brightnessVal = 0;
  setBrightnessInternal(brightnessVal);

  // attach servos
  servoAccuHyd.attach(PIN_SERVO_ACCU_HYD);
  servoBrkL.attach(PIN_SERVO_BRK_L);
  servoBrkR.attach(PIN_SERVO_BRK_R);

  // home positions
  servoAccuHyd.write(applyServoOffsetAndConstrain(0, servoOffsetAccuHyd));
  servoBrkL.write(applyServoOffsetAndConstrain(0, servoOffsetBrkL));
  servoBrkR.write(applyServoOffsetAndConstrain(0, servoOffsetBrkR));
  delay(150);

  setLedMask8Internal(0x00);

  Serial.println("Hardware initialised.");
  Serial.print("Brightness pin: "); Serial.println(PIN_BRIGHTNESS);
  Serial.print("LED pins LCK/DAT/CLK: "); Serial.print(PIN_LATCH); Serial.print("/"); Serial.print(PIN_DATA); Serial.print("/"); Serial.println(PIN_CLOCK);

  if (RUN_STARTUP_TEST) {
    delay(STARTUP_TEST_DELAY_MS);
    runStartupRoutine();
  }

  Serial.println("Ready. Commands: VER DIAG RESET ACCU <0..4> BRKL <0..3> BRKR <0..3> S1/S2/S3 LED LEDCH BR TEST OFF");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleSerialCommand(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 200) serialBuffer = serialBuffer.substring(serialBuffer.length() - 200);
    }
  }
  delay(2);
}
