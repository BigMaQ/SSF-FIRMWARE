/*
  PANEL_IDENT: "TILLER 1 CPT, v1.0 MAQ"
  Latch   -> Pin 2
  Clock   -> Pin 5
  Data    -> Pin 7
  Bright  -> Pin 3 (PWM)
  Button  -> Pin 4
  Hall    -> A7
*/

#include <avr/wdt.h>   // für RESET via Watchdog

const char* PANEL_IDENT = "IDENT: TILLER 1 CPT, v1.1 MAQ";

const int latchPin = 2;
const int clockPin = 5;
const int dataPin  = 7;
const int brightPin = 3;
const int buttonPin = 4;
const int hallPin   = A7;

// Globale Zustände
byte ledState = 0;
int brightness = 200;       // logische Helligkeit 0..255 (höher = heller)
int lastBright = -1;

// Letzte Werte für "nur bei Änderung"
int lastHall = -1;
int lastButton = -1;
byte lastLedState = 255;

// --- Hall smoothing ---
const int smoothSamples = 30;
int hallBuffer[smoothSamples];
int hallIndex = 0;
long hallSum = 0;

// --- DIAG ---
bool diagActive = false;
unsigned long diagTimer = 0;
int diagPhase = 0;
int diagBright = 0;

// Invertierte PWM-Ansteuerung: logische 0 (dunkel) -> 255, logische 255 (hell) -> 0
inline void applyBrightness(int val) {
  val = constrain(val, 0, 255);
  analogWrite(brightPin, 255 - val);
}

void setup() {
  Serial.begin(115200);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(brightPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(hallPin, INPUT);

  updateLEDs();

  // Buffer initialisieren
  for (int i = 0; i < smoothSamples; i++) {
    hallBuffer[i] = analogRead(hallPin);
    hallSum += hallBuffer[i];
  }

  // POST: alle LEDs 2x blinken, Gesamtdauer ~1 Sekunde
  runPOST();

  // Starthelligkeit anwenden
  applyBrightness(brightness);
  lastBright = brightness;

  Serial.println("READY");
}

void loop() {
  // Button nur bei Änderung senden
  int btn = digitalRead(buttonPin);
  if (btn != lastButton) {
    lastButton = btn;
    Serial.println(btn == LOW ? "BUTTON: PRESSED" : "BUTTON: RELEASED");
  }

  // Hall mit gleitendem Mittel
  hallSum -= hallBuffer[hallIndex];
  hallBuffer[hallIndex] = analogRead(hallPin);
  hallSum += hallBuffer[hallIndex];
  hallIndex = (hallIndex + 1) % smoothSamples;
  int hallVal = hallSum / smoothSamples;

  if (hallVal != lastHall) {
    lastHall = hallVal;
    char buf[16];
    sprintf(buf, "A7:%04d", hallVal);
    Serial.println(buf);
  }

  // Serial-Kommandos
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("VER")) {
      Serial.println(PANEL_IDENT);
    }
    else if (cmd.equalsIgnoreCase("RESET")) {
      Serial.println("RESETTING...");
      delay(50);          // kurze Pause, damit die Meldung rausgeht
      wdt_enable(WDTO_15MS); // Watchdog mit 15ms Timeout
      while (1) {}        // warten bis Reset
    }
    else if (cmd.startsWith("BR")) {
      int val = cmd.substring(2).toInt();
      val = constrain(val, 0, 255);
      if (val != lastBright) {
        brightness = val;
        applyBrightness(brightness);
        lastBright = brightness;
        Serial.print("BRIGHTNESS SET TO ");
        Serial.println(brightness);
      }
    }
    else if (cmd.startsWith("LEDCH")) {
      int space1 = cmd.indexOf(' ');
      int space2 = cmd.indexOf(' ', space1 + 1);
      int ch = cmd.substring(space1 + 1, space2).toInt();
      int st = cmd.substring(space2 + 1).toInt();
      if (ch >= 0 && ch < 8) {
        if (st == 1) ledState |= (1 << ch);
        else          ledState &= ~(1 << ch);
        updateLEDs();
        if (ledState != lastLedState) {
          lastLedState = ledState;
          Serial.print("LEDCH ");
          Serial.print(ch);
          Serial.print(" = ");
          Serial.println(st);
        }
      }
    }
    else if (cmd.equalsIgnoreCase("DIAG")) {
      runDiagStart();
    }
  }

  runDiagLoop();
}

void updateLEDs() {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, ledState);
  digitalWrite(latchPin, HIGH);
}

// --- POST: alle LEDs 2x blinken, Gesamtdauer ~1 Sekunde ---
void runPOST() {
  for (int ch = 0; ch <= 4; ch++) {
    ledState |= (1 << ch);
  }
  for (int i = 0; i < 2; i++) {
    applyBrightness(254);   // volle Helligkeit
    updateLEDs();
    delay(250);
    applyBrightness(0);     // aus
    updateLEDs();
    delay(250);
  }
  for (int ch = 0; ch <= 4; ch++) {
    ledState &= ~(1 << ch);
  }
  updateLEDs();
  applyBrightness(0);
  Serial.println("POST COMPLETE");
}

// --- DIAG Sequenz: dunkel -> fade up -> 3s hold -> fade down -> aus ---
void runDiagStart() {
  for (int ch = 0; ch <= 4; ch++) {
    ledState |= (1 << ch);
  }
  updateLEDs();

  diagActive = true;
  diagPhase = 0;
  diagBright = 0;
  applyBrightness(diagBright);
  diagTimer = millis();
  Serial.println("DIAG START");
}

void runDiagLoop() {
  if (!diagActive) return;

  unsigned long now = millis();

  switch (diagPhase) {
    case 0: // Fade UP
      if (now - diagTimer >= 20) {
        diagTimer = now;
        if (diagBright < 254) {
          diagBright++;
          applyBrightness(diagBright);
        } else {
          diagPhase = 1;
          diagTimer = now;
          Serial.println("FULL BRIGHTNESS HOLD");
        }
      }
      break;

    case 1: // Hold 3 Sekunden
      if (now - diagTimer >= 3000) {
        diagPhase = 2;
        diagTimer = now;
        Serial.println("FADE DOWN");
      }
      break;

    case 2: // Fade DOWN
      if (now - diagTimer >= 20) {
        diagTimer = now;
        if (diagBright > 0) {
          diagBright--;
          applyBrightness(diagBright);
        } else {
          diagPhase = 3;
          Serial.println("DIAG COMPLETE");
          for (int ch = 0; ch <= 4; ch++) {
            ledState &= ~(1 << ch);
          }
          updateLEDs();
          applyBrightness(0);
          diagActive = false;
        }
      }
      break;
  }
}
