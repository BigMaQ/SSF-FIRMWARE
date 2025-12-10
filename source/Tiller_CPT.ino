/*
  ============================================================================
  TILLER 1 CPT Firmware v2.1 - Refactored with Enhanced Smoothing
  
  Pinout:
    Latch   -> Pin 2   (shift register latch)
    Clock   -> Pin 5   (shift register clock)
    Data    -> Pin 7   (shift register serial data)
    Bright  -> Pin 3   (PWM backlight control)
    Button  -> Pin 4   (physical button - active LOW)
    Hall    -> A7      (Hall sensor input - analog)
  
  Protocol: Matches RMP format for consistency
    Commands: IDENT, VER, RESET, REQ, BR<val>, LED1:<byte>, SMO<val>, DBD<val>
    Output:   IN1:<shift_bits>, A7:<adc_value>, LED1:<shift_bits>, BR:<value>
  
  Hall Sensor Smoothing:
    Default: 60-sample moving average + 5-count deadband
    Reduces jitter/noise while maintaining responsiveness
    Configurable via SMO (samples) and DBD (deadband) commands
  ============================================================================
*/

#include <avr/wdt.h>   // for RESET via Watchdog

// Panel identification (must match INI file section name)
const char PANEL_IDENT[] = "TILLER 1 CPT, v2.1 MAQ";
const char PANEL_VERSION[] = "2.1";

// === Pin Definitions ===
const int PIN_LATCH = 2;      // Shift register latch
const int PIN_CLOCK = 5;      // Shift register clock
const int PIN_DATA  = 7;      // Shift register data
const int PIN_BRIGHT = 3;     // PWM brightness (inverted logic)
const int PIN_BUTTON = 4;     // Button input (active LOW)
const int PIN_HALL = A7;      // Hall sensor

// === State Variables ===
byte ledState = 0x00;         // Shift register LED state (8 bits)
int brightness = 200;         // Logical brightness: 0..255 (higher = brighter)
int lastBright = -1;          // Last sent brightness value

// === Change Detection ===
int lastHall = -1;            // Last Hall sensor reading
int lastButton = -1;          // Last button state (1=released, 0=pressed)
byte lastLedState = 0xFF;     // Last sent LED state (initialized to different value)

// === Hall Sensor Smoothing & Deadband ===
// Moving average filter with 60-sample window (strong smoothing)
// + Deadband filter to suppress noise (only send when change > threshold)
int HALL_SMOOTH_SAMPLES = 60;          // Default: 60 samples (configurable via SMO command)
int hallBuffer[60];                     // Max 60 samples
int hallBufferIndex = 0;
long hallBufferSum = 0;

// Deadband threshold: only send if change >= this value
// Prevents jitter from continuous +/-1 oscillations
int hallDeadband = 5;                   // Default: 5 ADC counts (configurable via DBD command)

// === Diagnostic Mode ===
bool diagActive = false;
unsigned long diagTimer = 0;
int diagPhase = 0;
int diagBright = 0;

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Apply brightness to backlight (inverted PWM)
 * Higher logical value = higher brightness
 * PWM: 0=full on, 255=off (common cathode design)
 */
inline void applyBrightness(int val) {
  val = constrain(val, 0, 255);
  analogWrite(PIN_BRIGHT, 255 - val);  // Inverted: logical 255 -> PWM 0 (full brightness)
}

/**
 * Update shift register with current LED state
 */
void updateShiftRegister() {
  digitalWrite(PIN_LATCH, LOW);
  shiftOut(PIN_DATA, PIN_CLOCK, MSBFIRST, ledState);
  digitalWrite(PIN_LATCH, HIGH);
}

/**
 * Format and send status response
 */
void sendStatus(const char* label, const char* value) {
  Serial.print(label);
  Serial.print(":");
  Serial.println(value);
}

/**
 * Initialize Hall sensor smoothing buffer with current readings
 */
void initHallBuffer() {
  for (int i = 0; i < HALL_SMOOTH_SAMPLES; i++) {
    hallBuffer[i] = analogRead(PIN_HALL);
    hallBufferSum += hallBuffer[i];
  }
}

/**
 * Read Hall sensor with moving average smoothing + deadband filtering
 * Returns filtered ADC value (0-1023) or -1 if no significant change
 */
int readHallSmoothed() {
  hallBufferSum -= hallBuffer[hallBufferIndex];
  hallBuffer[hallBufferIndex] = analogRead(PIN_HALL);
  hallBufferSum += hallBuffer[hallBufferIndex];
  hallBufferIndex = (hallBufferIndex + 1) % HALL_SMOOTH_SAMPLES;
  
  int filtered = hallBufferSum / HALL_SMOOTH_SAMPLES;
  
  // Deadband filter: only consider changes larger than threshold
  if (lastHall >= 0 && abs(filtered - lastHall) < hallDeadband) {
    return -1;  // No significant change, suppress output
  }
  
  return filtered;
}

/**
 * Format byte as binary string for display (e.g., "01010101")
 */
void formatBinary(byte val, char* buf, int bufsize) {
  if (bufsize < 9) return;
  for (int i = 7; i >= 0; i--) {
    buf[7 - i] = (val & (1 << i)) ? '1' : '0';
  }
  buf[8] = '\0';
}

// ============================================================================
// Initialization
// ============================================================================

void setup() {
  // Serial communication
  Serial.begin(115200);
  
  delay(100);  // Let serial stabilize
  
  // Configure pin modes
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_BRIGHT, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_HALL, INPUT);
  
  // Initialize shift register
  updateShiftRegister();
  
  // Initialize Hall sensor buffer (must do before first smoothing)
  initHallBuffer();
  
  // Power-On Self Test
  runPowerOnTest();
  
  // Apply startup brightness
  applyBrightness(brightness);
  lastBright = brightness;
  
  // Signal ready
  Serial.println("READY");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  // === Process Button (only when changed) ===
  int buttonRead = digitalRead(PIN_BUTTON);
  if (buttonRead != lastButton) {
    lastButton = buttonRead;
    // Format: IN1:00000001 (button state in LSB, bit 0)
    // Button pressed (LOW) = bit set to 1
    char buf[16];
    sprintf(buf, "IN1:%08b", buttonRead == LOW ? 0x01 : 0x00);
    Serial.println(buf);
  }
  
  // === Process Hall Sensor (with smoothing & deadband, only when changed) ===
  int hallVal = readHallSmoothed();
  if (hallVal >= 0 && hallVal != lastHall) {  // Only send if readHallSmoothed() returned valid value
    lastHall = hallVal;
    char buf[16];
    sprintf(buf, "A7:%04d", hallVal);
    Serial.println(buf);
  }
  
  // === Process Serial Commands ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() == 0) {
      return;  // Empty command
    }
    
    // ===== Command: IDENT =====
    if (cmd.equalsIgnoreCase("IDENT")) {
      Serial.print("IDENT:");
      Serial.println(PANEL_IDENT);
    }
    // ===== Command: VER =====
    else if (cmd.equalsIgnoreCase("VER")) {
      Serial.print("VER:");
      Serial.println(PANEL_VERSION);
    }
    // ===== Command: RESET =====
    else if (cmd.equalsIgnoreCase("RESET")) {
      Serial.println("RESETTING");
      delay(50);
      wdt_enable(WDTO_15MS);  // 15ms watchdog timer
      while (1) {}  // Wait for reset
    }
    // ===== Command: REQ (Request Status) =====
    else if (cmd.equalsIgnoreCase("REQ")) {
      // Send current status for all outputs
      char bufLED[16];
      sprintf(bufLED, "LED1:%08b", ledState);
      Serial.println(bufLED);
      
      char bufBR[16];
      sprintf(bufBR, "BR:%d", brightness);
      Serial.println(bufBR);
    }
    // ===== Command: BR<val> (Brightness) =====
    else if (cmd.startsWith("BR")) {
      String valStr = cmd.substring(2);
      int val = valStr.toInt();
      val = constrain(val, 0, 255);
      
      if (val != lastBright) {
        brightness = val;
        applyBrightness(brightness);
        lastBright = brightness;
        
        char buf[16];
        sprintf(buf, "BR:%d", brightness);
        Serial.println(buf);
      }
    }
    // ===== Command: LED1:<byte> (Shift Register Control) =====
    else if (cmd.startsWith("LED1:")) {
      String valStr = cmd.substring(5);  // Remove "LED1:" prefix
      
      // Parse binary string or hex
      byte newState = 0x00;
      if (valStr.length() == 8) {
        // Binary format: "01010101"
        for (int i = 0; i < 8; i++) {
          if (valStr[i] == '1') {
            newState |= (1 << (7 - i));
          }
        }
      } else {
        // Hex format: "0xAA" or "AA"
        newState = (byte)strtol(valStr.c_str(), NULL, 16);
      }
      
      if (newState != lastLedState) {
        ledState = newState;
        updateShiftRegister();
        lastLedState = ledState;
        
        char buf[16];
        sprintf(buf, "LED1:%08b", ledState);
        Serial.println(buf);
      }
    }
    // ===== Command: DIAG (Diagnostic Mode) =====
    else if (cmd.equalsIgnoreCase("DIAG")) {
      diagStart();
    }
    // ===== Command: SMO<val> (Hall Smoothing Samples) =====
    // Set number of samples for moving average (1-60)
    // Higher = smoother but more latency
    else if (cmd.startsWith("SMO")) {
      String valStr = cmd.substring(3);
      int val = valStr.toInt();
      val = constrain(val, 1, 60);
      HALL_SMOOTH_SAMPLES = val;
      
      // Re-initialize buffer with new sample count
      hallBufferIndex = 0;
      hallBufferSum = 0;
      for (int i = 0; i < HALL_SMOOTH_SAMPLES; i++) {
        hallBuffer[i] = analogRead(PIN_HALL);
        hallBufferSum += hallBuffer[i];
      }
      
      char buf[32];
      sprintf(buf, "SMO:%d", HALL_SMOOTH_SAMPLES);
      Serial.println(buf);
    }
    // ===== Command: DBD<val> (Hall Deadband Threshold) =====
    // Set deadband threshold for change detection (0-50)
    // Higher = less frequent updates, smoother experience
    else if (cmd.startsWith("DBD")) {
      String valStr = cmd.substring(3);
      int val = valStr.toInt();
      val = constrain(val, 0, 50);
      hallDeadband = val;
      
      char buf[32];
      sprintf(buf, "DBD:%d", hallDeadband);
      Serial.println(buf);
    }
  }
  
  // Update diagnostic mode if active
  diagLoop();
}

// ============================================================================
// Power-On Self Test (POST)
// ============================================================================
/**
 * POST Sequence: All LEDs blink 2x in ~1 second total
 * - Ramp all LEDs to full brightness (250ms)
 * - Hold (250ms)
 * - Ramp all LEDs to off (250ms)
 * - Hold off (250ms)
 * - Turn everything off
 */
void runPowerOnTest() {
  // Set all LED bits
  ledState = 0xFF;
  updateShiftRegister();
  
  // Blink 2 times
  for (int i = 0; i < 2; i++) {
    applyBrightness(254);  // Max brightness
    updateShiftRegister();
    delay(250);
    
    applyBrightness(0);    // Off
    updateShiftRegister();
    delay(250);
  }
  
  // Clear all LEDs
  ledState = 0x00;
  updateShiftRegister();
  applyBrightness(0);
  
  Serial.println("POST:OK");
}

// ============================================================================
// Diagnostic Mode (DIAG Command)
// ============================================================================
/**
 * Diagnostic Sequence:
 * 1. Fade up: 0 -> 254 (254 steps @ 20ms = ~5.1 seconds)
 * 2. Hold: 3 seconds at full brightness
 * 3. Fade down: 254 -> 0 (254 steps @ 20ms = ~5.1 seconds)
 * 4. Turn off
 * 
 * Purpose: Visually verify backlight control and fade smoothness
 */

void diagStart() {
  // Turn on all LED bits
  for (int i = 0; i < 8; i++) {
    ledState |= (1 << i);
  }
  updateShiftRegister();
  
  // Initialize diagnostic state
  diagActive = true;
  diagPhase = 0;
  diagBright = 0;
  applyBrightness(diagBright);
  diagTimer = millis();
  
  Serial.println("DIAG:START");
}

void diagLoop() {
  if (!diagActive) return;
  
  unsigned long now = millis();
  
  switch (diagPhase) {
    // === Phase 0: Fade UP ===
    case 0: {
      if (now - diagTimer >= 20) {
        diagTimer = now;
        if (diagBright < 254) {
          diagBright++;
          applyBrightness(diagBright);
        } else {
          // Transition to hold phase
          diagPhase = 1;
          diagTimer = now;
          Serial.println("DIAG:FADE_UP_DONE");
        }
      }
      break;
    }
    
    // === Phase 1: Hold 3 seconds ===
    case 1: {
      if (now - diagTimer >= 3000) {
        diagPhase = 2;
        diagTimer = now;
        Serial.println("DIAG:HOLDING");
      }
      break;
    }
    
    // === Phase 2: Fade DOWN ===
    case 2: {
      if (now - diagTimer >= 20) {
        diagTimer = now;
        if (diagBright > 0) {
          diagBright--;
          applyBrightness(diagBright);
        } else {
          // End diagnostic mode
          diagPhase = 3;
          diagTimer = now;
          Serial.println("DIAG:FADE_DOWN_DONE");
        }
      }
      break;
    }
    
    // === Phase 3: Complete ===
    case 3: {
      // Clear all LEDs
      ledState = 0x00;
      updateShiftRegister();
      applyBrightness(0);
      
      diagActive = false;
      Serial.println("DIAG:COMPLETE");
      break;
    }
  }
}
