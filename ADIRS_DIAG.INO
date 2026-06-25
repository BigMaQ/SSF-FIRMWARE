// ADIRS_DIAG.INO – Minimal ADIRS LED Driver Test (Arduino MEGA 2560)
// (w) 2025 M. Quatember
// Bare-metal test: only ADIRS LED drivers on D11/D12/D14
// Serial 115200 – commands: LED1:11111111; LED2:00000000;

// ============================================================================
// PIN DEFINITIONS – ADIRS LED DRIVER ONLY
// ============================================================================
// ADIRS_LEDDRV: LATCH D11, CLK D12, DATA D14, AMOUNT 2
const int ledLatch = 11;
const int ledClock = 12;
const int ledData  = 14;
#define LED_BYTES 2

// ============================================================================
// LED STATE
// ============================================================================
uint8_t ledState[LED_BYTES] = {0};

// ============================================================================
// HELPERS
// ============================================================================
void bin8(char* out, uint8_t b) {
  for (int i = 0; i < 8; i++) out[i] = ((b >> (7 - i)) & 1) ? '1' : '0';
  out[8] = 0;
}

uint8_t parseBin8(const char* s) {
  uint8_t v = 0;
  for (int i = 0; s[i]; i++) {
    if (s[i] == '0' || s[i] == '1') v = (v << 1) | (s[i] == '1' ? 1 : 0);
  }
  return v;
}

static int atoi_s(const char* s) {
  int v = 0, sign = 1;
  while (*s == ' ') s++;
  if (*s == '-') { sign = -1; s++; }
  while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
  return v * sign;
}

// ============================================================================
// LED OUTPUT – shift 2 bytes to ADIRS TLC5916 chain
// Shift order: LSByte first (byte 1 → byte 0), MSBFIRST per byte
// This matches OHP_SEC shiftOutOne convention
// ============================================================================
void applyADIRS() {
  digitalWrite(ledLatch, LOW);
  for (int8_t b = LED_BYTES - 1; b >= 0; b--) {
    shiftOut(ledData, ledClock, MSBFIRST, ledState[b]);
  }
  digitalWrite(ledLatch, HIGH);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  pinMode(ledLatch, OUTPUT);
  pinMode(ledClock, OUTPUT);
  pinMode(ledData, OUTPUT);
  digitalWrite(ledLatch, LOW);
  digitalWrite(ledClock, LOW);
  digitalWrite(ledData, LOW);

  Serial.begin(115200);
  Serial.println("ADIRS_DIAG:READY;");
  applyADIRS();
}

// ============================================================================
// SERIAL PARSER
// ============================================================================
char serialBuf[64] = "";

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') c = ';';

    int len = strlen(serialBuf);
    if (len < 63) { serialBuf[len] = c; serialBuf[len + 1] = 0; }

    char* semi;
    while ((semi = strchr(serialBuf, ';')) != NULL) {
      *semi = 0;
      char* tok = serialBuf;
      while (*tok == ' ') tok++;

      if (tok[0] == 0) {
        // empty token, skip
      } else {
        char* colon = strchr(tok, ':');
        if (colon) {
          *colon = 0;
          char* key = tok;
          char* val = colon + 1;

          // Trim key
          while (*key == ' ') key++;
          int ke = strlen(key) - 1;
          while (ke >= 0 && key[ke] == ' ') key[ke--] = 0;

          // Trim val
          while (*val == ' ') val++;
          int ve = strlen(val) - 1;
          while (ve >= 0 && val[ve] == ' ') val[ve--] = 0;

          // LED1 / LED2
          if (key[0] == 'L' && key[1] == 'E' && key[2] == 'D') {
            int idx = atoi_s(key + 3) - 1;  // 1-based → 0-based
            if (idx >= 0 && idx < LED_BYTES) {
              ledState[idx] = parseBin8(val);
              applyADIRS();
              char b8[9];
              bin8(b8, ledState[idx]);
              Serial.print("OK:LED"); Serial.print(idx + 1);
              Serial.print("="); Serial.println(b8);
            }
          }
          // VER / IDENT
          else if (strcmp(key, "VER") == 0 || strcmp(key, "IDENT") == 0) {
            Serial.println("IDENT:ADIRS_DIAG v1.0;");
          }
          // REQ – dump current state
          else if (strcmp(key, "REQ") == 0) {
            for (int i = 0; i < LED_BYTES; i++) {
              char b8[9]; bin8(b8, ledState[i]);
              Serial.print("LED"); Serial.print(i + 1);
              Serial.print(":"); Serial.print(b8); Serial.print(";");
            }
            Serial.println();
          }
        } else {
          // key-only commands
          if (strcmp(tok, "VER") == 0 || strcmp(tok, "IDENT") == 0) {
            Serial.println("IDENT:ADIRS_DIAG v1.0;");
          } else if (strcmp(tok, "REQ") == 0) {
            for (int i = 0; i < LED_BYTES; i++) {
              char b8[9]; bin8(b8, ledState[i]);
              Serial.print("LED"); Serial.print(i + 1);
              Serial.print(":"); Serial.print(b8); Serial.print(";");
            }
            Serial.println();
          }
        }
      }

      // Remove processed token, keep remainder
      int remaining = strlen(semi + 1);
      memmove(serialBuf, semi + 1, remaining + 1);
    }
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  processSerial();
}
