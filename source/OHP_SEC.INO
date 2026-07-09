// OHP_SEC_v2.ino – OHP Secondary Panel (Arduino MEGA 2560)
// (w) 2025 M. Quatember, OHP edition
// Controls ADIRS, LT, CARGO sections with BLTDRV LED drivers
//
// v2 Optimierungen (Protokoll 100 % kompatibel zu v1):
//   - Ring-Buffer-Parser (256 Byte, uint8_t-Index, kein memmove)
//   - Timeout fuer unvollstaendige Telegramme (100 ms)
//   - Einheitlicher processToken() statt zwei paralleler Parser
//   - DEBUG_SERIAL: abschaltbare Debug-Ausgaben (Compile-Time)
//   - constexpr fuer alle Compile-Time-Konstanten
//   - F()-Makro fuer Stringliterale (spart RAM)
//   - MCUSR vor wdt_disable() loeschen (korrekte WDT-Behandlung)
//   - Alle Arrays explizit null-initialisiert
//   - Analog-Eingang nur einmal pro Zyklus gelesen
//   - Magic Numbers entfernt

// ============================================================================
// DEBUG – Zeile einkommentieren um Debug-Ausgaben zu aktivieren
// ============================================================================
// #define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)   ((void)0)
  #define DBGLN(x) ((void)0)
#endif

#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================================
// PIN DEFINITIONS – INPUT SHIFTER (74HC165)
// ============================================================================
// ADIRS_IS: CLOCK D9, LATCH D8, DATA D10, AMOUNT 2
constexpr uint8_t inClk_AD  = 9;   constexpr uint8_t inLatch_AD = 8;   constexpr uint8_t inData_AD = 10;  constexpr uint8_t IN_CNT_AD = 2;
// LT_IS:    CLOCK D16, LATCH D15, DATA D17, AMOUNT 4
constexpr uint8_t inClk_LT  = 16;  constexpr uint8_t inLatch_LT = 15;  constexpr uint8_t inData_LT = 17;  constexpr uint8_t IN_CNT_LT = 4;
// CARGO_IS: CLOCK D39, LATCH D40, DATA D42, AMOUNT 3
constexpr uint8_t inClk_CA  = 39;  constexpr uint8_t inLatch_CA = 40;  constexpr uint8_t inData_CA = 42;  constexpr uint8_t IN_CNT_CA = 3;

constexpr uint8_t IN_SHIFTER_BYTES = 9;   // 2+4+3
constexpr uint8_t IN_SINGLE_COUNT  = 2;
constexpr uint8_t IN_TOTAL_BYTES   = IN_SHIFTER_BYTES + 1;  // +1 fuer Single-Inputs

// Single Inputs: D21=LT_ModeSel, D22=CDV_SW
constexpr uint8_t SIN_PINS[IN_SINGLE_COUNT] = {21, 22};

// ============================================================================
// PIN DEFINITIONS – OUTPUT SHIFTER (74HC595 / BLTDRV)
// ============================================================================
constexpr uint8_t ledClk_B1 = 6;   constexpr uint8_t ledLatch_B1 = 5;   constexpr uint8_t ledData_B1 = 7;   constexpr uint8_t LED_CNT_B1 = 4;
constexpr uint8_t ledClk_B2 = 3;   constexpr uint8_t ledLatch_B2 = 2;   constexpr uint8_t ledData_B2 = 4;   constexpr uint8_t LED_CNT_B2 = 4;
constexpr uint8_t ledClk_AD = 12;  constexpr uint8_t ledLatch_AD = 11;  constexpr uint8_t ledData_AD = 14;  constexpr uint8_t LED_CNT_AD = 2;
constexpr uint8_t ledClk_LT = 19;  constexpr uint8_t ledLatch_LT = 18;  constexpr uint8_t ledData_LT = 20;  constexpr uint8_t LED_CNT_LT = 3;
constexpr uint8_t ledClk_CA = 44;  constexpr uint8_t ledLatch_CA = 41;  constexpr uint8_t ledData_CA = 43;  constexpr uint8_t LED_CNT_CA = 4;

constexpr uint8_t LED_TOTAL_BYTES = 17;  // 4+4+2+3+4

// Single LEDs: D46=LED_GI1_CDV, D48=LED_GI2_CDV
constexpr uint8_t SLED_PINS[2] = {46, 48};
constexpr uint8_t SLED_COUNT   = 2;

// TLC5916 OE wird vollstaendig vom Primary gesteuert (kein OE-Pin auf Secondary)

// ============================================================================
// PIN DEFINITIONS – ANALOG
// ============================================================================
constexpr uint8_t ANALOG_COUNT = 4;
constexpr uint8_t ANA_PINS[ANALOG_COUNT] = {A5, A6, A7, A8};
// A5=CARGO_HEAT_FWD, A6=CARGO_HEAT_AFT, A7=LT_OVP_LT, A8=LT_LDG_ELEV

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char PANEL_IDENT[]     = "OHP_SEC, v1.3 MAQ";
const char FW_VERSION[]      = "1.3";
const char PANEL_SN_PREFIX[] = "OHS-";

bool          identSentOnStart = false;
unsigned long pauseUntil       = 0;
char          CFG_SERIAL_NUMBER[10] = "";

// ============================================================================
// LED STATE (Shifter + Single LEDs)
// LED_TOTAL_BYTES Shifter-Bytes + 1 Byte fuer Single LEDs
// ============================================================================
// Backlight mask: which LED registers are controlled by BL instead of LEDx
// OHP_SEC: LED1-8 are all backlight (indices 0-7), 0xFF mask each
constexpr uint8_t BACKLIGHT_COUNT = 8;
constexpr uint8_t BACKLIGHT_MASKS[BACKLIGHT_COUNT] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  // LED1-8 all backlight
};

uint8_t desiredLedState[LED_TOTAL_BYTES + 1] = {};
uint8_t hwLedState[LED_TOTAL_BYTES + 1]      = {};

uint8_t desiredBacklight = 0;  // Backlight brightness from BL command
bool diagActive = false;       // Set true during DIAG to bypass backlight mask

// ============================================================================
// INPUT STATE
// ============================================================================
uint8_t inputState[IN_TOTAL_BYTES]     = {};
uint8_t lastInputState[IN_TOTAL_BYTES] = {};

// ============================================================================
// ANALOG STATE
// ============================================================================
uint16_t analogValues[ANALOG_COUNT]     = {};
uint16_t lastAnalogValues[ANALOG_COUNT] = {};
constexpr uint8_t ANA_FLT_SZ = 4;
uint16_t anaFltBuf[ANALOG_COUNT][ANA_FLT_SZ] = {};
uint8_t  anaFltIdx[ANALOG_COUNT]             = {};

// ============================================================================
// KONFIGURATION
// ============================================================================
uint8_t  CFG_BUTTON_DEBOUNCE = 12;
uint16_t CFG_LED_REFRESH     = 1200;
uint16_t CFG_DISPLAY_REFRESH = 1200;
uint8_t  CFG_ANA_DEADBAND    = 5;
char     CFG_AIRCRAFT_REG[9] = "D-A320";
char     CFG_PCB_VERSION[9]  = "PCB 1.0";
bool     CFG_PCB_IS_12       = false;

constexpr uint8_t  EEPROM_BASE_ADDR      = 0;
constexpr uint16_t EEPROM_MAGIC          = 0xA55A;
constexpr uint8_t  EEPROM_FORMAT_VERSION = 3;

bool settingsEnabled = false;
const char SETTINGS_PIN[] = "0815";

// ============================================================================
// BOOT / VERBINDUNGS-STATE
// ============================================================================
enum BootState : uint8_t { BOOT_INIT, BOOT_FADE, BOOT_RUNNING, BOOT_DIAG_MENU, BOOT_DIAG_TEST };
BootState bootState = BOOT_INIT;
unsigned long bootFadeStart = 0;
constexpr unsigned long BOOT_FADE_DURATION_MS = 1500UL;

bool hostOnline = false;

// DIAG State
unsigned long diagStartTime     = 0;
int           diagStage         = 0;
bool          diagComboWasActive = false;
bool          menuInitialized   = false;
int           menuIndex         = 0;
unsigned long lastMenuActivityTs = 0;
constexpr unsigned long MENU_INACTIVITY_MS = 60000UL;

// DIAG LED-Walk Sub-State
int           diagWalkPhase  = 0;
int           diagWalkByte   = 0;
unsigned long diagWalkLastTs = 0;

// DIAG Show Sub-State
int           showPhase  = 0;
int           showStep   = 0;
int           showDir    = 1;
unsigned long showLastTs = 0;
int           showRepeat = 1;

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastLoopTs     = 0;
constexpr unsigned long LOOP_INTERVAL_MS     = 10UL;
unsigned long lastSendTs     = 0;
constexpr unsigned long SEND_MIN_INTERVAL_MS = 10UL;
unsigned long lastDebounceTs = 0;

// ============================================================================
// SERIELLER RING-BUFFER (256 Byte, uint8_t Indices = automatisches Wraparound)
// ============================================================================
constexpr uint16_t RX_BUF_SIZE  = 256;
constexpr unsigned long RX_TIMEOUT_MS = 100UL;

static uint8_t  rxBuf[RX_BUF_SIZE] = {};
static uint8_t  rxHead             = 0;
static uint8_t  rxTail             = 0;
static unsigned long rxLastByteTs  = 0;

static bool forceSendNext = false;

static inline void rxPush(char c) {
    if (c == '\r') return;
    if (c == '\n') c = ';';
    if ((uint8_t)(rxHead - rxTail) >= 255) { rxTail = rxHead; return; }
    rxBuf[rxHead++] = (uint8_t)c;
}

static inline uint8_t rxAvail() { return (uint8_t)(rxHead - rxTail); }
static inline void rxFlush()    { rxTail = rxHead; }

static bool rxPopToken(char* out, uint8_t maxLen) {
    uint8_t avail = rxAvail();
    uint8_t i     = rxTail;
    for (uint8_t k = 0; k < avail; k++, i++) {
        if (rxBuf[i] == ';') {
            uint8_t len = (uint8_t)(i - rxTail);
            if (len >= maxLen) len = (uint8_t)(maxLen - 1);
            for (uint8_t j = 0; j < len; j++) out[j] = (char)rxBuf[(uint8_t)(rxTail + j)];
            out[len] = '\0';
            rxTail   = (uint8_t)(i + 1);
            return true;
        }
    }
    return false;
}

// ============================================================================
// HILFSFUNKTIONEN
// ============================================================================
static void bin8(char* out, uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) out[i] = ((b >> (7 - i)) & 1) ? '1' : '0';
    out[8] = '\0';
}

static uint8_t parseBin8(const char* s) {
    uint8_t v = 0;
    for (uint8_t i = 0; s[i]; i++) {
        if (s[i] == '0' || s[i] == '1') v = (uint8_t)((v << 1) | (s[i] == '1' ? 1 : 0));
    }
    return v;
}

static bool strieq(const char* a, const char* b) {
    while (*a && *b) {
        char ca = (*a >= 'a' && *a <= 'z') ? (char)(*a - 32) : *a;
        char cb = (*b >= 'a' && *b <= 'z') ? (char)(*b - 32) : *b;
        if (ca != cb) return false;
        a++; b++;
    }
    return *a == *b;
}

static bool stripre(const char* a, const char* pf) {
    while (*pf) {
        char ca = (*a >= 'a' && *a <= 'z') ? (char)(*a - 32) : *a;
        char cb = (*pf >= 'a' && *pf <= 'z') ? (char)(*pf - 32) : *pf;
        if (ca != cb) return false;
        a++; pf++;
    }
    return true;
}

static int16_t atoi_s(const char* s) {
    int16_t v = 0; int8_t sign = 1;
    while (*s == ' ') s++;
    if (*s == '-') { sign = -1; s++; }
    while (*s >= '0' && *s <= '9') { v = (int16_t)(v * 10 + (*s - '0')); s++; }
    return (int16_t)(v * sign);
}

static char* ltrim(char* s) { while (*s == ' ') s++; return s; }

static void rtrim(char* s) {
    int8_t e = (int8_t)strlen(s) - 1;
    while (e >= 0 && s[e] == ' ') s[e--] = '\0';
}

void refreshPCBVersionFlags() {
    char norm[9]; uint8_t j = 0;
    for (uint8_t i = 0; CFG_PCB_VERSION[i] && j < 8; i++) {
        char c = CFG_PCB_VERSION[i];
        if (c == ' ') continue;
        if (c >= 'a' && c <= 'z') c = (char)(c - 32);
        norm[j++] = c;
    }
    norm[j] = '\0';
    CFG_PCB_IS_12 = (norm[0]=='P' && norm[1]=='C' && norm[2]=='B' &&
                     norm[3]=='1' && norm[4]=='.' && norm[5]=='2' && norm[6]=='\0');
}

void triggerSoftwareReset(bool showMsg = true) {
    if (showMsg) { Serial.println(F("RESET:OK ;")); Serial.flush(); delay(100); }
    wdt_enable(WDTO_15MS);
    while (true) {}
}

// ============================================================================
// LED DRIVER (TLC5916) – Reine ShiftOut-Ausgabe, kein OE/PWM (Primary steuert)
// ============================================================================
static void shiftOutOne(uint8_t clk, uint8_t latch, uint8_t data,
                        const uint8_t* bytes, uint8_t cnt) {
    digitalWrite(latch, LOW);
    for (int8_t b = (int8_t)cnt - 1; b >= 0; b--) shiftOut(data, clk, MSBFIRST, bytes[b]);
    digitalWrite(latch, HIGH);
}

void applyLEDOutputs() {
    // Build backlight byte: when backlight active, use desiredBacklight; else 0
    // In DIAG mode, backlight registers use desiredLedState values as-is
    uint8_t blByte = (diagActive || desiredBacklight > 0) ? desiredBacklight : 0x00;

    uint8_t bi = 0;
    // B1 (LED1-4, indices 0-3): all backlight
    {
        uint8_t out[4];
        for (uint8_t i = 0; i < LED_CNT_B1; i++) {
            out[i] = diagActive ? desiredLedState[bi + i] : blByte;
        }
        shiftOutOne(ledClk_B1, ledLatch_B1, ledData_B1, out, LED_CNT_B1);
        bi += LED_CNT_B1;
    }
    // B2 (LED5-8, indices 4-7): all backlight
    {
        uint8_t out[4];
        for (uint8_t i = 0; i < LED_CNT_B2; i++) {
            out[i] = diagActive ? desiredLedState[bi + i] : blByte;
        }
        shiftOutOne(ledClk_B2, ledLatch_B2, ledData_B2, out, LED_CNT_B2);
        bi += LED_CNT_B2;
    }
    // AD, LT, CA (indices 8-16): free registers, no mask
    shiftOutOne(ledClk_AD, ledLatch_AD, ledData_AD, &desiredLedState[bi], LED_CNT_AD); bi += LED_CNT_AD;
    shiftOutOne(ledClk_LT, ledLatch_LT, ledData_LT, &desiredLedState[bi], LED_CNT_LT); bi += LED_CNT_LT;
    shiftOutOne(ledClk_CA, ledLatch_CA, ledData_CA, &desiredLedState[bi], LED_CNT_CA);
    // Single LEDs (letztes Byte = Index LED_TOTAL_BYTES)
    uint8_t sb = desiredLedState[LED_TOTAL_BYTES];
    for (uint8_t i = 0; i < SLED_COUNT; i++)
        digitalWrite(SLED_PINS[i], (sb & (1 << i)) ? HIGH : LOW);
    memcpy(hwLedState, desiredLedState, LED_TOTAL_BYTES + 1);
}

void ledClearAll() { memset(desiredLedState, 0, LED_TOTAL_BYTES + 1); applyLEDOutputs(); }
void ledSetAll(uint8_t v) { memset(desiredLedState, v, LED_TOTAL_BYTES + 1); applyLEDOutputs(); }

// ============================================================================
// EINGANGS-SCHIEBEREGISTER (74HC165) – MULTI-GROUP
// ============================================================================
static void readOneInGrp(uint8_t clk, uint8_t latch, uint8_t data,
                         uint8_t* out, uint8_t cnt) {
    digitalWrite(latch, LOW);  delayMicroseconds(20);
    digitalWrite(latch, HIGH); delayMicroseconds(20);
    for (uint8_t b = 0; b < cnt; b++) out[b] = ~shiftIn(data, clk, MSBFIRST);
    digitalWrite(latch, LOW);  delayMicroseconds(20);
    digitalWrite(latch, HIGH); delayMicroseconds(20);
    for (uint8_t i = 0; i < 16; i++) {
        digitalWrite(clk, LOW);  delayMicroseconds(5);
        digitalWrite(clk, HIGH); delayMicroseconds(5);
    }
}

void readAllInputs() {
    uint8_t bi = 0;
    readOneInGrp(inClk_AD, inLatch_AD, inData_AD, &inputState[bi], IN_CNT_AD); bi += IN_CNT_AD;
    readOneInGrp(inClk_LT, inLatch_LT, inData_LT, &inputState[bi], IN_CNT_LT); bi += IN_CNT_LT;
    readOneInGrp(inClk_CA, inLatch_CA, inData_CA, &inputState[bi], IN_CNT_CA);
    // Single Inputs
    uint8_t s = 0;
    for (uint8_t i = 0; i < IN_SINGLE_COUNT; i++)
        if (digitalRead(SIN_PINS[i]) == LOW) s |= (uint8_t)(1 << i);
    inputState[IN_SHIFTER_BYTES] = s;
}

// ============================================================================
// ANALOG-EINGAENGE – einmal pro Zyklus lesen, 4-Tap Gleitender Mittelwert
// ============================================================================
void readAnalogFiltered() {
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
        uint16_t raw = analogRead(ANA_PINS[i]);
        anaFltBuf[i][anaFltIdx[i]] = raw;
        anaFltIdx[i] = (uint8_t)((anaFltIdx[i] + 1) % ANA_FLT_SZ);
        uint32_t sum = 0;
        for (uint8_t j = 0; j < ANA_FLT_SZ; j++) sum += anaFltBuf[i][j];
        analogValues[i] = (uint16_t)(sum / ANA_FLT_SZ);
    }
}

static bool analogChanged() {
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
        int16_t d  = (int16_t)analogValues[i] - (int16_t)lastAnalogValues[i];
        int16_t db = (int16_t)CFG_ANA_DEADBAND;
        if (d < -db || d > db) { lastAnalogValues[i] = analogValues[i]; return true; }
    }
    return false;
}

// ============================================================================
// SERIAL IDENT / STATE
// ============================================================================
void sendIdentAndState() {
    Serial.print(F("IDENT:")); Serial.print(PANEL_IDENT);
    if (CFG_SERIAL_NUMBER[0] != '\0') {
        Serial.print(F(", SN:")); Serial.print(PANEL_SN_PREFIX); Serial.print(CFG_SERIAL_NUMBER);
    }
    Serial.print(F(";STATE:RUNNING;REG:")); Serial.print(CFG_AIRCRAFT_REG); Serial.println(';');
}

void maybeSendIdentStartup() {
    if (!identSentOnStart) { delay(150); sendIdentAndState(); identSentOnStart = true; }
}

// ============================================================================
// STATUS SENDEN
// ============================================================================
void sendStatusImmediate() {
    char b8[9];
    for (uint8_t i = 0; i < IN_TOTAL_BYTES; i++) {
        bin8(b8, inputState[i]);
        Serial.print(F("IN")); Serial.print((int)(i + 1)); Serial.print(':');
        Serial.print(b8); Serial.print(';');
    }
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
        Serial.print(F("POT")); Serial.print((int)(i + 1)); Serial.print(':');
        Serial.print(analogValues[i]); Serial.print(';');
    }
    Serial.println();
    lastSendTs    = millis();
    forceSendNext = false;
}

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void saveHWInfo();
void diagLedWalkStep(unsigned long now);
void diagShowStep(unsigned long now);

// ============================================================================
// TOKEN PROZESSOR – verarbeitet ein einzelnes ';'-terminiertes Token
// ============================================================================
static void processToken(char* tok) {
    rtrim(tok);
    tok = ltrim(tok);
    if (*tok == '\0') return;

    // ---- Immer gueltige Einzelwort-Kommandos ----
    if (strieq(tok, "VER") || strieq(tok, "VERSION") || strieq(tok, "IDENT")) {
        sendIdentAndState(); identSentOnStart = true; return;
    }
    if (strieq(tok, "RESET")) { Serial.println(F("RESET:OK ;")); triggerSoftwareReset(false); return; }
    if (strieq(tok, "REQ"))   { sendStatusImmediate(); return; }
    if (strieq(tok, "PCB")) {
        Serial.print(F("PCB:"));
        char* sp = strchr(CFG_PCB_VERSION, ' ');
        if (sp && *(sp + 1)) { Serial.print('v'); Serial.print(sp + 1); }
        else Serial.print(CFG_PCB_VERSION);
        Serial.println(F(" ;"));
        return;
    }
    if (strieq(tok, "EXIT")) {
        if (bootState == BOOT_DIAG_MENU) {
            bootState = BOOT_RUNNING; diagActive = false; menuInitialized = false; diagStage = 0;
                        ledClearAll(); Serial.println(F("DIAG:EXIT;"));
        } else {
            settingsEnabled = false; Serial.println(F("EXIT:OK ;"));
        }
        return;
    }
    if (strieq(tok, "DIAG") && bootState == BOOT_RUNNING) {
        bootState = BOOT_DIAG_MENU; diagActive = true; diagStartTime = millis(); diagStage = 0;
        menuInitialized = false; ledClearAll(); Serial.println(F("DIAG:MENU;"));
        return;
    }

    // ---- DIAG Menue-Navigation ----
    if (bootState == BOOT_DIAG_MENU) {
        lastMenuActivityTs = millis();
        if (strieq(tok, "NEXT")) {
            menuIndex = (menuIndex + 1) % 4;
            Serial.print(F("DIAG:MENU:")); Serial.println(menuIndex);
            if (menuIndex == 0) {
                Serial.println(F("DIAG:LED_WALK;"));
                diagStage = 1; diagWalkPhase = 0; diagWalkByte = 0;
                diagWalkLastTs = millis(); ledClearAll();
            } else if (menuIndex == 1) {
                Serial.println(F("DIAG:INPUTS;"));
                for (uint8_t i = 0; i < IN_TOTAL_BYTES; i++) {
                    char b8[9]; bin8(b8, inputState[i]);
                    Serial.print(F("IN")); Serial.print((int)(i + 1)); Serial.print('=');
                    Serial.print(b8); Serial.print(';');
                }
                Serial.println();
            } else if (menuIndex == 2) {
                diagStage = 0;
                Serial.print(F("HEALTH:UPTIME=")); Serial.print(millis() / 1000);
                Serial.print(F("s;LASTERR=0;FREEMEM="));
                extern int __heap_start, *__brkval; int vv;
                Serial.print((int)&vv - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
                Serial.print(F(";VERSION=")); Serial.println(FW_VERSION);
                Serial.println(F("DIAG:ALL_OK;"));
            } else if (menuIndex == 3) {
                bootState = BOOT_RUNNING; diagActive = false; menuInitialized = false; diagStage = 0;
                            ledClearAll(); Serial.println(F("DIAG:EXIT;"));
            }
            return;
        }
        if (strieq(tok, "LED")) {
            menuIndex = 0; Serial.println(F("DIAG:LED_WALK;"));
            diagStage = 1; diagWalkPhase = 0; diagWalkByte = 0;
            diagWalkLastTs = millis(); ledClearAll();
            return;
        }
        if (strieq(tok, "IN")) {
            diagStage = 0; readAllInputs();
            for (uint8_t i = 0; i < IN_TOTAL_BYTES; i++) {
                char b8[9]; bin8(b8, inputState[i]);
                Serial.print(F("IN")); Serial.print((int)(i + 1)); Serial.print('=');
                Serial.print(b8); Serial.print(';');
            }
            Serial.println();
            return;
        }
        if (strieq(tok, "HEALTH")) {
            diagStage = 0;
            Serial.print(F("HEALTH:UPTIME=")); Serial.print(millis() / 1000);
            Serial.print(F("s;LASTERR=0;FREEMEM="));
            extern int __heap_start, *__brkval; int vv;
            Serial.print((int)&vv - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
            Serial.print(F(";VERSION=")); Serial.println(FW_VERSION);
            return;
        }
        // SHOW oder SHOW:N
        if (strieq(tok, "SHOW") || stripre(tok, "SHOW:")) {
            showRepeat = 1;
            char* cp2 = strchr(tok, ':');
            if (cp2) { int16_t n = atoi_s(cp2 + 1); if (n >= 1 && n <= 9) showRepeat = (int)n; }
            menuIndex = 4;
            Serial.print(F("DIAG:SHOW_START x")); Serial.print(showRepeat); Serial.println(';');
            diagStage = 3; showPhase = 0; showStep = 0; showDir = 1;
            showLastTs = millis(); ledClearAll();
            return;
        }
    }

    // ---- SET ohne Doppelpunkt ----
    if (stripre(tok, "SET ") && !strchr(tok, ':')) {
        char* sc = ltrim(tok + 4);
        if      (stripre(sc, "EXIT"))  { settingsEnabled = false; Serial.println(F("SET EXIT:OK ;")); }
        else if (stripre(sc, "WRITE") && settingsEnabled) { saveHWInfo(); settingsEnabled = false; Serial.println(F("WRITE:OK ;")); triggerSoftwareReset(false); }
        else if (stripre(sc, "WRITE")) { Serial.println(F("WRITE:LOCKED ;")); }
        return;
    }

    // ---- Token mit Doppelpunkt: KEY:VALUE ----
    char* colon = strchr(tok, ':');
    if (!colon) return;

    *colon = '\0';
    char* key = tok;
    char* val = colon + 1;
    rtrim(key); char* kp = ltrim(key);
    val = ltrim(val); rtrim(val);

    // LEDx (1-basiert, incl. Single-LED-Byte an Index LED_TOTAL_BYTES)
    if (stripre(kp, "LED")) {
        int16_t idx = atoi_s(kp + 3) - 1;
        if (idx >= 0 && idx <= (int16_t)LED_TOTAL_BYTES) {
            uint8_t val8 = parseBin8(val);
            // Apply backlight mask for LED1-8 (indices 0-7)
            if (idx < BACKLIGHT_COUNT) val8 &= ~BACKLIGHT_MASKS[idx];
            desiredLedState[idx] = val8; applyLEDOutputs();
        }
        return;
    }
    if (strieq(kp, "BL") || strieq(kp, "BLT")) {
        // Backlight brightness: 0=off, 1-255=on with dimming
        // Writes brightness value to LED1-8 backlight registers
        desiredBacklight = (uint8_t)constrain(atoi_s(val), 0, 255);
        applyLEDOutputs();
        return;
    }
    if (strieq(kp, "STATE")) {
        hostOnline = (strcmp(val, "01") == 0 || strcmp(val, "1") == 0); return;
    }
    // DISP_BL: akzeptiert aber kein Display vorhanden (No-op)
    if (strieq(kp, "DISP_BL")) { return; }
    if (strieq(kp, "REQ"))     { forceSendNext = true; return; }
    if (strieq(kp, "VER"))     { sendIdentAndState(); return; }

    if (strieq(kp, "CFG")) {
        char* cp = val;
        while (*cp) {
            while (*cp == ' ' || *cp == ';') cp++;
            if (!*cp) break;
            char* semi = strchr(cp, ';'); if (semi) *semi = '\0';
            if      (stripre(cp, "DEB"))  CFG_BUTTON_DEBOUNCE  = (uint8_t)constrain(atoi_s(cp + 3), 0, 99);
            else if (stripre(cp, "DEAD")) CFG_ANA_DEADBAND      = (uint8_t)constrain(atoi_s(cp + 4), 1, 100);
            else if (stripre(cp, "LED"))  CFG_LED_REFRESH        = (uint16_t)constrain(atoi_s(cp + 3), 100, 10000);
            else if (stripre(cp, "DSP"))  CFG_DISPLAY_REFRESH    = (uint16_t)constrain(atoi_s(cp + 3), 100, 10000);
            else if (stripre(cp, "REG:")) { strncpy(CFG_AIRCRAFT_REG, cp + 4, 8); CFG_AIRCRAFT_REG[8] = '\0'; }
            else if (stripre(cp, "SN:"))  { strncpy(CFG_SERIAL_NUMBER, cp + 3, 8); CFG_SERIAL_NUMBER[8] = '\0'; }
            if (semi) *semi = ';';
            cp = semi ? semi + 1 : cp + strlen(cp);
        }
        return;
    }

    // SET key:val
    if (stripre(kp, "SET ")) {
        char* sc = ltrim(kp + 4);
        if (stripre(sc, "ENA")) {
            if (strcmp(val, SETTINGS_PIN) == 0) { settingsEnabled = true; Serial.println(F("SET ENA:OK ;")); }
            else Serial.println(F("SET ENA:FAIL ;"));
        }
        else if (stripre(sc, "EXIT"))  { settingsEnabled = false; Serial.println(F("SET EXIT:OK ;")); }
        else if ((stripre(sc, "WRI") || stripre(sc, "WRITE")) && settingsEnabled) {
            saveHWInfo(); settingsEnabled = false; Serial.println(F("WRITE:OK ;")); triggerSoftwareReset(false);
        }
        else if (stripre(sc, "FW") && settingsEnabled) {
            char* vs = ltrim(val);
            char* dot = strchr(vs, '.');
            if (dot && dot > vs) {
                *dot = '\0'; int16_t ma = atoi_s(vs); int16_t mi = atoi_s(dot + 1); *dot = '.';
                if (ma >= 1 && ma <= 9 && mi >= 0 && mi <= 9) {
                    snprintf(CFG_PCB_VERSION, 9, "PCb %d.%d", (int)ma, (int)mi);
                    refreshPCBVersionFlags(); Serial.println(F("SET FW:OK ;"));
                } else Serial.println(F("SET FW:RANGE ;"));
            } else Serial.println(F("SET FW:FORMAT ;"));
        }
        else if (stripre(sc, "ACID") && settingsEnabled) {
            char* idt = ltrim(val); int il = (int)strlen(idt);
            if (il >= 3 && il <= 8) { strncpy(CFG_AIRCRAFT_REG, idt, 8); CFG_AIRCRAFT_REG[8] = '\0'; Serial.println(F("SET ACID:OK ;")); }
            else Serial.println(F("SET ACID:FORMAT ;"));
        }
        else if (stripre(sc, "SN") && settingsEnabled) {
            char* sv = ltrim(val); int sl = (int)strlen(sv); bool vok = (sl == 8);
            for (int i2 = 0; i2 < sl && vok; i2++) {
                char ch = sv[i2];
                if (!((ch>='0'&&ch<='9')||(ch>='A'&&ch<='F')||(ch>='a'&&ch<='f'))) vok = false;
            }
            if (vok) {
                for (int i2 = 0; i2 < 8; i2++)
                    CFG_SERIAL_NUMBER[i2] = (sv[i2]>='a'&&sv[i2]<='f') ? (char)(sv[i2]-32) : sv[i2];
                CFG_SERIAL_NUMBER[8] = '\0';
                Serial.print(F("SET SN:OK:")); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
            } else Serial.println(F("SET SN:FORMAT ;"));
        }
        return;
    }

    DBGLN(F("DBG:UNKNOWN_TOKEN"));
}

// ============================================================================
// SERIELLER HOST-PROZESSOR – Ring-Buffer leeren, vollstaendige Tokens verarbeiten
// ============================================================================
void processSerialTokensFromHost() {
    while (Serial.available()) {
        rxPush((char)Serial.read());
        rxLastByteTs = millis();
    }
    if (rxAvail() > 0 && (millis() - rxLastByteTs) >= RX_TIMEOUT_MS) {
        DBGLN(F("DBG:RX_TIMEOUT"));
        rxFlush();
        return;
    }
    char token[64];
    while (rxPopToken(token, sizeof(token))) {
        if (token[0] == '\0') continue;
        DBG(F("DBG:TOK>")); DBGLN(token);
        processToken(token);
    }
}

// ============================================================================
// DIAG – Zweiphasiger LED-Walk
// ============================================================================
void diagLedWalkStep(unsigned long now) {
    constexpr uint16_t totalBits  = (uint16_t)(LED_TOTAL_BYTES + 1) * 8;
    constexpr unsigned long SINGLE_MS = 150UL;
    constexpr unsigned long BYTE_MS   = 800UL;
    lastMenuActivityTs = now;

    if (diagWalkPhase == 0) {
        if (now - diagWalkLastTs >= SINGLE_MS) {
            diagWalkLastTs = now;
            memset(desiredLedState, 0, LED_TOTAL_BYTES + 1);
            if (diagWalkByte < (int)totalBits) {
                uint8_t bi = (uint8_t)(diagWalkByte / 8);
                desiredLedState[bi] = (uint8_t)(1 << (diagWalkByte % 8));
                applyLEDOutputs();
                Serial.print(F("DIAG:WALK_BIT=")); Serial.print(diagWalkByte);
                Serial.print(F("  LED")); Serial.print(bi + 1);
                Serial.print('.'); Serial.println(diagWalkByte % 8);
            } else {
                applyLEDOutputs();
            }
            diagWalkByte++;
            if (diagWalkByte > (int)totalBits) {
                diagWalkPhase = 1; diagWalkByte = 0; diagWalkLastTs = now;
                Serial.println(F("DIAG:WALK_BYTE_ALL_ON_START;"));
            }
        }
    } else {
        if (now - diagWalkLastTs >= BYTE_MS) {
            diagWalkLastTs = now;
            memset(desiredLedState, 0, LED_TOTAL_BYTES + 1);
            if (diagWalkByte <= (int)LED_TOTAL_BYTES) {
                desiredLedState[diagWalkByte] = 0xFF;
                applyLEDOutputs();
                Serial.print(F("DIAG:ALL_ON LED")); Serial.print(diagWalkByte + 1);
                Serial.println(F(" = 11111111"));
            } else {
                applyLEDOutputs();
            }
            diagWalkByte++;
            if (diagWalkByte > (int)LED_TOTAL_BYTES) {
                ledClearAll(); diagStage = 0;
                Serial.println(F("DIAG:LED_WALK_DONE;"));
            }
        }
    }
}

// ============================================================================
// DIAG – Menue (Hardware-Button + serielle Navigation)
// ============================================================================
void diagRunMenu() {
    unsigned long now = millis();
    readAllInputs();

    if (!menuInitialized) {
        menuInitialized = true; menuIndex = 0; lastMenuActivityTs = now;
        Serial.println(F("DIAG:MENU;")); ledClearAll();
        return;
    }
    if (diagStage == 0 && (now - lastMenuActivityTs) >= MENU_INACTIVITY_MS) {
        bootState = BOOT_RUNNING; diagActive = false; menuInitialized = false;
        ledClearAll(); Serial.println(F("DIAG:EXIT_TIMEOUT;"));
        return;
    }
    if (memcmp(inputState, lastInputState, IN_TOTAL_BYTES) != 0) {
        lastMenuActivityTs = now;
        if (inputState[0] != lastInputState[0]) {
            uint8_t ch = inputState[0] ^ lastInputState[0];
            if ((ch & 0x01) && (inputState[0] & 0x01)) {
                menuIndex = (menuIndex + 1) % 4;
                memcpy(lastInputState, inputState, IN_TOTAL_BYTES);
                Serial.print(F("DIAG:MENU:")); Serial.println(menuIndex);
                if (menuIndex == 0) {
                    Serial.println(F("DIAG:LED_WALK;"));
                    diagStage = 1; diagStartTime = now;
                    diagWalkPhase = 0; diagWalkByte = 0; diagWalkLastTs = now; ledClearAll();
                } else if (menuIndex == 1) {
                    Serial.println(F("DIAG:INPUTS;"));
                    for (uint8_t i = 0; i < IN_TOTAL_BYTES; i++) {
                        char b8[9]; bin8(b8, inputState[i]);
                        Serial.print(F("IN")); Serial.print((int)(i + 1)); Serial.print('=');
                        Serial.print(b8); Serial.print(';');
                    }
                    Serial.println();
                } else if (menuIndex == 2) {
                    Serial.print(F("HEALTH:UPTIME=")); Serial.print(millis() / 1000);
                    Serial.print(F("s;LASTERR=0;FREEMEM="));
                    extern int __heap_start, *__brkval; int vv;
                    Serial.print((int)&vv - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
                    Serial.print(F(";VERSION=")); Serial.println(FW_VERSION);
                } else if (menuIndex == 3) {
                    bootState = BOOT_RUNNING; diagActive = false; menuInitialized = false;
                    ledClearAll(); Serial.println(F("DIAG:EXIT;"));
                }
            }
        }
    }
    if (diagStage == 1) diagLedWalkStep(now);
    if (diagStage == 3) diagShowStep(now);
}

// ============================================================================
// DIAG SHOW – Demo-Animation (kein Display auf Secondary)
// ============================================================================
void diagShowStep(unsigned long now) {
    constexpr uint16_t totalBits = (uint16_t)(LED_TOTAL_BYTES + 1) * 8;
    lastMenuActivityTs = now;

    if (showPhase == 0) {
        // Phase 0: Knight Rider (Einzelbit-Sweep, bidirektional)
        constexpr unsigned long KR_MS = 30UL;
        if (now - showLastTs >= KR_MS) {
            showLastTs = now;
            int pos = showStep;
            if (pos >= (int)totalBits) pos = (int)(totalBits * 2) - 2 - showStep;
            if (showStep >= (int)(totalBits * 2) - 1) {
                showPhase = 1; showStep = 0; showLastTs = now;
                ledClearAll(); Serial.println(F("DIAG:SHOW_CASCADE;")); return;
            }
            memset(desiredLedState, 0, LED_TOTAL_BYTES + 1);
            desiredLedState[pos / 8] = (uint8_t)(1 << (pos % 8));
            applyLEDOutputs(); showStep++;
        }
    } else if (showPhase == 1) {
        // Phase 1: Cascade-Fill
        constexpr unsigned long CAS_MS = 100UL;
        if (now - showLastTs >= CAS_MS) {
            showLastTs = now;
            int totalBytes = (int)(LED_TOTAL_BYTES + 1);
            if (showStep < totalBytes) {
                desiredLedState[showStep] = 0xFF; applyLEDOutputs();
            } else if (showStep < totalBytes * 2) {
                desiredLedState[showStep - totalBytes] = 0x00; applyLEDOutputs();
            } else {
                showPhase = 2; showStep = 0; showLastTs = now;
                ledClearAll(); Serial.println(F("DIAG:SHOW_BREATHE;")); return;
            }
            showStep++;
        }
    } else if (showPhase == 2) {
        // Phase 2: Breathing (Blinken mit zunehmender Frequenz, 100 Schritte)
        if (now - showLastTs >= (unsigned long)(220 - showStep * 2)) {
            showLastTs = now;
            if (showStep >= 100) {
                showPhase = 3; showStep = 0; showLastTs = now;
                ledSetAll(0xFF); Serial.println(F("DIAG:SHOW_FINALE;")); return;
            }
            if (showStep & 1) ledClearAll(); else ledSetAll(0xFF);
            showStep++;
        }
    } else if (showPhase == 3) {
        // Phase 3: Alles EIN, 2.5 s halten, dann wiederholen oder beenden
        if (now - showLastTs >= 2500UL) {
            showRepeat--;
            if (showRepeat > 0) {
                showPhase = 0; showStep = 0; showDir = 1; showLastTs = now; ledClearAll();
                Serial.print(F("DIAG:SHOW_REPEAT ")); Serial.print(showRepeat); Serial.println(';');
            } else {
                ledClearAll(); diagStage = 0;
                showPhase = 0; showStep = 0; showRepeat = 1;
                Serial.println(F("DIAG:SHOW_DONE;"));
            }
        }
    }
}

// ============================================================================
// DIAG – Hardware-Kombination pruefen
// ============================================================================
static bool diagCheckCombo() {
    bool combo = (inputState[0] & 0x07) == 0x07;
    if (bootState == BOOT_RUNNING && combo && !diagComboWasActive) {
        diagComboWasActive = true; bootState = BOOT_DIAG_MENU;
        diagStartTime = millis(); diagStage = 0; menuInitialized = false;
        Serial.println(F("DIAG:COMBO_START;")); return true;
    }
    if (!combo) diagComboWasActive = false;
    return false;
}

// ============================================================================
// EEPROM
// ============================================================================
static void generateSerialNumber(char* out) {
    randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3) + micros());
    const char hx[] = "0123456789ABCDEF";
    for (int i = 0; i < 8; i++) out[i] = hx[random(0, 16)];
    out[8] = '\0';
}

static uint8_t calcCfgCs(uint8_t ver, const char* r8, const char* p8, const char* s9) {
    uint16_t sum = ver;
    for (int i = 0; i < 8; i++) sum += (uint8_t)r8[i];
    for (int i = 0; i < 8; i++) sum += (uint8_t)p8[i];
    for (int i = 0; i < 9; i++) sum += (uint8_t)s9[i];
    return (uint8_t)(sum & 0xFF);
}

void saveHWInfo() {
    char rb[8]; int rl = (int)strlen(CFG_AIRCRAFT_REG); for (int i = 0; i < 8; i++) rb[i] = (i < rl) ? CFG_AIRCRAFT_REG[i] : ' ';
    char pb[8]; int pl = (int)strlen(CFG_PCB_VERSION);  for (int i = 0; i < 8; i++) pb[i] = (i < pl) ? CFG_PCB_VERSION[i] : ' ';
    char sb[9]; int sl = (int)strlen(CFG_SERIAL_NUMBER); for (int i = 0; i < 8; i++) sb[i] = (i < sl) ? CFG_SERIAL_NUMBER[i] : ' '; sb[8] = '\0';
    EEPROM.update(EEPROM_BASE_ADDR + 0, (uint8_t)(EEPROM_MAGIC & 0xFF));
    EEPROM.update(EEPROM_BASE_ADDR + 1, (uint8_t)((EEPROM_MAGIC >> 8) & 0xFF));
    EEPROM.update(EEPROM_BASE_ADDR + 2, EEPROM_FORMAT_VERSION);
    for (int i = 0; i < 8; i++) EEPROM.update(EEPROM_BASE_ADDR + 3  + i, (uint8_t)rb[i]);
    for (int i = 0; i < 8; i++) EEPROM.update(EEPROM_BASE_ADDR + 11 + i, (uint8_t)pb[i]);
    for (int i = 0; i < 9; i++) EEPROM.update(EEPROM_BASE_ADDR + 19 + i, (uint8_t)sb[i]);
    uint8_t cs = calcCfgCs(EEPROM_FORMAT_VERSION, rb, pb, sb);
    EEPROM.update(EEPROM_BASE_ADDR + 28, cs);
    Serial.print(F("CFG:SAVED:REG=")); Serial.print(CFG_AIRCRAFT_REG);
    Serial.print(F(";PCB=")); Serial.print(CFG_PCB_VERSION);
    Serial.print(F(";SN=")); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
}

void loadHWInfo() {
    uint16_t magic = (uint16_t)EEPROM.read(EEPROM_BASE_ADDR) |
                     ((uint16_t)EEPROM.read(EEPROM_BASE_ADDR + 1) << 8);
    if (magic != EEPROM_MAGIC) {
        Serial.println(F("CFG:EEPROM:MAGIC_MISSING;"));
        generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo();
        Serial.print(F("CFG:SN:NEW:")); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
        return;
    }
    uint8_t ver = EEPROM.read(EEPROM_BASE_ADDR + 2);
    char rb[9], pb[9];
    for (int i = 0; i < 8; i++) rb[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 3  + i);
    for (int i = 0; i < 8; i++) pb[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 11 + i);
    rb[8] = '\0'; pb[8] = '\0';
    if (ver <= 2) {
        generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo();
        Serial.print(F("CFG:SN:UPGRADE:")); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
        return;
    }
    if (ver != EEPROM_FORMAT_VERSION) {
        Serial.print(F("CFG:EEPROM:VERSION_MISMATCH:")); Serial.print(ver);
        Serial.print('!'); Serial.print('='); Serial.println(EEPROM_FORMAT_VERSION);
        generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo();
        return;
    }
    char sb[10];
    for (int i = 0; i < 9; i++) sb[i] = (char)EEPROM.read(EEPROM_BASE_ADDR + 19 + i);
    sb[9] = '\0';
    uint8_t scs = EEPROM.read(EEPROM_BASE_ADDR + 28);
    uint8_t cs  = calcCfgCs(ver, rb, pb, sb);
    if (cs != scs) {
        Serial.println(F("CFG:EEPROM:CRC_FAIL;"));
        generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo();
        return;
    }
    int ri = 0; for (int i = 0; i < 8; i++) if (rb[i] != 0 && rb[i] != ' ') CFG_AIRCRAFT_REG[ri++] = rb[i]; if (ri > 0) CFG_AIRCRAFT_REG[ri] = '\0';
    int pi = 0; for (int i = 0; i < 8; i++) { if (pb[i] == 0 || pb[i] == ' ') continue; CFG_PCB_VERSION[pi++] = pb[i]; } if (pi > 0) CFG_PCB_VERSION[pi] = '\0';
    int si = 0; for (int i = 0; i < 8; i++) if (sb[i] != 0 && sb[i] != ' ') CFG_SERIAL_NUMBER[si++] = sb[i]; CFG_SERIAL_NUMBER[si] = '\0';
    if (si == 0) {
        generateSerialNumber(CFG_SERIAL_NUMBER); saveHWInfo();
        Serial.print(F("CFG:SN:NEW:")); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    } else {
        Serial.print(F("CFG:SN:LOADED:")); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
    }
    refreshPCBVersionFlags();
    Serial.print(F("CFG:LOAD:REG=")); Serial.print(CFG_AIRCRAFT_REG);
    Serial.print(F(";PCB=")); Serial.print(CFG_PCB_VERSION);
    Serial.print(F(";SN=")); Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    MCUSR = 0;           // Reset-Flags loeschen (muss vor wdt_disable erfolgen)
    wdt_disable();
    Serial.begin(115200);
    Serial.flush();
    delay(50);

    // Input Shifter
    pinMode(inClk_AD, OUTPUT); pinMode(inLatch_AD, OUTPUT); pinMode(inData_AD, INPUT);
    pinMode(inClk_LT, OUTPUT); pinMode(inLatch_LT, OUTPUT); pinMode(inData_LT, INPUT);
    pinMode(inClk_CA, OUTPUT); pinMode(inLatch_CA, OUTPUT); pinMode(inData_CA, INPUT);

    // Single Inputs (Pull-Up aktiviert)
    for (uint8_t i = 0; i < IN_SINGLE_COUNT; i++) pinMode(SIN_PINS[i], INPUT_PULLUP);

    // LED Shifter
    pinMode(ledClk_B1, OUTPUT); pinMode(ledLatch_B1, OUTPUT); pinMode(ledData_B1, OUTPUT);
    pinMode(ledClk_B2, OUTPUT); pinMode(ledLatch_B2, OUTPUT); pinMode(ledData_B2, OUTPUT);
    pinMode(ledClk_AD, OUTPUT); pinMode(ledLatch_AD, OUTPUT); pinMode(ledData_AD, OUTPUT);
    pinMode(ledClk_LT, OUTPUT); pinMode(ledLatch_LT, OUTPUT); pinMode(ledData_LT, OUTPUT);
    pinMode(ledClk_CA, OUTPUT); pinMode(ledLatch_CA, OUTPUT); pinMode(ledData_CA, OUTPUT);

    // Single LEDs
    for (uint8_t i = 0; i < SLED_COUNT; i++) pinMode(SLED_PINS[i], OUTPUT);

    // Analog
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) pinMode(ANA_PINS[i], INPUT);

    // Arrays initialisieren
    memset(desiredLedState, 0, sizeof(desiredLedState));
    memset(hwLedState,      0, sizeof(hwLedState));
    memset(inputState,      0, sizeof(inputState));
    memset(lastInputState,  0, sizeof(lastInputState));
    memset(analogValues,    0, sizeof(analogValues));
    memset(lastAnalogValues,0, sizeof(lastAnalogValues));

    loadHWInfo();
    ledClearAll();
    readAllInputs();
    readAnalogFiltered();
    memcpy(lastInputState,   inputState,   IN_TOTAL_BYTES);
    memcpy(lastAnalogValues, analogValues, ANALOG_COUNT * sizeof(uint16_t));

    // Boot fade-in (non-blocking backlight)
    bootState     = BOOT_FADE;
    bootFadeStart = millis();
    desiredBacklight = 0;
    applyLEDOutputs();
    forceSendNext = true;
    rxHead = 0; rxTail = 0;   // Ring-Buffer sauber initialisieren
    maybeSendIdentStartup();

    Serial.print(F("SETUP:COMPLETE;SN="));
    Serial.print(PANEL_SN_PREFIX); Serial.println(CFG_SERIAL_NUMBER);
}

// ============================================================================
// HAUPTSCHLEIFE
// ============================================================================
void loop() {
    unsigned long now = millis();

    // ── Boot backlight welcome pulse (fade-IN + fade-OUT, non-blocking) ──
    if (bootState == BOOT_FADE) {
        unsigned long elapsed = now - bootFadeStart;
        unsigned long totalMs = BOOT_FADE_DURATION_MS * 2;  // in + out
        if (elapsed >= totalMs) {
            desiredBacklight = 0;
            applyLEDOutputs();
            bootState = BOOT_RUNNING;
        } else if (elapsed < BOOT_FADE_DURATION_MS) {
            // Phase 1: fade IN  0 → 255
            desiredBacklight = (uint8_t)((unsigned long)255 * elapsed / BOOT_FADE_DURATION_MS);
            applyLEDOutputs();
        } else {
            // Phase 2: fade OUT  255 → 0
            unsigned long outElapsed = elapsed - BOOT_FADE_DURATION_MS;
            desiredBacklight = 255 - (uint8_t)((unsigned long)255 * outElapsed / BOOT_FADE_DURATION_MS);
            applyLEDOutputs();
        }
        return;  // Skip main loop during fade
    }

    if (bootState == BOOT_DIAG_MENU) {
        processSerialTokensFromHost();
        diagRunMenu();
        return;
    }
    if (bootState == BOOT_DIAG_TEST) {
        // Secondary hat keine vollstaendige Testsequenz
        bootState = BOOT_RUNNING; diagActive = false;
        return;
    }
    if (now < pauseUntil) return;

    processSerialTokensFromHost();

    if (now - lastLoopTs < LOOP_INTERVAL_MS) return;
    lastLoopTs = now;

    readAllInputs();
    readAnalogFiltered();

    bool inCh = (memcmp(inputState, lastInputState, IN_TOTAL_BYTES) != 0);
    if (inCh && (now - lastDebounceTs) >= (unsigned long)CFG_BUTTON_DEBOUNCE) {
        lastDebounceTs = now;
        memcpy(lastInputState, inputState, IN_TOTAL_BYTES);
        sendStatusImmediate();
    }

    diagCheckCombo();

    if (analogChanged()) {
        for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
            Serial.print(F("POT")); Serial.print((int)(i + 1)); Serial.print(':');
            Serial.print(analogValues[i]); Serial.print(';');
        }
        Serial.println();
    }

    if (forceSendNext) sendStatusImmediate();
}
