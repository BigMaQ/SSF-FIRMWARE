// OHP_PRI_v2.ino – OHP Primary Panel (Arduino MEGA 2560)
// (w) 2025 M. Quatember, OHP edition
// Controls HYD_ELEC, FIRE, EMELEC, ELEC, AC sections
//
// v2 Optimierungen (Protokoll 100 % kompatibel zu v1):
//   - Ring-Buffer-Parser (256 Byte, uint8_t-Index, kein memmove)
//   - Timeout fuer unvollstaendige Telegramme (100 ms)
//   - Einheitlicher processToken() statt zwei paralleler Parser
//   - DEBUG_SERIAL: auschaltbare Debug-Ausgaben (Compile-Time)
//   - constexpr fuer alle Compile-Time-Konstanten
//   - F()-Makro fuer Stringliterale (spart RAM)
//   - MCUSR vor wdt_disable() loeschen (korrekte WDT-Behandlung)
//   - Alle Arrays explizit null-initialisiert
//   - Analог-Eingang nur einmal pro Zyklus gelesen
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

#include <LedControl.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================================
// PIN DEFINITIONS – INPUT SHIFTER (74HC165)
// ============================================================================
constexpr uint8_t inClk_HE  = 3;   constexpr uint8_t inLatch_HE = 2;   constexpr uint8_t inData_HE = 4;   constexpr uint8_t IN_CNT_HE = 2;
constexpr uint8_t inClk_FI  = 19;  constexpr uint8_t inLatch_FI = 18;  constexpr uint8_t inData_FI = 14;  constexpr uint8_t IN_CNT_FI = 1;
constexpr uint8_t inClk_EM  = 25;  constexpr uint8_t inLatch_EM = 24;  constexpr uint8_t inData_EM = 26;  constexpr uint8_t IN_CNT_EM = 4;
constexpr uint8_t inClk_EL  = 36;  constexpr uint8_t inLatch_EL = 35;  constexpr uint8_t inData_EL = 37;  constexpr uint8_t IN_CNT_EL = 2;
constexpr uint8_t inClk_AC  = 42;  constexpr uint8_t inLatch_AC = 41;  constexpr uint8_t inData_AC = 43;  constexpr uint8_t IN_CNT_AC = 2;

constexpr uint8_t IN_SHIFTER_BYTES = 11;   // 2+1+4+2+2
constexpr uint8_t IN_TOTAL_BYTES   = IN_SHIFTER_BYTES;

// ============================================================================
// PIN DEFINITIONS – OUTPUT SHIFTER / TLC5916
// ============================================================================
constexpr uint8_t ledClk_HE = 6;   constexpr uint8_t ledLatch_HE = 5;   constexpr uint8_t ledData_HE = 7;   constexpr uint8_t LED_CNT_HE = 3;
constexpr uint8_t ledClk_FI = 16;  constexpr uint8_t ledLatch_FI = 21;  constexpr uint8_t ledData_FI = 17;  constexpr uint8_t LED_CNT_FI = 2;
constexpr uint8_t ledClk_EM = 28;  constexpr uint8_t ledLatch_EM = 27;  constexpr uint8_t ledData_EM = 29;  constexpr uint8_t LED_CNT_EM = 4;
constexpr uint8_t ledClk_EL = 39;  constexpr uint8_t ledLatch_EL = 38;  constexpr uint8_t ledData_EL = 40;  constexpr uint8_t LED_CNT_EL = 3;
constexpr uint8_t ledClk_AC = 45;  constexpr uint8_t ledLatch_AC = 44;  constexpr uint8_t ledData_AC = 46;  constexpr uint8_t LED_CNT_AC = 2;

// LED_TOTAL_BYTES = 14 Shifter-Bytes + 1 Direct-LED-Byte (Index 14)
constexpr uint8_t LED_TOTAL_BYTES = 15;

// Direct LEDs (active-LOW, LED ON = LOW)
// LED14: bit0=D9(eng2_2_off), bit1=D8(eng2_2_fault), bit2=D20(apuagentsqd), bit3=D23(apuagentdisch)
constexpr uint8_t DLED_PINS[4]   = {9, 8, 20, 23};
constexpr uint8_t DLED_COUNT     = 4;
constexpr bool    DLED_INVERT[4] = {false, false, true, true};

// TLC5916 OE (Output Enable) – LOW=aktiv; PWM steuert Helligkeit
constexpr uint8_t OE_ANNU_PIN  = 11;
constexpr uint8_t OE_BACKL_PIN = 10;

uint8_t annuBrightness  = 255;
uint8_t backlBrightness = 255;

// ============================================================================
// PIN DEFINITIONS – MAX7219 DISPLAYS
// ============================================================================
constexpr uint8_t dspClkPin  = 33;
constexpr uint8_t dspDinPin  = 32;
constexpr uint8_t dspLoadPin = 34;
constexpr uint8_t dspNumDev  = 1;

// ============================================================================
// PIN DEFINITIONS – ANALOG
// ============================================================================
constexpr uint8_t ANALOG_COUNT = 6;
constexpr uint8_t ANA_PINS[ANALOG_COUNT] = {A9, A8, A10, A13, A14, A15};
// A9=AC_FWD_Cabin, A8=AC_AFT_Cabin, A10=AC_Cockpit,
// A13=FIRE_ENG1_AGENT1, A14=FIRE_ENG1_AGENT2, A15=FIRE_ENG1_TEST

// ============================================================================
// PANEL IDENTIFICATION
// ============================================================================
const char PANEL_IDENT[]     = "OHP_PRI, v1.2 MAQ";
const char FW_VERSION[]      = "1.2";
const char PANEL_SN_PREFIX[] = "OHP-";

bool          identSentOnStart = false;
unsigned long pauseUntil       = 0;
char          CFG_SERIAL_NUMBER[10] = "";

// ============================================================================
// MAX7219 DISPLAY
// ============================================================================
LedControl lc = LedControl(dspDinPin, dspClkPin, dspLoadPin, dspNumDev);

uint8_t displayBrightness = 15;
uint8_t showSavedDispBr   = 15;

// ============================================================================
// LED STATE
// ============================================================================
uint8_t desiredLedState[LED_TOTAL_BYTES] = {};
uint8_t hwLedState[LED_TOTAL_BYTES]      = {};

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
enum BootState : uint8_t { BOOT_INIT, BOOT_RUNNING, BOOT_DIAG_MENU, BOOT_DIAG_TEST };
BootState bootState = BOOT_INIT;

bool          bootMessageShown = false;
unsigned long bootMessageStart = 0;
constexpr unsigned long BOOT_MSG_DURATION_MS = 5000UL;

bool hostOnline = false;

// DIAG State
unsigned long diagStartTime     = 0;
int           diagStage         = 0;
bool          diagComboWasActive = false;
bool          diagReturnToMenu  = false;
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
unsigned long showLastTs = 0;
int           showRepeat = 1;

// DIAG Display Sub-State
unsigned long diagDspStart = 0;

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
constexpr unsigned long RX_TIMEOUT_MS = 100UL;   // Timeout fuer unvollstaendige Telegramme

static uint8_t  rxBuf[RX_BUF_SIZE] = {};
static uint8_t  rxHead             = 0;   // Naechste Schreibposition
static uint8_t  rxTail             = 0;   // Naechste Leseposition
static unsigned long rxLastByteTs  = 0;   // Zeitstempel letztes empfangenes Byte

static bool forceSendNext = false;

// Byte in Ring-Buffer schreiben (\r ignorieren, \n → ';')
static inline void rxPush(char c) {
    if (c == '\r') return;
    if (c == '\n') c = ';';
    if ((uint8_t)(rxHead - rxTail) >= 255) { rxTail = rxHead; return; }  // Buffer voll: Reset
    rxBuf[rxHead++] = (uint8_t)c;
}

// Verfuegbare Bytes im Buffer
static inline uint8_t rxAvail() { return (uint8_t)(rxHead - rxTail); }

// Buffer leeren (Timeout / Fehler)
static inline void rxFlush() { rxTail = rxHead; }

// Naechstes ';'-terminiertes Token extrahieren.
// Gibt true zurueck wenn ein vollstaendiges Token gefunden wurde.
static bool rxPopToken(char* out, uint8_t maxLen) {
    uint8_t avail = rxAvail();
    uint8_t i     = rxTail;
    for (uint8_t k = 0; k < avail; k++, i++) {
        if (rxBuf[i] == ';') {
            uint8_t len = (uint8_t)(i - rxTail);
            if (len >= maxLen) len = maxLen - 1;
            for (uint8_t j = 0; j < len; j++) out[j] = (char)rxBuf[(uint8_t)(rxTail + j)];
            out[len] = '\0';
            rxTail = (uint8_t)(i + 1);
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

// Pruefen ob String 'a' mit Praefix 'pf' beginnt (case-insensitive)
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
// MAX7219 DISPLAY FUNKTIONEN
// ============================================================================
static uint8_t charTo7Seg(char c) {
    if (c == 'o') return 0b00011101;
    if (c == 'i') return 0b00010000;
    c = (char)toupper(c);
    switch (c) {
        case '0': return 0b01111110; case '1': return 0b00110000; case '2': return 0b01101101;
        case '3': return 0b01111001; case '4': return 0b00110011; case '5': return 0b01011011;
        case '6': return 0b01011111; case '7': return 0b01110000; case '8': return 0b01111111;
        case '9': return 0b01111011; case 'A': return 0b01110111; case 'B': return 0b00011111;
        case 'C': return 0b01001110; case 'D': return 0b00111101; case 'E': return 0b01001111;
        case 'F': return 0b01000111; case 'H': return 0b00110111; case 'I': return 0b00110000;
        case 'L': return 0b00001110; case 'N': return 0b00010101; case 'O': return 0b01111110;
        case 'P': return 0b01100111; case 'R': return 0b00000101; case 'S': return 0b01011011;
        case 'T': return 0b00001111; case 'U': return 0b00111110; case 'Y': return 0b00111011;
        case '-': return 0b00000001; case ' ': return 0b00000000; default:  return 0b00000000;
    }
}

static void updateDisplay(int dev, const char* txt) {
    int tl = (int)strlen(txt), tp = 0;
    for (int dp = 0; dp < 6; dp++) {
        if (tp >= tl) { lc.setRow(dev, 5 - dp, 0); continue; }
        char c = txt[tp];
        if (c == '.') { tp++; if (tp >= tl) { lc.setRow(dev, 5 - dp, 0); continue; } c = txt[tp]; }
        uint8_t pat = charTo7Seg(c);
        if (tp + 1 < tl && txt[tp + 1] == '.') pat |= 0b10000000;
        lc.setRow(dev, 5 - dp, pat);
        tp++;
    }
}

static void dspSetVal(int dev, uint8_t digitOfs, int16_t v, bool blank0) {
    for (int d = 0; d < 3; d++) lc.setRow(dev, digitOfs + d, 0);
    if (blank0 && v == 0) return;
    char buf[4]; bool neg = (v < 0);
    uint16_t av = neg ? (uint16_t)(-v) : (uint16_t)v;
    int8_t p = 2; buf[3] = '\0';
    if (av == 0) buf[p--] = '0';
    else while (av > 0 && p >= 0) { buf[p--] = (char)('0' + av % 10); av /= 10; }
    if (neg && p >= 0) buf[p--] = '-';
    while (p >= 0) buf[p--] = ' ';
    for (int d = 0; d < 3; d++) lc.setRow(dev, digitOfs + (2 - d), charTo7Seg(buf[d]));
}

static int dspEffLen(const char* s) {
    int cnt = 0;
    for (int i = 0; s[i]; i++) if (s[i] != '.') cnt++;
    return cnt;
}

static void dspSetValStr(int dev, uint8_t digitOfs, const char* txt) {
    for (int d = 0; d < 3; d++) lc.setRow(dev, digitOfs + d, 0);
    if (!txt || !txt[0] || dspEffLen(txt) == 0) return;
    int elen   = dspEffLen(txt);
    int srcLen = (int)strlen(txt);
    int srcPos = 0;
    int padLeft = (3 > elen) ? (3 - elen) : 0;
    uint8_t pat[3] = {0, 0, 0};
    for (int d = padLeft; d < 3 && srcPos < srcLen; ) {
        char c = txt[srcPos];
        if (c == '.') { srcPos++; continue; }
        uint8_t p = charTo7Seg(c);
        if (srcPos + 1 < srcLen && txt[srcPos + 1] == '.') p |= 0b10000000;
        pat[d++] = p;
        srcPos++;
    }
    for (int d = 0; d < 3; d++) lc.setRow(dev, digitOfs + (2 - d), pat[d]);
}

static void setDisplayBrightness(uint8_t b) {
    displayBrightness = constrain(b, 0, 15);
    lc.setIntensity(0, displayBrightness);
}

static void initDisplays() {
    lc.shutdown(0, true);
    lc.setScanLimit(0, 7);
    lc.setIntensity(0, displayBrightness);
    lc.clearDisplay(0);
    lc.shutdown(0, false);
    // Startup-Test: alle Segmente fuer 500 ms
    for (int i = 0; i < 6; i++) lc.setRow(0, i, 0b01111111);
    delay(500);
    lc.clearDisplay(0);
}

// ============================================================================
// LED DRIVER (TLC5916) – MULTI-GROUP mit OE/PWM Helligkeitssteuerung
// ============================================================================
static void shiftOutOne(uint8_t clk, uint8_t latch, uint8_t data,
                        const uint8_t* bytes, uint8_t cnt) {
    digitalWrite(latch, LOW);
    for (int8_t b = (int8_t)cnt - 1; b >= 0; b--) shiftOut(data, clk, MSBFIRST, bytes[b]);
    digitalWrite(latch, HIGH);
}

void applyLEDOutputs() {
    uint8_t bi = 0;
    analogWrite(OE_ANNU_PIN,  (uint8_t)(255 - annuBrightness));
    analogWrite(OE_BACKL_PIN, (uint8_t)(255 - backlBrightness));
    shiftOutOne(ledClk_HE, ledLatch_HE, ledData_HE, &desiredLedState[bi], LED_CNT_HE); bi += LED_CNT_HE;
    shiftOutOne(ledClk_FI, ledLatch_FI, ledData_FI, &desiredLedState[bi], LED_CNT_FI); bi += LED_CNT_FI;
    shiftOutOne(ledClk_EM, ledLatch_EM, ledData_EM, &desiredLedState[bi], LED_CNT_EM); bi += LED_CNT_EM;
    shiftOutOne(ledClk_EL, ledLatch_EL, ledData_EL, &desiredLedState[bi], LED_CNT_EL); bi += LED_CNT_EL;
    shiftOutOne(ledClk_AC, ledLatch_AC, ledData_AC, &desiredLedState[bi], LED_CNT_AC);
    // Direct LEDs (byte index 14)
    uint8_t dl = desiredLedState[14];
    for (uint8_t i = 0; i < DLED_COUNT; i++) {
        bool on = (bool)((dl >> i) & 1);
        if (DLED_INVERT[i]) on = !on;
        digitalWrite(DLED_PINS[i], on ? LOW : HIGH);
    }
    memcpy(hwLedState, desiredLedState, LED_TOTAL_BYTES);
}

void ledClearAll() { memset(desiredLedState, 0, LED_TOTAL_BYTES); applyLEDOutputs(); }
void ledSetAll(uint8_t v) { memset(desiredLedState, v, LED_TOTAL_BYTES); applyLEDOutputs(); }

// ============================================================================
// EINGANGS-SCHIEBEREGISTER (74HC165) – MULTI-GROUP
// ============================================================================
static void readOneInGrp(uint8_t clk, uint8_t latch, uint8_t data,
                         uint8_t* out, uint8_t cnt) {
    digitalWrite(latch, LOW);  delayMicroseconds(20);
    digitalWrite(latch, HIGH); delayMicroseconds(20);
    for (uint8_t b = 0; b < cnt; b++) out[b] = ~shiftIn(data, clk, MSBFIRST);
    // Schieberegister zuruecksetzen
    digitalWrite(latch, LOW);  delayMicroseconds(20);
    digitalWrite(latch, HIGH); delayMicroseconds(20);
    for (uint8_t i = 0; i < 16; i++) {
        digitalWrite(clk, LOW);  delayMicroseconds(5);
        digitalWrite(clk, HIGH); delayMicroseconds(5);
    }
}

void readAllInputs() {
    uint8_t bi = 0;
    readOneInGrp(inClk_HE, inLatch_HE, inData_HE, &inputState[bi], IN_CNT_HE); bi += IN_CNT_HE;
    readOneInGrp(inClk_FI, inLatch_FI, inData_FI, &inputState[bi], IN_CNT_FI); bi += IN_CNT_FI;
    readOneInGrp(inClk_EM, inLatch_EM, inData_EM, &inputState[bi], IN_CNT_EM); bi += IN_CNT_EM;
    readOneInGrp(inClk_EL, inLatch_EL, inData_EL, &inputState[bi], IN_CNT_EL); bi += IN_CNT_EL;
    readOneInGrp(inClk_AC, inLatch_AC, inData_AC, &inputState[bi], IN_CNT_AC);
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
void diagRunDisplay(unsigned long now);
void diagShowStep(unsigned long now);
void diagRunFull();

// ============================================================================
// TOKEN PROZESSOR – verarbeitet ein einzelnes ';'-terminiertes Token
// ============================================================================
static void processToken(char* tok) {
    // Whitespace bereinigen
    rtrim(tok);
    tok = ltrim(tok);
    if (*tok == '\0') return;

    // ---- Immer gueltige Einzelwort-Kommandos (kein Doppelpunkt) ----
    if (strieq(tok, "VER") || strieq(tok, "VERSION") || strieq(tok, "IDENT")) {
        sendIdentAndState(); identSentOnStart = true;
        bootMessageShown = true; bootMessageStart = millis();
        return;
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
            bootState = BOOT_RUNNING; menuInitialized = false; diagStage = 0;
            ledClearAll(); Serial.println(F("DIAG:EXIT;"));
        } else {
            settingsEnabled = false; Serial.println(F("EXIT:OK ;"));
        }
        return;
    }
    // DIAG-Einstieg (nur im Normalbetrieb)
    if (strieq(tok, "DIAG") && bootState == BOOT_RUNNING) {
        bootState = BOOT_DIAG_MENU; diagStartTime = millis(); diagStage = 0;
        menuInitialized = false; ledClearAll(); Serial.println(F("DIAG:MENU;"));
        return;
    }

    // ---- DIAG Menue-Navigation (nur im DIAG-Modus) ----
    if (bootState == BOOT_DIAG_MENU) {
        lastMenuActivityTs = millis();
        if (strieq(tok, "NEXT")) {
            menuIndex = (menuIndex + 1) % 5;
            Serial.print(F("DIAG:MENU:")); Serial.println(menuIndex);
            if (menuIndex == 0) {
                Serial.println(F("DIAG:LED_WALK;"));
                diagStage = 1; diagWalkPhase = 0; diagWalkByte = 0;
                diagWalkLastTs = millis(); ledClearAll();
            } else if (menuIndex == 1) {
                Serial.println(F("DIAG:INPUTS;"));
                for (uint8_t i = 0; i < IN_TOTAL_BYTES; i++) {
                    char b8[9]; bin8(b8, inputState[i]);
                    Serial.print(F("IN")); Serial.print((int)i); Serial.print('=');
                    Serial.print(b8); Serial.print(';');
                }
                Serial.println();
            } else if (menuIndex == 2) {
                Serial.println(F("DIAG:DSP_START;"));
                diagStage = 2; diagDspStart = millis(); ledClearAll();
            } else if (menuIndex == 3) {
                Serial.print(F("HEALTH:UPTIME=")); Serial.print(millis() / 1000);
                Serial.print(F("s;LASTERR=0;FREEMEM="));
                extern int __heap_start, *__brkval; int vv;
                Serial.print((int)&vv - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
                Serial.print(F(";VERSION=")); Serial.println(FW_VERSION);
                Serial.println(F("DIAG:ALL_OK;"));
            } else if (menuIndex == 4) {
                bootState = BOOT_RUNNING; menuInitialized = false; diagStage = 0;
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
                Serial.print(F("IN")); Serial.print((int)i); Serial.print('=');
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
        if (strieq(tok, "DSP")) {
            diagStage = 2; diagDspStart = millis(); ledClearAll();
            Serial.println(F("DIAG:DSP_START;")); return;
        }
        // SHOW oder SHOW:N
        if (strieq(tok, "SHOW") || stripre(tok, "SHOW:")) {
            showRepeat = 1;
            char* cp2 = strchr(tok, ':');
            if (cp2) { int16_t n = atoi_s(cp2 + 1); if (n >= 1 && n <= 9) showRepeat = (int)n; }
            menuIndex = 5;
            Serial.print(F("DIAG:SHOW_START x")); Serial.print(showRepeat); Serial.println(';');
            diagStage = 3; showPhase = 0; showStep = 0; showLastTs = millis(); ledClearAll();
            for (int d = 0; d < 6; d++) lc.setRow(0, d, 0);
            showSavedDispBr = displayBrightness; setDisplayBrightness(0);
            annuBrightness = 255; backlBrightness = 255;
            return;
        }
    }

    // ---- SET ohne Doppelpunkt (z.B. "SET EXIT", "SET WRITE") ----
    if (stripre(tok, "SET ") && !strchr(tok, ':')) {
        char* sc = ltrim(tok + 4);
        if      (stripre(sc, "EXIT"))  { settingsEnabled = false; Serial.println(F("SET EXIT:OK ;")); }
        else if (stripre(sc, "WRITE") && settingsEnabled) { saveHWInfo(); settingsEnabled = false; Serial.println(F("WRITE:OK ;")); triggerSoftwareReset(false); }
        else if (stripre(sc, "WRITE")) { Serial.println(F("WRITE:LOCKED ;")); }
        return;
    }

    // ---- Token mit Doppelpunkt: KEY:VALUE ----
    char* colon = strchr(tok, ':');
    if (!colon) return;  // Unbekanntes Kommando

    *colon = '\0';
    char* key = tok;
    char* val = colon + 1;
    rtrim(key); char* kp = ltrim(key);
    val = ltrim(val); rtrim(val);

    // LEDx (index 1-basiert, index 1..LED_TOTAL_BYTES)
    if (stripre(kp, "LED")) {
        int16_t idx = atoi_s(kp + 3) - 1;
        if (idx >= 0 && idx < (int16_t)LED_TOTAL_BYTES) {
            desiredLedState[idx] = parseBin8(val); applyLEDOutputs();
        }
        return;
    }
    if (strieq(kp, "STATE")) {
        hostOnline = (strcmp(val, "01") == 0 || strcmp(val, "1") == 0); return;
    }
    if (strieq(kp, "DSP1")) {
        int16_t v = atoi_s(val);
        if (v < 0 || val[0] == '-') { for (int d = 3; d < 6; d++) lc.setRow(0, d, 0); }
        else if (strchr(val, '.'))  dspSetValStr(0, 3, val);
        else                        dspSetVal(0, 3, v, false);
        return;
    }
    if (strieq(kp, "DSP2")) {
        int16_t v = atoi_s(val);
        if (v < 0 || val[0] == '-') { for (int d = 0; d < 3; d++) lc.setRow(0, d, 0); }
        else if (strchr(val, '.'))  dspSetValStr(0, 0, val);
        else                        dspSetVal(0, 0, v, false);
        return;
    }
    if (strieq(kp, "ANN"))     { annuBrightness  = (uint8_t)constrain(atoi_s(val), 0, 255); applyLEDOutputs(); return; }
    if (strieq(kp, "BL"))      { backlBrightness = (uint8_t)constrain(atoi_s(val), 0, 255); applyLEDOutputs(); return; }
    if (strieq(kp, "DISP_BL")) { setDisplayBrightness((uint8_t)atoi_s(val)); return; }
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

    // SET key:val  (z.B. "SET ENA:0815", "SET FW:1.2", "SET SN:ABCDEF12")
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
    // UART-FIFO in Ring-Buffer uebertragen
    while (Serial.available()) {
        rxPush((char)Serial.read());
        rxLastByteTs = millis();
    }
    // Timeout-Check: unvollstaendiges Telegramm verwerfen
    if (rxAvail() > 0 && (millis() - rxLastByteTs) >= RX_TIMEOUT_MS) {
        DBGLN(F("DBG:RX_TIMEOUT"));
        rxFlush();
        return;
    }
    // Alle vollstaendigen Tokens verarbeiten
    char token[64];
    while (rxPopToken(token, sizeof(token))) {
        if (token[0] == '\0') continue;
        DBG(F("DBG:TOK>")); DBGLN(token);
        processToken(token);
    }
}

// ============================================================================
// DIAG – Zweiphasiger LED-Walk (Einzelbit, dann Byte-Komplett)
// ============================================================================
void diagLedWalkStep(unsigned long now) {
    constexpr uint16_t totalBits  = (uint16_t)LED_TOTAL_BYTES * 8;
    constexpr unsigned long SINGLE_MS = 150UL;
    constexpr unsigned long BYTE_MS   = 800UL;
    lastMenuActivityTs = now;

    if (diagWalkPhase == 0) {
        if (now - diagWalkLastTs >= SINGLE_MS) {
            diagWalkLastTs = now;
            memset(desiredLedState, 0, LED_TOTAL_BYTES);
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
            memset(desiredLedState, 0, LED_TOTAL_BYTES);
            if (diagWalkByte < (int)LED_TOTAL_BYTES) {
                desiredLedState[diagWalkByte] = 0xFF;
                annuBrightness  = (diagWalkByte % 2 == 0) ? 255 : 64;
                backlBrightness = (diagWalkByte % 2 == 0) ? 64  : 255;
                applyLEDOutputs();
                Serial.print(F("DIAG:ALL_ON LED")); Serial.print(diagWalkByte + 1);
                Serial.print(F(" ANN=")); Serial.print(annuBrightness);
                Serial.print(F(" BL=")); Serial.println(backlBrightness);
            } else {
                applyLEDOutputs();
            }
            diagWalkByte++;
            if (diagWalkByte > (int)LED_TOTAL_BYTES) {
                annuBrightness = 255; backlBrightness = 255;
                ledClearAll(); diagStage = 0;
                Serial.println(F("DIAG:LED_WALK_DONE;"));
            }
        }
    }
}

// ============================================================================
// DIAG – Display-Test: Counter + Segment-Sweep
// ============================================================================
void diagRunDisplay(unsigned long now) {
    unsigned long e = now - diagDspStart;
    lastMenuActivityTs = now;

    if (e < 5000UL) {
        int dig  = (int)((e / 400) % 10);
        int dpPos = (int)((e / 400) % 3);
        for (int g = 0; g < 2; g++) {
            uint8_t ofs = (uint8_t)(g * 3);
            for (int d = 0; d < 3; d++) {
                uint8_t pat = charTo7Seg((char)('0' + dig));
                if (d == dpPos) pat |= 0b10000000;
                lc.setRow(0, ofs + (2 - d), pat);
            }
        }
        if (e < 200UL) Serial.println(F("DIAG:DSP_COUNT;"));
    } else if (e < 9000UL) {
        int seg = (int)(((e - 5000UL) / 250) % 8);
        uint8_t pat = (uint8_t)(1 << seg);
        for (int d = 0; d < 6; d++) lc.setRow(0, d, pat);
        if (e < 5200UL) Serial.println(F("DIAG:DSP_SEGMENT;"));
    } else {
        for (int d = 0; d < 6; d++) lc.setRow(0, d, 0);
        diagStage = 0;
        Serial.println(F("DIAG:DSP_DONE;"));
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
        bootState = BOOT_RUNNING; menuInitialized = false;
        ledClearAll(); Serial.println(F("DIAG:EXIT_TIMEOUT;"));
        return;
    }
    if (memcmp(inputState, lastInputState, IN_TOTAL_BYTES) != 0) {
        lastMenuActivityTs = now;
        if (inputState[0] != lastInputState[0]) {
            uint8_t ch = inputState[0] ^ lastInputState[0];
            if ((ch & 0x01) && (inputState[0] & 0x01)) {
                menuIndex = (menuIndex + 1) % 5;
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
                        Serial.print(F("IN")); Serial.print((int)i); Serial.print('=');
                        Serial.print(b8); Serial.print(';');
                    }
                    Serial.println();
                } else if (menuIndex == 2) {
                    Serial.println(F("DIAG:DSP_START;")); diagStage = 2; diagDspStart = now; ledClearAll();
                } else if (menuIndex == 3) {
                    Serial.print(F("HEALTH:UPTIME=")); Serial.print(millis() / 1000);
                    Serial.print(F("s;LASTERR=0;FREEMEM="));
                    extern int __heap_start, *__brkval; int vv;
                    Serial.print((int)&vv - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
                    Serial.print(F(";VERSION=")); Serial.println(FW_VERSION);
                } else if (menuIndex == 4) {
                    bootState = BOOT_RUNNING; menuInitialized = false;
                    ledClearAll(); Serial.println(F("DIAG:EXIT;"));
                }
            }
        }
    }
    if (diagStage == 1) diagLedWalkStep(now);
    if (diagStage == 2) diagRunDisplay(now);
    if (diagStage == 3) diagShowStep(now);
}

// ============================================================================
// DIAG SHOW – Demo-Animation mit PWM-Breathing und Display-Inhalten
// ============================================================================
void diagShowStep(unsigned long now) {
    constexpr uint16_t totalBits = (uint16_t)LED_TOTAL_BYTES * 8;
    lastMenuActivityTs = now;

    if (showPhase == 0) {
        // Phase 0: Knight Rider (links → rechts → links)
        constexpr unsigned long KR_MS = 30UL;
        if (now - showLastTs >= KR_MS) {
            showLastTs = now;
            int pos = showStep;
            if (pos >= (int)totalBits) pos = (int)(totalBits * 2) - 2 - showStep;
            if (showStep >= (int)(totalBits * 2) - 1) {
                showPhase = 1; showStep = 0; showLastTs = now;
                ledClearAll(); Serial.println(F("DIAG:SHOW_CASCADE;")); return;
            }
            memset(desiredLedState, 0, LED_TOTAL_BYTES);
            desiredLedState[pos / 8] = (uint8_t)(1 << (pos % 8));
            applyLEDOutputs(); showStep++;
        }
    } else if (showPhase == 1) {
        // Phase 1: Cascade-Fill
        constexpr unsigned long CAS_MS = 100UL;
        if (now - showLastTs >= CAS_MS) {
            showLastTs = now;
            if (showStep < (int)LED_TOTAL_BYTES) {
                desiredLedState[showStep] = 0xFF; applyLEDOutputs();
            } else if (showStep < (int)(LED_TOTAL_BYTES * 2)) {
                desiredLedState[showStep - LED_TOTAL_BYTES] = 0x00; applyLEDOutputs();
            } else {
                showPhase = 2; showStep = 0; showLastTs = now;
                ledSetAll(0xFF);
                annuBrightness = 0; backlBrightness = 0; applyLEDOutputs();
                setDisplayBrightness(0);
                for (int d = 0; d < 6; d++) lc.setRow(0, d, 0xFF);
                Serial.println(F("DIAG:SHOW_BREATHE;")); return;
            }
            showStep++;
        }
    } else if (showPhase == 2) {
        // Phase 2: Breathing-PWM mit Display-Inhalten (7 Zyklen)
        unsigned long elapsed = now - showLastTs;
        int cycle = (int)(elapsed / 2000);
        if (cycle >= 7) {
            showPhase = 3; showStep = 0; showLastTs = now;
            annuBrightness = 255; backlBrightness = 255;
            ledSetAll(0xFF); setDisplayBrightness(15);
            for (int d = 0; d < 6; d++) lc.setChar(0, d, '8', false);
            Serial.println(F("DIAG:SHOW_FINALE;")); return;
        }
        int fp = (int)(elapsed % 2000);
        uint8_t br = (fp < 1000) ? (uint8_t)(fp / 4) : (uint8_t)(255 - (fp - 1000) / 4);
        annuBrightness = br; backlBrightness = br; applyLEDOutputs();
        setDisplayBrightness(br / 17);
        char dspBuf[7] = "      ";
        if (cycle == 0) {
            const char* L = "A320  "; const char* R = " OHP  ";
            int vis = (fp < 400) ? 1 : (fp < 800) ? 2 : (fp < 1200) ? 3 : 4;
            for (int i = 0; i < 3; i++) dspBuf[i]     = (i < vis && L[i]) ? L[i] : ' ';
            for (int i = 0; i < 3; i++) dspBuf[i + 3] = (i < vis && R[i]) ? R[i] : ' ';
            updateDisplay(0, dspBuf);
        } else if (cycle == 1) {
            int dig = (fp / 150) % 10;
            for (int d = 0; d < 6; d++) lc.setChar(0, d, (char)('0' + dig), false);
        } else if (cycle == 2) {
            for (int i = 0; i < 3; i++) dspBuf[i]     = "brI"[i];
            for (int i = 0; i < 3; i++) dspBuf[i + 3] = "tES"[i];
            updateDisplay(0, dspBuf);
        } else if (cycle == 3) {
            int seg = (fp / 180) % 8;
            uint8_t pat = (uint8_t)(1 << seg);
            for (int d = 0; d < 6; d++) lc.setRow(0, d, pat);
            if (fp < 100) Serial.println(F("DIAG:SEGMENT_DANCE;"));
        } else if (cycle == 4) {
            for (int i = 0; i < 3; i++) dspBuf[i]     = "dIA"[i];
            for (int i = 0; i < 3; i++) dspBuf[i + 3] = "SHO"[i];
            updateDisplay(0, dspBuf);
        } else if (cycle == 5) {
            int dig = (fp / 120) % 10;
            for (int d = 0; d < 6; d++) lc.setChar(0, d, (char)('0' + dig), false);
        } else {
            for (int i = 0; i < 3; i++) dspBuf[i]     = "A32"[i];
            for (int i = 0; i < 3; i++) dspBuf[i + 3] = "OHP"[i];
            updateDisplay(0, dspBuf);
        }
    } else if (showPhase == 3) {
        // Phase 3: Alles EIN, 2.5 s halten, dann wiederholen oder beenden
        if (now - showLastTs >= 2500UL) {
            showRepeat--;
            if (showRepeat > 0) {
                showPhase = 0; showStep = 0; showLastTs = now; ledClearAll();
                for (int d = 0; d < 6; d++) lc.setRow(0, d, 0);
                setDisplayBrightness(0); annuBrightness = 255; backlBrightness = 255;
                Serial.print(F("DIAG:SHOW_REPEAT ")); Serial.print(showRepeat); Serial.println(';');
            } else {
                ledClearAll();
                annuBrightness = 255; backlBrightness = 255; applyLEDOutputs();
                setDisplayBrightness(showSavedDispBr);
                for (int d = 0; d < 6; d++) lc.setRow(0, d, 0);
                diagStage = 0; showPhase = 0; showStep = 0; showRepeat = 1;
                Serial.println(F("DIAG:SHOW_DONE;"));
            }
        }
    }
}

// ============================================================================
// DIAG – Vollstaendige automatische Testsequenz (BOOT_DIAG_TEST)
// ============================================================================
void diagRunFull() {
    unsigned long e = millis() - diagStartTime;
    if (e < 2000UL) {
        ledClearAll();
        for (int d = 0; d < dspNumDev; d++) lc.clearDisplay(d);
        if (e < 500UL) Serial.println(F("DIAG:FULL_START;"));
    } else if (e < 7000UL) {
        ledClearAll();
        for (int d = 0; d < dspNumDev; d++) {
            int dig  = (int)((e - 2000) / 500 % 10);
            int dpP  = (int)((e - 2000) / 500 % 6);
            for (int i = 0; i < 6; i++) {
                uint8_t p = charTo7Seg((char)('0' + dig));
                if (i == dpP && d == 0) p |= 0x80;
                lc.setRow(d, i, p);
            }
        }
    } else if (e < 10000UL) {
        ledSetAll(0xFF);
        if (e < 10100UL) Serial.println(F("DIAG:ALL_LEDS_ON;"));
    } else if (e < 18000UL) {
        if (e < 10100UL) { diagWalkPhase = 0; diagWalkByte = 0; diagWalkLastTs = (unsigned long)(e - 150); Serial.println(F("DIAG:WALK_BIT_START;")); }
        diagLedWalkStep(e);
    } else if (e < 21000UL) {
        ledClearAll(); readAllInputs();
        for (uint8_t i = 0; i < IN_TOTAL_BYTES; i++) {
            char b8[9]; bin8(b8, inputState[i]);
            Serial.print(F("DIAG:IN")); Serial.print((int)i); Serial.print('=');
            Serial.print(b8); Serial.print(';');
        }
        Serial.println();
    } else if (e < 26000UL) {
        ledClearAll();
        if (e >= 23000UL) {
            int si = (int)(((e - 23000UL) / 300) % 8);
            uint8_t sp = (uint8_t)(1 << si);
            for (int d = 0; d < dspNumDev; d++) for (int i = 0; i < 6; i++) lc.setRow(d, i, sp);
        }
    } else if (e < 31000UL) {
        ledSetAll(0xFF);
        int fp = (int)((e - 26000UL) % 2000);
        uint8_t br = (fp < 1000) ? (uint8_t)(fp / 4) : (uint8_t)(255 - (fp - 1000) / 4);
        for (int d = 0; d < dspNumDev; d++) lc.setIntensity(d, br / 17);
    } else if (e < 34000UL) {
        ledClearAll();
    } else {
        setDisplayBrightness(displayBrightness); ledClearAll();
        for (int d = 0; d < dspNumDev; d++) lc.clearDisplay(d);
        bootState = diagReturnToMenu ? BOOT_DIAG_MENU : BOOT_RUNNING;
        if (diagReturnToMenu) { menuInitialized = false; diagReturnToMenu = false; }
        Serial.println(F("DIAG:COMPLETE;"));
    }
}

// ============================================================================
// DIAG – Hardware-Kombination pruefe (3 Bits in inputState[0])
// ============================================================================
bool diagCheckCombo() {
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
    pinMode(inClk_HE, OUTPUT); pinMode(inLatch_HE, OUTPUT); pinMode(inData_HE, INPUT);
    pinMode(inClk_FI, OUTPUT); pinMode(inLatch_FI, OUTPUT); pinMode(inData_FI, INPUT);
    pinMode(inClk_EM, OUTPUT); pinMode(inLatch_EM, OUTPUT); pinMode(inData_EM, INPUT);
    pinMode(inClk_EL, OUTPUT); pinMode(inLatch_EL, OUTPUT); pinMode(inData_EL, INPUT);
    pinMode(inClk_AC, OUTPUT); pinMode(inLatch_AC, OUTPUT); pinMode(inData_AC, INPUT);

    // Direct LEDs (active-low, initial state: AUS = HIGH)
    for (uint8_t i = 0; i < DLED_COUNT; i++) {
        pinMode(DLED_PINS[i], OUTPUT); digitalWrite(DLED_PINS[i], HIGH);
    }

    // LED Shifter
    pinMode(ledClk_HE, OUTPUT); pinMode(ledLatch_HE, OUTPUT); pinMode(ledData_HE, OUTPUT);
    pinMode(ledClk_FI, OUTPUT); pinMode(ledLatch_FI, OUTPUT); pinMode(ledData_FI, OUTPUT);
    pinMode(ledClk_EM, OUTPUT); pinMode(ledLatch_EM, OUTPUT); pinMode(ledData_EM, OUTPUT);
    pinMode(ledClk_EL, OUTPUT); pinMode(ledLatch_EL, OUTPUT); pinMode(ledData_EL, OUTPUT);
    pinMode(ledClk_AC, OUTPUT); pinMode(ledLatch_AC, OUTPUT); pinMode(ledData_AC, OUTPUT);

    // TLC5916 OE Pins – PWM-Ausgaenge, initial LOW (Ausgaenge aktiv)
    pinMode(OE_ANNU_PIN,  OUTPUT); digitalWrite(OE_ANNU_PIN,  LOW);
    pinMode(OE_BACKL_PIN, OUTPUT); digitalWrite(OE_BACKL_PIN, LOW);

    // Analog
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) pinMode(ANA_PINS[i], INPUT);

    // Arrays initialisieren
    memset(desiredLedState, 0, sizeof(desiredLedState));
    memset(hwLedState,      0, sizeof(hwLedState));
    memset(inputState,      0, sizeof(inputState));
    memset(lastInputState,  0, sizeof(lastInputState));
    memset(analogValues,    0, sizeof(analogValues));
    memset(lastAnalogValues,0, sizeof(lastAnalogValues));

    initDisplays();
    loadHWInfo();
    ledClearAll();
    readAllInputs();
    readAnalogFiltered();
    memcpy(lastInputState,   inputState,    IN_TOTAL_BYTES);
    memcpy(lastAnalogValues, analogValues,  ANALOG_COUNT * sizeof(uint16_t));

    bootState     = BOOT_RUNNING;
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

    if (bootState == BOOT_DIAG_MENU) {
        processSerialTokensFromHost();
        diagRunMenu();
        return;
    }
    if (bootState == BOOT_DIAG_TEST) {
        diagRunFull();
        return;
    }
    if (now < pauseUntil) return;

    // Boot-Display nach BOOT_MSG_DURATION_MS loeschen
    if (bootMessageShown && (now - bootMessageStart) >= BOOT_MSG_DURATION_MS) {
        bootMessageShown = false;
        for (int d = 0; d < dspNumDev; d++) lc.clearDisplay(d);
    }

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
