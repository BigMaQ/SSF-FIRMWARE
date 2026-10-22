#pragma once
//
// buttons.h — ISIS panel buttons (PCF8574 on I2C_NUM_0) + debounce logic.
//
// Bit mapping (ISIS_FIRMWARE_SPEC.md section 5, identical to the ISIS INI section):
//   b1 BARO-  b2 BARO+  b3 BARO-Push(STD)  b4 RST   b5 BUGS  b6 LS  b7 BRT-  b8 BRT+
//
// The PCF8574 inputs are pulled up (10k to 3V3) -> a pressed key reads 0.
//
// The pure part (bit mapping, debounce, hold detection) is host-unit-tested;
// the I2C/PCF8574 layer is compiled into the ESP32 build only.
//
#include <stdint.h>

enum IsisButton : uint8_t {
    BTN_BARO_MINUS = 0, // b1
    BTN_BARO_PLUS  = 1, // b2
    BTN_BARO_PUSH  = 2, // b3
    BTN_RST        = 3, // b4
    BTN_BUGS       = 4, // b5
    BTN_LS         = 5, // b6
    BTN_BRT_MINUS  = 6, // b7
    BTN_BRT_PLUS   = 7, // b8
    BTN_COUNT      = 8
};

#define ISIS_BTN_NONE 0xFF

// Software debounce / hold times (spec section 5).
#define ISIS_BTN_DEBOUNCE_MS 20
#define ISIS_BTN_RST_HOLD_MS 2000

// PCF8574 I2C address range is 0x20..0x27 (A0..A2 tied to GND -> 0x20).
#define ISIS_PCF8574_ADDR 0x20

// --- pure helpers -----------------------------------------------------------
inline uint8_t buttonsBit(IsisButton b) { return (uint8_t)(1u << (uint8_t)b); }

// Raw port byte (pulled up, active low) -> "is this key pressed?"
inline bool buttonsIsDown(uint8_t raw, IsisButton b)
{
    return (raw & buttonsBit(b)) == 0;
}

// Raw port byte -> logical mask where bit n == 1 means "key n is pressed".
inline uint8_t buttonsDownMask(uint8_t raw)
{
    return (uint8_t)(~raw & 0xFFu);
}

// Mouse-free network-setup escape hatch (spec section 4.1 trigger 4):
// BRT+ and BRT- held simultaneously while powering up.
inline bool buttonsBootCombo(uint8_t raw)
{
    const uint8_t want = buttonsBit(BTN_BRT_PLUS) | buttonsBit(BTN_BRT_MINUS);
    return (buttonsDownMask(raw) & want) == want;
}

const char *buttonsName(IsisButton b);

// ---------------------------------------------------------------------------
// Debounce: 20 ms software debounce on top of the hardware RC.
// update() returns the mask of keys whose *stable* state changed.
// ---------------------------------------------------------------------------
class ButtonDebouncer {
public:
    void setDebounceMs(uint32_t ms) { debounceMs_ = ms; }
    uint32_t debounceMs() const { return debounceMs_; }

    uint8_t update(uint8_t raw, uint32_t nowMs);

    // Stable logical state, bit set == pressed.
    uint8_t stableMask() const { return stable_; }
    bool    pressed(IsisButton b) const { return (stable_ & buttonsBit(b)) != 0; }

    void reset()
    {
        stable_    = 0;
        candidate_ = 0;
        sinceMs_   = 0;
        primed_    = false;
    }

    // Seed the debouncer with the current raw byte without emitting an edge
    // (used once at boot so a held key is not reported as a change).
    void prime(uint8_t raw, uint32_t nowMs)
    {
        stable_    = buttonsDownMask(raw);
        candidate_ = 0;
        sinceMs_   = nowMs;
        primed_    = true;
    }

    bool primed() const { return primed_; }

private:
    uint32_t debounceMs_ = ISIS_BTN_DEBOUNCE_MS;
    uint8_t  stable_     = 0;
    uint8_t  candidate_  = 0;
    uint32_t sinceMs_    = 0;
    bool     primed_     = false;
};

// ---------------------------------------------------------------------------
// Hold detection - fires exactly once after `holdMs` of continuous pressing.
// ---------------------------------------------------------------------------
class ButtonHold {
public:
    void setHoldMs(uint32_t ms) { holdMs_ = ms; }
    uint32_t holdMs() const { return holdMs_; }

    // Returns true exactly once per continuous press that reaches holdMs.
    bool fired(bool pressed, uint32_t nowMs)
    {
        if (!pressed) {
            holding_ = false;
            fired_   = false;
            return false;
        }
        if (!holding_) {
            holding_  = true;
            fired_    = false;
            startMs_  = nowMs;
            return false;
        }
        if (!fired_ && (uint32_t)(nowMs - startMs_) >= holdMs_) {
            fired_ = true;
            return true;
        }
        return false;
    }

    bool holding() const { return holding_; }

    void reset()
    {
        holding_ = false;
        fired_   = false;
        startMs_ = 0;
    }

private:
    uint32_t holdMs_  = ISIS_BTN_RST_HOLD_MS;
    uint32_t startMs_ = 0;
    bool     holding_ = false;
    bool     fired_   = false;
};

// ---------------------------------------------------------------------------
// Hardware layer (ESP32 only)
// ---------------------------------------------------------------------------
#if defined(ISIS_ESP32)
// I2C_NUM_0 on GPIO 1 (SDA) / GPIO 2 (SCL), 100 kHz, INT on GPIO 40.
// Returns true when the PCF8574 answered.
bool buttonsBegin();

// Read the raw port byte (0xFF when the chip does not answer).
uint8_t buttonsReadRaw();

// INT line low == at least one input changed (active low).
bool buttonsIntAsserted();

// Full button service: read, debounce, dispatch local actions and uplinks.
void buttonsService(uint32_t nowMs);

// Local UI state owned by the buttons (bug menu / LS overlay flags).
bool buttonsBugMenuActive();
bool buttonsLsActive();

// QNH control loop (spec section 5).  buttons.cpp owns the value; net.cpp
// pushes incoming dsp4 values through buttonsQnhSetRemote() so the settle
// window is honoured in exactly one place.
void    buttonsQnhSetRemote(int32_t mb, uint32_t nowMs);
int32_t buttonsQnhValue();
bool    buttonsQnhRemoteAccepted(uint32_t nowMs);
#endif
