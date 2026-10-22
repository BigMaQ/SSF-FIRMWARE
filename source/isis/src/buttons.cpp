//
// buttons.cpp — ISIS buttons: PCF8574 bit mapping, debounce/hold logic and the
// local actions (brightness, LS, bug menu, attitude reset, QNH).
//
// Local actions run entirely inside the device (ISIS_FIRMWARE_SPEC.md section 5);
// only QNH has to go back to the simulator, because it is a shared baro
// reference.  That is done with a QNHSET uplink frame - always in mb.
//
// Sources: ISIS_FIRMWARE_SPEC.md section 5, IP_TRANSPORT_SPEC.md section 9/12.
//
#include "buttons.h"

#include <stdlib.h>

// ---------------------------------------------------------------------------
// Pure logic
// ---------------------------------------------------------------------------
const char *buttonsName(IsisButton b)
{
    switch (b) {
    case BTN_BARO_MINUS: return "BARO-";
    case BTN_BARO_PLUS:  return "BARO+";
    case BTN_BARO_PUSH:  return "BAROPUSH";
    case BTN_RST:        return "RST";
    case BTN_BUGS:       return "BUGS";
    case BTN_LS:         return "LS";
    case BTN_BRT_MINUS:  return "BRT-";
    case BTN_BRT_PLUS:   return "BRT+";
    default:             return "UNKNOWN";
    }
}

uint8_t ButtonDebouncer::update(uint8_t raw, uint32_t nowMs)
{
    const uint8_t rawPressed = buttonsDownMask(raw);

    if (!primed_) {
        // First sample after boot: adopt it without reporting an edge.
        prime(raw, nowMs);
        return 0;
    }

    const uint8_t diff = (uint8_t)(rawPressed ^ stable_);
    if (diff == 0) {
        candidate_ = 0; // bounced back to the stable state
        return 0;
    }

    if (diff != candidate_) {
        candidate_ = diff; // new candidate state -> restart the debounce window
        sinceMs_   = nowMs;
        return 0;
    }

    if ((uint32_t)(nowMs - sinceMs_) < debounceMs_) return 0;

    stable_    = rawPressed;
    candidate_ = 0;
    return diff;
}

// ===========================================================================
// Hardware layer (ESP32 only)
// ===========================================================================
#if defined(ISIS_ESP32)

#include <Arduino.h>
#include <driver/i2c.h>

#include "ISISCommon.h"     // isisState, isisSettings
#include "displaycfg.h"
#include "net.h"
#include "proto.h"

namespace {

const int kPinSda  = 1;  // I2C_NUM_0 - the touch keeps I2C_NUM_1 (19/45 @400k)
const int kPinScl  = 2;
const int kPinInt  = 40; // PCF8574 INT, active low
const uint32_t kI2cFreq = 100000; // 100 kHz for the flat ribbon (spec section 12)

ButtonDebouncer g_debounce;
ButtonHold      g_rstHold;
QnhSettle       g_qnhSettle;
BaroRotation    g_baroRot;

bool     g_haveChip     = false;
bool     g_bugMenu      = false;
bool     g_lsActive     = false;
uint32_t g_lastPollMs   = 0;

// RST / BUGS / LS / brightness are ISS-local.  QNH is mirrored to the simulator.
void sendBtnUplink(IsisButton b, bool down)
{
    char frame[48];
    if (protoBuildBtn(frame, sizeof(frame), buttonsName(b), down ? 1 : 0) > 0) {
        netSendUplink(frame);
    }
}

} // namespace

bool buttonsBegin()
{
    i2c_config_t cfg = {};
    cfg.mode             = I2C_MODE_MASTER;
    cfg.sda_io_num       = (gpio_num_t)kPinSda;
    cfg.scl_io_num       = (gpio_num_t)kPinScl;
    cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = kI2cFreq;

    if (i2c_param_config(I2C_NUM_0, &cfg) != ESP_OK) return false;
    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        // Already installed (e.g. after a soft restart) - keep going.
    }

    pinMode(kPinInt, INPUT_PULLUP);

    g_debounce.setDebounceMs(ISIS_BTN_DEBOUNCE_MS);
    g_rstHold.setHoldMs(ISIS_BTN_RST_HOLD_MS);

    // Probe the port expander; a missing chip must not stop the display.
    const uint8_t probe = buttonsReadRaw();
    g_haveChip = (probe != 0xFF) || (digitalRead(kPinInt) == LOW);

    // Adopt the current key state without emitting edges.
    g_debounce.prime(probe, millis());
    return g_haveChip;
}

uint8_t buttonsReadRaw()
{
    uint8_t value = 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((ISIS_PCF8574_ADDR << 1) | I2C_MASTER_READ), true);
    i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    const esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) return 0xFF;
    return value;
}

bool buttonsIntAsserted()
{
    return digitalRead(kPinInt) == LOW;
}

bool buttonsBugMenuActive() { return g_bugMenu; }
bool buttonsLsActive() { return g_lsActive; }

// --- QNH -------------------------------------------------------------------
// Called from the button handler.  Takes the direction of one detent and
// returns the new value in mb.  One uplink frame per detent (not per loop).
static void qnhLocalStep(int direction, uint32_t nowMs)
{
    const int32_t step = g_baroRot.step(direction, nowMs); // +-1 mb or +-10 mb
    int32_t value = isisState.mbPressure + step;
    if (value < 0) value = 0;
    if (value > 9999) value = 9999;
    isisState.mbPressure = value;

    // Turning deselects / overrides STD mode on the real instrument when not
    // in STD.  Keep it simple: turning always leaves STD mode.
    isisState.isStdPressure = 0;

    g_qnhSettle.onLocalChange(nowMs);
    isisState.forceRedraw = true;

    char frame[32];
    if (protoBuildQnhSet(frame, sizeof(frame), value) > 0) netSendUplink(frame);
}

// Remote side of the control loop: called by net.cpp for every dsp4 value.
void buttonsQnhSetRemote(int32_t mb, uint32_t nowMs)
{
    if (!g_qnhSettle.remoteAccepted(nowMs)) return; // settle window -> ignore
    if (mb < 0) mb = 0;
    if (mb > 9999) mb = 9999;
    if (isisState.mbPressure != mb) {
        isisState.mbPressure = mb;
        isisState.forceRedraw = true;
    }
}

int32_t buttonsQnhValue() { return isisState.mbPressure; }
bool buttonsQnhRemoteAccepted(uint32_t nowMs) { return g_qnhSettle.remoteAccepted(nowMs); }

// --- main service ----------------------------------------------------------
void buttonsService(uint32_t nowMs)
{
    // INT is active low; a missing chip falls back to a 10 ms poll so the
    // panel stays usable when the expander is not wired yet.
    const bool intLow = g_haveChip && buttonsIntAsserted();
    const bool periodicDue = (uint32_t)(nowMs - g_lastPollMs) >= 10;
    if (!intLow && !periodicDue) return;
    g_lastPollMs = nowMs;

    const uint8_t raw    = buttonsReadRaw();
    const uint8_t change = g_debounce.update(raw, nowMs);
    if (!change) return;

    // ---- BARO rotation ----
    if (change & buttonsBit(BTN_BARO_MINUS)) qnhLocalStep(-1, nowMs);
    if (change & buttonsBit(BTN_BARO_PLUS))  qnhLocalStep(+1, nowMs);

    // ---- BARO push: STD toggle, deselect bug value ----
    if (change & buttonsBit(BTN_BARO_PUSH)) {
        isisState.isStdPressure = isisState.isStdPressure ? 0 : 1;
        isisState.forceRedraw   = true;
        sendBtnUplink(BTN_BARO_PUSH, g_debounce.pressed(BTN_BARO_PUSH));
    }

    // ---- BUGS / LS: local overlays (rendering is phase 2, spec section 9) ----
    if (change & buttonsBit(BTN_BUGS)) {
        const bool down = g_debounce.pressed(BTN_BUGS);
        if (down) g_bugMenu = !g_bugMenu;
        isisState.forceRedraw = true;
        sendBtnUplink(BTN_BUGS, down);
    }
    if (change & buttonsBit(BTN_LS)) {
        const bool down = g_debounce.pressed(BTN_LS);
        if (down) g_lsActive = !g_lsActive;
        isisState.forceRedraw = true;
        sendBtnUplink(BTN_LS, down);
    }

    // ---- Brightness: local only, never reported to the FDC (spec section 6) ----
    if (change & buttonsBit(BTN_BRT_MINUS)) {
        IsisCfg &cfg = displaycfgCurrent();
        cfg.blBase = displaycfgBlStep(cfg.blBase, -1);
        displaycfgApplyBrightness(cfg);
        displaycfgSaveNvs(cfg, CFG_BL_BASE);
        sendBtnUplink(BTN_BRT_MINUS, g_debounce.pressed(BTN_BRT_MINUS));
    }
    if (change & buttonsBit(BTN_BRT_PLUS)) {
        IsisCfg &cfg = displaycfgCurrent();
        cfg.blBase = displaycfgBlStep(cfg.blBase, +1);
        displaycfgApplyBrightness(cfg);
        displaycfgSaveNvs(cfg, CFG_BL_BASE);
        sendBtnUplink(BTN_BRT_PLUS, g_debounce.pressed(BTN_BRT_PLUS));
    }

    // ---- RST: attitude reset, 2 s hold ("ATT 10 s" on the real unit) ----
    // The panel is fed from the simulator, so a reset means: re-level the
    // attitude display until fresh data arrives.
    if (g_rstHold.fired(g_debounce.pressed(BTN_RST), nowMs)) {
        isisState.rawPitchAngle = 0.0f;
        isisState.rawBankAngle  = 0.0f;
        isisState.pitchAngle    = 0.0f;
        isisState.bankAngle     = 0.0f;
        isisState.ballPos       = 0.0f;
        isisState.rawBallPos    = 0.0f;
        isisState.forceRedraw   = true;
        sendBtnUplink(BTN_RST, true);
    }
}

#endif // ISIS_ESP32
