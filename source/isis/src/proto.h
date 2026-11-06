#pragma once
//
// proto.h — ISIS wire protocol: downlink parser + uplink frame builders.
//
// Pure logic: this header and proto.cpp must compile without Arduino / ESP32-SDK.
// They are compiled on the host (PlatformIO env "native") and unit-tested.
//
// Wire format (ISIS_FIRMWARE_SPEC.md section 3, IP_TRANSPORT_SPEC.md section 3):
//   downlink : SEQ42|DSP1:280;DSP2:0.78;DSP3:35000;DSP4:1013;DSP5:0;
//                     DSP6:-3.2;DSP7:12.5;DSP8:0.1;DISP_BL:180;CFG:scale_w=400;
//   uplink   : QNHSET:1013;  BTN:BUGS:1;  ENC:BARO:0;  OTA:progress=42;
//
// Copyright note: the DSP channel mapping is taken unchanged from CC_ISIS
// (MIT License, Copyright (c) 2024 Chris Crawford / elral,
//  https://github.com/ccrawford/CC_ISIS). The numeric message IDs are identical
// to CC_ISIS::set() so the display behaves like the original firmware.
//
#include <stdint.h>
#include <stddef.h>

#include "displaycfg.h"

// Maximum bytes we look at per incoming frame.
//
// The spec (section 3) said "maximal 96 Byte je Frame verarbeiten" - that is a
// leftover from the MobiFlight serial payload limit.  Measured on the real FDC
// (capture from FDC_IP/tools/isis_fake.py, 13.10.2026) a data frame is 155 bytes:
//
//   SEQ1|DSP1:280;...;DISP_BL:180;CFG:scale_w=480;CFG:scale_h=480;
//   CFG:offset_x=0;CFG:offset_y=0;                    <- 155 bytes total
//
// With a 96-byte window the cut lands inside "CFG", so the whole runtime
// configuration (scaling, offsets, CFG:ota=1) was silently dropped.  UDP has no
// such limit (MTU ~1500), so the cap is raised here.  The parsing rules are
// unchanged: only complete "key:value;" pairs are used, an incomplete tail is
// discarded, unknown keys are ignored.
#define ISIS_MAX_FRAME 512

// Number of DSP channels (spec section 3, table).
#define ISIS_DSP_COUNT 13

// ---------------------------------------------------------------------------
// DSP channel table — index 0..7 corresponds to dsp1..dsp8.
// The message IDs are the ones CC_ISIS::set() handles.
// ---------------------------------------------------------------------------
enum IsisDspIndex : uint8_t {
    ISIS_DSP_AIRSPEED = 0, // dsp1  messageID 60   atof
    ISIS_DSP_MACH     = 1, // dsp2  messageID 102  atof
    ISIS_DSP_ALTITUDE = 2, // dsp3  messageID 77   atof
    ISIS_DSP_QNH      = 3, // dsp4  messageID 100  atoi
    ISIS_DSP_STD      = 4, // dsp5  messageID 101  atoi
    ISIS_DSP_PITCH    = 5, // dsp6  messageID 80   atof
    ISIS_DSP_BANK     = 6, // dsp7  messageID 72   atof
    ISIS_DSP_BALL     = 7, // dsp8  messageID 71   atof
    // Landing System (Runde 7) - Kanaele 9..13, eigene Message-IDs >= 103
    ISIS_DSP_LS_ACTIVE = 8,  // dsp9   messageID 103  atoi  0/1
    ISIS_DSP_LOC_DEV   = 9,  // dsp10  messageID 104  atof  -1..1
    ISIS_DSP_LOC_VALID = 10, // dsp11  messageID 105  atoi  0/1
    ISIS_DSP_GS_DEV    = 11, // dsp12  messageID 106  atof  -1..1
    ISIS_DSP_GS_VALID  = 12  // dsp13  messageID 107  atoi  0/1
};

// The 8 display values in the exact order/units the original firmware expects.
struct IsisDisplayValues {
    float airspeed = 0.0f;  // kt      (ID 60)
    float mach     = 0.0f;  // Mach    (ID 102)
    float altitude = 0.0f;  // ft      (ID 77)
    int32_t qnhMb  = 1013;  // mb/hPa  (ID 100)
    int32_t std    = 0;     // 0/1     (ID 101)
    float pitch    = 0.0f;  // deg     (ID 80)
    float bank     = 0.0f;  // deg     (ID 72)
    float ball     = 0.0f;  // -1..1   (ID 71)
    int32_t lsActive = 0;   // 0/1     (ID 103) LS-Anzeige eingeschaltet
    float locDev   = 0.0f;  // -1..1   (ID 104) Localizer-Ablage, + rechts
    int32_t locValid = 0;   // 0/1     (ID 105) Localizer-Signal gueltig
    float gsDev    = 0.0f;  // -1..1   (ID 106) Gleitpfad-Ablage, + oben
    int32_t gsValid  = 0;   // 0/1     (ID 107) Gleitpfad-Signal gueltig
    int32_t dispBl = -1;    // 0..255, -1 = "not present in this frame"
};

// "DSP1".."DSP8" -> 0..7, anything else -> -1
int protoDspIndex(const char *key);

// index 0..7 -> CC_ISIS message ID, out of range -> -1
int protoDspMessageId(int index);

// true when the channel is parsed with atoi() (integers), false -> atof()
bool protoDspIsInt(int index);

// Apply one "DSP<n>:<value>" pair to the value struct.
// Returns true when the channel index was valid.
bool protoApplyDsp(int index, const char *value, IsisDisplayValues &values);

// ---------------------------------------------------------------------------
// Frame parsing
// ---------------------------------------------------------------------------
enum class ProtoStatus : uint8_t {
    EMPTY,     // nothing usable in the frame
    OK,        // parsed at least one recognised field
    DUPLICATE, // sequence number identical to the previous frame -> ignore
    OVERSIZE   // frame longer than ISIS_MAX_FRAME (tail is cut off, see proto.h)
};

struct ProtoResult {
    ProtoStatus status   = ProtoStatus::EMPTY;
    bool        hasSeq   = false;
    uint32_t    seq      = 0;
    uint16_t    dspMask  = 0;     // bit n set: DSP(n+1) was present
    bool        dispBl   = false; // DISP_BL was present
    bool        cfg      = false; // at least one CFG: key was present
    bool        cfgChanged = false; // CFG changed the resolved config
    bool        otaRequest = false; // CFG:ota=1 seen
    uint32_t    cfgMask  = 0;     // which CFG_* fields were present
    bool        sim      = false; // SIM:0/1 — laeuft der Simulator?
    bool        simPresent = false; // ... und war das Feld im Frame?
    char        lang[4]  = {0};   // LANG:de/en/fr/th - Sprache des FDC
    bool        langPresent = false;
    bool        img      = false; // CFG:IMG=1 — Bildstrom statt lokale Grafik
    bool        imgPresent = false;
    int         otaDemo  = -2;    // CFG:fm=<n> — Optik-Demo (0..100), -1 = aus
    bool        otaDemoPresent = false;
};

// Parser context: remembers the last accepted sequence number.
struct ProtoContext {
    uint32_t lastSeq = 0;
    bool     haveSeq = false;
};

// Parse one downlink frame.
// Unknown keys are ignored (never abort). Values without a ';' are discarded.
// A frame that parses to nothing keeps `values`/`cfg` untouched and yields EMPTY.
ProtoResult protoParseFrame(const char *frame, IsisDisplayValues &values, IsisCfg &cfg,
                            ProtoContext &context);

// ---------------------------------------------------------------------------
// Uplink frame builders (pure string builders, return the written length
// excluding the terminating NUL, or -1 when the buffer is too small)
// ---------------------------------------------------------------------------
int protoBuildBeacon(char *buf, size_t len, const char *mac, const char *ip, const char *fw);
int protoBuildQnhSet(char *buf, size_t len, int32_t mb);
int protoBuildBtn(char *buf, size_t len, const char *name, int state);
int protoBuildEncoder(char *buf, size_t len, const char *name, int direction);
int protoBuildOtaProgress(char *buf, size_t len, int percent);
int protoBuildOtaDone(char *buf, size_t len);

// ---------------------------------------------------------------------------
// QNH settle logic (spec section 5 "Regelkreis-Falle")
//
// After a local BARO change the incoming dsp4 value is ignored for ~1 s so the
// locally driven value and the value coming back from the simulator do not
// fight each other. While turning, the firmware owns the value.
// ---------------------------------------------------------------------------
class QnhSettle {
public:
    static const uint32_t DEFAULT_SETTLE_MS = 1000;

    void setSettleMs(uint32_t ms) { settleMs_ = ms; }
    uint32_t settleMs() const { return settleMs_; }

    // A local BARO step happened at `nowMs` -> remember it and own the value.
    void onLocalChange(uint32_t nowMs) {
        lastLocalMs_ = nowMs;
        localActive_ = true;
    }

    // May the incoming dsp4 value be taken over right now?
    bool remoteAccepted(uint32_t nowMs) const {
        if (!localActive_) return true;
        return (uint32_t)(nowMs - lastLocalMs_) >= settleMs_;
    }

    // Firmware currently owns the QNH value (within settle window).
    bool localActive() const { return localActive_; }

    void reset() {
        localActive_ = false;
        lastLocalMs_ = 0;
    }

private:
    uint32_t lastLocalMs_ = 0;
    uint32_t settleMs_    = DEFAULT_SETTLE_MS;
    bool     localActive_ = false;
};

// ---------------------------------------------------------------------------
// BARO acceleration: slow turning = 1 mb per detent, fast turning = 10 mb.
// A step is "fast" when it follows the previous one within `fastMs`.
// ---------------------------------------------------------------------------
class BaroRotation {
public:
    static const uint32_t DEFAULT_FAST_MS = 150;

    void setFastMs(uint32_t ms) { fastMs_ = ms; }
    uint32_t fastMs() const { return fastMs_; }

    // Takes the direction (+1/-1) of one detent at `nowMs` and returns the
    // magnitude in mb (1 or 10). Does not apply anything itself.
    int step(int direction, uint32_t nowMs) {
        int magnitude = 1;
        if (haveLast_ && (uint32_t)(nowMs - lastMs_) <= fastMs_) magnitude = 10;
        lastMs_  = nowMs;
        haveLast_ = true;
        return direction < 0 ? -magnitude : magnitude;
    }

    void reset() { haveLast_ = false; }

private:
    uint32_t fastMs_  = DEFAULT_FAST_MS;
    uint32_t lastMs_  = 0;
    bool     haveLast_ = false;
};

// ---------------------------------------------------------------------------
// Heartbeat / keepalive watchdog (spec section 4)
// Offline threshold 5 s: the display then goes to a defined state instead of
// showing stale values forever.
// ---------------------------------------------------------------------------
class DataWatchdog {
public:
    static const uint32_t DEFAULT_OFFLINE_MS = 5000;

    void setOfflineMs(uint32_t ms) { offlineMs_ = ms; }
    uint32_t offlineMs() const { return offlineMs_; }

    void onData(uint32_t nowMs) {
        lastDataMs_ = nowMs;
        haveData_   = true;
        tickedOffline_ = false;
    }

    bool online(uint32_t nowMs) const {
        if (!haveData_) return false;
        return (uint32_t)(nowMs - lastDataMs_) < offlineMs_;
    }

    // Returns true exactly once per transition online -> offline.
    bool offlineEdge(uint32_t nowMs) {
        if (online(nowMs)) {
            tickedOffline_ = false;
            return false;
        }
        if (!tickedOffline_) {
            tickedOffline_ = true;
            return true;
        }
        return false;
    }

    bool everHadData() const { return haveData_; }

private:
    uint32_t offlineMs_    = DEFAULT_OFFLINE_MS;
    uint32_t lastDataMs_   = 0;
    bool     haveData_     = false;
    bool     tickedOffline_ = false;
};

// Uplink kinds used for the "one frame per detent" bookkeeping.
enum IsisUplinkKind : uint8_t {
    UPLINK_NONE = 0,
    UPLINK_QNH,
    UPLINK_BTN,
    UPLINK_ENC
};
