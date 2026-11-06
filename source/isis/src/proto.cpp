//
// proto.cpp — ISIS wire protocol implementation (pure logic, host testable).
//
// Message-ID mapping and value conversion are identical to the original
// CC_ISIS::set() (MIT License, Copyright (c) 2024 Chris Crawford / elral,
// https://github.com/ccrawford/CC_ISIS) so the display behaves like the
// upstream firmware.  See ISIS_FIRMWARE_SPEC.md section 3.
//
#include "proto.h"
#include "net.h"

// aus net.cpp (Hardware-Revision, siehe net.h)
uint8_t isisHwRev();

// Der Host-Test (pio test -e native) hat kein NVS. Ohne diesen Stub bleibt
// protoBuildBeacon ein undefiniertes Symbol und ALLE nativen Tests scheitern
// schon beim Linken - der Test-Env ist sonst nicht benutzbar.
#if defined(ISIS_NATIVE)
uint8_t isisHwRev() { return 1; }
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// DSP channel table
// ---------------------------------------------------------------------------
namespace {

struct DspSpec {
    const char *key;   // wire key
    int         msgId; // CC_ISIS message ID (identical to the original firmware)
    bool        isInt; // atoi() instead of atof()
};

const DspSpec kDsp[ISIS_DSP_COUNT] = {
    { "DSP1", 60,  false }, // Airspeed   (kt)
    { "DSP2", 102, false }, // Mach
    { "DSP3", 77,  false }, // Altitude   (ft)
    { "DSP4", 100, true  }, // QNH        (mb)
    { "DSP5", 101, true  }, // Baro STD mode
    { "DSP6", 80,  false }, // Pitch      (deg)
    { "DSP7", 72,  false }, // Bank       (deg)
    { "DSP8", 71,  false }, // Ball / slip-skid
    // Landing System (Runde 7)
    { "DSP9",  103, true  }, // LS-Anzeige an/aus
    { "DSP10", 104, false }, // Localizer-Ablage -1..1
    { "DSP11", 105, true  }, // Localizer gueltig
    { "DSP12", 106, false }, // Gleitpfad-Ablage -1..1
    { "DSP13", 107, true  }  // Gleitpfad gueltig
};

} // namespace

int protoDspIndex(const char *key)
{
    if (!key) return -1;

    // Gross-/Kleinschreibung egal: der FDC sendet "DSP1", im Panel-Monitor
    // tippt man aber eher "dsp1". Vorher kam so ein Testbefehl stumm nicht an
    // und die Suche ging beim Panel los statt beim Tippfehler.
    auto up = [](char c) -> char { return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; };

    // Exactly "DSP" + one digit 1..8.  "DSP10" must NOT be read as "DSP1"
    // (channels 9..19 are phase 2, spec section 9).
    if (up(key[0]) != 'D' || up(key[1]) != 'S' || up(key[2]) != 'P') return -1;
    // "DSP1".."DSP9" (eine Ziffer) oder "DSP10".."DSP13" (zwei Ziffern).
    if (key[3] < '1' || key[3] > '9') return -1;
    const int einer = key[3] - '0';
    if (key[4] == '\0') return einer - 1;
    if (key[4] < '0' || key[4] > '9' || key[5] != '\0') return -1;
    const int zwei = einer * 10 + (key[4] - '0');
    if (zwei < 10 || zwei > ISIS_DSP_COUNT) return -1;
    return zwei - 1;
}

int protoDspMessageId(int index)
{
    if (index < 0 || index >= ISIS_DSP_COUNT) return -1;
    return kDsp[index].msgId;
}

bool protoDspIsInt(int index)
{
    if (index < 0 || index >= ISIS_DSP_COUNT) return false;
    return kDsp[index].isInt;
}

bool protoApplyDsp(int index, const char *value, IsisDisplayValues &values)
{
    if (index < 0 || index >= ISIS_DSP_COUNT) return false;
    if (!value) return false;

    if (kDsp[index].isInt) {
        // atoi() semantics like the original: parse the leading integer.
        int32_t v = (int32_t)strtol(value, nullptr, 10);
        switch (index) {
        case ISIS_DSP_QNH: values.qnhMb = v; break;
        case ISIS_DSP_STD: values.std = v ? 1 : 0; break;
        case ISIS_DSP_LS_ACTIVE:  values.lsActive = v ? 1 : 0; break;
        case ISIS_DSP_LOC_VALID:  values.locValid = v ? 1 : 0; break;
        case ISIS_DSP_GS_VALID:   values.gsValid  = v ? 1 : 0; break;
        default: break;
        }
        return true;
    }

    float v = strtof(value, nullptr);
    switch (index) {
    case ISIS_DSP_AIRSPEED: values.airspeed = v; break;
    case ISIS_DSP_MACH:     values.mach = v; break;
    case ISIS_DSP_ALTITUDE: values.altitude = v; break;
    case ISIS_DSP_PITCH:    values.pitch = v; break;
    case ISIS_DSP_BANK:     values.bank = v; break;
    case ISIS_DSP_BALL:     values.ball = v; break;
    case ISIS_DSP_LOC_DEV:  values.locDev = v; break;
    case ISIS_DSP_GS_DEV:   values.gsDev  = v; break;
    default: break;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Frame parsing
// ---------------------------------------------------------------------------
static void parseHeaderSeq(const char *header, size_t len, ProtoResult &res)
{
    // Look for "SEQ<digits>" inside the header (before the first '|').
    for (size_t i = 0; i + 3 < len; ++i) {
        if (header[i] == 'S' && header[i + 1] == 'E' && header[i + 2] == 'Q') {
            size_t j = i + 3;
            if (j >= len || header[j] < '0' || header[j] > '9') continue;
            uint32_t seq = 0;
            bool     any = false;
            while (j < len && header[j] >= '0' && header[j] <= '9') {
                seq = seq * 10u + (uint32_t)(header[j] - '0');
                ++j;
                any = true;
            }
            if (any) {
                res.hasSeq = true;
                res.seq    = seq;
            }
            return;
        }
    }
}

ProtoResult protoParseFrame(const char *frame, IsisDisplayValues &values, IsisCfg &cfg,
                            ProtoContext &context)
{
    ProtoResult res;
    // Merkt sich, dass zuletzt ein CFG:-Token kam: nachfolgende key=value
    // Tokens werden dann ebenfalls als CFG uebernommen (Toleranz-Regel).
    bool cfgKontext = false;

    if (!frame) return res;

    size_t rawLen = strlen(frame);
    if (rawLen == 0) return res;

    const bool oversize = rawLen > ISIS_MAX_FRAME;
    // Process at most ISIS_MAX_FRAME bytes (spec section 3).
    size_t len = oversize ? (size_t)ISIS_MAX_FRAME : rawLen;

    char work[ISIS_MAX_FRAME + 1];
    memcpy(work, frame, len);
    work[len] = '\0';

    // --- header (up to the first '|') ---
    const char *bar = strchr(work, '|');
    const char *body;
    if (bar) {
        parseHeaderSeq(work, (size_t)(bar - work), res);
        body = bar + 1;
    } else {
        // Tolerant: a frame without a header is treated as pure payload.
        body = work;
    }

    // --- duplicate sequence numbers (UDP duplicates) are dropped entirely ---
    if (res.hasSeq) {
        if (context.haveSeq && context.lastSeq == res.seq) {
            res.status = ProtoStatus::DUPLICATE;
            return res;
        }
        context.lastSeq = res.seq;
        context.haveSeq = true;
    }

    // --- body: "KEY:VALUE;" pairs.  A value without a terminating ';' is
    //     discarded (spec section 3). ---
    const char *p = body;
    while (*p) {
        const char *semi = strchr(p, ';');
        if (!semi) break; // trailing partial token -> discard

        // token = [p, semi)
        const char *colon = (const char *)memchr(p, ':', (size_t)(semi - p));
        if (colon) {
            char key[12];
            size_t keyLen = (size_t)(colon - p);
            if (keyLen > 0 && keyLen < sizeof(key)) {
                memcpy(key, p, keyLen);
                key[keyLen] = '\0';

                char val[64];
                size_t valLen = (size_t)(semi - colon - 1);
                if (valLen >= sizeof(val)) valLen = sizeof(val) - 1;
                memcpy(val, colon + 1, valLen);
                val[valLen] = '\0';

                int idx = protoDspIndex(key);
                if (idx >= 0) {
                    protoApplyDsp(idx, val, values);
                    res.dspMask |= (uint16_t)(1u << idx);
                } else if (strcmp(key, "LANG") == 0) {
                    // Sprache der FDC-Oberflaeche: die Statusmeldungen auf dem
                    // Panel sollen derselben Sprache folgen.
                    strncpy(res.lang, val, sizeof(res.lang) - 1);
                    res.lang[sizeof(res.lang) - 1] = '\0';
                    res.langPresent = true;
                } else if (strcmp(key, "SIM") == 0) {
                    // Lebenszeichen des FDC mit Simulator-Status (IP_TRANSPORT_SPEC).
                    res.sim        = (strtol(val, nullptr, 10) != 0);
                    res.simPresent = true;
                } else if (strcmp(key, "DISP_BL") == 0 || strcmp(key, "DSP_BL") == 0) {
                    // DSP_BL accepted for tolerance with the other panels'
                    // firmware (IP_TRANSPORT_SPEC.md section 6).
                    int32_t bl = (int32_t)strtol(val, nullptr, 10);
                    if (bl < 0) bl = 0;
                    if (bl > 255) bl = 255;
                    values.dispBl = bl;
                    res.dispBl   = true;
                } else if (cfgKontext && strchr(val, '=') != nullptr) {
                    // Toleranz: "CFG:scale_w=384;scale_h=384;off_x=72;"
                    // Ohne diese Regel werden alle Tokens nach dem ersten als
                    // unbekannte Keys still verworfen - der Frame sieht dann
                    // aus, als haette nur der erste Wert gewirkt (verzerrtes
                    // Bild, Offset bleibt 0).
                    uint32_t m = displaycfgApplyKv(cfg, val);
                    if (m) {
                        res.cfg        = true;
                        res.cfgChanged = true;
                        res.cfgMask   |= m;
                        if (m & CFG_OTA) res.otaRequest = cfg.ota;
                    }
                } else if (strcmp(key, "CFG") == 0) {
                    cfgKontext = true;
                    // CFG:IMG=1/0 schaltet den Bildstrom (glassout-Kanal) um.
                    // Bewusst hier und nicht in displaycfg: der Modus ist
                    // fluechtig und gehoert nicht ins NVS.
                    const char *img = strstr(val, "IMG=");
                    if (img) {
                        res.img        = (atoi(img + 4) != 0);
                        res.imgPresent = true;
                    }
                    // CFG:fm=1 kuendigt einen USB-/COM-Flash an: das Overlay
                    // bleibt stehen, weil ein SoC-Reset das TFT nicht loescht -
                    // der Bildschirm zeigt FLASH MODE also auch waehrend
                    // esptool den Chip im Bootloader bearbeitet.
                    // Der Parser bleibt frei von Netz-Aufrufen: net.h ist im
                    // Host-Test (ISIS_NATIVE) leer, ein Direktaufruf zerlegt
                    // dort den Build. net.cpp wendet das Flag an.
                    const char *fm = strstr(val, "fm=");
                    if (fm) {
                        res.otaDemo        = atoi(fm + 3) ? 0 : -1;
                        res.otaDemoPresent = true;
                    }
                    uint32_t m = displaycfgApplyKv(cfg, val);
                    if (m) {
                        res.cfg        = true;
                        res.cfgChanged = true;
                        res.cfgMask |= m;
                        // ota=1 oeffnet, ota=0 schliesst das Zeitfenster.
                        if (m & CFG_OTA) res.otaRequest = cfg.ota;
                    }
                }
                // anything else: unknown key -> ignored, never abort
            }
        }
        p = semi + 1;
    }

    // SIM (und kuenftig LANG) sind vollwertige Felder: ein reiner
    // Lebenszeichen-Frame "SEQ42|SIM:0;" MUSS als OK gelten, sonst bricht
    // handleFrame ab, der Watchdog hungert und das Panel bleibt auf
    // "FDC NOT CONNECTED" stehen, obwohl der FDC sendet.
    if (res.dspMask || res.dispBl || res.cfg || res.simPresent || res.langPresent ||
        res.imgPresent) {
        res.status = ProtoStatus::OK;
    } else {
        res.status = oversize ? ProtoStatus::OVERSIZE : ProtoStatus::EMPTY;
    }
    return res;
}

// ---------------------------------------------------------------------------
// Uplink frame builders
// ---------------------------------------------------------------------------
// All builders funnel through this: on overflow the buffer is cleared and -1
// is returned, so a caller can never send a truncated frame.
static int finish(char *buf, size_t len, int written)
{
    if (!buf || len == 0 || written < 0 || (size_t)written >= len) {
        if (buf && len > 0) buf[0] = '\0';
        return -1;
    }
    return written;
}

int protoBuildBeacon(char *buf, size_t len, const char *mac, const char *ip, const char *fw)
{
    if (!buf || len == 0) return -1;
    return finish(buf, len,
                  snprintf(buf, len, "IDENT:ISIS;MAC:%s;IP:%s;FW:%s;HW:ESP32S3-R%u;ED:MAQ;",
                           mac ? mac : "00:00:00:00:00:00",
                           ip ? ip : "0.0.0.0",
                           fw ? fw : "0.0.0",
                           (unsigned)isisHwRev()));
}

int protoBuildQnhSet(char *buf, size_t len, int32_t mb)
{
    if (!buf || len == 0) return -1;
    // QNHSET is always sent in mb (hPa) - the display unit is presentation only
    // (spec section 5).
    if (mb < 0) mb = 0;
    if (mb > 9999) mb = 9999;
    return finish(buf, len, snprintf(buf, len, "QNHSET:%ld;", (long)mb));
}

int protoBuildBtn(char *buf, size_t len, const char *name, int state)
{
    if (!buf || len == 0 || !name) {
        if (buf && len > 0) buf[0] = '\0';
        return -1;
    }
    return finish(buf, len, snprintf(buf, len, "BTN:%s:%d;", name, state ? 1 : 0));
}

int protoBuildEncoder(char *buf, size_t len, const char *name, int direction)
{
    if (!buf || len == 0 || !name) {
        if (buf && len > 0) buf[0] = '\0';
        return -1;
    }
    // Direction follows the original MobiFlight encoding:
    // 0 = increase, 2 = decrease.
    return finish(buf, len, snprintf(buf, len, "ENC:%s:%d;", name, direction));
}

int protoBuildOtaProgress(char *buf, size_t len, int percent)
{
    if (!buf || len == 0) return -1;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return finish(buf, len, snprintf(buf, len, "OTA:progress=%d;", percent));
}

int protoBuildOtaDone(char *buf, size_t len)
{
    if (!buf || len == 0) return -1;
    return finish(buf, len, snprintf(buf, len, "OTA:done;"));
}
