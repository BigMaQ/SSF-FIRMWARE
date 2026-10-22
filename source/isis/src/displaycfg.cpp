//
// displaycfg.cpp — runtime display configuration: scaling/offset + backlight.
//
// The pure part below has no ESP32 dependency and is host-unit-tested.
// The hardware part (NVS persistence, PWM, scaled compositing) is compiled only
// into the ESP32 build.
//
// Sources: ISIS_FIRMWARE_SPEC.md section 6 (brightness) and 7 (scaling/offset).
//
#include "displaycfg.h"

#include <stdlib.h>
#include <string.h>

#if defined(ISIS_ESP32)
#include <Arduino.h>
#include <Preferences.h>
#include "displaycanvas.h"
#include "ISISCommon.h"
#endif

// ---------------------------------------------------------------------------
// Clamping / derived values
// ---------------------------------------------------------------------------
int32_t displaycfgClampScale(int32_t value)
{
    if (value < ISIS_SCALE_MIN) return ISIS_SCALE_MIN;
    if (value > ISIS_SCALE_MAX) return ISIS_SCALE_MAX;
    return value;
}

int32_t displaycfgClampOffset(int32_t value)
{
    if (value < ISIS_OFF_MIN) return ISIS_OFF_MIN;
    if (value > ISIS_OFF_MAX) return ISIS_OFF_MAX;
    return value;
}

int32_t displaycfgClampBl(int32_t value)
{
    if (value < 0) return 0;
    if (value > 255) return 255;
    return value;
}

float displaycfgZoomX(const IsisCfg &cfg)
{
    return (float)cfg.scaleW / 480.0f;
}

float displaycfgZoomY(const IsisCfg &cfg)
{
    return (float)cfg.scaleH / 480.0f;
}

// ---------------------------------------------------------------------------
// CFG key handling.  Unknown keys are ignored (never an error) - spec section 3.
// ---------------------------------------------------------------------------
static int32_t cfgToInt(const char *value, int32_t fallback)
{
    if (!value || !*value) return fallback;
    return (int32_t)strtol(value, nullptr, 10);
}

// Accepts both the INI spelling (offset_x) and the short one used in the
// example frame (off_x).  ISIS_FIRMWARE_SPEC.md sections 7 and 12.
uint32_t displaycfgApplyKey(IsisCfg &cfg, const char *key, const char *value)
{
    if (!key || !*key) return 0;

    if (strcmp(key, "scale_w") == 0) {
        cfg.scaleW = displaycfgClampScale(cfgToInt(value, cfg.scaleW));
        return CFG_SCALE_W;
    }
    if (strcmp(key, "scale_h") == 0) {
        cfg.scaleH = displaycfgClampScale(cfgToInt(value, cfg.scaleH));
        return CFG_SCALE_H;
    }
    if (strcmp(key, "off_x") == 0 || strcmp(key, "offset_x") == 0) {
        cfg.offX = displaycfgClampOffset(cfgToInt(value, cfg.offX));
        return CFG_OFF_X;
    }
    if (strcmp(key, "off_y") == 0 || strcmp(key, "offset_y") == 0) {
        cfg.offY = displaycfgClampOffset(cfgToInt(value, cfg.offY));
        return CFG_OFF_Y;
    }
    if (strcmp(key, "disp_bl") == 0 || strcmp(key, "bl_base") == 0) {
        cfg.blBase = displaycfgClampBl(cfgToInt(value, cfg.blBase));
        return CFG_BL_BASE;
    }
    if (strcmp(key, "assign_label") == 0) {
        // Klartext fuer das Display. Nur lesbare Zeichen uebernehmen, Kleinbuchstaben
        // hochziehen, zu lang = abschneiden. Nie ein Fehler (wie alle CFG-Keys).
        if (!value) return 0;
        size_t n = 0;
        for (const char *p = value; *p && n + 1 < ISIS_ASSIGN_LABEL_LEN; ++p) {
            char c = *p;
            if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')
                || c == '-' || c == '_' || c == '.' || c == '+' || c == ' ') {
                cfg.assignLabel[n++] = c;
            } else if (c >= 'a' && c <= 'z') {
                cfg.assignLabel[n++] = (char)(c - 'a' + 'A');
            }
        }
        cfg.assignLabel[n] = '\0';
        return CFG_ASSIGN_LABEL;
    }
    if (strcmp(key, "assign") == 0) {
        if (!value || !*value) return 0;
        // "off"/"none" beenden den Modus, unbekannte Namen werden ignoriert
        // (wie alle CFG-Keys: nie ein Fehler, nur keine Wirkung).
        IsisAssignZone zone = isisAssignZoneFromName(value);
        if (zone == ISIS_ASSIGN_NONE
            && strcmp(value, "none") != 0 && strcmp(value, "off") != 0
            && strcmp(value, "0") != 0) {
            return 0;
        }
        cfg.assignZone = zone;
        return CFG_ASSIGN;
    }
    if (strcmp(key, "ota") == 0) {
        cfg.ota = cfgToInt(value, 0) != 0;
        return CFG_OTA;
    }
    if (strcmp(key, "net") == 0) {
        if (!value) return 0;
        if (strcmp(value, "static") == 0) {
            cfg.netMode = ISIS_NET_STATIC;
        } else if (strcmp(value, "static_nogw") == 0) {
            // Spec section 14: static IP with gateway 0.0.0.0 -> no route
            // out of the local subnet.  Recommended default for pit operation.
            cfg.netMode = ISIS_NET_STATIC_NOGW;
        } else if (strcmp(value, "dhcp") == 0) {
            cfg.netMode = ISIS_NET_DHCP;
        } else {
            return 0;
        }
        return CFG_NET;
    }
    return 0; // unknown key -> ignored
}

// ---------------------------------------------------------------------------
// Assign mode: name <-> zone (pure logic, host-tested)
// ---------------------------------------------------------------------------
namespace {
struct AssignName { const char *name; IsisAssignZone zone; };
const AssignName kAssignNames[] = {
    { "none", ISIS_ASSIGN_NONE },
    { "off",  ISIS_ASSIGN_NONE },
    { "baro", ISIS_ASSIGN_BARO },
    { "alt",  ISIS_ASSIGN_ALT  },
    { "spd",  ISIS_ASSIGN_SPD  },
    { "mach", ISIS_ASSIGN_MACH },
    { "att",  ISIS_ASSIGN_ATT  },
    { "bank", ISIS_ASSIGN_BANK },
    { "slip", ISIS_ASSIGN_SLIP },
    { "all",  ISIS_ASSIGN_ALL  },
};
} // namespace

IsisAssignZone isisAssignZoneFromName(const char *name)
{
    if (!name || !*name) return ISIS_ASSIGN_NONE;
    for (const AssignName &e : kAssignNames) {
        if (strcmp(name, e.name) == 0) return e.zone;
    }
    return ISIS_ASSIGN_NONE;
}

const char *isisAssignZoneName(IsisAssignZone zone)
{
    switch (zone) {
        case ISIS_ASSIGN_BARO: return "baro";
        case ISIS_ASSIGN_ALT:  return "alt";
        case ISIS_ASSIGN_SPD:  return "spd";
        case ISIS_ASSIGN_MACH: return "mach";
        case ISIS_ASSIGN_ATT:  return "att";
        case ISIS_ASSIGN_BANK: return "bank";
        case ISIS_ASSIGN_SLIP: return "slip";
        case ISIS_ASSIGN_ALL:  return "all";
        case ISIS_ASSIGN_NONE:
        default:               return "none";
    }
}

// Payload of a CFG token: "key=value"
uint32_t displaycfgApplyKv(IsisCfg &cfg, const char *kv)
{
    if (!kv || !*kv) return 0;

    const char *eq = strchr(kv, '=');
    if (!eq) return 0; // no '=' -> not a key/value pair, ignore

    char key[24];
    size_t keyLen = (size_t)(eq - kv);
    if (keyLen >= sizeof(key)) keyLen = sizeof(key) - 1;
    memcpy(key, kv, keyLen);
    key[keyLen] = '\0';

    return displaycfgApplyKey(cfg, key, eq + 1);
}

// ---------------------------------------------------------------------------
// Priority: FDC (CFG:) > NVS (boarding page) > firmware default.
// ---------------------------------------------------------------------------
static void applyLayers(IsisCfg &out, const IsisCfg &src, uint32_t mask)
{
    if (mask & CFG_SCALE_W) out.scaleW = src.scaleW;
    if (mask & CFG_SCALE_H) out.scaleH = src.scaleH;
    if (mask & CFG_OFF_X)   out.offX   = src.offX;
    if (mask & CFG_OFF_Y)   out.offY   = src.offY;
    if (mask & CFG_BL_BASE) out.blBase = src.blBase;
    if (mask & CFG_OTA)     out.ota    = src.ota;
    if (mask & CFG_NET)     out.netMode = src.netMode;
    if (mask & CFG_ASSIGN)  out.assignZone = src.assignZone;
    if (mask & CFG_ASSIGN_LABEL) {
        strncpy(out.assignLabel, src.assignLabel, ISIS_ASSIGN_LABEL_LEN - 1);
        out.assignLabel[ISIS_ASSIGN_LABEL_LEN - 1] = '\0';
    }
}

uint32_t displaycfgResolve(IsisCfg &out, const IsisCfgLayers &layers)
{
    out = layers.defaults;
    applyLayers(out, layers.nvs, layers.nvsMask); // second priority
    applyLayers(out, layers.fdc, layers.fdcMask); // highest priority
    return (layers.nvsMask | layers.fdcMask) & CFG_ALL;
}

// Local BRT +/- changes the base value in steps (spec section 6).
// The value is reported to nobody - the FDC overwrites it on the next update,
// deliberately, otherwise this would be a second control loop.
int32_t displaycfgBlStep(int32_t base, int direction)
{
    if (direction == 0) return displaycfgClampBl(base);
    return displaycfgClampBl(base + (direction > 0 ? ISIS_BL_LOCAL_STEP : -ISIS_BL_LOCAL_STEP));
}

// ===========================================================================
// Hardware layer (ESP32 only)
// ===========================================================================
#if defined(ISIS_ESP32)

namespace {
Preferences   g_prefs;
bool          g_panelReady = false;
const uint32_t kDefaultPaletteDepth = 8;
} // namespace

// The single effective configuration (FDC > NVS > default) used everywhere.
IsisCfg &displaycfgCurrent()
{
    static IsisCfg current;
    return current;
}

// The panel hardware init happens here once.  CC_ISIS::begin() calls
// setColorDepth()/init()/setRotation() on the canvas object, so the panel must
// be prepared before that - see main.cpp setup order.
void displaycfgBegin()
{
    if (g_panelReady) return;

    // 8 bit palette like the original firmware (the sprites use palette indices
    // and 8184/TFT_PINK as transparency).
    lcdPanel.setColorDepth(kDefaultPaletteDepth);

    // Backlight: GPIO 38 is driven by the LovyanGFX Light_PWM instance that
    // 4inchLCDConfig_Guition.h declared.  ledcWrite under the hood - we only
    // lower the PWM frequency to ~5 kHz and keep 8 bit resolution
    // (spec section 6).
    {
        auto lightCfg   = lcdPanel._light_instance.config();
        lightCfg.pin_bl = GPIO_NUM_38;
        lightCfg.freq   = 5000;
        lcdPanel._light_instance.config(lightCfg);
    }

    lcdPanel.init();
    lcdPanel.setRotation(3); // USB jack at the bottom (same as the original)
    lcdPanel.fillScreen(TFT_BLACK);

    g_panelReady = true;
}

void displaycfgApplyBrightness(const IsisCfg &cfg)
{
    // brightnessGamma() maps 1..100 % to the 8 bit PWM range including a
    // minimum floor so the panel never goes completely dark.
    uint8_t percent = (uint8_t)((displaycfgClampBl(cfg.blBase) * 100) / 255);
    if (percent < 1) percent = 1;
    uint8_t pwm = brightnessGamma(percent);
    isisSetBacklightPwm(pwm);
    Serial.printf("[isis] BL: blBase=%d -> %d %% -> PWM %d/255\n",
                  (int)cfg.blBase, (int)percent, (int)pwm);
}

// Compose the 480x480 canvas onto the panel.
// Identity (scale 480/480, offset 0/0) -> byte exact 1:1 copy.
// Otherwise -> scaled into the target rectangle via pushRotateZoom, i.e.
// LovyanGFX's pushImageRotateZoom path (spec section 7).
void displaycfgComposite(const IsisCfg &cfg)
{
    if (!g_panelReady) return;

    if (displaycfgIdentity(cfg)) {
        lcd.pushSprite(&lcdPanel, 0, 0);
        return;
    }

    const float zoomX = displaycfgZoomX(cfg);
    const float zoomY = displaycfgZoomY(cfg);

    // Destination rectangle centre.  pushRotateZoom() places the sprite's
    // pivot/centre at the given point, so the target rectangle has to be
    // converted into its centre.
    const float dstX = (float)cfg.offX + (float)cfg.scaleW / 2.0f;
    const float dstY = (float)cfg.offY + (float)cfg.scaleH / 2.0f;

    lcd.pushRotateZoom(&lcdPanel, dstX, dstY, 0.0f, zoomX, zoomY);
}

void displaycfgSaveNvs(const IsisCfg &cfg, uint32_t mask)
{
    if (!g_prefs.begin("isis", false)) return;
    if (mask & CFG_SCALE_W) g_prefs.putInt("scale_w", cfg.scaleW);
    if (mask & CFG_SCALE_H) g_prefs.putInt("scale_h", cfg.scaleH);
    if (mask & CFG_OFF_X)   g_prefs.putInt("off_x", cfg.offX);
    if (mask & CFG_OFF_Y)   g_prefs.putInt("off_y", cfg.offY);
    if (mask & CFG_BL_BASE) g_prefs.putInt("bl_base", cfg.blBase);
    if (mask & CFG_NET)     g_prefs.putInt("net_mode", (int)cfg.netMode);
    g_prefs.end();
}

uint32_t displaycfgLoadNvs(IsisCfg &out)
{
    uint32_t mask = 0;
    if (!g_prefs.begin("isis", true)) return 0;

    if (g_prefs.isKey("scale_w")) {
        out.scaleW = displaycfgClampScale(g_prefs.getInt("scale_w", out.scaleW));
        mask |= CFG_SCALE_W;
    }
    if (g_prefs.isKey("scale_h")) {
        out.scaleH = displaycfgClampScale(g_prefs.getInt("scale_h", out.scaleH));
        mask |= CFG_SCALE_H;
    }
    if (g_prefs.isKey("off_x")) {
        out.offX = displaycfgClampOffset(g_prefs.getInt("off_x", out.offX));
        mask |= CFG_OFF_X;
    }
    if (g_prefs.isKey("off_y")) {
        out.offY = displaycfgClampOffset(g_prefs.getInt("off_y", out.offY));
        mask |= CFG_OFF_Y;
    }
    if (g_prefs.isKey("bl_base")) {
        out.blBase = displaycfgClampBl(g_prefs.getInt("bl_base", out.blBase));
        mask |= CFG_BL_BASE;
    }
    if (g_prefs.isKey("net_mode")) {
        int m = g_prefs.getInt("net_mode", (int)ISIS_NET_DHCP);
        if (m < 0 || m > 2) m = ISIS_NET_DHCP;
        out.netMode = (IsisNetMode)m;
        mask |= CFG_NET;
    }
    g_prefs.end();
    return mask;
}

void displaycfgEraseNvs()
{
    if (!g_prefs.begin("isis", false)) return;
    g_prefs.clear();
    g_prefs.end();
}

#endif // ISIS_ESP32
