#pragma once
//
// displaycfg.h — runtime display configuration (scaling/offset) + backlight.
//
// Pure logic part (IsisCfg, CFG apply, priority resolution, clamping) compiles
// without the ESP32 SDK and is host-tested. The hardware layer at the bottom is
// only compiled into the ESP32 build (ISIS_ESP32).
//
// Sources: ISIS_FIRMWARE_SPEC.md sections 6 and 7, IP_TRANSPORT_SPEC.md section 12.
//
#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// CFG field bit masks (one bit per configurable field)
// ---------------------------------------------------------------------------
enum IsisCfgField : uint32_t {
    CFG_SCALE_W = 1u << 0,
    CFG_SCALE_H = 1u << 1,
    CFG_OFF_X   = 1u << 2,
    CFG_OFF_Y   = 1u << 3,
    CFG_BL_BASE = 1u << 4,
    CFG_OTA     = 1u << 5,
    CFG_NET     = 1u << 6,
    CFG_ASSIGN  = 1u << 7,   // fluechtig: Zuordnungs-Modus (nie in den NVS)
    CFG_ASSIGN_LABEL = 1u << 8, // fluechtig: Name des zugeordneten Bedienelements
    CFG_ALL     = 0x1FFu
};

// Laenge der Zuordnungs-Beschriftung inkl. Nullbyte (15 sichtbare Zeichen reichen
// fuer "BARO-DEC", "IP1-B5", "DSP12" — laengere Namen werden abgeschnitten).
#define ISIS_ASSIGN_LABEL_LEN 16

// Zuordnungs-Modus (CFG:assign=<zone>): der FDC sagt dem Panel, welches Bedienelement
// gerade zugeordnet wird. Das Panel laesst diesen Wert blinken und zeigt einen Hinweis
// ("Assignment in progress") an. Bewusst kein neuer Textbefehl — der Modus reist als
// Flag in der bestehenden CFG-Schicht mit und darf NIE persistiert werden.
enum IsisAssignZone : uint8_t {
    ISIS_ASSIGN_NONE = 0,
    ISIS_ASSIGN_BARO,   // Druck-/QNH-Fenster
    ISIS_ASSIGN_ALT,    // Hoehenband
    ISIS_ASSIGN_SPD,    // Geschwindigkeitsband
    ISIS_ASSIGN_MACH,   // Mach-Anzeige
    ISIS_ASSIGN_ATT,    // kuenstlicher Horizont / Nick
    ISIS_ASSIGN_BANK,   // Rollwinkel
    ISIS_ASSIGN_SLIP,   // Kugel / Schieben
    ISIS_ASSIGN_ALL     // Panel-Identifikation (alles blinkt)
};

// "none"/"off"/"" -> ISIS_ASSIGN_NONE; unbekannter Name -> ISIS_ASSIGN_NONE.
IsisAssignZone isisAssignZoneFromName(const char *name);

// Umgekehrter Weg (Log/Tests): stabiler Kleinbuchstaben-Name.
const char *isisAssignZoneName(IsisAssignZone zone);

// Scaling limits (spec section 7: "scale 240…480 px").
#define ISIS_SCALE_MIN 240
#define ISIS_SCALE_MAX 480
#define ISIS_OFF_MIN   (-240)
#define ISIS_OFF_MAX   480

// Default backlight base value (0..255, spec section 6).
#define ISIS_BL_DEFAULT 200

// Local BRT +/- step: 5 % of the full range (spec section 6).
#define ISIS_BL_LOCAL_STEP 13 // ~5 % of 255

// Network mode requested via CFG:net=... (spec section 14 "No-Gateway-Option").
enum IsisNetMode : uint8_t {
    ISIS_NET_DHCP      = 0, // default: DHCP from the local network
    ISIS_NET_STATIC    = 1, // static IP, gateway given by the boarding page
    ISIS_NET_STATIC_NOGW= 2  // static IP, gateway 0.0.0.0 -> no route off-subnet
};

struct IsisCfg {
    int32_t    scaleW  = 480;             // target render width  (px)
    int32_t    scaleH  = 480;             // target render height (px)
    int32_t    offX    = 0;               // target offset x
    int32_t    offY    = 0;               // target offset y
    int32_t    blBase  = ISIS_BL_DEFAULT; // DISP_BL base value 0..255
    bool       ota     = false;           // CFG:ota=1 -> open ArduinoOTA for 10 min
    IsisNetMode netMode = ISIS_NET_DHCP;
    IsisAssignZone assignZone = ISIS_ASSIGN_NONE; // CFG:assign=<zone>, fluechtig
    // CFG:assign_label=<text>: Klartext fuer das Display (Taster haben keine
    // Anzeigeposition — dort bleibt nur dieser Hinweis).
    char           assignLabel[ISIS_ASSIGN_LABEL_LEN] = {0};
};

// Felder, die in den NVS duerfen. Assign ist ein Modus und wird hier bewusst
// herausgefiltert: sonst startet das Panel nach einem Neustart im Zuordnungs-Blinken.
inline uint32_t displaycfgPersistMask(uint32_t mask) {
    return mask & CFG_ALL & ~CFG_OTA & ~CFG_ASSIGN & ~CFG_ASSIGN_LABEL;
}

// True when nothing has to be scaled/offset (identity mapping, byte-exact output).
inline bool displaycfgIdentity(const IsisCfg &cfg) {
    return cfg.scaleW == 480 && cfg.scaleH == 480 && cfg.offX == 0 && cfg.offY == 0;
}

int32_t displaycfgClampScale(int32_t value);
int32_t displaycfgClampOffset(int32_t value);
int32_t displaycfgClampBl(int32_t value);

// Horizontal/vertical zoom factors, derived from scale_w/scale_h.
float displaycfgZoomX(const IsisCfg &cfg);
float displaycfgZoomY(const IsisCfg &cfg);

// Apply a single "CFG:<key>=<value>" pair. `key` is the part before '='.
// Returns the field mask that was recognised (0 when the key is unknown —
// unknown keys are ignored, never an error).
uint32_t displaycfgApplyKey(IsisCfg &cfg, const char *key, const char *value);

// Apply the "name=value" payload of a CFG token (the part after "CFG:").
uint32_t displaycfgApplyKv(IsisCfg &cfg, const char *kv);

// ---------------------------------------------------------------------------
// Parameter priority: FDC (CFG:) > NVS (boarding page) > firmware default.
// Each layer carries a mask of the fields it actually provides.
// ---------------------------------------------------------------------------
struct IsisCfgLayers {
    IsisCfg  defaults;
    IsisCfg  nvs;
    uint32_t nvsMask = 0;
    IsisCfg  fdc;
    uint32_t fdcMask = 0;
};

// Resolve the effective configuration and return the mask of fields that came
// from NVS or FDC (i.e. everything that is worth persisting).
uint32_t displaycfgResolve(IsisCfg &out, const IsisCfgLayers &layers);

// Backlight: local BRT +/- changes the base value in steps, clamped 0..255.
int32_t displaycfgBlStep(int32_t base, int direction);

// ---------------------------------------------------------------------------
// Hardware layer (ESP32 only)
// ---------------------------------------------------------------------------
#if defined(ISIS_ESP32)
// Configure the backlight PWM (GPIO 38 via the LovyanGFX Light_PWM instance,
// ledcWrite under the hood) and load the persisted config from NVS.
void displaycfgBegin();

// Apply the current backlight base value (with gamma) to the panel.
void displaycfgApplyBrightness(const IsisCfg &cfg);

// Push the composed 480x480 scene sprite to the panel, honouring scale/offset.
// `identity == true` performs a byte-exact 1:1 copy.
void displaycfgComposite(const IsisCfg &cfg);

// Persist the given field mask into NVS (namespace "isis").
void displaycfgSaveNvs(const IsisCfg &cfg, uint32_t mask);

// Read the NVS layer (namespace "isis").
uint32_t displaycfgLoadNvs(IsisCfg &out);

// Factory reset of the display config part of NVS.
void displaycfgEraseNvs();

// The effective (resolved) configuration used by the whole application.
// Priority FDC > NVS > default is applied by the caller through
// displaycfgResolve(); layers are held in net.cpp / main.cpp.
IsisCfg &displaycfgCurrent();
#endif
