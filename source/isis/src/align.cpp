//
// align.cpp — Touch-Ausrichtwerkzeug fuer das ISIS-Panel.
//
// Zeichnet bewusst direkt auf `lcdPanel` (das echte Display). Die
// Bedienoberflaeche darf NICHT durch displaycfgComposite() laufen, sonst
// wuerde sie mitskaliert und die Touch-Koordinaten passten nicht mehr zu den
// Schaltflaechen. Das Bild darunter (Raster/Instrument) wird weiterhin
// regulaer komponiert; die Leiste liegt einfach oben drauf.
//
// Schrift: Auf lcdPanel sind die LovyanGFX-Fonts nutzbar. Auf dem
// 8-Bit-Palette-Canvas `lcd` fuehrt drawString() zum Crash - hier ist es
// erlaubt, weil lcdPanel ein echter LGFX-Treiber ist.
//
#include "align.h"

#include "displaycanvas.h"
#include "displaycfg.h"
#include "grid.h"

#include <LovyanGFX.hpp>

namespace {

IsisCfg *g_cfg      = nullptr;   // zeigt auf die Config der Firmware
bool     g_aktiv    = false;
bool     g_saveReq  = false;
bool     g_aspect   = true;      // Keep Aspect Ratio, Vorgabe: an
uint32_t g_lastTouch = 0;
int16_t  g_tx = -1, g_ty = -1;   // letzter Touch (fuer den Fadenkreuz-Punkt)
bool     g_dirty     = true;     // Leiste neu zeichnen?
bool     g_redraw    = true;     // Bild im Canvas neu komponieren?
bool     g_gedrueckt = false;    // war beim letzten Frame ein Finger drauf?

// --- Layout im Display-Raum (480x480) ---------------------------------------
constexpr int BAR_Y      = 374;  // Oberkante der Bedienleiste
constexpr int BTN_Y      = 378;
constexpr int BTN_H      = 46;
constexpr int TEXT_Y     = 432;

struct Taste { int x, w; const char *label; };

// Acht Tasten nebeneinander: 8*56 + 7*4 = 476 -> passt in 480.
// Zehn Tasten: Breite und Hoehe getrennt, sonst laesst sich scale_h nicht
// einstellen. 10*44 + 9*2 = 458 -> passt in 480.
constexpr int BTN_W = 44;
constexpr int BTN_GAP = 2;

Taste taste(int i)
{
    return Taste{ 4 + i * (BTN_W + BTN_GAP), BTN_W, "" };
}

// Breite/Hoehe getrennt: W-/W+ und H-/H+. "<" ">" "^" "v" verschieben.
const char *LABELS[10] = { "W-", "W+", "H-", "H+", "<", ">", "^", "v", "ASP", "SAV" };
constexpr int TASTEN = 10;

// Schrittweiten
constexpr int SCALE_STEP = 8;
constexpr int OFF_STEP   = 8;

void clampiere()
{
    if (!g_cfg) return;
    if (g_cfg->scaleW < ISIS_SCALE_MIN) g_cfg->scaleW = ISIS_SCALE_MIN;
    if (g_cfg->scaleW > ISIS_SCALE_MAX) g_cfg->scaleW = ISIS_SCALE_MAX;
    if (g_cfg->scaleH < ISIS_SCALE_MIN) g_cfg->scaleH = ISIS_SCALE_MIN;
    if (g_cfg->scaleH > ISIS_SCALE_MAX) g_cfg->scaleH = ISIS_SCALE_MAX;
    if (g_aspect) g_cfg->scaleH = g_cfg->scaleW;
    if (g_cfg->offX < ISIS_OFF_MIN) g_cfg->offX = ISIS_OFF_MIN;
    if (g_cfg->offX > ISIS_OFF_MAX) g_cfg->offX = ISIS_OFF_MAX;
    if (g_cfg->offY < ISIS_OFF_MIN) g_cfg->offY = ISIS_OFF_MIN;
    if (g_cfg->offY > ISIS_OFF_MAX) g_cfg->offY = ISIS_OFF_MAX;
}

// Orientierungshilfe: beschriftete Marker an den vier Raendern. Sie werden im
// Koordinatensystem von lcdPanel gezeichnet - erscheint "OBEN" also z.B. links,
// liegt das Panel mechanisch um 90 Grad verdreht zu dem, was die Firmware
// annimmt (Rotation 3 = USB-Anschluss unten).
void zeichneOrientierung()
{
    lcdPanel.setFont(&fonts::Font2);
    lcdPanel.setTextSize(1);
    lcdPanel.setTextDatum(middle_center);

    auto marke = [](int x, int y, const char *txt, uint16_t fg, uint16_t bg) {
        const int w = 76, h = 22;
        lcdPanel.fillRect(x - w/2, y - h/2, w, h, bg);
        lcdPanel.drawRect(x - w/2, y - h/2, w, h, fg);
        lcdPanel.setTextColor(fg, bg);
        lcdPanel.drawString(txt, x, y);
    };

    marke(240,  14, "OBEN",  TFT_WHITE, TFT_MAROON);
    marke(240, 466, "UNTEN", TFT_WHITE, TFT_NAVY);
    marke( 52, 240, "LINKS", TFT_BLACK, TFT_YELLOW);
    marke(428, 240, "RECHTS",TFT_BLACK, TFT_ORANGE);

    // Ein Pfeil nach oben macht die Leserichtung eindeutig.
    lcdPanel.fillTriangle(240, 30, 228, 54, 252, 54, TFT_WHITE);
    lcdPanel.fillRect(236, 54, 8, 18, TFT_WHITE);
}

void zeichneTaste(int i, bool aktiv)
{
    const Taste t = taste(i);
    lcdPanel.fillRect(t.x, BTN_Y, t.w, BTN_H, aktiv ? TFT_DARKGREEN : TFT_NAVY);
    lcdPanel.drawRect(t.x, BTN_Y, t.w, BTN_H, TFT_WHITE);
    lcdPanel.setTextColor(TFT_WHITE, aktiv ? TFT_DARKGREEN : TFT_NAVY);
    lcdPanel.setTextDatum(middle_center);
    lcdPanel.drawString(LABELS[i], t.x + t.w / 2, BTN_Y + BTN_H / 2);
}

void zeichneLeiste()
{
    // Leiste deckend leeren, damit kein Bild darunter durchscheint.
    lcdPanel.fillRect(0, BAR_Y, 480, 480 - BAR_Y, TFT_BLACK);
    lcdPanel.drawFastHLine(0, BAR_Y, 480, TFT_WHITE);

    lcdPanel.setFont(&fonts::Font2);

    for (int i = 0; i < TASTEN; ++i) zeichneTaste(i, false);

    // Die vierte Taste ist der Aspect-Schalter: Beschriftung je nach Zustand.
    {
        const Taste t = taste(8);
        lcdPanel.fillRect(t.x, BTN_Y, t.w, BTN_H, g_aspect ? TFT_DARKGREEN : TFT_DARKGREY);
        lcdPanel.drawRect(t.x, BTN_Y, t.w, BTN_H, TFT_WHITE);
        lcdPanel.setTextColor(TFT_WHITE, g_aspect ? TFT_DARKGREEN : TFT_DARKGREY);
        lcdPanel.setTextDatum(middle_center);
        lcdPanel.drawString(g_aspect ? "ASP!" : "ASP", t.x + t.w / 2, BTN_Y + BTN_H / 2);
    }

    // Wertezeile
    char buf[64];
    snprintf(buf, sizeof(buf), "%d/%d   off %d/%d   %s",
             g_cfg ? g_cfg->scaleW : 0, g_cfg ? g_cfg->scaleH : 0,
             g_cfg ? g_cfg->offX : 0,  g_cfg ? g_cfg->offY : 0,
             g_aspect ? "ASPECT" : "frei");
    lcdPanel.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcdPanel.setTextDatum(top_left);
    lcdPanel.drawString(buf, 8, TEXT_Y);

    // Letzten Touch als kleinen Punkt zeigen (Plausibilitaetscheck: kommt der
    // Touch dort an, wo man hindrueckt?)
    if (g_tx >= 0 && g_ty >= 0) {
        lcdPanel.fillCircle(g_tx, g_ty, 4, TFT_RED);
    }

    zeichneOrientierung();
}


void verarbeiteTouch(int16_t x, int16_t y)
{
    if (!g_cfg) return;
    g_tx = x; g_ty = y;

    // Unterhalb der Leiste -> Tasten
    if (y >= BTN_Y && y <= BTN_Y + BTN_H) {
        for (int i = 0; i < TASTEN; ++i) {
            const Taste t = taste(i);
            if (x < t.x || x > t.x + t.w) continue;
            switch (i) {
                case 0: g_cfg->scaleW -= SCALE_STEP; break;   // Breite kleiner
                case 1: g_cfg->scaleW += SCALE_STEP; break;   // Breite groesser
                case 2: g_cfg->scaleH -= SCALE_STEP; break;   // Hoehe kleiner
                case 3: g_cfg->scaleH += SCALE_STEP; break;   // Hoehe groesser
                case 4: g_cfg->offX   -= OFF_STEP;   break;   // links
                case 5: g_cfg->offX   += OFF_STEP;   break;   // rechts
                case 6: g_cfg->offY   -= OFF_STEP;   break;   // hoch
                case 7: g_cfg->offY   += OFF_STEP;   break;   // runter
                case 8: g_aspect = !g_aspect;        break;   // Aspect
                case 9: g_saveReq = true;            break;   // SAVE
                default: break;
            }
            // Aspect an: die gerade veraenderte Kante zieht die andere nach.
            if (g_aspect) {
                if (i <= 1) g_cfg->scaleH = g_cfg->scaleW;
                else if (i <= 3) g_cfg->scaleW = g_cfg->scaleH;
            }
            clampiere();
            gridDraw();          // Raster neu, damit man die Wirkung sieht
            g_dirty  = true;
            g_redraw = true;     // Bild neu komponieren
            return;
        }
    }

    // Oben rechts: Schliessen (X)
    if (x >= 424 && x <= 476 && y >= 0 && y <= 40) {
        g_aktiv = false;
        return;
    }
}

} // namespace

bool alignBegin(IsisCfg *cfg)
{
    g_cfg = cfg;
    return g_cfg != nullptr;
}

bool alignToggle()
{
    if (!g_cfg) return false;
    g_aktiv = !g_aktiv;
    if (g_aktiv) {
        g_aspect = (g_cfg->scaleW == g_cfg->scaleH);
        gridDraw();            // Raster als Bezug einschalten
        g_dirty  = true;
        g_redraw = true;
        g_gedrueckt = false;   // nicht aus einem alten Druck heraus starten
    }
    return g_aktiv;
}

bool alignActive() { return g_aktiv; }
int  alignScale()  { return g_cfg ? g_cfg->scaleW : 0; }
int  alignOffX()   { return g_cfg ? g_cfg->offX : 0; }
int  alignOffY()   { return g_cfg ? g_cfg->offY : 0; }
bool alignKeepAspect() { return g_aspect; }

void alignForceRedraw()
{
    g_dirty = true;
}

// Wird true, wenn das Bild (Raster/Instrument) neu komponiert werden muss.
bool alignTakeRedraw()
{
    const bool r = g_redraw;
    g_redraw = false;
    return r;
}

bool alignTakeSaveRequest()
{
    const bool r = g_saveReq;
    g_saveReq = false;
    if (r && g_cfg) {
        // Genau die vier Felder persistieren - Backlight/OTA/Netz bleiben
        // unangetastet, sonst wuerde ein versehentlicher Save die Zuordnung
        // des Panels ueberschreiben.
        displaycfgSaveNvs(*g_cfg, CFG_SCALE_W | CFG_SCALE_H | CFG_OFF_X | CFG_OFF_Y);
    }
    return r;
}

void alignTick()
{
    if (!g_aktiv || !g_cfg) return;

    // Flankenauswertung statt Zeitfenster: Ein Druck loest GENAU EINEN Schritt
    // aus. Vorher wurde alle 160 ms nachgetriggert, solange der Finger lag -
    // das ergab die Doppelspruenge.
    uint16_t tx = 0, ty = 0;
    const bool finger = lcdPanel.getTouch(&tx, &ty) > 0;
    if (finger && !g_gedrueckt) {
        g_gedrueckt = true;
        g_lastTouch = millis();
        verarbeiteTouch((int16_t)tx, (int16_t)ty);
        g_dirty = true;
    } else if (!finger && g_gedrueckt) {
        g_gedrueckt = false;
        g_dirty = true;          // Punkt verschwindet wieder
    }

    // Nur neu zeichnen, wenn sich etwas geaendert hat. Jeden Frame zu malen
    // liess die Leiste flackern.
    if (g_dirty) {
        g_dirty = false;
        zeichneLeiste();
    }
}
