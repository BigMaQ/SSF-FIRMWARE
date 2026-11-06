//
// grid.cpp — Kalibrier-Testmuster (siehe grid.h).
//
// Zeichnet ins Kompositions-Canvas `lcd`. ACHTUNG Rollen:
//   lcd      = 480x480-Canvas, hier wird gezeichnet
//   lcdPanel = das echte Display, wird NUR von displaycfgComposite() beschrieben
// Wer direkt auf lcdPanel malt, wird vom naechsten Composite ueberschrieben -
// das ergibt genau das Flackern zwischen Muster und Instrument.
//
#include "grid.h"

#include "displaycanvas.h"

// Muss das Muster im Canvas neu gezeichnet werden?
static bool g_gridRedraw = true;   // extern IsisCanvas lcd

namespace {

bool     g_aktiv = false;
uint32_t g_letzteZeichnungMs = 0;

const int KANTE = 480;

uint16_t farbeFuerSchritt(int i)
{
    switch (i) {
        case 240: return TFT_RED;      // Mitte
        case 120:
        case 360: return TFT_GREEN;    // Viertel
        case 60:
        case 180:
        case 300:
        case 420: return TFT_YELLOW;   // Achtel
        default:  return TFT_BLUE;
    }
}

}  // namespace

bool gridToggle()
{
    g_gridRedraw = true;   // naechster Frame zeichnet das Muster neu
    g_aktiv = !g_aktiv;
    g_letzteZeichnungMs = 0;
    Serial.printf("[grid] Testmuster %s\n", g_aktiv ? "AN" : "AUS");
    return g_aktiv;
}

bool gridActive()
{
    return g_aktiv;
}

void gridDraw()
{
    if (!g_aktiv) return;

    // Das Canvas wird jeden Frame von isis.update() neu beschrieben — das
    // Muster muss also jeden Frame neu drauf. Reines Zeichnen ohne Font,
    // deshalb kostet es kaum etwas.
    lcd.fillScreen(TFT_BLACK);

    // Aussenkante: hier endet das Display. Was davon nicht zu sehen ist,
    // liegt unter dem Rahmen.
    lcd.drawRect(0, 0, KANTE, KANTE, TFT_WHITE);
    lcd.drawRect(1, 1, KANTE - 2, KANTE - 2, TFT_WHITE);

    // Raster alle 60 px, farbcodiert.
    for (int i = 60; i < KANTE; i += 60) {
        const uint16_t c = farbeFuerSchritt(i);
        lcd.drawLine(i, 0, i, KANTE - 1, c);
        lcd.drawLine(i + 1, 0, i + 1, KANTE - 1, c);
        lcd.drawLine(0, i, KANTE - 1, i, c);
        lcd.drawLine(0, i + 1, KANTE - 1, i + 1, c);
    }

    // Mittelpunkt deutlich markieren.
    lcd.fillCircle(KANTE / 2, KANTE / 2, 8, TFT_RED);
    lcd.drawCircle(KANTE / 2, KANTE / 2, 9, TFT_WHITE);
}

// Muster im Canvas neu zeichnen? Wird nach dem Zeichnen zurueckgesetzt, damit
// der Hauptloop nicht jeden Frame neu komponiert (Skalierung ist teuer).
bool gridTakeRedraw()
{
    const bool r = g_gridRedraw;
    g_gridRedraw = false;
    return r;
}
