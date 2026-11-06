//
// main.cpp — A320 ISIS panel, own firmware (no MobiFlight).
//
// Replaces the MobiFlight firmware core: setup(), loop(), WiFi, display
// initialisation, PSRAM sprite allocation and the watchdog.
//
// Data path:  UDP 5005 -> proto.cpp (parser) -> isisState -> CC_ISIS::update()
//             (unchanged upstream graphics) -> canvas -> displaycfgComposite()
//
// Display/graphics code: MIT License, Copyright (c) 2024 Chris Crawford / elral
// (https://github.com/ccrawford/CC_ISIS) - see LICENSE_CC_ISIS.txt.
//
// Build:     pio run -e esp32s3
// Flash:     pio run -e esp32s3 -t upload          (USB, first time only)
// Tests:     pio test -e native
//
#include <Arduino.h>
#include <esp_task_wdt.h>

#include "demo.h"
#include "align.h"
#include "grid.h"        // Kalibrier-Testmuster        // BLANK / RESET / SHOW
#include "CC_ISIS.h"
#include "ISISCommon.h"
#include "displaycanvas.h"
#include "displaycfg.h"
#include "imgstream.h"   // Bildstrom (GlassOut -> FDC -> UDP 5007)
#include "net.h"
#include "portal.h"   // isisSerialPoll(): Konsolenbefehle
#include "Sprites/LangText.h"
#include "Sprites/OtaDigits.h"
#include "buttons.h"

#ifndef ISIS_FW_VERSION
#define ISIS_FW_VERSION "0.0.0"
#endif

#ifndef ISIS_CANVAS_SIZE
#define ISIS_CANVAS_SIZE 480
#endif

#define ISIS_FRAME_INTERVAL_MS 50   // 20 Hz render/service tick
#define ISIS_WDT_TIMEOUT_S     30

CC_ISIS isis;

namespace {

uint32_t g_lastTickMs = 0;

// Snapshot of the smoothed values to detect real changes (spec risk table:
// "draw() nur bei Aenderung").
struct SmoothSnapshot {
    float pitch = 0.0f;
    float bank  = 0.0f;
    float speed = 0.0f;
    float alt   = 0.0f;
};

SmoothSnapshot snapshot()
{
    SmoothSnapshot s;
    s.pitch = isisState.pitchAngle;
    s.bank  = isisState.bankAngle;
    s.speed = isisState.airspeed;
    s.alt   = isisState.altitude;
    return s;
}

bool changed(const SmoothSnapshot &a, const SmoothSnapshot &b)
{
    return fabsf(a.pitch - b.pitch) > 0.0001f ||
           fabsf(a.bank - b.bank) > 0.0001f ||
           fabsf(a.speed - b.speed) > 0.0001f ||
           fabsf(a.alt - b.alt) > 0.0001f;
}

} // namespace

#if defined(ISIS_ESP32) && ISIS_BOOT_TRACE
// ---------------------------------------------------------------------------
// Boot-Trace (temporaer, nur mit -DISIS_BOOT_TRACE=1): zeigt, wie weit der
// Start kommt. Ein Haenger VOR setup() laesst sich sonst nicht von einem
// Haenger IN setup() unterscheiden, weil Serial erst dort geoeffnet wird.
// ---------------------------------------------------------------------------
static struct BootTrace {
    BootTrace()
    {
        Serial.begin(115200);
        delay(120);
        Serial.println("### TRACE: globale Konstruktoren laufen (main.cpp) ###");
        Serial.flush();
    }
} g_bootTrace;
#endif

void setup()
{
#if defined(ISIS_ESP32) && ISIS_BOOT_TRACE
    Serial.println("### TRACE: setup() betreten ###");
    Serial.flush();
#endif
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("=== A320 ISIS (own firmware, no MobiFlight) ===");
    Serial.printf("FW %s   PSRAM %u bytes free / %u total\n", ISIS_FW_VERSION,
                  (unsigned)ESP.getFreePsram(), (unsigned)ESP.getPsramSize());

    // ---- watchdog (spec section 10, criterion 8) ----
    esp_task_wdt_init(ISIS_WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    // ---- buttons first: BRT+ / BRT- held at boot forces the setup mode ----
    bool needSetup = false;
    const char *setupReason = "";

    if (buttonsBegin()) {
        const uint8_t raw = buttonsReadRaw();
        if (buttonsBootCombo(raw)) {
            needSetup   = true;
            setupReason = "boot combo BRT+ / BRT-";
        }
    } else {
        Serial.println("[isis] no PCF8574 found - buttons disabled");
    }

    // ---- display: panel + composition canvas ----
#if defined(ISIS_ESP32) && ISIS_BOOT_TRACE
    Serial.println("### TRACE: buttons fertig, rufe displaycfgBegin() ###"); Serial.flush();
#endif
    displaycfgBegin();
#if defined(ISIS_ESP32) && ISIS_BOOT_TRACE
    Serial.println("### TRACE: displaycfgBegin() zurueck ###"); Serial.flush();
#endif

    // ---- stored settings (NVS namespace "isis", spec section 2) ----
    // Must run before isis.begin(): CC_ISIS reads isisSettings.lcdBrightness
    // and the unit settings from it.  A virgin NVS is filled with defaults.
    if (!loadSettings()) {
        Serial.println("[isis] settings: defaults (fresh NVS)");
    } else {
        Serial.printf("[isis] settings: v%d, brightness %u, unit %u\n",
                      isisSettings.version, (unsigned)isisSettings.lcdBrightness,
                      (unsigned)isisSettings.baroUnit);
    }

#if defined(ISIS_ESP32) && ISIS_BOOT_TRACE
    Serial.println("### TRACE: settings geladen, Sprite folgt ###"); Serial.flush();
#endif
    // WICHTIG: PSRAM benutzen und 16 Bit Farbtiefe.
    // 480x480 @8 bit = 230 KB - allein das sprengt den internen RAM des S3,
    // dann scheitern die grossen Zeichen-Sprites (Attitude 320x350) und es
    // bleibt nur der kleine Kohlsman-Sprite sichtbar.  16 Bit passt in PSRAM
    // (460 KB von 8 MB) und entspricht dem Farbmodell des Panels, damit die
    // 8-Bit-Paletten-Sprites korrekt konvertiert werden.
    lcd.setPsram(true);
    lcd.setColorDepth(16);
    if (!lcd.createSprite(ISIS_CANVAS_SIZE, ISIS_CANVAS_SIZE)) {
        Serial.println("[isis] FATAL: 480x480 canvas could not be allocated (PSRAM!)");
    }
    lcd.fillSprite(TFT_BLACK);

    // ---- the unchanged upstream display device ----
#if defined(ISIS_ESP32) && ISIS_BOOT_TRACE
    Serial.println("### TRACE: rufe isis.begin() ###"); Serial.flush();
#endif
    isis.begin();
#if defined(ISIS_ESP32) && ISIS_BOOT_TRACE
    Serial.println("### TRACE: isis.begin() zurueck ###"); Serial.flush();
#endif

    // ---- network ----
    if (!needSetup) {
        if (!netBegin()) {
            needSetup   = true;
            setupReason = "keine WLAN-Verbindung innerhalb 30 s (Zugangsdaten siehe Log)";
        }
    }

    if (needSetup) {
        // SoftAP + captive portal, returns only to reboot (spec section 4.1).
        netEnterSetup(setupReason);
    }

    // First full paint.
    isisState.forceRedraw = true;
    isis.update();
    alignBegin(&displaycfgCurrent());
    displaycfgComposite(displaycfgCurrent());

    // Bildstrom-Empfaenger (UDP 5007) - Puffer in PSRAM, eigener Socket.
    // Startet im Modus "lokales Zeichnen"; erst CFG:IMG=1; schaltet um.
    imgstreamBegin();

    g_lastTickMs = millis();
    Serial.println("[isis] ready");
}

// ---------------------------------------------------------------------------
// Statusanzeige ueber dem Instrument (Airbus-Gruen wie beim PFD-Boot).
//   FDC antwortet nicht  -> "FDC NOT CONNECTED"
//   FDC da, kein Sim     -> "SIM NOT CONNECTED"
// ---------------------------------------------------------------------------
static int g_lastLink = -1;

static void drawLinkBanner(int status)
{
    // Ins Kompositions-Canvas `lcd` zeichnen, damit displaycfgComposite() das
    // Banner wie das Instrument skaliert. Wer auf lcdPanel (das echte Display)
    // malt, wird vom naechsten Composite ueberschrieben -> Flackern.
    IsisCanvas &ziel = lcd;
    const uint16_t gruen = 0x07E0;          // PFD-Gruen
    // Rahmen etwas kleiner als frueher (448x88): die Bitmaps werden mit
    // Schriftgroesse 22/13 erzeugt (269x36 bzw. 135x22) und brauchen den
    // Platz nicht mehr.
    const int y = 202;
    const int rahmenX = 60, rahmenW = 360, rahmenH = 78;
    ziel.fillRect(rahmenX, y, rahmenW, rahmenH, TFT_BLACK);
    ziel.drawRect(rahmenX, y, rahmenW, rahmenH, gruen);
    ziel.drawRect(rahmenX + 1, y + 1, rahmenW - 2, rahmenH - 2, gruen);

    // Sprache folgt dem FDC (LANG:de/en/fr/th). Texte liegen als Bitmaps in
    // Flash - so sind Thai-Schrift und franzoesische Akzente korrekt, ohne
    // eine Unicode-Schrift auf dem Panel zu brauchen.
    const LangBanner *bsprache = langBanner(netLanguage());
    const bool fdcWeg = (status == NET_LINK_FDC_OFFLINE);

    const uint8_t *haupt = fdcWeg ? bsprache->fdc_main : bsprache->sim_main;
    const int hauptW = fdcWeg ? bsprache->fdc_main_w : bsprache->sim_main_w;
    const int hauptH = fdcWeg ? bsprache->fdc_main_h : bsprache->sim_main_h;
    const uint8_t *unter = fdcWeg ? bsprache->fdc_sub : bsprache->sim_sub;
    const int unterW = fdcWeg ? bsprache->fdc_sub_w : bsprache->sim_sub_w;
    const int unterH = fdcWeg ? bsprache->fdc_sub_h : bsprache->sim_sub_h;

    if (hauptW <= rahmenW - 24) {
        ziel.drawBitmap(240 - hauptW / 2, y + 8, haupt, hauptW, hauptH, gruen);
    }
    if (unterW <= rahmenW - 24) {
        ziel.drawBitmap(240 - unterW / 2, y + 48, unter, unterW, unterH, gruen);
    }
}

// ---------------------------------------------------------------------------
// Overlay waehrend eines Firmware-Updates (OTA)
//   Der Bildschirm zeigt sonst Instrumente oder ein Verbindungsbanner. Beim
//   Flashen ist beides falsch: das Panel schreibt gerade in den zweiten
//   App-Slot und startet danach neu. Deshalb Vollbild-Overlay in der
//   FDC-Sprache, mit Fortschritt und der Warnung, den Strom nicht zu kappen.
// ---------------------------------------------------------------------------
// --- OTA-Overlay: Grafikdaten in PSRAM spiegeln -----------------------------
// Waehrend eines Updates schreibt espota in den zweiten App-Slot, und in dieser
// Zeit ist der Flash-Cache abgeschaltet. Bitmaps und Ziffern, die im Flash
// liegen (.rodata), duerfen dann NICHT gelesen werden - das hat in v0.2.3 zu
// einem Panic-Reboot mitten im Upload gefuehrt. Deshalb werden sie beim
// Oeffnen des OTA-Fensters einmal nach PSRAM kopiert und von dort gezeichnet.
// Geometrie des OTA-Overlays (480x480)
static const int OTA_RAHMEN_Y = 130, OTA_RAHMEN_H = 220;
static const int OTA_TITEL_Y  = 146;
static const int OTA_BALKEN_X = 60,  OTA_BALKEN_Y = 212;
static const int OTA_BALKEN_W = 360, OTA_BALKEN_H = 28;
static const int OTA_ZAHL_Y   = 274;   // Mitte der Prozentzahl
static const int OTA_UNTEN_Y  = 302;

static uint8_t *g_otaTitel  = nullptr;
static uint8_t *g_otaUnter  = nullptr;
static int g_otaTitelW = 0, g_otaTitelH = 0;
static int g_otaUnterW = 0, g_otaUnterH = 0;
static uint8_t *g_otaGlyph[11] = {nullptr};
static uint8_t g_otaGlyphW[11] = {0};
static uint8_t g_otaGlyphH[11] = {0};
static bool    g_otaArtBereit = false;

static uint8_t *psramKopie(const uint8_t *quelle, int bytes)
{
    if (!quelle || bytes <= 0) return nullptr;
    uint8_t *ziel = (uint8_t *)ps_malloc(bytes);
    if (!ziel) return nullptr;
    memcpy(ziel, quelle, bytes);
    return ziel;
}

static void otaArtInPsram()
{
    if (g_otaArtBereit) return;
    const LangBanner *sp = langBanner(netLanguage());
    g_otaTitelW = sp->ota_main_w; g_otaTitelH = sp->ota_main_h;
    g_otaUnterW = sp->ota_sub_w;  g_otaUnterH = sp->ota_sub_h;
    g_otaTitel = psramKopie(sp->ota_main, (g_otaTitelW * g_otaTitelH + 7) / 8);
    g_otaUnter = psramKopie(sp->ota_sub,  (g_otaUnterW * g_otaUnterH + 7) / 8);
    for (int i = 0; i < 11; ++i) {
        const OtaGlyph &g = OTA_GLYPHEN[i];
        g_otaGlyphW[i] = g.breite;
        g_otaGlyphH[i] = g.hoehe;
        g_otaGlyph[i] = psramKopie(g.daten, (g.breite * g.hoehe + 7) / 8);
    }
    g_otaArtBereit = true;
    Serial.printf("[isis] OTA-Grafik in PSRAM (%d + %d + 11 Zeichen)\n",
                  (g_otaTitelW * g_otaTitelH + 7) / 8, (g_otaUnterW * g_otaUnterH + 7) / 8);
}

// Prozentzahl aus den Ziffern-Bitmaps, mittig um (cx, cy)
static void otaZeichneZahl(int cx, int cy, int prozent)
{
    char txt[8];
    snprintf(txt, sizeof(txt), "%d", prozent < 0 ? 0 : prozent);
    const int luecke = 4, pctLuecke = 10;
    int idx[8] = {0};
    int n = 0;
    for (const char *p = txt; *p && n < 8; ++p) {
        if (*p >= '0' && *p <= '9') idx[n++] = *p - '0';
    }
    if (!g_otaArtBereit) return;
    int breite = pctLuecke + g_otaGlyphW[10];
    for (int i = 0; i < n; ++i) breite += (i ? luecke : 0) + g_otaGlyphW[idx[i]];
    int x = cx - breite / 2;
    for (int i = 0; i < n; ++i) {
        if (i) x += luecke;
        if (g_otaGlyph[idx[i]]) {
            lcd.drawBitmap(x, cy - g_otaGlyphH[idx[i]] / 2, g_otaGlyph[idx[i]],
                           g_otaGlyphW[idx[i]], g_otaGlyphH[idx[i]], TFT_WHITE);
        }
        x += g_otaGlyphW[idx[i]];
    }
    x += pctLuecke;
    if (g_otaGlyph[10]) {
        lcd.drawBitmap(x, cy - g_otaGlyphH[10] / 2, g_otaGlyph[10],
                       g_otaGlyphW[10], g_otaGlyphH[10], TFT_WHITE);
    }
}

// Nur Balken und Zahl neu zeichnen - waehrend des Uploads kein Vollbild-Push
static void drawOtaProgress(int prozent)
{
    if (prozent < 0) prozent = 0;
    if (prozent > 100) prozent = 100;
    const uint16_t amber = 0xFD20;
    const uint16_t gruen = 0x07E0;
    // KEIN Serial/printf hier: waehrend des Uploads ist der Flash-Cache aus,
    // ein UART-Interrupt mit Flash-Code fuehrt zum Panic-Reboot. Stattdessen
    // nur mitzaehlen - abfragbar per OTADBG nach dem Update.
    netOtaCountDraw(prozent);
    lcd.fillRect(OTA_BALKEN_X + 1, OTA_BALKEN_Y + 1, OTA_BALKEN_W - 2, OTA_BALKEN_H - 2, TFT_BLACK);
    lcd.drawRect(OTA_BALKEN_X, OTA_BALKEN_Y, OTA_BALKEN_W, OTA_BALKEN_H, amber);
    const int fuell = (OTA_BALKEN_W - 4) * prozent / 100;
    if (fuell > 0) lcd.fillRect(OTA_BALKEN_X + 2, OTA_BALKEN_Y + 2, fuell, OTA_BALKEN_H - 4, gruen);
    lcd.fillRect(OTA_BALKEN_X, OTA_BALKEN_Y + OTA_BALKEN_H + 2, OTA_BALKEN_W, 56, TFT_BLACK);
    otaZeichneZahl(240, OTA_ZAHL_Y, prozent);
    netOtaCountDraw(prozent);   // Vollbild = auch ein Zeichen-Takt
}

static void drawOtaOverlay(int prozent)
{
    const uint16_t amber = 0xFD20;   // auffaellig, aber nicht grell
    const uint16_t gruen = 0x07E0;

    if (prozent < 0) prozent = 0;
    if (prozent > 100) prozent = 100;
    otaArtInPsram();   // erst spiegeln, dann (ggf. aus PSRAM) zeichnen

    lcd.fillScreen(TFT_BLACK);
    lcd.drawRect(16, OTA_RAHMEN_Y, 448, OTA_RAHMEN_H, amber);
    lcd.drawRect(17, OTA_RAHMEN_Y + 1, 446, OTA_RAHMEN_H - 2, amber);

    const LangBanner *sp = langBanner(netLanguage());
    // Aus PSRAM zeichnen, sobald gespiegelt - sonst direkt aus dem Flash
    const uint8_t *titel = g_otaArtBereit ? g_otaTitel : sp->ota_main;
    const int titelW = g_otaArtBereit ? g_otaTitelW : sp->ota_main_w;
    const int titelH = g_otaArtBereit ? g_otaTitelH : sp->ota_main_h;
    const uint8_t *unter = g_otaArtBereit ? g_otaUnter : sp->ota_sub;
    const int unterW = g_otaArtBereit ? g_otaUnterW : sp->ota_sub_w;
    const int unterH = g_otaArtBereit ? g_otaUnterH : sp->ota_sub_h;

    if (titel && titelW <= 430) {
        lcd.drawBitmap(240 - titelW / 2, OTA_TITEL_Y, titel, titelW, titelH, TFT_WHITE);
    }

    // Fortschrittsbalken
    lcd.drawRect(OTA_BALKEN_X, OTA_BALKEN_Y, OTA_BALKEN_W, OTA_BALKEN_H, amber);
    const int fuell = (OTA_BALKEN_W - 4) * prozent / 100;
    if (fuell > 0) lcd.fillRect(OTA_BALKEN_X + 2, OTA_BALKEN_Y + 2, fuell, OTA_BALKEN_H - 4, gruen);

    otaZeichneZahl(240, OTA_ZAHL_Y, prozent);

    if (unter && unterW <= 430) {
        lcd.drawBitmap(240 - unterW / 2, OTA_UNTEN_Y, unter, unterW, unterH, amber);
    }

    // Zurueck auf die Standardeinstellungen des Instruments
    lcd.setTextFont(&fonts::Font0);
    lcd.setTextDatum(top_left);
}

static uint32_t g_lastWakeMs = 0;   // letzter Zeitpunkt mit Flugdaten

void loop()
{
    const uint32_t now = millis();
    esp_task_wdt_reset();

    // ---- network: receive frames, beacon, OTA ----
    netLoop(now);

    // ---- Bildstrom: Chunks abholen, reassemblieren, direkt aufs Panel ----
    // Laeuft unabhaengig vom 20-Hz-Render-Takt: jedes ankommende Bild wird
    // sofort gezeichnet, damit keine Frames im Taktfenster verfallen.
    imgstreamService(now);

    // ---- OTA laeuft: Zeichnen aussetzen ----
    // Der SPI-Push des 480x480-Canvas bremst den Upload so stark, dass espota
    // am Ende mit "Error Uploading" abbricht. Waehrend des Schreibvorgangs
    // bekommt der OTA-Stack die Loop-Zeit allein.
    if (netOtaDemoPct() >= 0) {
        // Nur Optik-Abstimmung (OTADEMO): Overlay zeigen, Konsole bleibt
        // bedienbar - sonst kaeme man an keinen Screenshot und nicht wieder raus.
        static int lastDemo = -2;
        const int demo = netOtaDemoPct();
        if (demo != lastDemo) {
            lastDemo = demo;
            drawOtaOverlay(demo);
        }
        isisSerialPoll();
        return;
    }

    // Firmware-Update: Overlay zeigen, sobald das OTA-Fenster offen ist.
    // Waehrend der Upload wirklich schreibt, wird NICHT gezeichnet: dabei ist
    // der Flash-Cache abgeschaltet, und Titel/Untertitel liegen als Bitmaps im
    // Flash (.rodata) - ein Lesezugriff fuehrt zum Panic-Reboot ("No core dump
    // partition"). Deshalb statisch bei 0 % stehen lassen und nur den
    // OTA-Stack bedienen.
    static bool otaOverlayGezeigt = false;
    if (netOtaInProgress()) {
        // Fortschritt nur in 5-%-Schritten zeichnen: jeder Push blockiert kurz,
        // und in den Schreibphasen ist Flash-Lesen tabu (siehe otaArtInPsram).
        static int gezeigt = -5;
        const int pct = netOtaProgress();
        if (pct >= 0 && (pct - gezeigt >= 5 || pct == 100)) {
            gezeigt = pct;
            drawOtaProgress(pct);
        }
        for (int i = 0; i < 4; ++i) {
            netServiceOta();
            delay(1);
        }
        esp_task_wdt_reset();
        return;
    }
    if (netOtaWindowOpen(millis())) {
        if (!otaOverlayGezeigt) {
            otaOverlayGezeigt = true;
            drawOtaOverlay(netOtaProgress() < 0 ? 0 : netOtaProgress());
        }
    } else {
        otaOverlayGezeigt = false;
    }

    // ---- buttons ----
    buttonsService(now);

    // ---- Konsole (WIFI, INFO, LSPOS, DIM) auch im Normalbetrieb ----
    isisSerialPoll();

    // ---- Bildschirmschoner ----
    // Ohne Simulator steht stundenlang dieselbe Meldung auf dem Display. Kein
    // Einbrennen (das ist ein IPS-Panel), aber das Backlight altert und die
    // Meldung blendet. Nach der Ruhezeit abblenden; sobald wieder Flugdaten
    // kommen, sofort zurueck auf normale Helligkeit.
    if (netStaConnected()) {
        if (netLinkStatus(now) == NET_LINK_OK) {
            g_lastWakeMs = now;
            if (isisDimPercent() < 100) isisSetDim(100);
        } else if (isisDimPercent() == 100 &&
                   (uint32_t)(now - g_lastWakeMs) >= (uint32_t)isisDimSeconds() * 1000UL) {
            isisSetDim(isisDimTarget());
        }
    }

    // ---- 20 Hz render tick ----
    if ((uint32_t)(now - g_lastTickMs) >= ISIS_FRAME_INTERVAL_MS) {
        g_lastTickMs = now;

        // ---- Bildstrom hat den Schirm ----
        // Solange CFG:IMG=1 aktiv ist UND Bilder ankommen, zeichnet CC_ISIS
        // nichts: die Pixel kommen bereits von GlassOut. Der Composite-Push
        // wuerde mit dem Stream um denselben RGB-Bus konkurrieren (Flackern).
        // Faellt der Strom aus (500 ms), laeuft alles unveraendert weiter wie
        // vorher - nie ein schwarzer Schirm.
        static bool streamZeigteBild = false;
        if (imgstreamActive(now)) {
            streamZeigteBild = true;
            g_lastLink       = (int)netLinkStatus(now);
            return;
        }
        if (streamZeigteBild) {
            // Rueckfall auf lokales Zeichnen: einmal vollstaendig neu malen,
            // sonst bliebe das letzte Stream-Bild stehen.
            streamZeigteBild      = false;
            isisState.forceRedraw = true;
            g_lastLink            = -1;
        }

        const SmoothSnapshot before = snapshot();
        const bool forced           = isisState.forceRedraw;
        isisState.forceRedraw       = false;

        // Werkstatt-Kommando SHOW: ueberschreibt waehrend des Demo-Flugs die
        // vom FDC kommenden Werte. Muss vor dem Zeichnen laufen.
        demoTick(now);

        // Kalibrier-Testmuster (GRID) hat Vorrang: dann wird das Instrument
        // gar nicht erst gezeichnet. Beides jeden Frame zu malen kostet nur
        // Rechenzeit und liesse das Muster gegen das Instrument flackern.
        const bool kalibrier = gridActive() || alignActive();

        // Unchanged upstream graphics pipeline.
        if (!kalibrier) {
            isis.update();
        }

        const SmoothSnapshot after = snapshot();

        // ---- Statusanzeige ----
        //   kein FDC  : Bild dunkel, nur die Meldung (kein 1013 im Hintergrund)
        //   kein Sim  : Instrument ruhig im Hintergrund, Meldung darueber
        const int link = (int)netLinkStatus(now);

        bool compose = forced || changed(before, after);

        if (kalibrier) {
            // Nur das Muster zeigen - keine Banner, kein DIAG-Badge.
            // WICHTIG: nur bei echter Aenderung neu komponieren. Die Skalierung
            // laeuft ueber pushRotateZoom (Software-Interpolation) und braucht
            // pro 480x480-Bild mehrere hundert Millisekunden - jeden Frame
            // gepusht blockiert das den Hauptloop und das Panel wird taub.
            // Im Align-Modus MUSS das Raster mitgezeichnet werden, auch wenn
            // GRID selbst nicht getoggelt wurde - es ist die Messlatte des
            // Werkzeugs. Vorher haing es an gridActive() und war deshalb weg.
            const bool gridNeu = gridActive() ? gridTakeRedraw() : false;
            const bool alignNeu = alignTakeRedraw();
            const bool neu = gridNeu || alignNeu;
            if (neu) gridDraw();
            compose = neu;
        } else if (link == NET_LINK_FDC_OFFLINE) {
            // Alles aus - ein Panel ohne FDC soll nichts vortaeuschen.
            lcd.fillScreen(TFT_BLACK);
            drawLinkBanner(link);
        } else if (link == NET_LINK_SIM_OFFLINE) {
            drawLinkBanner(link);
        } else if (g_lastLink >= 0 && g_lastLink != link) {
            // Meldung verschwindet -> Instrument vollstaendig neu zeichnen
            isisState.forceRedraw = true;
        }

        if (link != NET_LINK_OK) {
            // Bei stehender Meldung reichen 2 Bilder/s - ein Vollbild-Push
            // kostet bei 480x480 zu viel fuer 20 Hz.
            static uint32_t lastBannerMs = 0;
            if ((uint32_t)(now - lastBannerMs) >= 500) {
                lastBannerMs = now;
                compose = true;
            }
        }
        g_lastLink = link;

        // Diagnose-Kennzeichnung (Panel-Monitor / Live-Diag): rechts neben dem
        // Baro-Wert. Das Badge wird als Sprite ins Kompositions-Canvas gepusht
        // (isis.drawDiagBadge) - Text direkt auf `lcd` zu zeichnen laesst das
        // Panel abstuerzen, `lcd` ist ein LGFX_Sprite mit 8-Bit-Palette.
        static bool lastDiag = false;
        const bool diag = netDiagActive(now);
        if (diag != lastDiag) {
            isisState.forceRedraw = true;
            compose = true;
            lastDiag = diag;
        }
        // isis.update() hat das Canvas gerade neu gezeichnet -> Badge obendrauf.
        if (diag) {
            isis.drawDiagBadge(true);
        }


        // Only touch the panel when the picture really changed.
        if (compose) {
            displaycfgComposite(displaycfgCurrent());
            // Der Composite leert die Raender ausserhalb des Zielrechtecks -
            // dort liegt die Bedienleiste. Ohne dieses Neusetzen verliert sie
            // jeder Frame ihr dirty-Flag und verschwindet bis zum naechsten
            // Touch.
            if (alignActive()) alignForceRedraw();
            alignTick();
            // SAV-Taste: die NVS-Schreibung steckt in alignTakeSaveRequest().
            // Ohne diesen Aufruf wurde die Anfrage nur gesetzt und nie
            // abgeholt - der Knopf tat sichtbar nichts.
            alignTakeSaveRequest();
        }
    }
}
