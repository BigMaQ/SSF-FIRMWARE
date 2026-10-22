// render_main.cpp — Desktop-Renderer fuer das A320-ISIS (CC_ISIS, MIT, Chris Crawford).
//
// Fuehrt den UNVERAENDERTEN Upstream-Zeichencode (CC_ISIS.cpp + ISISCommon.cpp +
// Sprites + Fonts) auf einem reinen Speicher-Framebuffer aus und schreibt das
// Ergebnis als PPM (P6). Es wird nichts nachgezeichnet, nichts ersetzt.
//
// Datenweg exakt wie auf der Hardware:
//   CC_ISIS::set(messageID, "wert")   <-- dieselben Message-IDs wie am ESP32
//   CC_ISIS::update()                 <-- updateInputValues() (Glaettung) + draw()
//   Panel-Framebuffer                 <-- 480x480 RGB565
//
// Aufruf:
//   isis_render --out out/01.ppm [--frames 400] [--msg 60=280] [--msg 101=1] ...
//               [--rotate 90] [--out-raw raw/01.ppm] [--brightness-menu 65] [--quiet]
//   isis_render --selftest
//
// Ausgabe auf stdout: eine JSON-Zeile mit dem tatsaechlich gezeichneten Zustand
// (isisState), Bildgeometrie und FNV-1a-Hash des Framebuffers.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "CC_ISIS.h"          // Upstream (Kopie in build/src) -> ISISCommon.h -> Desktop-Display-Config
#include "Panel_Memory.hpp"

// ---------------------------------------------------------------------------

// Beweist die Byte-Reihenfolge des Panel-Framebuffers, indem bekannte Farben
// ueber den normalen LGFX-Weg gefuellt und die Rohbytes geprueft werden.
// Ohne diesen Nachweis waere jede Farbaussage des Renderers wertlos.
static int selftest()
{
    struct { const char *name; uint32_t color; } cases[] = {
        {"TFT_RED",   0xF800},
        {"TFT_GREEN", 0x07E0},
        {"TFT_BLUE",  0x001F},
        {"TFT_WHITE", 0xFFFF},
    };
    CC_ISIS cc;
    cc.begin();
    auto &panel = lcd._panel_instance;
    if (!panel.fb_data()) { fprintf(stderr, "kein Framebuffer\n"); return 3; }

    bool big_endian_ok = true;
    printf("Byte-Order-Nachweis (Panel-Framebuffer):\n");
    for (auto &c : cases)
    {
        lcd.fillScreen((uint16_t)c.color);
        uint8_t b[2] = {0, 0};
        panel.pixel_bytes(10, 10, b);
        const uint8_t exp_hi = (uint8_t)(c.color >> 8);
        const uint8_t exp_lo = (uint8_t)(c.color & 0xFF);
        const bool be = (b[0] == exp_hi && b[1] == exp_lo);
        const bool le = (b[0] == exp_lo && b[1] == exp_hi);
        if (!be) big_endian_ok = false;
        printf("  %-10s 0x%04X -> Speicher %02X %02X  (%s)\n", c.name, c.color, b[0], b[1],
               be ? "BIG-ENDIAN: hi,lo" : (le ? "little-endian: lo,hi" : "unbekannt!"));
    }
    printf("Ergebnis: Framebuffer ist %s.\n",
           big_endian_ok ? "BIG-ENDIAN RGB565 (so liest der RGB-Bus)" : "NICHT big-endian");
    if (!big_endian_ok)
    {
        fprintf(stderr, "WARNUNG: Die Pixel-Interpretation in Panel_Memory::pixel_rgb565 passt nicht "
                        "zur hier gemessenen Reihenfolge — Farben waeren vertauscht.\n");
        return 5;
    }
    return 0;
}

static uint64_t fnv1a64(const uint8_t *p, size_t n)
{
    uint64_t h = 1469598103934665603ULL;
    for (size_t i = 0; i < n; ++i)
    {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

static void json_escape(std::string &out, const std::string &s)
{
    for (char c : s)
    {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c == '\n')        { out += "\\n"; }
        else                       { out += c; }
    }
}

// Overlay des Brightness-Menues. Wird vom Upstream-Hauptloop genauso auf einen
// Sprite komponiert und dann transparent gepusht (BrightnessMenu::draw() ist
// Upstream-Code; nur die Komposition drumherum stellt der Renderer).
static void draw_brightness_menu_overlay(int percent)
{
    isisState.lcdBrightness = percent;
    brightnessMenu.show();

    LGFX_Sprite overlay(&lcd);
    overlay.setColorDepth(16);
    overlay.createSprite(lcd.width(), lcd.height());
    overlay.fillSprite(TFT_MAIN_TRANSPARENT);
    brightnessMenu.draw(&overlay);
    overlay.pushSprite(0, 0, TFT_MAIN_TRANSPARENT);
    overlay.deleteSprite();
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s --out <datei.ppm> [--frames N] [--msg ID=WERT]... [--rotate GRAD]\n"
            "         [--out-raw <datei.ppm>] [--brightness-menu PCT] [--quiet]\n"
            "       %s --selftest\n"
            "\n"
            "  --out <datei.ppm>       Ziel (PPM P6, 480x480) in Ansicht wie auf dem Display\n"
            "  --rotate GRAD           Drehung der Ansicht: 90 (Default, aufrecht), 0, 180, 270\n"
            "  --out-raw <datei.ppm>   zusaetzlich der rohe Framebuffer (Rotation 0)\n"
            "  --frames N              Anzahl update()-Zyklen (Glaettung einschwingen), Default 400\n"
            "  --msg ID=WERT           Upstream-Message-ID setzen, mehrfach moeglich\n"
            "                          (60 Airspeed kt, 71 Slip/Skid, 72 Bank, 77 Altitude ft,\n"
            "                           80 Pitch, 100 QNH mb, 101 STD-Modus, 102 Mach,\n"
            "                           2/3/5/6/10 CDI+GS+NAV-Quelle)\n"
            "  --brightness-menu PCT   Brightness-Overlay (0-100) auf das Bild legen\n"
            "  --selftest              Byte-Reihenfolge des Framebuffers pruefen\n"
            "  --quiet                 keine JSON-Zusammenfassung auf stdout\n",
            argv0, argv0);
}

int main(int argc, char **argv)
{
    std::string out, out_raw;
    std::vector<std::pair<int, std::string>> msgs;
    int frames = 400;
    int menu_percent = -1;
    int rotate = 90;
    bool quiet = false;

    // --selftest vorab (braucht keine weiteren Argumente)
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--selftest") return selftest();
    }

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        auto next = [&](const char *what) -> const char * {
            if (i + 1 >= argc) { fprintf(stderr, "Fehler: %s braucht einen Wert\n", what); exit(2); }
            return argv[++i];
        };
        if (a == "--out")                     { out = next("--out"); }
        else if (a == "--out-raw")            { out_raw = next("--out-raw"); }
        else if (a == "--rotate")             { rotate = atoi(next("--rotate")); }
        else if (a == "--frames")             { frames = atoi(next("--frames")); }
        else if (a == "--quiet")              { quiet = true; }
        else if (a == "--brightness-menu")    { menu_percent = atoi(next("--brightness-menu")); }
        else if (a == "--selftest")           { /* oben erledigt */ }
        else if (a == "--msg")
        {
            const std::string kv = next("--msg");
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) { fprintf(stderr, "Fehler: --msg erwartet ID=WERT (war '%s')\n", kv.c_str()); return 2; }
            msgs.emplace_back(atoi(kv.substr(0, eq).c_str()), kv.substr(eq + 1));
        }
        else if (a == "--help" || a == "-h")  { usage(argv[0]); return 0; }
        else { fprintf(stderr, "Fehler: unbekanntes Argument '%s'\n", a.c_str()); usage(argv[0]); return 2; }
    }
    if (out.empty() || frames < 1) { usage(argv[0]); return 2; }

    // --- Upstream-Device genauso starten wie die Firmware -------------------
    CC_ISIS cc;
    cc.begin();          // lcd.init(), setupSprites(), setRotation(3), fillScreen(black)

    if (!lcd.getPanel() || lcd.width() != 480 || lcd.height() != 480)
    {
        fprintf(stderr, "Fehler: Panel nicht initialisiert (%dx%d)\n", (int)lcd.width(), (int)lcd.height());
        return 3;
    }

    // --- Werte einspeisen (identische Message-IDs wie die echte Firmware) ---
    for (auto &m : msgs)
    {
        std::vector<char> buf(m.second.begin(), m.second.end());
        buf.push_back('\0');
        cc.set((int16_t)m.first, buf.data());
    }

    // --- Einschwingen + Zeichnen -------------------------------------------
    for (int i = 0; i < frames; ++i) cc.update();

    if (menu_percent >= 0) draw_brightness_menu_overlay(menu_percent < 1 ? 1 : (menu_percent > 100 ? 100 : menu_percent));

    auto &panel = lcd._panel_instance;
    if (!panel.fb_data()) { fprintf(stderr, "Fehler: kein Framebuffer\n"); return 3; }
    if (!panel.write_ppm(rotate, out.c_str()))
    {
        fprintf(stderr, "Fehler: '%s' konnte nicht geschrieben werden\n", out.c_str());
        return 4;
    }
    if (!out_raw.empty() && !panel.write_ppm(0, out_raw.c_str()))
    {
        fprintf(stderr, "Fehler: '%s' konnte nicht geschrieben werden\n", out_raw.c_str());
        return 4;
    }

    if (!quiet)
    {
        const size_t bytes = (size_t)panel.fb_stride() * (size_t)panel.fb_height();
        std::string o;
        char tmp[512];
        snprintf(tmp, sizeof(tmp),
                 "{\"out\":\"%s\",\"rotate\":%d,\"frames\":%d,\"width\":%d,\"height\":%d,\"stride\":%d,"
                 "\"fnv1a\":\"%016llx\",\"msgs\":[",
                 out.c_str(), rotate, frames, panel.fb_width(), panel.fb_height(), panel.fb_stride(),
                 (unsigned long long)fnv1a64(panel.fb_data(), bytes));
        o += tmp;
        for (size_t i = 0; i < msgs.size(); ++i)
        {
            o += (i ? "," : "");
            o += "[" + std::to_string(msgs[i].first) + ",\"";
            json_escape(o, msgs[i].second);
            o += "\"]";
        }
        const float parr[8] = {isisState.pitchAngle, isisState.bankAngle, isisState.airspeed,
                               isisState.altitude, (float)isisState.mbPressure,
                               (float)isisState.isStdPressure, isisState.machSpeed, isisState.ballPos};
        const float rarr[4] = {isisState.rawPitchAngle, isisState.rawBankAngle,
                               isisState.rawAirspeed, isisState.rawAltitude};
        snprintf(tmp, sizeof(tmp),
                 "],\"state\":{\"pitch\":%.3f,\"bank\":%.3f,\"airspeed\":%.3f,\"altitude\":%.3f,"
                 "\"qnh_mb\":%d,\"std\":%d,\"mach\":%.3f,\"slip\":%.3f,\"navSource\":%d,"
                 "\"cdiValid\":%d,\"gsiValid\":%d,\"brightness\":%d},"
                 "\"raw\":{\"pitch\":%.3f,\"bank\":%.3f,\"airspeed\":%.3f,\"altitude\":%.3f},"
                 "\"converged\":%s}\n",
                 parr[0], parr[1], parr[2], parr[3], isisState.mbPressure, isisState.isStdPressure,
                 parr[6], parr[7], isisState.navSource, isisState.cdiNeedleValid, isisState.gsiNeedleValid,
                 isisState.lcdBrightness, rarr[0], rarr[1], rarr[2], rarr[3],
                 (parr[0] == rarr[0] && parr[1] == rarr[1] && parr[2] == rarr[2] && parr[3] == rarr[3])
                     ? "true" : "false");
        o += tmp;
        fputs(o.c_str(), stdout);
    }
    return 0;
}
