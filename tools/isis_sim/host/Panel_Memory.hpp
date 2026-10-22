// Panel_Memory — Offscreen-Framebuffer fuer den Desktop-Renderer.
//
// Zweck: das Verhalten des Guition-Panels (ESP32-S3 + ST7701 am RGB-Parallelbus)
// auf dem PC nachbilden, ohne Display und ohne SDL.
//
// Warum das geht: das echte Panel (`lgfx::Panel_ST7701_guition_esp32_4848S040`,
// Basis `Panel_RGB`) erbt von `Panel_FrameBufferBase` und schreibt alle Pixel in
// den RGB-Framebuffer im Speicher (PSRAM). Genau diese Basisklasse wird hier
// wiederverwendet — nur die Speicherzuordnung macht diese Klasse selbst. Die
// Zeichenlogik (CC_ISIS.cpp + Sprites + Fonts) laeuft damit bitgenau denselben
// Weg wie auf der Hardware: gleiche Pixel-Pipeline, gleiche Farbtiefe, gleiche
// Sprite-/Transparenz-Komposition.
//
// Drei Hardware-Eigenschaften werden bewusst nachgebildet:
//
//  1) Farbtiefe ist fest RGB565 (2 Byte/Pixel). `setColorDepth()` wird ignoriert
//     — genau wie `Panel_RGB::setColorDepth()`, das nur `_write_depth` zurueckgibt.
//     Der Aufruf `lcd.setColorDepth(8)` in CC_ISIS::begin() ist auf der Hardware
//     ebenfalls wirkungslos.
//
//  2) BYTE-REIHENFOLGE: `_write_depth` ist `rgb565_2Byte`, also die *getauschte*
//     Variante. Der Farbwert im Speicher steht dadurch BIG-ENDIAN — genau das,
//     was der RGB-Parallelbus MSB-first ausliest. Der Renderer liest die Pixel
//     deshalb als (hi<<8)|lo. Nachweis: `isis_render --selftest` fuellt das
//     Panel mit TFT_GREEN (0x07E0) und prueft, dass die Speicherbytes 0x07,0xE0
//     lauten. Unabhaengig bestaetigt durch die Assets: die generierten
//     Sprite-Header sind laut Kopfzeile "byte-swapped RGB565", und der
//     Transparent-Schluessel des Upstream (8184 = 0x1FF8) ist das bytegetauschte
//     TFT_MAGENTA (0xF81F).
//
//  3) `setRotation(3)` (Upstream: "USB-Buchse nach unten") wirkt nur auf die
//     Adressierung. Der Speicher enthaelt deshalb das Bild um 90 Grad gedreht
//     (Zeichenrahmen (x,y) -> Speicher (y, 479-x)). Fuer die Ansicht "wie auf der
//     Glasscheibe" dreht write_ppm() um 90 Grad zurueck; `--rotate 0` liefert den
//     rohen Framebuffer fuer Pixelvergleiche.
#pragma once

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <LovyanGFX.hpp>
#include <lgfx/v1/Bus.hpp>
#include <lgfx/v1/panel/Panel_FrameBufferBase.hpp>

namespace isis_sim
{

class Panel_Memory : public lgfx::v1::Panel_FrameBufferBase
{
public:
    Panel_Memory()
    {
        // identisch zu Panel_RGB::Panel_RGB()
        _write_depth = lgfx::v1::color_depth_t::rgb565_2Byte;
        _read_depth  = lgfx::v1::color_depth_t::rgb565_2Byte;
    }

    ~Panel_Memory() override { free_buffer(); }

    // Wie die echte RGB-Hardware: Farbtiefe ist verdrahtet, kein Umschalten.
    lgfx::v1::color_depth_t setColorDepth(lgfx::v1::color_depth_t) override { return _write_depth; }

    bool init(bool use_reset) override
    {
        // setzt Rotation/Invert; der Bus ist Bus_NULL und wird nie angefasst
        if (!Panel_FrameBufferBase::init(use_reset)) return false;
        if (!_fb)
        {
            const int w = _cfg.memory_width;
            const int h = _cfg.memory_height;
            _stride = w * (int)sizeof(uint16_t);
            const size_t bytes = (size_t)_stride * (size_t)h;
            _fb = (uint8_t *)lgfx::heap_alloc(bytes);
            if (!_fb) return false;
            memset(_fb, 0, bytes);
            _rows = (uint8_t **)lgfx::heap_alloc(sizeof(uint8_t *) * (size_t)h);
            if (!_rows)
            {
                lgfx::heap_free(_fb);
                _fb = nullptr;
                return false;
            }
            for (int y = 0; y < h; ++y) _rows[y] = _fb + (size_t)y * (size_t)_stride;
            _lines_buffer = _rows;
        }
        return true;
    }

    // --- Zugriff fuer den Renderer -----------------------------------------
    const uint8_t *fb_data() const { return _fb; }
    int            fb_stride() const { return _stride; }
    int            fb_width() const { return _cfg.memory_width; }
    int            fb_height() const { return _cfg.memory_height; }

    // Rohbytes eines Pixels in Speicherreihenfolge (zur Byte-Order-Pruefung)
    bool pixel_bytes(int x, int y, uint8_t out[2]) const
    {
        if (!_fb || x < 0 || y < 0 || x >= fb_width() || y >= fb_height()) return false;
        const uint8_t *p = _fb + (size_t)y * (size_t)_stride + (size_t)x * 2;
        out[0] = p[0];
        out[1] = p[1];
        return true;
    }

    // RGB565 in Bus-Reihenfolge (Big-Endian, s. Kopfkommentar Punkt 2)
    uint16_t pixel_rgb565(int x, int y) const
    {
        uint8_t b[2] = {0, 0};
        pixel_bytes(x, y, b);
        return (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    }

    // 1:1 der Speicherinhalt des Panels — das, was der RGB-Bus ausliest.
    bool write_ppm(int rotation, const char *path) const
    {
        if (!_fb) return false;
        const int w = fb_width();
        const int h = fb_height();
        rotation = ((rotation % 360) + 360) % 360;

        int ow = w, oh = h;
        if (rotation == 90 || rotation == 270) { ow = h; oh = w; }

        FILE *f = fopen(path, "wb");
        if (!f) return false;
        fprintf(f, "P6\n%d %d\n255\n", ow, oh);

        std::vector<uint8_t> row((size_t)ow * 3);
        for (int y = 0; y < oh; ++y)
        {
            for (int x = 0; x < ow; ++x)
            {
                int sx, sy;
                // Urbild des Ausgabepixels in Speicherkoordinaten
                switch (rotation)
                {
                case 90:  sx = y;             sy = h - 1 - x; break; // im Uhrzeigersinn
                case 180: sx = w - 1 - x;     sy = h - 1 - y; break;
                case 270: sx = w - 1 - y;     sy = x;         break;
                default:  sx = x;             sy = y;         break;
                }
                const uint16_t c  = pixel_rgb565(sx, sy);
                const uint8_t  r5 = (uint8_t)((c >> 11) & 0x1F);
                const uint8_t  g6 = (uint8_t)((c >> 5) & 0x3F);
                const uint8_t  b5 = (uint8_t)(c & 0x1F);
                row[(size_t)x * 3 + 0] = (uint8_t)((r5 << 3) | (r5 >> 2));
                row[(size_t)x * 3 + 1] = (uint8_t)((g6 << 2) | (g6 >> 4));
                row[(size_t)x * 3 + 2] = (uint8_t)((b5 << 3) | (b5 >> 2));
            }
            if (fwrite(row.data(), 1, row.size(), f) != row.size())
            {
                fclose(f);
                return false;
            }
        }
        fclose(f);
        return true;
    }

private:
    void free_buffer()
    {
        if (_rows) { lgfx::heap_free(_rows); _rows = nullptr; }
        if (_fb)   { lgfx::heap_free(_fb);   _fb = nullptr; }
        _lines_buffer = nullptr;
    }

    uint8_t  *_fb = nullptr;
    uint8_t **_rows = nullptr;
    int       _stride = 0;
};

} // namespace isis_sim
