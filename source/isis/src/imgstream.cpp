//
// imgstream.cpp — Bildstrom-Empfaenger fuer das ISIS-Panel.
//
// Datenweg:  GlassOut (Sim)  ->  FDC infra/panel_stream.py  ->  UDP 5007  ->  hier
//
// Warum eigener Puffer in PSRAM und kein Canvas: `lcd` ist ein
// LGFX_Sprite mit 8-Bit-Palette (displaycanvas.h). Ein JPEG da hinein
// gezeichnet wird auf 256 Farben eingedampft und sieht als Foto wie Matsch
// aus. Deshalb decode -> RGB565-Framebuffer (PSRAM) -> lcdPanel.pushImage().
//
// Takt: JPEGDEC mit MCU-Callback, Zeile fuer Zeile in den Framebuffer.
// 480x480 braucht ~35-60 ms je Bild; das reicht fuer 10-15 fps, mehr gibt
// WLAN + Dekoder auf dem S3 nicht her.
//
#include "imgstream.h"

#include <Arduino.h>
#include <JPEGDEC.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_heap_caps.h>
#include <string.h>

#include "displaycanvas.h"  // lcdPanel
#include "net.h"            // netOtaInProgress()

namespace {

constexpr uint16_t STREAM_PORT      = 5007;
constexpr size_t   HDR_LEN          = 21;     // siehe Spec Abschnitt 4
constexpr size_t   MAX_CHUNKS       = 128;    // 128 * 1200 = 153600 Byte
constexpr uint32_t FRAME_TIMEOUT_MS = 200;    // unvollstaendig -> verwerfen
constexpr uint32_t WATCHDOG_MS      = 500;    // kein Bild -> lokal zeichnen

WiFiUDP g_udp;
bool    g_started   = false;
bool    g_mode      = false;
uint32_t g_lastShow = 0;      // millis() des letzten gezeichneten Bildes
uint32_t g_frames   = 0;
uint32_t g_dropped  = 0;
uint32_t g_badCrc   = 0;
uint32_t g_decodeMs = 0;
uint32_t g_fps      = 0;
uint32_t g_fpsMark  = 0;
uint32_t g_fpsCount = 0;
uint16_t g_lastW    = 0;
uint16_t g_lastH    = 0;

uint16_t *g_fb    = nullptr;   // RGB565, IMGSTREAM_PANEL_W * IMGSTREAM_PANEL_H
uint8_t  *g_jpeg  = nullptr;   // Reassembly-Puffer (JPEG)

// --- Reassembly-Zustand ---------------------------------------------------
bool     g_have      = false;
uint32_t g_frameId   = 0;
uint16_t g_cnt       = 0;
uint16_t g_w         = 0;
uint16_t g_h         = 0;
uint32_t g_crc       = 0;
uint32_t g_len       = 0;      // bereits angekommene Nutzlast-Bytes
uint64_t g_seen[2]   = {0, 0}; // Bitmap der Chunks (bis 128)
uint32_t g_lastChunk = 0;

JPEGDEC g_jpg;

// --- CRC32 (identisch zu zlib.crc32 im FDC) -------------------------------
uint32_t g_crcTab[256];

void crcInit()
{
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        g_crcTab[i] = c;
    }
}

uint32_t crc32Of(const uint8_t *d, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) c = g_crcTab[(c ^ d[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// --- Zeichnen -------------------------------------------------------------
int drawMcu(JPEGDRAW *pDraw)
{
    const int x0 = pDraw->x;
    int y0 = pDraw->y;
    const int w = pDraw->iWidth;
    const int h = pDraw->iHeight;
    const uint16_t *src = pDraw->pPixels;

    for (int y = 0; y < h; ++y) {
        const int dy = y0 + y;
        if (dy < 0 || dy >= IMGSTREAM_PANEL_H) continue;
        int dx = x0;
        if (dx < 0) {
            src -= dx;
            dx = 0;
        }
        const int maxW = IMGSTREAM_PANEL_W - dx;
        if (maxW <= 0) continue;
        const int n = (w < maxW) ? w : maxW;
        memcpy(&g_fb[dy * IMGSTREAM_PANEL_W + dx], src, (size_t)n * 2);
        src += w;
    }
    return 1;
}

void clearAround(int w, int h, int offX, int offY)
{
    if (offY > 0) memset(g_fb, 0, (size_t)offY * IMGSTREAM_PANEL_W * 2);
    for (int y = 0; y < h; ++y) {
        uint16_t *row = &g_fb[(offY + y) * IMGSTREAM_PANEL_W];
        if (offX > 0) memset(row, 0, (size_t)offX * 2);
        const int rechts = IMGSTREAM_PANEL_W - offX - w;
        if (rechts > 0) memset(&row[offX + w], 0, (size_t)rechts * 2);
    }
    const int unten = IMGSTREAM_PANEL_H - offY - h;
    if (unten > 0)
        memset(&g_fb[(offY + h) * IMGSTREAM_PANEL_W], 0, (size_t)unten * IMGSTREAM_PANEL_W * 2);
}

bool decodeAndShow(const uint8_t *jpeg, size_t len)
{
    if (!g_fb) return false;

    const uint32_t t0 = millis();
    if (!g_jpg.openRAM((uint8_t *)jpeg, (int)len, drawMcu)) return false;
    g_jpg.setPixelType(RGB565_LITTLE_ENDIAN);

    int nw = g_jpg.getWidth();
    int nh = g_jpg.getHeight();
    if (nw <= 0 || nh <= 0) {
        g_jpg.close();
        return false;
    }

    // Der FDC liefert bereits die Panelgroesse. Ist das Quellbild groesser,
    // dekodieren wir eine Stufe kleiner - ein 768er JPEG auf 480 zu quetschen
    // kostet sonst Rechenzeit ohne Gewinn.
    uint8_t opts = 0;
    if ((nw > IMGSTREAM_PANEL_W || nh > IMGSTREAM_PANEL_H) &&
        (nw >= IMGSTREAM_PANEL_W * 2 || nh >= IMGSTREAM_PANEL_H * 2)) {
        opts = JPEG_SCALE_HALF;
    }

    const int offX = 0;
    const int offY = 0;
    if (g_lastW != (uint16_t)nw || g_lastH != (uint16_t)nh) {
        clearAround(nw >> (opts == JPEG_SCALE_HALF ? 1 : 0),
                    nh >> (opts == JPEG_SCALE_HALF ? 1 : 0), offX, offY);
        g_lastW = (uint16_t)nw;
        g_lastH = (uint16_t)nh;
    }

    const int ok = g_jpg.decode(offX, offY, opts);
    g_jpg.close();
    if (ok != 1) return false;

    lcdPanel.pushImage(0, 0, IMGSTREAM_PANEL_W, IMGSTREAM_PANEL_H, g_fb);

    g_decodeMs = millis() - t0;
    g_lastShow = millis();
    g_frames++;
    g_fpsCount++;
    return true;
}

void resetFrame()
{
    g_have = false;
    g_len  = 0;
    g_seen[0] = g_seen[1] = 0;
}

bool frameComplete()
{
    const uint16_t need = g_cnt;
    for (uint16_t i = 0; i < need; ++i) {
        const uint64_t bit = 1ULL << (i & 63);
        if (!(g_seen[i >> 6] & bit)) return false;
    }
    return true;
}

void handleChunk(const uint8_t *d, int n, uint32_t nowMs)
{
    if (n < (int)HDR_LEN || memcmp(d, "IMG1", 4) != 0) return;

    const uint32_t frame = rd32(d + 4);
    const uint16_t idx   = rd16(d + 8);
    const uint16_t cnt   = rd16(d + 10);
    const uint16_t w     = rd16(d + 12);
    const uint16_t h     = rd16(d + 14);
    const uint8_t  flags = d[16];
    const uint32_t crc   = rd32(d + 17);
    const size_t   plen  = (size_t)(n - (int)HDR_LEN);

    if (cnt == 0 || cnt > MAX_CHUNKS || idx >= cnt) return;
    if (!(flags & 0x01)) return;  // nur JPEG

    const size_t off = (size_t)idx * IMGSTREAM_CHUNK_PAYLOAD;
    if (off + plen > IMGSTREAM_MAX_JPEG) return;

    // Neuer Frame oder Luecke erkannt -> alten Rest verwerfen.
    if (!g_have || frame != g_frameId) {
        if (g_have) g_dropped++;
        resetFrame();
        g_have    = true;
        g_frameId = frame;
        g_cnt     = cnt;
        g_w       = w;
        g_h       = h;
        g_crc     = crc;
    }

    memcpy(&g_jpeg[off], d + HDR_LEN, plen);
    const uint64_t bit = 1ULL << (idx & 63);
    if (!(g_seen[idx >> 6] & bit)) {
        g_seen[idx >> 6] |= bit;
        g_len += (uint32_t)plen;
    }
    g_lastChunk = nowMs;

    if (!frameComplete()) return;

    // Gesamtlaenge ist die Summe der angekommenen Nutzlasten (die letzte
    // Chunk-Laenge ist kuerzer als IMGSTREAM_CHUNK_PAYLOAD, deshalb nicht
    // idx*1200+plen).
    const size_t total = g_len;

    const uint32_t crcIst = crc32Of(g_jpeg, total);
    if (crcIst != g_crc) {
        g_badCrc++;
        g_dropped++;
        resetFrame();
        return;
    }

    if (!decodeAndShow(g_jpeg, total)) {
        g_dropped++;
    }
    resetFrame();
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool imgstreamBegin()
{
    if (g_started) return true;

    crcInit();

    g_fb = (uint16_t *)heap_caps_malloc((size_t)IMGSTREAM_PANEL_W * IMGSTREAM_PANEL_H * 2,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_jpeg = (uint8_t *)heap_caps_malloc(IMGSTREAM_MAX_JPEG, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_fb || !g_jpeg) {
        Serial.println("[IMG] PSRAM-Puffer fehlen - Bildstrom aus");
        return false;
    }
    memset(g_fb, 0, (size_t)IMGSTREAM_PANEL_W * IMGSTREAM_PANEL_H * 2);

    if (!g_udp.begin(STREAM_PORT)) {
        Serial.printf("[IMG] UDP %u belegt - Bildstrom aus\n", (unsigned)STREAM_PORT);
        return false;
    }
    g_started = true;
    Serial.printf("[IMG] Bildstrom bereit: UDP %u, %.0f KB Framebuffer in PSRAM\n",
                  (unsigned)STREAM_PORT,
                  (double)((size_t)IMGSTREAM_PANEL_W * IMGSTREAM_PANEL_H * 2) / 1024.0);
    return true;
}

void imgstreamService(uint32_t nowMs)
{
    if (!g_started) return;

    // Waehrend eines OTA-Uploads nicht dekodieren und nicht pushen: der
    // Flash-Schreibpfad vertraegt keine zusaetzliche Arbeit im Hauptloop
    // (siehe a320-firmware, ISIS/OTA-Regeln). Der Bildstrom pausiert einfach
    // und laeuft nach dem Update weiter.
    if (netOtaInProgress()) return;

    // Unvollstaendige Frames verfallen - das naechste Bild kommt in ~50 ms.
    if (g_have && (uint32_t)(nowMs - g_lastChunk) > FRAME_TIMEOUT_MS) {
        g_dropped++;
        resetFrame();
    }

    int size = g_udp.parsePacket();
    while (size > 0) {
        uint8_t buf[HDR_LEN + IMGSTREAM_CHUNK_PAYLOAD];
        const int n = g_udp.read(buf, sizeof(buf));
        if (n > 0) handleChunk(buf, n, nowMs);
        size = g_udp.parsePacket();
    }

    // fps einmal pro Sekunde ausrechnen
    if ((uint32_t)(nowMs - g_fpsMark) >= 1000) {
        g_fps     = g_fpsCount;
        g_fpsCount = 0;
        g_fpsMark = nowMs;
        if (g_mode && g_fps) {
            Serial.printf("[IMG] %u fps, decode %u ms, %ux%u, verworfen %u (crc %u)\n",
                          (unsigned)g_fps, (unsigned)g_decodeMs, (unsigned)g_lastW,
                          (unsigned)g_lastH, (unsigned)g_dropped, (unsigned)g_badCrc);
        }
    }
}

bool imgstreamActive(uint32_t nowMs)
{
    if (!g_mode || !g_started) return false;
    return (uint32_t)(nowMs - g_lastShow) <= WATCHDOG_MS;
}

void imgstreamSetMode(bool on)
{
    if (on == g_mode) return;
    g_mode = on;
    resetFrame();
    g_lastShow = 0;  // beim Einschalten erst wieder auf ein echtes Bild warten
    Serial.printf("[IMG] Modus %s\n", on ? "Bildstrom" : "lokales Zeichnen");
}

bool imgstreamMode() { return g_mode; }

ImgStreamStats imgstreamStats()
{
    ImgStreamStats s;
    s.frames   = g_frames;
    s.dropped  = g_dropped;
    s.badCrc   = g_badCrc;
    s.fps      = g_fps;
    s.decodeMs = g_decodeMs;
    s.lastW    = g_lastW;
    s.lastH    = g_lastH;
    return s;
}
