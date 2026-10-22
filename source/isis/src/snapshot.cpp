#include "snapshot.h"

#include <Arduino.h>
#include <WiFi.h>

#include "displaycanvas.h"
#include "net.h"          // netSendChunk: Versand ueber den Uplink-Socket

// 1 KB je Paket: klein genug fuer jedes WLAN, gross genug fuer ~230 Pakete.
#define SHOT_NUTZ 1024
#define SHOT_MAGIC "ISIS"

void isisSendSnapshot(uint32_t ip, uint16_t port, uint16_t maxPakete, uint16_t pauseMs,
                      uint16_t chunkBytes)
{
    const int w = lcd.width();
    const int h = lcd.height();
    if (w <= 0 || h <= 0) {
        Serial.println("[shot] kein Zeichenpuffer - Abbruch");
        return;
    }

    if (chunkBytes < 64 || chunkBytes > SHOT_NUTZ) chunkBytes = SHOT_NUTZ;
    if (chunkBytes & 1) ++chunkBytes;                 // volle Pixel, sonst Matsch
    const uint32_t bytesGesamt = (uint32_t)w * (uint32_t)h * 2u;
    const uint16_t kanaele = (uint16_t)((bytesGesamt + chunkBytes - 1) / chunkBytes);

    Serial.printf("[shot] %dx%d, %u Bytes, %u Pakete -> %u.%u.%u.%u:%u\n",
                  w, h, (unsigned)bytesGesamt, (unsigned)kanaele,
                  (unsigned)((ip >> 24) & 0xFF), (unsigned)((ip >> 16) & 0xFF),
                  (unsigned)((ip >> 8) & 0xFF), (unsigned)(ip & 0xFF), (unsigned)port);

    const uint32_t start = millis();
    uint8_t puffer[SHOT_NUTZ];
    uint16_t pos = 0;
    uint16_t kanal = 0;

    uint16_t okPakete = 0, fehler = 0;
    auto sendKanal = [&]() {
        uint8_t kopf[8];
        kopf[0] = 'I'; kopf[1] = 'S'; kopf[2] = 'I'; kopf[3] = 'S';
        kopf[4] = (uint8_t)(kanal & 0xFF);
        kopf[5] = (uint8_t)(kanal >> 8);
        kopf[6] = (uint8_t)(kanaele & 0xFF);
        kopf[7] = (uint8_t)(kanaele >> 8);
        if (netSendChunk(ip, port, kopf, sizeof(kopf), puffer, pos)) ++okPakete;
        else ++fehler;
        ++kanal;
        pos = 0;
    };

    bool fertig = false;
    for (int y = 0; y < h && !fertig; ++y) {
        for (int x = 0; x < w && !fertig; ++x) {
            const uint16_t px = lcd.readPixel(x, y);   // RGB565
            puffer[pos++] = (uint8_t)(px & 0xFF);
            puffer[pos++] = (uint8_t)(px >> 8);
            if (pos >= chunkBytes) {
                sendKanal();
                // Taktung: der Schwall darf nicht als UDP-Flood durchgehen.
                if (pauseMs) delay(pauseMs);
                if (maxPakete && kanal >= maxPakete) { fertig = true; break; }
            }
        }
        if ((y & 63) == 0) delay(1);   // WLAN-Luft holen, Watchdog ruhig halten
    }
    if (pos > 0) sendKanal();

    Serial.printf("[shot] fertig: %u Pakete (%u ok, %u Fehler) in %lu ms\n",
                  (unsigned)kanal, (unsigned)okPakete, (unsigned)fehler,
                  (unsigned long)(millis() - start));
}

// ---------------------------------------------------------------------------
// Serielle Variante: Base64-Zeilen ueber die Konsole.
// ---------------------------------------------------------------------------
static void b64Block(const uint8_t *daten, size_t len, char *ziel)
{
    static const char *tab =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t i = 0, o = 0;
    while (i + 2 < len) {
        const uint32_t v = ((uint32_t)daten[i] << 16) | ((uint32_t)daten[i + 1] << 8) | daten[i + 2];
        ziel[o++] = tab[(v >> 18) & 0x3F];
        ziel[o++] = tab[(v >> 12) & 0x3F];
        ziel[o++] = tab[(v >> 6) & 0x3F];
        ziel[o++] = tab[v & 0x3F];
        i += 3;
    }
    if (i < len) {
        uint32_t v = (uint32_t)daten[i] << 16;
        if (i + 1 < len) v |= (uint32_t)daten[i + 1] << 8;
        ziel[o++] = tab[(v >> 18) & 0x3F];
        ziel[o++] = tab[(v >> 12) & 0x3F];
        ziel[o++] = (i + 1 < len) ? tab[(v >> 6) & 0x3F] : '=';
        ziel[o++] = '=';
    }
    ziel[o] = 0;
}

void isisSendSnapshotSerial(uint8_t faktor)
{
    if (faktor < 1) faktor = 1;
    const int w = lcd.width();
    const int h = lcd.height();
    if (w <= 0 || h <= 0) {
        Serial.println("[shot] kein Zeichenpuffer");
        return;
    }
    const int bw = w / faktor;
    const int bh = h / faktor;
    const size_t nutz = 384;                       // 384 Byte -> 512 Base64-Zeichen
    uint8_t puffer[384];
    char    b64[520];
    size_t  pos = 0;
    uint16_t nr = 0;
    const uint32_t start = millis();

    Serial.printf("[shot] Konsole: %dx%d (Faktor %u), %u Zeilen\n",
                  bw, bh, (unsigned)faktor, (unsigned)((bw * bh * 2 + nutz - 1) / nutz));

    for (int y = 0; y < bh; ++y) {
        for (int x = 0; x < bw; ++x) {
            const uint16_t px = lcd.readPixel(x * faktor, y * faktor);
            puffer[pos++] = (uint8_t)(px & 0xFF);
            puffer[pos++] = (uint8_t)(px >> 8);
            if (pos >= nutz) {
                b64Block(puffer, pos, b64);
                Serial.printf("SHOTB64|%u|%s\n", (unsigned)nr++, b64);
                pos = 0;
            }
        }
        if ((y & 15) == 0) delay(1);
    }
    if (pos > 0) {
        b64Block(puffer, pos, b64);
        Serial.printf("SHOTB64|%u|%s\n", (unsigned)nr++, b64);
    }
    Serial.printf("SHOTEND|%u\n", (unsigned)nr);
    Serial.printf("[shot] fertig in %lu ms\n", (unsigned long)(millis() - start));
}
