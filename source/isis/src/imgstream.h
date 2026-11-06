#pragma once
//
// imgstream.h — Bildstrom-Empfaenger (GlassOut -> FDC -> Panel).
//
// Gegenstueck zu infra/panel_stream.py im FDC. Wire-Format:
// docs/STREAM_TRANSPORT_SPEC.md, Abschnitt 4 — UDP 5007, Chunks mit
// "IMG1"-Kopf, JPEG-Nutzlast, CRC32 ueber das ganze Bild.
//
// Steuerung kommt ueber den bestehenden Textkanal (UDP 5005):
//   CFG:IMG=1;  -> Bildstrom-Modus, Firmware zeigt die empfangenen Frames
//   CFG:IMG=0;  -> lokales Zeichnen (CC_ISIS), Stream wird ignoriert
//
// Faellt der Strom aus (500 ms kein vollstaendiges Bild), schaltet
// imgstreamActive() auf false und main.cpp zeichnet wieder selbst. Ein
// schwarzer Schirm darf nie das Ergebnis sein.
//
#include <stdint.h>

// Panel-Aufloesung, an die gezeichnet wird (Guition ESP32-4848S040).
#define IMGSTREAM_PANEL_W 480
#define IMGSTREAM_PANEL_H 480

// Nutzlast je Chunk - muss zum Sender passen (panel_stream.py: CHUNK_PAYLOAD).
#define IMGSTREAM_CHUNK_PAYLOAD 1200

// Groesster akzeptierter JPEG-Frame (Chunks * 1200). 96 KB reichen fuer
// 480x480 bei Q95 mit Reserve.
#define IMGSTREAM_MAX_JPEG (96 * 1024)

struct ImgStreamStats {
    uint32_t frames;      // vollstaendig empfangene und gezeichnete Bilder
    uint32_t dropped;     // unvollstaendig verworfene Bilder (Timeout/CRC)
    uint32_t badCrc;      // CRC-Fehler
    uint32_t fps;         // Bilder in der letzten Sekunde
    uint32_t decodeMs;    // Dauer des letzten Dekodierlaufs
    uint32_t lastW;       // Groesse des zuletzt gezeichneten Bildes
    uint32_t lastH;
};

// Socket oeffnen und Puffer in PSRAM anlegen. Nach netBegin() aufrufen.
bool imgstreamBegin();

// Im Hauptloop zyklisch aufrufen: Pakete abholen, reassemblieren, zeichnen.
void imgstreamService(uint32_t nowMs);

// true, wenn der Stream gerade das Bild stellt (Modus an UND Bild frisch).
bool imgstreamActive(uint32_t nowMs);

void imgstreamSetMode(bool on);
bool imgstreamMode();

// Statistik fuer Konsole/Diagnose.
ImgStreamStats imgstreamStats();
