#pragma once
//
// snapshot.h — Bildschirmkopie ueber das Netz.
//
// Warum: die Anzeige laesst sich nur am Geraet beurteilen. Wer nicht davor
// steht (oder abends im Chat sitzt), braucht einen Weg, das Bild zu sehen -
// fuer Absprachen, Dokumentation und Fehlersuche an der Grafik.
//
// Der Zeichenpuffer ist ein Sprite (`lcd` in displaycanvas.h), also auslesbar.
// Uebertragung als UDP-Strom, weil das Panel im Normalbetrieb keinen
// HTTP-Server laufen hat (der existiert nur im Einrichtungsmodus).
//
// Paketaufbau (alles Little Endian):
//   Bytes 0..3  "ISIS"
//   Bytes 4..5  Kanalnummer (0-basiert)
//   Bytes 6..7  Anzahl Kanaele
//   Bytes 8..   Nutzdaten, RGB565, zeilenweise von oben links
//
#include <cstdint>

// Schickt die aktuelle Anzeige an ip:port. Blockiert einige Sekunden
// (Pixel aus dem PSRAM lesen), danach laeuft die Anzeige normal weiter.
// maxPakete = 0 : alles senden. Sonst nur die ersten maxPakete (Diagnose).
// pauseMs    = Wartezeit zwischen zwei Paketen. Ein Schwall von 450 Paketen in
//              750 ms wird von der Netzwerksicherheit (UDM-IPS) als UDP-Flood
//              verworfen - mit kleiner Taktung kommt alles an.
// chunkBytes = Nutzdaten je Paket (Default 1024, gerade Zahl, 64..1400).
void isisSendSnapshot(uint32_t ip, uint16_t port, uint16_t maxPakete = 0,
                      uint16_t pauseMs = 0, uint16_t chunkBytes = 1024);

// Bildschirmkopie als Base64 ueber die USB-Konsole (115200 8N1).
// faktor = 1 volle Groesse (480x480, ~55 s), 2 = halbe (240x240, ~14 s).
// Der Nachweis-Kanal: die Konsole funktioniert immer, waehrend UDP zwischen
// VLAN10 und dem Entwicklungs-PC je nach Netzsicherheit stumm verworfen wird.
// Zeilenformat:  SHOTB64|<nr>|<base64>   ...   SHOTEND|<anzahl>
void isisSendSnapshotSerial(uint8_t faktor = 2);
