//
// portal.h — eigenes Netzwerk-Setup-Portal (SoftAP + HTTP ohne JavaScript).
//
#pragma once

#if defined(ISIS_ESP32)

// Startet SoftAP + Portal und blockiert bis gespeichert wurde (true) oder der
// Timeout zuschlaegt (false). Der Aufrufer startet danach neu.
// onApUp wird gerufen, sobald der AP steht (Anzeige zeichnen).
bool portalRunSetup(const char *apName, void (*onApUp)());

#endif // ISIS_ESP32

// Konsolenbefehle ueber die USB-Schnittstelle (115200 8N1):
//   WIFI <ssid>|<passwort>   INFO   LSPOS <gsX> <locY> [<full>] [<locX>]
//   DIM <prozent> [<sekunden>]
// Muss AUCH im Normalbetrieb laufen, nicht nur im Setup-Portal - sonst greifen
// die Befehle am fliegenden Geraet nicht.
void isisSerialPoll();
