#pragma once
//
// net.h — network / transport layer: WiFi, UDP data (5005), discovery beacon
// (5006), uplink, captive-portal provisioning and OTA.
//
// ESP32 only.  Everything that has to be host-testable lives in proto.cpp /
// displaycfg.cpp / buttons.cpp instead.
//
// Sources: ISIS_FIRMWARE_SPEC.md sections 4, 4.1, 13, 14
//          IP_TRANSPORT_SPEC.md  sections 3, 4
//
#if defined(ISIS_ESP32)

#include <Arduino.h>   // String/Runtime fuer die OTA-API
#include <stdint.h>
#include "proto.h"

#define ISIS_UDP_DATA_PORT   5005
#define ISIS_UDP_DISCOVERY_PORT 5006

#define ISIS_BEACON_FAST_MS  5000   // 5 s until the first IDENT-ACK
#define ISIS_BEACON_SLOW_MS  60000  // 60 s afterwards
#define ISIS_OTA_WINDOW_MS   600000 // CFG:ota=1 opens OTA for 10 minutes

// Bring up WiFi (STA with NVS credentials) and both UDP sockets.
// Returns false when the device ended up in setup mode.
bool netBegin();

// Service sockets, beacon timing, watchdog and OTA.  Call from loop().
void netLoop(uint32_t nowMs);

// True while the FDC is delivering data (< 5 s since the last frame).
bool netOnline(uint32_t nowMs);

// Verbindungszustand fuer die Anzeige:
//   FDC_OFFLINE  - seit >5 s kein Frame
//   SIM_OFFLINE  - FDC antwortet, meldet aber SIM:0
//   OK           - FDC da und Simulator verbunden (oder Flag unbekannt)
enum NetLinkStatus : uint8_t {
    NET_LINK_FDC_OFFLINE = 0,
    NET_LINK_SIM_OFFLINE = 1,
    NET_LINK_OK          = 2,
};
NetLinkStatus netLinkStatus(uint32_t nowMs);

// Diagnose-Modus: solange das FDC-Werkzeug (Panel-Monitor oder Live-Diag)
// alle 60 s ein "DIAG:1" schickt, ist der Simulator-Zustand egal — das
// Sim-Banner verschwindet und rechts neben dem Baro-Wert steht rot DIAG.
// Mehr als 60 s ohne neues DIAG:1 beendet den Modus (Zeitstempel laeuft aus,
// ueberlebt also auch einen abgestuerzten Monitor).
void netNoteDiag(uint32_t nowMs);
bool netDiagActive(uint32_t nowMs);

// true, wenn das Panel im Stationsbetrieb im WLAN haengt (nicht Setup-AP).
bool netStaConnected();

// Adresse des letzten Gegenuebers (FDC). false, solange noch keins bekannt ist.
// Wird fuer die Bildschirmkopie gebraucht (snapshot.cpp).
bool netPeerAddress(uint32_t &ip, uint16_t &port);

// Rohes Paket ueber denselben Socket schicken, den auch der Uplink benutzt.
// Wichtig fuer die Bildschirmkopie: dieser Socket kommt nachweislich ueber die
// VLAN-Grenze (Beacons/Uplink), ein frischer Socket mit freiem Quellport wird
// von der Netzsicherheit verworfen.
bool netSendChunk(uint32_t ip, uint16_t port, const uint8_t *kopf, size_t kopfLen,
                  const uint8_t *daten, size_t len);

// Sprache der FDC-Oberflaeche ("de", "en", "fr", "th"); Default "en".
const char *netLanguage();

// Send one uplink frame to the FDC (BTN:, QNHSET:, ENC:, OTA:progress=...).
// Does nothing while no peer is known.
void netSendUplink(const char *frame);

// Einzelantwort an eine beliebige Adresse (z. B. "IDENT:..." auf ein "VER" aus
// dem Panel-Monitor). Laeuft ueber den Discovery-Socket — derselbe Weg, den
// "IDENT?" -> sendBeacon() schon nimmt, und damit sicher ausserhalb des
// Empfangspuffers des Daten-Sockets.
void netSendTo(const IPAddress &dst, uint16_t port, const char *frame);

// Remember who talks to us so uplinks can be addressed.
void netNotePeer(uint32_t ip, uint16_t port);

// Beacon helpers.
void netResetBeacon();        // back to the 5 s interval (FDC "Scan" menu)
bool netBeaconFast();

// Identity strings for the beacon / info page.
const char *netMacString();
const char *netMacSerial();   // MAC ohne Doppelpunkte (Seriennummer)
const char *netIpString();
const char *netFwVersion();

// --- OTA (spec section 13) --------------------------------------------------
void netOpenOtaWindow(uint32_t nowMs,
                        uint32_t windowMs = 0);  // CFG:ota=1, 0 = Standardfenster
// OTA-Passwort aus NVS (wird beim ersten Zugriff einmalig erzeugt).
const String& netOtaPassword();
// Neues OTA-Passwort in NVS ablegen (leer = unveraendert lassen).
bool netSetOtaPassword(const String& pw);
bool netOtaWindowOpen(uint32_t nowMs);
// true, solange espota Daten schreibt (dann nicht zeichnen).
bool netOtaInProgress();
// Fortschritt 0..100 des laufenden Uploads (espota).
int  netOtaProgress();
// Nur fuer die Optik-Abstimmung: Overlay mit festem Prozentwert zeigen.
void netSetOtaDemo(int prozent);   // -1 = aus
int  netOtaDemoPct();
// Diagnose: wie oft wurde das Fortschritts-Overlay neu gezeichnet?
void netOtaCountDraw(int prozent);
int  netOtaDrawCount();
int  netOtaLastPct();
int  netOtaProgressCalls();
// Hardware-Revision des Panels (NVS, Default 1). Analog zu den Arduino-Panels,
// die ihre Rev im EEPROM halten und per IDENT/HWREV melden.
uint8_t isisHwRev();
bool    isisSetHwRev(uint8_t rev);
void netServiceOta();

// --- provisioning / captive portal (spec section 4.1) -----------------------
// Starts SoftAP + captive portal and does not return until the portal closes
// or the 10 minute timeout expires, then reboots.  `reason` is only logged.
void netEnterSetup(const char *reason);

bool netWifiConnected();
void netEraseWifiCredentials();

// Info page shown on the panel while the setup portal is running.
void netDrawSetupScreen();

#endif // ISIS_ESP32
