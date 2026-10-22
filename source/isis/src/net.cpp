//
// net.cpp — network / transport layer for the A320 ISIS panel.
//
//   UDP 5005  data + control in, uplink out (text protocol, ';' terminated)
//   UDP 5006  discovery beacon (broadcast) + IDENT?/IDENT-ACK
//   SoftAP + captive portal only while provisioning (never in normal operation)
//   ArduinoOTA on UDP 3232 only after CFG:ota=1 and only for 10 minutes
//
// Hardening: the panel never opens an outgoing connection (no NTP, no cloud,
// no update polling).  Credentials live exclusively in NVS - nothing in code.
//
// Sources: ISIS_FIRMWARE_SPEC.md sections 4, 4.1, 13, 14
//          IP_TRANSPORT_SPEC.md  sections 3, 4
//
#include "net.h"

#if defined(ISIS_ESP32)

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include "portal.h"
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include <esp_mac.h>

#include "ISISCommon.h"   // isisState, isisSettings
#include "buttons.h"
#include "displaycanvas.h"
#include "snapshot.h"      // SHOT: Bildschirmkopie per UDP
#include "displaycfg.h"

#ifndef ISIS_FW_VERSION
#define ISIS_FW_VERSION "0.0.0"
#endif

namespace {

WiFiUDP g_udpData;    // 5005
WiFiUDP g_udpDisc;    // 5006

ProtoContext             g_protoCtx;
IsisDisplayValues        g_values;
DataWatchdog             g_watchdog;

// SIM-Flag aus dem letzten Frame: -1 = noch nie gesehen (alter FDC)
int                      g_simFlag = -1;

// Sprache des FDC fuer die Statusmeldungen (Default Englisch).
char                     g_lang[4] = "en";

// CFG layers, priority FDC > NVS > default (spec section 4.1)
IsisCfgLayers g_cfgLayers;
uint32_t      g_cfgPersisted = 0;

bool     g_shotPend       = false;   // Bildschirmkopie angefordert
uint32_t g_shotPendIp     = 0;
uint16_t g_shotPendPort   = 0;
uint16_t g_shotPendPakete = 0;
uint16_t g_shotPendPause  = 0;
uint16_t g_shotPendChunk  = 1024;

uint32_t g_peerIp   = 0;
uint16_t g_peerPort = 0;
bool     g_havePeer = false;

uint32_t g_beaconLastMs  = 0;
bool     g_beaconFast    = true;

uint32_t g_otaUntilMs    = 0;
bool     g_otaRunning    = false;
bool     g_otaWindowOpen = false;
bool     g_otaInProgress = false;
String   g_otaPassword;
int      g_otaProgress = -1;   // -1 = noch kein Fortschritt
static int g_otaProgressCnt = 0; // wie oft onProgress gefeuert hat
int      g_otaDemoPct  = -1;   // >=0 -> Overlay ohne echten Flash

bool     g_connected     = false;
uint32_t g_lastStatusMs  = 0;

char g_mac[18] = "00:00:00:00:00:00";
char g_ip[16]  = "0.0.0.0";

// --- small helpers ---------------------------------------------------------
void formatMac(const uint8_t *mac, char *out)
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// MAC direkt aus der eFuse: WiFi.macAddress() liefert 00:00:00:00:00:00,
// solange kein Interface gestartet ist. Genau daran scheiterte der AP-Name
// ("ISIS-0000" im WLAN, "ISIS-2218" auf dem Splash).
static void readStationMac(char out[18])
{
    uint8_t mac[6] = {0};
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
#else
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
#endif
    formatMac(mac, out);
}

void refreshIdentity()
{
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    bool blank = true;
    for (int i = 0; i < 6; i++) if (mac[i] != 0) { blank = false; break; }
    if (blank) readStationMac(g_mac);
    else       formatMac(mac, g_mac);

    IPAddress ip = WiFi.localIP();
    snprintf(g_ip, sizeof(g_ip), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

uint32_t ipToU32(const IPAddress &ip)
{
    return ((uint32_t)ip[0] << 24) | ((uint32_t)ip[1] << 16) |
           ((uint32_t)ip[2] << 8) | (uint32_t)ip[3];
}

// ---------------------------------------------------------------------------
// Beacon (spec section 4)
// ---------------------------------------------------------------------------
void sendBeacon(const IPAddress *dst, uint16_t port)
{
    char frame[128];
    if (protoBuildBeacon(frame, sizeof(frame), g_mac, g_ip, ISIS_FW_VERSION) < 0) return;

    if (dst) {
        g_udpDisc.beginPacket(*dst, port);
    } else {
        g_udpDisc.beginPacket(IPAddress(255, 255, 255, 255), ISIS_UDP_DISCOVERY_PORT);
    }
    g_udpDisc.write((const uint8_t *)frame, strlen(frame));
    g_udpDisc.endPacket();
}

// ---------------------------------------------------------------------------
// CFG handling: FDC > NVS > default
// ---------------------------------------------------------------------------
void resolveAndApplyCfg()
{
    IsisCfg resolved;
    const uint32_t mask = displaycfgResolve(resolved, g_cfgLayers);

    // Keep the single effective configuration used by the whole application.
    displaycfgCurrent() = resolved;

    if (resolved.ota) {
        // handled by the caller (needs the current millis())
    }

    // Persist whatever the FDC just told us so the device starts correctly
    // even without an FDC (spec section 7).
    // displaycfgPersistMask filtert Assign heraus (Modus, kein Zustand).
    const uint32_t persist = displaycfgPersistMask(g_cfgLayers.fdcMask);
    if (persist & ~g_cfgPersisted) {
        displaycfgSaveNvs(resolved, persist);
        g_cfgPersisted |= persist;
    }

    if (mask) displaycfgApplyBrightness(resolved);
}

// ---------------------------------------------------------------------------
// Apply a parsed downlink frame to isisState.
// ---------------------------------------------------------------------------
void applyValues(const IsisDisplayValues &v, uint16_t mask, uint32_t nowMs)
{
    const auto bit = [](int idx) -> uint16_t { return (uint16_t)(1u << idx); };

    if (mask & bit(ISIS_DSP_AIRSPEED)) isisState.rawAirspeed = v.airspeed;
    if (mask & bit(ISIS_DSP_MACH))     isisState.machSpeed   = v.mach;
    if (mask & bit(ISIS_DSP_ALTITUDE)) isisState.rawAltitude = v.altitude;
    if (mask & bit(ISIS_DSP_PITCH))    isisState.rawPitchAngle = v.pitch;
    if (mask & bit(ISIS_DSP_BANK))     isisState.rawBankAngle  = v.bank;
    if (mask & bit(ISIS_DSP_BALL))     isisState.ballPos       = v.ball;

    // QNH runs through the settle window so a locally turned value and the
    // value coming back from the simulator do not fight (spec section 5).
    if (mask & bit(ISIS_DSP_QNH)) buttonsQnhSetRemote(v.qnhMb, nowMs);

    if (mask & bit(ISIS_DSP_STD)) isisState.isStdPressure = v.std ? 1 : 0;

    // Landing System (Runde 7)
    if (mask & bit(ISIS_DSP_LS_ACTIVE)) isisState.lsActive   = v.lsActive ? 1 : 0;
    if (mask & bit(ISIS_DSP_LOC_DEV))   isisState.lsLocDev   = v.locDev;
    if (mask & bit(ISIS_DSP_LOC_VALID)) isisState.lsLocValid = v.locValid ? 1 : 0;
    if (mask & bit(ISIS_DSP_GS_DEV))    isisState.lsGsDev    = v.gsDev;
    if (mask & bit(ISIS_DSP_GS_VALID))  isisState.lsGsValid  = v.gsValid ? 1 : 0;
}

void handleFrame(const char *buf, const IPAddress &from, uint16_t fromPort, uint32_t nowMs)
{
    if (!buf || !*buf) return;

    // Bildschirmkopie anfordern: ein Frame mit "SHOT;" irgendwo drin. Das Bild
    // geht dann an den Absender auf Port 5020. Bewusst hier oben und nicht im
    // Parser - der Trigger soll unabhaengig von DSP/CFG/SIM funktionieren.
    const char *shot = strstr(buf, "SHOT");
    if (shot != nullptr) {
        uint16_t maxPakete = 0, pauseMs = 0, chunkBytes = 1024;
        const char *p1 = strchr(shot, ':');
        if (p1) {
            maxPakete = (uint16_t)atoi(p1 + 1);
            const char *p2 = strchr(p1 + 1, ':');
            if (p2) {
                pauseMs = (uint16_t)atoi(p2 + 1);
                const char *p3 = strchr(p2 + 1, ':');
                if (p3) chunkBytes = (uint16_t)atoi(p3 + 1);
            }
        }
        // NICHT sofort senden: handleFrame() laeuft mitten in serviceSocket(),
        // d.h. der Empfangspuffer von g_udpData ist belegt. Der Versand wird
        // vorgemerkt und in netLoop() ausserhalb des Empfangs erledigt.
        g_shotPendIp     = ipToU32(from);
        g_shotPendPort   = fromPort;
        g_shotPendPakete = maxPakete;
        g_shotPendPause  = pauseMs;
        g_shotPendChunk  = chunkBytes;
        g_shotPend       = true;
    }

    // --- discovery chatter ---------------------------------------------------
    if (strncmp(buf, "IDENT-ACK", 9) == 0) {
        // The FDC has seen us: stretch the beacon interval to 60 s.
        g_beaconFast = false;
        netNotePeer(ipToU32(from), fromPort);
        return;
    }
    if (strncmp(buf, "IDENT?", 6) == 0) {
        // Scan probe from the FDC options menu: answer immediately, unicast.
        netNotePeer(ipToU32(from), fromPort);
        if (g_connected) sendBeacon(&from, fromPort);
        return;
    }

    // --- data frame ----------------------------------------------------------
    const ProtoResult res = protoParseFrame(buf, g_values, g_cfgLayers.fdc, g_protoCtx);

    if (res.status == ProtoStatus::DUPLICATE) {
        // UDP duplicate: ignore the payload but count it as a sign of life.
        netNotePeer(ipToU32(from), fromPort);
        g_watchdog.onData(nowMs);
        return;
    }
    if (res.status != ProtoStatus::OK) return;

    netNotePeer(ipToU32(from), fromPort);
    g_watchdog.onData(nowMs);

    // Diagnose: erster Frame und danach jede Sekunde eine kurze Zeile.
    // Ohne das ist auf der Konsole nicht zu sehen, ob der FDC ueberhaupt ankommt.
    static uint32_t frames = 0;
    static uint32_t lastLogMs = 0;
    frames++;
    if (frames == 1) {
        Serial.printf("[isis] erster Frame vom FDC %s:%u (%u Bytes)\n",
                      from.toString().c_str(), (unsigned)fromPort,
                      (unsigned)strlen(buf));
    }
    if ((uint32_t)(nowMs - lastLogMs) >= 5000) {
        lastLogMs = nowMs;
        Serial.printf("[isis] Frames gesamt %lu  |  SPD %.1f  ALT %.0f  PITCH %.2f  BANK %.2f  QNH %d"
                      "  |  LS %d LOC %d %.2f GS %d %.2f\n",
                      (unsigned long)frames, isisState.rawAirspeed, isisState.rawAltitude,
                      isisState.rawPitchAngle, isisState.rawBankAngle, (int)isisState.mbPressure,
                      isisState.lsActive, isisState.lsLocValid, isisState.lsLocDev,
                      isisState.lsGsValid, isisState.lsGsDev);
    }

    if (res.langPresent && strcmp(res.lang, g_lang) != 0) {
        snprintf(g_lang, sizeof(g_lang), "%s", res.lang);
        Serial.printf("[isis] Sprache vom FDC: %s\n", g_lang);
        isisState.forceRedraw = true;
    }

    if (res.simPresent) {
        if (res.sim != g_simFlag) {
            Serial.printf("[isis] Simulator %s (SIM:%d)\n",
                          res.sim ? "verbunden" : "NICHT verbunden", res.sim ? 1 : 0);
        }
        g_simFlag = res.sim ? 1 : 0;
    }

    applyValues(g_values, res.dspMask, nowMs);

    if (res.dispBl) {
        // DISP_BL is the base brightness from the FDC (highest priority).
        g_cfgLayers.fdc.blBase = g_values.dispBl;
        g_cfgLayers.fdcMask |= CFG_BL_BASE;
        g_cfgPersisted &= ~CFG_BL_BASE;
    }

    if (res.cfgMask) {
        g_cfgLayers.fdcMask |= res.cfgMask;
        g_cfgPersisted &= ~res.cfgMask;
        resolveAndApplyCfg();
    } else if (res.dispBl) {
        // Wichtig: neu aufloesen, nicht die zwischengespeicherte Konfiguration
        // anwenden. displaycfgCurrent() ist der Stand VOR diesem Frame - damit
        // wurde jede DISP_BL-Aenderung verschluckt (blBase blieb auf dem
        // NVS-Startwert stehen).
        resolveAndApplyCfg();
    }

    if (res.otaRequest) netOpenOtaWindow(nowMs);

    isisState.forceRedraw = true;
}

void serviceSocket(WiFiUDP &sock, uint32_t nowMs)
{
    int size = sock.parsePacket();
    while (size > 0) {
        // Sized from the parser limit in proto.h: real FDC frames are ~155 bytes.
        char buf[ISIS_MAX_FRAME + 1];
        const int n = sock.read(buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            handleFrame(buf, sock.remoteIP(), sock.remotePort(), nowMs);
        }
        size = sock.parsePacket();
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
const char *netMacString() { return g_mac; }
const char *netMacSerial()
{
    // "CC:68:C7:98:22:18" -> "CC68C7982218"
    static char ser[13];
    int j = 0;
    for (int i = 0; g_mac[i] && j < 12; ++i)
        if (g_mac[i] != ':') ser[j++] = g_mac[i];
    ser[j] = '\0';
    return ser;
}
const char *netIpString() { return g_ip; }
const char *netFwVersion() { return ISIS_FW_VERSION; }

void netNotePeer(uint32_t ip, uint16_t port)
{
    g_peerIp   = ip;
    g_peerPort = (port != 0) ? port : ISIS_UDP_DATA_PORT;
    g_havePeer = true;
}

bool netWifiConnected() { return WiFi.status() == WL_CONNECTED; }

bool netOnline(uint32_t nowMs) { return g_watchdog.online(nowMs); }

const char *netLanguage() { return g_lang; }

NetLinkStatus netLinkStatus(uint32_t nowMs)
{
    if (!netOnline(nowMs))            return NET_LINK_FDC_OFFLINE;
    if (g_simFlag == 0)               return NET_LINK_SIM_OFFLINE;
    return NET_LINK_OK;
}
bool netStaConnected()
{
    return WiFi.status() == WL_CONNECTED;
}


void netResetBeacon()
{
    g_beaconFast = true;
    g_beaconLastMs = 0;
}

bool netBeaconFast() { return g_beaconFast; }

void netSendUplink(const char *frame)
{
    if (!frame || !*frame) return;
    if (!g_connected || !g_havePeer) return; // nothing to talk to yet

    g_udpData.beginPacket(IPAddress(g_peerIp), g_peerPort);
    g_udpData.write((const uint8_t *)frame, strlen(frame));
    g_udpData.endPacket();
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------
bool netBegin()
{
    // --- configuration: NVS layer ---
    IsisCfg nvsCfg;
    g_cfgLayers.nvsMask = displaycfgLoadNvs(nvsCfg);
    g_cfgLayers.nvs     = nvsCfg;

    IsisCfg resolved;
    displaycfgResolve(resolved, g_cfgLayers);
    displaycfgCurrent() = resolved;
    displaycfgApplyBrightness(resolved);

    // --- WiFi station, credentials exclusively from NVS ---
    WiFi.persistent(true);   // credentials are stored in NVS by the core
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);    // keep UDP latency low (10-20 Hz data stream)

    if (resolved.netMode == ISIS_NET_DHCP) {
        WiFi.begin();        // no SSID/password in the source, ever
    } else {
        // Static IP.  Gateway 0.0.0.0 for "static_nogw" -> no route off-subnet
        // (spec section 14).  Values come from NVS (boarding page).
        IPAddress ip, gw, sn, dns;
        Preferences p;
        if (p.begin("isis", true)) {
            ip  = IPAddress(p.getUInt("ip_addr", 0));
            gw  = IPAddress(resolved.netMode == ISIS_NET_STATIC_NOGW
                                ? 0u : p.getUInt("ip_gw", 0));
            sn  = IPAddress(p.getUInt("ip_mask", 0));
            dns = IPAddress(p.getUInt("ip_dns", 0));
            p.end();
            if ((uint32_t)ip != 0 && (uint32_t)sn != 0) {
                WiFi.config(ip, gw, sn, dns);
            }
        }
        WiFi.begin();
    }

    // Verbinden mit grosszuegigem Zeitfenster: WPA2-Handshake + DHCP in einem
    // VLAN brauchen locker 10-20 s. Mit 10 s ging das Panel reproduzierbar in
    // den Setup-Modus, obwohl die Zugangsdaten korrekt in der NVS standen.
    const uint32_t t0 = millis();
    uint32_t lastLog = 0;
    while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - t0) < 30000) {
        esp_task_wdt_reset();
        const uint32_t el = (uint32_t)(millis() - t0);
        if (el - lastLog >= 5000) {
            lastLog = el;
            const int st = (int)WiFi.status();
            const char *stName = (st == WL_NO_SSID_AVAIL) ? "SSID nicht gefunden"
                               : (st == WL_CONNECT_FAILED) ? "Verbindung abgelehnt (Passwort?)"
                               : (st == WL_CONNECTION_LOST) ? "Verbindung verloren"
                               : (st == WL_IDLE_STATUS) ? "idle"
                               : (st == WL_DISCONNECTED) ? "getrennt" : "?";
            Serial.printf("[isis] warte auf WLAN: %lu s, Status %d (%s)\n",
                          (unsigned long)(el / 1000), st, stName);
        }
        delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[isis] WLAN verbunden nach %lu s, IP %s\n",
                      (unsigned long)((millis() - t0) / 1000),
                      WiFi.localIP().toString().c_str());
    } else {
        Serial.printf("[isis] keine WLAN-Verbindung nach %lu s (Status %d)\n",
                      (unsigned long)((millis() - t0) / 1000), (int)WiFi.status());
    }

    if (WiFi.status() != WL_CONNECTED) {
        g_connected = false;
        return false;
    }

    g_connected = true;
    refreshIdentity();

    g_udpData.begin(ISIS_UDP_DATA_PORT);
    g_udpDisc.begin(ISIS_UDP_DISCOVERY_PORT);

    // Send the first beacon right away (spec section 4).
    g_beaconFast   = true;
    g_beaconLastMs = millis() - ISIS_BEACON_FAST_MS;

    Serial.printf("[isis] WiFi up: %s  MAC %s  FW %s\n", g_ip, g_mac, ISIS_FW_VERSION);
    return true;
}

// ---------------------------------------------------------------------------
// Loop service
// ---------------------------------------------------------------------------
void netLoop(uint32_t nowMs)
{
    if (g_connected) {
        serviceSocket(g_udpData, nowMs);
        serviceSocket(g_udpDisc, nowMs);

        // --- angeforderte Bildschirmkopie (ausserhalb des Empfangs senden) ---
        if (g_shotPend) {
            g_shotPend = false;
            isisSendSnapshot(g_shotPendIp, g_shotPendPort, g_shotPendPakete,
                             g_shotPendPause, g_shotPendChunk);
        }

        // --- beacon ---
        const uint32_t interval = g_beaconFast ? ISIS_BEACON_FAST_MS : ISIS_BEACON_SLOW_MS;
        if ((uint32_t)(nowMs - g_beaconLastMs) >= interval) {
            g_beaconLastMs = nowMs;
            sendBeacon(nullptr, ISIS_UDP_DISCOVERY_PORT);
            // Zusaetzlich UNICAST an den FDC: haengt das Panel in einem anderen
            // VLAN als der FDC, kommt der Broadcast nie an und der FDC haelt das
            // Panel fuer offline (sendet dann nur einen Frame). Die IP stammt aus
            // dem letzten empfangenen Frame.
            if (g_havePeer) {
                const IPAddress peer((uint8_t)(g_peerIp >> 24),
                                     (uint8_t)((g_peerIp >> 16) & 0xFF),
                                     (uint8_t)((g_peerIp >> 8) & 0xFF),
                                     (uint8_t)(g_peerIp & 0xFF));
                sendBeacon(&peer, ISIS_UDP_DISCOVERY_PORT);
            }
            refreshIdentity();
        }

        // --- keepalive watchdog: > 5 s without data -> defined state ---
        if (g_watchdog.offlineEdge(nowMs)) {
            Serial.println("[isis] FDC offline (>5 s) - showing defined state");
            isisState.rawAirspeed = 0.0f;
            isisState.rawAltitude = 0.0f;
            isisState.machSpeed   = 0.0f;
            isisState.forceRedraw = true;
        }

        if (WiFi.status() != WL_CONNECTED) g_connected = false;
    } else if ((uint32_t)(nowMs - g_lastStatusMs) >= 1000) {
        g_lastStatusMs = nowMs;
        if (WiFi.status() == WL_CONNECTED) {
            g_connected = true;
            refreshIdentity();
            netResetBeacon();
        }
    }

    // --- OTA ---
    netServiceOta();
    if (g_otaWindowOpen && (int32_t)(nowMs - g_otaUntilMs) >= 0) {
        g_otaWindowOpen = false;
        // Wichtig: ArduinoOTA NICHT beenden. Nach end() bekommt ein zweites
        // begin() den UDP-Socket nicht mehr hoch - dann laeuft jeder weitere
        // OTA-Versuch ins Leere ("No response from the ESP"), bis man das
        // Panel neu startet. Der Socket bleibt offen; Pakete werden nur
        // verarbeitet, solange das Fenster offen ist (netServiceOta).
        if (!g_otaInProgress) {
            Serial.println("[isis] OTA window closed");
        }
    }
}

// ---------------------------------------------------------------------------
// OTA (spec section 13) — only on request, only for 10 minutes
// ---------------------------------------------------------------------------
bool netOtaWindowOpen(uint32_t nowMs)
{
    if (!g_otaWindowOpen) return false;
    return (int32_t)(nowMs - g_otaUntilMs) < 0;
}

// Passwort aus NVS holen; ist keins da, einmalig eines erzeugen.
// Wird beim ersten Zugriff geladen, damit "OTAPW" auf der Konsole das
// Passwort auch dann zeigen kann, wenn das OTA-Fenster noch zu ist.
static String loadOrCreateOtaPassword()
{
    Preferences p;
    String pw;
    if (p.begin("isis", false)) {
        pw = p.getString("ota_pass", "");
        if (pw.length() == 0) {
            char gen[17];
            for (int i = 0; i < 16; ++i) gen[i] = "0123456789abcdef"[esp_random() & 0xF];
            gen[16] = '\0';
            pw = gen;
            p.putString("ota_pass", pw);
        }
        p.end();
    }
    return pw;
}

const String& netOtaPassword()
{
    if (g_otaPassword.length() == 0) g_otaPassword = loadOrCreateOtaPassword();
    return g_otaPassword;
}

bool netSetOtaPassword(const String& pw)
{
    if (pw.length() == 0) return false;
    Preferences p;
    if (!p.begin("isis", false)) return false;
    p.putString("ota_pass", pw);
    p.end();
    g_otaPassword = pw;
    Serial.println("[isis] OTA-Passwort in NVS gespeichert");
    return true;
}

void netOpenOtaWindow(uint32_t nowMs, uint32_t windowMs)
{
    const uint32_t dauer = (windowMs == 0) ? ISIS_OTA_WINDOW_MS : windowMs;

    if (g_otaRunning) {
        g_otaUntilMs = nowMs + dauer; // extend
        g_otaWindowOpen = true;
        Serial.printf("[isis] OTA window extended (%u s)\n", (unsigned)(dauer / 1000));
        return;
    }

    // Nichts Geheimes ist einkompiliert: das Passwort liegt in NVS.
    g_otaPassword = netOtaPassword();

    ArduinoOTA.setPort(3232);
    ArduinoOTA.setHostname("ISIS");
    if (g_otaPassword.length()) ArduinoOTA.setPassword(g_otaPassword.c_str());

    ArduinoOTA.onStart([&]() {
        g_otaInProgress = true;
        g_otaProgress   = 0;
        Serial.println("[isis] OTA start");
    });
    ArduinoOTA.onProgress([&](unsigned int progress, unsigned int total) {
        const int pct = total ? (int)((progress * 100u) / total) : 0;
        g_otaProgress = pct;
        g_otaProgressCnt++;
        char frame[32];
        if (protoBuildOtaProgress(frame, sizeof(frame), pct) > 0) netSendUplink(frame);
    });
    ArduinoOTA.onEnd([&]() {
        char frame[32];
        if (protoBuildOtaDone(frame, sizeof(frame)) > 0) netSendUplink(frame);
        g_otaInProgress = false;
        Serial.println("[isis] OTA done - rebooting");
    });
    ArduinoOTA.onError([&](ota_error_t err) {
        g_otaInProgress = false;
        Serial.printf("[isis] OTA error %u\n", (unsigned)err);
    });

    ArduinoOTA.begin();
    g_otaRunning     = true;
    g_otaWindowOpen  = true;
    g_otaUntilMs     = nowMs + dauer;
    Serial.printf("[isis] OTA window open for %u s\n", (unsigned)(dauer / 1000));
}

void netServiceOta()
{
    if (!g_otaRunning) return;
    // Ausserhalb des Zeitfensters wird nichts angenommen - der Socket bleibt
    // aber initialisiert (siehe netLoop), damit das naechste Fenster sofort
    // wieder erreichbar ist.
    if (!g_otaInProgress && !netOtaWindowOpen(millis())) return;
    ArduinoOTA.handle();
}

bool netOtaInProgress()
{
    return g_otaInProgress;
}

int netOtaProgress()
{
    return g_otaProgress;
}

void netSetOtaDemo(int prozent)
{
    g_otaDemoPct = (prozent < 0) ? -1 : (prozent > 100 ? 100 : prozent);
    Serial.printf("[isis] OTA-Demo %d %%\n", g_otaDemoPct);
}

int netOtaDemoPct()
{
    return g_otaDemoPct;
}

// Waehrend eines Uploads darf nicht geloggt werden (UART-Interrupt mit
// Flash-Code -> Panic). Deshalb zaehlen wir nur und fragen hinterher per
// Konsole ab (OTADBG).
static int g_otaDrawCount   = 0;
static int g_otaLastPct     = -1;

void netOtaCountDraw(int prozent)
{
    g_otaDrawCount++;
    g_otaLastPct = prozent;
}

int netOtaProgressCalls() { return g_otaProgressCnt; }

int netOtaDrawCount() { return g_otaDrawCount; }
int netOtaLastPct()   { return g_otaLastPct; }

// ---------------------------------------------------------------------------
// Hardware-Revision (NVS, Default 1)
//   Die Arduino-Panels fuehren ihre Rev im EEPROM (ACP: HWREV_MIN/MAX). Der
//   ESP hat kein EEPROM, also NVS - gleiche Idee, gleiche Meldung.
// ---------------------------------------------------------------------------
static uint8_t g_hwRevCache = 0;

uint8_t isisHwRev()
{
    if (g_hwRevCache == 0) {
        uint8_t rev = 1;
        Preferences p;
        if (p.begin("isis", false)) {
            rev = (uint8_t)p.getUChar("hwrev", 1);
            p.end();
        }
        if (rev < 1 || rev > 9) rev = 1;
        g_hwRevCache = rev;
    }
    return g_hwRevCache;
}

bool isisSetHwRev(uint8_t rev)
{
    if (rev < 1 || rev > 9) return false;
    Preferences p;
    if (!p.begin("isis", false)) return false;
    p.putUChar("hwrev", rev);
    p.end();
    g_hwRevCache = rev;
    return true;
}

// ---------------------------------------------------------------------------
// Provisioning: SoftAP + captive portal (spec section 4.1)
// ---------------------------------------------------------------------------
void netEraseWifiCredentials()
{
    WiFi.disconnect(true, true); // wipe the stored SSID/password from NVS
}

void netDrawSetupScreen()
{
    // Lesbarkeit auf 480x480: echte FreeFonts statt der 8-px-Standardschrift.
    // Vorher stand hier setTextSize(0.9f) -> ~7 px Glyphen, am Panel unlesbar.
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    lcd.setFont(&fonts::FreeSansBold24pt7b);
    lcd.setTextDatum(top_center);
    lcd.drawString("NETZWERK-SETUP", 240, 10);
    lcd.drawFastHLine(20, 62, 440, TFT_DARKGREY);

    char ap[32];
    const char *mac = g_mac;
    snprintf(ap, sizeof(ap), "ISIS-%c%c%c%c",
             mac[12], mac[13], mac[15], mac[16]); // last two MAC bytes, no colons

    lcd.setTextDatum(top_left);

    // 1) WLAN-Name
    lcd.setFont(&fonts::FreeSans12pt7b);
    lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    lcd.drawString("1)  WLAN am Handy waehlen", 20, 78);
    lcd.setFont(&fonts::FreeSansBold24pt7b);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.drawString(ap, 20, 106);

    // 2) Portal-Adresse
    lcd.setFont(&fonts::FreeSans12pt7b);
    lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    lcd.drawString("2)  Im Browser oeffnen", 20, 168);
    lcd.setFont(&fonts::FreeSansBold24pt7b);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.drawString("192.168.4.1", 20, 196);

    // 3) + 4) Ablauf
    lcd.setFont(&fonts::FreeSans12pt7b);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.drawString("3)  WLAN + Passwort eingeben", 20, 258);
    lcd.drawString("4)  Speichern -> Neustart", 20, 288);

    // Fusszeile: Identitaet + Timeout
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.drawString(String("FW ") + ISIS_FW_VERSION + "   MAC " + g_mac, 20, 336);
    lcd.drawString("Timeout 10 min, danach Neustart", 20, 372);
}

void netEnterSetup(const char *reason)
{
    Serial.printf("[isis] entering setup mode (%s)\n", reason ? reason : "");

    // SoftAP-Name aus der eFuse-MAC (g_mac ist hier noch leer).
    char apMac[18];
    char apName[24];
    readStationMac(apMac);
    snprintf(apName, sizeof(apName), "ISIS-%c%c%c%c", apMac[12], apMac[13], apMac[15], apMac[16]);
    Serial.printf("[isis] SoftAP: %s (MAC %s)\n", apName, apMac);

    // Eigenes Portal (kein JavaScript, siehe portal.cpp).
    const bool connected = portalRunSetup(apName, []() {
        refreshIdentity();
        IsisCfg bright = displaycfgCurrent();
        bright.blBase = 255;
        displaycfgApplyBrightness(bright);
        netDrawSetupScreen();
        displaycfgComposite(bright);
    });

    if (connected) {
        Serial.println("[isis] credentials saved - rebooting");
    } else {
        Serial.println("[isis] setup beendet ohne WLAN - rebooting");
    }
    delay(400);
    ESP.restart();
}

#endif // ISIS_ESP32

bool netPeerAddress(uint32_t &ip, uint16_t &port)
{
    if (!g_havePeer) return false;
    ip   = g_peerIp;
    port = g_peerPort;
    return true;
}

bool netSendChunk(uint32_t ip, uint16_t port, const uint8_t *kopf, size_t kopfLen,
                  const uint8_t *daten, size_t len)
{
    if (!g_connected) return false;
    if (g_udpData.beginPacket(IPAddress(ip), port) != 1) return false;
    const size_t w1 = g_udpData.write(kopf, kopfLen);
    const size_t w2 = daten ? g_udpData.write(daten, len) : 0;
    const int    e  = g_udpData.endPacket();
    return (e == 1 && w1 == kopfLen && w2 == len);
}
