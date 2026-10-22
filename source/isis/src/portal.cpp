//
// portal.cpp — eigenes Netzwerk-Setup-Portal fuer das A320-ISIS-Panel.
//
// Warum nicht WiFiManager: dessen Portal-Seite ist eine JavaScript-Formularseite.
// Auf mehreren Handy-Browsern (Captive-Portal-Mini-Browser) fuellt das Antippen
// eines Netzes das SSID-Feld nicht, und der Speichern-POST kommt nie am Panel an
// ("Seite kann nicht geladen werden"). Hier gibt es deshalb bewusst KEIN
// JavaScript: die Netzliste besteht aus Links, das Formular ist ein normales
// HTML-Formular mit absoluter Ziel-URL.
//
// Ablauf:
//   SoftAP ISIS-<MAC> (offen, 192.168.4.1) + DNS-Catch-All -> jede Anfrage
//   landet auf dieser Seite. GET "/" Formular, GET "/use?ssid=.." Feld
//   vorbefuellen, GET "/scan" neu suchen, POST "/save" speichern + verbinden.
//
// Die Zugangsdaten landen ausschliesslich in der NVS (WiFi.persistent), nie im Code.
//
#include "portal.h"

#ifndef ISIS_FW_VERSION
#define ISIS_FW_VERSION "0.0.0"   // wird per build_flag gesetzt
#endif
#include "ISISCommon.h"
#include "CC_ISIS.h"
#include "net.h"        // netPeerAddress (SHOT)
#include "snapshot.h"   // SHOT: Bildschirmkopie   // LSPOS-Kommando (Skalenlage)

#if defined(ISIS_ESP32)

#include <Arduino.h>
#include <DNSServer.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

namespace {

WebServer  g_server(80);
DNSServer  g_dns;
bool       g_connecting   = false;
uint32_t   g_connectStart = 0;
char       g_lastSsid[33] = {0};
char       g_apName[24]   = {0};

void htmlEscape(const char *in, char *out, size_t outLen)
{
    size_t o = 0;
    for (size_t i = 0; in[i] && o + 7 < outLen; ++i) {
        const char c = in[i];
        if (c == '&')      { memcpy(out + o, "&amp;", 5);  o += 5; }
        else if (c == '<') { memcpy(out + o, "&lt;", 4);   o += 4; }
        else if (c == '>') { memcpy(out + o, "&gt;", 4);   o += 4; }
        else if (c == '"') { memcpy(out + o, "&quot;", 6); o += 6; }
        else if ((unsigned char)c < 0x20) { out[o++] = ' '; }
        else { out[o++] = c; }
    }
    out[o] = 0;
}

// Gemeinsames Geruest. Kein JavaScript, alles inline, grosse Schrift fuer Handy.
void sendPage(const char *ssidValue, bool scan)
{
    char ssidEsc[200];
    htmlEscape(ssidValue ? ssidValue : "", ssidEsc, sizeof(ssidEsc));

    String html;
    html.reserve(4096);
    html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
              "<meta name='viewport' content='width=device-width,initial-scale=1'>"
              "<meta http-equiv='Cache-Control' content='no-store'>"
              "<title>A320 ISIS</title></head>"
              "<body style='font-family:sans-serif;background:#111;color:#eee;margin:0;padding:18px;font-size:18px'>"
              "<h1 style='color:#3cf;margin:0 0 4px 0;font-size:1.7em'>A320 ISIS</h1>"
              "<p style='color:#888;margin:0 0 18px 0'>Netzwerk-Setup</p>");

    html += F("<form method='POST' action='/save'>"
              "<p style='margin:0 0 6px 0'>WLAN-Name (SSID)</p>"
              "<input name='ssid' value='");
    html += ssidEsc;
    html += F("' autocomplete='off' style='font-size:1.1em;width:100%;padding:12px;"
              "box-sizing:border-box;border-radius:6px;border:1px solid #555;background:#222;color:#fff'>"
              "<p style='margin:16px 0 6px 0'>Passwort</p>"
              "<input name='pass' type='password' autocomplete='off' style='font-size:1.1em;width:100%;"
              "padding:12px;box-sizing:border-box;border-radius:6px;border:1px solid #555;background:#222;color:#fff'>"
              "<p style='margin:20px 0 0 0'><button style='font-size:1.1em;padding:14px 22px;"
              "border-radius:6px;border:0;background:#3cf;color:#012;font-weight:bold'>Speichern</button></p>"
              "</form>");

    if (scan) {
        const int n = WiFi.scanNetworks();
        html += F("<hr style='border:0;border-top:1px solid #333;margin:26px 0'>"
                  "<p style='margin:0 0 8px 0;color:#888'>Gefundene Netze (antippen setzt den Namen):</p><ul style='padding-left:20px;margin:0'>");
        for (int i = 0; i < n; ++i) {
            char name[200], esc[400];
            snprintf(name, sizeof(name), "%s", WiFi.SSID(i).c_str());
            htmlEscape(name, esc, sizeof(esc));
            html += F("<li style='margin:0 0 10px 0'><a href='/use?ssid=");
            html += esc;   // wird vom Server url-dekodiert
            html += F("' style='color:#3cf;font-size:1.15em;text-decoration:none'>");
            html += esc;
            html += F("</a> <span style='color:#666'>");
            html += String(WiFi.RSSI(i));
            html += F(" dBm</span></li>");
        }
        if (n <= 0) html += F("<li style='color:#c66'>keine Netze gefunden</li>");
        html += F("</ul>");
        WiFi.scanDelete();
    }

    html += F("<p style='margin:26px 0 0 0'><a href='/scan' style='color:#3cf;text-decoration:none'>"
              "&#8635; Neu suchen</a></p>");
    html += F("<p style='margin:22px 0 0 0;color:#555;font-size:0.8em'>A320 ISIS Firmware ");
    html += ISIS_FW_VERSION;
    html += F("</p></body></html>");

    g_server.sendHeader("Cache-Control", "no-store");
    g_server.send(200, "text/html; charset=utf-8", html);
}

void handleRoot()
{
    Serial.println("[portal] GET /");
    sendPage("", true);
}

void handleUse()
{
    String ssid = g_server.arg("ssid");
    Serial.printf("[portal] GET /use  ssid='%s'\n", ssid.c_str());
    sendPage(ssid.c_str(), true);
}

void handleScan()
{
    Serial.println("[portal] GET /scan");
    sendPage(g_lastSsid, true);
}

// Wendet Zugangsdaten an: speichern (NVS) + verbinden. Genutzt von der
// Webseite und von der Konsole (Notweg).
void applyCredentials(const char *ssid, const char *pass)
{
    snprintf(g_lastSsid, sizeof(g_lastSsid), "%s", ssid);
    WiFi.persistent(true);
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(ssid, pass);
    g_connecting   = true;
    g_connectStart = millis();
    Serial.printf("[portal] verbinde mit '%s' ...\n", ssid);
}

// Notweg: WLAN-Zugangsdaten ueber die USB-Konsole. Sinnvoll, wenn gar kein
// Handy zur Hand ist:  WIFI <ssid>|<passwort>
void serialPollImpl()
{
    static char buf[192];
    static size_t len = 0;
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n' || len >= sizeof(buf) - 1) {
            buf[len] = 0;
            len = 0;
            if (strncmp(buf, "WIFI ", 5) == 0) {
                char *arg = buf + 5;
                char *sep = strchr(arg, '|');
                if (sep) {
                    *sep = 0;
                    if (*arg && *(sep + 1)) {
                        Serial.printf("[portal] Konsole: speichere '%s' (%u Zeichen)\n",
                                      arg, (unsigned)strlen(sep + 1));
                        applyCredentials(arg, sep + 1);
                    }
                } else {
                    Serial.println("[portal] Format: WIFI <ssid>|<passwort>");
                }
            } else if (strncmp(buf, "LSPOS ", 6) == 0) {
                int gsX = -1, locTop = -1, full = -1, locX = -1;
                const int n = sscanf(buf + 6, "%d %d %d %d", &gsX, &locTop, &full, &locX);
                if (n >= 2) {
                    isisSetLsLayout(gsX, locTop, full, locX);
                } else {
                    Serial.println("[isis] Format: LSPOS <gsX> <locTop> [<fullPx>] [<locX>]");
                }
            } else if (strncmp(buf, "DIM ", 4) == 0) {
                int prozent = -1, sekunden = -1;
                const int n = sscanf(buf + 4, "%d %d", &prozent, &sekunden);
                if (n >= 1) {
                    isisSetDimPolicy(prozent, sekunden > 0 ? sekunden : isisDimSeconds());
                } else {
                    Serial.println("[isis] Format: DIM <prozent> [<sekunden>]");
                }
            } else if (strncmp(buf, "SHOTS", 5) == 0) {
                int faktor = 2;
                if (buf[5] == ' ') faktor = atoi(buf + 6);
                if (faktor < 1) faktor = 1;
                if (faktor > 4) faktor = 4;
                isisSendSnapshotSerial((uint8_t)faktor);
            } else if (strcmp(buf, "SHOT") == 0) {
                uint32_t ip = 0;
                uint16_t port = 0;
                if (!netPeerAddress(ip, port)) {
                    Serial.println("[shot] kein Gegenueber bekannt - FDC muss einmal gesendet haben");
                } else {
                    isisSendSnapshot(ip, 5020);   // 5020 = Empfaenger auf dem Notebook
                }
            } else if (strcmp(buf, "BLI") == 0) {
                Serial.printf("[isis] BLI: LEDC Kanal7 duty=%u  lcdPanel.getBrightness=%u  Dim=%d %%\n",
                              (unsigned)ledcRead(7), (unsigned)lcdPanel.getBrightness(),
                              isisDimPercent());
            } else if (strncmp(buf, "BLX ", 4) == 0) {
                // Roher PWM-Test auf GPIO 38, an LovyanGFX vorbei. Damit laesst
                // sich klaeren, ob das Backlight ueberhaupt auf PWM reagiert.
                int wert = atoi(buf + 4);
                if (wert < 0)   wert = 0;
                if (wert > 255) wert = 255;
                ledcSetup(7, 5000, 8);
                ledcAttachPin(GPIO_NUM_38, 7);
                ledcWrite(7, wert);
                Serial.printf("[isis] BLX roh: GPIO38 PWM-Kanal 7 = %d\n", wert);
            } else if (strncmp(buf, "BLS ", 4) == 0) {
                // BLS <pin> <freq> <duty>  - PWM-Test auf beliebigem Kandidaten-Pin
                int pin = -1, freq = 100, duty = 0;
                if (sscanf(buf + 4, "%d %d %d", &pin, &freq, &duty) >= 2) {
                    if (duty < 0)   duty = 0;
                    if (duty > 255) duty = 255;
                    if (freq < 50)  freq = 50;
                    ledcSetup(7, freq, 8);
                    ledcAttachPin((gpio_num_t)pin, 7);
                    ledcWrite(7, duty);
                    Serial.printf("[isis] BLS: GPIO%d %d Hz duty %d\n", pin, freq, duty);
                } else {
                    Serial.println("[isis] Format: BLS <pin> <freq> <duty>");
                }
            } else if (strncmp(buf, "BLP ", 4) == 0) {
                // Digitalpegel auf einem Kandidaten-Pin setzen: BLP <pin> <0|1>
                int pin = -1, pegel = -1;
                if (sscanf(buf + 4, "%d %d", &pin, &pegel) == 2) {
                    if (pin == 38 || pin == 46 || pin == 1 || pin == 2 || pin == 40) {
                        ledcDetachPin((gpio_num_t)pin);   // sonst ignoriert digitalWrite
                        pinMode(pin, OUTPUT);
                        digitalWrite(pin, pegel ? HIGH : LOW);
                        Serial.printf("[isis] BLP: GPIO%d = %s (bleibt so bis zum Neustart)\n",
                                      pin, pegel ? "HIGH" : "LOW");
                    } else {
                        Serial.println("[isis] BLP: erlaubt sind 38, 46, 1, 2, 40");
                    }
                } else {
                    Serial.println("[isis] Format: BLP <pin> <0|1>");
                }
            } else if (strcmp(buf, "VER") == 0) {
                // Antwortformat wie bei den Arduino-Panels. Der FCC-Scan
                // schickt "VER\n" an jeden Port und wertet die Zeile
                // "IDENT:<Panel>, v<Version>" aus (parse_ident_response).
                // Ohne das bleibt das ESP-Panel beim Scan unerkannt.
                // Format wie bei den Arduino-Panels:
                //   IDENT:ACP 1 CPT, v2.4 MAQ;STATE:RUNNING;HWREV:1;
                // Erster Token nach dem Komma bleibt die Version (so liest es
                // der FCC-Scanner), MAQ/STATE/HWREV sind Zusatzinfos.
                Serial.printf("IDENT:ISIS, v%s MAQ, SN:MAC-%s;STATE:RUNNING;HWREV:%u;\n",
                              ISIS_FW_VERSION, netMacSerial(), (unsigned)isisHwRev());
            } else if (strncmp(buf, "OTAPW", 5) == 0) {
                // OTA-Passwort fuer espota (FCC-IP-Flash) anzeigen oder setzen:
                //   OTAPW            -> Passwort anzeigen
                //   OTAPW <passwort> -> Passwort setzen (landet in NVS)
                // Nur ueber die serielle Konsole - nie ueber das Netz.
                if (buf[5] == ' ') {
                    String neu = String(buf + 6);
                    neu.trim();
                    if (netSetOtaPassword(neu)) {
                        Serial.printf("[isis] OTAPW gesetzt: %s\n", neu.c_str());
                    } else {
                        Serial.println("[isis] OTAPW: Passwort leer, nichts geaendert");
                    }
                } else {
                    Serial.printf("[isis] OTAPW %s\n", netOtaPassword().c_str());
                }
            } else if (strncmp(buf, "FM", 2) == 0 && (buf[2] == '\0' || buf[2] == ' ')) {
                // Flash-Banner an/aus - der FCC kuendigt damit den USB-Flash an
                int an = (buf[2] == '\0') ? 1 : atoi(buf + 3);
                netSetOtaDemo(an ? 0 : -1);
                Serial.printf("FM=%d\n", an ? 1 : 0);
            } else if (strncmp(buf, "OTADEMO", 7) == 0) {
                // Nur fuer die Optik: Overlay mit festem Prozentwert zeigen,
                // ohne dass wirklich geflasht wird.  OTADEMO -1  schaltet aus.
                int pct = -1;
                if (sscanf(buf + 7, "%d", &pct) != 1) pct = -1;
                netSetOtaDemo(pct);
            } else if (strcmp(buf, "OTADBG") == 0) {
                // Diagnose: wie oft wurde das Fortschritts-Overlay gezeichnet?
                Serial.printf("[isis] OTA: %d Zeichen-Takte, onProgress %d x, zuletzt %d %%\n",
                              netOtaDrawCount(), netOtaProgressCalls(), netOtaLastPct());
            } else if (strncmp(buf, "HWREV", 5) == 0) {
                // Hardware-Revision anzeigen oder setzen: HWREV [1..9]
                int rev = 0;
                if (sscanf(buf + 5, "%d", &rev) == 1) {
                    if (isisSetHwRev((uint8_t)rev)) {
                        Serial.printf("[isis] HW-Rev %d gesetzt\n", rev);
                    } else {
                        Serial.println("[isis] HWREV: erlaubt 1..9");
                    }
                } else {
                    Serial.printf("[isis] HWREV %u\n", (unsigned)isisHwRev());
                }
            } else if ((strncmp(buf, "OTA", 3) == 0) && (buf[3] == '\0' || buf[3] == ' ')) {
                // OTA-Fenster direkt oeffnen: OTA [sekunden]
                // Der FCC schickt dafuer sonst den Frame CFG:ota=1.
                int sek = 0;
                if (sscanf(buf + 3, "%d", &sek) != 1 || sek <= 0) sek = 0;
                netOpenOtaWindow(millis(), sek > 0 ? (uint32_t)sek * 1000u : 0);
                Serial.printf("[isis] OTA offen: Port 3232, IP %s, Passwort %s\n",
                              WiFi.localIP().toString().c_str(),
                              netOtaPassword().c_str());
            } else if (strcmp(buf, "INFO") == 0) {
                Serial.printf("[portal] AP '%s'  IP %s\n", g_apName,
                              WiFi.softAPIP().toString().c_str());
            } else if (buf[0]) {
                Serial.println("[portal] Befehle: WIFI <ssid>|<passwort>   INFO   LSPOS <gsX> <locTop> [<fullPx>] [<locX>]   DIM <prozent> [<sekunden>]   OTA [sekunden]   OTAPW [<passwort>]   SHOT   SHOTS [faktor]   BLX <0-255>   BLP <pin> <0|1>");
            }
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }
}

// Gemeinsame Speicher-Routine fuer unsere Seite (/save, Felder ssid/pass) und
// fuer die ALTE WiFiManager-Seite (/wifisave, Felder s/p). Handys cachen die
// Portal-Seite; ohne diesen Weg landet so ein alter Aufruf im Nichts.
void doSave(const String &ssid, const String &pass, const char *source)
{
    Serial.printf("[portal] save (%s)  ssid='%s'  pass=%u Zeichen\n",
                  source, ssid.c_str(), (unsigned)pass.length());

    if (ssid.length() == 0) {
        g_server.sendHeader("Cache-Control", "no-store");
        g_server.send(200, "text/html; charset=utf-8",
                      "<body style='font-family:sans-serif;background:#111;color:#f88;padding:18px;font-size:18px'>"
                      "<h2>Kein Netzwerkname</h2><p>Bitte oben einen Namen eintippen oder ein Netz antippen.</p>"
                      "<p><a href='/' style='color:#3cf'>zurueck</a></p></body>");
        return;
    }
    if (pass.length() > 0 && pass.length() < 8) {
        g_server.sendHeader("Cache-Control", "no-store");
        g_server.send(200, "text/html; charset=utf-8",
                      "<body style='font-family:sans-serif;background:#111;color:#f88;padding:18px;font-size:18px'>"
                      "<h2>Passwort zu kurz</h2><p>WPA2 braucht mindestens 8 Zeichen.</p>"
                      "<p><a href='/' style='color:#3cf'>zurueck</a></p></body>");
        return;
    }

    // Speichern + verbinden. WiFi.persistent(true) schreibt die Daten in die NVS.
    applyCredentials(ssid.c_str(), pass.c_str());

    g_server.sendHeader("Cache-Control", "no-store");
    g_server.send(200, "text/html; charset=utf-8",
                  "<body style='font-family:sans-serif;background:#111;color:#eee;padding:18px;font-size:18px'>"
                  "<h2 style='color:#3cf'>Gespeichert</h2>"
                  "<p>Das Panel verbindet sich jetzt mit dem WLAN.</p>"
                  "<p>Bei Erfolg startet es neu und meldet sich beim FDC.</p>"
                  "<p style='color:#888'>Klappt es nicht, verbindet es sich nach 20 s wieder als "
                  "<b>ISIS-xxx</b> &mdash; dann diese Seite neu laden.</p></body>");
}

void handleSave()
{
    doSave(g_server.arg("ssid"), g_server.arg("pass"), "POST /save");
}

// Kompatibilitaet: alte WiFiManager-Seite -> GET/POST /wifisave mit s= und p=
void handleWifiSaveCompat()
{
    String ssid = g_server.hasArg("s") ? g_server.arg("s") : g_server.arg("ssid");
    String pass = g_server.hasArg("p") ? g_server.arg("p") : g_server.arg("pass");
    doSave(ssid, pass, "wifisave (alte Seite)");
}

// Alles, was die alte Seite sonst noch aufruft: freundlich beantworten statt 302.
void handleLegacy()
{
    Serial.printf("[portal] legacy %s -> ok\n", g_server.uri().c_str());
    g_server.sendHeader("Cache-Control", "no-store");
    g_server.send(200, "text/plain", "ok");
}

void handleWifiLegacy()
{
    // Die alte Seite holt hier ihre Netzliste ab -> auf unsere Seite umleiten.
    g_server.sendHeader("Location", "http://192.168.4.1/", true);
    g_server.send(302, "text/plain", "");
}

void handleInfo()
{
    char mac[18] = {0};
    uint8_t m[6] = {0};
    WiFi.macAddress(m);
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             m[0], m[1], m[2], m[3], m[4], m[5]);
    g_server.send(200, "text/plain",
                  String("ISIS FW ") + ISIS_FW_VERSION + "\nMAC " + mac + "\nAP-IP 192.168.4.1\n");
}

void handleNotFound()
{
    // Captive-Portal-Erkennung (Apple, Android, Windows) ruft beliebige Pfade auf.
    // Ein 302 auf die eigene Seite bringt den Mini-Browser dorthin.
    Serial.printf("[portal] %s %s -> 302 /\n",
                  g_server.method() == HTTP_POST ? "POST" : "GET",
                  g_server.uri().c_str());
    g_server.sendHeader("Location", "http://192.168.4.1/", true);
    g_server.send(302, "text/plain", "");
}

} // namespace

bool portalRunSetup(const char *apName, void (*onApUp)())
{
    WiFi.persistent(true);
    WiFi.mode(WIFI_AP_STA);
    // Ohne das versucht die STA-Seite dauernd, sich mit gespeicherten (falschen)
    // Daten zu verbinden -> Scanning + Kanalwechsel -> der AP ist am Handy
    // zeitweise nicht sichtbar. Die Credentials bleiben dabei erhalten.
    WiFi.disconnect(false, false);
    delay(100);

    // WICHTIG: softAPConfig() NICHT aufrufen. Die Standardwerte sind bereits
    // 192.168.4.1/24 mit laufendem DHCP-Server; ein eigener softAPConfig-Aufruf
    // vor softAP() laesst den DHCP-Server in dieser Core-Version nicht starten.
    // Symptom am Handy: WLAN verbunden, aber APIPA (169.254.x.x) statt 192.168.4.x.
    snprintf(g_apName, sizeof(g_apName), "%s", apName);
    if (!WiFi.softAP(apName, "", 1, 0)) {   // offen, Kanal 1 (wie WiFiManager)
        Serial.println("[portal] SoftAP konnte nicht gestartet werden");
        return false;
    }
    delay(200);
    const IPAddress apIp = WiFi.softAPIP();
    Serial.printf("[portal] SoftAP '%s' auf %s  (Clients duerfen 192.168.4.x bekommen)\n",
                  apName, apIp.toString().c_str());
    if ((uint32_t)apIp == 0) {
        Serial.println("[portal] WARNUNG: AP hat keine IP - DHCP wird nicht laufen");
    }

    // DNS-Catch-All: jede Namensauflösung landet auf dem Panel -> Captive Portal
    g_dns.start(53, "*", apIp);

    g_server.on("/",        HTTP_GET,  handleRoot);
    g_server.on("/setup",   HTTP_GET,  handleRoot);   // frische URL, umgeht den Handy-Cache
    g_server.on("/use",     HTTP_GET,  handleUse);
    g_server.on("/scan",    HTTP_GET,  handleScan);
    g_server.on("/save",    HTTP_POST, handleSave);
    g_server.on("/info",    HTTP_GET,  handleInfo);
    // Alt-Endpunkte der WiFiManager-Seite (Cache auf dem Handy!)
    g_server.on("/wifisave",    HTTP_GET,  handleWifiSaveCompat);
    g_server.on("/wifisave",    HTTP_POST, handleWifiSaveCompat);
    g_server.on("/wifi",        HTTP_GET,  handleWifiLegacy);
    g_server.on("/wifinotscan", HTTP_GET,  handleWifiLegacy);
    g_server.on("/param",       HTTP_ANY,  handleLegacy);
    g_server.on("/paramsave",   HTTP_ANY,  handleLegacy);
    g_server.on("/statuschange",HTTP_ANY,  handleLegacy);
    g_server.on("/close",       HTTP_ANY,  handleLegacy);
    g_server.on("/erase",       HTTP_ANY,  handleLegacy);
    g_server.onNotFound(handleNotFound);
    g_server.begin();
    Serial.println("[portal] HTTP-Server auf Port 80 bereit");

    if (onApUp) onApUp();

    const uint32_t t0 = millis();
    while (true) {
        g_server.handleClient();
        g_dns.processNextRequest();
        isisSerialPoll();
        esp_task_wdt_reset();
        delay(2);

        if (g_connecting && WiFi.status() == WL_CONNECTED) {
            Serial.printf("[portal] verbunden: %s  IP %s\n",
                          g_lastSsid, WiFi.localIP().toString().c_str());
            delay(1200);            // Antwortseite noch ausliefern
            return true;
        }
        if (g_connecting && (uint32_t)(millis() - g_connectStart) > 20000UL) {
            Serial.println("[portal] Verbindung fehlgeschlagen - Portal bleibt offen");
            g_connecting = false;
        }
        // Alle 10 s Status: zeigt, ob das Handy wirklich am AP haengt
        static uint32_t lastStatus = 0;
        static int lastClients = -1;
        if ((uint32_t)(millis() - lastStatus) > 10000UL) {
            lastStatus = millis();
            const int clients = WiFi.softAPgetStationNum();
            Serial.printf("[portal] AP '%s' Clients=%d  Kanal=%d  STA=%d %s\n",
                          g_apName, clients, WiFi.channel(), (int)WiFi.status(),
                          WiFi.status() == WL_CONNECTED ? "(verbunden)" : "(keine Verbindung)");
            if (clients != lastClients) {
                Serial.printf("[portal] Client-Aenderung: %d -> %d\n", lastClients, clients);
                lastClients = clients;
            }
        }

        if ((uint32_t)(millis() - t0) > 600000UL) {
            Serial.println("[portal] Setup-Timeout (10 min)");
            return false;
        }
    }
}

#endif // ISIS_ESP32

// Oeffentlicher Zugang fuer main.cpp: die eigentliche Arbeit liegt im anonymen
// Namespace oben (serialPollImpl), damit keine Namenskollision mit Bibliotheken
// entsteht.
void isisSerialPoll()
{
    serialPollImpl();
}
