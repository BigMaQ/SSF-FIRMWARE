#!/usr/bin/env python3
"""WLAN-Einrichtung für kopflose Anzeige-Clients — über das Handy.

Der Client hat keine Tastatur und keine Maus.  Dieses Werkzeug entscheidet beim
Start und dann zyklisch, wie der Client ins Netz kommt:

    LAN da?      → LAN nutzen, nichts weiter tun
    sonst WLAN?  → gespeichertes Profil verbinden
    sonst        → Einrichtungsmodus: eigener Access Point + Web-Seite fürs Handy

Im Einrichtungsmodus läuft ein eigener AP (``FDC-Setup-<hostname>``) mit
DHCP/DNS über NetworkManager (``ipv4.method shared``) und einer kleinen Seite
auf Port 80.  Damit das Handy die Seite auch ohne Tippen findet, beantwortet ein
Dnsmasq-Snippet jede DNS-Anfrage mit unserer Adresse und die bekannten
Captive-Portal-Proben (Android ``/generate_204``, iOS ``/hotspot-detect.html``)
werden umgeleitet — dann meldet das Handy „Netzwerk anmelden" von allein.

SSID und Adresse stehen zusätzlich auf dem Bildschirm
(``/run/fdc-wlan-setup.json`` → ``tools/panel_viewer.py``), weil man ohne
Beschriftung nicht weiß, welcher AP gemeint ist.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Optional

STANDARD_STATE = "/run/fdc-wlan-setup.json"
STANDARD_SSID_PRAEFIX = "FDC-Setup"
AP_ADRESSE = "192.168.4.1/24"
AP_IP = "192.168.4.1"
AP_NAME = "fdc-setup-ap"
#: So lange ohne LAN/WLAN bis der Einrichtungsmodus startet.
KARENZ_S = 8.0
#: Prüfintervall im Hintergrund.
PRUEF_S = 30.0

#: Captive-Portal-Proben: hier antworten wir mit einer Umleitung auf die Seite.
PORTAL_PFADE = ("/generate_204", "/gen_204", "/hotspot-detect.html",
                "/connecttest.txt", "/ncsi.txt", "/canonical.html",
                "/success.txt", "/redirect", "/captive")


# ---------------------------------------------------------------------------
# nmcli / Netz-Helfer
# ---------------------------------------------------------------------------
def nmcli(*args: str, timeout: float = 25.0) -> tuple[int, str]:
    """``nmcli`` aufrufen und (Rückgabewert, Ausgabe) liefern."""
    befehl = ["nmcli", *args]
    try:
        ergebnis = subprocess.run(befehl, capture_output=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, f"{type(exc).__name__}: {exc}"
    aus = (ergebnis.stdout or b"").decode("utf-8", "replace")
    fehler = (ergebnis.stderr or b"").decode("utf-8", "replace")
    return ergebnis.returncode, (aus + fehler).strip()


def terse_felder(text: str, trenner: str = ":") -> list[list[str]]:
    """``nmcli -t`` zerlegen — Maskierung mit Backslash beachten.

    Ein SSID wie ``Cafe:Rita`` kommt als ``Cafe\\:Rita`` an; naiv zu trennen
    zerlegt den Namen und macht die Liste unbrauchbar.
    """
    zeilen = []
    for roh in text.splitlines():
        if not roh.strip():
            continue
        felder, aktuell, i = [], "", 0
        while i < len(roh):
            zeichen = roh[i]
            if zeichen == "\\" and i + 1 < len(roh):
                aktuell += roh[i + 1]
                i += 2
                continue
            if zeichen == trenner:
                felder.append(aktuell)
                aktuell = ""
            else:
                aktuell += zeichen
            i += 1
        felder.append(aktuell)
        zeilen.append(felder)
    return zeilen


def netz_karten() -> list[str]:
    """Alle Netzwerkkarten (ohne loopback)."""
    basis = Path("/sys/class/net")
    if not basis.exists():
        return []
    return sorted(e.name for e in basis.iterdir() if e.name != "lo")


def lan_bereit() -> bool:
    """Kabel liegt an **und** es gibt eine IPv4-Adresse."""
    for name in netz_karten():
        if name.startswith(("en", "eth")):
            try:
                carrier = (Path("/sys/class/net") / name / "carrier").read_text().strip()
            except OSError:
                continue
            if carrier == "1" and _hat_ipv4(name):
                return True
    return False


def wlan_karte() -> str:
    """Name der WLAN-Karte (leer, wenn es keine gibt)."""
    for name in netz_karten():
        if (Path("/sys/class/net") / name / "wireless").exists() or \
                (Path("/sys/class/net") / name / "phy80211").exists():
            return name
    return ""


def _hat_ipv4(geraet: str) -> bool:
    try:
        ergebnis = subprocess.run(["ip", "-4", "-o", "addr", "show", "dev", geraet],
                                  capture_output=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return b"inet " in (ergebnis.stdout or b"")


def _hat_standardroute(geraet: str) -> bool:
    """Führt die Standardroute über dieses Gerät? Nur dann ist es ein Uplink.

    ``_hat_ipv4`` allein reicht NICHT: im AP-Modus hat die WLAN-Karte die eigene
    Adresse 192.168.4.1. Damit hielt sich der Client für „verbunden", der Wächter
    schaltete den Einrichtungs-AP ab — der Hotspot verschwand nach spätestens
    30 s von selbst (Feldbefund 18.11., 192.168.20.65).
    """
    try:
        lauf = subprocess.run(["ip", "-4", "route", "show", "default", "dev", geraet],
                              capture_output=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return bool((lauf.stdout or b"").strip())


def wlan_verbunden(karte: Optional[str] = None) -> bool:
    karte = karte or wlan_karte()
    if not karte:
        return False
    if ap_laeuft():                      # wir sind der Hotspot, nicht der Client
        return False
    if not _hat_standardroute(karte):
        return False
    rc, aus = nmcli("-t", "-f", "DEVICE,STATE", "device", "status")
    if rc != 0:
        return True                      # Route steht; nmcli-Abfrage ist Beiwerk
    for zeile in terse_felder(aus):
        if len(zeile) >= 2 and zeile[0] == karte:
            return zeile[1] == "connected"
    return True


def gespeicherte_wlan_profile() -> list[str]:
    rc, aus = nmcli("-t", "-f", "NAME,TYPE", "connection", "show")
    if rc != 0:
        return []
    return [zeile[0] for zeile in terse_felder(aus)
            if len(zeile) >= 2 and zeile[1] in ("802-11-wireless", "wifi")]


def profil_verbinden(profil: str, warte_s: float = 25.0) -> bool:
    rc, _ = nmcli("-w", str(int(warte_s)), "connection", "up", profil)
    return rc == 0 and wlan_verbunden()


def netze_scannen() -> list[dict]:
    """WLAN-Liste für die Seite (SSID, Signal, Sicherheit)."""
    karte = wlan_karte()
    if not karte:
        return []
    nmcli("device", "wifi", "rescan", timeout=12)
    rc, aus = nmcli("-t", "-f", "SSID,SIGNAL,SECURITY", "device", "wifi", "list",
                    timeout=15)
    if rc != 0:
        return []
    netze, gesehen = [], set()
    for zeile in terse_felder(aus):
        if len(zeile) < 3 or not zeile[0].strip():
            continue
        ssid = zeile[0].strip()
        if ssid in gesehen:
            continue
        gesehen.add(ssid)
        try:
            signal = int(zeile[1] or 0)
        except ValueError:
            signal = 0
        netze.append({"ssid": ssid, "signal": signal,
                      "sicherheit": zeile[2].strip() or "offen"})
    netze.sort(key=lambda n: -n["signal"])
    return netze


def ap_ssid() -> str:
    kurz = "".join(zeichen for zeichen in _hostname() if zeichen.isalnum() or zeichen in "-_")[:16]
    return f"{STANDARD_SSID_PRAEFIX}-{kurz or 'client'}"


def _hostname() -> str:
    try:
        return os.uname().nodename
    except OSError:                                        # pragma: no cover
        return "client"


PORTAL_VERZEICHNIS = "/etc/NetworkManager/dnsmasq-shared.d"
PORTAL_DATEI = "fdc-portal.conf"


def portal_dns_einrichten(adresse: str = AP_ADRESSE) -> bool:
    """Handy soll den Hinweis „Anmeldung im Netzwerk nötig" von selbst oeffnen.

    NetworkManagers dnsmasq beantwortet im AP-Modus normale DNS-Anfragen. Die
    Erreichbarkeitspruefungen der Handys (``connectivitycheck.gstatic.com`` bei
    Android, ``captive.apple.com`` bei iOS) bekommen dann eine echte Antwort und
    das Geraet denkt, alles sei in Ordnung — der Nutzer muss die Adresse von Hand
    eintippen. Der DNS-Drop-in beantwortet **jede** Anfrage mit der Box und
    schickt die Portal-Adresse per DHCP mit (Option 114 = RFC 8910, 160 = aeltere
    Android-Geraete). Dann erscheint die Einrichtungsseite wie bei jedem Hotel-WLAN.
    """
    ip = str(adresse).split("/")[0]
    inhalt = ("# FDC-Einrichtung: alle Namensaufloesungen auf die Box zeigen,\n"
              "# damit Handys die Einrichtungsseite selbst oeffnen.\n"
              f"address=/#/{ip}\n"
              f"dhcp-option=114,http://{ip}/\n"
              f"dhcp-option=160,http://{ip}/\n")
    pfad = Path(PORTAL_VERZEICHNIS) / PORTAL_DATEI
    try:
        pfad.parent.mkdir(parents=True, exist_ok=True)
        if not pfad.exists() or pfad.read_text(encoding="utf-8") != inhalt:
            pfad.write_text(inhalt, encoding="utf-8")
            os.chmod(pfad, 0o644)
        return True
    except OSError as exc:
        print(f"[wlan-setup] Portal-DNS nicht eingerichtet: {exc}", file=sys.stderr,
              flush=True)
        return False


def ap_starten(ssid: Optional[str] = None, passwort: Optional[str] = None) -> tuple[bool, str]:
    """Einrichtungs-AP hochziehen (NetworkManager, DHCP/DNS inklusive).

    Ohne Passphrase läuft der AP **offen**: das Einrichten lebt nur, solange
    kein LAN da ist, und Handys quittieren Sonderfälle gern mit „Passwort zu
    lang oder zu kurz" statt zu verbinden.  Ein Passwort nur, wenn ausdrücklich
    eines gesetzt ist (Parameter oder ``FDC_SETUP_PSK``).
    """
    karte = wlan_karte()
    if not karte:
        return False, "keine WLAN-Karte gefunden"
    ssid = ssid or ap_ssid()
    psk = (passwort if passwort is not None
           else os.environ.get("FDC_SETUP_PSK", "")).strip()
    portal_dns_einrichten()          # damit das Handy die Seite selbst öffnet
    nmcli("connection", "delete", AP_NAME, timeout=12)      # Reste aufräumen
    # Drei Anläufe, vom ausführlichen zum schlichten Aufruf. Grund: welche
    # Eigenschaft eine WLAN-Firmware (hier mwifiex/mlan0) ablehnt, ist von
    # Kernel zu Kernel verschieden — der Dienst darf daran nicht scheitern,
    # denn der AP ist auf einer Box ohne Netz der EINZIGE Zugang.
    basis = [
        "connection", "add", "type", "wifi", "ifname", karte, "con-name", AP_NAME,
        "autoconnect", "no", "ssid", ssid, "802-11-wireless.mode", "ap",
        "802-11-wireless.band", "bg",
    ]
    sicherheit = (["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", psk]
                  if psk else ["wifi-sec.key-mgmt", "none"])
    ip = ["ipv4.method", "shared", "ipv4.addresses", AP_ADRESSE, "ipv6.method", "ignore"]
    # Reihenfolge nach Befund (gemessen auf mlan0 / mwifiex, 18.11.):
    # ``wifi-sec.key-mgmt none`` laesst NetworkManager trotzdem ein
    # Sicherheitsobjekt anlegen und verlangt dann WEP-Schluessel
    # („Secrets were required" / „password for 802-11-wireless-security.wep-key0
    # not given") — der AP kommt damit NIE hoch. Ein offener AP entsteht nur,
    # wenn man den Sicherheitsblock ganz weglaesst.
    offen = basis + ip
    versuche = ([offen,
                 basis + sicherheit + ip,
                 basis + ["802-11-wireless.powersave", "2"] + sicherheit + ip]
                if not psk else
                [basis + ["802-11-wireless.powersave", "2"] + sicherheit + ip,
                 basis + sicherheit + ip])
    meldungen: list[str] = []
    for nummer, argumente in enumerate(versuche, start=1):
        nmcli("connection", "delete", AP_NAME, timeout=12)
        rc, aus = nmcli(*argumente, timeout=25)
        if rc != 0:
            meldungen.append(f"Versuch {nummer} anlegen fehlgeschlagen: {aus.strip()[:160]}")
            continue
        rc, aus = nmcli("-w", "30", "connection", "up", AP_NAME, timeout=45)
        if rc == 0:
            if nummer > 1:
                print(f"[wlan-setup] AP im Versuch {nummer} gestartet", flush=True)
            return True, ssid
        meldungen.append(f"Versuch {nummer} starten fehlgeschlagen: {aus.strip()[:160]}")
    # Letzte Rettung: NetworkManagers eigener Hotspot-Kurzbefehl. Er setzt die
    # Verbindung anders zusammen und kommt mit Karten durch, die `mode ap`
    # ablehnen.
    nmcli("connection", "delete", AP_NAME, timeout=12)
    rc, aus = nmcli("device", "wifi", "hotspot", "ifname", karte, "ssid", ssid,
                    "con-name", AP_NAME, timeout=40)
    if rc == 0:
        print("[wlan-setup] AP über den Hotspot-Kurzbefehl gestartet", flush=True)
        return True, ssid
    meldungen.append(f"hotspot fehlgeschlagen: {aus.strip()[:160]}")
    return False, " | ".join(meldungen)


def ap_stoppen() -> None:
    nmcli("connection", "down", AP_NAME, timeout=20)
    nmcli("connection", "delete", AP_NAME, timeout=20)


def ap_laeuft() -> bool:
    rc, aus = nmcli("-t", "-f", "NAME", "connection", "show", "--active")
    if rc != 0:
        return False
    return any(zeile and zeile[0] == AP_NAME for zeile in terse_felder(aus))


def verbinden_und_zurueck(ssid: str, passwort: str,
                          karte: Optional[str] = None) -> tuple[bool, str]:
    """Wie ``verbinden``, aber ohne die Box offline zurueckzulassen.

    Der AP muss fuer den Verbindungsversuch weichen (eine Karte kann nicht
    gleichzeitig AP und Station sein). Scheitert der Versuch, wird der AP sofort
    wieder aufgebaut — sonst bleibt ein Client zurueck, den niemand mehr
    erreicht: kein LAN, kein WLAN, kein Hotspot.
    """
    ok, meldung = verbinden(ssid, passwort, karte)
    if ok:
        return True, meldung
    print(f"[wlan-setup] Verbinden mit \"{ssid}\" fehlgeschlagen: {meldung}",
          file=sys.stderr, flush=True)
    wieder, ap_meldung = ap_starten()
    if wieder:
        meldung = f"{meldung} — Einrichtungs-AP ist wieder da ({ap_meldung})"
    else:
        meldung = f"{meldung} — AP-Neustart fehlgeschlagen: {ap_meldung}"
    # Auf dem Panel sichtbar machen: gleicher Weg wie beim AP-Fehler.
    try:
        zustand = zustand_lesen()
        zustand.update({"aktiv": True, "stand": "setup", "fehler": meldung,
                        "ssid": ap_ssid(), "url": f"http://{AP_IP}"})
        zustand_schreiben(STANDARD_STATE, zustand)
    except Exception:                       # noqa: BLE001 - Anzeige ist Beiwerk
        pass
    return False, meldung


def verbinden(ssid: str, passwort: str, karte: Optional[str] = None) -> tuple[bool, str]:
    """Mit einem WLAN verbinden und als Profil merken."""
    karte = karte or wlan_karte()
    if not ssid.strip():
        return False, "keine SSID angegeben"
    if not karte:
        return False, "keine WLAN-Karte gefunden"
    # Der AP muss weg, sonst kann die Karte nicht als Station arbeiten.
    if ap_laeuft():
        ap_stoppen()
        time.sleep(2)
    args = ["-w", "40", "device", "wifi", "connect", ssid]
    if passwort:
        args += ["password", passwort]
    if karte:
        args += ["ifname", karte]
    rc, aus = nmcli(*args, timeout=55)
    if rc != 0:
        return False, (aus or "Verbindung fehlgeschlagen")[:300]
    # rc == 0 heisst nur „Assoziation und Aktivierung durch". Ohne Adresse ist
    # die Box im WLAN aber nicht erreichbar — real passiert: Seite meldete
    # Erfolg, Box hatte keine IP, und weil der AP schon weg war, kam niemand
    # mehr dran.
    for _ in range(12):                     # bis ~24 s auf DHCP warten
        if _hat_ipv4(karte):
            return True, f"verbunden mit {ssid}"
        time.sleep(2)
    return False, ("Verbunden, aber keine Adresse bekommen (DHCP) — "
                   "Passwort/Netz prüfen")


# ---------------------------------------------------------------------------
# Entscheidung + Zustandsdatei
# ---------------------------------------------------------------------------
def entscheide(lan: bool, wlan: bool) -> str:
    """``lan`` | ``wlan`` | ``setup`` — die Reihenfolge ist der ganze Trick."""
    if lan:
        return "lan"
    if wlan:
        return "wlan"
    return "setup"


def zustand_schreiben(pfad: str, daten: dict) -> None:
    """Zustandsdatei atomar schreiben, weltlesbar (der Viewer läuft als Nutzer)."""
    ziel = Path(pfad)
    daten = dict(daten)
    daten["seit"] = int(time.time())
    try:
        ziel.parent.mkdir(parents=True, exist_ok=True)
        tmp = ziel.with_suffix(ziel.suffix + ".tmp")
        tmp.write_text(json.dumps(daten, indent=2, ensure_ascii=False) + "\n",
                       encoding="utf-8")
        os.chmod(tmp, 0o644)
        tmp.replace(ziel)
    except OSError:
        pass


def zustand_lesen(pfad: str = STANDARD_STATE) -> dict:
    try:
        return json.loads(Path(pfad).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


# ---------------------------------------------------------------------------
# Web-Seite fürs Handy
# ---------------------------------------------------------------------------
def seite_html(ssid: str, adresse: str, stand: str = "setup") -> str:
    """Einrichtungsseite — bewusst ohne externe Ressourcen (kein Internet im AP)."""
    return f"""<!DOCTYPE html>
<html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Anzeige-Client einrichten</title>
<style>
:root{{color-scheme:dark}}
body{{margin:0;padding:18px;font:16px/1.45 system-ui,-apple-system,sans-serif;
background:#14161a;color:#e6e9ef}}
h1{{font-size:20px;margin:0 0 4px}}
p.lead{{color:#9aa3b0;margin:0 0 16px}}
.card{{background:#1d2026;border:1px solid #2c313a;border-radius:10px;padding:14px;margin:0 0 14px}}
.step{{display:flex;gap:10px;margin:6px 0}}
.num{{flex:0 0 24px;height:24px;border-radius:12px;background:#2b6cb0;color:#fff;
text-align:center;font-weight:700;font-size:13px;line-height:24px}}
code{{background:#11151b;padding:2px 6px;border-radius:5px;font-size:15px}}
input,select,button{{font:inherit;width:100%;box-sizing:border-box;padding:11px;
border-radius:8px;border:1px solid #333a45;background:#11151b;color:#e6e9ef;margin:6px 0}}
button{{background:#2b6cb0;border-color:#2b6cb0;font-weight:600}}
button:disabled{{opacity:.5}}
.ok{{color:#4ade80}} .fehler{{color:#f87171}}
ul{{list-style:none;padding:0;margin:8px 0}}
li{{padding:10px;border:1px solid #2c313a;border-radius:8px;margin-bottom:8px;
cursor:pointer;display:flex;justify-content:space-between;gap:8px}}
li.sel{{border-color:#2b6cb0;background:#1b2735}}
small{{color:#9aa3b0}}
</style></head><body>
<h1>Anzeige-Client einrichten</h1>
<p class="lead">Status: <b>{stand}</b> &middot; verbunden mit <code>{ssid}</code></p>

<div class="card">
<div class="step"><div class="num">1</div><div>WLAN auswählen und Passwort eingeben.</div></div>
<div class="step"><div class="num">2</div><div>Der Client verbindet sich und startet die Anzeige neu.</div></div>
<div class="step"><div class="num">3</div><div>Danach ist diese Seite nicht mehr nötig.</div></div>
<small>Kabel (LAN) geht immer: einstecken, fertig — dann wird WLAN nicht gebraucht.</small>
</div>

<div class="card">
<label for="ssid">Netzwerk</label>
<select id="ssid"></select>
<input id="manuell" placeholder="oder SSID von Hand eingeben">
<label for="pass">Passwort</label>
<input id="pass" type="password" autocomplete="current-password" placeholder="WLAN-Passwort">
<button id="los">Verbinden</button>
<div id="meldung"></div>
</div>

<div class="card">
<small>Netze aus der Suche beim Start (im Einrichtungsmodus kann der Client nicht
scannen — Antippen setzt den Namen, oder oben von Hand eintippen):</small>
<ul id="netze"><li><small>suche …</small></li></ul>
<button id="neu" style="background:#2c313a;border-color:#333a45">Neu suchen</button>
</div>

<p><small>Adresse dieser Seite: <code>{adresse}</code></small></p>

<script>
const ssidFeld = document.getElementById('ssid');
const manuell = document.getElementById('manuell');
const meldung = document.getElementById('meldung');
function zeige(text, klasse) {{
  meldung.textContent = text; meldung.className = klasse || '';
}}
async function scan() {{
  const liste = document.getElementById('netze');
  liste.innerHTML = '<li><small>suche …</small></li>';
  const antwort = await fetch('/api/scan');
  const netze = await antwort.json();
  ssidFeld.innerHTML = '';
  liste.innerHTML = '';
  if (!netze.length) {{ liste.innerHTML = '<li><small>nichts gefunden</small></li>'; return; }}
  for (const netz of netze) {{
    const wahl = document.createElement('option');
    wahl.value = netz.ssid; wahl.textContent = netz.ssid + ' (' + netz.signal + '%)';
    ssidFeld.appendChild(wahl);
    const punkt = document.createElement('li');
    punkt.innerHTML = '<span>' + netz.ssid + '<br><small>' + netz.sicherheit +
                      '</small></span><b>' + netz.signal + '%</b>';
    punkt.onclick = () => {{ ssidFeld.value = netz.ssid; manuell.value = netz.ssid; }};
    liste.appendChild(punkt);
  }}
}}
document.getElementById('neu').onclick = scan;
document.getElementById('los').onclick = async () => {{
  const ssid = (manuell.value || ssidFeld.value || '').trim();
  const pass = document.getElementById('pass').value;
  if (!ssid) {{ zeige('Bitte ein Netz wählen.', 'fehler'); return; }}
  zeige('verbinde mit ' + ssid + ' …');
  document.getElementById('los').disabled = true;
  const antwort = await fetch('/api/connect', {{
    method: 'POST', headers: {{'Content-Type': 'application/json'}},
    body: JSON.stringify({{ssid: ssid, pass: pass}})
  }});
  const ergebnis = await antwort.json();
  document.getElementById('los').disabled = false;
  zeige(ergebnis.meldung, ergebnis.ok ? 'ok' : 'fehler');
}};
scan();
</script>
</body></html>
"""


def ist_portal_probe(pfad: str) -> bool:
    return pfad in PORTAL_PFADE


# ---------------------------------------------------------------------------
# HTTP-Server
# ---------------------------------------------------------------------------
def baue_handler(state_pfad: str, ssid: str,
                 adresse: "str | Callable[[], str]",
                 verbinder: Callable[[str, str], tuple[bool, str]] = verbinden_und_zurueck,
                 scanner: Callable[[], list] = netze_scannen):
    """Handler bauen.  ``adresse`` darf eine Funktion sein (Adresse erst nach dem
    Binden des Ports bekannt — so bleibt der Test ohne feste IP möglich)."""

    class Handler(BaseHTTPRequestHandler):
        server_version = "FDC-WLAN-Setup/1.0"

        def _ziel(self) -> str:
            wert = adresse() if callable(adresse) else adresse
            return f"http://{wert}/"

        def _send(self, code: int, rumpf: bytes = b"", art: str = "text/html; charset=utf-8",
                  zusatz: Optional[dict] = None) -> None:
            self.send_response(code)
            self.send_header("Content-Type", art)
            self.send_header("Content-Length", str(len(rumpf)))
            self.send_header("Cache-Control", "no-store")
            for name, wert in (zusatz or {}).items():
                self.send_header(name, wert)
            self.end_headers()
            if rumpf:
                self.wfile.write(rumpf)

        def _json(self, daten: dict | list, code: int = 200) -> None:
            self._send(code, json.dumps(daten, ensure_ascii=False).encode("utf-8"),
                       "application/json; charset=utf-8")

        def do_GET(self) -> None:  # noqa: N802 - HTTP
            pfad = self.path.split("?")[0].rstrip("/") or "/"
            if ist_portal_probe(pfad):
                # Handy fragt „Internet da?" — wir leiten auf die Einrichtungsseite.
                self._send(302, b"", "text/html", {"Location": self._ziel()})
                return
            if pfad == "/api/scan":
                # Im AP-Modus ist kein echter Scan möglich (eine Karte, ein Kanal):
                # dann die beim Start gemerkte Liste liefern.
                gemerkt = zustand_lesen(state_pfad).get("netze") or []
                self._json(gemerkt or scanner())
                return
            if pfad == "/api/status":
                self._json(zustand_lesen(state_pfad) or {"stand": "setup", "ssid": ssid})
                return
            if pfad in ("/", "/index.html"):
                stand = str(zustand_lesen(state_pfad).get("stand") or "setup")
                self._send(200, seite_html(ssid, self._ziel().rstrip("/"), stand).encode("utf-8"))
                return
            self._send(302, b"", "text/html", {"Location": self._ziel()})

        def do_POST(self) -> None:  # noqa: N802 - HTTP
            pfad = self.path.split("?")[0].rstrip("/")
            if pfad != "/api/connect":
                self._json({"ok": False, "meldung": "unbekannter Pfad"}, 404)
                return
            laenge = int(self.headers.get("Content-Length") or 0)
            try:
                daten = json.loads(self.rfile.read(laenge).decode("utf-8", "replace") or "{}")
            except ValueError:
                self._json({"ok": False, "meldung": "Anfrage nicht lesbar"}, 400)
                return
            gewaehlt = str(daten.get("ssid") or "")
            # Ohne diese Zeile ist ein gescheiterter Verbindungsversuch spaeter
            # nicht nachvollziehbar: der nmcli-Text ging nur an das Handy, und
            # das ist nach dem AP-Abbau weg. Passwort wird NIE geloggt.
            print(f"[wlan-setup] Verbindungsversuch zu \"{gewaehlt}\"", file=sys.stderr,
                  flush=True)
            ok, meldung = verbinder(gewaehlt, str(daten.get("pass") or ""))
            print(f"[wlan-setup] Ergebnis für \"{gewaehlt}\": "
                  f"{'ok' if ok else 'gescheitert'} — {meldung}", file=sys.stderr,
                  flush=True)
            if ok:
                zustand_schreiben(state_pfad, {"aktiv": False, "stand": "wlan",
                                               "ssid": daten.get("ssid"), "url": ""})
            self._json({"ok": ok, "meldung": meldung})

        def log_message(self, format: str, *args) -> None:  # noqa: A002
            sys.stderr.write("[wlan-setup] %s - %s\n" % (self.address_string(), format % args))

    return Handler


# ---------------------------------------------------------------------------
# Hintergrund-Prüfung und Start
# ---------------------------------------------------------------------------
class Waechter(threading.Thread):
    """Hält den Zustand aktuell: LAN → WLAN → Einrichtungsmodus."""

    def __init__(self, state_pfad: str, ssid: str, intervall: float = PRUEF_S) -> None:
        super().__init__(daemon=True, name="WLAN-Waechter")
        self.state_pfad = state_pfad
        self.ssid = ssid
        self.intervall = intervall
        self._laufen = True
        self.letzter_stand = ""
        self.letzte_netze: list[dict] = []

    def pruefen(self) -> str:
        # Prüfplatz-Schalter: am Schreibtisch liegt immer ein Kabel, sonst käme man
        # nie in den Einrichtungsmodus.  FDC_WLAN_ERZWINGEN=setup|lAN|wlan.
        erzwungen = os.environ.get("FDC_WLAN_ERZWINGEN", "").strip().lower()
        if erzwungen in ("setup", "lan", "wlan"):
            stand = erzwungen
            lan, wlan = erzwungen == "lan", erzwungen == "wlan"
        else:
            lan = lan_bereit()
            wlan = wlan_verbunden() if not lan else False
            stand = entscheide(lan, wlan)
        if stand == "lan":
            if ap_laeuft():
                ap_stoppen()
            zustand_schreiben(self.state_pfad, {"aktiv": False, "stand": "lan",
                                                "ssid": "", "url": ""})
        elif stand == "wlan":
            if ap_laeuft():
                ap_stoppen()
            zustand_schreiben(self.state_pfad, {"aktiv": False, "stand": "wlan",
                                                "ssid": "", "url": f"http://{AP_IP}"})
        else:
            fehler = ""
            if not ap_laeuft():
                # WICHTIG: erst scannen, dann den AP hochziehen.  Im AP-Modus kann
                # dieselbe Karte nicht mehr scannen — die Netze, die das Handy zur
                # Auswahl braucht, müssen vorher in die Zustandsdatei.
                try:
                    # Nur scannen, wenn wir noch keine Liste haben: im AP-Modus
                    # kann dieselbe Karte nicht scannen und liefert 0 Netze
                    # (so kam auf 192.168.20.65 eine leere Auswahlliste an).
                    # Die alte Liste aus der Zustandsdatei ist besser als eine leere.
                    if not self.letzte_netze:
                        self.letzte_netze = netze_scannen()
                except Exception as exc:                    # noqa: BLE001
                    self.letzte_netze = []
                    fehler = f"Scan: {type(exc).__name__}: {exc}"
                if not self.letzte_netze:
                    alt = zustand_lesen(self.state_pfad).get("netze")
                    if isinstance(alt, list) and alt:
                        self.letzte_netze = alt
                try:
                    ok, meldung = ap_starten(self.ssid)
                except Exception as exc:                    # noqa: BLE001
                    ok, meldung = False, f"{type(exc).__name__}: {exc}"
                if not ok:
                    fehler = meldung
                    print(f"[wlan-setup] AP nicht gestartet: {meldung}", file=sys.stderr,
                          flush=True)
            # Der Grund MUSS in die Zustandsdatei: ohne SSH ist der Bildschirm die
            # einzige Auskunft. Vorher stand er nur im Journal — vor der Box
            # stehend sah man dann bloß „kein Netz".
            zustand_schreiben(self.state_pfad, {"aktiv": True, "stand": "setup",
                                                "ssid": self.ssid,
                                                "url": f"http://{AP_IP}",
                                                "netze": self.letzte_netze[:24],
                                                "fehler": fehler,
                                                "hinweis": "LAN nutzen oder WLAN einrichten"})
        self.letzter_stand = stand
        return stand

    def run(self) -> None:
        time.sleep(KARENZ_S)
        while self._laufen:
            try:
                self.pruefen()
            except Exception as exc:                        # noqa: BLE001 - nie sterben
                print(f"[wlan-setup] Prüfung fehlgeschlagen: {exc}", file=sys.stderr, flush=True)
            time.sleep(self.intervall)

    def stoppen(self) -> None:
        self._laufen = False


def main(argv: Optional[list[str]] = None) -> int:
    zerleger = argparse.ArgumentParser(description="WLAN-Einrichtung für Anzeige-Clients")
    zerleger.add_argument("--state", default=STANDARD_STATE, help="Pfad der Zustandsdatei")
    zerleger.add_argument("--port", type=int, default=80)
    zerleger.add_argument("--ssid", default="", help="SSID des Einrichtungs-AP")
    zerleger.add_argument("--adresse", default=AP_IP, help="Adresse, unter der die Seite läuft")
    zerleger.add_argument("--pruefen", action="store_true",
                          help="nur den Zustand bestimmen und ausgeben")
    args = zerleger.parse_args(argv)

    ssid = args.ssid or ap_ssid()
    if args.pruefen:
        stand = Waechter(args.state, ssid).pruefen()
        print(json.dumps({"stand": stand, "ssid": ssid, "adresse": args.adresse,
                          "wlan_karte": wlan_karte(), "lan": lan_bereit()},
                         ensure_ascii=False))
        return 0

    waechter = Waechter(args.state, ssid)
    waechter.start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port),
                                 baue_handler(args.state, ssid, args.adresse))
    print(f"[wlan-setup] Seite auf http://{args.adresse}:{args.port}  ssid={ssid}  "
          f"karte={wlan_karte() or 'keine'}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        waechter.stoppen()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
