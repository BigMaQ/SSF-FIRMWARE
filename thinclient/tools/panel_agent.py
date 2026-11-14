#!/usr/bin/env python3
"""FDC-Panel-Agent — kleiner Verwaltungsdienst auf dem Anzeige-Client.

Der Viewer zeigt Bilder, dieser Agent macht den Client **fernkonfigurierbar**.
Der FDC fragt hier Status ab und schiebt Layouts rein, ohne dass jemand per SSH
auf dem Gerät arbeiten muss:

    GET  /status    Zustand: Hostname, Rolle, laufende Panels, Auflösung, Version
    GET  /config    aktuelles Layout als JSON
    PUT  /config    neues Layout setzen -> Datei schreiben -> Viewer neu starten
    POST /restart   Viewer-Dienst neu starten
    GET  /log?n=80  letzte Zeilen des Viewer-Dienstes

Sicherheit: Token im Header ``X-FDC-Token``. Token-Datei (Default
``/etc/fdc/agent.token``) enthält eine Zeile. Fehlt sie, läuft der Agent im
offenen Modus und **sagt das laut** — für Labor okay, für Kunden nicht.

Aufruf (systemd):
    panel_agent.py --config /etc/fdc/panels.json --unit fdc-panel --port 8790
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

VERSION = "1.0.0"
STANDARD_LAYOUT = "/etc/fdc/panels.json"
STANDARD_TOKEN = "/etc/fdc/agent.token"
STANDARD_UNIT = "fdc-panel"
#: OTA-Zustand (vom Dienst fdc-ota geschrieben) — Anzeige im FDC und Popup am Client.
OTA_STATE = "/run/fdc-ota.json"
#: Protokoll des abgekoppelten Update-Laufs.
OTA_LOG = "/var/log/fdc-ota.log"


def zustand_schreiben_ota(daten: dict) -> None:
    """OTA-Zustand schreiben (der Agent meldet „läuft" vor dem Neustart)."""
    daten = dict(daten)
    daten["geprueft"] = int(time.time())
    try:
        ziel = Path(OTA_STATE)
        tmp = ziel.with_suffix(".tmp")
        tmp.write_text(json.dumps(daten, indent=2, ensure_ascii=False) + "\n",
                       encoding="utf-8")
        os.chmod(tmp, 0o644)
        tmp.replace(ziel)
    except OSError:
        pass
MAX_BODY = 64 * 1024
#: Der Agent legt bei jedem angemeldeten Zugriff diese Datei an.  Der Viewer
#: liest nur ihr Alter und zeigt damit „FDC AN/AUS" im Diag-Overlay — ohne dass
#: der FDC selbst auf dem Client laufen muss.
HERZSCHLAG = Path("/tmp/fdc-heartbeat")


def herzschlag_schreiben(pfad: Path = HERZSCHLAG) -> None:
    try:
        pfad.parent.mkdir(parents=True, exist_ok=True)
        pfad.write_text(str(int(time.time())), encoding="utf-8")
    except OSError:
        pass                      # Herzschlag ist Beiwerk, nie ein Fehlerfall


# ---------------------------------------------------------------------------
# Layout prüfen — dieselben Regeln wie der Viewer
# ---------------------------------------------------------------------------
def pruefe_layout(daten) -> dict:
    """Wirft ``ValueError`` bei Unsinn, gibt das normalisierte Layout zurück."""
    if not isinstance(daten, dict):
        raise ValueError("Layout muss ein JSON-Objekt sein")
    panels = daten.get("panels")
    if not isinstance(panels, list) or not panels:
        raise ValueError("'panels' fehlt oder ist leer")
    flaeche = daten.get("canvas") or {}
    sauber = {
        "engine": str(daten.get("engine") or "").strip(),
        "vollbild": bool(daten.get("vollbild", True)),
        "debug": bool(daten.get("debug", False)),
        "mode": str(daten.get("mode") or ("canvas" if flaeche else "windows")).lower(),
        "panels": [],
    }
    if sauber["mode"] not in ("windows", "canvas"):
        raise ValueError("'mode' muss windows oder canvas sein")
    if flaeche:
        try:
            breite = int(flaeche.get("width") or 0)
            hoehe = int(flaeche.get("height") or 0)
            drehung = float(flaeche.get("rotate") or 0.0)
            schirm = int(flaeche.get("screen", -1))
        except (TypeError, ValueError):
            raise ValueError("'canvas': width/height/rotate/screen müssen Zahlen sein")
        if not (0 < breite <= 16384) or not (0 < hoehe <= 16384):
            raise ValueError("'canvas': width/height müssen 1 … 16384 sein")
        if not (-360.0 <= drehung <= 360.0):
            raise ValueError("'canvas': rotate muss -360 … 360 sein")
        sauber["canvas"] = {"width": breite, "height": hoehe,
                            "rotate": drehung, "screen": schirm}
    if not sauber["engine"]:
        raise ValueError("'engine' fehlt (Host:Port der GlassOut-Engine)")
    gesehen = set()
    for eintrag in panels:
        if not isinstance(eintrag, dict):
            raise ValueError("jeder Panel-Eintrag muss ein Objekt sein")
        name = str(eintrag.get("panel") or "").strip()
        if not name:
            raise ValueError("'panel' (Panel-ID) fehlt in einem Eintrag")
        if name in gesehen:
            raise ValueError(f"Panel '{name}' doppelt im Layout")
        gesehen.add(name)
        rect = eintrag.get("rect") or [0, 0, 0, 0]
        if not isinstance(rect, list) or len(rect) != 4:
            raise ValueError(f"{name}: 'rect' braucht 4 Zahlen [x,y,w,h]")
        try:
            rect = [int(v) for v in rect]
        except (TypeError, ValueError):
            raise ValueError(f"{name}: 'rect' enthält keine Zahlen")
        if rect[2] < 0 or rect[3] < 0:
            raise ValueError(f"{name}: Breite/Höhe in 'rect' darf nicht negativ sein")
        neu = {"panel": name, "rect": rect,
               "screen": int(eintrag.get("screen") or 0)}
        # Platzierung und Skalierung: der Client rechnet das Rechteck aus der
        # Bildschirmgröße (rect = 0,0,0,0) oder nimmt absolute Pixel (rect gesetzt).
        position = str(eintrag.get("position") or "full").strip().lower()
        if position not in ("full", "left", "right", "top", "bottom"):
            raise ValueError(f"{name}: 'position' muss full|left|right|top|bottom sein")
        neu["position"] = position
        try:
            anteil = float(eintrag.get("size") or 1.0)
            zoom = float(eintrag.get("zoom") or 1.0)
        except (TypeError, ValueError):
            raise ValueError(f"{name}: 'size'/'zoom' müssen Zahlen sein")
        if not 0.05 <= anteil <= 1.0:
            raise ValueError(f"{name}: 'size' muss zwischen 0.05 und 1.0 liegen")
        if not 0.1 <= zoom <= 4.0:
            raise ValueError(f"{name}: 'zoom' muss zwischen 0.1 und 4.0 liegen")
        neu["size"] = anteil
        neu["zoom"] = zoom
        scale = str(eintrag.get("scale") or "contain").strip().lower()
        if scale not in ("contain", "stretch", "cover"):
            raise ValueError(f"{name}: 'scale' muss contain|stretch|cover sein")
        neu["scale"] = scale
        # Pixelmaße und Offsets: das ist die Einstellung, die der Anwender im FDC
        # wirklich dreht ("960 Pixel breit, 40 Pixel nach rechts").
        try:
            breite = int(eintrag.get("width") or 0)
            hoehe = int(eintrag.get("height") or 0)
            off_x = int(eintrag.get("offset_x", eintrag.get("x")) or 0)
            off_y = int(eintrag.get("offset_y", eintrag.get("y")) or 0)
        except (TypeError, ValueError):
            raise ValueError(f"{name}: 'width'/'height'/'offset_x'/'offset_y' "
                             f"müssen ganze Zahlen sein")
        if not (0 <= breite <= 16384) or not (0 <= hoehe <= 16384):
            raise ValueError(f"{name}: 'width'/'height' müssen 0 … 16384 sein")
        if not (-16384 <= off_x <= 16384) or not (-16384 <= off_y <= 16384):
            raise ValueError(f"{name}: Offsets müssen -16384 … 16384 sein")
        neu["width"], neu["height"] = breite, hoehe
        neu["offset_x"], neu["offset_y"] = off_x, off_y
        neu["diag"] = bool(eintrag.get("diag", False))
        # "canvas": true -> dieses Panel gehört auf die gedrehte Leinwand
        neu["canvas"] = bool(eintrag.get("canvas", False))
        if eintrag.get("crop"):
            crop = eintrag["crop"]
            if not isinstance(crop, list) or len(crop) != 4:
                raise ValueError(f"{name}: 'crop' braucht 4 Zahlen [x,y,w,h]")
            neu["crop"] = [int(v) for v in crop]
        fps = int(eintrag.get("fps") or 15)
        if not 1 <= fps <= 60:
            raise ValueError(f"{name}: 'fps' muss zwischen 1 und 60 liegen")
        neu["fps"] = fps
        if eintrag.get("engine_fps"):
            neu["engine_fps"] = int(eintrag["engine_fps"])
        if eintrag.get("strecken"):
            neu["strecken"] = True
        sauber["panels"].append(neu)
    return sauber


# ---------------------------------------------------------------------------
# Systemzugriffe — alles über injizierbare Funktionen, damit testbar
# ---------------------------------------------------------------------------
class System:
    """Dünne Hülle um systemctl & Co.  Im Test durch Attrappen ersetzt."""

    def __init__(self, unit: str, layout_pfad: str) -> None:
        self.unit = unit
        self.layout_pfad = Path(layout_pfad)
        # Läuft der Agent als normaler Benutzer (so ist er gedacht), braucht er
        # für systemctl ein enges sudo-Recht.  Als root entfällt das Präfix.
        self._praefix: list[str] = []
        if os.geteuid() != 0 and shutil.which("sudo"):
            self._praefix = ["sudo", "-n"]

    def dienst_laeuft(self) -> Optional[bool]:
        if not shutil.which("systemctl"):
            return None
        ergebnis = subprocess.run([*self._praefix, "systemctl", "is-active", self.unit],
                                  capture_output=True, text=True)
        return ergebnis.stdout.strip() == "active"

    def neu_starten(self) -> tuple[bool, str]:
        if not shutil.which("systemctl"):
            return False, "systemctl fehlt"
        ergebnis = subprocess.run([*self._praefix, "systemctl", "restart", self.unit],
                                  capture_output=True, text=True)
        if ergebnis.returncode != 0:
            return False, (ergebnis.stderr or ergebnis.stdout).strip()[:400]
        return True, "neu gestartet"

    def journal(self, zeilen: int) -> str:
        if not shutil.which("journalctl"):
            return "(journalctl fehlt)"
        ergebnis = subprocess.run(
            ["journalctl", "-u", self.unit, "-n", str(zeilen), "--no-pager",
             "-o", "short-iso"],
            capture_output=True, text=True)
        return (ergebnis.stdout or ergebnis.stderr or "").strip()[-20000:]

    def layout_lesen(self) -> dict:
        if not self.layout_pfad.exists():
            return {}
        try:
            return json.loads(self.layout_pfad.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def layout_schreiben(self, daten: dict) -> None:
        self.layout_pfad.parent.mkdir(parents=True, exist_ok=True)
        # atomar: erst temporär, dann umbenennen — der Viewer liest die Datei
        # beim Start, ein halb geschriebenes Layout wäre ein Absturzgrund
        tmp = self.layout_pfad.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(daten, indent=2, ensure_ascii=False) + "\n",
                       encoding="utf-8")
        tmp.replace(self.layout_pfad)

    def paket_version(self) -> str:
        """Version des installierten Client-Pakets (``/opt/fdc/VERSION``)."""
        for pfad in (Path("/opt/fdc/VERSION"), Path("/opt/fdc/version.txt")):
            try:
                text = pfad.read_text(encoding="utf-8").strip()
            except OSError:
                continue
            if text:
                return text.splitlines()[0].strip()
        return "0.0.0"

    def ota_zustand(self) -> dict:
        """OTA-Stand aus ``/run/fdc-ota.json`` (leer, wenn nie geprüft)."""
        try:
            daten = json.loads(Path(OTA_STATE).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        return daten if isinstance(daten, dict) else {}

    def ota_installieren(self) -> tuple[bool, str, dict]:
        """Update anstoßen — **abgekoppelt**, Antwort kommt sofort.

        Das Update startet diesen Agenten am Ende selbst neu.  Würde der Agent
        auf den Unterprozess warten, killt der Neustart mitten im Update die
        eigene Antwort (real passiert: ``Empty reply from server``) und der
        Endzustand wird nie geschrieben.  Deshalb eigener Prozess/Session und
        sofortige Rückmeldung; das Ergebnis steht in ``/run/fdc-ota.json``.
        """
        zustand = self.ota_zustand()
        if zustand.get("laeuft"):
            return True, "Update läuft bereits", zustand
        zustand_schreiben_ota({"laeuft": True,
                               "verfuegbar": True,
                               "aktuell": self.paket_version(),
                               "version": str(zustand.get("version") or "")})
        # Eigene systemd-Unit: der Installer startet fdc-agent am Ende neu, und
        # das würde jeden Prozess in derselben cgroup mitkillen (real passiert:
        # „laeuft" blieb stehen, kein Rollback, kein Abschluss).
        befehl = self._praefix + ["/usr/bin/systemd-run",
                                  "--unit=fdc-ota-install", "--collect",
                                  "--description=FDC OTA-Installation",
                                  sys.executable, "/opt/fdc/tools/ota_update.py",
                                  "--installieren"]
        protokoll = None
        try:
            protokoll = open(OTA_LOG, "ab", buffering=0)
        except OSError:
            protokoll = None
        ziel_ausgabe = protokoll if protokoll is not None else subprocess.DEVNULL
        try:
            subprocess.Popen(befehl, stdout=ziel_ausgabe, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, close_fds=True)
        except OSError as exc:
            return False, f"{type(exc).__name__}: {exc}", {}
        finally:
            if protokoll is not None:
                protokoll.close()
        return True, "Update läuft — Status über /status abfragen", {}

    def bildschirmfoto(self, schirm: int = 0) -> tuple[Optional[bytes], str]:
        """PNG des Client-Bildschirms — Grundlage für den Live-Picker im FDC.

        Als Unterprozess, weil der Agent selbst keine Display-Verbindung hat und
        eine Qt-Anwendung sich nicht nebenher in einem HTTP-Thread hochziehen lässt.
        """
        helfer = Path(__file__).with_name("panel_grab.py")
        if not helfer.exists():
            return None, f"{helfer} fehlt (Paket unvollständig)"
        try:
            ergebnis = subprocess.run(
                [sys.executable, str(helfer), "--display", str(int(schirm))],
                capture_output=True, timeout=20)
        except subprocess.TimeoutExpired:
            return None, "Zeitüberschreitung beim Bildschirmfoto"
        if ergebnis.returncode != 0 or not ergebnis.stdout:
            meldung = (ergebnis.stderr or b"").decode("utf-8", "replace").strip()
            return None, meldung[-300:] or "kein Bild erhalten"
        return ergebnis.stdout, ""

    def bildschirme(self) -> list:
        if not shutil.which("xrandr"):
            return []
        ergebnis = subprocess.run(["xrandr", "--query"], capture_output=True,
                                  text=True, env={**os.environ, "DISPLAY": ":0"})
        schirme = []
        for zeile in (ergebnis.stdout or "").splitlines():
            if " connected" in zeile:
                teile = zeile.split()
                schirme.append({"name": teile[0],
                                "aktiv": " primary " in zeile or "*" in zeile,
                                "modus": next((t for t in teile if "x" in t and "+" in t), "")})
        return schirme


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------
def baue_handler(system: System, token: str, startzeit: float):
    class Handler(BaseHTTPRequestHandler):
        server_version = f"FDC-PanelAgent/{VERSION}"

        # -- Hilfen --------------------------------------------------------
        def _antwort(self, code: int, daten) -> None:
            roh = json.dumps(daten, ensure_ascii=False, indent=2).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(roh)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(roh)

        def _fehler(self, code: int, text: str) -> None:
            self._antwort(code, {"ok": False, "fehler": text})

        def _antwort_bild(self, daten: bytes) -> None:
            """Binäre Antwort (PNG) — kein JSON."""

            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(daten)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(daten)

        def _auth_ok(self) -> bool:
            if not token or self.headers.get("X-FDC-Token", "").strip() == token:
                # Modulglobale lesen, damit Tests den Pfad ersetzen können
                herzschlag_schreiben(HERZSCHLAG)
                return True
            return False

        def _body(self) -> dict:
            laenge = int(self.headers.get("Content-Length") or 0)
            if laenge <= 0:
                raise ValueError("leerer Body")
            if laenge > MAX_BODY:
                raise ValueError("Body zu groß")
            roh = self.rfile.read(laenge)
            return json.loads(roh.decode("utf-8"))

        def _darf_schreiben(self) -> bool:
            if not self._auth_ok():
                self._fehler(403, "Token fehlt oder falsch")
                return False
            return True

        # -- Routen --------------------------------------------------------
        def do_GET(self) -> None:  # noqa: N802
            pfad = self.path.split("?")[0].rstrip("/") or "/"
            if pfad == "/status":
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                layout = system.layout_lesen()
                return self._antwort(200, {
                    "ok": True,
                    "agent": VERSION,
                    "hostname": socket.gethostname(),
                    "adresse": self._eigene_ip(),
                    "laufzeit_s": round(time.monotonic() - startzeit, 1),
                    "dienst_laeuft": system.dienst_laeuft(),
                    "unit": system.unit,
                    "layout_datei": str(system.layout_pfad),
                    "layout": layout,
                    "bildschirme": system.bildschirme(),
                    # Paketversion (VERSION im /opt/fdc) und OTA-Stand:
                    # der FDC zeigt beides an, ohne selbst nach GitLab zu fragen.
                    "version": system.paket_version(),
                    "ota": system.ota_zustand(),
                })
            if pfad == "/config":
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                return self._antwort(200, system.layout_lesen() or {})
            if pfad == "/screen":
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                schirm = 0
                if "?" in self.path:
                    for teil in self.path.split("?")[1].split("&"):
                        if teil.startswith("display="):
                            try:
                                schirm = max(0, min(8, int(teil[8:])))
                            except ValueError:
                                schirm = 0
                bild, fehler = system.bildschirmfoto(schirm)
                if bild is None:
                    return self._fehler(500, f"Bildschirmfoto fehlgeschlagen: {fehler}")
                return self._antwort_bild(bild)
            if pfad == "/log":
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                zeilen = 80
                if "?" in self.path:
                    for teil in self.path.split("?")[1].split("&"):
                        if teil.startswith("n="):
                            try:
                                zeilen = max(1, min(500, int(teil[2:])))
                            except ValueError:
                                pass
                return self._antwort(200, {"ok": True, "log": system.journal(zeilen)})
            return self._fehler(404, f"unbekannter Pfad {pfad}")

        def do_PUT(self) -> None:  # noqa: N802
            if not self._darf_schreiben():
                return
            if self.path.split("?")[0].rstrip("/") != "/config":
                return self._fehler(404, "nur /config ist beschreibbar")
            try:
                roh = self._body()
                layout = pruefe_layout(roh)
            except (ValueError, json.JSONDecodeError) as exc:
                return self._fehler(400, str(exc))
            try:
                system.layout_schreiben(layout)
            except OSError as exc:
                return self._fehler(500, f"Layout nicht schreibbar: {exc}")
            ok, meldung = system.neu_starten()
            return self._antwort(200 if ok else 500,
                                 {"ok": ok, "meldung": meldung, "layout": layout})

        def do_POST(self) -> None:  # noqa: N802
            if not self._darf_schreiben():
                return
            pfad = self.path.split("?")[0].rstrip("/")
            if pfad == "/update":
                ok, meldung, daten = system.ota_installieren()
                return self._antwort(200 if ok else 500,
                                     {"ok": ok, "meldung": meldung,
                                      "version": system.paket_version(),
                                      "ergebnis": daten})
            if pfad != "/restart":
                return self._fehler(404, "unbekannter Pfad")
            ok, meldung = system.neu_starten()
            return self._antwort(200 if ok else 500, {"ok": ok, "meldung": meldung})

        def _eigene_ip(self) -> str:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.connect(("192.0.2.1", 9))     # kein echter Verkehr
                    return s.getsockname()[0]
            except OSError:
                return ""

        def log_message(self, format: str, *args) -> None:  # noqa: A002 (Qt/Qt-los: Qt-Signatur)
            sys.stderr.write("[agent] %s - %s\n" % (self.address_string(), format % args))

    return Handler


def eigene_ip() -> str:
    """LAN-Adresse dieses Rechners (Routing-Probe, kein Paketversand)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("192.0.2.1", 9))
            return sock.getsockname()[0]
    except OSError:
        return ""


def eigene_mac() -> str:
    """MAC der ersten echten Netzwerkkarte (für die stabile Kennung im FDC)."""
    basis = Path("/sys/class/net")
    if not basis.exists():
        return ""
    for eintrag in sorted(basis.iterdir()):
        if eintrag.name == "lo":
            continue
        try:
            adresse = (eintrag / "address").read_text(encoding="ascii").strip()
        except OSError:
            continue
        if adresse and adresse != "00:00:00:00:00:00":
            return adresse.upper()
    return ""


class Beacon(threading.Thread):
    """Meldet den Client per UDP-Broadcast im Netz an (Port 5010).

    Der FDC findet damit Anzeige-Clients ohne IP-Eintrag.  Gleiches Format wie
    die ISIS-Panels (``IDENT:..;MAC:..;IP:..;FW:..;``), aber eigener Port — so
    mischen sich Anzeige-Clients und Panels nicht in einer Liste.
    """

    def __init__(self, port: int = 5010, intervall: float = 15.0,
                 agent_port: int = 8790, version: str = "") -> None:
        super().__init__(daemon=True, name="Beacon")
        self.version = version or VERSION
        self.port = int(port)
        self.intervall = max(2.0, float(intervall))
        self.agent_port = int(agent_port)
        self._laufen = True

    def text(self) -> str:
        return (f"IDENT:{socket.gethostname()};MAC:{eigene_mac()};"
                f"IP:{eigene_ip()};PORT:{self.agent_port};"
                f"FW:{self.version};HW:thinclient;")

    def run(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except OSError as exc:
            print(f"[beacon] nicht möglich: {exc}", file=sys.stderr, flush=True)
            return
        with sock:
            while self._laufen:
                try:
                    sock.sendto(self.text().encode("ascii", "replace"),
                                ("255.255.255.255", self.port))
                except OSError:
                    pass
                time.sleep(self.intervall)

    def stoppen(self) -> None:
        self._laufen = False


def token_lesen(pfad: str) -> str:
    try:
        return Path(pfad).read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description="FDC-Panel-Agent (Fernkonfiguration)")
    p.add_argument("--config", default=STANDARD_LAYOUT, help="Pfad der Layoutdatei")
    p.add_argument("--token", default=STANDARD_TOKEN, help="Pfad der Token-Datei")
    p.add_argument("--unit", default=STANDARD_UNIT, help="systemd-Unit des Viewers")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8790)
    p.add_argument("--beacon-port", type=int, default=5010,
                   help="UDP-Port der Netz-Anmeldung (0 = aus)")
    args = p.parse_args(argv)

    token = token_lesen(args.token)
    if not token:
        print(f"[agent] WARNUNG: keine Token-Datei unter {args.token} — "
              f"der Agent ist offen für jeden im Netz!", file=sys.stderr)
    system = System(args.unit, args.config)
    server = ThreadingHTTPServer((args.host, args.port),
                                 baue_handler(system, token, time.monotonic()))
    beacon = None
    if args.beacon_port:
        beacon = Beacon(port=args.beacon_port, agent_port=args.port,
                        version=system.paket_version())
        beacon.start()
    print(f"[agent] hört auf {args.host}:{args.port}  unit={args.unit}  "
          f"layout={args.config}  auth={'ja' if token else 'NEIN'}"
          f"  beacon={'UDP %d' % args.beacon_port if beacon else 'aus'}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if beacon is not None:
            beacon.stoppen()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
