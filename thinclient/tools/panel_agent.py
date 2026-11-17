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
import re
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
#: Solange diese Datei existiert, laeuft die Einrichtung (apt) — Neustart gesperrt.
FIRSTBOOT_MARKER = "/var/lib/fdc/.firstboot-laeuft"
STANDARD_LAYOUT = "/etc/fdc/panels.json"
#: Geraete-Identitaet: Name + Zuordnung "Monitor 1/2 -> bis zu 2 Panels".
STANDARD_GERAET = "/etc/fdc/geraet.json"

try:                                    # als Paket (Tests, FDC-Tools)
    from tools import os_update, panel_lang, power_idle
    from tools.geraet_ident import (anzeigename, geraet_aus_layout, ident_text,
                                    mac_kuerzel, layout_aus_geraet, seriennummer,
                                    standard_name)
except ImportError:                     # direkt aus /opt/fdc/tools gestartet
    import os_update, panel_lang, power_idle                             # type: ignore
    from geraet_ident import (anzeigename, geraet_aus_layout, ident_text,  # type: ignore
                              mac_kuerzel, layout_aus_geraet, seriennummer,
                              standard_name)
STANDARD_TOKEN = "/etc/fdc/agent.token"
STANDARD_UNIT = "fdc-panel"
#: OTA-Zustand (vom Dienst fdc-ota geschrieben) — Anzeige im FDC und Popup am Client.
OTA_STATE = "/run/fdc-ota.json"
OS_LOG = "/var/log/fdc-os.log"
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
def _geraet_normalisieren(geraet: object) -> dict:
    """Geraete-Dict in feste Form bringen (nur bekannte Felder, max. 2 je Monitor)."""
    quelle = geraet if isinstance(geraet, dict) else {}
    ergebnis: dict = {
        "name": str(quelle.get("name") or "").strip(),
        "sn": str(quelle.get("sn") or "").strip(),
        # Sprache vom FDC: das Panel zeigt Texte in derselben Sprache (gecacht,
        # damit es auch ohne FDC beim Start stimmt)
        "lang": panel_lang.sprache_pruefen(quelle.get("lang")),
        "author": str(quelle.get("author") or "MAQ").strip() or "MAQ",
        "monitor1": [],
        "monitor2": [],
        "monitor1_drehen": bool(quelle.get("monitor1_drehen")),
        "monitor2_drehen": bool(quelle.get("monitor2_drehen")),
    }
    if not ergebnis["name"]:
        ergebnis["name"] = standard_name(eigene_mac())
    gesehen: set[str] = set()
    for monitor in ("monitor1", "monitor2"):
        for eintrag in (quelle.get(monitor) or [])[:2]:
            if isinstance(eintrag, str):
                eintrag = {"panel": eintrag}
            if not isinstance(eintrag, dict):
                continue
            panel = str(eintrag.get("panel") or "").strip()
            if not panel or panel in gesehen:
                continue
            gesehen.add(panel)
            sauber: dict = {"panel": panel}
            try:
                dreh = int(eintrag.get("rotate") or 0)
            except (TypeError, ValueError):
                dreh = 0
            if dreh:
                sauber["rotate"] = dreh
            ergebnis[monitor].append(sauber)
    return ergebnis


def pruefe_geraet(roh: object) -> dict:
    """Body von ``PUT /geraet`` pruefen.  Raise ``ValueError`` bei Unsinn."""
    if not isinstance(roh, dict):
        raise ValueError("Geraet muss ein JSON-Objekt sein")
    for monitor in ("monitor1", "monitor2"):
        eintraege = roh.get(monitor)
        if eintraege is None:
            continue
        if not isinstance(eintraege, list):
            raise ValueError(f"{monitor} muss eine Liste sein")
        if len(eintraege) > 2:
            raise ValueError(f"{monitor}: hoechstens 2 Panels")
        for eintrag in eintraege:
            panel = eintrag if isinstance(eintrag, str) else (eintrag or {}).get("panel") if isinstance(eintrag, dict) else None
            if not str(panel or "").strip():
                raise ValueError(f"{monitor}: Eintrag ohne Panel-Namen")
    geraet = _geraet_normalisieren(roh)
    if not geraet["monitor1"] and not geraet["monitor2"]:
        raise ValueError("mindestens ein Panel zuordnen")
    return geraet


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
    # Mehrere Leinwände (eine je Schirm) — muss durchgereicht werden, sonst
    # schluckt der Agent die Zuordnung still.
    leinwaende = []
    for eintrag in (daten.get("leinwaende") or []):
        if not isinstance(eintrag, dict):
            raise ValueError("'leinwaende': jeder Eintrag muss ein Objekt sein")
        try:
            schirm = int(eintrag.get("screen") or 0)
            breite = int(eintrag.get("width") or 0)
            hoehe = int(eintrag.get("height") or 0)
            drehung = float(eintrag.get("rotate") or 0.0)
        except (TypeError, ValueError):
            raise ValueError("'leinwaende': screen/width/height/rotate müssen Zahlen sein")
        if not (0 <= schirm <= 1):
            raise ValueError("'leinwaende': screen muss 0 oder 1 sein")
        if not (0 < breite <= 16384) or not (0 < hoehe <= 16384):
            raise ValueError("'leinwaende': width/height müssen 1 … 16384 sein")
        if not (-360.0 <= drehung <= 360.0):
            raise ValueError("'leinwaende': rotate muss -360 … 360 sein")
        leinwaende.append({"screen": schirm, "width": breite,
                           "height": hoehe, "rotate": drehung})
    if leinwaende:
        sauber["leinwaende"] = leinwaende
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

    def __init__(self, unit: str, layout_pfad: str,
                 geraet_pfad: str = STANDARD_GERAET) -> None:
        self.unit = unit
        self.layout_pfad = Path(layout_pfad)
        self.geraet_pfad = Path(geraet_pfad)
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

    # -- Geraete-Identitaet ------------------------------------------------
    def geraet_lesen(self) -> dict:
        """Name + Monitor-Zuordnung.  Fehlt die Datei, aus dem Layout ableiten.

        So hat ein bereits eingerichteter Client ohne Zutun eine Identitaet —
        die Firmware bleibt universell, das Geraet erfaehrt vom FDC, wer es ist.
        """
        try:
            roh = self.geraet_pfad.read_text(encoding="utf-8-sig")
            geraet = json.loads(roh)
            if isinstance(geraet, dict):
                return _geraet_normalisieren(geraet)
        except (OSError, ValueError):
            pass
        return _geraet_normalisieren(geraet_aus_layout(self.layout_lesen() or {}, eigene_mac()))

    def geraet_schreiben(self, geraet: dict) -> None:
        daten = _geraet_normalisieren(geraet)
        # Sprache vom FDC mitschreiben: der Viewer läuft als ssf und liest den
        # Cache, wenn das FDC beim nächsten Start nicht da ist.
        if daten.get("lang"):
            try:
                panel_lang.cache_schreiben(str(daten.get("lang")))
            except OSError:
                pass
        self.geraet_pfad.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.geraet_pfad.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(daten, indent=2, ensure_ascii=False), encoding="utf-8")
        os.replace(tmp, self.geraet_pfad)

    def seriennummer(self) -> str:
        """S/N des Geräts: /etc/fdc/serial (SMBIOS), sonst aus der MAC."""
        return seriennummer(self.geraet_lesen(), eigene_mac())

    def ident(self) -> str:
        """IDENT-String im Format der seriellen Panels."""
        return ident_text(self.geraet_lesen(), eigene_mac(), self.paket_version())

    def anzeigename(self) -> str:
        return anzeigename(self.geraet_lesen(), eigene_mac())

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

    # -- Energie: Reboot, Shutdown, Auto-Power-off -------------------------
    def power_lesen(self) -> dict:
        """Konfiguration und Zustand des Auto-Power-off."""
        return {"config": power_idle.config_lesen(),
                "zustand": power_idle.zustand_lesen()}

    def power_schreiben(self, daten: dict) -> dict:
        cfg = power_idle.config_schreiben(daten)
        return {"config": cfg, "zustand": power_idle.zustand_lesen()}

    def power_aktion(self, aktion: str) -> tuple[bool, str]:
        """Reboot oder Herunterfahren — **abgekoppelt** gestartet.

        Wartet der Agent auf ``systemctl``, ist die Antwort weg, sobald das
        System die Dienste beendet (der Aufrufer sieht nur einen Abbruch).
        """
        aktion = str(aktion or "").strip().lower()
        if aktion in ("shutdown", "aus", "poweroff"):
            befehl = "poweroff"
        elif aktion in ("reboot", "neustart"):
            befehl = "reboot"
        else:
            return False, "aktion muss reboot oder poweroff sein"
        # Waehrend der ersten Einrichtung laeuft apt. Ein Neustart mitten darin
        # hinterlaesst ein halb konfiguriertes dpkg (X-Server fehlt, Viewer
        # startet nie) — deshalb hier hart verweigern statt spaeter reparieren.
        if Path(FIRSTBOOT_MARKER).exists():
            return False, ("Einrichtung laeuft noch — Neustart gesperrt, "
                           "bitte warten (Log: /var/lib/fdc/firstboot.log)")
        zeile = self._praefix + ["/usr/bin/systemctl", befehl]
        try:
            subprocess.Popen(zeile, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                             close_fds=True, start_new_session=True)
        except OSError as exc:
            return False, f"{type(exc).__name__}: {exc}"
        return True, f"{befehl} ausgeloest"

    # -- OS-Aktualisierungen ----------------------------------------------
    def os_zustand(self) -> dict:
        return os_update.status()

    def os_quittieren(self) -> dict:
        """Neustart-Frage abhaken (FDC soll nicht wiederholt fragen)."""
        return os_update.quittieren()

    def os_installieren(self) -> tuple[bool, str, dict]:
        """OS-Update anstossen (eigene Unit, damit der Agent neu starten darf)."""
        zustand = os_update.zustand_lesen()
        if zustand.get("laeuft"):
            return True, "OS-Update laeuft bereits", zustand
        os_update.zustand_schreiben({"laeuft": True, "start": time.time()})
        befehl = self._praefix + ["/usr/bin/systemd-run", "--unit=fdc-os-install",
                                  "--collect", "--description=FDC OS-Update",
                                  sys.executable, "/opt/fdc/tools/os_update.py",
                                  "--installieren"]
        protokoll = None
        try:
            protokoll = open(OS_LOG, "ab", buffering=0)
        except OSError:
            protokoll = None
        ziel = protokoll if protokoll is not None else subprocess.DEVNULL
        try:
            subprocess.Popen(befehl, stdout=ziel, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, close_fds=True)
        except OSError as exc:
            os_update.zustand_schreiben({"laeuft": False,
                                         "meldung": f"{type(exc).__name__}: {exc}"})
            return False, f"{type(exc).__name__}: {exc}", {}
        finally:
            if protokoll is not None:
                protokoll.close()
        return True, "OS-Update laeuft — Status ueber /os abfragen", {}

    # -- Automatik (Firmware und OS getrennt) ------------------------------
    def auto_lesen(self) -> dict:
        return os_update.auto_config_lesen()

    def auto_schreiben(self, daten: dict) -> dict:
        return os_update.auto_config_schreiben(daten)

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
        """Schirme mit aktueller Auflösung **und allen Modi** (xrandr).

        Die Modi sind wichtig: im Pit haengen auch aeltere 17-Zoll-Schirme ohne
        HD, und das FDC soll die Flaeche aus einer echten Aufloesung waehlen
        koennen statt aus einer geratenen Zahl.
        """
        if not shutil.which("xrandr"):
            return []
        try:
            ergebnis = subprocess.run(["xrandr", "--query"], capture_output=True,
                                      text=True, timeout=10,
                                      env={**os.environ, "DISPLAY": ":0"})
        except subprocess.SubprocessError:
            return []
        schirme: list = []
        aktuell = None
        for zeile in (ergebnis.stdout or "").splitlines():
            if " connected" in zeile:
                teile = zeile.split()
                treffer = re.search(r"(\d+)x(\d+)\+", zeile)
                aktuell = {"name": teile[0],
                           "aktiv": "connected primary" in zeile,
                           "modus": next((t for t in teile if "x" in t and "+" in t), ""),
                           "aufloesung": (f"{treffer.group(1)}x{treffer.group(2)}"
                                          if treffer else ""),
                           "modi": []}
                schirme.append(aktuell)
                continue
            if aktuell is None:
                continue
            if not zeile.startswith((" ", "\t")):
                aktuell = None            # naechster Schirm-Abschnitt
                continue
            # Moduszeilen sind eingerueckt: "  1920x1080     60.00*+  50.00"
            for marke in zeile.split():
                if re.fullmatch(r"\d{3,5}x\d{3,5}", marke) and marke not in aktuell["modi"]:
                    aktuell["modi"].append(marke)
        for schirm in schirme:
            if not schirm["aufloesung"] and schirm["modi"]:
                schirm["aufloesung"] = schirm["modi"][0]
            schirm["modi"].sort(key=lambda m: tuple(int(n) for n in m.split("x")),
                                reverse=True)
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

        def _antwort_text(self, code: int, text: str) -> None:
            roh = str(text).encode("utf-8", "replace")
            try:
                self.send_response(code)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(roh)))
                self.end_headers()
                self.wfile.write(roh)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _geraet_setzen(self) -> None:
            """Geraete-Identitaet setzen und daraus das Layout bauen.

            Damit ist die Firmware universell: der FDC sagt dem Geraet, wer es
            ist (Name + Monitor-Zuordnung), das Geraet rechnet sich sein Layout
            selbst aus Name, Bildschirmgroesse und Anzahl der Panels.
            """
            try:
                roh = self._body()
                geraet = pruefe_geraet(roh)
            except (ValueError, json.JSONDecodeError) as exc:
                return self._fehler(400, str(exc))
            try:
                system.geraet_schreiben(geraet)
            except OSError as exc:
                return self._fehler(500, f"Geraet nicht schreibbar: {exc}")
            layout = None
            if bool(roh.get("layout_bauen", True)):
                alt = system.layout_lesen() or {}
                layout = layout_aus_geraet(
                    geraet, system.bildschirme(),
                    engine=str(alt.get("engine") or ""),
                    gedreht=[geraet.get("monitor1_drehen"), geraet.get("monitor2_drehen")],
                )
                try:
                    system.layout_schreiben(layout)
                except OSError as exc:
                    return self._fehler(500, f"Layout nicht schreibbar: {exc}")
            ok, meldung = system.neu_starten()
            return self._antwort(200 if ok else 500,
                                 {"ok": ok, "meldung": meldung,
                                  "ident": system.ident(),
                                  "anzeigename": system.anzeigename(),
                                  "geraet": geraet, "layout": layout})

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
                    # Energie + OS-Stand: das FDC zeigt beides im Client-Fenster
                    "power": system.power_lesen(),
                    "os": system.os_zustand(),
                    "auto": system.auto_lesen(),
                    # Identitaet wie bei den seriellen Panels: der FDC zeigt
                    # Name, FW, S/N und HW-Rev aus genau diesem String.
                    "ident": system.ident(),
                    "anzeigename": system.anzeigename(),
                    "geraet": system.geraet_lesen(),
                    # Echte S/N (SMBIOS) getrennt: das FDC schreibt sie in die
                    # S/N-Spalte des Panel-Tabs, auch wenn im Geräte-Eintrag
                    # nichts steht.
                    "seriennummer": system.seriennummer(),
                    # Sprache am Panel folgt dem FDC (Cache, sonst Installer)
                    "sprache": panel_lang.als_dict(
                        panel_lang.sprache(system.geraet_lesen())),
                })
            if pfad == "/ident":
                # Klartext wie eine serielle IDENT-Antwort — zum Vergleich im
                # Serial Monitor und fuer einfache Pruefungen mit curl.
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                return self._antwort_text(200, system.ident())
            if pfad == "/geraet":
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                return self._antwort(200, {"ok": True, "ident": system.ident(),
                                           "anzeigename": system.anzeigename(),
                                           "seriennummer": system.seriennummer(),
                                           "geraet": system.geraet_lesen(),
                                           "bildschirme": system.bildschirme(),
                                           "version": system.paket_version()})
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
            if pfad == "/power":
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                daten = system.power_lesen()
                return self._antwort(200, {"ok": True, **daten,
                                           "hostname": socket.gethostname()})
            if pfad == "/os":
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                return self._antwort(200, {"ok": True, "os": system.os_zustand(),
                                           "auto": system.auto_lesen()})
            if pfad == "/auto":
                if not self._auth_ok():
                    return self._fehler(403, "Token fehlt oder falsch")
                return self._antwort(200, {"ok": True, "auto": system.auto_lesen()})
            return self._fehler(404, f"unbekannter Pfad {pfad}")

        def do_PUT(self) -> None:  # noqa: N802
            if not self._darf_schreiben():
                return
            pfad = self.path.split("?")[0].rstrip("/")
            if pfad == "/geraet":
                return self._geraet_setzen()
            if pfad == "/power":
                try:
                    daten = self._body()
                except (ValueError, json.JSONDecodeError) as exc:
                    return self._fehler(400, str(exc))
                if not isinstance(daten, dict):
                    return self._fehler(400, "Objekt erwartet")
                werte = system.power_schreiben(daten)
                return self._antwort(200, {"ok": True, **werte})
            if pfad == "/auto":
                try:
                    daten = self._body()
                except (ValueError, json.JSONDecodeError) as exc:
                    return self._fehler(400, str(exc))
                if not isinstance(daten, dict):
                    return self._fehler(400, "Objekt erwartet")
                return self._antwort(200, {"ok": True,
                                           "auto": system.auto_schreiben(daten)})
            if pfad != "/config":
                return self._fehler(404,
                                    "beschreibbar: /config, /geraet, /power, /auto")
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
            if pfad == "/power":
                try:
                    daten = self._body()
                except (ValueError, json.JSONDecodeError) as exc:
                    return self._fehler(400, str(exc))
                aktion = str((daten or {}).get("aktion") or "").strip()
                ok, meldung = system.power_aktion(aktion)
                return self._antwort(200 if ok else 400,
                                     {"ok": ok, "meldung": meldung, "aktion": aktion})
            if pfad == "/os":
                # Ohne Body: Update starten.  Mit {"quittiert": true}: nur die
                # Reboot-Frage abhaken (FDC fragt sonst bei jedem Oeffnen neu).
                try:
                    daten = self._body()
                except (ValueError, json.JSONDecodeError):
                    daten = {}
                if isinstance(daten, dict) and daten.get("quittiert"):
                    system.os_quittieren()
                    return self._antwort(200, {"ok": True, "meldung": "quittiert",
                                               "os": system.os_zustand()})
                ok, meldung, ergebnis = system.os_installieren()
                return self._antwort(200 if ok else 500,
                                     {"ok": ok, "meldung": meldung,
                                      "ergebnis": ergebnis,
                                      "os": system.os_zustand()})
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
                 agent_port: int = 8790, version: str = "",
                 system: "System | None" = None) -> None:
        super().__init__(daemon=True, name="Beacon")
        self.version = version or VERSION
        self.port = int(port)
        self.intervall = max(2.0, float(intervall))
        self.agent_port = int(agent_port)
        self.system = system
        self._laufen = True

    def text(self) -> str:
        """Beacon-Text: meldet sich wie ein Panel mit Anzeigename und S/N."""
        name = socket.gethostname()
        sn = ""
        if self.system is not None:
            try:
                geraet = self.system.geraet_lesen()
                name = anzeigename(geraet, eigene_mac())
                sn = seriennummer(geraet, eigene_mac())
                self.version = self.system.paket_version() or self.version
            except Exception:
                pass
        return (f"IDENT:{name};MAC:{eigene_mac()};"
                f"IP:{eigene_ip()};PORT:{self.agent_port};"
                f"FW:{self.version};HW:thinclient;SN:{sn};")

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
    p.add_argument("--geraet", default=STANDARD_GERAET,
                   help="Pfad der Geraete-Identitaet (Name + Monitor-Zuordnung)")
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
    system = System(args.unit, args.config, args.geraet)
    server = ThreadingHTTPServer((args.host, args.port),
                                 baue_handler(system, token, time.monotonic()))
    beacon = None
    if args.beacon_port:
        beacon = Beacon(port=args.beacon_port, agent_port=args.port,
                        version=system.paket_version(), system=system)
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
