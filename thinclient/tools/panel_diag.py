"""Diagnose-Overlay und Zustandssammlung für die Panel-Clients.

Kein Teil des Bildpfads: das Overlay wird nur zusätzlich über das Panelbild
gezeichnet, wenn der FDC es im Layout einschaltet (``"diag": true``).

Gesammelt wird, was man am kopflosen Client sonst nur im Log sieht:

* **GlassOut** — antwortet die Engine auf ``/status``?
* **Simulator** — ist MSFS verbunden und welches Flugzeug?
* **FDC** — hat sich der FDC in der letzten Zeit gemeldet?  (Der Agent legt bei
  jedem Aufruf einen Herzschlag ab; der Viewer liest nur dessen Alter.  So
  funktioniert die Anzeige auch, wenn FDC und Client getrennte Prozesse und
  Rechner sind.)
* **Bildrate** — kommt vom Fenster, nicht von hier.
* **CPU-Last** — aus ``/proc/stat``, Differenz zweier Abtastungen.
"""
from __future__ import annotations

import json
import os
import threading
import time
import urllib.request
from typing import Optional

from pathlib import Path

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen

#: Ampel-Farben. „warn" heißt: erreichbar, aber nicht betriebsbereit
#: (z. B. Engine läuft, Sim aber nicht im Cockpit).
GRUEN = QColor(61, 220, 132)
GELB = QColor(255, 193, 7)
ROT = QColor(244, 90, 80)
GRAU = QColor(150, 160, 170)


def farbe(zustand: str) -> QColor:
    return {"on": GRUEN, "warn": GELB, "off": ROT}.get(zustand, GRAU)


class DiagMonitor:
    """Fragt die GlassOut-Engine ab und misst die CPU-Last dieses Rechners."""

    def __init__(self, host: str, port: int,
                 heartbeat: str = "/tmp/fdc-heartbeat",
                 intervall: float = 2.0) -> None:
        self.host = host
        self.port = int(port)
        self.heartbeat = heartbeat
        self.intervall = max(1.0, float(intervall))
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._werte = {"glassout": "unknown", "sim": "unknown", "fdc": "unknown",
                       "cpu": 0.0, "engine": f"{host}:{port}", "msfs": "",
                       "version": paket_version(), "ota_verfuegbar": False,
                       "ota_version": ""}
        self._cpu_letzte = self._cpu_stand()

    # -- CPU ---------------------------------------------------------------
    @staticmethod
    def _cpu_stand() -> tuple[int, int]:
        with open("/proc/stat", encoding="ascii") as datei:
            felder = datei.readline().split()[1:]
        werte = [int(v) for v in felder]
        gesamt = sum(werte)
        leer = sum(werte[3:5])          # idle + iowait
        return gesamt, leer

    def _cpu_anteil(self) -> float:
        gesamt, leer = self._cpu_stand()
        d_gesamt = gesamt - self._cpu_letzte[0]
        d_leer = leer - self._cpu_letzte[1]
        self._cpu_letzte = (gesamt, leer)
        if d_gesamt <= 0:
            return float(self._werte["cpu"])
        return max(0.0, min(100.0, 100.0 * (d_gesamt - d_leer) / d_gesamt))

    # -- Zustände ----------------------------------------------------------
    def _engine_fragen(self) -> None:
        adresse = f"http://{self.host}:{self.port}/status"
        try:
            with urllib.request.urlopen(adresse, timeout=1.5) as antwort:
                roh = json.loads(antwort.read().decode("utf-8", "replace"))
        except Exception:                     # noqa: BLE001 - Netz darf alles werfen
            self._werte["glassout"] = "off"
            self._werte["sim"] = "off"
            self._werte["msfs"] = ""
            return
        self._werte["glassout"] = "on"
        zustand = roh.get("state") or {}
        verbindung = str(zustand.get("simConnect") or zustand.get("msfsConnection") or "")
        if verbindung.lower().startswith("connected"):
            self._werte["sim"] = "on"
        elif zustand.get("msfsDetected"):
            self._werte["sim"] = "warn"
        else:
            self._werte["sim"] = "off"
        self._werte["msfs"] = str(zustand.get("aircraft") or "")

    def _fdc_fragen(self) -> None:
        try:
            alter = time.time() - os.path.getmtime(self.heartbeat)
        except OSError:
            self._werte["fdc"] = "off"
            return
        self._werte["fdc"] = "on" if alter < 90 else "warn"

    def _ota_fragen(self) -> None:
        """Version und Update-Stand für die Diag-Zeile auffrischen."""
        stand = ota_stand()
        self._werte["version"] = str(stand.get("aktuell") or paket_version() or "")
        self._werte["ota_verfuegbar"] = bool(stand.get("verfuegbar"))
        self._werte["ota_version"] = str(stand.get("version") or "")

    def _schleife(self) -> None:
        while self._running:
            try:
                self._engine_fragen()
                self._fdc_fragen()
                self._ota_fragen()
                self._werte["cpu"] = self._cpu_anteil()
            except Exception:                 # noqa: BLE001 - Schleife nie sterben lassen
                pass
            time.sleep(self.intervall)

    def starten(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._schleife, daemon=True,
                                        name="DiagMonitor")
        self._thread.start()

    def stoppen(self) -> None:
        self._running = False

    def werte(self) -> dict:
        return dict(self._werte)


# ---------------------------------------------------------------------------
# Zeichnen
# ---------------------------------------------------------------------------
def diag_zeichnen(maler: QPainter, flaeche, werte: dict, fps: float = 0.0,
                  titel: str = "FDC DIAG", klein: bool = False) -> None:
    """Statusfeld oben links — Ampelpunkte, Werte rechtsbündig."""
    breite_seite = min(flaeche.width(), flaeche.height())
    rand = max(10, int(breite_seite * 0.018))
    breite = int(max(260, min(400, flaeche.width() * 0.30)))
    zeilen_hoehe = max(20, int(breite_seite * 0.032))
    kopf = int(zeilen_hoehe * 1.25)

    zeilen = [
        ("GlassOut", werte.get("glassout"), werte.get("engine") or ""),
        ("Simulator", werte.get("sim"), werte.get("msfs") or ""),
        ("FDC", werte.get("fdc"), ""),
        ("Bildrate", "info", f"{fps:.1f} fps"),
        ("CPU Last", "info", f"{werte.get('cpu', 0.0):.0f} %"),
        ("Version", "warn" if werte.get("ota_verfuegbar") else "info",
         version_text(werte)),
    ]
    hoehe = kopf + len(zeilen) * zeilen_hoehe + rand // 2
    box = QRectF(rand, rand, breite, hoehe)

    maler.setPen(QPen(QColor(90, 120, 150, 220), 1))
    maler.setBrush(QColor(10, 16, 24, 218))
    maler.drawRoundedRect(box, 9, 9)

    # Kopfzeile mit Verlauf aus zwei Balken (kein Gradient nötig)
    kopf_rect = QRectF(rand, rand, breite, kopf)
    maler.setPen(Qt.NoPen)
    maler.setBrush(QColor(38, 66, 96, 235))
    maler.drawRoundedRect(kopf_rect, 9, 9)
    maler.drawRect(QRectF(rand, rand + kopf - 9, breite, 9))
    maler.setPen(QColor(226, 240, 255))
    maler.setFont(QFont("DejaVu Sans", max(8, int(zeilen_hoehe * 0.42)), QFont.Bold))
    maler.drawText(QRectF(rand + 12, rand, breite - 24, kopf),
                   Qt.AlignVCenter | Qt.AlignLeft, titel)

    schrift = QFont("DejaVu Sans", max(7, int(zeilen_hoehe * 0.36)))
    schrift_wert = QFont("DejaVu Sans Mono", max(7, int(zeilen_hoehe * 0.36)), QFont.Bold)
    y = rand + kopf
    for name, zustand, detail in zeilen:
        mitte = QRectF(rand + 12, y, breite - 24, zeilen_hoehe)
        # Ampelpunkt (bei reinen Messwerten ein neutraler Punkt)
        punkt = farbe(zustand) if zustand != "info" else QColor(120, 170, 210)
        maler.setPen(Qt.NoPen)
        maler.setBrush(punkt)
        maler.drawEllipse(QRectF(rand + 12, y + zeilen_hoehe / 2 - 4, 8, 8))
        maler.setPen(QColor(205, 215, 225))
        maler.setFont(schrift)
        maler.drawText(QRectF(rand + 28, y, breite * 0.42, zeilen_hoehe),
                       Qt.AlignVCenter | Qt.AlignLeft, name)
        if zustand == "info":
            wert_text, wert_farbe = detail, QColor(170, 210, 240)
        else:
            wert_text = {"on": "AN", "warn": "TEILWEISE", "off": "AUS"}.get(zustand, "?")
            wert_farbe = farbe(zustand)
            if detail and zustand == "on":
                wert_text = f"{wert_text}  {detail}"
        maler.setFont(schrift_wert)
        maler.setPen(wert_farbe)
        maler.drawText(QRectF(rand + 28, y, breite - 40, zeilen_hoehe),
                       Qt.AlignVCenter | Qt.AlignRight, wert_text)
        y += zeilen_hoehe


def unavail_zeichnen(maler: QPainter, flaeche, panel_id: str,
                     sekunden: float = 0.0) -> None:
    """Grüne Meldung wie auf einem echten Display, wenn keine Daten kommen.

    Ohne Tastatur am Client ist eine schwarze Fläche Blindflug: man soll am
    Bildschirm ablesen können, welches Panel fehlt und wie lange schon.
    """
    x0 = flaeche.x() if hasattr(flaeche, "x") else 0.0
    y0 = flaeche.y() if hasattr(flaeche, "y") else 0.0
    breite = flaeche.width()
    hoehe = flaeche.height()
    grosse = max(13, int(min(breite, hoehe) * 0.055))
    kleine = max(9, int(grosse * 0.55))
    mitte = y0 + hoehe / 2.0

    maler.setPen(GRUEN)
    schrift = QFont("DejaVu Sans", grosse, QFont.Bold)
    maler.setFont(schrift)
    titel_rect = QRectF(x0, mitte - grosse * 1.6, breite, grosse * 2.0)
    maler.drawText(titel_rect, Qt.AlignCenter, "SYSTEMS UNAVAIL")

    maler.setPen(QColor(150, 200, 170))
    maler.setFont(QFont("DejaVu Sans", kleine))
    detail = f"{panel_id} — keine Daten"
    if sekunden > 0:
        detail += f"  ({sekunden:.0f} s)"
    maler.drawText(QRectF(x0, mitte + grosse * 0.5, breite, grosse * 1.6),
                   Qt.AlignCenter, detail)


# ---------------------------------------------------------------------------
# Einrichtungs-Hinweis (kein Netz: WLAN über das Handy einrichten)
# ---------------------------------------------------------------------------
SETUP_DATEI = "/run/fdc-wlan-setup.json"


def setup_hinweis(pfad: str = SETUP_DATEI) -> dict:
    """Zustand des WLAN-Setups lesen (leer, wenn nichts einzurichten ist)."""
    try:
        daten = json.loads(Path(pfad).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return daten if isinstance(daten, dict) and daten.get("aktiv") else {}


def setup_zeichnen(maler: QPainter, flaeche, ssid: str, adresse: str) -> int:
    """Anleitung auf dem Bildschirm — ohne Tastatur der einzige Weg zum Nutzer.

    Ohne diesen Hinweis sieht der Client aus wie „kaputt": schwarzer Schirm, kein
    Netz, und niemand weiß, dass er auf ein Handy wartet.

    Die Schrift wird an der **Textlänge** eingepasst, nicht an der Fläche: beim
    ersten Versuch lief die SSID über den Kastenrand hinaus und wurde
    abgeschnitten („wählen" → „w").    Gemessen wird mit QFontMetrics, verkleinert
    bis es passt.  Rückgabe: die verwendete Schriftgröße (für Tests).
    """
    x0 = flaeche.x() if hasattr(flaeche, "x") else 0.0
    y0 = flaeche.y() if hasattr(flaeche, "y") else 0.0
    breite = flaeche.width()
    hoehe = flaeche.height()
    start = max(11, int(min(breite * 0.055, hoehe * 0.06)))

    kasten = QRectF(x0 + breite * 0.05, y0 + hoehe * 0.16,
                    breite * 0.90, hoehe * 0.66)
    maler.setPen(Qt.NoPen)
    maler.setBrush(QColor(0, 0, 0, 215))
    maler.drawRoundedRect(kasten, start * 0.4, start * 0.4)
    maler.setBrush(Qt.NoBrush)
    maler.setPen(QPen(GELB, max(1, start // 8)))
    maler.drawRoundedRect(kasten, start * 0.4, start * 0.4)

    inner = kasten.adjusted(start * 0.7, start * 0.5, -start * 0.7, -start * 0.5)
    titel = "WLAN EINRICHTEN"
    zeilen = [f'1  Handy: WLAN "{ssid}" wählen',
              f"2  Browser: {adresse}",
              "3  Netz + Passwort eingeben",
              "LAN-Kabel einstecken geht auch."]

    # Eine gemeinsame Größe für alle Zeilen: verschiedene Größen sähen nach Fehler aus.
    groesse = start
    while groesse > 8:
        schrift = QFont("DejaVu Sans", groesse)
        maler.setFont(schrift)
        masse = maler.fontMetrics()
        titel_gross = QFont("DejaVu Sans", int(groesse * 1.35), QFont.Bold)
        maler.setFont(titel_gross)
        breiteste = max([masse.horizontalAdvance(z) for z in zeilen] +
                        [maler.fontMetrics().horizontalAdvance(titel)])
        hoehe_gesamt = (groesse * 1.35 * 1.9) + len(zeilen) * groesse * 1.8
        if breiteste <= inner.width() and hoehe_gesamt <= inner.height():
            break
        groesse -= 1

    maler.setPen(GELB)
    maler.setFont(QFont("DejaVu Sans", int(groesse * 1.35), QFont.Bold))
    maler.drawText(QRectF(inner.x(), inner.y(), inner.width(), groesse * 1.9),
                   Qt.AlignLeft | Qt.AlignVCenter, titel)

    y = inner.y() + groesse * 2.2
    for nummer, zeile in enumerate(zeilen):
        # Der LAN-Hinweis ist die Notlösung, nicht der Hauptweg: dezenter.
        maler.setPen(QColor(150, 156, 166) if nummer == len(zeilen) - 1
                     else QColor(235, 240, 245))
        maler.setFont(QFont("DejaVu Sans", groesse))
        maler.drawText(QRectF(inner.x(), y, inner.width(), groesse * 1.7),
                       Qt.AlignLeft | Qt.AlignVCenter, zeile)
        y += groesse * 1.8
    return groesse


# ---------------------------------------------------------------------------
# Version und OTA (Update aus dem öffentlichen Repo)
# ---------------------------------------------------------------------------
OTA_DATEI = "/run/fdc-ota.json"
#: Zustand des OS-Updates (schreibt ``fdc-os-install``, s. os_update.py)
OS_DATEI = "/run/fdc-os.json"
VERSION_DATEI = "/opt/fdc/VERSION"


def paket_version(pfad: str = VERSION_DATEI) -> str:
    """Installierte Client-Version (``/opt/fdc/VERSION``)."""
    try:
        text = Path(pfad).read_text(encoding="utf-8").strip()
    except OSError:
        return ""
    return text.splitlines()[0].strip() if text else ""


def _stand_mit_dateizeit(daten: dict, pfad: Path) -> dict:
    """Zeitstempel ergänzen: ältere Zustandsdateien haben kein ``stand``.

    Ohne den könnte ein Einmal-Hinweis nie auslaufen (oder nie erscheinen) —
    die Dateizeit ist der ehrlichste Ersatz.
    """
    if not daten.get("stand"):
        try:
            daten["dateizeit"] = pfad.stat().st_mtime
        except OSError:
            pass
    return daten


def ota_stand(pfad: str = OTA_DATEI) -> dict:
    """OTA-Zustand vom Dienst ``fdc-ota`` lesen."""
    datei = Path(pfad)
    try:
        daten = json.loads(datei.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return _stand_mit_dateizeit(daten, datei) if isinstance(daten, dict) else {}


def version_text(werte: dict) -> str:
    """Text für die Diag-Zeile: ``0.1.0 (aktuell)`` bzw. ``0.1.0 → 0.2.0``."""
    aktuell = str(werte.get("version") or paket_version() or "—")
    if werte.get("ota_verfuegbar"):
        return f"{aktuell} → {werte.get('ota_version') or '?'}"
    return f"{aktuell} (aktuell)"


def ota_popup(maler: QPainter, flaeche, text: str, farbe_punkt=None) -> None:
    """Kleines Feld unten rechts: „Update verfügbar" / „Update wird installiert".

    Bewusst klein und in der Ecke — es soll nicht wie eine Störung des Panels
    aussehen, aber ohne Hinweis am Schirm merkt niemand, dass ein Update wartet.
    """
    if not text:
        return
    x0 = flaeche.x() if hasattr(flaeche, "x") else 0.0
    y0 = flaeche.y() if hasattr(flaeche, "y") else 0.0
    breite = flaeche.width()
    hoehe = flaeche.height()
    groesse = max(9, int(min(breite, hoehe) * 0.026))
    schrift = QFont("DejaVu Sans", groesse)
    maler.setFont(schrift)
    masse = maler.fontMetrics()
    text_breite = masse.horizontalAdvance(text)
    kasten_breite = min(breite * 0.9, text_breite + groesse * 3.2)
    kasten_hoehe = groesse * 2.6
    x = x0 + breite - kasten_breite - groesse * 0.8
    y = y0 + hoehe - kasten_hoehe - groesse * 0.8
    box = QRectF(x, y, kasten_breite, kasten_hoehe)

    maler.setPen(QPen(QColor(90, 120, 150, 200), 1))
    maler.setBrush(QColor(10, 16, 24, 225))
    maler.drawRoundedRect(box, groesse * 0.5, groesse * 0.5)
    maler.setPen(Qt.NoPen)
    maler.setBrush(farbe_punkt or GELB)
    maler.drawEllipse(QRectF(x + groesse * 0.8, y + kasten_hoehe / 2 - groesse * 0.35,
                             groesse * 0.7, groesse * 0.7))
    maler.setPen(QColor(235, 240, 245))
    maler.setFont(schrift)
    maler.drawText(QRectF(x + groesse * 1.9, y, kasten_breite - groesse * 2.4, kasten_hoehe),
                   Qt.AlignVCenter | Qt.AlignLeft, text)


#: Wie lange ein erledigter Hinweis am Panel stehen bleibt.  Ein „Update
#: installiert" soll man sehen, aber es darf nicht für immer im Eck kleben.
POPUP_TIMEOUT_S = 30.0


def _frisch(stand: dict, jetzt: Optional[float] = None) -> bool:
    """Wurde der Zustand in den letzten :data:`POPUP_TIMEOUT_S` geschrieben?"""
    try:
        stamp = stand.get("stand") or stand.get("dateizeit") or 0
        alter = (time.time() if jetzt is None else jetzt) - float(stamp)
    except (TypeError, ValueError):
        return False
    return 0 <= alter <= POPUP_TIMEOUT_S


def ota_text(stand: dict, jetzt: Optional[float] = None) -> str:
    """Popup-Text aus dem OTA-Zustand bauen (leer = kein Popup).

    „Installiert" ist ein Einmal-Hinweis und verschwindet nach 30 s;
    „läuft" und „verfügbar" sind Zustände und bleiben stehen.
    """
    if not stand:
        return ""
    if str(stand.get("meldung") or "").lower().startswith("aktualisiert"):
        if not _frisch(stand, jetzt):
            return ""
        return f"Update installiert: {stand.get('aktuell', '?')}"
    if stand.get("laeuft"):
        return "Update wird installiert …"
    if stand.get("verfuegbar"):
        return f"Update verfügbar: {stand.get('version', '?')}"
    return ""


def os_stand(pfad: Path | str = OS_DATEI) -> dict:
    """OS-Update-Zustand des Clients lesen (``/run/fdc-os.json``)."""
    datei = Path(pfad)
    try:
        daten = json.loads(datei.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return _stand_mit_dateizeit(daten, datei) if isinstance(daten, dict) else {}


def os_text(stand: dict, jetzt: Optional[float] = None) -> str:
    """Popup-Text für laufende/erledigte OS-Updates (leer = kein Popup)."""
    if not stand:
        return ""
    if stand.get("laeuft"):
        return "OS-Update läuft …"
    if stand.get("ok") is False and stand.get("meldung"):
        return "OS-Update fehlgeschlagen"
    if stand.get("reboot_noetig") and not stand.get("reboot_gestartet"):
        return "OS-Update fertig — Neustart nötig"
    if _frisch(stand, jetzt) and stand.get("ok") is not None:
        vorher, nachher = stand.get("vorher"), stand.get("nachher")
        if isinstance(vorher, int) and isinstance(nachher, int):
            return f"OS-Update fertig: {max(0, vorher - nachher)} Paket(e)"
        return "OS-Update fertig"
    return ""


def status_popup(ota: dict, os_zustand: dict, jetzt: Optional[float] = None) -> str:
    """Ein Hinweis fürs Panel: laufende Updates zuerst, dann Erledigtes.

    Beides gleichzeitig geht nicht — sonst überdeckt ein Kasten den anderen.
    """
    return (os_text(os_zustand or {}, jetzt)
            or ota_text(ota or {}, jetzt))
