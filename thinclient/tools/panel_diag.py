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
HERMES_BLAU = QColor(0, 170, 255)
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
                     sekunden: float = 0.0, detail: str = "",
                     titel: str = "SYSTEMS UNAVAIL") -> None:
    """Grüne Meldung wie auf einem echten Display, wenn keine Daten kommen.

    Ohne Tastatur am Client ist eine schwarze Fläche Blindflug: man soll am
    Bildschirm ablesen können, welches Panel fehlt und wie lange schon.
    """
    x0 = flaeche.x() if hasattr(flaeche, "x") else 0.0
    y0 = flaeche.y() if hasattr(flaeche, "y") else 0.0
    breite = flaeche.width()
    hoehe = flaeche.height()
    # Erst leeren.  Sonst schreibt sich „SYSTEMS UNAVAIL" über die alten
    # Instrumente (real passiert, nachdem GlassOut wegbricht): ein Standbild mit
    # grünem Text darüber sieht aus wie ein halb funktionierendes Panel.  Ein
    # blankes Feld ist die ehrliche Aussage.  Nur die eigene Fläche — bei zwei
    # Panels auf einem Schirm darf das andere Panel nicht mit verschwinden.
    maler.fillRect(QRectF(x0, y0, breite, hoehe), QColor(0, 0, 0))
    grosse = max(13, int(min(breite, hoehe) * 0.055))
    kleine = max(9, int(grosse * 0.55))
    mitte = y0 + hoehe / 2.0

    maler.setPen(GRUEN)
    schrift = QFont("DejaVu Sans", grosse, QFont.Bold)
    maler.setFont(schrift)
    titel_rect = QRectF(x0, mitte - grosse * 1.6, breite, grosse * 2.0)
    maler.drawText(titel_rect, Qt.AlignCenter, titel or "SYSTEMS UNAVAIL")

    maler.setPen(QColor(150, 200, 170))
    maler.setFont(QFont("DejaVu Sans", kleine))
    # Der Text kommt (übersetzt) von aussen; der Default bleibt deutsch.
    text = detail or f"{panel_id} — keine Daten"
    if sekunden > 0:
        text += f"  ({sekunden:.0f} s)"
    maler.drawText(QRectF(x0, mitte + grosse * 0.5, breite, grosse * 1.6),
                   Qt.AlignCenter, text)


def diag_splash_zeichnen(maler: QPainter, flaeche, bericht, sprache_code: str = "en") -> None:
    """Fehlerbild „Panel - Diag" — was der Client selbst über sich weiß.

    Ohne SSH und ohne Tastatur ist der Bildschirm die einzige Auskunft.  Die
    Zeilen kommen fertig aus ``panel_selfcheck.bericht()``; die Beschriftungen
    werden übersetzt, damit die Anzeige derselben Sprache folgt wie das Panel.
    """
    try:
        from tools import panel_lang as pl
    except ImportError:
        import panel_lang as pl

    x0 = flaeche.x() if hasattr(flaeche, "x") else 0.0
    y0 = flaeche.y() if hasattr(flaeche, "y") else 0.0
    breite = flaeche.width()
    hoehe = flaeche.height()
    maler.fillRect(QRectF(x0, y0, breite, hoehe), QColor(8, 8, 10))

    zeilen = list(getattr(bericht, "zeilen", []) or [])
    status = str(getattr(bericht, "status", "fehler")).upper()
    probleme = problem_texte(bericht, sprache_code)

    grosse = max(12, int(min(breite * 0.040, hoehe * 0.055)))
    klein = max(9, int(grosse * 0.62))
    rand = int(min(breite, hoehe) * 0.045)

    # Kopf: TITEL + Status
    titel = pl.text("diag_titel", sprache_code)
    farbe_status = {"FEHLER": QColor(235, 70, 70),
                    "WARNUNG": QColor(235, 190, 60)}.get(status, QColor(80, 220, 140))
    maler.setFont(QFont("DejaVu Sans", int(grosse * 1.15), QFont.Bold))
    maler.setPen(QColor(230, 230, 235))
    maler.drawText(QRectF(x0 + rand, y0 + rand, breite - 2 * rand, grosse * 1.6),
                   Qt.AlignLeft | Qt.AlignVCenter, titel)
    status_text = pl.text("status_" + status.lower(), sprache_code)
    maler.setPen(farbe_status)
    maler.drawText(QRectF(x0 + rand, y0 + rand, breite - 2 * rand, grosse * 1.6),
                   Qt.AlignRight | Qt.AlignVCenter, status_text)

    # Trennlinie
    y = y0 + rand + grosse * 2.0
    maler.setPen(QColor(60, 60, 70))
    maler.drawLine(int(x0 + rand), int(y), int(x0 + breite - rand), int(y))
    y += grosse * 0.6

    # Schlüssel/Wert-Tabelle, Schrift wird an die längste Zeile angepasst
    werte = [(pl.text("diag_" + _diag_schluessel(k), sprache_code),
              _wert_text(wert, sprache_code))
             for k, wert in zeilen]
    # Zeilenhöhe so wählen, dass ALLE Zeilen Platz haben — eine Tafel, die die
    # entscheidenden Zeilen (Rechte, Uhr) abschneidet, ist wertlos.
    fuss = klein * 2.0
    verfuegbar = max(1.0, (y0 + hoehe - rand - fuss) - y)
    anzahl = max(1, len(werte))
    zeilen_hoehe = int(min(grosse * 1.45, verfuegbar / anzahl))
    klein = max(8, min(klein, int(zeilen_hoehe * 0.66)))
    spalte_links = breite * 0.40
    spalte_rechts = breite * 0.55
    for schluessel, wert in werte:
        maler.setFont(QFont("DejaVu Sans", klein))
        masse = maler.fontMetrics()
        maler.setPen(QColor(150, 160, 175))
        maler.drawText(QRectF(x0 + rand, y, spalte_links, zeilen_hoehe),
                       Qt.AlignLeft | Qt.AlignVCenter,
                       masse.elidedText(schluessel, Qt.ElideRight, int(spalte_links - 6)))
        maler.setPen(_wert_farbe(wert))
        maler.drawText(QRectF(x0 + rand + spalte_links, y, spalte_rechts, zeilen_hoehe),
                       Qt.AlignLeft | Qt.AlignVCenter,
                       masse.elidedText(wert, Qt.ElideRight, int(spalte_rechts - 6)))
        y += zeilen_hoehe

    # Fuß: erste Meldung + Hinweis
    if probleme:
        maler.setFont(QFont("DejaVu Sans", klein))
        maler.setPen(QColor(240, 200, 90))
        maler.drawText(QRectF(x0 + rand, y0 + hoehe - rand - zeilen_hoehe,
                              breite - 2 * rand, zeilen_hoehe),
                       Qt.AlignLeft | Qt.AlignVCenter, probleme[0][:120])
    maler.setFont(QFont("DejaVu Sans", max(8, int(klein * 0.85))))
    maler.setPen(QColor(110, 115, 125))
    maler.drawText(QRectF(x0 + rand, y0 + hoehe - rand - zeilen_hoehe,
                          breite - 2 * rand, zeilen_hoehe),
                   Qt.AlignRight | Qt.AlignVCenter,
                   pl.text("diag_hinweis", sprache_code))


def problem_texte(bericht, sprache_code: str = "en") -> list[str]:
    """Problemzeilen in der Sprache der Anlage (Code → Satz)."""
    try:
        from tools import panel_lang as pl
    except ImportError:
        import panel_lang as pl

    codes = list(getattr(bericht, "codes", None) or [])
    if not codes:                        # alter Bericht ohne Codes
        return list(getattr(bericht, "probleme", None) or [])
    texte = []
    for code, werte in codes:
        try:
            texte.append(pl.text("prob_" + str(code), sprache_code, **(werte or {})))
        except Exception:                # noqa: BLE001 - Anzeige darf nie kippen
            texte.append(str(code))
    return texte


def _wert_text(wert, sprache_code: str = "en") -> str:
    """Tabellenwert in die Sprache der Anlage bringen.

    ``panel_selfcheck`` liefert neutrale Tokens (``ok``, ``error``,
    ``missing:websocket``, ``stale:120``) — nur so kann dieselbe Zeile in vier
    Sprachen erscheinen.  Alles, was kein Token ist (IP, Version, Datum), geht
    unverändert durch.
    """
    try:
        from tools import panel_lang as pl
    except ImportError:
        import panel_lang as pl

    text = str(wert if wert is not None else "")
    if text.startswith("missing:"):
        return pl.text("diag_wert_missing", sprache_code, liste=text.split(":", 1)[1])
    if text.startswith("stale:"):
        return pl.text("diag_wert_stale", sprache_code, sekunden=text.split(":", 1)[1])
    if text in ("ok", "error", "complete", "connected", "no_fdc", "yes", "no", "none"):
        return pl.text("diag_wert_" + text, sprache_code)
    return text


def _wert_farbe(wert) -> QColor:
    """Nur echte Probleme rot — sonst ist die ganze Tafel rot und sagt nichts."""
    text = str(wert or "").strip()
    if text in ("error", "no_fdc", "none", "no") or text.startswith(("missing:", "stale:")):
        return QColor(250, 110, 110)
    if text in ("ok", "complete", "connected", "yes"):
        return QColor(120, 220, 150)
    return QColor(220, 225, 230)


def _diag_schluessel(name: str) -> str:
    """``Stream Active`` → ``stream_active`` (Schlüssel in panel_lang)."""
    return (str(name).strip().lower().replace(" ", "_")
            .replace("ä", "ae").replace("ö", "oe").replace("ü", "ue"))


def warnband_zeichnen(maler: QPainter, flaeche, text: str) -> None:
    """Schmales Band am unteren Rand, solange das Bild läuft aber etwas klemmt.

    Ein unplausible Uhr oder fehlende Schreibrechte sind keine Sichtfehler — das
    Panel soll weiter Instrumente zeigen.  Ganz ignorieren darf man es aber
    auch nicht, sonst steht man beim Update-Rätselraten wieder im Dunkeln.
    """
    if not text:
        return
    x0 = flaeche.x() if hasattr(flaeche, "x") else 0.0
    y0 = flaeche.y() if hasattr(flaeche, "y") else 0.0
    breite, hoehe = flaeche.width(), flaeche.height()
    band = max(18.0, hoehe * 0.045)
    oben = y0 + hoehe - band
    maler.fillRect(QRectF(x0, oben, breite, band), QColor(20, 16, 6, 210))
    maler.setPen(QColor(235, 190, 60))
    maler.setFont(QFont("DejaVu Sans", max(8, int(band * 0.52))))
    maler.drawText(QRectF(x0 + band * 0.3, oben, breite - band * 0.6, band),
                   Qt.AlignLeft | Qt.AlignVCenter,
                   text if len(text) <= 110 else text[:107] + "...")


def abdunkeln(maler: QPainter, flaeche, staerke: int = 190) -> None:
    """Bildschirmschoner: alles abdimmen, ohne den Text unlesbar zu machen.

    Wird über die ganze Fläche gelegt, wenn lange kein Bild mehr kam.  ``staerke``
    ist das Alpha der schwarzen Lage (0 = aus, 255 = komplett schwarz).
    """
    wert = max(0, min(255, int(staerke)))
    if wert == 0:
        return
    x0 = flaeche.x() if hasattr(flaeche, "x") else 0.0
    y0 = flaeche.y() if hasattr(flaeche, "y") else 0.0
    maler.fillRect(QRectF(x0, y0, flaeche.width(), flaeche.height()),
                   QColor(0, 0, 0, wert))


def splash_zeichnen(maler: QPainter, flaeche, sprache_code: str = "en",
                    adresse: str = "") -> None:
    """Startbild ohne Konfiguration: „Panel im FDC einrichten".

    Ein frisch installierter Client hat noch keine Panel-Zuordnung — der Viewer
    läuft (damit der Agent erreichbar ist), zeigt aber nichts an. Ohne diesen
    Hinweis sieht das aus wie ein kaputtes Gerät. Sprache folgt dem FDC bzw.
    dem Cache (``panel_lang``), Default Englisch.
    """
    try:                                     # als Paket (Tests, Agent)
        from tools import panel_lang as pl
    except ImportError:                      # als Skript gestartet
        import panel_lang as pl
    x0 = flaeche.x() if hasattr(flaeche, "x") else 0.0
    y0 = flaeche.y() if hasattr(flaeche, "y") else 0.0
    breite = flaeche.width()
    hoehe = flaeche.height()
    groesse = max(12, int(min(breite * 0.040, hoehe * 0.055)))

    titel = pl.text("splash_titel", sprache_code)
    zeilen = [pl.text("splash_text", sprache_code),
              pl.text("splash_weg", sprache_code)]
    if adresse:
        zeilen.append(pl.text("splash_ip", sprache_code, ip=adresse))
    zeilen.append(pl.text("splash_sprache", sprache_code,
                          sprache=pl.SPRACHNAMEN.get(sprache_code, sprache_code)))

    kasten = QRectF(x0 + breite * 0.08, y0 + hoehe * 0.22,
                    breite * 0.84, hoehe * 0.52)
    maler.setPen(Qt.NoPen)
    maler.setBrush(QColor(0, 0, 0, 225))
    maler.drawRoundedRect(kasten, groesse * 0.5, groesse * 0.5)
    maler.setBrush(Qt.NoBrush)
    maler.setPen(QPen(HERMES_BLAU, max(1, groesse // 7)))
    maler.drawRoundedRect(kasten, groesse * 0.5, groesse * 0.5)

    inner = kasten.adjusted(groesse, groesse * 0.8, -groesse, -groesse * 0.8)
    # Schrift so lange verkleinern, bis alles in den Kasten passt
    while groesse > 9:
        maler.setFont(QFont("DejaVu Sans", int(groesse * 1.4), QFont.Bold))
        breit_titel = maler.fontMetrics().horizontalAdvance(titel)
        maler.setFont(QFont("DejaVu Sans", groesse))
        masse = maler.fontMetrics()
        breiteste = max([masse.horizontalAdvance(z) for z in zeilen] + [breit_titel])
        hoehe_gesamt = groesse * 1.4 * 2.2 + len(zeilen) * groesse * 1.8
        if breiteste <= inner.width() and hoehe_gesamt <= inner.height():
            break
        groesse -= 1

    maler.setPen(HERMES_BLAU)
    maler.setFont(QFont("DejaVu Sans", int(groesse * 1.4), QFont.Bold))
    maler.drawText(QRectF(inner.x(), inner.y(), inner.width(), groesse * 2.0),
                   Qt.AlignCenter, titel)
    y = inner.y() + groesse * 2.4
    for nummer, zeile in enumerate(zeilen):
        # Adresse und Sprache sind Beiwerk, die Anweisung ist die Hauptsache
        maler.setPen(QColor(235, 240, 245) if nummer < 2 else QColor(160, 168, 178))
        maler.setFont(QFont("DejaVu Sans", groesse))
        maler.drawText(QRectF(inner.x(), y, inner.width(), groesse * 1.7),
                       Qt.AlignCenter, zeile)
        y += groesse * 1.8


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


def setup_zeichnen(maler: QPainter, flaeche, ssid: str, adresse: str,
                   fehler: str = "") -> int:
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
    if fehler:
        # Warum kein Hotspot kommt, sieht man sonst nur im Journal — und das ist
        # ohne Netz und ohne SSH unerreichbar. Deshalb hier auf den Schirm, über
        # zwei Zeilen umgebrochen und an Wortgrenzen gekürzt: eine harte
        # Zeichengrenze schnitt mitten in „powersave" ab, also genau die Stelle,
        # die den Fehler erklärt.
        maler.setPen(QColor(240, 120, 120))
        maler.setFont(QFont("DejaVu Sans", max(8, int(groesse * 0.8))))
        masse = maler.fontMetrics()
        rest = str(fehler).strip().replace("\n", " ").split()
        for _ in range(2):
            if not rest:
                break
            zeile_fehler = ""
            while rest:
                probe = (zeile_fehler + " " + rest[0]).strip()
                if masse.horizontalAdvance(probe) > inner.width() and zeile_fehler:
                    break
                zeile_fehler = probe
                rest.pop(0)
            if rest:                       # gekürzt → Auslassungszeichen
                zeile_fehler = zeile_fehler.rstrip() + " …"
            maler.drawText(QRectF(inner.x(), y, inner.width(), groesse * 1.3),
                           Qt.AlignLeft | Qt.AlignVCenter, zeile_fehler)
            y += groesse * 1.3
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
