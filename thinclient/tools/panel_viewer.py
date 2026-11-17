#!/usr/bin/env python3
"""Schlanker Vollbild-Viewer für GlassOut-Panels (x86-Client, z. B. Wyse 3040).

Warum es das gibt
-----------------
Der Bildstrom zu den ESP-Panels läuft über JPEG-Neukodierung im FDC.  Für große
Schirme (17"/19" am Thin Client) ist das unnötig: dort läuft ein x86-Rechner, der
die Panels **direkt** von der GlassOut-Engine holen kann.  Damit ist der FDC aus
dem Pixelpfad — er macht nur noch Panel-Management.

Kein Browser, kein Node, kein Chromium: Qt dekodiert JPEG selbst, skaliert auf die
Fenstergröße und bleibt bei 2 GB RAM locker im Rahmen.

Beispiele
---------
Ein Panel, Vollbild auf dem ersten Schirm::

    panel_viewer.py --engine 192.168.40.20:8787 --panel PFD_FO

Zwei Panels nebeneinander auf dem ersten Display (PFD + ND, je 960x960)::

    panel_viewer.py --engine sim:8787 --panel PFD_FO --rect 0,0,960,960 \\
                    --panel ND_FO --rect 960,0,960,960

ECAM auf dem zweiten Display, obere und untere Hälfte::

    panel_viewer.py --engine sim:8787 --panel ECAM_UPPER --screen 1 \\
                    --rect 0,0,1280,512 --panel ECAM_LOWER --rect 0,512,1280,512

Layout aus Datei (vom FDC erzeugt, dann bleibt Panel-Management dort)::

    panel_viewer.py --layout /etc/fdc/panels.json

Layout-Datei::

    {"engine": "192.168.40.20:8787",
     "panels": [{"panel": "PFD_FO", "screen": 0, "rect": [0, 0, 960, 960],
                 "crop": [96, 96, 768, 768], "fps": 15},
                {"panel": "ND_FO",  "screen": 0, "rect": [960, 0, 960, 960],
                 "crop": [96, 96, 768, 768], "fps": 15}]}
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

# Import auch dann ermöglichen, wenn das Skript direkt gestartet wird
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from infra.panel_stream import GlassOutSource, StreamConfig, parse_glassout_frame  # noqa: E402

from PySide6.QtCore import QBuffer, QObject, QRectF, Qt, QTimer, Signal  # noqa: E402
from PySide6.QtGui import QImage, QPainter, QPen, QPixmap  # noqa: E402
from PySide6.QtWidgets import QApplication, QWidget  # noqa: E402

try:                                     # als Paket (Tests)
    from tools.panel_diag import (DiagMonitor, abdunkeln, diag_splash_zeichnen,
                                  diag_zeichnen, os_stand, ota_popup, ota_stand,
                                  setup_hinweis, setup_zeichnen, splash_zeichnen,
                                  problem_texte, status_popup,
                                  unavail_zeichnen, warnband_zeichnen)
    from tools import panel_lang, panel_selfcheck
except ImportError:                      # als Skript gestartet
    from panel_diag import (DiagMonitor, abdunkeln, diag_splash_zeichnen,
                            diag_zeichnen, os_stand, ota_popup, ota_stand,
                            setup_hinweis, setup_zeichnen, splash_zeichnen,
                            problem_texte, status_popup,
                            unavail_zeichnen, warnband_zeichnen)
    import panel_lang
    import panel_selfcheck


# ---------------------------------------------------------------------------
# Fenster für genau ein Panel
# ---------------------------------------------------------------------------
#: Schirme, die laut Auftrag existieren müssten, am Gerät aber fehlen
#: ({"panel": "pfd-fo", "schirm": 1}) — wird als Warnband gezeigt, statt die
#: Panels stillschweigend übereinander zu legen.
FEHLENDE_SCHIRME: list[dict] = []


def fehlenden_schirm_merken(panel_id: str, schirm: int) -> None:
    """Festhalten: dieses Panel gehört auf einen Schirm, den es nicht gibt."""
    eintrag = {"panel": str(panel_id), "schirm": int(schirm)}
    if eintrag not in FEHLENDE_SCHIRME:
        FEHLENDE_SCHIRME.append(eintrag)


def schirm_merker_leeren(panels) -> None:
    """Alte Befunde für diese Panels verwerfen.

    Sonst überlebt eine Meldung aus einem früheren Lauf, und nach dem
    Umkonfigurieren steht ein Warnband für einen Schirm auf dem Glas, der längst
    wieder da ist.  Es zählt nur, was das **aktuelle** Layout betrifft.
    """
    namen = {a.panel if hasattr(a, "panel") else str(a) for a in panels}
    FEHLENDE_SCHIRME[:] = [e for e in FEHLENDE_SCHIRME
                           if e.get("panel") not in namen]


def schirm_warnung(schirme_anzahl: int, panels=None) -> str:
    """Ein Satz für das Warnband (deutsch; der Viewer übersetzt nicht).

    Wichtig: das Panel wird **nicht** angezeigt (nicht auf Schirm 1 geschoben) —
    der Text sagt das auch so, sonst sucht man es am falschen Ort.

    ``panels`` grenzt ein, wen die Meldung betrifft: ein PFD-Fenster darf nicht
    das Warnband der ECAM-Leinwand tragen (und umgekehrt).  Ohne ``panels`` gilt
    alles Gemerkte.
    """
    befunde = list(FEHLENDE_SCHIRME)
    if panels is not None:
        namen = {a if isinstance(a, str) else getattr(a, "panel", "") for a in panels}
        befunde = [e for e in befunde if e.get("panel") in namen]
    if not befunde:
        return ""
    panel = befunde[0]["panel"]
    schirm = befunde[0]["schirm"]
    weitere = len(befunde) - 1
    text = (f"Schirm {schirm + 1} fehlt (Gerät meldet {schirme_anzahl}) - "
            f"{panel} wird nicht angezeigt")
    if weitere:
        text += f" (+{weitere} weiteres)"
    return text


class DiagFensterMixin:
    """Selbstauskunft (Diag-Tafel) und Bildschirmschoner — für beide Fensterarten.

    Panel- und Canvas-Fenster zeigen beide Bilder und beide können ohne Bild
    dastehen.  Ohne diese gemeinsame Grundlage dimmt nur die eine Hälfte ab —
    genau das war im Feld zu sehen (ECAM auf Leinwand blieb hell).
    """

    def _diag_grundwerte(self) -> None:
        # Bildschirmschoner: nach dieser Zeit ohne Bild wird alles abgedimmt
        self._dim_nach = float(os.environ.get("FDC_DIM_NACH", "45") or 45)
        self._dim_staerke = int(os.environ.get("FDC_DIM_ALPHA", "190") or 190)
        self._gestartet = time.monotonic()
        #: Selbstauskunft höchstens alle 15 s neu erheben (liest Dateien, fragt nmcli)
        self._bericht: Optional[object] = None
        self._bericht_stand = 0.0

    def _stream_aktiv(self):                     # von den Unterklassen gefüllt
        return None

    def selbstauskunft(self):
        """Diag-Bericht des Geräts (gecacht)."""
        jetzt = time.monotonic()
        if self._bericht is not None and (jetzt - self._bericht_stand) < 15.0:
            return self._bericht
        try:
            self._bericht = panel_selfcheck.laufender_bericht(
                stream_aktiv=self._stream_aktiv())
        except Exception:                        # noqa: BLE001 - Diagnose darf nie kippen
            self._bericht = None
        self._bericht_stand = jetzt
        return self._bericht

    def abdunkeln_wenn_noetig(self, maler, flaeche, sekunden: float) -> bool:
        if self._dim_nach <= 0 or self._dim_staerke <= 0:
            return False
        if sekunden < self._dim_nach:
            return False
        abdunkeln(maler, flaeche, self._dim_staerke)
        return True


class PanelFenster(DiagFensterMixin, QWidget):
    """Zeigt ein Panel bildschirmfüllend im zugewiesenen Rechteck."""

    #: Frame vom Netz-Thread in den GUI-Thread (Qt queued connection)
    frame_da = Signal(bytes, int, int)

    def __init__(self, panel_id: str, rect: tuple[int, int, int, int],
                 screen: int = 0, vollbild: bool = True,
                 strecken: bool = False, debug: bool = False,
                 scale: str = "contain", zoom: float = 1.0,
                 diag: bool = False, monitor: Optional["DiagMonitor"] = None) -> None:
        super().__init__()
        self.panel_id = panel_id
        #: Diag-Overlay (GlassOut/Sim/FDC/Bildrate/CPU) — vom FDC geschaltet
        self.diag = diag
        self.monitor = monitor
        #: contain = ganzes Bild sichtbar (Balken), stretch = verzerrt füllen,
        #: cover = füllen und Überstand abschneiden (für Ausschnitte im Panel)
        self.scale = "stretch" if strecken else (scale or "contain")
        self.zoom = max(0.1, min(4.0, float(zoom or 1.0)))
        self.strecken = self.scale == "stretch"
        self.debug = debug
        self._pixmap: Optional[QPixmap] = None
        self._bisher = 0
        self._kb_summe = 0.0
        self._fps_fenster: list[float] = []
        self._letzte_anzeige = 0.0
        self._letzter_frame = 0.0
        #: Im Testbetrieb kommt kein Netzwerkbild - dann auch keine Wartemeldung
        self._testbetrieb = False
        self._diag_grundwerte()
        # Ohne Tastatur/Maus am Client ist eine stumme schwarze Fläche ein
        # Blindflug: der Timer lässt die Wartemeldung auch dann erscheinen,
        # wenn gar keine Frames mehr kommen (und nichts ein Neuzeichnen auslöst).
        self._wache = QTimer(self)
        self._wache.timeout.connect(self.update)
        self._wache.start(1000)
        self.setWindowTitle(f"FDC – {panel_id}")
        self.setStyleSheet("background: black;")
        # Vollbild ohne Rahmen: kein Taskbar, kein Fokusklau
        if vollbild:
            self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        x, y, w, h = rect
        self.setGeometry(x, y, w, h)
        schirme = QApplication.screens()
        if 0 <= screen < len(schirme):
            self.setScreen(schirme[screen])
            if vollbild and (w, h) == (0, 0):
                self.setGeometry(schirme[screen].geometry())
        elif screen > 0:
            # Auftrag sagt Schirm 2, am Gerät hängt einer.  Merken — der
            # Viewer baut solche Fenster gar nicht mehr (siehe main()).  Wer
            # dieses Fenster trotzdem erzeugt, bekommt es auf dem Hauptschirm
            # ohne Ausweich-Logik zu sehen; die Anzeige ist dann bewusst die
            # des Aufrufers, nicht unsere stille Notlösung.
            fehlenden_schirm_merken(panel_id, screen)
        self.frame_da.connect(self._frame_anzeigen, Qt.QueuedConnection)

    # -- vom Netz ----------------------------------------------------------
    def frame_uebernehmen(self, jpeg: bytes, breite: int, hoehe: int) -> None:
        """Wird im Netz-Thread aufgerufen — nur weiterreichen."""
        self.frame_da.emit(bytes(jpeg), int(breite), int(hoehe))

    # -- in der GUI --------------------------------------------------------
    def _frame_anzeigen(self, jpeg: bytes, breite: int, hoehe: int) -> None:
        bild = QImage.fromData(jpeg, "JPEG")
        if bild.isNull():
            return
        bild_merken(self.panel_id)
        self._pixmap = QPixmap.fromImage(bild)
        self._bisher += 1
        self._kb_summe += len(jpeg) / 1024.0
        jetzt = time.monotonic()
        self._letzter_frame = jetzt
        self._fps_fenster.append(jetzt)
        self._fps_fenster = [t for t in self._fps_fenster if jetzt - t <= 2.0]
        self.update()

    def _wartemeldung(self) -> str:
        """Text für „kein Bild" — in der Sprache des FDC (panel_lang)."""
        code = panel_lang.sprache()
        if not self._letzter_frame:
            return f"{self.panel_id}\n{panel_lang.text('warte_erstes_bild', code)}"
        sekunden = time.monotonic() - self._letzter_frame
        if sekunden < 3.0:
            return ""
        if sekunden < 15.0:
            return (f"{self.panel_id}\n"
                    f"{panel_lang.text('kein_bild_seit', code, s=f'{sekunden:.0f}')}")
        return (f"{self.panel_id}\n"
                f"{panel_lang.text('kein_bild_seit', code, s=f'{sekunden:.0f}')}\n"
                f"{panel_lang.text('engine_frage', code)}")

    def _kein_bild_sekunden(self) -> float:
        """Sekunden ohne Bild — „nie eins bekommen" zählt ab Programmstart.

        Vorher stand hier ``0.0``, solange noch nie ein Frame ankam.  Damit
        griff der Bildschirmschoner genau dann nicht, wenn der Stream von
        Anfang an tot war (Engine aus, Modul fehlt) — also im einzigen Fall,
        für den er gedacht ist.
        """
        bezug = self._letzter_frame or self._gestartet
        return max(0.0, time.monotonic() - bezug)

    def _stream_aktiv(self):                     # noqa: D102 (Mixin-Haken)
        return bool(self._pixmap is not None and not self._pixmap.isNull()
                    and self._kein_bild_sekunden() < 5.0)

    def dimmen(self, maler: QPainter) -> None:
        """Bildschirmschoner: bei langer Bildlosigkeit alles abdimmen."""
        self.abdunkeln_wenn_noetig(maler, QRectF(self.rect()),
                                   self._kein_bild_sekunden())

    def zielrechteck(self) -> tuple[float, float, float, float]:
        """Wohin das Bild gezeichnet wird (x, y, w, h) im Fenster."""
        if self._pixmap is None or self._pixmap.isNull():
            return (0.0, 0.0, float(self.width()), float(self.height()))
        pw, ph = self._pixmap.width(), self._pixmap.height()
        if self.scale == "stretch":
            return (0.0, 0.0, float(self.width()), float(self.height()))
        if self.scale == "cover":
            faktor = max(self.width() / pw, self.height() / ph) * self.zoom
        else:                                    # contain
            faktor = min(self.width() / pw, self.height() / ph) * self.zoom
        w, h = pw * faktor, ph * faktor
        return ((self.width() - w) / 2.0, (self.height() - h) / 2.0, w, h)

    def paintEvent(self, _event) -> None:  # noqa: N802 (Qt-Namen)
        maler = QPainter(self)
        try:
            self._zeichnen(maler)
        finally:
            # Ohne finally bleibt der Painter nach einer Ausnahme offen: Qt
            # schreibt dann pro Frame "endPaint() called with active painter"
            # und die Fläche zeigt nichts mehr.
            maler.end()


    def _zeichnen(self, maler: QPainter) -> None:
        maler.fillRect(self.rect(), Qt.black)
        meldung = self._wartemeldung() if not self._testbetrieb else ""
        hinweis = setup_hinweis() if not self._testbetrieb else {}
        # Kommt eine Meldung, wird das letzte Bild NICHT mehr gezeichnet: sonst
        # steht ein Standbild der Instrumente unter „SYSTEMS UNAVAIL" und das
        # Panel sieht halb funktionsfähig aus (real so gesehen, nachdem
        # GlassOut wegbricht).  Ein blankes Feld mit Meldung ist die Wahrheit.
        if not meldung and not hinweis and self._pixmap is not None and not self._pixmap.isNull():
            maler.setRenderHint(QPainter.SmoothPixmapTransform, True)
            x, y, w, h = self.zielrechteck()
            # Überstand (cover/zoom) wird vom Fenster beschnitten - genau das
            # braucht man, wenn das Bild in einen MIP-Ausschnitt passen soll.
            maler.drawPixmap(QRectF(x, y, w, h), self._pixmap, QRectF(self._pixmap.rect()))
        if hinweis:
            # Kein Netz: der Client wartet auf die WLAN-Einrichtung über das
            # Handy — die Anleitung ist wichtiger als „SYSTEMS UNAVAIL".
            setup_zeichnen(maler, QRectF(self.rect()),
                           str(hinweis.get("ssid") or ""),
                           str(hinweis.get("url") or ""))
        elif meldung:
            # Erst prüfen, ob am Gerät selbst etwas kaputt ist (fehlendes Modul,
            # kein Schreibrecht, unplausible Uhr): dann zeigt der Bildschirm die
            # Diag-Tafel statt der knappen Meldung — ohne SSH ist das die
            # einzige Auskunft, die der Betreiber vor Ort bekommt.
            bericht = self.selbstauskunft()
            if bericht is not None and getattr(bericht, "fehler", False):
                diag_splash_zeichnen(maler, QRectF(self.rect()), bericht,
                                     panel_lang.sprache())
            else:
                code = panel_lang.sprache()
                unavail_zeichnen(maler, QRectF(self.rect()), self.panel_id,
                                 self._kein_bild_sekunden(),
                                 detail=panel_lang.text("keine_daten", code,
                                                        panel=self.panel_id),
                                 titel=panel_lang.text("unavail", code))
                self.dimmen(maler)
        elif self.diag and self.monitor is not None:
            diag_zeichnen(maler, QRectF(self.rect()), self.monitor.werte(),
                          fps=len(self._fps_fenster) / 2.0)
        if not meldung and not hinweis:
            # Bild läuft, aber am Gerät klemmt etwas (Uhr, Rechte, Module):
            # schmales Band, damit man es sieht ohne das Panel zu verdecken.
            bericht = self.selbstauskunft()
            if bericht is not None:
                code = panel_lang.sprache()
                texte = problem_texte(bericht, code)
                # Fehlender Schirm zuerst: das erklärt, warum zwei Panels
                # übereinander liegen — dringlicher als eine schiefe Uhr.
                fehlt = schirm_warnung(len(QApplication.screens()),
                                        [self.panel_id])
                if fehlt:
                    texte.insert(0, fehlt)
                if texte:
                    warnband_zeichnen(maler, QRectF(self.rect()),
                                      panel_lang.text("diag_warnband", code,
                                                      text=texte[0]))
        if not self._testbetrieb and not hinweis:
            # Update-Hinweis klein in die Ecke: es soll nicht wie eine Störung
            # des Panels aussehen, aber ohne Hinweis merkt niemand davon.
            from PySide6.QtGui import QColor as _QColor
            ota = ota_stand()
            os_zustand = os_stand()
            text = status_popup(ota, os_zustand)
            if text:
                # Grün = erledigt, Gelb = Handlung offen (Neustart/läuft), Rot = Fehler
                if "fehlgeschlagen" in text:
                    punkt = _QColor(255, 90, 90)
                elif "Neustart" in text:
                    punkt = _QColor(255, 193, 7)
                elif "fertig" in text or "installiert" in text:
                    punkt = _QColor(61, 220, 132)
                else:
                    punkt = _QColor(255, 193, 7)
                ota_popup(maler, QRectF(self.rect()), text, punkt)
        if self.debug:
            from PySide6.QtGui import QColor, QFont
            fps = len(self._fps_fenster) / 2.0
            kb = self._kb_summe / self._bisher if self._bisher else 0.0
            text = (f"{self.panel_id}   {fps:.1f} fps   {kb:.1f} KB/Bild   "
                    f"{self._bisher} Frames")
            schrift = QFont("DejaVu Sans", 16, QFont.Bold)
            maler.setFont(schrift)
            masse = maler.fontMetrics().boundingRect(text)
            kasten = masse.adjusted(-8, -6, 8, 6).translated(10, 8)
            # Dunkler Kasten dahinter: ohne den ist die Schrift über den
            # Farbbalken nicht lesbar (rot auf Weiß war der erste Versuch).
            maler.setPen(Qt.NoPen)
            maler.setBrush(QColor(0, 0, 0, 200))
            maler.drawRect(kasten)
            maler.setBrush(Qt.NoBrush)
            maler.setPen(QColor(255, 60, 60))
            maler.drawText(10, 8 + masse.height() - 2, text)


class CanvasFenster(DiagFensterMixin, QWidget):
    """Mehrere Panels auf *einer* gedrehten Fläche.

    Gedacht für den ECAM-Monitor, der physisch 45° schief im Panel sitzt: die
    Fläche trägt Upper- und Lower-ECAM in eigenen Rechtecken (oben/unten) und
    wird als Ganzes gedreht und in das Fenster eingepasst.  Nur so stimmen
    „oben" und „unten" wieder, wenn der Monitor schief hängt.

    Die Drehung wirkt auf die Fläche, nicht auf den Bildschirm: der Diag-Kasten
    wird danach im Bildschirmraum gezeichnet und bleibt gerade lesbar.
    """

    #: (panel_id, jpeg, breite, hoehe) vom Netz-Thread in den GUI-Thread
    frame_da = Signal(str, bytes, int, int)

    def __init__(self, rect: tuple[int, int, int, int], canvas_w: int, canvas_h: int,
                 rotate: float = 0.0, vollbild: bool = True, debug: bool = False,
                 monitor: Optional["DiagMonitor"] = None) -> None:
        super().__init__()
        self.canvas_w = max(1, int(canvas_w))
        self.canvas_h = max(1, int(canvas_h))
        self.rotate = float(rotate or 0.0)
        self.zoom = 1.0
        self.debug = debug
        self.monitor = monitor
        self.teile: dict[str, dict] = {}
        self._testbetrieb = False
        self._diag_grundwerte()
        self.setWindowTitle("FDC – Canvas")
        self.setStyleSheet("background: black;")
        if vollbild:
            self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        self.setGeometry(*rect)
        self._wache = QTimer(self)
        self._wache.timeout.connect(self.update)
        self._wache.start(1000)
        self.frame_da.connect(self._frame_anzeigen, Qt.QueuedConnection)

    # -- Aufbau ------------------------------------------------------------
    def teil_anlegen(self, panel_id: str, rect: tuple[int, int, int, int],
                     scale: str = "contain", diag: bool = False,
                     zoom: float = 1.0) -> None:
        self.teile[str(panel_id)] = {
            "rect": (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])),
            "scale": scale or "contain",
            "zoom": max(0.1, min(4.0, float(zoom or 1.0))),
            "pixmap": None, "letzter": 0.0, "bisher": 0,
            "kb": 0.0, "fenster": [], "diag": bool(diag),
        }

    # -- Netz --------------------------------------------------------------
    def frame_uebernehmen(self, panel_id: str, jpeg: bytes,
                          breite: int, hoehe: int) -> None:
        self.frame_da.emit(str(panel_id), bytes(jpeg), int(breite), int(hoehe))

    def _frame_anzeigen(self, panel_id: str, jpeg: bytes,
                        breite: int, hoehe: int) -> None:
        teil = self.teile.get(panel_id)
        if teil is None:
            return
        bild = QImage.fromData(jpeg, "JPEG")
        if bild.isNull():
            return
        bild_merken(panel_id)
        teil["pixmap"] = QPixmap.fromImage(bild)
        teil["bisher"] += 1
        teil["kb"] = teil["kb"] + len(jpeg) / 1024.0
        jetzt = time.monotonic()
        teil["letzter"] = jetzt
        teil["fenster"].append(jetzt)
        teil["fenster"] = [t for t in teil["fenster"] if jetzt - t <= 2.0]
        self.update()

    def bildrate(self, panel_id: str) -> float:
        teil = self.teile.get(panel_id)
        return len(teil["fenster"]) / 2.0 if teil else 0.0

    def keine_bild_sekunden(self, panel_id: str) -> float:
        """Sekunden ohne Bild für ein Teil — ohne je empfangenes Bild ab Start."""
        teil = self.teile.get(panel_id)
        if not teil:
            return 0.0
        bezug = teil["letzter"] or self._gestartet
        return max(0.0, time.monotonic() - bezug)

    def _stream_aktiv(self):                     # noqa: D102 (Mixin-Haken)
        if not self.teile:
            return None
        return any(self.keine_bild_sekunden(p) < 5.0 for p in self.teile)

    def laengste_stille(self) -> float:
        """Wie lange liefert keins der Teile mehr Bild?"""
        if not self.teile:
            return max(0.0, time.monotonic() - self._gestartet)
        return max(self.keine_bild_sekunden(p) for p in self.teile)

    # -- Zeichnen ----------------------------------------------------------
    def _transform(self, maler: QPainter) -> float:
        """Fläche so einpassen, dass das *gedrehte* Rechteck ins Fenster passt."""
        import math
        bogen = math.radians(self.rotate)
        cv, sv = abs(math.cos(bogen)), abs(math.sin(bogen))
        bbox_w = self.canvas_w * cv + self.canvas_h * sv
        bbox_h = self.canvas_w * sv + self.canvas_h * cv
        faktor = min(self.width() / bbox_w, self.height() / bbox_h) * self.zoom
        maler.translate(self.width() / 2.0, self.height() / 2.0)
        maler.rotate(self.rotate)
        maler.scale(faktor, faktor)
        maler.translate(-self.canvas_w / 2.0, -self.canvas_h / 2.0)
        return faktor

    @staticmethod
    def _teil_flaeche(teil: dict) -> QRectF:
        x, y, w, h = teil["rect"]
        pix = teil["pixmap"]
        if pix is None or pix.isNull() or teil["scale"] == "stretch":
            return QRectF(x, y, w, h)
        pw, ph = pix.width(), pix.height()
        if teil["scale"] == "cover":
            faktor = max(w / pw, h / ph) * teil["zoom"]
        else:
            faktor = min(w / pw, h / ph) * teil["zoom"]
        bw, bh = pw * faktor, ph * faktor
        return QRectF(x + (w - bw) / 2.0, y + (h - bh) / 2.0, bw, bh)

    def paintEvent(self, _event) -> None:  # noqa: N802 (Qt-Namen)
        maler = QPainter(self)
        try:
            self._zeichnen(maler)
        finally:
            maler.end()

    def _zeichnen(self, maler: QPainter) -> None:
        from PySide6.QtGui import QColor
        maler.fillRect(self.rect(), Qt.black)
        maler.setRenderHint(QPainter.SmoothPixmapTransform, True)
        maler.setRenderHint(QPainter.Antialiasing, True)
        maler.save()
        self._transform(maler)
        for panel_id, teil in self.teile.items():
            ziel = self._teil_flaeche(teil)
            if teil["pixmap"] is not None and not teil["pixmap"].isNull():
                maler.drawPixmap(ziel, teil["pixmap"],
                                 QRectF(teil["pixmap"].rect()))
            elif not self._testbetrieb:
                hinweis = setup_hinweis()
                if hinweis:
                    setup_zeichnen(maler, QRectF(*teil["rect"]),
                                   str(hinweis.get("ssid") or ""),
                                   str(hinweis.get("url") or ""))
                else:
                    bericht = self.selbstauskunft()
                    if bericht is not None and getattr(bericht, "fehler", False):
                        diag_splash_zeichnen(maler, QRectF(*teil["rect"]), bericht,
                                             panel_lang.sprache())
                    else:
                        unavail_zeichnen(maler, QRectF(*teil["rect"]), panel_id,
                                         self.keine_bild_sekunden(panel_id))
            # Fuge zwischen den Teilen sichtbar machen
            maler.setPen(QPen(QColor(28, 38, 48), 3))
            maler.setBrush(Qt.NoBrush)
            maler.drawRect(QRectF(*teil["rect"]))
        maler.restore()
        if not self._testbetrieb:
            for panel_id, teil in self.teile.items():
                if teil["diag"] and self.monitor is not None:
                    diag_zeichnen(maler, QRectF(self.rect()), self.monitor.werte(),
                                  fps=self.bildrate(panel_id))
                    break
            ota = ota_stand()
            os_zustand = os_stand()
            text = status_popup(ota, os_zustand)
            if text:
                if "fehlgeschlagen" in text:
                    punkt = QColor(255, 90, 90)
                elif "Neustart" in text:
                    punkt = QColor(255, 193, 7)
                elif "fertig" in text or "installiert" in text:
                    punkt = QColor(61, 220, 132)
                else:
                    punkt = QColor(255, 193, 7)
                ota_popup(maler, QRectF(self.rect()), text, punkt)
        # Geräteproblem oder fehlender Schirm: schmales Band, Bild bleibt
        bericht = self.selbstauskunft()
        texte = problem_texte(bericht, panel_lang.sprache()) if bericht else []
        fehlt = schirm_warnung(len(QApplication.screens()), list(self.teile))
        if fehlt:
            texte.insert(0, fehlt)
        if texte:
            warnband_zeichnen(maler, QRectF(self.rect()), texte[0])
        # Bildschirmschoner: liefert kein Teil mehr Bild, wird alles abgedimmt
        self.abdunkeln_wenn_noetig(maler, QRectF(self.rect()),
                                   self.laengste_stille())


class _Steuerung(QObject):
    """Verdrahtet je Panel eine Quelle mit ihrem Fenster."""

    def __init__(self) -> None:
        super().__init__()
        self.quellen: list[GlassOutSource] = []
        self.timer: list = []   # Testbild-Timer am Leben halten

    def anbauen(self, cfg: StreamConfig, fenster: PanelFenster) -> GlassOutSource:
        quelle = GlassOutSource(cfg.panel_id, cfg.engine_host, cfg.engine_port,
                                name=f"VIEW-{cfg.panel_id}",
                                crop=cfg.crop, target_fps=cfg.engine_fps,
                                log=lambda t: print(t, flush=True))
        quelle.start(lambda frame: fenster.frame_uebernehmen(frame.jpeg,
                                                             frame.width,
                                                             frame.height))
        self.quellen.append(quelle)
        return quelle

    def anbauen_canvas(self, cfg: StreamConfig, canvas: "CanvasFenster") -> GlassOutSource:
        """Wie ``anbauen``, aber mehrere Panels teilen sich eine Fläche."""
        quelle = GlassOutSource(cfg.panel_id, cfg.engine_host, cfg.engine_port,
                                name=f"VIEW-{cfg.panel_id}",
                                crop=cfg.crop, target_fps=cfg.engine_fps,
                                log=lambda t: print(t, flush=True))
        quelle.start(lambda frame: canvas.frame_uebernehmen(cfg.panel_id, frame.jpeg,
                                                            frame.width, frame.height))
        self.quellen.append(quelle)
        return quelle

    def testbild_anbauen(self, fenster: PanelFenster, fps: int = 15) -> QTimer:
        """Erzeugt Testbilder im Fenster — ohne Engine, ohne Netz."""
        from PySide6.QtGui import QColor, QFont, QPainter

        zaehler = {"n": 0}

        def tick() -> None:
            seite = 480
            bild = QImage(seite, seite, QImage.Format_RGB32)
            bild.fill(QColor(10, 18, 12))
            maler = QPainter(bild)
            maler.setRenderHint(QPainter.Antialiasing, True)
            # Farbbalken oben: prüft, ob Farben stimmen (der alte Fehlerbild-Test)
            farben = [(255, 255, 255), (255, 0, 255), (255, 255, 0), (255, 0, 0),
                      (0, 255, 255), (0, 0, 255), (0, 255, 0)]
            breite = seite // len(farben)
            for i, (r, g, b) in enumerate(farben):
                maler.fillRect(i * breite, 0, breite, 60, QColor(r, g, b))
            # Kreis + Zeiger: prüft Skalierung und Seitenverhältnis.
            # Der Zeiger MUSS aus der Kreismitte kommen — vorher lag der
            # Ursprung 45 px zu tief, das sah nach einem Anzeigefehler aus,
            # war aber ein Rechenfehler im Testbild.
            import math
            maler.setPen(QColor(0, 220, 255))
            cx = cy = seite // 2
            radius = (seite // 2) - 70          # lässt die Farbbalken oben frei
            maler.drawEllipse(cx - radius, cy - radius, 2 * radius, 2 * radius)
            winkel = (zaehler["n"] * 6) % 360
            maler.drawLine(cx, cy, int(cx + radius * math.sin(math.radians(winkel))),
                           int(cy - radius * math.cos(math.radians(winkel))))
            # Mitte markieren: dann sieht man sofort, ob eine Skalierung zieht
            maler.setPen(QColor(255, 255, 255))
            maler.drawLine(cx - 12, cy, cx + 12, cy)
            maler.drawLine(cx, cy - 12, cx, cy + 12)
            maler.setPen(QColor(255, 220, 0))
            maler.setFont(QFont("DejaVu Sans", 16, QFont.Bold))
            maler.drawText(20, seite - 40, f"TESTBILD {zaehler['n']:04d}")
            maler.drawText(20, seite - 14, f"{fps} fps  {seite}x{seite}")

            # Ausrichthilfen: der Rahmen im Panel verdeckt die Bildschirmkante,
            # deshalb Eckenwinkel und Kantenmitten zeichnen — damit lässt sich
            # der sichtbare Ausschnitt genau einmessen.
            rand = 2
            maler.setPen(QColor(0, 180, 90))
            maler.drawRect(rand, rand, seite - 2 * rand - 1, seite - 2 * rand - 1)
            weiss = QColor(255, 255, 255)
            lang, dick = 54, 3
            for (x, y, dx, dy) in ((rand, rand, 1, 1),
                                   (seite - rand - 1, rand, -1, 1),
                                   (rand, seite - rand - 1, 1, -1),
                                   (seite - rand - 1, seite - rand - 1, -1, -1)):
                maler.fillRect(x if dx > 0 else x - lang, y, lang, dick, weiss)   # waagerecht
                maler.fillRect(x if dx > 0 else x - dick, y if dy > 0 else y - lang,
                               dick, lang, weiss)                                 # senkrecht
            # Kantenmitten: zeigen die Bildmitte jeder Seite (Rahmen-Symmetrie)
            mitte, tick = seite // 2, 18
            for (x, y, w, h) in ((mitte - dick // 2, rand, dick, tick),
                                 (mitte - dick // 2, seite - rand - tick, dick, tick),
                                 (rand, mitte - dick // 2, tick, dick),
                                 (seite - rand - tick, mitte - dick // 2, tick, dick)):
                maler.fillRect(x, y, w, h, weiss)
            maler.end()
            puffer = QBuffer()
            puffer.open(QBuffer.WriteOnly)
            bild.save(puffer, "JPG", 85)
            zaehler["n"] += 1
            fenster.frame_uebernehmen(bytes(puffer.data()), seite, seite)

        timer = QTimer()
        timer.timeout.connect(tick)
        timer.start(max(50, int(1000 / max(1, fps))))
        tick()
        return timer

    def stop(self) -> None:
        for quelle in self.quellen:
            try:
                quelle.stop()
            except Exception:
                pass
        self.quellen.clear()


# ---------------------------------------------------------------------------
# Konfiguration
# ---------------------------------------------------------------------------
@dataclass
class PanelAuftrag:
    panel: str
    rect: tuple[int, int, int, int] = (0, 0, 0, 0)
    screen: int = 0
    crop: Optional[tuple[int, int, int, int]] = None
    fps: int = 15
    engine_fps: int = 15
    strecken: bool = False
    #: Kein Netz: der Viewer erzeugt selbst Testbilder.  Damit lassen sich X, Qt,
    #: Skalierung, Cursor und Blanking prüfen, wenn Sim oder Engine nicht laufen —
    #: auch für Kunden die erste Funktionskontrolle.
    test: bool = False
    #: Platzierung: full | left | right | top | bottom (rect gewinnt, wenn gesetzt)
    position: str = "full"
    #: Anteil der Bildschirmbreite/-höhe, den dieses Panel bekommt (0.1 … 1.0)
    size: float = 1.0
    #: Bildfüllung: contain (ganz sichtbar) | stretch (verzerrt) | cover (füllen, beschneiden)
    scale: str = "contain"
    #: Feinjustage der Skalierung, z. B. 1.05 um 5 % in den Ausschnitt zu zoomen
    zoom: float = 1.0
    #: Zielgröße in Pixeln; 0 = aus position/size ableiten
    width: int = 0
    height: int = 0
    #: Verschiebung in Pixeln nach dem Platzieren (Feinjustage am Ausschnitt)
    offset_x: int = 0
    offset_y: int = 0
    #: Diag-Overlay auf diesem Panel (GlassOut/Sim/FDC/Bildrate/CPU)
    diag: bool = False
    #: Dieses Panel liegt auf der gedrehten Leinwand statt in einem eigenen Fenster
    auf_leinwand: bool = False


def rect_berechnen(auftrag: "PanelAuftrag",
                   schirm: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    """Rechteck im Bildschirm aus Position + Anteil ableiten.

    ``schirm`` ist (x, y, w, h) des Zielbildschirms — bei mehreren Monitoren
    versetzt, deshalb werden die Offsets mitgenommen.  Eine ausdrückliche
    ``rect``-Angabe schlägt Position und Anteil (volle Kontrolle).
    """
    sx, sy, sw, sh = (int(v) for v in schirm)
    if auftrag.rect and auftrag.rect[2] > 0 and auftrag.rect[3] > 0:
        return (int(auftrag.rect[0]), int(auftrag.rect[1]),
                int(auftrag.rect[2]), int(auftrag.rect[3]))
    anteil = max(0.05, min(1.0, float(auftrag.size or 1.0)))
    pos = str(auftrag.position or "full").strip().lower()
    if pos == "left":
        x, y, breite, hoehe = sx, sy, int(sw * anteil), sh
    elif pos == "right":
        breite = int(sw * anteil)
        x, y, hoehe = sx + sw - breite, sy, sh
    elif pos == "top":
        x, y, breite, hoehe = sx, sy, sw, int(sh * anteil)
    elif pos == "bottom":
        hoehe = int(sh * anteil)
        x, y, breite = sx, sy + sh - hoehe, sw
    else:
        x, y, breite, hoehe = sx, sy, sw, sh
    # Pixelmaße schlagen den Anteil: der Anwender will „960x1080", nicht „null Komma fünf"
    if auftrag.width > 0:
        breite = int(auftrag.width)
    if auftrag.height > 0:
        hoehe = int(auftrag.height)
    # Offset zuletzt: verschiebt das fertige Rechteck (Feinjustage am Ausschnitt)
    return (x + int(auftrag.offset_x), y + int(auftrag.offset_y), breite, hoehe)


@dataclass
class Layout:
    engine_host: str = "127.0.0.1"
    engine_port: int = 8787
    vollbild: bool = True
    debug: bool = False
    panels: list[PanelAuftrag] = field(default_factory=list)
    #: "windows" = je Panel ein Fenster; "canvas" = eine gedrehte Fläche
    mode: str = "windows"
    canvas_w: int = 0
    canvas_h: int = 0
    #: Drehung der Fläche in Grad (Monitor schief im Panel -> 45)
    rotate: float = 0.0
    #: Bildschirm für die Fläche (-1 = derselbe wie das erste Panel)
    canvas_screen: int = -1
    #: Mehrere Leinwände — **eine je Schirm**: [{"screen", "width", "height",
    #: "rotate"}].  Der alte Einzel-Fall (``canvas`` + ``canvas_screen``) bleibt
    #: gültig und wird wie eine Leinwand behandelt.
    leinwaende: list = field(default_factory=list)


def _rect(text: str) -> tuple[int, int, int, int]:
    teile = [int(t) for t in str(text).replace(" ", "").split(",")]
    if len(teile) != 4:
        raise argparse.ArgumentTypeError("Rechteck braucht x,y,w,h")
    return (teile[0], teile[1], teile[2], teile[3])


def _crop(text: str) -> tuple[int, int, int, int]:
    return _rect(text)


def _engine(text: str) -> tuple[str, int]:
    host, _, port = str(text).partition(":")
    return (host or "127.0.0.1", int(port or 8787))


def layout_aus_datei(pfad: str) -> Layout:
    with open(pfad, encoding="utf-8") as datei:
        roh = json.load(datei)
    host, port = _engine(str(roh.get("engine") or "127.0.0.1:8787"))
    flaeche = roh.get("canvas") or {}
    leinwaende: list = []
    for eintrag in (roh.get("leinwaende") or []):
        if not isinstance(eintrag, dict):
            continue
        try:
            rahmen = {"screen": int(eintrag.get("screen") or 0),
                      "width": int(eintrag.get("width") or 0),
                      "height": int(eintrag.get("height") or 0),
                      "rotate": float(eintrag.get("rotate") or 0.0)}
        except (TypeError, ValueError):
            continue
        if rahmen["width"] > 0 and rahmen["height"] > 0:
            leinwaende.append(rahmen)
    layout = Layout(engine_host=host, engine_port=port,
                    vollbild=bool(roh.get("vollbild", True)),
                    debug=bool(roh.get("debug", False)),
                    mode=str(roh.get("mode") or ("canvas" if flaeche else "windows")).lower(),
                    canvas_w=int(flaeche.get("width") or 0),
                    canvas_h=int(flaeche.get("height") or 0),
                    rotate=float(flaeche.get("rotate") or 0.0),
                    canvas_screen=int(flaeche.get("screen", -1)),
                    leinwaende=leinwaende)
    for eintrag in roh.get("panels") or []:
        layout.panels.append(PanelAuftrag(
            panel=str(eintrag["panel"]),
            rect=tuple(int(v) for v in (eintrag.get("rect") or [0, 0, 0, 0]))[:4],  # type: ignore[arg-type]
            screen=int(eintrag.get("screen") or 0),
            crop=(tuple(int(v) for v in eintrag["crop"])[:4]
                  if eintrag.get("crop") else None),
            fps=int(eintrag.get("fps") or 15),
            engine_fps=int(eintrag.get("engine_fps") or eintrag.get("fps") or 15),
            strecken=bool(eintrag.get("strecken", False)),
            test=bool(eintrag.get("test", False)),
            position=str(eintrag.get("position") or "full"),
            width=int(eintrag.get("width") or 0),
            height=int(eintrag.get("height") or 0),
            offset_x=int(eintrag.get("offset_x") or eintrag.get("x") or 0),
            offset_y=int(eintrag.get("offset_y") or eintrag.get("y") or 0),
            diag=bool(eintrag.get("diag", False)),
            auf_leinwand=bool(eintrag.get("canvas", False)),
            size=float(eintrag.get("size") or 1.0),
            scale=str(eintrag.get("scale")
                      or ("stretch" if eintrag.get("strecken") else "contain")),
            zoom=float(eintrag.get("zoom") or 1.0),
        ))
    return layout


def layout_aus_argumenten(argv: list[str]) -> Layout:
    p = argparse.ArgumentParser(description="GlassOut-Panels im Vollbild anzeigen")
    p.add_argument("--engine", type=_engine, default=("127.0.0.1", 8787),
                   metavar="HOST[:PORT]", help="Adresse der GlassOut-Engine")
    p.add_argument("--layout", help="JSON-Layoutdatei (statt --panel/--rect)")
    p.add_argument("--panel", action="append", default=[],
                   help="Panel-ID, mehrfach für mehrere Panels")
    p.add_argument("--rect", type=_rect, action="append", default=[],
                   help="x,y,w,h je Panel (Reihenfolge wie --panel)")
    p.add_argument("--screen", type=int, action="append", default=[],
                   help="Bildschirmnummer je Panel (Default 0)")
    p.add_argument("--crop", type=_crop, action="append", default=[],
                   help="Ausschnitt in Quellpixeln je Panel: x,y,w,h")
    p.add_argument("--fps", type=int, default=15, help="Anzeige Rate (Engine: 10-120)")
    p.add_argument("--strecken", action="store_true", help="Seitenverhältnis ignorieren")
    p.add_argument("--fenster", action="store_true", help="nicht Vollbild (Debug am Desktop)")
    p.add_argument("--debug", action="store_true", help="fps/KB-Overlay einblenden")
    p.add_argument("--test", action="store_true",
                   help="Testbilder statt Netzwerk (X/Qt/Skalierung prüfen)")
    p.add_argument("--position", action="append", default=[],
                   choices=["full", "left", "right", "top", "bottom"],
                   help="Platzierung je Panel (left/right/top/bottom/full)")
    p.add_argument("--size", action="append", type=float, default=[],
                   help="Anteil des Bildschirms je Panel (0.1 … 1.0)")
    p.add_argument("--scale", action="append", default=[],
                   choices=["contain", "stretch", "cover"],
                   help="Bildfüllung: contain (ganz) / stretch (verzerrt) / cover (beschneiden)")
    p.add_argument("--zoom", action="append", type=float, default=[],
                   help="Feinjustage der Skalierung, z. B. 1.05")
    p.add_argument("--width", action="append", type=int, default=[],
                   help="Zielbreite in Pixeln")
    p.add_argument("--height", action="append", type=int, default=[],
                   help="Zielhöhe in Pixeln")
    p.add_argument("--offset-x", action="append", type=int, default=[],
                   help="Verschiebung in x (Pixel)")
    p.add_argument("--offset-y", action="append", type=int, default=[],
                   help="Verschiebung in y (Pixel)")
    args = p.parse_args(argv)

    if args.layout:
        return layout_aus_datei(args.layout)

    host, port = args.engine
    layout = Layout(engine_host=host, engine_port=port,
                    vollbild=not args.fenster, debug=args.debug)
    if not args.panel:
        p.error("mindestens ein --panel oder --layout nötig")
    for i, panel in enumerate(args.panel):
        layout.panels.append(PanelAuftrag(
            panel=panel,
            rect=args.rect[i] if i < len(args.rect) else (0, 0, 0, 0),
            screen=args.screen[i] if i < len(args.screen) else 0,
            crop=args.crop[i] if i < len(args.crop) else None,
            fps=args.fps, engine_fps=args.fps, strecken=args.strecken,
            test=args.test,
            position=args.position[i] if i < len(args.position) else "full",
            size=args.size[i] if i < len(args.size) else 1.0,
            scale=args.scale[i] if i < len(args.scale)
                  else ("stretch" if args.strecken else "contain"),
            zoom=args.zoom[i] if i < len(args.zoom) else 1.0,
            width=args.width[i] if i < len(args.width) else 0,
            height=args.height[i] if i < len(args.height) else 0,
            offset_x=args.offset_x[i] if i < len(args.offset_x) else 0,
            offset_y=args.offset_y[i] if i < len(args.offset_y) else 0))
    return layout


#: Das Auto-Power-off (fdc-poweroff.service) fragt hier nach, ob noch Bilder
#: kommen.  Bewusst /tmp: der Viewer laeuft als ``ssf`` und darf in /run nichts
#: anlegen.  Geschrieben wird hoechstens alle 5 s, nicht pro Frame.
STREAM_STATE = "/tmp/fdc-stream.json"
_letzte_merkung = 0.0


def bild_merken(panel: str = "") -> None:
    """Letztes ankommendes Bild vermerken (fuer Auto-Power-off)."""
    global _letzte_merkung
    jetzt = time.time()
    if jetzt - _letzte_merkung < 5.0:
        return
    _letzte_merkung = jetzt
    try:
        with open(STREAM_STATE, "w", encoding="utf-8") as datei:
            json.dump({"letztes_bild": jetzt, "panel": panel}, datei)
    except OSError:
        pass


def _leinwand_bauen(layout: Layout, steuerung: _Steuerung, schirme,
                    monitor: Optional[DiagMonitor]) -> list["CanvasFenster"]:
    """Leinwände bauen — **eine je Schirm**.

    Gedacht für den ECAM-Monitor, der 45° schief im Panel sitzt: eine Fläche
    trägt mehrere Panels, wird als Ganzes gedreht und eingepasst.  Zwei Monitore
    heißen zwei Leinwände (``leinwaende``), jede mit eigener Größe und Drehung;
    der alte Einzel-Fall (``canvas`` + ``screen``) bleibt gültig.

    Ein Schirm, den das Gerät nicht meldet, bekommt **keine** Leinwand: nichts
    anzeigen ist besser, als die Panels auf Schirm 1 zu überdecken.
    """
    teile = [a for a in layout.panels if a.auf_leinwand]
    schirm_merker_leeren(teile)          # nur die eigenen Panels
    if not teile:
        return []

    flaechen = list(layout.leinwaende or [])
    if not flaechen:
        flaechen = [{"screen": layout.canvas_screen if layout.canvas_screen >= 0 else 0,
                     "width": layout.canvas_w, "height": layout.canvas_h,
                     "rotate": layout.rotate}]

    gebaut: list["CanvasFenster"] = []
    for flaeche in flaechen:
        schirm_idx = int(flaeche.get("screen") or 0)
        meine = [a for a in teile if a.screen == schirm_idx]
        if not meine:
            continue
        if schirme and not (0 <= schirm_idx < len(schirme)):
            for auftrag in meine:
                fehlenden_schirm_merken(auftrag.panel, schirm_idx)
            print(f"[viewer] Schirm {schirm_idx + 1} gibt es nicht "
                  f"({len(schirme)} vorhanden) - Leinwand bleibt aus "
                  f"({', '.join(a.panel for a in meine)})", flush=True)
            continue
        idx = schirm_idx if 0 <= schirm_idx < len(schirme) else 0
        geo = schirme[idx].geometry() if schirme else None
        breite = geo.width() if geo else 1920
        hoehe = geo.height() if geo else 1080
        canvas_w = int(flaeche.get("width") or 0) or breite
        canvas_h = int(flaeche.get("height") or 0) or hoehe
        canvas = CanvasFenster(
            (geo.x(), geo.y(), breite, hoehe) if geo else (0, 0, breite, hoehe),
            canvas_w, canvas_h, rotate=float(flaeche.get("rotate") or 0.0),
            vollbild=layout.vollbild, debug=layout.debug, monitor=monitor)
        standard_hoehe = canvas_h // max(1, len(meine))
        for auftrag in meine:
            rect = (auftrag.offset_x, auftrag.offset_y,
                    auftrag.width or canvas_w,
                    auftrag.height or standard_hoehe)
            canvas.teil_anlegen(auftrag.panel, rect, scale=auftrag.scale,
                                diag=auftrag.diag, zoom=auftrag.zoom)
        for auftrag in meine:
            if auftrag.test:
                canvas._testbetrieb = True
                continue
            cfg = StreamConfig(slug=auftrag.panel, panel_id=auftrag.panel,
                               width=auftrag.width or 768, height=auftrag.height or 768,
                               fps=float(auftrag.fps), source="glassout",
                               engine_host=layout.engine_host,
                               engine_port=layout.engine_port,
                               engine_fps=auftrag.engine_fps, crop=auftrag.crop,
                               crop_remote=True)
            steuerung.anbauen_canvas(cfg, canvas)
        canvas.show()
        print(f"[viewer] Leinwand Schirm {schirm_idx + 1} {canvas_w}x{canvas_h} "
              f"gedreht {float(flaeche.get('rotate') or 0.0):g}° mit "
              f"{len(meine)} Teil(en): {', '.join(a.panel for a in meine)}",
              flush=True)
        gebaut.append(canvas)
    return gebaut


def eigene_ip() -> str:
    """Eigene Adresse ohne Netzzugriff ermitteln (für den Einrichtungs-Splash)."""
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("10.255.255.255", 1))          # verbindet nicht wirklich
        return str(sock.getsockname()[0])
    except OSError:
        return ""
    finally:
        if sock is not None:
            sock.close()


class SetupFenster(QWidget):
    """Vollbild-Hinweis, solange das Panel im FDC nicht eingerichtet ist.

    Der Viewer muss auch ohne Konfiguration laufen: der Agent meldet sich über
    ihn (Status, OTA, Bild fürs FDC-Panel-Fenster). Ohne Hinweis sieht ein
    frisch installierter Client wie ein kaputtes Gerät aus — deshalb „Bitte
    Panel im FDC einrichten" in der Sprache des FDC (Default Englisch).
    """

    def __init__(self, adresse: str = "", meldung: str = "") -> None:
        super().__init__()
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setStyleSheet("background:#000;")
        self.setCursor(Qt.BlankCursor)
        self.adresse = adresse
        #: Grund, warum nichts angezeigt wird (z. B. fehlender Schirm) — dann
        #: NICHT „bitte einrichten" behaupten, sondern den echten Grund nennen.
        self.meldung = meldung
        self._letzte_sprache = ""
        # Ohne Neuzeichnen bliebe ein Wechsel der Sprache unsichtbar
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(1000)

    def paintEvent(self, _ereignis) -> None:            # noqa: N802 (Qt)
        maler = QPainter(self)
        try:
            maler.fillRect(self.rect(), Qt.black)
            code = panel_lang.sprache()
            self._letzte_sprache = code
            splash_zeichnen(maler, QRectF(self.rect()), code, self.adresse)
            ota = ota_stand()
            os_zustand = os_stand()
            text = status_popup(ota, os_zustand)
            if text:
                from PySide6.QtGui import QColor
                punkt = (QColor(255, 90, 90) if "fehlgeschlagen" in text
                         else QColor(255, 193, 7) if "Neustart" in text
                         else QColor(61, 220, 132) if "fertig" in text
                         or "installiert" in text else QColor(255, 193, 7))
                ota_popup(maler, QRectF(self.rect()), text, punkt)
            # Steht ein Grund an, warum nichts angezeigt wird (fehlender
            # Schirm), gehört der auf den Schirm — „bitte einrichten" wäre
            # schlicht falsch.  Keine Selbstauskunft, kein Dimmen: dieser
            # Hinweis hat keine Panels, er *ist* die Anzeige.
            grund = getattr(self, "meldung", "") or schirm_warnung(
                len(QApplication.screens()))
            if grund:
                warnband_zeichnen(maler, QRectF(self.rect()), grund)
        finally:
            maler.end()


def fenster_bauen(layout: "Layout", schirme, monitor=None, steuerung=None):
    """Panels ohne Leinwand als Fenster anlegen — Rückgabe [(auftrag, fenster, rect)].

    Ein Panel, dessen Schirm das Gerät nicht meldet, wird **nicht** gebaut: die
    Konfiguration bleibt gültig (das Gerät speichert sie), aber ein nicht
    angeschlossener Monitor zeigt keine Panels, statt sie auf Schirm 1
    übereinanderzulegen.  Gemerkt wird es trotzdem (`FEHLENDE_SCHIRME`), damit
    der Betreiber am Bildschirm sieht, warum etwas fehlt.
    """
    fenster_teile = [a for a in layout.panels if not a.auf_leinwand]
    schirm_merker_leeren(fenster_teile)  # nur die eigenen Panels
    gebaut: list = []
    for auftrag in fenster_teile:
        if schirme and not (0 <= auftrag.screen < len(schirme)):
            fehlenden_schirm_merken(auftrag.panel, auftrag.screen)
            print(f"[viewer] Schirm {auftrag.screen + 1} gibt es nicht "
                  f"({len(schirme)} vorhanden) - {auftrag.panel} bleibt aus",
                  flush=True)
            continue
        idx = auftrag.screen if 0 <= auftrag.screen < len(schirme) else 0
        geo = schirme[idx].geometry() if schirme else None
        schirm = ((geo.x(), geo.y(), geo.width(), geo.height()) if geo
                  else (0, 0, 1920, 1080))
        rect = rect_berechnen(auftrag, schirm)
        win = PanelFenster(auftrag.panel, rect, auftrag.screen,
                           vollbild=layout.vollbild, strecken=auftrag.strecken,
                           debug=layout.debug, scale=auftrag.scale,
                           zoom=auftrag.zoom, diag=auftrag.diag, monitor=monitor)
        gebaut.append((auftrag, win, rect))
    return gebaut


def main(argv: Optional[list[str]] = None) -> int:
    layout = layout_aus_argumenten(list(argv if argv is not None else sys.argv[1:]))
    app = QApplication(sys.argv[:1])
    app.setOverrideCursor(Qt.BlankCursor)
    steuerung = _Steuerung()
    schirme = QApplication.screens()
    monitor: Optional[DiagMonitor] = None
    if any(a.diag for a in layout.panels):
        # Ein Monitor für alle Fenster: fragt die Engine und misst die CPU
        monitor = DiagMonitor(layout.engine_host, layout.engine_port)
        monitor.starten()
    # Panels mit "canvas": true gehören auf die gedrehte Leinwand, alle anderen
    # in ein eigenes Fenster.  Beides gleichzeitig ist der Normalfall: PFD/ND als
    # Fenster auf Schirm 0, die ECAM-Leinwand auf Schirm 1.
    gebaut = fenster_bauen(layout, schirme, monitor, steuerung)
    fenster = [win for _, win, _ in gebaut]
    for auftrag, win, rect in gebaut:
        cfg = StreamConfig(slug=auftrag.panel, panel_id=auftrag.panel,
                           width=rect[2] or 1024, height=rect[3] or 1024,
                           fps=float(auftrag.fps), source="glassout",
                           engine_host=layout.engine_host,
                           engine_port=layout.engine_port,
                           engine_fps=auftrag.engine_fps, crop=auftrag.crop,
                           crop_remote=True)
        if auftrag.test:
            win._testbetrieb = True
            timer = steuerung.testbild_anbauen(win, auftrag.fps)
            steuerung.timer.append(timer)
        else:
            steuerung.anbauen(cfg, win)
        win.show()
        print(f"[viewer] {auftrag.panel} -> Schirm {auftrag.screen} "
              f"{rect[0]},{rect[1]} {rect[2]}x{rect[3]} pos={auftrag.position} "
              f"scale={auftrag.scale} @ {auftrag.fps} fps "
              f"von {layout.engine_host}:{layout.engine_port}", flush=True)
    leinwaende_gebaut = _leinwand_bauen(layout, steuerung, schirme, monitor)
    if not fenster and not leinwaende_gebaut:
        # Nichts zu zeigen.  Zwei sehr verschiedene Fälle:
        #  * wirklich nichts eingerichtet -> Einrichtungs-Hinweis
        #  * Panels sind eingerichtet, aber ihr Schirm fehlt -> NICHT „bitte
        #    einrichten" behaupten, sondern sagen, welcher Schirm fehlt.
        eingerichtet = bool(layout.panels)
        meldung = schirm_warnung(len(schirme)) if eingerichtet else ""
        splash = SetupFenster(adresse=eigene_ip(), meldung=meldung)
        if schirme:
            splash.setGeometry(schirme[0].geometry())
        splash.showFullScreen()
        print("[viewer] " + ("Panel-Schirm fehlt — Hinweis aktiv"
                             if eingerichtet else
                             "kein Panel eingerichtet — Einrichtungs-Hinweis aktiv"),
              flush=True)
    # Sauber beenden, damit die Netz-Threads nicht hängen bleiben
    app.aboutToQuit.connect(steuerung.stop)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
