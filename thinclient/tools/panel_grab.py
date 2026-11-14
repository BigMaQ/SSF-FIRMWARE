#!/usr/bin/env python3
"""Bildschirmfoto des Anzeige-Clients — Grundlage für den Live-Picker im FDC.

Wird vom Agenten als Unterprozess gestartet (``GET /screen``).  Bewusst ein
eigener Prozess: der Agent läuft ohne Display-Verbindung, und eine Qt-Anwendung
lässt sich nicht sauber nebenher in einem HTTP-Thread hochziehen.

Aufruf:
    panel_grab.py --display 0 --ausgabe /tmp/bild.png
    panel_grab.py --display 1            # PNG nach stdout (für HTTP)
"""
from __future__ import annotations

import argparse
import os
import sys


def bildschirmfoto(display: int = 0) -> bytes:
    """PNG-Bytes des angegebenen Bildschirms."""
    os.environ.setdefault("DISPLAY", f":{int(display)}")
    # Kein X? Dann liefert Qt ein leeres Bild — der Aufrufer meldet das als Fehler.
    from PySide6.QtCore import QBuffer
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    schirme = app.screens()
    if not schirme:
        raise SystemExit("kein Bildschirm gefunden (läuft X auf diesem Display?)")
    index = int(display) if 0 <= int(display) < len(schirme) else 0
    pixmap = schirme[index].grabWindow(0)
    if pixmap.isNull():
        raise SystemExit("Bildschirmfoto leer — greift der X-Server auf :%d?" % display)
    puffer = QBuffer()
    puffer.open(QBuffer.WriteOnly)
    if not pixmap.save(puffer, "PNG"):
        raise SystemExit("PNG konnte nicht geschrieben werden")
    return bytes(puffer.data())


def main(argv: list[str] | None = None) -> int:
    zerleger = argparse.ArgumentParser(description="Bildschirmfoto des Clients")
    zerleger.add_argument("--display", type=int, default=0,
                          help="Bildschirmindex (Default 0)")
    zerleger.add_argument("--ausgabe", default="-",
                          help="Zieldatei, '-' für stdout (Default)")
    args = zerleger.parse_args(argv)

    daten = bildschirmfoto(args.display)
    if args.ausgabe in ("-", ""):
        sys.stdout.buffer.write(daten)
        sys.stdout.buffer.flush()
    else:
        with open(args.ausgabe, "wb") as datei:
            datei.write(daten)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
