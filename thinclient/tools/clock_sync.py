"""Uhrzeit eines Anzeige-Clients prüfen und notfalls stellen.

Hintergrund aus dem Feld: ein Thin Client stand 216 Tage in der Vergangenheit
(leere CMOS-Batterie, kein NTP im VLAN). Folge war nicht „nur" eine falsche
Anzeige, sondern ein **kaputter OTA-Weg**:

    URLError: [SSL: CERTIFICATE_VERIFY_FAILED] certificate is not yet valid

Das TLS-Zertifikat ist aus Sicht der Box noch nicht gültig — jedes Update läuft
ins Leere, und `apt` prüft seit Debian 12 ebenfalls `NotBefore`. Deshalb hier
eine Uhrzeit-Quelle, die **ohne TLS** funktioniert: der `Date:`-Kopf einer
normalen HTTP-Antwort. Damit kristallisiert sich das Problem selbst heraus.

Aufruf auf dem Client (als root, siehe ``fdc-clock.service``)::

    python3 /opt/fdc/tools/clock_sync.py --korrigieren

Rückgabe: 0 = Uhr stimmt (oder wurde gestellt), 2 = Abweichung zu groß und
nicht korrigierbar (kein root), 1 = keine Quelle erreichbar.
"""
from __future__ import annotations

import email.utils
import subprocess
import sys
import time
import urllib.request
from typing import Optional

#: Ab dieser Abweichung (Sekunden) gilt die Uhr als falsch. Zertifikate und
#: apt vertragen ein paar Minuten Toleranz, Tage nicht.
SCHWELLE = 300

#: Quellen ohne TLS — Reihenfolge: erst das eigene Umfeld, dann allgemein.
QUELLEN = ("http://deb.debian.org/debian/README",
           "http://gitlab.pcq-online.com",
           "http://www.debian.org")


def kopfzeit(quelle: str, timeout: float = 6.0) -> Optional[int]:
    """``Date:``-Kopf einer HTTP-Antwort als Unix-Zeit (None = nicht erreichbar)."""
    try:
        with urllib.request.urlopen(quelle, timeout=timeout) as antwort:
            kopf = antwort.headers.get("Date")
    except Exception:                       # noqa: BLE001 — Netz darf nie werfen
        return None
    if not kopf:
        return None
    try:
        return int(email.utils.parsedate_to_datetime(kopf).timestamp())
    except (TypeError, ValueError):
        return None


def zeit_holen(quellen=QUELLEN, timeout: float = 6.0) -> Optional[int]:
    """Erste erreichbare Quelle liefert die Zeit."""
    for quelle in quellen:
        wert = kopfzeit(quelle, timeout=timeout)
        if wert:
            return wert
    return None


def abweichung(jetzt: Optional[float] = None, zeit: Optional[int] = None) -> Optional[int]:
    """Sekunden, die die lokale Uhr daneben liegt (None = keine Zeit bekommen)."""
    if zeit is None:
        zeit = zeit_holen()
    if not zeit:
        return None
    return int(zeit - (time.time() if jetzt is None else jetzt))


def pruefen(schwelle: int = SCHWELLE, quellen=QUELLEN) -> dict:
    """Zustand der Uhr: ``{"ok", "abweichung", "hinweis"}``."""
    diff = abweichung(zeit=zeit_holen(quellen))
    if diff is None:
        return {"ok": True, "abweichung": None,
                "hinweis": "keine Zeitquelle erreichbar — Uhr ungeprüft"}
    ok = abs(diff) <= schwelle
    if ok:
        hinweis = f"Uhr stimmt ({diff:+d} s)"
    else:
        tage = diff / 86400.0
        hinweis = (f"Uhr falsch: {diff:+d} s ({tage:+.1f} Tage) — "
                   f"Updates scheitern an „certificate is not yet valid\"")
    return {"ok": ok, "abweichung": diff, "hinweis": hinweis}


def stellen(zeit: int) -> bool:
    """Systemzeit setzen (braucht root). Nutzt ``date -s @<epoch>``."""
    try:
        subprocess.run(["date", "-s", f"@{int(zeit)}"], check=True,
                       capture_output=True, timeout=15)
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def main(argv: Optional[list[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    korrigieren = "--korrigieren" in argv
    schwelle = SCHWELLE
    for i, arg in enumerate(argv):
        if arg == "--schwelle" and i + 1 < len(argv):
            try:
                schwelle = int(argv[i + 1])
            except ValueError:
                pass
    zeit = zeit_holen()
    zustand = pruefen(schwelle=schwelle)
    print(zustand["hinweis"])
    if zustand["ok"] and zustand["abweichung"] is not None:
        return 0
    if not korrigieren:
        return 2 if zustand["abweichung"] is not None else 0
    if not zeit:
        return 1
    if stellen(zeit):
        print(f"Uhr gestellt auf {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(zeit))}")
        # NTP anschalten, damit es nicht wieder wegläuft
        try:
            subprocess.run(["timedatectl", "set-ntp", "true"], check=False,
                           capture_output=True, timeout=10)
        except (OSError, subprocess.SubprocessError):
            pass
        return 0
    print("Uhr konnte nicht gestellt werden (kein root?)", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
