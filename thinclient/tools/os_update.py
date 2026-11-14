"""OS-Aktualisierungen fuer die Anzeige-Clients (Debian/apt).

Warum ein eigenes Werkzeug: der Client hat kein Terminal im Pit.  Das FDC soll
sehen, ob Sicherheitsupdates anstehen, sie auf Knopfdruck ausloesen — und auf
Wunsch selbsttaetig einspielen.  Firmware-Updates laufen getrennt ueber
``ota_update.py``; beide Schalter liegen in ``/etc/fdc/auto.json``.

Aufrufe::

    python3 os_update.py --pruefen [--schreiben]
    python3 os_update.py --installieren        (root, ueber systemd-run)
    python3 os_update.py --status

Zustand: ``/run/fdc-os.json`` (verschwindet beim Reboot — dann steht dort
"unbekannt", was fuer das FDC ehrlicher ist als ein alter Wert).

**Falle:** ``apt-get`` muss konfliktfrei laufen (``--force-confold`` +
``--force-confdef``), sonst bleibt eine Nachfrage zu ``sshd_config``/``grub``
stehen und der Dienst haengt bis zum Timeout.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Optional

OS_STATE = Path("/run/fdc-os.json")
AUTO_CONFIG = Path("/etc/fdc/auto.json")
REBOOT_NÖTIG = Path("/var/run/reboot-required")

#: Laufzeit in Sekunden — der Aufrufer wartet nicht endlos.
APT_TIMEOUT = 900

ENV = dict(os.environ, DEBIAN_FRONTEND="noninteractive",
           LC_ALL="C", LANG="C")


def _zahl(wert: Any, standard: float = 0.0) -> float:
    try:
        return float(wert)
    except (TypeError, ValueError):
        return standard


def auto_config_lesen(pfad: Path | str | None = None) -> dict:
    """Automatik-Schalter lesen: ``{firmware_auto, os_auto, auto_reboot}``.

    ``pfad=None`` heißt „die Standarddatei" — bewusst zur Laufzeit aufgelöst,
    damit Tests und Alternativpfade greifen (Default-Argumente werden beim
    Import gebunden und ignorieren ein späteres Patchen).
    """
    ziel = Path(pfad) if pfad is not None else AUTO_CONFIG
    cfg: dict[str, Any] = {"firmware_auto": False, "os_auto": False,
                           "auto_reboot": False}
    try:
        daten = json.loads(ziel.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        return cfg
    if isinstance(daten, dict):
        cfg["firmware_auto"] = bool(daten.get("firmware_auto"))
        cfg["os_auto"] = bool(daten.get("os_auto"))
        cfg["auto_reboot"] = bool(daten.get("auto_reboot"))
        cfg["os_auto_abstand_h"] = _zahl(daten.get("os_auto_abstand_h"), 24.0)
        cfg["os_auto_zeit"] = _zahl(daten.get("os_auto_zeit"), 0.0)
    return cfg


def auto_config_schreiben(cfg: dict, pfad: Path | str | None = None) -> dict:
    sauber = {
        "firmware_auto": bool(cfg.get("firmware_auto")),
        "os_auto": bool(cfg.get("os_auto")),
        # Neustart nach dem OS-Update ohne Rückfrage (Frage kommt sonst im FDC)
        "auto_reboot": bool(cfg.get("auto_reboot")),
        "os_auto_abstand_h": _zahl(cfg.get("os_auto_abstand_h"), 24.0),
        "os_auto_zeit": _zahl(cfg.get("os_auto_zeit"), 0.0),
    }
    ziel = Path(pfad) if pfad is not None else AUTO_CONFIG
    try:
        ziel.parent.mkdir(parents=True, exist_ok=True)
        temp = ziel.with_suffix(ziel.suffix + ".tmp")
        temp.write_text(json.dumps(sauber, indent=2), encoding="utf-8")
        os.chmod(temp, 0o644)
        temp.replace(ziel)
    except OSError:
        pass
    return sauber


def zustand_lesen(datei: Path | str | None = None) -> dict:
    ziel = Path(datei) if datei is not None else OS_STATE
    try:
        daten = json.loads(ziel.read_text(encoding="utf-8"))
        return daten if isinstance(daten, dict) else {}
    except (OSError, ValueError):
        return {}


def zustand_schreiben(daten: dict, datei: Path | str | None = None) -> dict:
    try:
        ziel = Path(datei) if datei is not None else OS_STATE
        ziel.write_text(json.dumps(daten, indent=2), encoding="utf-8")
        os.chmod(ziel, 0o644)
    except OSError:
        pass
    return daten


def _laufen(kommando: list[str], timeout: int = APT_TIMEOUT) -> tuple[int, str]:
    try:
        ergebnis = subprocess.run(kommando, capture_output=True, text=True,
                                  timeout=timeout, env=ENV, check=False)
        return ergebnis.returncode, (ergebnis.stdout or "") + (ergebnis.stderr or "")
    except FileNotFoundError:
        return 127, "apt nicht vorhanden"
    except subprocess.SubprocessError as fehler:
        return 1, str(fehler)


def pakete_parsen(ausgabe: str) -> list[str]:
    """``apt list --upgradable`` auswerten — gibt die Paketnamen zurueck."""
    namen: list[str] = []
    for zeile in str(ausgabe or "").splitlines():
        zeile = zeile.strip()
        if not zeile or zeile.lower().startswith(("listing", "auflistung")):
            continue
        name = zeile.split("/")[0].strip()
        if name and "/" in zeile and name not in namen:
            namen.append(name)
    return namen


def pruefen(schreiben: bool = False, state: Path | str | None = None,
            laufen=_laufen) -> dict:
    """Nachsehen, ob Updates anstehen (ohne zu installieren)."""
    # Der Index muss aktuell sein, sonst meldet apt veraltete Pakete.
    _laufen(["apt-get", "update", "-qq"], timeout=300)
    code, ausgabe = laufen(["apt", "list", "--upgradable"], timeout=180)
    pakete = pakete_parsen(ausgabe)
    daten = {
        "stand": time.time(),
        "ok": code == 0,
        "anzahl": len(pakete),
        "pakete": pakete[:40],
        "reboot_noetig": REBOOT_NÖTIG.exists(),
        "auto": auto_config_lesen(),
    }
    if schreiben:
        zustand_schreiben(daten, state)
    return daten


def verlauf_kuerzen(ausgabe: str, zeilen: int = 18) -> list[str]:
    """Die letzten Zeilen der apt-Ausgabe behalten (für Anzeige im FDC/Panel).

    ``-qq`` ist knapp, aber uninteressant ist es nicht: bei einem Fehler steht
    der Grund dort und nicht in der letzten Zeile.
    """
    relevante = []
    for zeile in str(ausgabe or "").splitlines():
        zeile = zeile.strip()
        if not zeile or zeile.startswith(("(Reading database", "Preparing to unpack",
                                          "Unpacking ", "Setting up ", "Selecting ")):
            continue
        relevante.append(zeile)
    return relevante[-zeilen:]


def neu_starten(kommando: str = "reboot") -> bool:
    """Client neu starten (root-Dienst)."""
    try:
        return subprocess.run(["systemctl", kommando], timeout=30,
                              check=False).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def installieren(state: Path | str | None = None, laufen=_laufen,
                 neustart=neu_starten) -> dict:
    """Updates einspielen (noninteractive, Konfigdateien behalten)."""
    start = time.time()
    zustand_schreiben({"laeuft": True, "start": start}, state)
    vorher = pruefen(schreiben=False, laufen=laufen)
    code, ausgabe = laufen([
        "apt-get", "-y", "-qq",
        "-o", "Dpkg::Options::=--force-confold",
        "-o", "Dpkg::Options::=--force-confdef",
        "upgrade",
    ])
    auto = auto_config_lesen()
    reboot_noetig = REBOOT_NÖTIG.exists()
    daten = {
        "stand": time.time(),
        "start": start,
        "dauer_s": round(time.time() - start, 1),
        "laeuft": False,
        "ok": code == 0,
        "vorher": vorher.get("anzahl", 0),
        "nachher": len(pakete_parsen(laufen(["apt", "list", "--upgradable"],
                                            timeout=120)[1])),
        "reboot_noetig": reboot_noetig,
        # Ohne Nachfrage: Frage im FDC waere hier sinnlos, niemand sitzt davor
        "reboot_quittiert": False,
        "meldung": ausgabe.strip().splitlines()[-1] if ausgabe.strip() else "",
        "verlauf": verlauf_kuerzen(ausgabe),
        "auto": auto,
    }
    # Auto-Reboot: nur wenn das Flag gesetzt ist UND ein Neustart nötig ist
    if reboot_noetig and auto.get("auto_reboot"):
        daten["reboot_gestartet"] = bool(neustart())
        daten["reboot_quittiert"] = True
    zustand_schreiben(daten, state)
    return daten


def quittieren(state: Path | str | None = None) -> dict:
    """Neustart-Frage abhaken, damit das FDC nicht bei jedem Öffnen fragt."""
    daten = zustand_lesen(state)
    daten["reboot_quittiert"] = True
    zustand_schreiben(daten, state)
    return daten


def status(state: Path | str | None = None) -> dict:
    daten = zustand_lesen(state)
    daten.setdefault("reboot_noetig", REBOOT_NÖTIG.exists())
    daten["auto"] = auto_config_lesen()
    return daten


def main(argv: Optional[list[str]] = None) -> int:
    teile = list(argv if argv is not None else sys.argv[1:])
    if "--installieren" in teile:
        print(json.dumps(installieren(), indent=2))
        return 0
    if "--status" in teile:
        print(json.dumps(status(), indent=2))
        return 0
    # Standard: pruefen (mit --schreiben auch ablegen)
    daten = pruefen(schreiben=("--schreiben" in teile or "--pruefen" in teile))
    print(json.dumps(daten, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
