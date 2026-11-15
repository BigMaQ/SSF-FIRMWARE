"""Auto-Power-off fuer die Anzeige-Clients (hinter dem Pit kommt man nicht ran).

Regeln — so abgestimmt mit Markus:

* **Eine Uhr, eine Zahl:** ``auto_off_min`` Minuten **ohne Bild und ohne
  FDC-Beacon** (UDP 5011) ⇒ herunterfahren.  Solange eines von beiden da ist,
  laeuft keine Uhr; kommt eines zurueck, faengt sie von vorn an.
* Frueher lagen zwei Schwellen davor (``leerlauf_min`` *plus* Abschaltuhr).
  Wer 2 Minuten einstellte, wartete real 32 — das war niemandem zu erklaeren
  (Fehlerbericht Markus, 15.11.).  ``leerlauf_min`` wird nur noch gelesen und
  gespeichert, damit alte ``power.json`` nicht kaputtgeht; es zaehlt nicht mehr.

Das Programm laeuft als root (``fdc-poweroff.service``), weil es am Ende
``systemctl poweroff`` ruft.  Die Entscheidung selbst steckt in ``tick()`` —
eine reine Funktion mit injizierbarer Zeit, damit sie ohne Timer testbar ist.
"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

BEACON_PORT = 5011
#: Das Bild-Alter schreibt der Viewer.  Nach ``/tmp``, weil der Viewer als
#: ``ssf`` laeuft und in ``/run`` (root, 0755) nichts anlegen darf.
STREAM_STATE = Path("/tmp/fdc-stream.json")
POWER_CONFIG = Path("/etc/fdc/power.json")
POWER_STATE = Path("/run/fdc-power.json")

#: Defaults: 30 min Ruhe, danach 60 min Gnadenfrist.
DEFAULTS: dict[str, Any] = {"an": True, "leerlauf_min": 30, "auto_off_min": 60}

TICK_S = 30.0


# --------------------------------------------------------------------------- io
def _zahl(wert: Any, standard: float) -> float:
    try:
        return float(str(wert).replace(",", "."))
    except (TypeError, ValueError):
        return float(standard)


def config_lesen(pfad: Path | str = POWER_CONFIG) -> dict:
    """Power-Konfiguration lesen; fehlende/defekte Datei ⇒ Defaults."""
    cfg = dict(DEFAULTS)
    try:
        daten = json.loads(Path(pfad).read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        return cfg
    if not isinstance(daten, dict):
        return cfg
    cfg["an"] = bool(daten.get("an", DEFAULTS["an"]))
    cfg["leerlauf_min"] = max(0.0, _zahl(daten.get("leerlauf_min"),
                                         DEFAULTS["leerlauf_min"]))
    cfg["auto_off_min"] = max(0.0, _zahl(daten.get("auto_off_min"),
                                         DEFAULTS["auto_off_min"]))
    return cfg


def config_schreiben(cfg: dict, pfad: Path | str = POWER_CONFIG) -> dict:
    """Konfiguration ablegen (atomar); gibt die geschriebenen Werte zurueck."""
    sauber = {
        "an": bool(cfg.get("an", DEFAULTS["an"])),
        "leerlauf_min": max(0.0, _zahl(cfg.get("leerlauf_min"),
                                        DEFAULTS["leerlauf_min"])),
        "auto_off_min": max(0.0, _zahl(cfg.get("auto_off_min"),
                                        DEFAULTS["auto_off_min"])),
    }
    ziel = Path(pfad)
    try:
        ziel.parent.mkdir(parents=True, exist_ok=True)
        temp = ziel.with_suffix(ziel.suffix + ".tmp")
        temp.write_text(json.dumps(sauber, indent=2), encoding="utf-8")
        os.chmod(temp, 0o644)
        temp.replace(ziel)
    except OSError:
        pass
    return sauber


def letztes_bild(datei: Path | str = STREAM_STATE) -> Optional[float]:
    """Zeitstempel des letzten Bildes (None = unbekannt, wird als 'alt' gewertet)."""
    try:
        daten = json.loads(Path(datei).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    wert = daten.get("letztes_bild")
    return _zahl(wert, 0.0) if wert is not None else None


def zustand_schreiben(zustand: dict, datei: Path | str = POWER_STATE) -> None:
    try:
        ziel = Path(datei)
        ziel.write_text(json.dumps(zustand, indent=2), encoding="utf-8")
        os.chmod(ziel, 0o644)
    except OSError:
        pass


def zustand_lesen(datei: Path | str = POWER_STATE) -> dict:
    try:
        daten = json.loads(Path(datei).read_text(encoding="utf-8"))
        return daten if isinstance(daten, dict) else {}
    except (OSError, ValueError):
        return {}


# ------------------------------------------------------------------- Entscheidung
def tick(jetzt: float, letztes_bild_ts: Optional[float],
         letzter_beacon_ts: Optional[float], cfg: dict,
         ruhe_seit: Optional[float] = None) -> dict:
    """Eine Runde der Abschaltlogik — reine Funktion.

    ``auto_off_min`` Minuten ohne Bild **und** ohne Beacon ⇒ ``poweroff``.
    ``ruhe_seit`` ist der Beginn der Ruhe: der Zeitpunkt der letzten Aktivität
    (Bild oder Beacon) — die Uhr zählt also ab „es ist gerade still geworden",
    nicht ab „ich habe es bemerkt".  Ohne jeden Zeitstempel startet sie beim
    ersten Durchlauf.

    Rueckgabe: ``ruhe_seit``, ``aus_in_s`` (Sekunden bis zum Herunterfahren,
    None = keine Uhr), ``wartet`` (True solange noch Aktivität da ist) und
    ``aktion`` = ``"poweroff"``, wenn es jetzt so weit ist.
    """
    ergebnis: dict[str, Any] = {"ruhe_seit": ruhe_seit, "aus_in_s": None,
                                "aktion": None, "an": bool(cfg.get("an", True))}
    if not ergebnis["an"]:
        ergebnis["ruhe_seit"] = None
        return ergebnis

    frist = _zahl(cfg.get("auto_off_min"), DEFAULTS["auto_off_min"]) * 60.0
    if frist <= 0:
        # 0 = Abschalten aus
        ergebnis["ruhe_seit"] = None
        return ergebnis

    aktivitaeten = [ts for ts in (letztes_bild_ts, letzter_beacon_ts) if ts]
    letzte = max(aktivitaeten) if aktivitaeten else None
    if letzte is None:
        # Nichts bekannt (kein Bild, kein Beacon) ⇒ Uhr ab jetzt
        letzte = ergebnis["ruhe_seit"] if ergebnis["ruhe_seit"] is not None else jetzt
    ergebnis["ruhe_seit"] = letzte
    rest = frist - (jetzt - letzte)
    if rest > 0 and aktivitaeten:
        ergebnis["wartet"] = True
    ergebnis["aus_in_s"] = max(0.0, round(rest, 1))
    if rest <= 0:
        ergebnis["aktion"] = "poweroff"
    return ergebnis


# ------------------------------------------------------------------- Beacon-Lauscher
class BeaconLauscher(threading.Thread):
    """Hoert auf den FDC-Beacon (UDP) und merkt sich den letzten Empfang."""

    def __init__(self, port: int = BEACON_PORT) -> None:
        super().__init__(daemon=True, name="FdcBeacon")
        self.port = int(port)
        self.letzter: Optional[float] = None
        self.text = ""
        self._laufen = True
        self._sock: Optional[socket.socket] = None

    def run(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", self.port))
            sock.settimeout(5.0)
            self._sock = sock
        except OSError:
            return
        while self._laufen:
            try:
                daten, _ = sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue
            text = daten.decode("utf-8", "replace").strip()
            if text.upper().startswith("FDC"):
                self.letzter = time.time()
                self.text = text

    def stoppen(self) -> None:
        self._laufen = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass


# ------------------------------------------------------------------------- Ablauf
def herunterfahren(befehl: str = "poweroff") -> bool:
    """Client herunterfahren (``poweroff``) bzw. neu starten (``reboot``)."""
    for kommandos in (["systemctl", befehl], ["/sbin/" + befehl]):
        try:
            subprocess.run(kommandos, timeout=20, check=False)
            return True
        except (OSError, subprocess.SubprocessError):
            continue
    return False


def einmal(lauscher: Optional[BeaconLauscher], cfg: dict,
           jetzt: Optional[float] = None,
           ausschalten: Callable[[str], object] = herunterfahren) -> dict:
    """Eine Runde: Aktivitaet lesen, entscheiden, Zustand melden, ggf. aus."""
    jetzt = time.time() if jetzt is None else jetzt
    alter = zustand_lesen()
    ergebnis = tick(jetzt, letztes_bild(), getattr(lauscher, "letzter", None), cfg,
                    ruhe_seit=alter.get("ruhe_seit"))
    ergebnis.update({
        "zeit": jetzt,
        "letztes_bild": letztes_bild(),
        "letzter_beacon": getattr(lauscher, "letzter", None),
        "config": cfg,
    })
    if ergebnis.get("aktion") == "poweroff":
        zustand_schreiben(ergebnis)
        ausschalten("poweroff")
    else:
        zustand_schreiben(ergebnis)
    return ergebnis


def main(argv: list[str] | None = None) -> int:
    teile = list(argv if argv is not None else sys.argv[1:])
    if "--pruefen" in teile or "--status" in teile:
        cfg = config_lesen()
        jetzt = time.time()
        lauf = zustand_lesen()
        print(json.dumps({"config": cfg, "letztes_bild": letztes_bild(),
                          "letzter_beacon": lauf.get("letzter_beacon"),
                          "aus_in_s": lauf.get("aus_in_s"),
                          "ruhe_seit": lauf.get("ruhe_seit"),
                          "jetzt": jetzt}, indent=2))
        return 0
    lauscher = BeaconLauscher()
    lauscher.start()
    print(f"[power] wache ueber Auto-Power-off (Beacon-Port {BEACON_PORT})", flush=True)
    while True:
        try:
            ergebnis = einmal(lauscher, config_lesen())
            if ergebnis.get("aus_in_s") is not None:
                print(f"[power] kein Bild/Beacon seit {ergebnis['ruhe_seit']:.0f} — "
                      f"aus in {ergebnis['aus_in_s'] / 60:.1f} min", flush=True)
        except Exception as fehler:            # Dienst darf nicht sterben
            print(f"[power] Fehler: {fehler}", flush=True)
        time.sleep(TICK_S)


if __name__ == "__main__":
    raise SystemExit(main())
