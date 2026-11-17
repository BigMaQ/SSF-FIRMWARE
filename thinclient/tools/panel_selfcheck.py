"""Selbstauskunft des Anzeige-Clients — Grundlage für den Diag-Splash.

Ziel: Steht der Client an einem Messetag ohne SSH-Zugang da, soll der
**Bildschirm** sagen, was los ist.  Alles, was hier drinsteht, ist aus dem
Gerät selbst ablesbar (kein Netz nötig):

==================  ====================================================
Status              FEHLER / WARNUNG / OK
IP                  eigene LAN-Adresse
Network             LAN / WiFi / keins
Connection          FDC-Beacon gesehen? (nur der Beacon, kein Ping)
Stream Active       schickt die Engine gerade Bilder?
Component Version   Version des Client-Pakets
Installation        fehlende Bausteine (z. B. Viewer-Modul)
Permissions         darf der Dienst nach /etc/fdc schreiben?
Clock               plausibel gegen das Build-Datum?
==================  ====================================================

Uhrzeit-Plausibilität ohne Netz: das Abbild legt sein Build-Datum ab; liegt
das „aktuelle" Datum davor, kann die Uhr nicht stimmen (leere CMOS-Batterie) —
und dann scheitert jedes TLS und damit jedes Update.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional

#: Datum, an dem das Abbild gebaut wurde (vom Build-Skript geschrieben)
BUILD_DATEI = Path("/opt/fdc/BUILDDATE")
#: Wie alt der letzte FDC-Beacon höchstens sein darf (Sekunden)
BEACON_FRISCH_S = 45.0
#: Rechte-Prüfpfad: dort schreibt der Agent (JSONs) und der Viewer
RECHTE_VERZEICHNIS = Path("/etc/fdc")
#: Module, ohne die der Viewer nicht streamen kann
PFLICHT_MODULE = ("websocket", "PySide6.QtWidgets", "PySide6.QtGui", "PySide6.QtNetwork")


@dataclass
class Bericht:
    """Was der Diag-Splash anzeigt."""

    status: str = "ok"                     # ok | warnung | fehler
    zeilen: list[tuple[str, str]] = field(default_factory=list)
    probleme: list[str] = field(default_factory=list)      # Klartext (Log)
    codes: list[tuple[str, dict]] = field(default_factory=list)   # (code, werte)

    @property
    def fehler(self) -> bool:
        return self.status == "fehler"

    def zeile(self, schluessel: str, wert: str) -> None:
        self.zeilen.append((schluessel, wert))

    def problem(self, code: str, werte: Optional[dict] = None) -> None:
        """Problem als Code (+Werte) ablegen — der Klartext kommt erst beim Malen.

        Damit ist die Tafel vier-sprachig; ein fest eingebauter deutscher Satz
        wäre in einer englischen Anlage unbrauchbar.
        """
        self.codes.append((code, dict(werte or {})))

    def problem_texte(self, sprache_code: str = "de") -> list[str]:
        try:
            from tools import panel_lang as pl
        except ImportError:                      # flache Installation auf der Box
            import panel_lang as pl

        texte = []
        for code, werte in self.codes:
            try:
                texte.append(pl.text("prob_" + code, sprache_code, **werte))
            except Exception:                    # noqa: BLE001 - Anzeige darf nie kippen
                texte.append(code)
        return texte

    def als_dict(self) -> dict:
        return {"status": self.status, "zeilen": list(self.zeilen),
                "probleme": list(self.probleme),
                "codes": [(c, dict(w)) for c, w in self.codes]}


# --------------------------------------------------------------------------- Uhr
def build_datum(pfad: Path | str = BUILD_DATEI) -> Optional[datetime]:
    """Build-Datum aus dem Abbild lesen — ``None``, wenn keins da ist."""
    for kandidat in (Path(pfad), Path("/opt/fdc/BUILDDATE"), Path("/etc/fdc/builddate")):
        try:
            roh = kandidat.read_text(encoding="utf-8", errors="replace").strip()
        except OSError:
            continue
        if not roh:
            continue
        for muster in ("%Y-%m-%dT%H:%M:%S", "%Y-%m-%d %H:%M:%S", "%Y-%m-%d"):
            try:
                return datetime.strptime(roh[:len(muster)], muster)
            except ValueError:
                continue
        try:
            return datetime.fromisoformat(roh.replace("Z", "+00:00")).replace(tzinfo=None)
        except ValueError:
            continue
    return None


def uhr_plausibel(jetzt: Optional[datetime] = None,
                  build: Optional[datetime] = None,
                  toleranz_s: float = 3600.0) -> tuple[bool, dict]:
    """Ist die Uhr plausibel?  Rückgabe: (ok, Werte für die Anzeige).

    Ein Datum **vor** dem Build-Datum ist physikalisch unmöglich und der
    sicherste Hinweis auf eine leere CMOS-Batterie.  Die Stunde Toleranz fängt
    Zeitzonen-Verschiebungen beim Bauen ab.  Die Werte kommen als Zahlen/Daten
    zurück, damit jede Sprache ihren eigenen Satz darum bauen kann.
    """
    jetzt = jetzt or datetime.now()
    build = build if build is not None else build_datum()
    if build is None:
        return True, {}
    if jetzt < build and (build - jetzt).total_seconds() > toleranz_s:
        return False, {"jetzt": f"{jetzt:%Y-%m-%d %H:%M}", "build": f"{build:%Y-%m-%d}"}
    return True, {}


# --------------------------------------------------------------------------- Netz
def netz_aus_nmcli(aus: str) -> str:
    """``nmcli -t -f DEVICE,TYPE,STATE device status`` → ``LAN``/``WiFi``/``keins``.

    Achtung: ``disconnected`` enthält das Wort ``connected`` — deshalb wird auf
    den **Anfang** des Zustands geprüft.  Und ein Kabel, das NetworkManager
    nicht führt (``unmanaged``, so wie Debian es nach der Installation
    hinterlässt), taucht hier gar nicht auf; dafür gibt es ``netz_aus_route``.
    """
    lan = wifi = False
    for zeile in (aus or "").splitlines():
        teile = zeile.split(":")
        if len(teile) < 3:
            continue
        typ, stand = teile[1].strip().lower(), teile[2].strip().lower()
        if not stand.startswith("connected"):
            continue
        if typ == "ethernet":
            lan = True
        elif typ.startswith("wifi"):
            wifi = True
    if lan:
        return "LAN"
    if wifi:
        return "WiFi"
    return "keins"


def netz_aus_route(aus: str) -> str:
    """``ip route get 192.0.2.1`` → ``LAN``/``WiFi``/``keins`` nach Gerätenamen.

    Der Gerätename ist die verlässliche Auskunft, wenn NetworkManager die
    Karte nicht führt: ``wl*``/``mlan*`` ist Funk, alles andere Kabel.
    """
    teile = (aus or "").split()
    geraet = ""
    for nummer, teil in enumerate(teile):
        if teil == "dev" and nummer + 1 < len(teile):
            geraet = teile[nummer + 1]
            break
    if not geraet:
        return "keins"
    return "WiFi" if geraet.startswith(("wl", "mlan")) else "LAN"


def fehlende_module(module=PFLICHT_MODULE,
                    finder: Optional[Callable[[str], bool]] = None) -> list[str]:
    """Welche Pflicht-Module fehlen (Import ohne Seiteneffekt prüfen)."""
    if finder is None:
        import importlib.util

        def standard_finder(name: str) -> bool:            # noqa: ANN001
            try:
                return importlib.util.find_spec(name) is not None
            except (ImportError, ValueError, AttributeError):
                return False

        finder = standard_finder
    return [name for name in module if not finder(name)]


def rechte_ok(verzeichnis: Path | str = RECHTE_VERZEICHNIS) -> tuple[bool, str]:
    """Darf dieser Prozess dort schreiben?  Rückgabe: (ok, betroffener Pfad).

    Nur der Pfad kommt zurück — den Satz darum baut die Anzeige in der
    jeweiligen Sprache.  (Agent und Viewer laufen als ``ssf``.)
    """
    pfad = Path(verzeichnis)
    if not pfad.is_dir():
        return False, str(pfad)
    if not os.access(pfad, os.W_OK | os.X_OK):
        return False, str(pfad)
    for name in ("panels.json", "geraet.json"):
        datei = pfad / name
        if datei.exists() and not os.access(datei, os.W_OK):
            return False, str(datei)
    return True, ""


def beacon_alter(state_pfad: Path | str, jetzt: Optional[float] = None) -> Optional[float]:
    """Alter des letzten FDC-Beacons in Sekunden (``None`` = nie gesehen).

    Quelle ist ``/run/fdc-power.json``: ``power_idle`` schreibt dort bei jedem
    Durchlauf den Zeitstempel des zuletzt gehörten Beacons.
    """
    try:
        roh = Path(state_pfad).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    if not roh.strip():
        return None
    wert: Optional[float] = None
    try:
        import json

        daten = json.loads(roh)
        if isinstance(daten, dict):
            kandidat = daten.get("letzter_beacon")
            if isinstance(kandidat, (int, float)):
                wert = float(kandidat)
    except ValueError:
        wert = None
    if wert is None:
        try:
            wert = float(roh.strip())
        except ValueError:
            return None
    return max(0.0, (jetzt if jetzt is not None else time.time()) - wert)


def eigene_ip() -> str:
    """LAN-Adresse ohne Paketversand (Routing-Probe)."""
    import socket

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.0.2.1", 9))
            return s.getsockname()[0]
    except OSError:
        return ""


def paket_version(pfad: Path | str = "/opt/fdc/VERSION") -> str:
    try:
        return Path(pfad).read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""


def netz_live(lauf: Optional[Callable[..., tuple[int, str]]] = None,
              route: Optional[Callable[..., tuple[int, str]]] = None) -> str:
    """Netzart bestimmen: erst NetworkManager, sonst die Route.

    Zwei Wege, weil keiner allein reicht: ``nmcli`` meldet Kabel, die es nicht
    führt (nach der Installation ``unmanaged``) nicht als verbunden, und die
    Route kennt kein WLAN-Detail.  Zusammen ist die Auskunft belastbar.
    """
    if lauf is None:
        def nmcli_lauf(*args, **kwargs):                    # noqa: ANN001
            return _unterprozess(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE",
                                  "device", "status"])

        lauf = nmcli_lauf
    if route is None:
        def route_lauf(*args, **kwargs):                    # noqa: ANN001
            return _unterprozess(["ip", "route", "get", "192.0.2.1"])

        route = route_lauf

    for pruefung, auswerter in ((lauf, netz_aus_nmcli), (route, netz_aus_route)):
        try:
            rc, aus = pruefung()
        except Exception:                                    # noqa: BLE001
            continue
        if rc == 0:
            art = auswerter(aus)
            if art != "keins":
                return art
    return "keins"


def _unterprozess(befehl: list[str]) -> tuple[int, str]:
    """Befehl ausführen, nie werfen (Diagnose darf nicht scheitern)."""
    import subprocess

    try:
        e = subprocess.run(befehl, capture_output=True, text=True, timeout=5)
        return e.returncode, e.stdout
    except (OSError, subprocess.SubprocessError):
        return 1, ""


def laufender_bericht(*, stream_aktiv: Optional[bool] = None,
                      version: str = "",
                      jetzt: Optional[datetime] = None,
                      netz: Optional[str] = None,
                      beacon_sekunden: Optional[float] = None,
                      ip: Optional[str] = None,
                      power_state: Path | str = "/run/fdc-power.json") -> Bericht:
    """Bericht für den Viewer: holt sich alle Werte selbst, wo möglich."""
    return bericht_live(
        version=version or paket_version(),
        stream_aktiv=stream_aktiv,
        ip=ip if ip is not None else eigene_ip(),
        netz=netz if netz is not None else netz_live(),
        beacon_sekunden=(beacon_sekunden if beacon_sekunden is not None
                         else beacon_alter(power_state)),
        jetzt=jetzt)


# --------------------------------------------------------------------------- Bericht
def bericht(*, version: str = "", ip: str = "", netz: str = "keins",
            stream_aktiv: Optional[bool] = None,
            beacon_sekunden: Optional[float] = None,
            fehlende: Optional[list[str]] = None,
            rechte: tuple[bool, str] = (True, ""),
            uhr: tuple[bool, dict] = (True, {}),
            build: Optional[datetime] = None) -> Bericht:
    """Alles zu einem Bericht zusammenfassen (reine Rechnung, testbar).

    Harte Fehler (das Gerät kann nicht funktionieren) stehen vorn, Warnungen
    (es läuft, aber etwas klemmt) dahinter — der Diag-Splash zeigt den ersten
    Eintrag in der Fußzeile, und das soll der wichtigste sein.
    """
    b = Bericht()
    hart: list[tuple[str, dict]] = []
    weich: list[tuple[str, dict]] = []

    b.zeile("IP", ip or "—")
    b.zeile("Network", {"keins": "none"}.get(netz or "keins", netz or "none"))
    if netz in ("", "keins"):
        # Kein Netz: das ist der Grund, warum weder Beacon noch Update geht.
        weich.append(("kein_netz", {}))

    if beacon_sekunden is None:
        b.zeile("Connection", "no_fdc")
        weich.append(("kein_fdc", {}))
    elif beacon_sekunden > BEACON_FRISCH_S:
        b.zeile("Connection", f"stale:{beacon_sekunden:.0f}")
        weich.append(("beacon_alt", {"sekunden": f"{beacon_sekunden:.0f}"}))
    else:
        b.zeile("Connection", "connected")

    if stream_aktiv is None:
        b.zeile("Stream", "—")
    else:
        b.zeile("Stream Active", "yes" if stream_aktiv else "no")

    b.zeile("Component Version", version or "—")
    b.zeile("Build", f"{build:%Y-%m-%d}" if build else "—")

    liste = list(fehlende or [])
    b.zeile("Installation", ("missing:" + ", ".join(liste)) if liste else "complete")
    if liste:
        hart.append(("module", {"liste": ", ".join(liste)}))

    ok_rechte, pfad = rechte
    # Kurz halten: die Tabelle hat eine Spaltenbreite, das Detail steht im Fuß
    b.zeile("Permissions", "ok" if ok_rechte else "error")
    if not ok_rechte:
        hart.append(("rechte", {"pfad": pfad or "/etc/fdc"}))

    ok_uhr, uhr_werte = uhr
    b.zeile("Clock", "ok" if ok_uhr else "error")
    if not ok_uhr:
        hart.append(("uhr", dict(uhr_werte)))

    b.codes = hart + weich
    b.status = "fehler" if hart else ("warnung" if weich else "ok")
    b.probleme = b.problem_texte("de")           # Klartext fürs Log
    return b


def bericht_live(*, version: str = "", stream_aktiv: Optional[bool] = None,
                 ip: str = "", netz: str = "keins",
                 beacon_sekunden: Optional[float] = None,
                 jetzt: Optional[datetime] = None,
                 build: Optional[datetime] = None) -> Bericht:
    """Bericht mit echten Prüfungen (für den Viewer)."""
    b = build if build is not None else build_datum()
    return bericht(version=version, ip=ip, netz=netz, stream_aktiv=stream_aktiv,
                   beacon_sekunden=beacon_sekunden,
                   fehlende=fehlende_module(), rechte=rechte_ok(),
                   uhr=uhr_plausibel(jetzt or datetime.now(), b), build=b)


def jetzt_utc_text() -> str:
    """Nur für Anzeigen: aktueller Zeitpunkt in lesbarer Form."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
