"""Geraete-Identitaet eines Anzeige-Clients (Thin Client / beliebiges IP-Geraet).

Der Client soll sich **wie ein serielles Panel** melden: ein IDENT-String im
gleichen Format, den FDC schon kennt::

    <Name> - <Panels>, v<FW> MAQ, SN:<Seriennummer>;HWREV:<hw>;STATE:running

Der Name wird aus der MAC gebildet (``Avionic-<letzte 6 Zeichen>``) und ist im
FDC aenderbar.  Aus der Zuordnung „Monitor 1: bis zu 2 Panels, Monitor 2: bis zu
2 Panels" entsteht sowohl der Ident *und* das Layout (``panels.json``), das der
Agent an den Viewer gibt — die Firmware bleibt universell, das Geraet erfaehrt
vom FDC, wer es ist.

Reine Funktionen, kein Qt, kein HTTP — damit ohne Geraet pruefbar.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Optional

#: Hoechstzahl Panels je Monitor.
PANELS_JE_MONITOR = 2

#: Vorsatz der Seriennummer (Thin Client).
SN_PRAEFIX = "TC-"


def mac_kuerzel(mac: str, laenge: int = 6) -> str:
    """``54:48:10:E8:7B:D0`` → ``e87bd0`` (Hex, klein, ohne Trenner)."""
    hex_zeichen = "".join(ch for ch in str(mac or "").lower() if ch in "0123456789abcdef")
    return hex_zeichen[-laenge:] if hex_zeichen else ""


def standard_name(mac: str) -> str:
    """Vorschlag fuer den Geraetenamen: ``Avionic-e87bd0``."""
    kuerzel = mac_kuerzel(mac)
    return f"Avionic-{kuerzel}" if kuerzel else "Avionic"


def panel_namen(geraet: dict) -> list[str]:
    """Alle zugewiesenen Panel-IDs in Monitor-Reihenfolge, ohne Doppelte."""
    namen: list[str] = []
    for monitor in ("monitor1", "monitor2"):
        for eintrag in geraet.get(monitor) or []:
            panel = str((eintrag or {}).get("panel") or "").strip()
            if panel and panel not in namen:
                namen.append(panel)
    return namen


def ist_eacam(panel: str) -> bool:
    return str(panel or "").strip().lower().startswith("ecam")


#: Kurzrolle je Panel-ID — ECAM hat zwei Haelften, PFD/ND zwei Seiten.
#: Ohne die Endung waeren ``ecam-upper`` und ``ecam-lower`` beide "ECAM" und
#: der Anzeigename verschluckte eine der beiden (Markus: "ECAM Upper, ECAM Lower
#: waere gut, noch abzubilden").
ROLLE_KURZ = {
    "pfd-captain": "PFD CPT", "pfd-fo": "PFD FO",
    "nd-captain": "ND CPT", "nd-fo": "ND FO",
    "ecam-upper": "ECAM UPPER", "ecam-lower": "ECAM LOWER",
    "mcdu-captain": "MCDU CPT", "mcdu-fo": "MCDU FO",
    "wxr": "WXR", "standby": "STBY", "dcdu": "DCDU",
}

#: Schreibweisen der Endungen (auch ``ecam_upper``, ``pfd-cpt``).
_SEITE_KURZ = {"captain": "CPT", "cpt": "CPT", "fo": "FO", "f/o": "FO",
               "upper": "UPPER", "up": "UPPER", "lower": "LOWER", "low": "LOWER"}


def anzeigenamen_kuerzen(panel: str) -> str:
    """``ecam-upper`` → ``ECAM UPPER``, ``pfd-captain`` → ``PFD CPT``."""
    p = str(panel or "").strip().lower().replace("_", "-")
    if not p:
        return ""
    if p in ROLLE_KURZ:
        return ROLLE_KURZ[p]
    teile = [t for t in p.split("-") if t]
    basis = teile[0].upper()
    seite = next((_SEITE_KURZ[t] for t in teile[1:] if t in _SEITE_KURZ), "")
    return f"{basis} {seite}".strip()


def rollen_text(geraet: dict) -> str:
    """``PFD, ND`` — die Rollen fuer den Anzeigenamen/IDENT (ohne Doppelte)."""
    rollen: list[str] = []
    for panel in panel_namen(geraet):
        rolle = anzeigenamen_kuerzen(panel)
        if rolle and rolle not in rollen:
            rollen.append(rolle)
    return ", ".join(rollen)


def anzeigename(geraet: dict, mac: str = "") -> str:
    """``Avionic-e87bd0 - PFD, ECAM`` (Name wie im Panel-Tab sichtbar)."""
    name = str(geraet.get("name") or "").strip() or standard_name(mac)
    rollen = rollen_text(geraet)
    return f"{name} - {rollen}" if rollen else name


#: Hier legt ``fdc-ident.service`` (root) die echte Geräte-Seriennummer ab.
#: Nötig, weil ``/sys/class/dmi/id/product_serial`` nur für root lesbar ist und
#: der Agent als ``ssf`` läuft.
SERIAL_DATEI = "/etc/fdc/serial"


def hardware_seriennummer(datei: Path | str | None = None) -> str:
    """Seriennummer des Geräts aus ``/etc/fdc/serial`` (leer = unbekannt).

    Werte wie „To Be Filled By O.E.M." sind kein Ergebnis und gelten als leer.
    """
    ziel = Path(datei) if datei is not None else Path(SERIAL_DATEI)
    try:
        wert = ziel.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""
    wert = wert.splitlines()[0].strip() if wert else ""
    if not wert or wert.lower() in {"0", "none", "unknown", "default string"}:
        return ""
    return wert


def seriennummer(geraet: dict, mac: str, datei: Path | str | None = None) -> str:
    """S/N bestimmen: gesetzt (von Hand) → Gerät (SMBIOS) → aus der MAC.

    Die Reihenfolge ist Absicht: was im Geräte-Eintrag steht, hat Markus selbst
    vergeben und gewinnt deshalb; sonst gilt die echte Seriennummer vom Gehäuse;
    erst wenn beides fehlt, wird eine aus der MAC erzeugt.
    """
    sn = str(geraet.get("sn") or "").strip()
    if sn:
        return sn
    echt = hardware_seriennummer(datei)
    if echt:
        return echt
    kuerzel = mac_kuerzel(mac)
    return f"{SN_PRAEFIX}{kuerzel}" if kuerzel else ""


def ident_text(geraet: dict, mac: str, version: str, hardware: str = "thinclient") -> str:
    """IDENT-String bauen — gleiches Format wie die seriellen Panels.

    Beispiel::

        Avionic-e87bd0 - PFD, ECAM, v0.6.0 MAQ, SN:TC-e87bd0;HWREV:thinclient;STATE:running
    """
    fw = str(version or "").strip() or "?"
    autor = str(geraet.get("author") or "MAQ").strip()
    sn = seriennummer(geraet, mac)
    hw = str(hardware or "thinclient").strip()
    kopf = f"{anzeigename(geraet, mac)}, v{fw} {autor}"
    if sn:
        kopf += f", SN:{sn}"
    return f"{kopf};HWREV:{hw};STATE:running"


# --------------------------------------------------------------------------
# Geraet aus vorhandenem Layout ableiten (Bestandsinstallationen)
# --------------------------------------------------------------------------
def geraet_aus_layout(layout: dict, mac: str = "", schirme: Optional[list] = None) -> dict:
    """Ein ``panels.json`` in eine Geraete-Zuordnung uebersetzen.

    Damit bekommt ein bereits eingerichteter Client ohne Zutun eine Identitaet:
    die Panels werden nach Schirm und Position auf Monitor 1/2 verteilt.
    """
    geraet: dict[str, Any] = {"name": standard_name(mac), "sn": "",
                              "monitor1": [], "monitor2": []}
    panels = list((layout or {}).get("panels") or [])
    if not panels:
        return geraet
    # Stabil sortieren: Schirm, dann x, dann y.
    def _schluessel(eintrag: dict):
        return (int(eintrag.get("screen") or 0), int(eintrag.get("x") or 0),
                int(eintrag.get("y") or 0))

    for eintrag in sorted(panels, key=_schluessel):
        schirm = int(eintrag.get("screen") or 0)
        monitor = "monitor1" if schirm <= 0 else "monitor2"
        if len(geraet[monitor]) >= PANELS_JE_MONITOR:
            continue
        name = str(eintrag.get("panel") or "").strip()
        if not name:
            continue
        geraet[monitor].append({"panel": name})
    return geraet


# --------------------------------------------------------------------------
# Layout aus der Zuordnung bauen
# --------------------------------------------------------------------------
def schirmgroesse(schirme: list, index: int) -> tuple[int, int, str]:
    """(Breite, Höhe, Herkunft) für den Schirm *index* (0-basiert).

    Monitore werden **einzeln** behandelt: jeder Schirm, den das Gerät meldet,
    liefert seine eigene Auflösung.  Meldet es einen Schirm nicht (Kabel nicht
    dran, xrandr kennt ihn nicht), wird die Auflösung des **ersten** gemeldeten
    Schirms geliehen — nicht eine Fantasiezahl.  1920×1080 bleibt nur die letzte
    Rückfallebene, wenn das Gerät gar nichts meldet.
    """
    gemeldet = [s for s in (schirme or []) if isinstance(s, dict)]
    treffer = _aufloesung(gemeldet, index)
    if treffer:
        return treffer[0], treffer[1], "gemeldet"
    leihe = _aufloesung(gemeldet, 0)
    if leihe:
        return leihe[0], leihe[1], "geliehen"
    return 1920, 1080, "Vorgabe"


def _aufloesung(schirme: list, index: int) -> Optional[tuple[int, int]]:
    """Auflösung eines gemeldeten Schirms, ``None`` wenn er fehlt."""
    if not (0 <= index < len(schirme or [])):
        return None
    schirm = schirme[index] or {}
    for feld in ("aufloesung", "modus"):
        text = str(schirm.get(feld) or "")
        treffer = re.match(r"(\d+)x(\d+)", text)
        if treffer:
            return int(treffer.group(1)), int(treffer.group(2))
    return None


def _schirmgroesse(schirme: list, index: int) -> tuple[int, int]:
    """Nur Breite/Höhe — Kurzform für das Layout."""
    breite, hoehe, _ = schirmgroesse(schirme, index)
    return breite, hoehe


def layout_aus_geraet(geraet: dict, schirme: Optional[list] = None,
                      engine: str = "", fps: int = 10,
                      gedreht: Optional[list] = None) -> dict:
    """Aus der Monitor-Zuordnung ein ``panels.json`` bauen.

    Ein Panel je Monitor fuellt den Schirm, zwei teilen ihn halb (links/rechts).
    ``gedreht[i]`` baut Monitor *i* als gedrehte Leinwand (Hochformat-Panel,
    z. B. ECAM oben/unten) — sonst normale Fenster.

    Beide Monitore werden **getrennt** behandelt: jeder rechnet mit seiner
    eigenen Auflösung, und für jeden gedrehten Monitor entsteht eine eigene
    Leinwand in ``leinwaende`` (nicht eine gemeinsame).
    """
    schirme = list(schirme or [])
    gedreht = list(gedreht or [])
    layout: dict[str, Any] = {"engine": engine, "panels": []}
    leinwaende: list[dict] = []
    for index, monitor in enumerate(("monitor1", "monitor2")):
        eintraege = [e for e in (geraet.get(monitor) or []) if str(e.get("panel") or "").strip()]
        if not eintraege:
            continue
        breite, hoehe = _schirmgroesse(schirme, index)
        gedreht_hier = bool(gedreht[index]) if index < len(gedreht) else False
        if gedreht_hier:
            # „gedreht" heißt: der Monitor ist schief verbaut, das **Bild** wird
            # ausgeglichen.  Die Fläche bleibt die echte Monitor-Größe, die
            # Anordnung (halbe/halbe) bleibt wie beim normalen Schirm — sonst
            # wandern die Panels auf dem Schirm an andere Plätze.
            dreh = int((eintraege[0] or {}).get("rotate") or 90)
            leinwaende.append({"screen": index, "width": breite,
                               "height": hoehe, "rotate": dreh})
            teile = len(eintraege)
            teil_breite = breite // teile
            for nummer, eintrag in enumerate(eintraege):
                layout["panels"].append({
                    "panel": str(eintrag["panel"]).strip(),
                    "screen": index, "x": nummer * teil_breite, "y": 0,
                    "width": teil_breite, "height": hoehe,
                    "fps": int(eintrag.get("fps") or fps),
                    "canvas": True,
                })
            continue
        teile = len(eintraege)
        teil_breite = breite // teile
        for nummer, eintrag in enumerate(eintraege):
            layout["panels"].append({
                "panel": str(eintrag["panel"]).strip(),
                "screen": index, "x": nummer * teil_breite, "y": 0,
                "width": teil_breite, "height": hoehe,
                "fps": int(eintrag.get("fps") or fps),
            })
    if leinwaende:
        layout["leinwaende"] = leinwaende
        layout["mode"] = "canvas"
        # Erste Leinwand auch althergebracht als ``canvas`` (ältere Clients)
        layout["canvas"] = {"width": leinwaende[0]["width"],
                            "height": leinwaende[0]["height"],
                            "rotate": leinwaende[0]["rotate"],
                            "screen": leinwaende[0]["screen"]}
    return layout
