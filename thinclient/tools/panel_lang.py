"""Sprache am Anzeige-Client: folgt dem FDC, wird lokal gecacht.

Markus' Vorgabe (15.11.): *„Die Sprache am Panel soll der Sprache am FDC folgen
(und cachen falls sie ohne FDC starten)"*.

Reihenfolge der Quellen:

1. **FDC-Beacon** — das FDC schickt alle 15 s ``FDC;NAME:…;LANG:de;`` nach
   UDP 5011. Der Agent hört mit und legt die Sprache ab.
2. **Cache** ``~/.fdc/lang`` (beschreibbar für ``ssf``) — gilt, wenn kein FDC da
   ist, und überlebt den Neustart (``/run`` und ``/tmp`` tun das nicht).
3. **Installationssprache** ``/etc/fdc/lang`` — schreibt der Installer aus der
   Sprachauswahl des Debian-Installers.
4. **``en``** — Default der Anleitung am Bildschirm.

Die Texte hier sind absichtlich klein gehalten: nur was ohne Tastatur am
Bildschirm stehen muss (kein Bild, Einrichtung nötig, WLAN-Setup läuft).
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Optional

#: Sprachen, die FDC und Panel gemeinsam können (Reihenfolge = Menü im Setup).
SPRACHEN = ("de", "en", "fr", "th")
STANDARD = "en"

#: Name fürs Panel-Setup (Mehrsprachig, englischer Name dahinter).
SPRACHNAMEN = {"de": "Deutsch", "en": "English", "fr": "Français", "th": "ไทย"}

#: Installer-Default (root, vom Preseed geschrieben).
INSTALL_DATEI = Path("/etc/fdc/lang")


#: Benutzer, dem der Viewer/Agent gehört — dort liegt der Cache.
DIENST_BENUTZER = "ssf"


def _heim() -> Path:
    """Heimatverzeichnis des Dienst-Benutzers (Cache liegt dort, nicht in /run).

    Wichtig: ``power_idle`` läuft als **root** und schreibt den Cache trotzdem
    in die Heimat von ``ssf`` — sonst liest der Viewer (als ``ssf``) an einer
    anderen Stelle als der Schreiber und die Sprache bliebe leer.
    """
    for kandidat in (Path("/home") / DIENST_BENUTZER,
                     Path(os.path.expanduser("~"))):
        if kandidat.is_dir():
            return kandidat
    return Path(os.path.expanduser("~"))


def cache_datei() -> Path:
    return _heim() / ".fdc" / "lang"


# --------------------------------------------------------------------------- Texte
TEXTE: dict[str, dict[str, str]] = {
    "de": {
        "warte_erstes_bild": "warte auf erstes Bild …",
        "kein_bild_seit": "seit {s} s kein Bild",
        "engine_frage": "Panel in der Engine vorhanden?",
        "keine_daten": "{panel} — keine Daten",
        "unavail": "SYSTEMS UNAVAIL",
        "splash_titel": "Panel noch nicht eingerichtet",
        "splash_text": "Bitte dieses Panel im FDC einrichten:",
        "splash_weg": "Extras → Anzeige-Clients …",
        "splash_ip": "Adresse: {ip}",
        "splash_sprache": "Sprache: {sprache}",
        "wlan_titel": "WLAN einrichten",
        "wlan_text": "Kein Netzwerkkabel gefunden. Mit dem Handy verbinden:",
        "wlan_netz": "WLAN: {ssid}",
        "wlan_url": "Seite öffnen: {url}",
    },
    "en": {
        "warte_erstes_bild": "waiting for first frame …",
        "kein_bild_seit": "no frame for {s} s",
        "engine_frage": "Panel present in the engine?",
        "keine_daten": "{panel} — no data",
        "unavail": "SYSTEMS UNAVAIL",
        "splash_titel": "Panel not configured yet",
        "splash_text": "Please set up this panel in the FDC:",
        "splash_weg": "Extras → Display clients …",
        "splash_ip": "Address: {ip}",
        "splash_sprache": "Language: {sprache}",
        "wlan_titel": "Set up Wi-Fi",
        "wlan_text": "No network cable found. Connect with your phone:",
        "wlan_netz": "Network: {ssid}",
        "wlan_url": "Open page: {url}",
    },
    "fr": {
        "warte_erstes_bild": "en attente de la première image …",
        "kein_bild_seit": "aucune image depuis {s} s",
        "engine_frage": "Panneau présent dans l'engine ?",
        "keine_daten": "{panel} — aucune donnée",
        "unavail": "SYSTEMS UNAVAIL",
        "splash_titel": "Panneau pas encore configuré",
        "splash_text": "Veuillez configurer ce panneau dans le FDC :",
        "splash_weg": "Extras → Clients d'affichage …",
        "splash_ip": "Adresse : {ip}",
        "splash_sprache": "Langue : {sprache}",
        "wlan_titel": "Configurer le Wi-Fi",
        "wlan_text": "Aucun câble réseau détecté. Connectez-vous avec le téléphone :",
        "wlan_netz": "Réseau : {ssid}",
        "wlan_url": "Ouvrir la page : {url}",
    },
    "th": {
        "warte_erstes_bild": "กำลังรอภาพแรก …",
        "kein_bild_seit": "ไม่มีภาพมา {s} วินาที",
        "engine_frage": "มีแผงนี้ในเอนจินหรือไม่?",
        "keine_daten": "{panel} — ไม่มีข้อมูล",
        "unavail": "SYSTEMS UNAVAIL",
        "splash_titel": "ยังไม่ได้ตั้งค่าแผงนี้",
        "splash_text": "กรุณาตั้งค่าแผงนี้ใน FDC:",
        "splash_weg": "Extras → Display clients …",
        "splash_ip": "ที่อยู่: {ip}",
        "splash_sprache": "ภาษา: {sprache}",
        "wlan_titel": "ตั้งค่าไวไฟ",
        "wlan_text": "ไม่พบสายเครือข่าย เชื่อมต่อด้วยโทรศัพท์:",
        "wlan_netz": "เครือข่าย: {ssid}",
        "wlan_url": "เปิดหน้า: {url}",
    },
}


# --------------------------------------------------------------------------- Dateien
def _erste_zeile(pfad: Path) -> str:
    try:
        text = pfad.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""
    return text.splitlines()[0].strip().lower() if text else ""


def sprache_pruefen(wert: object) -> str:
    """``DE``, ``de-DE``, ``de_DE.UTF-8`` → ``de``; Unbekanntes → ``""``."""
    roh = str(wert or "").strip().lower().replace("-", "_")
    if not roh:
        return ""
    kurz = roh.split("_")[0]
    if kurz in SPRACHEN:
        return kurz
    if kurz.startswith("th"):
        return "th"
    return ""


def cache_lesen(datei: Optional[Path | str] = None) -> str:
    ziel = Path(datei) if datei is not None else cache_datei()
    return sprache_pruefen(_erste_zeile(ziel))


def cache_schreiben(sprache: str, datei: Optional[Path | str] = None) -> str:
    """Sprache merken (nur für ``ssf`` beschreibbar, nicht in /run)."""
    code = sprache_pruefen(sprache) or STANDARD
    ziel = Path(datei) if datei is not None else cache_datei()
    try:
        ziel.parent.mkdir(parents=True, exist_ok=True)
        ziel.write_text(code + "\n", encoding="utf-8")
        os.chmod(ziel, 0o644)
    except OSError:
        return code
    # root schreibt in die Heimat von ssf: Rechte mitgeben, sonst kommt der
    # Viewer später nicht mehr dran (oder nicht mehr weg davon).
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        try:
            import pwd
            eintrag = pwd.getpwnam(DIENST_BENUTZER)
            os.chown(ziel.parent, eintrag.pw_uid, eintrag.pw_gid)
            os.chown(ziel, eintrag.pw_uid, eintrag.pw_gid)
        except (KeyError, OSError):
            pass
    return code


def install_sprache(datei: Optional[Path | str] = None) -> str:
    ziel = Path(datei) if datei is not None else INSTALL_DATEI
    return sprache_pruefen(_erste_zeile(ziel))


def sprache(geraet: Optional[dict] = None,
            cache: Optional[Path | str] = None,
            install: Optional[Path | str] = None) -> str:
    """Aktive Sprache: Geräteeintrag → Cache → Installer → ``en``."""
    if isinstance(geraet, dict):
        aus_geraet = sprache_pruefen(geraet.get("lang") or geraet.get("language"))
        if aus_geraet:
            return aus_geraet
    return (cache_lesen(cache) or install_sprache(install) or STANDARD)


def text(schluessel: str, sprache_code: Optional[str] = None,
         geraet: Optional[dict] = None, **werte: object) -> str:
    """Text in der aktiven Sprache (fehlender Schlüssel ⇒ Englisch ⇒ Schlüssel)."""
    code = sprache_pruefen(sprache_code) or sprache(geraet)
    satz = (TEXTE.get(code, {}).get(schluessel)
            or TEXTE[STANDARD].get(schluessel)
            or schluessel)
    if werte:
        try:
            return satz.format(**werte)
        except (KeyError, IndexError, ValueError):
            return satz
    return satz


# --------------------------------------------------------------------------- Beacon
def aus_beacon(text_: str) -> str:
    """``FDC;NAME:x;ZEIT:1;LANG:de;`` → ``de`` (leer, wenn nicht drin)."""
    for teil in str(text_ or "").replace("\n", ";").split(";"):
        schluessel, _, wert = teil.partition(":")
        if schluessel.strip().upper() == "LANG":
            return sprache_pruefen(wert)
    return ""


def beacon_text(sprache_code: str) -> str:
    """Sprach-Teil für den FDC-Beacon (``LANG:de;``)."""
    code = sprache_pruefen(sprache_code)
    return f"LANG:{code};" if code else ""


def als_dict(sprache_code: Optional[str] = None) -> dict:
    """Kurzfassung für ``/status`` (Sprache + Beschriftung)."""
    code = sprache_pruefen(sprache_code) or sprache()
    return {"sprache": code, "sprachname": SPRACHNAMEN.get(code, code),
            "sprachen": list(SPRACHEN)}


if __name__ == "__main__":                     # kleine Prüfung auf der Box
    print(json.dumps({**als_dict(), "cache": str(cache_datei()),
                      "install": str(INSTALL_DATEI)}, ensure_ascii=False, indent=2))
