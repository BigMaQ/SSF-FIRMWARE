#!/usr/bin/env python3
"""OTA-Update für Anzeige-Clients — sucht im (öffentlichen) GitLab-Repo nach Updates.

Der Client holt ein Manifest (``version.json``) über HTTPS, vergleicht die
Versionsnummer mit der eigenen und lädt bei Bedarf die einzelnen Dateien nach.
Kein git auf dem Client, keine Anmeldung: das Repo ist öffentlich, das Manifest
enthält je Datei eine SHA-256-Summe.

Ablauf beim Installieren:

1. jede Datei nach ``/opt/fdc.new/…`` laden und die Summe prüfen
2. ``/opt/fdc`` → ``/opt/fdc.alt`` (Sicherung), ``/opt/fdc.new`` → ``/opt/fdc``
3. Dienste neu starten; schlägt das fehl, zurückrollen

Das Ganze läuft als root (schreibt nach ``/opt/fdc``), gestartet als Dienst
``fdc-ota.service``.  Ergebnis landet in ``/run/fdc-ota.json`` — von dort lesen
der Agent (Anzeige im FDC) und der Viewer (Popup) mit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional

try:                                    # als Paket (Tests, FDC-Tools)
    from tools import os_update
except ImportError:                     # direkt aus /opt/fdc/tools gestartet
    import os_update                    # type: ignore

STANDARD_ZIEL = "/opt/fdc"
STANDARD_STATE = "/run/fdc-ota.json"
STANDARD_KONFIG = "/etc/fdc/ota.json"
STANDARD_DIENSTE = ("fdc-panel", "fdc-agent")
MAX_DATEI_BYTES = 8 * 1024 * 1024
#: Voreinstellung: öffentliches Repo mit den Client-Dateien.
STANDARD_MANIFEST = ("https://gitlab.pcq-online.com/BigMaQ/fdc-thinclient/"
                     "-/raw/main/version.json")


# ---------------------------------------------------------------------------
# Versionen
# ---------------------------------------------------------------------------
def version_zerlegen(text: str) -> tuple[int, ...]:
    """``"0.2.1"`` → ``(0, 2, 1)``; Unfug wird zu ``(0,)``."""
    zahlen = []
    for teil in str(text or "").strip().lstrip("vV").split("."):
        ziffern = "".join(zeichen for zeichen in teil if zeichen.isdigit())
        if not ziffern:
            break
        zahlen.append(int(ziffern))
    return tuple(zahlen) or (0,)


def ist_neuer(neu: str, alt: str) -> bool:
    """Ist ``neu`` echt neuer als ``alt``? Gleiche Version = kein Update."""
    neu_t, alt_t = version_zerlegen(neu), version_zerlegen(alt)
    laenge = max(len(neu_t), len(alt_t))
    neu_t += (0,) * (laenge - len(neu_t))
    alt_t += (0,) * (laenge - len(alt_t))
    return neu_t > alt_t


def lokale_version(ziel: str = STANDARD_ZIEL) -> str:
    for pfad in (Path(ziel) / "VERSION", Path(ziel) / "version.txt"):
        try:
            text = pfad.read_text(encoding="utf-8").strip()
        except OSError:
            continue
        if text:
            return text.splitlines()[0].strip()
    return "0.0.0"


# ---------------------------------------------------------------------------
# Netz
# ---------------------------------------------------------------------------
def hole(url: str, timeout: float = 20.0) -> bytes:
    anfrage = urllib.request.Request(url, headers={"Cache-Control": "no-cache",
                                                   "User-Agent": "FDC-Client-OTA"})
    try:
        with urllib.request.urlopen(anfrage, timeout=timeout) as antwort:
            return antwort.read(MAX_DATEI_BYTES)
    except urllib.error.HTTPError as exc:
        # Klartext statt „Expecting value": im FDC-Fenster landet das wortwoertlich.
        hinweis = {404: "Datei nicht gefunden — Repo oeffentlich? Pfad/Branch richtig?",
                   401: "Anmeldung verlangt — Repo ist nicht oeffentlich",
                   403: "Zugriff verboten"}.get(exc.code, "unerwartete Antwort")
        raise ValueError(f"HTTP {exc.code} bei {url}: {hinweis}") from exc


def manifest_holen(url: str, timeout: float = 20.0) -> dict:
    roh = hole(url, timeout)
    text = roh.decode("utf-8", "replace").lstrip("\ufeff \t\r\n")
    if text[:1] not in ("{", "["):
        # GitLab schickt bei privaten Repos eine Login-Seite statt 401.
        raise ValueError("Manifest ist HTML statt JSON — Repo nicht öffentlich "
                         "oder Pfad/Branch falsch")
    try:
        daten = json.loads(text)
    except ValueError as exc:
        raise ValueError(f"Manifest ist kein JSON: {exc}") from exc
    if not isinstance(daten, dict) or not str(daten.get("version") or "").strip():
        raise ValueError("Manifest ohne Version")
    if not isinstance(daten.get("dateien"), list) or not daten["dateien"]:
        raise ValueError("Manifest ohne Dateiliste")
    return daten


def datei_url(manifest_url: str, basis: Optional[str], pfad: str) -> str:
    """Adresse einer Einzeldatei bilden.

    ``basis`` im Manifest (oder als Parameter) hat Vorrang; sonst wird von der
    Manifest-Adresse auf das Raw-Verzeichnis umgehängt.
    """
    if basis:
        return basis.rstrip("/") + "/" + pfad.lstrip("/")
    if "/-/raw/" in manifest_url:
        return manifest_url.split("/-/raw/")[0] + "/-/raw/" + \
            manifest_url.split("/-/raw/")[1].rsplit("/", 1)[0] + "/" + pfad.lstrip("/")
    return manifest_url.rsplit("/", 1)[0] + "/" + pfad.lstrip("/")


# ---------------------------------------------------------------------------
# Zustand für Agent und Viewer
# ---------------------------------------------------------------------------
def zustand_schreiben(pfad: str, daten: dict) -> None:
    daten = dict(daten)
    daten["geprueft"] = int(time.time())
    try:
        ziel = Path(pfad)
        ziel.parent.mkdir(parents=True, exist_ok=True)
        tmp = ziel.with_suffix(ziel.suffix + ".tmp")
        tmp.write_text(json.dumps(daten, indent=2, ensure_ascii=False) + "\n",
                       encoding="utf-8")
        os.chmod(tmp, 0o644)
        tmp.replace(ziel)
    except OSError:
        pass


def zustand_lesen(pfad: str = STANDARD_STATE) -> dict:
    try:
        return json.loads(Path(pfad).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def konfiguration_lesen(pfad: str = STANDARD_KONFIG) -> dict:
    try:
        daten = json.loads(Path(pfad).read_text(encoding="utf-8"))
        return daten if isinstance(daten, dict) else {}
    except (OSError, ValueError):
        return {}


# ---------------------------------------------------------------------------
# Prüfen und Installieren
# ---------------------------------------------------------------------------
def pruefen(manifest_url: str, ziel: str = STANDARD_ZIEL,
            timeout: float = 20.0) -> dict:
    """Manifest holen und mit der lokalen Version vergleichen."""
    aktuell = lokale_version(ziel)
    try:
        manifest = manifest_holen(manifest_url, timeout)
    except (urllib.error.URLError, OSError, ValueError) as exc:
        return {"verfuegbar": False, "aktuell": aktuell, "version": aktuell,
                "fehler": f"{type(exc).__name__}: {exc}"[:200]}
    neu = str(manifest.get("version") or "").strip()
    ergebnis = {"verfuegbar": ist_neuer(neu, aktuell), "aktuell": aktuell,
                "version": neu, "hinweis": str(manifest.get("hinweis") or ""),
                "datum": str(manifest.get("datum") or ""),
                "manifest": manifest_url}
    if ergebnis["verfuegbar"]:
        ergebnis["dateien"] = [str(d.get("pfad") or "") for d in manifest["dateien"]
                               if isinstance(d, dict)]
    return ergebnis


def installieren(manifest_url: str, ziel: str = STANDARD_ZIEL,
                 dienste: tuple[str, ...] = STANDARD_DIENSTE,
                 state_pfad: str = STANDARD_STATE,
                 timeout: float = 25.0,
                 neustart: bool = True) -> dict:
    """Dateien laden, prüfen, austauschen.  Bei Fehler bleibt alles wie es war."""
    ergebnis = pruefen(manifest_url, ziel, timeout)
    if not ergebnis.get("verfuegbar"):
        zustand_schreiben(state_pfad, {"verfuegbar": False,
                                       "aktuell": ergebnis.get("aktuell"),
                                       "version": ergebnis.get("version"),
                                       "meldung": "kein Update nötig",
                                       "fehler": ergebnis.get("fehler", "")})
        return ergebnis

    manifest = manifest_holen(manifest_url, timeout)
    basis = str(manifest.get("basis") or konfiguration_lesen().get("dateien_basis") or "")
    staging = Path(str(ziel) + ".new")
    if staging.exists():
        shutil.rmtree(staging, ignore_errors=True)
    # Die bestehende Installation als Grundlage kopieren: im Manifest stehen nur
    # die Programmdateien.  Ein leeres Staging hätte alles andere gelöscht
    # (real passiert: /opt/fdc/wait-for-x.sh fehlte danach, der Viewer lief in
    # eine Neustart-Schleife).
    if Path(ziel).exists():
        shutil.copytree(ziel, staging)
    else:
        staging.mkdir(parents=True)

    try:
        for eintrag in manifest["dateien"]:
            pfad = str(eintrag.get("pfad") or "").strip()
            summe = str(eintrag.get("sha256") or "").strip().lower()
            if not pfad or pfad.startswith("/") or ".." in pfad.split("/"):
                raise ValueError(f"unsicherer Pfad im Manifest: {pfad!r}")
            daten = hole(datei_url(manifest_url, basis, pfad), timeout)
            if summe and hashlib.sha256(daten).hexdigest() != summe:
                raise ValueError(f"Prüfsumme falsch: {pfad}")
            ziel_datei = staging / pfad
            ziel_datei.parent.mkdir(parents=True, exist_ok=True)
            ziel_datei.write_bytes(daten)
        (staging / "VERSION").write_text(str(manifest["version"]).strip() + "\n",
                                        encoding="utf-8")
    except Exception as exc:                                # noqa: BLE001
        shutil.rmtree(staging, ignore_errors=True)
        zustand_schreiben(state_pfad, {"verfuegbar": True,
                                       "aktuell": lokale_version(ziel),
                                       "version": str(manifest.get("version")),
                                       "meldung": "Installation abgebrochen",
                                       "fehler": f"{type(exc).__name__}: {exc}"[:250]})
        raise

    # Vor dem Austausch melden, dass es läuft: der Dienst fdc-panel wird gleich
    # neu gestartet und der Viewer soll „Update wird installiert" zeigen.
    zustand_schreiben(state_pfad, {"laeuft": True, "verfuegbar": True,
                                   "aktuell": lokale_version(ziel),
                                   "version": str(manifest["version"])})

    sicherung = Path(str(ziel) + ".alt")
    if sicherung.exists():
        shutil.rmtree(sicherung, ignore_errors=True)
    if Path(ziel).exists():
        Path(ziel).rename(sicherung)
    staging.rename(ziel)

    neustart_ok, meldung = True, ""
    if neustart:
        neustart_ok, meldung = dienste_neu_starten(dienste)
        if not neustart_ok:
            # Rollback: ein halb laufender Client ist schlimmer als eine alte Version.
            shutil.rmtree(ziel, ignore_errors=True)
            if sicherung.exists():
                sicherung.rename(ziel)
            dienste_neu_starten(dienste)
            zustand_schreiben(state_pfad, {
                "verfuegbar": True, "aktuell": lokale_version(ziel),
                "version": str(manifest.get("version")),
                "meldung": "Neustart fehlgeschlagen — zurückgerollt",
                "fehler": meldung[:250]})
            return {**ergebnis, "installiert": False, "zurueckgerollt": True,
                    "fehler": meldung[:250]}

    zustand_schreiben(state_pfad, {"stand": time.time(),
                                   "verfuegbar": False,
                                   "aktuell": str(manifest["version"]),
                                   "version": str(manifest["version"]),
                                   "meldung": f"aktualisiert auf {manifest['version']}",
                                   "fehler": ""})
    return {**ergebnis, "installiert": True, "aktuell": str(manifest["version"])}


def dienste_neu_starten(dienste: tuple[str, ...]) -> tuple[bool, str]:
    """Dienste neu starten; Rückgabe (ok, Meldung)."""
    meldungen = []
    for dienst in dienste:
        try:
            ergebnis = subprocess.run(["systemctl", "restart", dienst],
                                      capture_output=True, timeout=90)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, f"{dienst}: {type(exc).__name__}: {exc}"
        if ergebnis.returncode != 0:
            meldungen.append((ergebnis.stderr or b"").decode("utf-8", "replace").strip())
    if meldungen:
        return False, "; ".join(meldungen)[:250]
    return True, "Dienste neu gestartet"


# ---------------------------------------------------------------------------
# Dienst-Schleife
# ---------------------------------------------------------------------------
class Waechter:
    """Prüft zyklisch auf Updates (ohne zu installieren)."""

    def __init__(self, manifest_url: str, state_pfad: str = STANDARD_STATE,
                 ziel: str = STANDARD_ZIEL, intervall_s: float = 21600.0) -> None:
        self.manifest_url = manifest_url
        self.state_pfad = state_pfad
        self.ziel = ziel
        self.intervall_s = intervall_s
        self._laufen = True

    def einmal(self) -> dict:
        ergebnis = pruefen(self.manifest_url, self.ziel)
        alt = zustand_lesen(self.state_pfad)
        # Ein gemeldeter Stand bleibt sichtbar, solange die Prüfung nicht
        # erfolgreich war: sonst blinkt „Update verfügbar" nach einem
        # Netzaussetzer weg und niemand sieht es wieder.
        if ergebnis.get("fehler") and alt.get("verfuegbar"):
            zustand_schreiben(self.state_pfad, alt)
            ergebnis = alt
        else:
            zustand_schreiben(self.state_pfad, ergebnis)
        try:
            self._automaten(ergebnis)
        except Exception as exc:                        # noqa: BLE001 - nie sterben
            print(f"[ota] Automatik: {exc}", file=sys.stderr, flush=True)
        return ergebnis

    # -- Automatik: Firmware und OS getrennt -------------------------------
    def _automaten(self, ergebnis: dict) -> None:
        """Schalter aus ``/etc/fdc/auto.json`` ausfuehren.

        ``firmware_auto``  — verfuegbares Update sofort einspielen
        ``os_auto``        — OS-Updates hoechstens alle ``os_auto_abstand_h``
                             Stunden einspielen (eigene Unit, damit ein
                             Dienst-Neustart mitten im Lauf nichts abbricht)
        Der OS-Stand wird immer mitgeschrieben, damit das FDC zeigen kann, ob
        Sicherheitsupdates anstehen — unabhaengig von der Automatik.
        """
        cfg = os_update.auto_config_lesen()
        try:
            os_update.pruefen(schreiben=True)
        except Exception:                                # noqa: BLE001
            pass
        if cfg.get("firmware_auto") and ergebnis.get("verfuegbar"):
            print(f"[ota] Automatik: installiere {ergebnis.get('version')}", flush=True)
            installieren(self.manifest_url, self.ziel, state_pfad=self.state_pfad)
            return
        if cfg.get("os_auto"):
            stand = os_update.zustand_lesen()
            anzahl = int(stand.get("anzahl") or 0)
            abstand = float(cfg.get("os_auto_abstand_h") or 24.0) * 3600.0
            letzte = float(stand.get("auto_zeit") or 0.0)
            if anzahl > 0 and (time.time() - letzte) > abstand:
                print(f"[ota] Automatik: {anzahl} OS-Updates einspielen", flush=True)
                os_update.zustand_schreiben({"laeuft": True, "auto_zeit": time.time()})
                try:
                    subprocess.Popen(["systemd-run", "--unit=fdc-os-install",
                                      "--collect", "--description=FDC OS-Update",
                                      sys.executable, "/opt/fdc/tools/os_update.py",
                                      "--installieren"],
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL,
                                     stdin=subprocess.DEVNULL, close_fds=True)
                except OSError as exc:
                    print(f"[ota] OS-Update nicht gestartet: {exc}", flush=True)

    def laufen(self) -> None:
        while self._laufen:
            try:
                self.einmal()
            except Exception as exc:                        # noqa: BLE001 - nie sterben
                print(f"[ota] Prüfung fehlgeschlagen: {exc}", file=sys.stderr, flush=True)
            for _ in range(int(max(60.0, self.intervall_s))):
                if not self._laufen:
                    break
                time.sleep(1.0)

    def stoppen(self) -> None:
        self._laufen = False


def main(argv: Optional[list[str]] = None) -> int:
    konfig = konfiguration_lesen()
    zerleger = argparse.ArgumentParser(description="OTA-Update für Anzeige-Clients")
    zerleger.add_argument("--manifest", default=konfig.get("manifest") or STANDARD_MANIFEST)
    zerleger.add_argument("--ziel", default=STANDARD_ZIEL)
    zerleger.add_argument("--state", default=STANDARD_STATE)
    zerleger.add_argument("--intervall", type=float,
                          default=float(konfig.get("pruefintervall_s") or 21600))
    zerleger.add_argument("--pruefen", action="store_true",
                          help="nur prüfen, Zustand schreiben, beenden")
    zerleger.add_argument("--installieren", action="store_true",
                          help="sofort installieren")
    args = zerleger.parse_args(argv)

    if args.installieren:
        try:
            ergebnis = installieren(args.manifest, args.ziel, state_pfad=args.state)
        except Exception as exc:                            # noqa: BLE001
            print(json.dumps({"ok": False, "fehler": f"{type(exc).__name__}: {exc}"},
                             ensure_ascii=False))
            return 1
        print(json.dumps(ergebnis, ensure_ascii=False))
        return 0

    waechter = Waechter(args.manifest, args.state, args.ziel, args.intervall)
    if args.pruefen:
        print(json.dumps(waechter.einmal(), ensure_ascii=False))
        return 0

    print(f"[ota] prüfe {args.manifest} alle {int(args.intervall)} s", flush=True)
    try:
        waechter.laufen()
    except KeyboardInterrupt:
        pass
    finally:
        waechter.stoppen()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
