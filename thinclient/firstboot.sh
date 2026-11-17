#!/bin/sh
# ---------------------------------------------------------------------------
# Zweiter Teil der Einrichtung — läuft beim ERSTEN Start als root, mit Netz.
#
# Reihenfolge (so mit Markus abgestimmt):
#   1. Netz abwarten; ohne Kabel den WLAN-Einrichtungsmodus starten
#   2. Abhängigkeiten installieren (X, Qt, Netz, Werkzeuge)
#   3. Basisinstallation aus GitLab ziehen (Fallback GitHub)
#   4. Dienste einrichten und starten
#   5. System-Updates einspielen
#   6. Neustart falls nötig — danach läuft der Client im Beacon-Modus und zeigt
#      „Bitte Panel im FDC einrichten", bis das FDC ihn zuordnet.
#
# Ergebnis und Verlauf: /var/lib/fdc/firstboot.log
# ---------------------------------------------------------------------------
set -u

LOG=/var/lib/fdc/firstboot.log
MARKER=/var/lib/fdc/.firstboot-done
mkdir -p /var/lib/fdc
exec >>"$LOG" 2>&1

schritt() { echo; echo "=== $(date '+%F %T') $* ==="; }

if [ -e "$MARKER" ]; then
    echo "erster Start war schon — nichts zu tun"
    exit 0
fi

echo "=== $(date '+%F %T') FDC-Client: erster Start ==="
export DEBIAN_FRONTEND=noninteractive
APT="apt-get -y -qq -o Dpkg::Options::=--force-confold -o Dpkg::Options::=--force-confdef"

# --- 0. Laufende Einrichtung sichtbar machen --------------------------------
# Solange diese Datei liegt, verweigert der Agent Herunterfahren/Neustart
# (tools/panel_agent.py). Ein Reboot mitten in apt hinterlaesst ein halb
# konfiguriertes dpkg — dann fehlt der X-Server und der Viewer startet nie.
LAEUFT=/var/lib/fdc/.firstboot-laeuft
echo "$$" > "$LAEUFT"
trap 'rm -f "$LAEUFT"' EXIT INT TERM

# --- 0b. Halb fertige Paketdatenbank reparieren ----------------------------
# Kam ein Reboot oder Stromausfall mitten in apt, sind Pakete entpackt aber
# nicht konfiguriert. Jedes weitere apt laeuft dann auf Fehler; Pakete wie
# xserver-xorg fehlen still. Deshalb erst aufraeumen, dann installieren.
if dpkg --audit 2>/dev/null | grep -q . || dpkg -l 2>/dev/null | grep -qE '^i[UF] '; then
    echo "dpkg halb fertig — ziehe es gerade"
    dpkg --configure -a || true
    $APT -f install || true
fi

# --- 1. Netz ---------------------------------------------------------------
netz_da() {
    # Ein Ziel im Internet erreichen ist der ehrlichste Test (DNS + Routing).
    for _ in 1 2 3; do
        ping -c1 -W2 1.1.1.1 >/dev/null 2>&1 && return 0
        sleep 2
    done
    return 1
}

schritt "Netz prüfen"
if ! netz_da; then
    echo "kein Netz über Kabel — WLAN-Einrichtung starten (Anleitung am Bildschirm)"
    systemctl start fdc-wlan-setup.service 2>/dev/null || true
    # Der Einrichtungs-AP läuft, bis jemand am Handy ein WLAN einträgt.
    i=0
    while [ $i -lt 900 ]; do          # 30 Minuten Gnadenfrist
        netz_da && break
        i=$((i + 5))
        sleep 5
    done
    systemctl stop fdc-wlan-setup.service 2>/dev/null || true
    if ! netz_da; then
        echo "FEHLER: kein Netz nach 30 min — absichtlich KEIN Marker gesetzt,"
        echo "        der Dienst versucht es beim nächsten Start erneut."
        exit 1
    fi
fi
echo "Netz steht"

# --- 1b. Uhrzeit -----------------------------------------------------------
# Vor jedem TLS-Zugriff: eine falsche Uhr lässt jedes Zertifikat „not yet
# valid" aussehen und apt seit Debian 12 ebenfalls scheitern. Das Werkzeug ist
# schon bei der Installation mitgekommen (install-offline.sh).
schritt "Uhrzeit prüfen"
# Umlaute im Kommentar wären hier egal, die Zeilen darunter nicht: erst NTP
# scharf schalten (installiert die Vorlage vom Medium), dann als Notnagel die
# Zeit aus dem Date-Kopf einer HTTP-Antwort holen — das braucht kein TLS.
if [ -s /root/fdc-iso/timesyncd.conf ]; then
    install -m 644 /root/fdc-iso/timesyncd.conf /etc/systemd/timesyncd.conf
fi
if [ -s /root/fdc-iso/timesyncd-lokal.conf ]; then
    mkdir -p /etc/systemd/timesyncd.conf.d
    install -m 644 /root/fdc-iso/timesyncd-lokal.conf /etc/systemd/timesyncd.conf.d/90-lokal.conf
fi
systemctl enable --now systemd-timesyncd 2>/dev/null || true
timedatectl set-ntp true 2>/dev/null || true
[ -x /opt/fdc/clock-fix.sh ] && /opt/fdc/clock-fix.sh || true

# --- 2. Abhängigkeiten -----------------------------------------------------
schritt "Paketlisten und Abhängigkeiten"
$APT update || echo "WARNUNG: apt update mit Fehlern"
PAKETE="openssh-server sudo systemd-timesyncd curl ca-certificates gnupg git jq \
        python3-websocket \
        xserver-xorg xinit x11-xserver-utils xserver-xorg-video-fbdev unclutter-xfixes \
        python3 python3-pyside6.qtcore python3-pyside6.qtgui python3-pyside6.qtwidgets \
        python3-pyside6.qtnetwork network-manager wpasupplicant dnsmasq-base iproute2"
$APT install $PAKETE || echo "WARNUNG: einzelne Pakete fehlen (siehe oben)"
# Zweiter Anlauf für den Fall, dass der Installer sein Netz nicht gefunden hat
# (dann kam pkgsel nicht durch und SSH/Sudo fehlen). Kostet nichts, rettet aber
# den Zugang: ohne sshd kommt niemand mehr auf die Kiste.  python3-websocket
# gehört dazu — ohne das Modul verbindet der Viewer keinen Stream und bleibt
# schwarz, obwohl Konfiguration und Engine stimmen.
for paket in openssh-server sudo systemd-timesyncd python3-websocket; do
    dpkg-query -W -f='${Status}' "$paket" 2>/dev/null | grep -q 'install ok installed' \
        || { echo "  $paket fehlt — hole nach"; $APT install "$paket" || true; }
done
systemctl enable --now ssh 2>/dev/null || true
python3 -c 'import websocket' 2>/dev/null || echo "WARNUNG: websocket-Modul fehlt — Stream bleibt schwarz"
python3 -c 'import PySide6, PySide6.QtWidgets; print("PySide6", PySide6.__version__)' \
    || echo "WARNUNG: PySide6 nicht importierbar"

# --- 3. Basisinstallation aus GitLab (Fallback GitHub) ---------------------
schritt "Basisinstallation ziehen"
# Reihenfolge: mitgeliefertes Paket vom ISO → GitLab → GitHub.  Das ISO-Paket
# ist der Stand zum Build-Zeitpunkt und macht den ersten Start netzunabhängig;
# die beiden URLs ziehen neuere Stände nach.
QUELLEN="datei:/root/fdc-iso/paket.tar.gz
https://gitlab.pcq-online.com/BigMaQ/SSF-FIRMWARE/-/archive/main/SSF-FIRMWARE-main.tar.gz
https://github.com/BigMaQ/SSF-FIRMWARE/archive/refs/heads/main.tar.gz"
TMP=$(mktemp -d)
GEHOLT=""
for url in $QUELLEN; do
    echo "versuche: $url"
    if [ "${url#datei:}" != "$url" ]; then
        quelle="${url#datei:}"
        [ -s "$quelle" ] && cp "$quelle" "$TMP/pkg.tgz"
    else
        curl -fsSL --max-time 120 -o "$TMP/pkg.tgz" "$url"
    fi
    if [ -s "$TMP/pkg.tgz" ] && tar -tzf "$TMP/pkg.tgz" >/dev/null 2>&1; then
        GEHOLT="$url"
        break
    fi
    echo "  fehlgeschlagen"
done
if [ -z "$GEHOLT" ]; then
    echo "FEHLER: keine Quelle erreichbar (GitLab und GitHub)"
    rm -rf "$TMP"
    exit 1
fi
echo "Quelle: $GEHOLT"
tar -xzf "$TMP/pkg.tgz" -C "$TMP"
WURZEL=$(find "$TMP" -maxdepth 1 -mindepth 1 -type d | head -1)
# Im Firmware-Repo liegt der Client unter thinclient/ (so heißt auch der
# OTA-Ordner) — nicht im Wurzelverzeichnis des Repos.
CODE="$WURZEL/thinclient"
[ -d "$CODE/tools" ] || CODE="$WURZEL"
rm -rf /opt/fdc.alt
[ -d /opt/fdc ] && mv /opt/fdc /opt/fdc.alt
mkdir -p /opt/fdc
cp -r "$CODE/infra" "$CODE/tools" /opt/fdc/
cp "$CODE/wait-for-x.sh" "$CODE/serial-erzeugen.sh" /opt/fdc/ 2>/dev/null || true
if [ -s "$CODE/version.json" ]; then
    jq -r .version "$CODE/version.json" > /opt/fdc/VERSION 2>/dev/null || echo "0.0.0" > /opt/fdc/VERSION
elif [ -s "$CODE/VERSION" ]; then
    cp "$CODE/VERSION" /opt/fdc/VERSION
fi
cp /root/fdc-iso/firstboot.sh /opt/fdc/firstboot.sh
chmod 755 /opt/fdc/*.sh 2>/dev/null || true
# Units kommen vom ISO: die systemd-Dateien liegen im PRIVATEN FDC-Repo, das
# öffentliche Firmware-Repo trägt nur den Programmcode (thinclient/).
UNITS=/var/lib/fdc/units
rm -rf "$UNITS"; mkdir -p "$UNITS"
for DATEI in /root/fdc-iso/units/*.service; do
    [ -s "$DATEI" ] && cp "$DATEI" "$UNITS/"
done
[ -n "$(ls -A "$UNITS" 2>/dev/null)" ] || echo "WARNUNG: keine Units im ISO-Kit"
rm -rf "$TMP"
echo "installiert: $(cat /opt/fdc/VERSION 2>/dev/null || echo unbekannt)"

# --- 4. Dienste ------------------------------------------------------------
schritt "Dienste einrichten"
install -m 755 -d /etc/fdc
for DATEI in "$UNITS"/*.service; do
    [ -s "$DATEI" ] || continue
    install -m 644 "$DATEI" "/etc/systemd/system/$(basename "$DATEI")"
done
systemctl daemon-reload
# Seriennummer zuerst (das IDENT der anderen Dienste hängt daran) — das Skript
# kommt mit dem Paket, deshalb hier nochmal schreiben, diesmal aus /opt/fdc.
[ -x /opt/fdc/serial-erzeugen.sh ] && /opt/fdc/serial-erzeugen.sh /etc/fdc/serial
systemctl enable --now fdc-rechte.service 2>/dev/null || true
systemctl enable --now fdc-ident.service 2>/dev/null || true
# enable --now, nicht nur enable: frisch installiert läuft bis zum ersten Neustart
# sonst NICHTS davon — Auto-Power-off und OTA blieben stumm.
# Uhr-Dienst ist eigenständig (läuft bei jedem Start vor dem Agent)
systemctl enable --now fdc-clock.service 2>/dev/null || true

# Rechte in /etc/fdc: Installer schreibt als root, gespeichert wird als ssf.
# Idempotent — repariert auch Boxen, die von einer älteren Version kommen.
if [ -f /opt/fdc/tools/ota_update.py ]; then
    python3 /opt/fdc/tools/ota_update.py --rechte || true
fi
for dienst in fdc-agent fdc-panel fdc-poweroff fdc-ota fdc-wlan-setup; do
    systemctl enable --now "$dienst.service" 2>/dev/null || true
done
# fdc-wlan-setup läuft bewusst DAUERHAFT: der Wächter darin entscheidet selbst
# zwischen LAN / WLAN / Einrichtungs-AP.  Vorher wurde er hier abgeschaltet —
# Folge: Kabel gezogen, und es passierte nichts, obwohl die Box dann nur noch
# über den AP erreichbar wäre.

# Beacon-Modus: der Agent meldet sich, der Viewer zeigt den Einrichtungs-Hinweis
systemctl restart fdc-agent.service 2>/dev/null || systemctl start fdc-agent.service 2>/dev/null || true
systemctl restart fdc-panel.service 2>/dev/null || systemctl start fdc-panel.service 2>/dev/null || true

# --- 5. System-Updates -----------------------------------------------------
schritt "System-Updates"
$APT full-upgrade || echo "WARNUNG: full-upgrade mit Fehlern"
$APT autoremove -y >/dev/null 2>&1 || true

# --- 5b. OTA abwarten, bevor irgendetwas neu startet -----------------------
# fdc-ota.service wurde oben gestartet und schreibt /opt/fdc neu. Ein Neustart
# mitten in diesem Lauf schneidet die Dateiliste ab — genau so fehlten schon
# clock-fix.sh und timesyncd.conf, waehrend Tools und Units da waren.
schritt "OTA abwarten"
for _ in $(seq 1 60); do
    systemctl is-active --quiet fdc-ota.service || break
    sleep 5
done
systemctl is-active --quiet fdc-ota.service && echo "WARNUNG: OTA laeuft noch (5 min)"

# --- 5c. Vollstaendigkeit von /opt/fdc pruefen -----------------------------
# Waehrend der Einrichtung ist schon einmal eine Dateiliste halb geschrieben
# worden: clock-fix.sh und timesyncd.conf fehlten, waehrend Tools und Units da
# waren — Folge: fdc-clock startet mit 203/EXEC, die Uhr bleibt falsch.
# Das ISO-Kit liegt noch unter /root/fdc-iso, also von dort nachziehen.
schritt "Dateien in /opt/fdc pruefen"
for DATEI in clock-fix.sh timesyncd.conf wait-for-x.sh serial-erzeugen.sh firstboot.sh; do
    if [ ! -s "/opt/fdc/$DATEI" ] && [ -s "/root/fdc-iso/$DATEI" ]; then
        install -m 755 "/root/fdc-iso/$DATEI" "/opt/fdc/$DATEI" && echo "  nachgezogen: $DATEI"
    fi
done
for DATEI in infra/panel_stream.py tools/panel_viewer.py tools/panel_agent.py; do
    [ -s "/opt/fdc/$DATEI" ] || echo "  WARNUNG: /opt/fdc/$DATEI fehlt"
done
systemctl restart fdc-clock.service 2>/dev/null || true

# --- 5d. Dienste wirklich eingeschaltet? -----------------------------------
# Ein abgebrochener Erstlauf hat fdc-wlan-setup "disabled" zurueckgelassen: kein
# Einrichtungs-AP, kein Hotspot am Handy — und am Panel stand nur "no network".
# enable --now ist idempotent, kostet nichts und deckt genau diese Luecke.
schritt "Dienste pruefen"
for DIENST in fdc-agent fdc-panel fdc-ota fdc-poweroff fdc-wlan-setup fdc-clock fdc-rechte fdc-ident; do
    if ! systemctl is-enabled --quiet "$DIENST.service" 2>/dev/null; then
        echo "  $DIENST war aus — wird eingeschaltet"
        systemctl enable --now "$DIENST.service" 2>/dev/null || echo "  WARNUNG: $DIENST laesst sich nicht einschalten"
    fi
done

# --- 6. Neustart, falls nötig (genau einmal) -------------------------------
schritt "Abschluss"
touch "$MARKER"
# reboot-required setzt nicht jedes Paket (beim Kernel-Upgrade fehlt sie oft) —
# deshalb zusätzlich prüfen, ob der laufende Kernel noch der neueste ist.
neuer_kernel() {
    installiert=$(dpkg-query -W -f '${Package}\n' 'linux-image-*-amd64' 2>/dev/null \
                  | sed -e 's/^linux-image-//' -e 's/-amd64$//' | sort -V | tail -1)
    [ -n "${installiert:-}" ] && [ "$installiert" != "$(uname -r)" ]
}
if [ -e /var/run/reboot-required ] || [ -e /run/reboot-required ] || neuer_kernel; then
    echo "Neustart nötig (Kernel: $(uname -r), installiert: $(dpkg-query -W -f '${Package}\n' 'linux-image-*-amd64' 2>/dev/null | sed -e 's/^linux-image-//' -e 's/-amd64$//' | sort -V | tail -1)) — jetzt"
    sync
    systemctl reboot
    exit 0
fi
echo "fertig ohne Neustart — Dienste laufen:"
systemctl is-active fdc-agent fdc-panel fdc-poweroff fdc-ota 2>&1
