#!/bin/sh
# ---------------------------------------------------------------------------
# Seriennummer des Anzeige-Clients ermitteln und ablegen (läuft als root).
#
# Warum überhaupt: /sys/class/dmi/id/product_serial ist 0400 root — der Agent
# läuft als "ssf" und kann die echte Seriennummer (Dell-Service-Tag, steht auf
# dem Gehäuse) nicht lesen. Deshalb schreibt dieser Oneshot sie einmalig nach
# /etc/fdc/serial (0644); Agent, IDENT und FDC lesen nur noch die Datei.
#
# Reihenfolge: Gehäuse-/Produkt-Seriennummer aus dem SMBIOS, sonst eine aus der
# MAC-Adresse erzeugte Ersatznummer (TC-xxxxxx) — ein Gerät ohne Kennung ist im
# Panel-Tab wertlos, weil man es nicht wiedererkennt.
#
# Idempotent: steht schon etwas in der Datei, passiert nichts.
# ---------------------------------------------------------------------------
set -u

ZIEL="${1:-/etc/fdc/serial}"

if [ -s "$ZIEL" ]; then
    echo "Seriennummer steht schon: $(cat "$ZIEL")"
    exit 0
fi

# Werte wie "To Be Filled By O.E.M." sind kein Ergebnis
brauchbar() {
    wert=$(printf '%s' "${1:-}" | tr -d ' \t\r\n')
    case "$wert" in
        ""|"0"|"None"|"none"|"Unknown"|"unknown") echo "" ;;
        "Default"*|"To Be Filled"*|"System Serial"*|"Not Specified"*|"Not Applicable"*) echo "" ;;
        *) echo "$wert" ;;
    esac
}

SN=""
for quelle in product_serial chassis_serial board_serial; do
    datei="/sys/class/dmi/id/$quelle"
    [ -r "$datei" ] || continue
    SN=$(brauchbar "$(cat "$datei" 2>/dev/null)")
    [ -n "$SN" ] && break
done

if [ -z "$SN" ]; then
    # Kein SMBIOS-Wert: aus der MAC der ersten echten Schnittstelle ableiten
    MAC=$(cat /sys/class/net/*/address 2>/dev/null \
          | grep -Ev '^00:00:00:00:00:00$' | head -1 | tr -d ':' | tr 'A-F' 'a-f')
    if [ -n "$MAC" ]; then
        SN="TC-$(printf '%s' "$MAC" | cut -c7-12)"
    else
        SN="TC-unbekannt"
    fi
fi

mkdir -p "$(dirname "$ZIEL")"
printf '%s\n' "$SN" > "$ZIEL"
chmod 644 "$ZIEL"
echo "Seriennummer: $SN -> $ZIEL"
