#!/bin/sh
# Uhrzeit des Clients prüfen und notfalls stellen — läuft als root vor dem Agent.
#
# Warum: ohne richtige Uhr scheitert jedes TLS-Update („certificate is not yet
# valid"), und apt prüft seit Debian 12 ebenfalls NotBefore. Ein Client mit
# leerer CMOS-Batterie bleibt sonst dauerhaft ohne Updates.
set -u
FDC="${FDC_DIR:-/opt/fdc}"
SCHWELLE="${1:-300}"

echo "clock-fix: $(date -Is)"

# NTP anschalten, falls möglich (die eigentliche Korrektur macht das Python-Tool)
timedatectl set-ntp true 2>/dev/null || true

if [ -x "$FDC/tools/clock_sync.py" ] || [ -f "$FDC/tools/clock_sync.py" ]; then
    python3 "$FDC/tools/clock_sync.py" --korrigieren --schwelle "$SCHWELLE" || true
else
    echo "clock-fix: $FDC/tools/clock_sync.py fehlt" >&2
fi

echo "clock-fix danach: $(date -Is)"
exit 0
