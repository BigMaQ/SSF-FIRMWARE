#!/bin/sh
# Wartet, bis der X-Server auf :0 bereit ist (der Viewer startet als Dienst,
# X kommt aus der Konsolen-Sitzung - die Reihenfolge ist nicht garantiert).
i=0
while [ $i -lt 120 ]; do
    if [ -e /tmp/.X11-unix/X0 ]; then
        exit 0
    fi
    i=$((i + 1))
    sleep 1
done
echo "X auf :0 kam nicht hoch - startet getty@tty1 mit Autologin?" >&2
exit 1
