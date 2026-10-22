#!/usr/bin/env python3
"""Approach-Simulator fuer das ISIS-Panel (ohne FDC, ohne Sim).

Simuliert einen kompletten ILS-Anflug: von 10.000 ft / 12 NM Endanflug
runter bis auf die Landebahn, mit Localizer- und Gleitpfad-Ablagen, die
sich beim Auffangen auf Null einschwingen. Das Panel zeigt dabei alles,
was es im echten Flug auch zeigt: Speed, Mach, Hoehe, QNH, Lage, LS.

Aufruf (auf dem Notebook, das Panel ist im WLAN):

    python approach_sim.py 192.168.10.15          # Standardzeitraffer
    python approach_sim.py 192.168.10.15 1.0      # Echtzeit (langsam)
    python approach_sim.py 192.168.10.15 8.0      # achtfach schnell

Wichtig: Das Panel verwirft Bursts. Ein Frame alle 1,2 s ist sicher,
alles unter 0,2 s geht unter. Der Zeitraffer vergroessert deshalb nur
den simulierten Zeitschritt, nicht die Senderate.
"""

import math
import socket
import sys
import time

# --------------------------------------------------------------------------
# Verbindung
# --------------------------------------------------------------------------
ZIEL_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.15"
ZEITRAFFER = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
PORT_DATEN = 5005
PORT_IDENT = 5006
TAKT = 1.2                 # Sekunden pro gesendetem Frame (Burst-Grenze!)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
seq = 0


def sende(text):
    sock.sendto(text.encode("ascii", "replace"), (ZIEL_IP, PORT_DATEN))


def frame(spd, alt, qnh, pitch, bank, ball, ls, loc, loc_v, gs, gs_v,
          std=0, sim=1, lang="de"):
    """Baut einen vollstaendigen Frame mit allen 13 Kanaelen."""
    global seq
    seq += 1
    mach = spd / 660.0 * (0.85 ** 1)  # grobe Naeherung, reicht optisch
    return ("SEQ%d|DSP1:%d;DSP2:%.3f;DSP3:%d;DSP4:%d;DSP5:%d;"
            "DSP6:%.1f;DSP7:%.1f;DSP8:%.2f;"
            "DSP9:%d;DSP10:%.2f;DSP11:%d;DSP12:%.2f;DSP13:%d;"
            "SIM:%d;LANG:%s;"
            % (seq, int(spd), mach, int(alt), int(qnh), std,
               pitch, bank, ball,
               ls, loc, loc_v, gs, gs_v, sim, lang))


def klemm(wert, unten, oben):
    return max(unten, min(oben, wert))


# --------------------------------------------------------------------------
# Flugprofil
# --------------------------------------------------------------------------
BOGEN = 0.7            # Grad Gleitpfad-Vollausschlag
LOC_VOLL = 2.5         # Grad Localizer-Vollausschlag
PFAD_GRAD = 3.0        # 3-Grad-Gleitpfad
QNH = 1016

distanz = 12.0         # NM bis zum Aufsetzpunkt
hoehe = 10000.0        # ft (Startpunkt "von FL100 kommend")
speed = 250.0          # kt
zeit = 0.0
ls_an = False
meldung_gedruckt = False

# Hoehe auf dem 3-Grad-Pfad ueber dem Aufsetzpunkt
def pfad_hoehe(d):
    return max(0.0, (d - 0.3) * 6076.0 * math.tan(math.radians(PFAD_GRAD)))

PFAD_BEI_START = pfad_hoehe(12.0)
START_HOEHE = 10000.0

print("=== ILS-Anflug  12 NM / 10.000 ft  ->  Aufsetzpunkt")
print(f"    Zeitraffer {ZEITRAFFER:g}x, ein Frame alle {TAKT:g} s\n")

while distanz > 0.0:
    dt = TAKT * ZEITRAFFER
    zeit += dt

    # --- Geschwindigkeit: 250 kt bis zum Endanflug, dann Approach-Speed
    if distanz > 8.0:
        speed_ziel = 250.0
    elif distanz > 4.5:
        speed_ziel = 175.0
    else:
        speed_ziel = 140.0
    speed += klemm(speed_ziel - speed, -4.0, 4.0)

    # --- Distanz abbauen (kt -> NM/s)
    distanz = max(0.0, distanz - speed / 3600.0 * dt)

    # --- Hoehenprofil, bewusst vorgegeben statt integriert:
    #     Bis 6 NM ein starker Sinkflug (aus FL100 ist man zwangslaeufig viel zu
    #     hoch, der Gleitpfad-Diamant steht oben), ab 6 NM laeuft die Maschine
    #     mit 220 ft Ueberhoehe auf den Pfad ein. Genau dieser Einfang ist die
    #     Bewegung, die man am Display sehen soll.
    pfad = pfad_hoehe(distanz)
    if distanz >= 6.0:
        extra = 220.0 + (START_HOEHE - pfad_hoehe(12.0) - 220.0) * (distanz - 6.0) / 6.0
    else:
        # Einfang von oben, der Rest laeuft gegen Null - und unter 0,7 NM ist
        # der Flieger auf dem Pfad (sonst wuerde der Winkel zum Aufsetzpunkt
        # wieder aufgehen und der Diamant unnoetig zappeln).
        fade = klemm((distanz - 0.7) / 1.0, 0.0, 1.0)
        extra = 220.0 * math.exp(-(6.0 - distanz) / 2.0) * fade
    hoehe = pfad + extra

    # --- Localizer: bis 8 NM stehen 0,9 (Intercept), danach schnell auffangen
    if distanz > 8.0:
        loc_abweichung = 0.90          # noch im Intercept, voller Versatz
    else:
        loc_abweichung = 0.90 * math.exp(-(8.0 - distanz) / 2.0)

    # --- Gleitpfad: echte Hoehendifferenz zum Pfad, in Grad umgerechnet
    if distanz > 0.4:
        hoehendifferenz = hoehe - pfad
        winkel = math.degrees(math.atan2(hoehendifferenz, distanz * 6076.0))
        gs_abweichung = winkel
    else:
        gs_abweichung = 0.0

    # --- Lage: bei dem steilen Sinkflug Nase deutlich runter, im Endanflug ruhig
    if distanz > 8.0:
        pitch = -3.5 + 0.6 * math.sin(zeit / 7.0)
    elif distanz > 1.0:
        pitch = 2.5 + 0.7 * math.sin(zeit / 9.0)
    else:
        pitch = 3.5 + 0.9 * math.sin(zeit / 6.0)
    if distanz > 7.0:
        bank = 8.0 * math.sin(zeit / 14.0)      # Kurve auf den Localizer
    elif distanz > 0.5:
        bank = 0.7 * math.sin(zeit / 5.0)
    else:
        bank = 0.0
    ball = 0.05 * math.sin(zeit / 3.7)

    # --- LS einschalten, sobald das ILS-Signal da ist (ab 9 NM)
    if distanz < 9.0:
        ls_an = True

    # --- Normierung auf -1..1 fuer das Panel
    loc_norm = klemm(loc_abweichung / LOC_VOLL, -1.0, 1.0)
    gs_norm = klemm(gs_abweichung / BOGEN, -1.0, 1.0)

    sende(frame(speed, hoehe, QNH, pitch, bank, ball,
                int(ls_an), loc_norm, int(ls_an), gs_norm, int(ls_an)))

    # --- Statuszeile (jeder zweite Frame, sonst wird die Konsole zu laut)
    if seq % 2 == 0:
        print("  %5.2f NM  %6.0f ft  %3.0f kt  LOC %+.2f  GS %+.2f  LS %d"
              % (distanz, hoehe, speed, loc_norm, gs_norm, ls_an))

    if not meldung_gedruckt and hoehe <= 100.0:
        print("  --- 100 ft ueber der Bahn")
        meldung_gedruckt = True

    time.sleep(TAKT)

# --------------------------------------------------------------------------
# Aufgesetzt: LS aus, Werte einfrieren (Panel zeigt dann den Endzustand)
# --------------------------------------------------------------------------
print("\n=== Aufgesetzt - LS aus, Anzeige bleibt stehen")
for _ in range(3):
    sende(frame(0, 0, QNH, 0.0, 0.0, 0.0, 0, 0.0, 0, 0.0, 0))
    time.sleep(TAKT)

print("fertig.")
