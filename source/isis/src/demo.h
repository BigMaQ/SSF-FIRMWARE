#pragma once
//
// demo.h — Werkstatt-Kommandos fuer das ISIS-Panel.
//
// Alle drei laufen ueber UDP (Panel-Monitor im FDC) und ueber die
// USB-Konsole:
//
//   BLANK  — alle angezeigten Werte auf Startwerte zuruecksetzen
//   RESET  — Panel neu booten
//   SHOW   — 30 s Demo-Flug: Start, Steigflug auf 5000 ft, Linkskurve,
//            Rueckflug und Landung mit Rollout
//
#include "Arduino.h"

// Alle Werte auf Startwerte (Airspeed 0, Altitude 0, Lage waagerecht,
// QNH 1013, LS aus). Beendet einen laufenden SHOW-Durchlauf.
void demoBlank();

// Panel neu starten. Kehrt nicht zurueck.
void demoReboot();

// Demo-Flug starten (30 s). Laeuft einmal ab und endet danach von selbst.
void demoStartShow(uint32_t nowMs);

// Laeuft gerade ein Demo-Flug?
bool demoShowActive(uint32_t nowMs);

// Pro Frame aufrufen: ueberschreibt waehrend des Demo-Flugs die Werte, die
// vom FDC kommen. Vor isis.update() aufrufen.
void demoTick(uint32_t nowMs);
