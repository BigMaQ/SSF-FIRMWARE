#pragma once
//
// grid.h — Kalibrier-Testmuster fuer das ISIS-Panel.
//
// Zweck: Rahmen/Sichtfenster vermessen. Das Muster zeichnet ein farbcodiertes
// Raster ueber die volle 480x480-Flaeche, unabhaengig von scale_w/h. Damit
// laesst sich am eingebauten Panel ablesen, welche Bildbereiche der gelaserte
// Rahmen verdeckt — statt die aktive Displayflaeche zu raten.
//
//   weiss  = Displayrand (0 und 479 px)
//   rot    = Mittelpunkt (240/240) und Mittelkreuz
//   gruen  = Viertel (120/360)
//   gelb   = Achtel-Linien, blau = die uebrigen 60-px-Schritte
//
// Kommando: GRID (schaltet ein/aus), erreichbar ueber UDP wie BLANK/SHOW.
//
#include "Arduino.h"

// Muster ein-/ausschalten. Rueckgabe: neuer Zustand.
bool gridToggle();

// Aktueller Zustand.
bool gridActive();

// Pro Frame vor displaycfgComposite() aufrufen: zeichnet das Muster ins
// Kompositions-Canvas, solange es aktiv ist.
void gridDraw();

// true, wenn das Muster neu gezeichnet werden muss (dann Flag loeschen).
bool gridTakeRedraw();
