#pragma once
//
// align.h — Touch-Ausrichtwerkzeug fuer das ISIS-Panel.
//
// Zweck: Rahmen/Sichtfenster am eingebauten Panel einstellen, ohne fuer jede
// Aenderung neu flashen oder CFG-Frames von Hand tippen zu muessen.
//
// Kommando ALIGN (UDP) startet das Werkzeug. Es liegt UEBER dem Bild und wird
// bewusst direkt auf lcdPanel gezeichnet (nicht ins 480x480-Canvas): sonst
// wuerde die Bedienoberflaeche selbst mitskaliert und die Touch-Koordinaten
// passten nicht mehr zu den Schaltflaechen.
//
// Bedienung (Touch):
//   [ - ] / [ + ]      Bild kleiner / groesser
//   Pfeiltasten        verschieben
//   [ASPECT]           Seitenverhaeltnis halten an/aus
//   [SAVE]             Werte in die NVS schreiben (ueberleben Reboot)
//   [X]                Werkzeug schliessen
//
#include "Arduino.h"
#include "displaycfg.h"

// Config der Firmware bekannt machen (einmal nach dem Start aufrufen).
bool alignBegin(IsisCfg *cfg);

// Werkzeug ein-/ausschalten. Rueckgabe: neuer Zustand.
bool alignToggle();

// Laeuft das Werkzeug gerade?
bool alignActive();

// Pro Frame nach displaycfgComposite() aufrufen: verarbeitet Touch-Eingaben
// und zeichnet die Bedienleiste.
void alignTick();

// Geaenderte Werte (werden von der Firmware in IsisCfg uebernommen).
int  alignScale();
int  alignOffX();
int  alignOffY();
bool alignKeepAspect();

// Zuletzt [SAVE] gedrueckt? Wird beim Abholen zurueckgesetzt.
bool alignTakeSaveRequest();

// true, wenn das Bild neu komponiert werden muss.
bool alignTakeRedraw();

// Leiste neu zeichnen lassen. Muss nach jedem displaycfgComposite() gerufen
// werden: der Composite leert die Raender ausserhalb des Zielrechtecks und
// loescht damit Teile der Bedienleiste.
void alignForceRedraw();
