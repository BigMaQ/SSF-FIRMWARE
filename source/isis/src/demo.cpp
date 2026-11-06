//
// demo.cpp — BLANK / RESET / SHOW fuer das ISIS-Panel.
//
// SHOW ist eine reine Werte-Choreografie: waehrend der 30 s ueberschreibt
// demoTick() die von aussen kommenden Werte. Der FDC darf also weiter
// senden — die Demo hat Vorrang, bis sie ausgelaufen ist.
//
#include "demo.h"

#include <math.h>

#include "ISISCommon.h"

namespace {

// Gesamtdauer eines Durchlaufs.
const uint32_t SHOW_DURATION_MS = 30000;

// Phasengrenzen (ms ab Start).
const uint32_t T_ROLL_END   = 3000;   //  0..3000   am Boden beschleunigen
const uint32_t T_ROTATE_END = 4500;   //  3000..4500 Rotieren und abheben
const uint32_t T_CLIMB_END  = 12000;  //  4500..12000 Steigflug auf 5000 ft
const uint32_t T_TURN_END   = 19000;  // 12000..19000 Linkskurve (Turn Around)
const uint32_t T_LEVEL_END  = 21000;  // 19000..21000 ausleiten
const uint32_t T_DESC_END   = 26000;  // 21000..26000 Sinkflug zur Bahn
                                      // 26000..30000 Aufsetzen und Rollout

bool     g_active  = false;
uint32_t g_startMs = 0;

float lerp(float a, float b, float t) { return a + (b - a) * t; }

// Fortschritt 0..1 innerhalb eines Abschnitts.
float frac(uint32_t t, uint32_t von, uint32_t bis)
{
    if (t <= von) return 0.0f;
    if (t >= bis) return 1.0f;
    return (float)(t - von) / (float)(bis - von);
}

// Weicher Ein-/Auslauf, damit die Uebergaenge nicht rucken.
float ease(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

}  // namespace

void demoBlank()
{
    g_active = false;

    isisState.rawAirspeed      = 0.0f;
    isisState.airspeed         = 0.0f;
    isisState.trueAirspeed     = 0.0f;
    isisState.machSpeed        = 0.0f;
    isisState.rawAltitude      = 0.0f;
    isisState.altitude         = 0.0f;
    isisState.rawVerticalSpeed = 0;
    isisState.verticalSpeed    = 0;
    isisState.rawPitchAngle    = 0.0f;
    isisState.pitchAngle       = 0.0f;
    isisState.rawBankAngle     = 0.0f;
    isisState.bankAngle        = 0.0f;
    isisState.rawBallPos       = 0.0f;
    isisState.ballPos          = 0.0f;
    isisState.mbPressure       = 1013;
    isisState.isStdPressure    = 0;
    isisState.lsActive         = 0;
    isisState.lsLocDev         = 0.0f;
    isisState.lsLocValid       = 0;
    isisState.lsGsDev          = 0.0f;
    isisState.lsGsValid        = 0;
    isisState.forceRedraw      = true;

    Serial.println("[demo] BLANK - Werte auf Startwerte");
}

void demoReboot()
{
    Serial.println("[demo] RESET - Panel bootet neu");
    Serial.flush();
    delay(150);          // Antwort/Logzeile noch rausschreiben
    ESP.restart();
}

void demoStartShow(uint32_t nowMs)
{
    g_active  = true;
    g_startMs = nowMs;
    isisState.forceRedraw = true;
    Serial.println("[demo] SHOW - 30 s Demo-Flug gestartet");
}

bool demoShowActive(uint32_t nowMs)
{
    if (!g_active) return false;
    if ((uint32_t)(nowMs - g_startMs) >= SHOW_DURATION_MS) {
        g_active = false;
        Serial.println("[demo] SHOW beendet");
        return false;
    }
    return true;
}

void demoTick(uint32_t nowMs)
{
    if (!demoShowActive(nowMs)) return;

    const uint32_t t = (uint32_t)(nowMs - g_startMs);

    float speed = 0.0f;
    float alt   = 0.0f;
    float pitch = 0.0f;
    float bank  = 0.0f;
    int   vs    = 0;

    if (t < T_ROLL_END) {
        // Anrollen: 0 -> 140 kt am Boden
        speed = lerp(0.0f, 140.0f, ease(frac(t, 0, T_ROLL_END)));
        pitch = 0.0f;
        alt   = 0.0f;
        vs    = 0;
    } else if (t < T_ROTATE_END) {
        // Rotieren und abheben
        const float k = ease(frac(t, T_ROLL_END, T_ROTATE_END));
        speed = 140.0f;
        pitch = lerp(0.0f, 12.0f, k);
        alt   = lerp(0.0f, 120.0f, k);
        vs    = 1500;
    } else if (t < T_CLIMB_END) {
        // Steigflug bis 5000 ft
        const float k = ease(frac(t, T_ROTATE_END, T_CLIMB_END));
        speed = 140.0f;
        pitch = lerp(12.0f, 6.0f, k);
        alt   = lerp(120.0f, 5000.0f, k);
        vs    = 1800;
    } else if (t < T_TURN_END) {
        // Turn Around: Linkskurve bei 5000 ft
        const float k = ease(frac(t, T_CLIMB_END, T_TURN_END));
        speed = 140.0f;
        pitch = 6.0f;
        alt   = 5000.0f;
        bank  = lerp(0.0f, -25.0f, k);   // links = negativer Bank
        vs    = 0;
    } else if (t < T_LEVEL_END) {
        // Ausleiten
        const float k = ease(frac(t, T_TURN_END, T_LEVEL_END));
        speed = 140.0f;
        pitch = 4.0f;
        alt   = 5000.0f;
        bank  = lerp(-25.0f, 0.0f, k);
        vs    = 0;
    } else if (t < T_DESC_END) {
        // Rueckflug als Sinkflug zur Bahn
        const float k = ease(frac(t, T_LEVEL_END, T_DESC_END));
        speed = 140.0f;
        pitch = lerp(4.0f, -3.0f, k);
        alt   = lerp(5000.0f, 0.0f, k);
        vs    = -1200;
    } else {
        // Aufsetzen und ausrollen
        const float k = ease(frac(t, T_DESC_END, SHOW_DURATION_MS));
        speed = lerp(140.0f, 0.0f, k);
        alt   = 0.0f;
        pitch = lerp(-3.0f, 0.0f, k);
        vs    = 0;
    }

    isisState.rawAirspeed      = speed;
    isisState.trueAirspeed     = speed;
    isisState.rawAltitude      = alt;
    isisState.rawPitchAngle    = pitch;
    isisState.rawBankAngle     = bank;
    isisState.rawVerticalSpeed = vs;
    // Grobe Naeherung auf Meereshoehe (~0.0021 Mach pro Knoten).
    isisState.machSpeed        = speed * 0.0021f;

    // Ab dem Sinkflug ein ILS mitlaufen lassen: die Diamanten wandern zur
    // Mitte, damit die LS-Anzeige im Show-Fall auch etwas zu tun hat.
    if (t >= T_LEVEL_END) {
        const float approach = frac(t, T_LEVEL_END, T_DESC_END);
        isisState.lsActive   = 1;
        isisState.lsLocValid = 1;
        isisState.lsGsValid  = 1;
        isisState.lsLocDev   = lerp(-0.40f, 0.0f, approach);
        isisState.lsGsDev    = lerp(0.30f, 0.0f, approach);
    } else {
        isisState.lsActive   = 0;
        isisState.lsLocValid = 0;
        isisState.lsGsValid  = 0;
        isisState.lsLocDev   = 0.0f;
        isisState.lsGsDev    = 0.0f;
    }
}
