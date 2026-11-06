#pragma once

#include "Arduino.h"
#include "ISISCommon.h"

#define PRESS_COLOR TFT_BLUE

class CC_ISIS : public CC_ISIS_Base
{
public:
    CC_ISIS();
    void begin();
    void attach();
    void detach();
    void set(int16_t messageID, char *setPoint);

    void update();

    // Diagnose-Kennzeichnung (Panel-Monitor / Live-Diag) rechts neben dem
    // Baro-Wert. Bewusst als Sprite: auf lcd ist KEIN Font geladen
    // (lcd.loadFont(A320ISIS24) steht auskommentiert), ein lcd.drawString mit
    // setTextSize hat das Panel beim FDC-Start zum Absturz gebracht.
    void drawDiagBadge(bool on);

private:
    bool _initialized;

    void setupSprites();
    void updateInputValues();
    void drawBackground();
    void drawPressure();
    void drawSpeedTape();
    void drawAltTape();
    void drawAttitude();
    void drawMach();
    void drawLS();

    void draw();



};

// Landing-System: Lage der Skalen zur Laufzeit verschieben.
// Aufruf ueber die USB-Konsole: LSPOS <gsX> <locTop> [<vollerAusschlagPx>]
// Damit laesst sich die Position am echten Geraet einjustieren, ohne neu zu flashen.
void isisSetLsLayout(int gsX, int locTop, int devFull, int locX);
