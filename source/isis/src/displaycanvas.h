#pragma once
//
// displaycanvas.h — the drawing surface CC_ISIS renders into.
//
// Why this indirection exists (ISIS_FIRMWARE_SPEC.md section 7):
// the whole scene is drawn in a 480x480 coordinate system by the existing,
// unchanged CC_ISIS sprites (attSprite, speedSprite, altSprite, kohlsSprite).
// To be able to shrink the whole output at runtime without re-flashing -
// CFG:scale_w/scale_h/off_x/off_y - the scene is composed into an off-screen
// 480x480 sprite which displaycfgComposite() then blits to the panel: byte
// exact when no scaling is configured, otherwise through LovyanGFX'
// pushImageRotateZoom path.
//
// `lcd` keeps its name from the upstream firmware so CC_ISIS.cpp stays byte
// identical; only its type changes (composition canvas instead of the panel).
//
#include "4inchLCDConfig_Guition.h"
#include <LovyanGFX.hpp>

// The real Guition panel: RGB bus, ST7701, backlight (Light_PWM) on GPIO 38.
extern LGFX lcdPanel;

// 480x480 off-screen composition canvas, 8 bit palette, allocated in PSRAM.
class IsisCanvas : public LGFX_Sprite {
public:
    explicit IsisCanvas(LovyanGFX *parent) : LGFX_Sprite(parent) {}

    // Keep the color_depth_t overload of the base class reachable.
    using LGFX_Sprite::setColorDepth;

    // The upstream code calls setColorDepth() in begin(), i.e. after the
    // sprite already exists.  Re-setting the depth at that point would drop
    // the palette flag, so it is only honoured while the buffer is being
    // created.
    void setColorDepth(uint8_t bpp)
    {
        if (bufferLength() == 0) LGFX_Sprite::setColorDepth(bpp);
    }

    // The panel owns the hardware - nothing to initialise on the canvas.
    void init() {}

    // Brightness always belongs to the physical panel.
    void setBrightness(uint8_t brightness) { lcdPanel.setBrightness(brightness); }
    uint8_t getBrightness() const { return lcdPanel.getBrightness(); }
};

// The object the upstream drawing code calls "lcd".
extern IsisCanvas lcd;
