// Desktop-Ersatz fuer "4inchLCDConfig_Guition.h" — nur Desktop-Renderer.
//
// Diese Datei wird im Build-Verzeichnis (build/src) UNTER DEM NAMEN
// 4inchLCDConfig_Guition.h eingebunden. Der Upstream bleibt unangetastet:
// ISISCommon.h macht `#ifdef USE_GUITION_SCREEN` -> `#include "4inchLCDConfig_Guition.h"`
// und bekommt hier statt Bus_RGB/Panel_ST7701 einen reinen Speicher-Framebuffer.
//
// Nachgebildet werden nur die fuer die Grafik relevanten Panel-Parameter:
//   memory/panel 480x480, offset 0/0, Rotation uebernimmt der Aufrufer
//   (CC_ISIS::begin() -> lcd.setRotation(3)).
// Pins, RGB-Timings, Touch und Hintergrundlicht entfallen — sie haben auf die
// Pixel keinen Einfluss.
#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "Panel_Memory.hpp"
#include <lgfx/v1/Bus.hpp>

class LGFX : public lgfx::LGFX_Device
{
public:
    lgfx::v1::Bus_NULL     _bus_instance;   // Panel_FrameBufferBase tippt den Bus nie an
    isis_sim::Panel_Memory _panel_instance;

    LGFX(void)
    {
        {
            auto cfg = _panel_instance.config();

            cfg.memory_width  = 480;
            cfg.memory_height = 480;
            cfg.panel_width   = 480;
            cfg.panel_height  = 480;

            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.pin_cs   = -1;   // keine Pins auf dem Host
            cfg.pin_rst  = -1;

            _panel_instance.config(cfg);
        }

        _panel_instance.setBus(&_bus_instance);
        setPanel(&_panel_instance);
    }
};
