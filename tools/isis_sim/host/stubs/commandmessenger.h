// Host-Shim fuer "commandmessenger.h" (MobiFlight) — nur Desktop-Renderer.
//
// Im Renderer gibt es keine serielle MobiFlight-Bruecke. Von diesem Header
// braucht der Upstream nur zwei Dinge:
//   1. die Kommando-IDs kEncoderChange / kButtonChange (aus dem Original-Enum)
//   2. das globale `cmdMessenger`-Objekt (wird nur benutzt, wenn Knoepfe/Encoder
//      ausgeloest werden — im Renderer nie).
//
// Enum-Werte 1:1 aus dem echten Core uebernommen:
//   github.com/elral/MobiFlight-FirmwareSource, Branch `ESP32_support`,
//   src/commandmessenger.h (kEncoderChange = 6, kButtonChange = 7).
#pragma once

#include <Arduino.h>

enum {
    kInitModule,             // 0
    kSetModule,              // 1
    kSetPin,                 // 2
    kSetStepper,             // 3
    kSetServo,               // 4
    kStatus,                 // 5, Command to report status
    kEncoderChange,          // 6
    kButtonChange,           // 7
    kStepperChange,          // 8
    kGetInfo,                // 9
    kInfo,                   // 10
    kSetConfig,              // 11
    kGetConfig,              // 12
    kResetConfig,            // 13
    kSaveConfig,             // 14
    kConfigSaved,            // 15
    kActivateConfig,         // 16
    kConfigActivated,        // 17
    kSetPowerSavingMode,     // 18
    kSetName,                // 19
    kGenNewSerial,           // 20
    kResetStepper,           // 21
    kSetZeroStepper,         // 22
    kTrigger,                // 23
    kResetBoard,             // 24
    kSetLcdDisplayI2C,       // 25
    kSetModuleBrightness,    // 26
    kSetShiftRegisterPins,   // 27
    kAnalogChange,           // 28
    kInputShifterChange,     // 29
    kDigInMuxChange,         // 30
    kSetStepperSpeedAccel,   // 31
    kSetCustomDevice,        // 32
    kSetModuleSingleSegment, // 33
    kRetriggerDone,          // 34
    kDebug = 0xFF            // 255
};

struct CmdMessengerStub
{
    void sendCmdStart(int) {}
    void sendCmdEnd() {}
    void sendCmd(int) {}
    void sendCmd(int, const char *) {}
    template <typename T> void sendCmdArg(const T &) {}
};

inline CmdMessengerStub cmdMessenger;
