// Host-Shim fuer "MFEEPROM.h" (MobiFlight) — nur Desktop-Renderer.
//
// Das echte MFEEPROM haelt die Geraetekonfiguration im ESP32-EEPROM/NVS.
// Hier gibt es kein EEPROM. Verhalten des Shims bewusst "jungfräuliches Board":
//   * `read_block()` liefert immer false  → `loadSettings()` faellt auf die
//     Default-Werte aus CC_ISIS_Settings() zurueck (lcdBrightness = 100).
//   * `write_block()` liefert false, `commit()` ist ein No-Op → nichts wird
//     gespeichert, jeder Renderlauf ist reproduzierbar.
// Fuer die Grafik ist nur `lcdBrightness` betroffen, und das ist im Offscreen-Render
// ohne Hintergrundlicht ohnehin wirkungslos.
//
// Interface nach dem Original: github.com/elral/MobiFlight-FirmwareSource,
// Branch `ESP32_support`, src/MF_Modules/MFEEPROM.h (Stand der CC_ISIS-Referenz).
#pragma once

#include <cstdint>

class MFEEPROM
{
private:
    uint16_t _eepromLength = 0; // 0 == kein gespeicherter Stand vorhanden

public:
    MFEEPROM() {}
    void     init(void) {}
    uint16_t get_length(void) { return _eepromLength; }
    uint8_t  read_byte(uint16_t) { return 0; }
    bool     write_byte(uint16_t, const uint8_t) { return false; }
    void     commit() {}

    template <typename T>
    bool read_block(uint16_t adr, T &t)
    {
        (void)adr;
        (void)t;
        return false;
    }

    template <typename T>
    bool read_block(uint16_t adr, T &t, uint16_t len)
    {
        (void)adr;
        (void)t;
        (void)len;
        return false;
    }

    template <typename T>
    const bool write_block(uint16_t adr, const T &t)
    {
        (void)adr;
        (void)t;
        return false;
    }

    template <typename T>
    const bool write_block(uint16_t adr, const T &t, uint16_t len)
    {
        (void)adr;
        (void)t;
        (void)len;
        return false;
    }
};

// Der Upstream referenziert das globale Objekt (ISISCommon.h: `extern MFEEPROM MFeeprom;`).
inline MFEEPROM MFeeprom;
