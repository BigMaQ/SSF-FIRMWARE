// Host-Shim fuer "<Arduino.h>" — NUR fuer den Desktop-Renderer (tools/isis_sim).
//
// Zweck: CC_ISIS.cpp / ISISCommon.h unveraendert auf dem PC uebersetzen.
// Der Upstream-Code ruft Arduino-Grundlagen (String, millis, min/max, PROGMEM,
// Serial, ESP_LOG*). Kein Byte davon beeinflusst die Zeichenlogik — es geht nur
// darum, dass es uebersetzt.
//
// WICHTIG: ARDUINO darf NICHT definiert werden. LovyanGFX waehlt die Plattform
// ueber `#elif defined (ARDUINO)`; wir brauchen den Pfad
// `__linux__ && LGFX_LINUX_FB` (--> platforms/framebuffer/common.hpp). Damit
// fehlen auch die `#if defined(ARDUINO)`-Overloads in LGFXBase.hpp, die
// Arduino-`String` erwarten — dafuer liefert dieser Shim einen `String` mit
// implizitem `operator const char*()`.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <chrono>
#include <string>
#include <thread>
#include <algorithm>

// --- Arduino-Grundtypen / Makros -------------------------------------------
#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef PGM_P
#define PGM_P const char *
#endif
#define F(str) (str)
#define pgm_read_byte(addr)  (*(const uint8_t  *)(const void *)(addr))
#define pgm_read_word(addr)  (*(const uint16_t *)(const void *)(addr))
#define pgm_read_dword(addr) (*(const uint32_t *)(const void *)(addr))

#define LOW  0
#define HIGH 1
#define INPUT        0
#define OUTPUT       1
#define INPUT_PULLUP 2

#define ARDUINO_ARCH_ESP32 1
#define ESP32 1
#define ESP_LOGV(tag, ...) do {} while (0)
#define ESP_LOGD(tag, ...) do {} while (0)
#define ESP_LOGI(tag, ...) do {} while (0)
#define ESP_LOGW(tag, ...) do {} while (0)
#define ESP_LOGE(tag, ...) do {} while (0)

// min/max/abs wie in Arduino (global, float-faehig)
using std::abs;
using std::max;
using std::min;

// --- Zeitgeber (Host) ------------------------------------------------------
inline unsigned long millis(void)
{
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return (unsigned long)duration_cast<milliseconds>(steady_clock::now() - t0).count();
}
inline unsigned long micros(void)
{
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return (unsigned long)duration_cast<microseconds>(steady_clock::now() - t0).count();
}
inline void delay(unsigned long ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
inline void delayMicroseconds(unsigned int us)
{
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

// --- String ----------------------------------------------------------------
// Schlankes Gegenstueck zu Arduino-`String`. Braucht genau das, was der
// Upstream benutzt (Konstruktion aus int, Konkatenation mit Literal, length(),
// c_str()) und — fuer LGFX-Aufrufe ohne ARDUINO — eine implizite Umwandlung
// nach `const char*`.
class String
{
public:
    String() {}
    String(const char *s) : _s(s ? s : "") {}
    String(char c) : _s(1, c) {}
    String(int v) : _s(std::to_string(v)) {}
    String(unsigned int v) : _s(std::to_string(v)) {}
    String(long v) : _s(std::to_string(v)) {}
    String(unsigned long v) : _s(std::to_string(v)) {}
    String(float v) : _s(std::to_string(v)) {}
    String(double v) : _s(std::to_string(v)) {}

    operator const char *() const { return _s.c_str(); }
    const char *c_str() const { return _s.c_str(); }
    size_t      length() const { return _s.size(); }
    bool        isEmpty() const { return _s.empty(); }
    char        operator[](size_t i) const { return _s[i]; }

    String &operator+=(const String &o) { _s += o._s; return *this; }
    String &operator+=(const char *o) { if (o) _s += o; return *this; }
    String &operator+=(char o) { _s += o; return *this; }
    String &operator+=(int v) { _s += std::to_string(v); return *this; }

    String operator+(const String &o) const { String r(*this); r += o; return r; }
    String operator+(const char *o) const { String r(*this); r += o; return r; }
    String operator+(char o) const { String r(*this); r += o; return r; }

    bool operator==(const String &o) const { return _s == o._s; }
    bool operator!=(const String &o) const { return _s != o._s; }

    const std::string &std_str() const { return _s; }

private:
    std::string _s;
};

inline String operator+(const char *a, const String &b) { String r(a); r += b; return r; }

// --- Serial ----------------------------------------------------------------
// Nur Platzhalter: auf dem Desktop gibt es keine serielle MobiFlight-Bruecke.
// Ausgaben laufen ueber stdout des Renderers, nicht hierher.
struct HostSerialStub
{
    void begin(unsigned long = 115200) {}
    void end() {}
    operator bool() const { return false; }
    template <typename... A> void print(A &&...) {}
    template <typename... A> void println(A &&...) {}
    template <typename... A> void printf(const char *, A &&...) {}
    void flush() {}
};
inline HostSerialStub Serial;
