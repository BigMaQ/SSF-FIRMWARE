// Host-Shim fuer "<Preferences.h>" (ESP32-NVS) — nur Desktop-Renderer.
//
// Der Upstream speichert in `CC_ISIS_Base::saveState()/restoreState()` ein paar
// Zustandsfelder in NVS. Fuer die Grafik ist das irrelevant, aber es muss
// uebersetzen UND darf keinen Zufall ins Bild bringen. Deshalb: einfacher
// In-Memory-Speicher pro Prozesslauf (nichts wird auf Platte geschrieben, jeder
// Renderlauf startet gleich).
#pragma once

#include <cstdint>
#include <cstring>
#include <map>
#include <string>

class Preferences
{
public:
    bool begin(const char *name, bool readOnly = false)
    {
        _ns = name ? name : "";
        _ro = readOnly;
        return true;
    }
    void end() { _ns.clear(); }
    bool clear() { _t.clear(); return true; }
    bool remove(const char *key) { _t.erase(_ns + "/" + key); return true; }
    bool isKey(const char *key) const { return _t.find(_ns + "/" + key) != _t.end(); }

    size_t putBool(const char *k, bool v) { return store(k, v ? (int64_t)1 : (int64_t)0); }
    size_t putInt(const char *k, int32_t v) { return store(k, (int64_t)v); }
    size_t putUInt(const char *k, uint32_t v) { return store(k, (int64_t)v); }
    size_t putShort(const char *k, int16_t v) { return store(k, (int64_t)v); }
    size_t putLong(const char *k, int32_t v) { return store(k, (int64_t)v); }
    size_t putFloat(const char *k, float v)
    {
        store(k + 0, (int64_t)bytes_of<float>(v));
        return sizeof(float);
    }
    size_t putBytes(const char *k, const void *buf, size_t len)
    {
        if (!buf || !k || _ro) return 0;
        std::string blob((size_t)len, '\0');
        memcpy(&blob[0], buf, len);
        _blobs[_ns + "/" + k] = blob;
        return len;
    }
    size_t putString(const char *k, const String &v);

    bool getBool(const char *k, bool def = false) const { int64_t v; return load(k, v) ? v != 0 : def; }
    int32_t getInt(const char *k, int32_t def = 0) const { int64_t v; return load(k, v) ? (int32_t)v : def; }
    uint32_t getUInt(const char *k, uint32_t def = 0) const { int64_t v; return load(k, v) ? (uint32_t)v : def; }
    int16_t getShort(const char *k, int16_t def = 0) const { int64_t v; return load(k, v) ? (int16_t)v : def; }
    int32_t getLong(const char *k, int32_t def = 0) const { int64_t v; return load(k, v) ? (int32_t)v : def; }
    float getFloat(const char *k, float def = 0.0f) const
    {
        int64_t raw;
        if (!load(k, raw)) return def;
        uint32_t bits = (uint32_t)raw;
        float out;
        memcpy(&out, &bits, sizeof(out));
        return out;
    }
    size_t getBytes(const char *k, void *buf, size_t maxLen) const
    {
        auto it = _blobs.find(_ns + "/" + k);
        if (it == _blobs.end() || !buf) return 0;
        size_t n = it->second.size() < maxLen ? it->second.size() : maxLen;
        memcpy(buf, it->second.data(), n);
        return n;
    }
    size_t getBytesLength(const char *k) const
    {
        auto it = _blobs.find(_ns + "/" + k);
        return it == _blobs.end() ? 0 : it->second.size();
    }
    size_t getString(const char *k, char *buf, size_t maxLen) const
    {
        auto it = _blobs.find(_ns + "/" + k);
        if (it == _blobs.end() || !buf || maxLen == 0) return 0;
        size_t n = it->second.size() < maxLen - 1 ? it->second.size() : maxLen - 1;
        memcpy(buf, it->second.data(), n);
        buf[n] = '\0';
        return n;
    }

private:
    template <typename T>
    static uint32_t bytes_of(const T &v)
    {
        uint32_t bits = 0;
        memcpy(&bits, &v, sizeof(T) < sizeof(bits) ? sizeof(T) : sizeof(bits));
        return bits;
    }

    size_t store(const char *k, int64_t v)
    {
        if (!k || _ro) return 0;
        _t[_ns + "/" + k] = v;
        return sizeof(v);
    }
    bool load(const char *k, int64_t &out) const
    {
        auto it = _t.find(_ns + "/" + k);
        if (it == _t.end()) return false;
        out = it->second;
        return true;
    }

    std::string _ns;
    bool _ro = false;
    std::map<std::string, int64_t> _t;
    std::map<std::string, std::string> _blobs;
};

inline size_t Preferences::putString(const char *k, const String &v)
{
    return putBytes(k, v.c_str(), v.length() + 1);
}
