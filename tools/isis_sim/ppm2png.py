#!/usr/bin/env python3
"""ppm2png.py — kleiner Helfer: PPM (P6) -> PNG (RGB) und Drehen.

Nur fuer die Sichtpruefung / Zwischenkontrolle. Fuer die eigentliche Ausgabe
ist die Drehung im Renderer (--rotate) implementiert.
"""
import sys
import zlib
import struct


def read_ppm(path):
    data = open(path, "rb").read()
    if not data.startswith(b"P6"):
        raise ValueError("kein P6-PPM: " + path)
    parts = []
    i = 2
    while len(parts) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        parts.append(int(data[i:j]))
        i = j
    i += 1  # genau ein Whitespace nach der Maxval
    w, h, _ = parts
    return w, h, data[i:i + w * h * 3]


def write_png(path, w, h, rgb):
    def chunk(tag, payload):
        return (struct.pack(">I", len(payload)) + tag + payload +
                struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))
    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)
        raw += rgb[y * stride:(y + 1) * stride]
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    png += chunk(b"IEND", b"")
    open(path, "wb").write(png)


def rotate(rgb, w, h, degrees):
    """Dreht das Bild im/gegen den Uhrzeigersinn (Vielfache von 90)."""
    degrees %= 360
    if degrees == 0:
        return w, h, rgb
    if degrees == 180:
        stride = w * 3
        out = bytearray(len(rgb))
        for y in range(h):
            src = rgb[(h - 1 - y) * stride:(h - y) * stride]
            out[y * stride:(y + 1) * stride] = b"".join(
                src[x * 3:x * 3 + 3] for x in range(w - 1, -1, -1))
        return w, h, bytes(out)
    # 90/270: Spalten lesen
    stride = w * 3
    cols = []
    if degrees == 90:   # im Uhrzeigersinn
        for x in range(w):
            r = rgb[x * 3::stride]
            g = rgb[x * 3 + 1::stride]
            b = rgb[x * 3 + 2::stride]
            cols.append(bytes(b"".join(bytes(t) for t in zip(r, g, b))[::-1]))
    else:               # 270
        for x in range(w - 1, -1, -1):
            r = rgb[x * 3::stride]
            g = rgb[x * 3 + 1::stride]
            b = rgb[x * 3 + 2::stride]
            cols.append(bytes(b"".join(bytes(t) for t in zip(r, g, b))))
    return h, w, b"".join(cols)


if __name__ == "__main__":
    src = sys.argv[1]
    w, h, rgb = read_ppm(src)
    for deg in (0, 90, 180, 270):
        dw, dh, drgb = rotate(rgb, w, h, deg)
        out = src.rsplit(".", 1)[0] + ".rot%d.png" % deg
        write_png(out, dw, dh, drgb)
        print(out, dw, "x", dh)
