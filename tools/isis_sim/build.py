#!/usr/bin/env python3
"""build.py — uebersetzt den Desktop-Renderer fuer das A320-ISIS.

Warum kein Makefile: der Projektpfad enthaelt ein Leerzeichen
("/mnt/interchange/Project A320/..."), damit kann make seine Pattern-Regeln
nicht bilden. Dieses Skript ruft g++/gcc direkt auf (Parallelbau, inkrementell)
und ist damit unabhaengig von Leerzeichen im Pfad.

Es wird ausschliesslich der host-taugliche Teil von LovyanGFX gebaut:
  platforms/framebuffer  -> Zeitgeber, heap_alloc, leere spi/i2c-Stubs
                           (kein SDL, kein X11, kein Display noetig)
  Panel_FrameBufferBase  -> dieselbe Basisklasse, auf der auch das echte
                           RGB-Panel (Panel_RGB/Panel_ST7701) aufsetzt.

Aufruf:  python3 build.py [-j N] [-v] [--clean] [--with-fonts]
"""
from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(ROOT, "build")
SRC = os.path.join(BUILD, "src")
HOST = os.path.join(ROOT, "host")
LGFX = os.path.join(ROOT, "third_party", "LovyanGFX")
OBJ = os.path.join(BUILD, "obj")
BIN = os.path.join(BUILD, "isis_render")

WARN = [
    "-Wno-unused-variable", "-Wno-unused-but-set-variable", "-Wno-unused-function",
    "-Wno-misleading-indentation", "-Wno-sign-compare", "-Wno-unused-value",
]
DEFS = ["-DLGFX_LINUX_FB"]
INCS = ["-I" + SRC, "-I" + HOST, "-I" + os.path.join(LGFX, "src"),
        "-I" + os.path.join(LGFX, "src", "lgfx", "utility")]
CXXFLAGS = ["-std=gnu++17", "-O2", "-g0"] + DEFS + INCS + WARN
CFLAGS = ["-std=gnu11", "-O0", "-g0", "-w"] + DEFS + INCS  # .c sind reine Datentabellen


def glob(d, pat):
    import glob as g
    return sorted(g.glob(os.path.join(d, pat)))


def sources():
    cpp = [
        os.path.join(HOST, "render_main.cpp"),
        os.path.join(SRC, "CC_ISIS.cpp"),      # Upstream (Zeichenlogik)
        os.path.join(SRC, "ISISCommon.cpp"),   # Upstream
    ]
    cpp += glob(os.path.join(LGFX, "src", "lgfx", "v1"), "*.cpp")
    cpp += glob(os.path.join(LGFX, "src", "lgfx", "v1", "misc"), "*.cpp")
    cpp += [
        os.path.join(LGFX, "src", "lgfx", "v1", "panel", "Panel_Device.cpp"),
        os.path.join(LGFX, "src", "lgfx", "v1", "panel", "Panel_FrameBufferBase.cpp"),
        os.path.join(LGFX, "src", "lgfx", "v1", "platforms", "framebuffer", "common.cpp"),
    ]
    c = glob(os.path.join(LGFX, "src", "lgfx", "utility"), "*.c")
    # CJK-Fonttabellen: lgfx_fonts.cpp referenziert sie, auch wenn das ISIS nur
    # die mitgelieferten CC_ISIS-Fonts (A320ISIS24, PrimaSansMid32) nutzt.
    # Ohne sie bricht der Link ab (116 undefinierte Symbole). Die Zuarrays
    # werden unveraendert mitgebaut, nicht durch Platzhalter ersetzt.
    c += [os.path.join(LGFX, "src", "lgfx", "Fonts", "IPA", "lgfx_font_japan.c")]
    c += glob(os.path.join(LGFX, "src", "lgfx", "Fonts", "efont"), "lgfx_efont_*.c")
    return cpp, c


def obj_for(path: str) -> str:
    tag = hashlib.sha1(os.path.abspath(path).encode()).hexdigest()[:10]
    return os.path.join(OBJ, "%s_%s.o" % (tag, os.path.basename(path)))


def needs_build(src: str, obj: str) -> bool:
    if not os.path.exists(obj):
        return True
    return os.path.getmtime(src) > os.path.getmtime(obj)


def compile_one(job, verbose: bool):
    src, obj, cmd = job
    os.makedirs(OBJ, exist_ok=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return src, r.returncode, (r.stdout or "") + (r.stderr or "")
    if verbose and (r.stderr.strip()):
        return src, 0, r.stderr
    return src, 0, ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-j", "--jobs", type=int, default=min(8, os.cpu_count() or 4))
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--keep-src", action="store_true",
                    help="build/src nicht neu aufbauen")
    args = ap.parse_args()

    if args.clean:
        subprocess.run(["rm", "-rf", OBJ, BIN])

    if not args.keep_src or not os.path.exists(os.path.join(SRC, "CC_ISIS.cpp")):
        r = subprocess.run(["bash", os.path.join(ROOT, "prepare_build.sh")],
                           capture_output=True, text=True)
        sys.stdout.write(r.stdout)
        sys.stderr.write(r.stderr)
        if r.returncode != 0:
            print("prepare_build.sh fehlgeschlagen", file=sys.stderr)
            return 2

    cpp, c = sources()
    jobs = []
    for s in cpp:
        o = obj_for(s)
        if needs_build(s, o):
            jobs.append((s, o, ["g++"] + CXXFLAGS + ["-c", s, "-o", o]))
    for s in c:
        o = obj_for(s)
        if needs_build(s, o):
            jobs.append((s, o, ["gcc"] + CFLAGS + ["-c", s, "-o", o]))

    objs = [obj_for(s) for s in cpp + c]

    t0 = time.time()
    failed = []
    if jobs:
        with ThreadPoolExecutor(max_workers=args.jobs) as ex:
            for src, rc, out in ex.map(lambda j: compile_one(j, args.verbose), jobs):
                if rc != 0:
                    failed.append((src, out))
                elif out.strip():
                    sys.stderr.write("--- %s\n%s\n" % (src, out))
    if failed:
        for src, out in failed:
            print("=== FEHLER in %s\n%s" % (src, out), file=sys.stderr)
        return 1

    link = ["g++", "-o", BIN] + objs + ["-lpthread"]
    r = subprocess.run(link, capture_output=True, text=True)
    if r.returncode != 0:
        print("=== LINK-FEHLER\n" + (r.stdout or "") + (r.stderr or ""), file=sys.stderr)
        return 1

    print("gebaut: %s  (%d Objekte, %d neu, %.1fs)"
          % (BIN, len(objs), len(jobs), time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
