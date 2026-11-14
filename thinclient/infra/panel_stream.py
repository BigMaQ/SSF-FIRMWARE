# infra/panel_stream.py
"""Bildstrom („Stream") auf IP-Panels — Quelle ist GlassOut, Senke ist das Panel.

Spec: ``docs/STREAM_TRANSPORT_SPEC.md`` (Wire-Format, Ports, Firmware-Seite).

Grundgedanke
------------
Der bestehende Weg liefert **Werte** (``DSP1:280;``), und die Panel-Firmware zeichnet
die Symbologie selbst.  Dieser Kanal liefert stattdessen **fertige Pixel**: GlassOut
liest die Panel-Texturen direkt aus MSFS (DX12, kein Pop-out) und schickt JPEGs, der
FDC skaliert sie auf die Panelgröße und schiebt sie als gechunkte UDP-Datagramme raus.

Der Kanal ist **zusätzlich**: ohne ``stream = …`` in der Panel-Sektion ändert sich
nichts am bisherigen Verhalten.

Nichts hier importiert Qt — der Bildpfad ist damit headless testbar und läuft auch
im Test-/Werkzeugbetrieb ohne Oberfläche.
"""

from __future__ import annotations

import io as _io
import json
import math
import struct
import threading
import time
import urllib.error
import urllib.request
import zlib
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Protocol, runtime_checkable

#: Bildstrom-Port (Textkanal 5005, Discovery 5006 — siehe IP_TRANSPORT_SPEC).
IMAGE_PORT = 5007
#: Textkanal: DSP-Frames und CFG-Befehle (auch CFG:TYPE= für die Stream-Only-Firmware).
TEXT_PORT = 5005
#: Panel-Funktionen der Stream-Only-Firmware. Reihenfolge = NVS-Index in
#: `firmware/source/stream/src/paneltype.cpp` — dort parallel pflegen, sonst
#: bekommt ein Panel die falsche Funktion.
PANEL_TYPES = (
    "PFD_CPT", "PFD_FO", "ND_CPT", "ND_FO", "ECAM_U", "ECAM_L",
    "ISIS", "MCDU_CPT", "MCDU_FO", "STANDBY", "WXR",
)
#: Standard-Port der GlassOut-Engine.
GLASSOUT_PORT = 8787
#: Magic am Anfang jedes Bild-Datagramms; die Firmware verwirft alles andere.
MAGIC = b"IMG1"
#: magic(4s) frame(u32) idx(u16) cnt(u16) w(u16) h(u16) flags(u8) crc32(u32) = 21 Byte
HEADER = struct.Struct("<4sIHHHHBI")
HEADER_SIZE = HEADER.size
FLAG_JPEG = 0x01
FLAG_LAST = 0x02
#: Nutzdaten je Datagramm. Die Firmware legt Chunk ``idx`` auf ``idx * CHUNK_PAYLOAD``
#: im PSRAM ab — die Schrittweite MUSS auf beiden Seiten identisch sein. 1200 B passen
#: auch durch getunnelte Strecken (Site Magic, Cloudflare) ohne Fragmentierung.
#: ⚠️ Stand bis 07.11.2026 rechnete der Sender den 21-Byte-Kopf aus der MTU heraus
#: (1179 Nutzlast) und schickte damit gegen die 1200er-Schrittweite der Firmware: ab
#: Chunk 1 war jedes Bild um 21 B je Chunk verschoben, die CRC32 kippte und der
#: Empfaenger verwarf *jedes* Bild — Modus an, Chunks ankommen, kein Bild.
CHUNK_PAYLOAD = 1200
#: Datagramm = Kopf + Nutzlast (1221 B, passt in jede Ethernet-MTU).
DEFAULT_MTU = HEADER_SIZE + CHUNK_PAYLOAD
#: Unvollständige Bilder werden verworfen, nicht nachgefordert — das nächste kommt
#: in 40–100 ms.  200 ms ist großzügig für ein 20-fps-Panel.
FRAME_TIMEOUT_S = 0.2
#: Pause zwischen zwei Datagrammen (s). Ein Bild ist ein Burst aus 15–30 Paketen;
#: ungetaktet verschluckt der Empfaenger den Schwanz des Bursts, und weil es keine
#: Wiederholung gibt, ist damit der *ganze* Frame hin ("komplett 0"). Bei 20 Paketen
#: kostet 1 ms Taktung 20 ms pro Bild - bei 6-8 fps voellig unkritisch.
CHUNK_GAP_S = 0.001


class StreamError(RuntimeError):
    """Bildstrom-Fehler (Quelle, Kodierung oder Versand)."""


# ---------------------------------------------------------------------------
# Rahmen (GlassOut -> FDC)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class StreamFrame:
    """Ein Panelbild, wie es die GlassOut-Engine schickt (volles Panel, JPEG)."""

    panel_id: str
    frame_no: int
    width: int
    height: int
    engine_ts_ms: int
    jpeg: bytes

    def __post_init__(self) -> None:
        if not self.jpeg:
            raise StreamError(f"Frame {self.frame_no} ohne JPEG-Daten")


def parse_glassout_frame(buf: bytes) -> StreamFrame:
    """Zerlegt einen Binärframe der Engine (``docs/STREAM_TRANSPORT_SPEC.md``, 2).

    Format little endian::

        0   u8   panelIdLen
        1   N    panelId (UTF-8)
        N+1 u32  frameNumber
        N+5 u16  width
        N+7 u16  height
        N+9 u64  engineTimestampMs
        N+17 ... JPEG
    """
    if not isinstance(buf, (bytes, bytearray, memoryview)):
        raise StreamError("Binärframe erwartet")
    data = bytes(buf)
    if len(data) < 2:
        raise StreamError("Frame zu kurz")
    name_len = data[0]
    if name_len == 0 or 1 + name_len + 16 > len(data):
        raise StreamError("Frame-Header unvollständig")
    panel_id = data[1:1 + name_len].decode("utf-8", errors="replace")
    off = 1 + name_len
    frame_no, width, height = struct.unpack_from("<IHH", data, off)
    engine_ts = struct.unpack_from("<Q", data, off + 8)[0]
    jpeg = data[off + 16:]
    if len(jpeg) < 4 or jpeg[:2] != b"\xff\xd8":
        raise StreamError(f"Panel {panel_id}: kein JPEG im Frame {frame_no}")
    return StreamFrame(panel_id, frame_no, width, height, engine_ts, jpeg)


# ---------------------------------------------------------------------------
# Chunking (FDC -> Panel)
# ---------------------------------------------------------------------------
def build_chunks(frame_no: int, width: int, height: int, jpeg: bytes,
                 mtu: int = DEFAULT_MTU) -> list[bytes]:
    """Zerlegt ein JPEG in selbstbeschreibende Datagramme (Port IMAGE_PORT).

    Jeder Chunk trägt Frame-Nummer, Index, Gesamtzahl, Panelmaße und die CRC32 über
    das **ganze** JPEG — der Empfänger braucht keinen Handshake und kann Duplikate
    einfach überschreiben.
    """
    if mtu != DEFAULT_MTU:
        # Kein Freibrief: die Firmware rechnet mit einer festen Schrittweite von
        # CHUNK_PAYLOAD. Jeder andere Wert verschiebt das Bild und die CRC kippt.
        raise StreamError(f"MTU muss {DEFAULT_MTU} sein (Nutzlast {CHUNK_PAYLOAD} B = "
                          f"Firmware-Schrittweite), bekam {mtu}")
    nutzlast = CHUNK_PAYLOAD
    crc = zlib.crc32(jpeg) & 0xFFFFFFFF
    cnt = max(1, (len(jpeg) + nutzlast - 1) // nutzlast)
    if cnt > 0xFFFF:
        raise StreamError("Bild passt nicht in 65535 Chunks")
    out: list[bytes] = []
    for idx in range(cnt):
        teil = jpeg[idx * nutzlast:(idx + 1) * nutzlast]
        flags = FLAG_JPEG | (FLAG_LAST if idx == cnt - 1 else 0)
        out.append(HEADER.pack(MAGIC, frame_no & 0xFFFFFFFF, idx, cnt,
                               width, height, flags, crc) + teil)
    return out


class ChunkAssembler:
    """Empfangsseite: setzt Datagramme wieder zum JPEG zusammen.

    Das ist die Referenz für die Firmware-Implementierung — gleiche Regeln:
    unvollständige Frames verfallen nach ``timeout_s``, CRC wird geprüft,
    Reihenfolge ist egal.
    """

    def __init__(self, timeout_s: float = FRAME_TIMEOUT_S,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._timeout_s = timeout_s
        self._clock = clock
        self._frames: dict[int, dict] = {}
        self.dropped_incomplete = 0
        self.dropped_crc = 0

    def feed(self, datagram: bytes) -> Optional[bytes]:
        """Nimmt ein Datagramm; gibt das fertige JPEG zurück, sobald es vollständig ist."""
        self._prune()
        if len(datagram) < HEADER_SIZE:
            return None
        magic, frame_no, idx, cnt, w, h, flags, crc = HEADER.unpack_from(datagram, 0)
        if magic != MAGIC or cnt == 0 or idx >= cnt or w == 0 or h == 0:
            return None
        eintrag = self._frames.get(frame_no)
        if eintrag is None:
            eintrag = {"cnt": cnt, "w": w, "h": h, "crc": crc,
                       "teile": {}, "seit": self._clock()}
            self._frames[frame_no] = eintrag
        eintrag["teile"][idx] = datagram[HEADER_SIZE:]
        if len(eintrag["teile"]) < eintrag["cnt"]:
            return None
        self._frames.pop(frame_no, None)
        roh = b"".join(eintrag["teile"][i] for i in range(eintrag["cnt"]))
        if (zlib.crc32(roh) & 0xFFFFFFFF) != eintrag["crc"]:
            self.dropped_crc += 1
            return None
        return roh

    def _prune(self) -> None:
        jetzt = self._clock()
        for frame_no, eintrag in list(self._frames.items()):
            if jetzt - eintrag["seit"] > self._timeout_s:
                self._frames.pop(frame_no, None)
                self.dropped_incomplete += 1


# ---------------------------------------------------------------------------
# Quellen
# ---------------------------------------------------------------------------
@runtime_checkable
class PanelSource(Protocol):
    """Etwas, das Panelbilder liefert."""

    def start(self, on_frame: Callable[[StreamFrame], None],
              on_panels: Optional[Callable[[list], None]] = None) -> None: ...

    def stop(self) -> None: ...


class TestPatternSource:
    """Synthetische Frames ohne Sim — bringt ein Panel ohne MSFS in Betrieb.

    Zeichnet eine Art Kursrose mit tickender Zahl; Frame-Nummer und Zeit laufen mit,
    damit man auf dem Panel sofort sieht, ob Bilder ankommen.
    """

    #: Kein pytest-Testfall, obwohl der Klassenname mit "Test" beginnt.
    __test__ = False

    def __init__(self, panel_id: str = "TEST", width: int = 480, height: int = 480,
                 fps: float = 20.0, quality: int = 75) -> None:
        self.panel_id = panel_id
        self.width = width
        self.height = height
        self.fps = max(1.0, fps)
        self.quality = quality
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_no = 0

    def start(self, on_frame: Callable[[StreamFrame], None],
              on_panels: Optional[Callable[[list], None]] = None) -> None:
        from PIL import Image, ImageDraw

        if self._running:
            return
        self._running = True
        if on_panels is not None:
            on_panels([{"id": self.panel_id, "name": self.panel_id,
                        "width": self.width, "height": self.height}])

        def lauf() -> None:
            intervall = 1.0 / self.fps
            while self._running:
                t0 = time.monotonic()
                img = Image.new("RGB", (self.width, self.height), (12, 12, 16))
                d = ImageDraw.Draw(img)
                cx, cy, r = self.width // 2, self.height // 2, min(self.width, self.height) // 2 - 10
                d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(120, 200, 255), width=3)
                # Stundenmarken auf der Rose — reine Kosmetik, zeigt aber sofort,
                # dass ein vollständiges Bild (nicht nur ein Chunk) angekommen ist.
                for i in range(12):
                    x = cx + int(r * 0.85 * math.sin(i * math.pi / 6))
                    y = cy - int(r * 0.85 * math.cos(i * math.pi / 6))
                    d.line([x, y, cx + int((x - cx) * 1.12), cy + int((y - cy) * 1.12)],
                           fill=(200, 200, 200), width=2)
                d.text((10, 10), f"FDC STREAM  {self.panel_id}", fill=(230, 230, 230))
                d.text((10, self.height - 24), f"frame {self._frame_no}", fill=(150, 220, 150))
                d.line([cx, cy, cx + int(r * 0.7), cy - int(r * 0.2)], fill=(255, 80, 80), width=4)
                self._frame_no += 1
                puffer = _io.BytesIO()
                img.save(puffer, format="JPEG", quality=self.quality)
                on_frame(StreamFrame(self.panel_id, self._frame_no, self.width,
                                     self.height, int(time.time() * 1000),
                                     puffer.getvalue()))
                rest = intervall - (time.monotonic() - t0)
                if rest > 0:
                    time.sleep(rest)

        self._thread = threading.Thread(target=lauf, name="stream-testpattern", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


def _default_ws_factory(url: str):
    """WebSocket erzeugen — Import erst hier, damit das Modul ohne die Abhängigkeit
    importierbar bleibt (Tests, Werkzeuge)."""
    try:
        from websocket import create_connection  # websocket-client
    except ImportError as exc:  # pragma: no cover - Umgebungsfehler
        raise StreamError("websocket-client fehlt (pip install websocket-client)") from exc
    return create_connection(url, timeout=10)


class GlassOutSource:
    """Liest Panelbilder von der GlassOut-Engine (``ws://host:8787/ws``).

    ``connectionType=process`` ist Pflicht: reine Viewer halten die Engine nicht am
    Leben, sie fährt ~5 s nach dem letzten Prozess-Client herunter.
    """

    def __init__(self, panel_id: str, host: str = "127.0.0.1", port: int = GLASSOUT_PORT,
                 *, app_key: str = "fdc", name: str = "FDC",
                 crop: Optional[tuple[int, int, int, int]] = None,
                 target_fps: int = 10,
                 ws_factory: Optional[Callable[[str], object]] = None,
                 log: Optional[Callable[[str], None]] = None) -> None:
        self.panel_id = panel_id
        self.host = host
        self.port = port
        self.app_key = app_key
        self.name = name
        #: Ausschnitt schon in der Engine schneiden (Quellpixel!) — spart LAN-Verkehr
        #: und PIL-Arbeit.  Die Engine liefert dann nur diesen Ausschnitt als JPEG.
        self.crop = crop
        #: Abonnement-Rate.  Die Engine erlaubt 10–120 fps; alles über der Panel-Rate
        #: ist Funkverkehr, den der FDC hinterher wegwirft (Stream-Status zeigte
        #: "in 4081 / skipped 4081").  Also die Untergrenze, nicht 30.
        self.target_fps = max(10, min(120, int(target_fps)))
        self._ws_factory = ws_factory or _default_ws_factory
        self._log = log or (lambda _t: None)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._ws = None
        self.panels: list = []
        self.frames_in = 0

    # -- Lifecycle ---------------------------------------------------------
    def start(self, on_frame: Callable[[StreamFrame], None],
              on_panels: Optional[Callable[[list], None]] = None) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._reader_loop, args=(on_frame, on_panels),
            name=f"stream-{self.panel_id}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        ws = self._ws
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    @property
    def url(self) -> str:
        return (f"ws://{self.host}:{self.port}/ws?name={self.name}"
                f"&appKey={self.app_key}&connectionType=process")

    # -- Reader ------------------------------------------------------------
    def _reader_loop(self, on_frame: Callable[[StreamFrame], None],
                     on_panels: Optional[Callable[[list], None]]) -> None:
        wartezeit = 1.0
        while self._running:
            ws = None
            try:
                ws = self._ws_factory(self.url)
                self._ws = ws
                wartezeit = 1.0
                self._log(f"[stream] verbunden: {self.url}")
                self._reader(ws, on_frame, on_panels)
            except StreamError as exc:
                self._log(f"[stream] {exc}")
                return
            except Exception as exc:  # Verbindungsabbruch, Engine aus, …
                self._log(f"[stream] Verbindung weg ({type(exc).__name__}: {exc})")
            finally:
                self._ws = None
                schliessen = getattr(ws, "close", None)
                if callable(schliessen):
                    try:
                        schliessen()
                    except Exception:
                        pass
            if not self._running:
                return
            time.sleep(wartezeit)
            wartezeit = min(wartezeit * 2, 10.0)

    def _reader(self, ws, on_frame: Callable[[StreamFrame], None],
                on_panels: Optional[Callable[[list], None]]) -> None:
        abonniert = False
        while self._running:
            daten = ws.recv()
            if daten is None:
                return
            if isinstance(daten, bytes):
                try:
                    frame = parse_glassout_frame(daten)
                except StreamError as exc:
                    self._log(f"[stream] {exc}")
                    continue
                self.frames_in += 1
                on_frame(frame)
                continue
            # Textframe: JSON (welcome/panels/state/…)
            try:
                nachricht = json.loads(daten)
            except (ValueError, TypeError):
                continue
            art = nachricht.get("type")
            if art in ("welcome", "panels"):
                self.panels = list(nachricht.get("panels") or self.panels or [])
                if on_panels is not None and self.panels:
                    on_panels(self.panels)
                if not abonniert and art == "welcome":
                    self._subscribe(ws, self.panels)
                    abonniert = True

    def _subscribe(self, ws, panels: list) -> None:
        ziel = self.panel_id
        namen = {str(p.get("id", "")) for p in panels if isinstance(p, dict)}
        if panels and ziel not in namen:
            self._log(f"[stream] Panel '{ziel}' nicht in der Engine-Liste "
                      f"(vorhanden: {', '.join(sorted(namen)) or '—'})")
        nachricht = {"type": "subscribe.panel", "panelId": ziel,
                     "targetFps": self.target_fps}
        if self.crop:
            x, y, w, h = (int(v) for v in self.crop)
            # Der Ausschnitt gilt in QUELL-Pixeln (Panel-Nativgröße der Engine,
            # typ. 768x768 oder 1024x1024), nicht in Panel-Pixeln.
            nachricht["crop"] = {"x": x, "y": y, "w": w, "h": h}
        ws.send(json.dumps(nachricht))
        ausschnitt = (f", crop {self.crop[0]},{self.crop[1]} "
                      f"{self.crop[2]}x{self.crop[3]}" if self.crop else "")
        self._log(f"[stream] abonniert: {ziel} @ {self.target_fps} fps{ausschnitt}")


# ---------------------------------------------------------------------------
# Kodierung + Versand
# ---------------------------------------------------------------------------
class StreamEncoder:
    """Schneidet/skaliert ein Panelbild auf die Zielgröße und kodiert JPEG."""

    def __init__(self, width: int, height: int, quality: int = 75,
                 crop: Optional[tuple[int, int, int, int]] = None) -> None:
        if width <= 0 or height <= 0:
            raise StreamError(f"Zielgröße unbrauchbar: {width}x{height}")
        self.width = int(width)
        self.height = int(height)
        self.quality = max(30, min(95, int(quality)))
        if crop:
            self.crop = (int(crop[0]), int(crop[1]), int(crop[2]), int(crop[3]))
        else:
            self.crop = None
        self._letzte_nummer: Optional[int] = None
        self._letzte_qualitaet: Optional[int] = None
        self._letztes: Optional[tuple[int, int, bytes]] = None

    def encode(self, frame: StreamFrame, quality: Optional[int] = None) -> tuple[int, int, bytes]:
        """-> (breite, höhe, jpeg-bytes).  Gleiche Frame-Nummer = kein Neu-Kodieren."""
        q = self.quality if quality is None else max(30, min(95, int(quality)))
        if (self._letzte_nummer == frame.frame_no and self._letztes is not None
                and self._letzte_qualitaet == q):
            return self._letztes
        from PIL import Image

        with Image.open(_io.BytesIO(frame.jpeg)) as bild:
            bild = bild.convert("RGB")
            if self.crop:
                bild = bild.crop(self.crop)
            if bild.size != (self.width, self.height):
                # LANCZOS statt BILINEAR: GlassOut liefert 768x768, das Panel zeigt 480x480.
                # Beim Verkleinern verwischt BILINEAR die feinen PFD-Symbole (Skalen,
                # Wegweiser, Zahlen) zu Brei — LANCZOS kostet ~10 ms und ist sichtbar
                # schaerfer. Rechenzeit ist bei 6-8 fps reichlich vorhanden.
                resampling = getattr(Image, "Resampling", Image)
                bild = bild.resize((self.width, self.height), resampling.LANCZOS)
            puffer = _io.BytesIO()
            # subsampling=2 = 4:2:0 (Pillow-Standard). 4:4:4 NICHT verwenden: JPEGDEC
            # auf dem Panel gibt dann blockige Chroma-Artefakte ueber die ganze Flaeche
            # aus (Luma bleibt lesbar, Farben sind Muell) - live 07.11.2026 gesehen.
            # Schaerfe kommt stattdessen aus LANCZOS + hoher Qualitaet.
            bild.save(puffer, format="JPEG", quality=q,
                      subsampling=2, optimize=False)
        ergebnis = (self.width, self.height, puffer.getvalue())
        self._letzte_nummer = frame.frame_no
        self._letzte_qualitaet = q
        self._letztes = ergebnis
        return ergebnis


class QualityLadder:
    """Regelt die JPEG-Qualität am Byte-Budget je Bild.

    GlassOut benutzt intern dieselbe Leiter (Q40/60/75/85/100) und regelt sie
    pro Subscriber nach dem WebSocket-Pufferstand.  In unserer Proxy-Kette ist
    die Qualität aber *unser* Knopf: Die Engine-Lieferung ist nur die Quelle,
    über Funk geht die Neukodierung im FDC.

    Zwei Betriebsarten:

    * ``budget_bytes = 0`` — offener Kreis, feste Qualität.  Verhalten wie
      bisher, damit sich ohne Konfiguration nichts ändert.
    * ``budget_bytes > 0`` — die Stufe wandert die Leiter hinunter, solange das
      geglättete Bild größer als das Budget ist, und wieder hinauf, wenn es
      dauerhaft deutlich darunter liegt.  Das ist der Ersatz für die
      Adaptive-Qualität der Engine, die für UDP-Panels nicht greift.

    Bewusst *kein* Rückkanal nötig: der Regelkreis misst die Bytes, die wir
    tatsächlich verschicken.  Ein echtes Link-Feedback (Panel meldet
    Decode-Zeit/Verluste) kann später über ``melde_feedback()`` dazukommen,
    ohne die Leiter umzubauen.
    """

    STUFEN = (40, 60, 75, 85, 100)

    def __init__(self, start: int = 75, minimum: int = 40, maximum: int = 100,
                 budget_bytes: int = 0, fenster: int = 5,
                 steig_nach: int = 15) -> None:
        self.minimum = max(30, min(95, int(minimum)))
        self.maximum = max(self.minimum, max(30, min(95, int(maximum))))
        self.budget_bytes = max(0, int(budget_bytes))
        self.fenster = max(1, int(fenster))
        self.steig_nach = max(1, int(steig_nach))
        self._meldungen: list[int] = []
        self._ruhig = 0
        # Ohne Budget bleibt die eingestellte Qualität exakt stehen (Rückwärts-
        # kompatibilität: alte Konfiguration mit quality=90 liefert weiterhin 90).
        # Erst wer ein Budget setzt, will die Leiter - und wird auf ihre Stufen
        # eingerastet, damit die Regelung nicht zwischen 85 und 88 pendelt.
        if self.budget_bytes > 0:
            self._wert = self._naechste_stufe(int(start))
        else:
            self._wert = max(self.minimum, min(95, int(start)))

    # -- Stufen ------------------------------------------------------------
    def _leiter(self) -> list[int]:
        """Erlaubte Stufen innerhalb [minimum, maximum], aufsteigend."""
        stufen = [s for s in self.STUFEN if self.minimum <= s <= self.maximum]
        if not stufen:
            return [self.minimum]
        return stufen

    def _naechste_stufe(self, wert: int) -> int:
        """Auf die Leiter einrasten (nächstgelegene erlaubte Stufe)."""
        return min(self._leiter(), key=lambda s: abs(s - int(wert)))

    @property
    def aktuell(self) -> int:
        return self._wert

    def melde(self, bytes_groesse: int) -> Optional[int]:
        """Ein verschicktes Bild melden.  -> neue Stufe, wenn gewechselt wurde."""
        if self.budget_bytes <= 0:
            return None
        self._meldungen.append(int(bytes_groesse))
        if len(self._meldungen) > self.fenster:
            self._meldungen.pop(0)
        if len(self._meldungen) < self.fenster:
            return None
        schnitt = sum(self._meldungen) / len(self._meldungen)
        leiter = self._leiter()
        stelle = leiter.index(self._wert) if self._wert in leiter else 0
        if schnitt > self.budget_bytes:
            if stelle > 0:
                self._wert = leiter[stelle - 1]
                self._meldungen.clear()
                self._ruhig = 0
                return self._wert
            return None
        if schnitt <= 0.6 * self.budget_bytes:
            self._ruhig += 1
            if self._ruhig >= self.steig_nach and stelle < len(leiter) - 1:
                self._wert = leiter[stelle + 1]
                self._meldungen.clear()
                self._ruhig = 0
                return self._wert
        else:
            self._ruhig = 0
        return None

    def melde_feedback(self, decode_ms: float, verluste: int) -> Optional[int]:
        """Rückkanal (später): Panel meldet Decode-Zeit und verlorene Bilder.

        Noch nicht verdrahtet — die Firmware schickt derzeit kein ``STAT:``.
        Vorbereitet, damit die Regelung später ohne Umbau schärfer wird.
        """
        return None


class ImageSender:
    """Schickt Bilder als gechunkte UDP-Datagramme an ein Panel."""

    def __init__(self, ip: str, port: int = IMAGE_PORT, mtu: int = DEFAULT_MTU,
                 sock=None, chunk_gap: float = CHUNK_GAP_S) -> None:
        self.ip = ip
        self.port = port
        self.mtu = mtu
        self.chunk_gap = max(0.0, float(chunk_gap))
        self._sock = sock
        self.chunks_sent = 0
        self.bytes_sent = 0
        self.errors = 0

    def send(self, width: int, height: int, jpeg: bytes, frame_no: int) -> int:
        if self._sock is None:
            import socket as _socket
            self._sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
            self._sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_BROADCAST, 1)
        datagramme = build_chunks(frame_no, width, height, jpeg, self.mtu)
        for nummer, d in enumerate(datagramme):
            if self.chunk_gap and nummer:
                # Nur zwischen den Paketen, nicht vor dem ersten: der Bildaufbau
                # beginnt damit sofort.
                time.sleep(self.chunk_gap)
            try:
                self._sock.sendto(d, (self.ip, self.port))
            except OSError:
                self.errors += 1
                return 0
        self.chunks_sent += len(datagramme)
        self.bytes_sent += len(jpeg)
        return len(datagramme)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None


# ---------------------------------------------------------------------------
# Konfiguration aus der Panel-Sektion
# ---------------------------------------------------------------------------
def _parse_crop(text: str) -> Optional[tuple[int, int, int, int]]:
    if not text:
        return None
    teile = [t for t in str(text).replace(" ", "").split(",") if t]
    if len(teile) != 4:
        raise StreamError(f"stream_crop braucht 4 Zahlen (x,y,w,h), bekam '{text}'")
    try:
        x, y, w, h = (int(float(t)) for t in teile)
    except ValueError as exc:
        raise StreamError(f"stream_crop ist keine Zahlenliste: '{text}'") from exc
    if w <= 0 or h <= 0:
        raise StreamError(f"stream_crop mit w/h <= 0: '{text}'")
    return (x, y, w, h)


def _parse_bool(wert, standard: bool = False) -> bool:
    """INI-Wahrheitswerte: leer = ``standard``, sonst ja/nein/1/0/on/off/true."""
    if wert is None:
        return standard
    text = str(wert).strip().lower()
    if not text:
        return standard
    if text in ("1", "ja", "yes", "on", "true", "an"):
        return True
    if text in ("0", "nein", "no", "off", "false", "aus"):
        return False
    return standard


@dataclass
class StreamConfig:
    """Was ein Panel im Bildstrom braucht — direkt aus der Kanal-Konfiguration."""

    slug: str
    panel_id: str
    width: int
    height: int
    fps: float = 20.0
    quality: int = 75
    crop: Optional[tuple[int, int, int, int]] = None
    ip: str = "AUTO"
    port: int = IMAGE_PORT
    source: str = "glassout"
    engine_host: str = "127.0.0.1"
    engine_port: int = GLASSOUT_PORT
    #: Abonnement-Rate bei der Engine (10–120).  Über der Panel-Rate abonnieren
    #: heißt Bilder empfangen und wegwerfen — 10 ist die Untergrenze der API.
    engine_fps: int = 10
    #: ``crop`` schon in der Engine schneiden lassen (spart LAN + PIL-Zeit).
    #: Aus = Ausschnitt erst im FDC, z.B. wenn die Engine-Version es nicht kann.
    crop_remote: bool = True
    #: Grenzen der Qualitätsleiter und Byte-Budget je Bild (0 = feste Qualität).
    quality_min: int = 40
    quality_max: int = 100
    kb_budget: float = 0.0

    @classmethod
    def from_ip_cfg(cls, slug: str, ip_cfg: dict) -> Optional["StreamConfig"]:
        """Baut die Konfiguration aus der Sektion; ``None`` = kein Bildstrom gewünscht."""
        quelle = str(ip_cfg.get("stream") or "").strip().lower()
        if not quelle or quelle in ("aus", "off", "0", "nein", "no"):
            return None
        # stream_w/stream_h zuerst: scale_w/scale_h beschreiben die Groesse der
        # LOKALEN Zeichnung im Panel-Canvas (z. B. ISIS 410x410 auf 480x480),
        # nicht die Bildflaeche. Wer den Stream aufs ganze Display will, setzt
        # stream_w = 480, ohne die lokale Darstellung zu verschieben.
        try:
            breite = int(ip_cfg.get("stream_w") or ip_cfg.get("scale_w") or 480)
            hoehe = int(ip_cfg.get("stream_h") or ip_cfg.get("scale_h") or breite)
        except (TypeError, ValueError):
            raise StreamError(f"{slug}: stream_w/stream_h sind keine Zahlen")
        return cls(
            slug=slug,
            panel_id=str(ip_cfg.get("stream_panel") or slug),
            width=breite,
            height=hoehe,
            fps=float(ip_cfg.get("stream_fps") or 20),
            quality=int(ip_cfg.get("stream_quality") or 75),
            crop=_parse_crop(str(ip_cfg.get("stream_crop") or "")),
            ip=str(ip_cfg.get("stream_ip") or ip_cfg.get("ip") or "AUTO"),
            port=int(ip_cfg.get("stream_port") or IMAGE_PORT),
            source=quelle,
            engine_host=str(ip_cfg.get("stream_engine") or "127.0.0.1"),
            engine_port=int(ip_cfg.get("stream_engine_port") or GLASSOUT_PORT),
            engine_fps=int(ip_cfg.get("stream_engine_fps") or 10),
            crop_remote=_parse_bool(ip_cfg.get("stream_crop_remote"), True),
            quality_min=int(ip_cfg.get("stream_quality_min") or 40),
            quality_max=int(ip_cfg.get("stream_quality_max") or 100),
            kb_budget=float(ip_cfg.get("stream_kb_budget") or 0.0),
        )


def configs_from_mappings(mappings_dict: dict) -> list[StreamConfig]:
    """Alle Panels mit ``stream = …`` aus den geparsten INI-Sektionen ziehen.

    Das ist die Naht fuer die Oberflaeche: sie kennt ``current_mappings``
    (``{Panelname: [Mappings]}``) und muss hier nur noch durchreichen — welche
    Panels einen Bildstrom wollen, entscheidet allein die INI.
    """
    from infra.ip_transport import iter_ip_panels

    configs: list[StreamConfig] = []
    for panel_name, mappings in (mappings_dict or {}).items():
        try:
            ip_cfg = iter_ip_panels(mappings)
        except Exception:
            continue
        if not ip_cfg:
            continue
        cfg = StreamConfig.from_ip_cfg(str(panel_name), ip_cfg)
        if cfg is not None:
            configs.append(cfg)
    return configs


# ---------------------------------------------------------------------------
# Orchestrierung
# ---------------------------------------------------------------------------
class PanelStreamer:
    """Verbindet Quelle, Kodierung und Versand für mehrere Panels."""

    def __init__(self, configs: Iterable[StreamConfig], *,
                 source_factory: Optional[Callable[[StreamConfig], PanelSource]] = None,
                 sender_factory: Optional[Callable[[StreamConfig], ImageSender]] = None,
                 resolve_ip: Optional[Callable[[str], Optional[str]]] = None,
                 log: Optional[Callable[[str], None]] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._configs = list(configs)
        self._source_factory = source_factory or self._default_source
        self._sender_factory = sender_factory or (
            lambda cfg: ImageSender(cfg.ip if cfg.ip not in ("", "AUTO") else "127.0.0.1",
                                    cfg.port))
        self._resolve_ip = resolve_ip
        self._log = log or (lambda _t: None)
        self._clock = clock
        self._running = False
        self._sources: dict[str, PanelSource] = {}
        self._senders: dict[str, ImageSender] = {}
        self._encoder: dict[str, StreamEncoder] = {}
        self._leiter: dict[str, "QualityLadder"] = {}
        self._crop_warnung: set[str] = set()
        self._letzter_send: dict[str, float] = {}
        self._letzte_quelle: dict[str, float] = {}
        self.stats: dict[str, dict] = {}

    @staticmethod
    def _default_source(cfg: StreamConfig) -> PanelSource:
        if cfg.source == "test":
            return TestPatternSource(cfg.panel_id, cfg.width, cfg.height,
                                     fps=cfg.fps, quality=cfg.quality)
        return GlassOutSource(cfg.panel_id, cfg.engine_host, cfg.engine_port,
                              name=f"FDC-{cfg.slug}",
                              # Ausschnitt in der Engine schneiden lassen: die
                              # Engine liefert dann nur diese Region als JPEG.
                              crop=cfg.crop if cfg.crop_remote else None,
                              target_fps=cfg.engine_fps)

    # -- Lifecycle ---------------------------------------------------------
    def _ensure(self, cfg: StreamConfig) -> None:
        """Encoder/Sender/Statistik je Panel anlegen — idempotent, auch ohne ``start()``."""
        if cfg.slug in self._encoder:
            return
        # Ist der Ausschnitt schon in der Engine geschnitten, darf der Encoder ihn
        # NICHT nochmal anwenden — sonst wird zweimal geschnitten und das Bild
        # wandert aus dem Panel.
        self._encoder[cfg.slug] = StreamEncoder(cfg.width, cfg.height,
                                                cfg.quality,
                                                None if cfg.crop_remote else cfg.crop)
        self._leiter[cfg.slug] = QualityLadder(
            start=cfg.quality, minimum=cfg.quality_min, maximum=cfg.quality_max,
            budget_bytes=int(cfg.kb_budget * 1024) if cfg.kb_budget else 0)
        self._senders[cfg.slug] = self._sender_factory(cfg)
        self.stats[cfg.slug] = {"in": 0, "out": 0, "skipped": 0, "chunks": 0,
                                "bytes": 0, "encode_ms": 0.0,
                                "kb": 0.0, "kb_schnitt": 0.0,
                                "qualitaet": cfg.quality,
                                "budget_kb": cfg.kb_budget,
                                "engine_fps": cfg.engine_fps,
                                "crop_fern": cfg.crop_remote and cfg.crop is not None}

    def start(self) -> None:
        if self._running:
            return
        if not self._configs:
            self._log("[stream] keine Panel-Sektion mit 'stream = …' — nichts zu tun")
            return
        self._running = True
        for cfg in self._configs:
            self._ensure(cfg)
            quelle = self._source_factory(cfg)
            self._sources[cfg.slug] = quelle
            quelle.start(self._make_handler(cfg),
                         self._make_panels_handler(cfg))
            self._log(f"[stream] {cfg.slug}: Quelle '{cfg.source}' -> "
                      f"{cfg.panel_id} {cfg.width}x{cfg.height} @ {cfg.fps:g} fps "
                      f"-> {cfg.ip}:{cfg.port}")
            self._letzte_quelle[cfg.slug] = self._clock()

    def stop(self) -> None:
        self._running = False
        for quelle in self._sources.values():
            try:
                quelle.stop()
            except Exception:
                pass
        for sender in self._senders.values():
            try:
                sender.close()
            except Exception:
                pass
        self._sources.clear()
        self._senders.clear()

    def ist_verbunden(self, slug: str, timeout_s: float = 3.0) -> bool:
        """Kommt seit weniger als ``timeout_s`` Sekunden ein Bild? (Watchdog)."""
        letzte = self._letzte_quelle.get(slug)
        return letzte is not None and (self._clock() - letzte) <= timeout_s

    # -- Frame-Pfad --------------------------------------------------------
    def _make_panels_handler(self, cfg: StreamConfig):
        def panels(gefunden: list) -> None:
            ids = [str(p.get("id")) for p in gefunden if isinstance(p, dict)]
            self._log(f"[stream] {cfg.slug}: Engine meldet {len(ids)} Panel(s)"
                      + (f": {', '.join(sorted(ids))}" if ids else ""))
        return panels

    def _make_handler(self, cfg: StreamConfig):
        def handler(frame: StreamFrame) -> None:
            self.handle_frame(cfg, frame)
        return handler

    def handle_frame(self, cfg: StreamConfig, frame: StreamFrame) -> bool:
        """Ein Bild durch Kodierung, Ratenbegrenzung und Versand schicken."""
        self._ensure(cfg)
        jetzt = self._clock()
        stats = self.stats[cfg.slug]
        stats["in"] += 1
        self._letzte_quelle[cfg.slug] = jetzt
        # Plausibilitätsprüfung: schickt die Engine wirklich nur den Ausschnitt?
        # Wenn nicht (ältere Engine-Version), wird das Vollbild auf Panelgröße
        # gezogen — sichtbar falsch gerahmt.  Einmal laut werden, nicht schweigen.
        if cfg.crop_remote and cfg.crop and cfg.slug not in self._crop_warnung:
            erwartet = (int(cfg.crop[2]), int(cfg.crop[3]))
            if (frame.width, frame.height) != erwartet:
                self._crop_warnung.add(cfg.slug)
                self._log(f"[stream] {cfg.slug}: Engine liefert {frame.width}x{frame.height}, "
                          f"erwartet {erwartet[0]}x{erwartet[1]} — crop wird offenbar "
                          f"ignoriert. Entweder Engine aktualisieren oder "
                          f"stream_crop_remote = nein setzen.")
        intervall = 1.0 / max(1.0, cfg.fps)
        letzter = self._letzter_send.get(cfg.slug, 0.0)
        if jetzt - letzter < intervall:
            stats["skipped"] += 1
            return False
        try:
            t0 = self._clock()
            leiter = self._leiter.get(cfg.slug)
            qualitaet = leiter.aktuell if leiter is not None else cfg.quality
            breite, hoehe, jpeg = self._encoder[cfg.slug].encode(frame, qualitaet)
            dauer_ms = (self._clock() - t0) * 1000.0
        except StreamError as exc:
            self._log(f"[stream] {cfg.slug}: {exc}")
            return False
        sender = self._senders[cfg.slug]
        ziel = cfg.ip
        if ziel in ("", "AUTO") and self._resolve_ip is not None:
            ziel = self._resolve_ip(cfg.slug) or ""
            if ziel:
                sender.ip = ziel
        if not ziel or ziel == "AUTO":
            stats["skipped"] += 1
            return False
        geschickt = sender.send(breite, hoehe, jpeg, frame.frame_no)
        if not geschickt:
            return False
        self._letzter_send[cfg.slug] = jetzt
        stats["out"] += 1
        stats["chunks"] += geschickt
        stats["bytes"] += len(jpeg)
        stats["encode_ms"] = dauer_ms if not stats["encode_ms"] else (
            0.8 * stats["encode_ms"] + 0.2 * dauer_ms)
        kb = len(jpeg) / 1024.0
        stats["kb"] = kb
        stats["kb_schnitt"] = (kb if not stats["kb_schnitt"]
                               else 0.7 * stats["kb_schnitt"] + 0.3 * kb)
        if leiter is not None:
            neu = leiter.melde(len(jpeg))
            if neu is not None:
                stats["qualitaet"] = neu
                richtung = "runter" if neu < qualitaet else "hoch"
                self._log(f"[stream] {cfg.slug}: Qualität {qualitaet} -> {neu} "
                          f"({richtung}, {kb:.1f} KB/Bild vs. Budget "
                          f"{cfg.kb_budget:g} KB)")
        return True


# ---------------------------------------------------------------------------
# Berichte (ohne print: im Laufzeitcode ist stdout tabu, Ausgabe macht das Tool)
# ---------------------------------------------------------------------------
def query_status(host: str = "127.0.0.1", port: int = GLASSOUT_PORT,
                 timeout: float = 3.0) -> dict:
    """``GET /status`` der GlassOut-Engine — Verbindungstest ohne WebSocket."""
    url = f"http://{host}:{port}/status"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as antwort:
            return json.loads(antwort.read().decode("utf-8", errors="replace"))
    except (urllib.error.URLError, OSError, ValueError) as exc:
        raise StreamError(f"keine Engine unter {url}: {type(exc).__name__}: {exc}") from exc


def status_report(host: str = "127.0.0.1", port: int = GLASSOUT_PORT) -> str:
    """Menschenlesbarer Bericht zur Engine (wirft ``StreamError``, wenn keine da ist)."""
    status = query_status(host, port)
    panels = status.get("panels") or []
    zeilen = [
        f"Engine:      {host}:{port}",
        f"Profil:      {status.get('activeProfileId') or '—'}",
        f"DirectX:     {status.get('directX')}   Sim: {status.get('msfsConnection')}",
        f"Admin:       {status.get('isAdmin')}",
        f"Panels:      {len(panels)}",
    ]
    for p in panels:
        if isinstance(p, dict):
            zeilen.append(f"  - {str(p.get('id', '?')):<24} "
                          f"{p.get('width', '?')}x{p.get('height', '?')}  {p.get('name', '')}")
    return "\n".join(zeilen)


def selftest_report(repeat: int = 5, size: int = 320) -> tuple[bool, str]:
    """Testbild -> Chunks -> Reassembly -> JPEG prüfen.  Ohne Netz, ohne Panel."""
    from PIL import Image

    assembler = ChunkAssembler()
    frames: list[StreamFrame] = []
    quelle = TestPatternSource("TEST", size, size, fps=20.0)
    quelle.start(frames.append)
    try:
        ende = time.monotonic() + max(0.5, repeat / 20.0)
        while time.monotonic() < ende and len(frames) < repeat:
            time.sleep(0.02)
    finally:
        quelle.stop()
    if not frames:
        return False, "FEHLER: Testquelle hat kein Bild geliefert"

    encoder = StreamEncoder(size, size, 70)
    t0 = time.monotonic()
    gesamt_chunks = 0
    fertig = 0
    for frame in frames:
        breite, hoehe, jpeg = encoder.encode(frame)
        for d in build_chunks(frame.frame_no, breite, hoehe, jpeg):
            gesamt_chunks += 1
            ergebnis = assembler.feed(d)
            if ergebnis:
                mit = Image.open(_io.BytesIO(ergebnis))
                if mit.size != (size, size):
                    return False, f"FEHLER: Größe {mit.size} statt ({size}, {size})"
                fertig += 1
    dauer = max(time.monotonic() - t0, 1e-6)
    text = (f"Selbsttest: {fertig}/{len(frames)} Bilder rekonstruiert, "
            f"{gesamt_chunks} Chunks, {dauer * 1000:.1f} ms "
            f"({fertig / dauer:.0f} Bilder/s), "
            f"verworfen: {assembler.dropped_incomplete} unvollständig / "
            f"{assembler.dropped_crc} CRC")
    return fertig == len(frames), text


def collect_frames(host: str, port: int, panel_id: str, count: int = 5,
                   timeout_s: float = 15.0,
                   log: Optional[Callable[[str], None]] = None) -> list[StreamFrame]:
    """Live-Test gegen die Engine: bis zu ``count`` Frames einsammeln."""
    frames: list[StreamFrame] = []
    quelle = GlassOutSource(panel_id, host, port, log=log)
    quelle.start(frames.append,
                 lambda panels: (log or (lambda _t: None))(
                     f"Engine meldet {len(panels)} Panel(s)"))
    try:
        ende = time.monotonic() + timeout_s
        while time.monotonic() < ende and len(frames) < count:
            time.sleep(0.05)
    finally:
        quelle.stop()
    return frames
