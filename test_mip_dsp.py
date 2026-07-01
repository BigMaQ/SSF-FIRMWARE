# MIP DSP Stress Test – Python 3, kein GUI
# Sendet DSP1..DSP3 an COM3, startet mit 1s Takt, wird immer schneller
# (w) 2025 M. Quatember

import serial
import time
import sys

COM_PORT = "COM3"
BAUD = 115200

def dsp_cmd(n, val):
    """Baue DSPx:wert; Befehl"""
    return f"DSP{n}:{val};\r\n".encode("ascii")

def fmt_time(elapsed_ms):
    """Wandle Millisekunden in MM:SS:CC (Zentelsekunden)"""
    total_cs = elapsed_ms // 10            # Zentelsekunden
    cs = total_cs % 100
    total_sec = total_cs // 100
    mm = (total_sec // 60) % 100
    ss = total_sec % 60
    return f"{mm:02d}.{ss:02d}.{cs:02d}"

def main():
    print(f"Öffne {COM_PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(COM_PORT, BAUD, timeout=0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(1.5)  # Arduino reset abwarten
    except Exception as e:
        print(f"FEHLER: Kann {COM_PORT} nicht öffnen: {e}")
        sys.exit(1)

    print("Starte DSP-Stresstest. STRG+C zum Abbrechen.\n")

    v1 = 0      # DSP1 Zähler (CHRONO)
    v3 = 0      # DSP3 Zähler (ELAPSED)
    interval = 1.0   # Start-Takt in Sekunden
    start_ts = time.monotonic()
    cycle = 0

    try:
        while True:
            # DSP2 = Laufzeit (MM:SS:CC)
            elapsed_ms = int((time.monotonic() - start_ts) * 1000)
            t2 = fmt_time(elapsed_ms)

            # Baue drei Befehle
            cmd = (
                dsp_cmd(1, str(v1)) +
                dsp_cmd(2, t2) +
                dsp_cmd(3, str(v3))
            )
            ser.write(cmd)

            cycle += 1
            v1 = (v1 + 1) % 10000
            v3 = (v3 + 1) % 10000

            if cycle % 10 == 0:
                print(f"Zyklus {cycle:5d} | DSP1={v1:04d}  DSP2={t2}  DSP3={v3:04d}  Intervall={interval:.2f}s", end="\r")

            time.sleep(interval)

            # Alle 50 Zyklen schneller werden
            if cycle % 50 == 0 and interval > 0.05:
                interval = max(0.05, interval * 0.7)
                print(f"\n>> Neuer Takt: {interval:.3f}s (Zyklus {cycle})")

    except KeyboardInterrupt:
        print(f"\n\nABGEBROCHEN nach {cycle} Zyklen.")
    finally:
        ser.close()
        print(f"{COM_PORT} geschlossen.")

if __name__ == "__main__":
    main()
