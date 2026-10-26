import sys
import time
from pathlib import Path
import serial

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2101"
    name = sys.argv[2] if len(sys.argv) > 2 else "fase2.csv"
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0  # segundos de lectura
    out = Path(__file__).resolve().parents[4] / "data" / name

    # Abrir el puerto reinicia el Uno, así que el ensayo empieza con el script ya escuchando
    with serial.Serial(port, 115200, timeout=1) as ser, open(out, "w") as f:
        f.write("t_us,pulses,direction\n")
        print(f"Capturando {duration:g} s de {port} -> {out}")
        n = 0
        end = time.monotonic() + duration
        while time.monotonic() < end:
            line = ser.readline().decode(errors="ignore").strip()
            row = line.split(",")
            if len(row) != 3:
                continue
            try:
                [int(x) for x in row]  # descarta cabecera y basura
            except ValueError:
                continue
            f.write(line + "\n")
            n += 1
        print(f"Listo: {n} muestras en {out}")
