import sys
from pathlib import Path
import serial

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2101"
    out = Path(__file__).resolve().parents[4] / "data" / "fase1.csv"

    # Abrir el puerto reinicia el Uno, así que el ensayo empieza con el script ya escuchando
    with serial.Serial(port, 115200, timeout=4) as ser, open(out, "w") as f:
        f.write("t_us,pulses\n")
        print(f"Capturando de {port} -> {out}")
        n = 0
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                if n > 0:
                    break  # silencio tras recibir datos: ensayo terminado
                continue
            row = line.split(",")
            if len(row) != 2:
                continue
            try:
                [int(x) for x in row]  # descarta cabecera y basura
            except ValueError:
                continue
            f.write(line + "\n")
            n += 1
        print(f"Listo: {n} muestras en {out}")
