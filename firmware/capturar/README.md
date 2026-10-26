# Lectura de valores

Como capturar de arduino:

```bash
uv run capturar [puerto] [archivo] [segundos]
```

| Argumento  | Por defecto            | Descripción                          |
| ---------- | ---------------------- | ------------------------------------ |
| `puerto`   | `/dev/cu.usbmodem2101` | Puerto serie del Arduino             |
| `archivo`  | `fase2.csv`            | Nombre del CSV, se guarda en `data/` |
| `segundos` | `10`                   | Tiempo de lectura del puerto serie   |

Ejemplo:

```bash
uv run capturar /dev/cu.usbmodem2101 fase2.csv 20
```

El CSV tiene las columnas `t_us,pulses,direction` (`direction`: 0 = FORWARD, 1 = INVERSE).

Al abrir el puerto el Arduino se reinicia, así que los primeros segundos cuentan dentro del tiempo de lectura.
