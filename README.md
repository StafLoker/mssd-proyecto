# MSSD proyecto

Universidad Carlos III de Madrid | **Máster Universitario en Robótica y Automatización**

Asignatura: *Modelizacion y Simulacion de Sistemas Dinamicos*

Curso: 2026 - 2027

Participantes:
- Stefan Oshchypok
- Anthony Manzueta Difo

## Sistema físico plateado  

Description: Sistema de control de clima.

Componentes:
- DHT11
- Button (+ 10 KOm resistencia)
- L293D DC motor driver
- DC motor con encoder
- Power Supply Module (5V o 3.3V + max 990mA)

Diagramas y esquemas:

<img src="assets/MSSD-layout.png" width="400" />
<img src="assets/MSSD-schematic.png" width="800" />

Entradas del sistema:
- Temperatura (señal digital)
- Boton (pull-down). Cambia el sentido giratorio a traves entradas de punte H de driver.

Salidas del sistema:
- Velocidad (w(t)) de motor

## Modelización y simulation

### Limitaciones

Aquí se menciona los limites escogidos para modelo del sistema:

- *DHT11*. Es un sensor digital que envía ya los datos de entrada ([protocolo de comunicación](https://www.electronicwings.com/sensors-modules/dht11)). Por lo que no se pretende establecer una relación entre el *T(t)* y *V(t)*. Se considera como vector entrada *u(t)*.
- *L293D DC motor driver*. Al ser que su respuesta de dicho componente si lo vemos desde punta de vista de dominio electromagnético es casi instantáneo, se consider que influye de ninguna forma al motor (**ganancia 1**). Por lo que se trata dicho componente como operación identidad. Se simplifica que electronica de propio driver por sus capacidad inductivas puede influir al valores de salida como tension y corriente. En caso de querer modelar el componente habrá que describir el schema de circuito que descrita en [data sheet](https://www.ti.com/product/L293D) usando leyes de Kirchhoff.
- *DC motor con encoder*. Para modelar sera escogido un modelo continuo *DESS* basado en **sistema mecánica rotational**, la parte de circuito eléctrica de motor sera despreciada se establece que **corriente *I* sera constate**.
- *Botón*. Para modeles sera pensado como un modelo DEVS.

### Planteamiento

Sera en dos fases. En la primera se resuelve la modelización y simulacion + comparación con modelo físico contado dos componentes que son:
- Driver
- Motor DC con encoder

En la segunda fase, se añadirá el sensor (no se simula sino considera como entreda *u(t)*) y boton.

### Primera fase

