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
- Power Supply Module (5V o 3.3V + max 700mA)

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
- *Botón*. Sera pensado como un modelo DEVS en dominio electromagnético.

### Planteamiento

Sera en dos fases. En la primera se resuelve la modelización y simulacion + comparación con modelo físico contado dos componentes que son:
- Driver
- Motor DC con encoder

En la segunda fase, se añadirá el sensor (no se simula sino considera como entreda *u(t)*) y boton.

### Primera fase

Primero, el driver. Como se ha mencionado no se cuenta como un componente que influye a la salida de motor. Sin embargo en esta fase se considera que se rota en un sentido, o sea `DRIVER_DIR_A_PIN = 1` y `DRIVER_OUT_B_PIN = 0`. Al ser que se simplifica que solo se rota en un sentido (Forward) por lo que la salidas de driver sera: *OUT_A = V_in V + 700 mA* (lo que envía Arduino) y *OUT_B = 0 V*. Corriente de salida: **700 mA** y Rango de tensiones: **0V - 5V**.

Ahora se queda la parte interesante es Motor DC con encoder.

#### Motor DC

##### Formulación en la Tupla DESS (primer orden)

Formalmente, la tupla del modelo continuo de primer orden $M = \langle U, X, Y, f, \lambda \rangle$ queda definida como:

- **Entrada ($U$)**: $u(t) = V_{in}(t) \in \mathbb{R}$ (tensión aplicada por el driver L293D).
- **Estado ($X$)**: $x(t) = \omega(t) \in \mathbb{R}_{\geq 0}$ (único almacén de energía: la inercia del rotor $J$).
- **Salida ($Y$)**: $y(t) = \omega(t) \in \mathbb{R}_{\geq 0}$, con $\lambda(u,x) = x$.
- **Función de dinámica ($f$)**:

$$
\dot{\omega}(t) = f(u,x) = -\frac{1}{\tau_m}\,\omega(t) + \frac{K_m}{\tau_m}\,V_{in}(t) = \frac{1}{\tau_m}\left( K_m V_{in}(t) - \omega(t) \right)
$$

###### Ecuación diferencial

$$
\dot{\omega}(t) = f(u,x) = \frac{1}{\tau_m}\left( K_m V_{in}(t) - \omega(t) \right)
$$

Desglose y significado de cada término

- $x(t) = \omega(t)$: la **variable de estado** (velocidad angular del ventilador, en rad/s).
- $u(t) = V_{in}(t)$: la **señal de entrada** (voltaje aplicado por el driver L293D, en voltios).
- $\tau_m$: la **constante de tiempo mecánica equivalente**, en segundos. Mide la inercia del sistema: indica cuánto tarda el ventilador en alcanzar el $63.2\%$ de su velocidad final ante un cambio brusco de voltaje.
- $K_m$: la **ganancia estática del motor**, en (rad/s)/V. Indica cuántos rad/s gana el ventilador por cada voltio adicional en régimen permanente.

##### Tupla DTSS (primer orden)

$$
\text{DTSS}_{\text{motor}} = \langle U, X, Y, \delta, \lambda, \Delta t \rangle
$$

Definición de elementos

- **Espacio de estados ($X$)**: $x(k) = \omega(k) \in \mathbb{R}_{\geq 0}$ rad/s (velocidad angular del ventilador).
- **Espacio de entradas ($U$)**: $u(k) = V_{in}(k) \in \mathbb{R}_{\geq 0}$ V (tensión suministrada por el driver L293D).
- **Espacio de salidas ($Y$)**: $y(k) = \omega(k) \in \mathbb{R}_{\geq 0}$ (medida directa de la velocidad).
- **Función de transición ($\delta$)**:

$$
\delta\left( \omega(k), V_{in}(k) \right) = \left( 1 - \frac{T_s}{\tau_m} \right)\omega(k) + \left( \frac{T_s K_m}{\tau_m} \right) V_{in}(k)
$$

- **Función de salida ($\lambda$)**: $y(k) = \lambda\left( \omega(k) \right) = \omega(k)$ (formulación de Moore).
- **Intervalo de muestreo ($\Delta t$)**: $\Delta t = T_s$ s.

###### Lectura intuitiva

$$
\underbrace{\omega(k+1)}_{\text{Velocidad futura}} =
\underbrace{\left( 1 - \frac{T_s}{\tau_m} \right)\omega(k)}_{\text{Inercia que conserva de hoy}} +
\underbrace{\left( \frac{T_s K_m}{\tau_m} \right) V_{in}(k)}_{\text{Empuje que le da el voltaje hoy}}
$$

Desglose y definiciones

- $T_s$ $(\Delta t)$ **— Período de muestreo**: es el intervalo de tiempo fijo (en segundos) que transcurre entre cada actualización síncrona del reloj digital. Representa el ritmo de ejecución del bucle de control.
- $\tau_m$ **— Constante de tiempo mecánica**: es el tiempo (en segundos) que tarda el motor continuo en alcanzar el $63.2\%$ de su velocidad final tras aplicar un escalón de voltaje. Mide la inercia del sistema.
- $K_m$ **— Ganancia estática del motor**: es la relación de transferencia en régimen permanente entre el voltaje aplicado y la velocidad alcanzada en vacío, expresada en rad/s. Indica cuánto gira el motor por cada voltio entregado cuando la aceleración es nula.
