# Proyecto Final: Celda Robotizada de Clasificación Oceanix

**Equipo de Desarrollo:**
- Alan Ariel Salazar
- Anton Dopico
- Pablo Barran Franco

**Institución:** Universidad Intercontinental de la Empresa  
**Profesor:** Eladio Dapena  
**Asignatura:** Tecnología de Automatización y Robotización Empresarial  
**Fecha de Finalización:** 5 de diciembre de 2025

---

## 1. Resumen General

La empresa OCEANIX S.A. requiere realizar el diseño y construcción de una maqueta para una celda robotizada que simule el proceso de empacar productos terminados.

Los productos terminados, procedentes de una de las plantas de envasado, deben ser inspeccionados para verificar que cumplan los estándares de calidad, y en el caso de no cumplirlos deberán ser descartados.

El empaquetado de los productos se realiza en pallets con capacidad para tres productos cada uno para su almacenamiento y posterior comercialización.

El sistema desarrollado integra un robot colaborativo Ned2 de seis grados de libertad con sensores inductivos para la detección de materiales ferrosos y un sistema de semaforización avanzado basado en microcontroladores Arduino. El flujo operativo completo comienza con la detección de piezas en un alimentador automatizado mediante sensor digital, continúa con la inspección de calidad mediante sensor inductivo durante una ventana temporal de 7 segundos con lógica de latching, y finaliza con la clasificación automática donde las piezas metálicas se descartan en una zona específica, mientras que las piezas no metálicas se paletizan en un almacén de tres posiciones.

La arquitectura del sistema se basa en una separación clara de responsabilidades mediante una configuración de doble Arduino: el Arduino Supervisor actúa como coordinador central del sistema gestionando la comunicación bidireccional con el robot, la lectura del sensor inductivo y el control del semáforo perimetral, mientras que el Arduino Periférico se especializa exclusivamente en el control de indicadores luminosos LED mediante comunicación I2C. Esta arquitectura modular permite un mantenimiento independiente de componentes y facilita futuras expansiones industriales.

## 2. Objetivo General

Diseñar y construir una celda robotizada que permita simular físicamente el envasado, inspección y paletizado de un producto de una planta de producción de la empresa OCEANIX S.A.

## 3. Diseño del Layout

### 3.1 Descripción y Justificación del Layout Propuesto

El diseño del layout de la celda robotizada se ha concebido con el objetivo de optimizar el flujo de materiales y minimizar los tiempos de ciclo mediante una disposición compacta y eficiente de todos los componentes. El robot Ned2 se posiciona en el centro geométrico del área de trabajo, permitiendo acceso equitativo tanto a la zona de inspección como a las posiciones de paletizado con movimientos mínimos y trayectorias optimizadas. Esta disposición central reduce el tiempo total de ciclo en aproximadamente un 30 por ciento comparado con disposiciones lineales tradicionales donde el robot debe desplazarse mayores distancias entre estaciones de trabajo.

La zona de inspección se ubica estratégicamente en el eje central del robot para garantizar precisión máxima en la detección inductiva de materiales ferrosos. El sensor inductivo se encuentra posicionado de manera que el robot pueda mantener la pieza directamente sobre el área de detección durante toda la ventana temporal de inspección de 7 segundos, minimizando interferencias electromagnéticas y asegurando lecturas consistentes.

El alimentador de piezas y el área de descartes se colocan en posiciones opuestas del layout para evitar interferencias físicas durante las operaciones de manipulación. Esta disposición permite que el robot ejecute movimientos de recogida y descarte sin necesidad de cruzar trayectorias que puedan generar conflictos espaciales.

Los tres slots de paletizado se disponen en formación triangular alrededor de un punto de aproximación común, optimizando el espacio disponible y permitiendo que el robot acceda a cada posición con movimientos mínimos desde un punto de referencia compartido. Esta configuración facilita la expansión futura del sistema a una segunda posición de inspección según se especifica en los requisitos del proyecto.

El semáforo perimetral se posiciona a una distancia de 500 milímetros del área de trabajo principal, garantizando visibilidad desde toda la zona operativa y proporcionando retroalimentación visual inmediata del estado del sistema a los operadores.

### 3.2 Indicadores Luminosos y Posición Física de Componentes

El sistema implementa múltiples niveles de indicadores luminosos estratégicamente distribuidos para proporcionar comunicación visual clara y efectiva con los operadores durante todas las fases del proceso.

**Semáforo Perimetral:** El semáforo perimetral consiste en tres LEDs de 5 milímetros (verde, amarillo y rojo) montados en una estructura elevada y visible desde toda la zona de trabajo. Este semáforo indica el estado operativo del sistema mediante diferentes modos de funcionamiento: amarillo titilante durante el estado idle cuando el sistema está disponible para recibir nuevas piezas, amarillo fijo durante la ventana de inspección de 7 segundos, rojo fijo cuando se detecta material metálico y la pieza será descartada, y verde fijo cuando la pieza ha pasado la inspección y será paletizada. El control del semáforo se realiza mediante comunicación I2C entre el Arduino Supervisor y el Arduino Periférico, garantizando sincronización precisa con el estado del sistema.

**LEDs del Robot Ned2:** El robot colaborativo Ned2 incorpora un anillo LED integrado que proporciona retroalimentación visual inmediata del estado de clasificación durante las operaciones. Los LEDs del robot se configuran en color amarillo durante la fase de inspección, rojo cuando se detecta metal y la pieza será descartada, y verde cuando la pieza es aceptada y será almacenada en el pallet. Esta retroalimentación visual complementa la información proporcionada por el semáforo perimetral y permite a los operadores identificar rápidamente el estado del robot incluso desde distancias cercanas.

**Zona de Seguridad:** Se implementan indicadores adicionales para demarcar visualmente las áreas de trabajo restringido y las zonas de seguridad donde los operadores no deben ingresar durante las operaciones automatizadas. Estos indicadores contribuyen a la seguridad operativa del sistema.

### 3.3 Esquema del Diseño e Imagen de la Implementación

**Esquema Técnico del Layout:**

El layout físico de la celda robotizada se organiza según las siguientes coordenadas y posiciones relativas:

- **Robot Ned2:** Posición central en coordenadas cartesianas (X: 0mm, Y: 0mm, Z: 200mm) respecto al sistema de referencia de la base del robot. Esta posición central permite acceso radial a todas las zonas de trabajo.

- **Zona de Alimentación:** Ubicada en la posición izquierda del layout respecto al robot, con coordenadas aproximadas (X: 0mm, Y: -247mm, Z: 129mm) para la posición de recogida. El alimentador incorpora un sensor digital que emite señal cuando hay un producto disponible en la posición de recogida.

- **Zona de Inspección:** Área circular de 100 milímetros de radio centrada en coordenadas (X: 288mm, Y: -264mm, Z: 172mm). Esta zona permite que el robot mantenga la pieza estática sobre el sensor inductivo durante la ventana temporal de inspección.

- **Pallet de 3 Slots:** Disposición triangular a 300 milímetros del robot en coordenadas base (X: 382mm, Y: 0mm, Z: 93mm). Los tres slots se distribuyen en incrementos de 50 milímetros en el eje Y: Slot 1 en Y: +50mm, Slot 2 en Y: 0mm, y Slot 3 en Y: -50mm.

- **Área de Descartes:** Ubicada en la posición derecha del layout en coordenadas (X: 237mm, Y: 193mm, Z: 136mm). Esta zona recibe las piezas que no han superado la inspección de calidad.

- **Semáforo Perimetral:** Posicionado a 500 milímetros del área de trabajo principal, montado en estructura elevada para máxima visibilidad.

**Arquitectura de Control:**

La arquitectura del sistema se organiza en tres capas funcionales claramente definidas:

- **Capa de Control (Python):** Implementada en el archivo `main.py`, esta capa gestiona la lógica de alto nivel del robot Ned2, orquesta el ciclo completo de clasificación, implementa la máquina de estados finita del robot, y gestiona la comunicación bidireccional con el Arduino Supervisor mediante protocolo digital de 2 bits.

- **Capa de Supervisión (Arduino Supervisor):** Implementada en el archivo `supervisor/supervisor.ino`, esta capa coordina todos los componentes del sistema, lee los estados del robot mediante pines digitales DO2/DO1, gestiona la lectura del sensor inductivo, implementa la lógica de latching de detección de metal, y controla el semáforo perimetral mediante comunicación I2C.

- **Capa de Actuación (Arduino Periférico):** Implementada en el archivo `periferico/periferico.ino`, esta capa se especializa exclusivamente en el control físico de los indicadores LED del semáforo perimetral, implementa el parpadeo no bloqueante del LED amarillo, y gestiona los estados mutuamente excluyentes de los tres LEDs.

**Comunicaciones:** El sistema utiliza dos protocolos de comunicación principales: protocolo digital bidireccional de 2 bits para la comunicación entre el robot y el supervisor (DO2/DO1 para estados del robot, DI2/DI1 para resultados del sensor), y bus I2C para la comunicación entre el supervisor y el periférico (dirección esclavo 0x32, comandos LED codificados en un byte).

**Imagen de la Implementación:**

A continuación se presenta la fotografía de la maqueta implementada mostrando la disposición física completa de todos los componentes de la celda robotizada:

[IMAGEN: Layout completo de la maqueta - Vista superior mostrando robot Ned2, alimentador, zona de inspección, pallet de 3 slots, área de descartes y semáforo perimetral]

**Evidencia Fotográfica de Operaciones:**

[IMAGEN: Recogida de pieza - Robot Ned2 en posición de alimentador con gripper cerrado sobre pieza]

[IMAGEN: Estado en inspección - Robot Ned2 manteniendo pieza sobre zona de inspección con semáforo perimetral en amarillo fijo]

[IMAGEN: Dejado en bandeja - Robot Ned2 depositando pieza en slot del pallet con semáforo perimetral en verde]

[IMAGEN: Dejado en reciclaje - Robot Ned2 depositando pieza en área de descartes con semáforo perimetral en rojo]

## 4. Componentes

### 4.1 Robot Ned2

#### Especificación Formal del Autómata

El sistema del robot Ned2 se modela como un autómata finito determinista (AFD) que gestiona las transiciones entre los diferentes estados operativos durante el ciclo de clasificación. La especificación formal se define mediante la tupla matemática $(E, X, f, x_0, X_m)$ donde:

**Conjunto de Estados ($X$):**

El autómata del robot Ned2 posee cuatro estados principales que representan las diferentes fases operativas:

- $x_0$: **INICIANDO** - Estado inicial de arranque del sistema. En este estado el robot se encuentra en proceso de inicialización, estableciendo conexión TCP/IP con el controlador Python, ejecutando calibración automática de sus ejes, y configurando los parámetros operativos iniciales. El robot permanece en posición HOME y comunica su estado al Arduino Supervisor mediante señales digitales DO2/DO1 = 00 (binario).

- $x_1$: **DISPONIBLE** - Estado seguro donde el robot se encuentra listo para recibir nuevas piezas. El robot permanece en posición HOME, el gripper se encuentra abierto, y el sistema está esperando la detección de una nueva pieza en el alimentador mediante el sensor digital DI5. El estado se comunica al supervisor mediante DO2/DO1 = 01 (binario). Este es el único estado marcado del autómata, representando el estado seguro final donde el sistema puede permanecer indefinidamente sin riesgo operativo.

- $x_2$: **OCUPADO** - Estado donde el robot se encuentra ejecutando movimientos o manipulaciones activas. Incluye las fases de recogida de pieza del alimentador, movimiento hacia la zona de inspección, movimiento hacia el área de descartes o pallet, y deposición de piezas. El estado se comunica mediante DO2/DO1 = 10 (binario).

- $x_3$: **EN_INSPECCIÓN** - Estado crítico donde el robot mantiene la pieza estática sobre la zona de inspección durante exactamente 7 segundos mientras el sensor inductivo realiza la detección de materiales ferrosos. Durante este estado el robot no ejecuta movimientos, permanece completamente estático, y el sistema implementa lógica de latching para garantizar que cualquier detección de metal durante la ventana temporal se mantenga permanente. El estado se comunica mediante DO2/DO1 = 11 (binario).

**Conjunto de Eventos ($E$):**

Los eventos que provocan transiciones entre estados se definen como:

- $e_1$: **Pieza detectada en alimentador** - Señal digital DI5 activa (nivel LOW) confirmada mediante algoritmo de debouncing que requiere al menos 2 de 3 lecturas consecutivas para validar la detección. Este evento se verifica mediante la función `hay_pieza_entrada()` que implementa el debouncing con intervalo de 50 milisegundos entre lecturas.

- $e_2$: **Espacio disponible en pallet** - Verificación interna mediante la función `obtener_slot_libre()` que retorna el índice del primer slot disponible (0, 1, o 2) o -1 si el pallet está completamente lleno. Este evento se evalúa antes de iniciar el ciclo de recogida para evitar operaciones innecesarias cuando no hay capacidad de almacenamiento.

- $e_3$: **Inspección completada** - Timer de 7 segundos expirado. Este evento se genera internamente cuando la función `inspeccionar_pieza()` completa la ventana temporal de inspección. El timer se implementa mediante comparación de `time.time()` con el tiempo de inicio de la inspección, garantizando exactamente 7 segundos de duración independientemente del número de lecturas del sensor realizadas.

- $e_4$: **Comando de parada de emergencia** - Evento de seguridad que puede ser activado manualmente mediante interrupción del programa (Ctrl+C) o automáticamente ante detección de colisiones físicas. Este evento fuerza la transición del sistema a estado INICIANDO desde cualquier estado actual, garantizando un retorno seguro.

**Función de Transición ($f: X \times E \rightarrow X$):**

La función de transición define las reglas deterministas que gobiernan los cambios de estado:

  $$
  \begin{align}
f(x_0, e_1) &= x_1 \quad \text{(Inicialización completa, sistema listo)} \\
f(x_1, e_1 \land e_2) &= x_2 \quad \text{(Pieza detectada y espacio disponible, inicio de ciclo)} \\
f(x_2, \text{movimiento\_completado}) &= x_3 \quad \text{(Robot en zona de inspección, inicio ventana temporal)} \\
f(x_3, e_3) &= x_2 \quad \text{(Inspección completada, procesamiento resultado)} \\
f(x_2, \text{deposito\_completado}) &= x_1 \quad \text{(Ciclo completado, retorno a disponible)} \\
f(x_i, e_4) &= x_0 \quad \forall i \neq 0 \quad \text{(Parada de emergencia desde cualquier estado)}
  \end{align}
  $$

**Estado Inicial ($x_0$):**

El estado inicial del autómata es $x_0$ (INICIANDO), establecido durante la inicialización de la clase `CeldaOceanix` mediante la llamada a `establecer_estado(self.ESTADO_INICIANDO)` en el método `__init__()`.

**Estados Marcados ($X_m$):**

El conjunto de estados marcados contiene únicamente el estado $x_1$ (DISPONIBLE), representando el único estado seguro final donde el sistema puede permanecer indefinidamente sin riesgo operativo. Todos los ciclos operativos completados retornan al sistema a este estado marcado.

#### Diagrama de Estados del Robot Ned2

El diagrama de estados del autómata del robot Ned2 se representa mediante la siguiente estructura de transiciones:

```
                    e₁ (Inicialización completa)
         ┌─────────────────────────────────────────┐
         │                                         │
         ▼                                         │
    (INICIANDO) x₀                                 │
         │                                         │
         │ e₁ ∧ e₂ (Pieza detectada y espacio)     │
         ▼                                         │
    (DISPONIBLE) x₁ ──────────────────────────────┘
         │                    ▲                      │
         │                    │                      │
         │ e₁ ∧ e₂            │ e₃ (Ciclo completado)│
         ▼                    │                      │
    (OCUPADO) x₂ ─────────────┘                      │
         │                    │                      │
         │ Movimiento         │                      │
         │ completado         │                      │
         ▼                    │                      │
    (EN_INSPECCIÓN) x₃ ───────┘                      │
         │                    │                      │
         │ e₃ (Timer 7s)      │                      │
         ▼                    │                      │
    (OCUPADO) x₂ ─────────────┘                      │
         │                    │                      │
         │ e₄ (Emergencia)    │ e₄ (Emergencia)      │
         └────────────────────┴──────────────────────┘
                              │
                              ▼
                         (INICIANDO) x₀
```

#### Programa Implementado

**Diseño Modular:**

La implementación del controlador del robot Ned2 se estructura mediante la clase `CeldaOceanix` en el archivo `main.py`, organizada en módulos funcionales claramente definidos:

```
┌─────────────────────────────────────────────────────────────┐
│                  CLASE CeldaOceanix                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  MÓDULO DE INICIALIZACIÓN                                  │
│  ├── __init__()                                            │
│  ├── _conectar_con_reintentos()                            │
│  └── _obtener_pose_home_oficial()                          │
│                                                             │
│  MÓDULO DE COMUNICACIÓN                                     │
│  ├── establecer_estado()                                   │
│  ├── leer_comando_arduino()                                │
│  └── hay_pieza_entrada()                                   │
│                                                             │
│  MÓDULO DE MOVIMIENTO                                       │
│  ├── _mover_con_validacion()                               │
│  ├── ir_a_destino_seguro()                                 │
│  └── salir_de_destino_seguro()                              │
│                                                             │
│  MÓDULO DE OPERACIONES                                      │
│  ├── recoger_pieza()                                       │
│  ├── inspeccionar_pieza()                                  │
│  └── depositar_en_pallet()                                 │
│                                                             │
│  MÓDULO DE GESTIÓN DE PALLET                               │
│  ├── gestionar_pallet_lleno()                              │
│  ├── obtener_slot_libre()                                  │
│  ├── _cargar_memoria()                                     │
│  └── _guardar_memoria()                                    │
│                                                             │
│  MÓDULO DE CICLO PRINCIPAL                                  │
│  ├── ejecutar_ciclo()                                      │
│  └── ejecutar_ciclo_bandeja()                              │
│                                                             │
│  MÓDULO DE FEEDBACK                                        │
│  ├── configurar_led()                                      │
│  ├── apagar_led()                                          │
│  └── reproducir_sonido()                                   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Descripción de Rutinas:**

**Método `__init__(self, ip_address, max_reintentos_conexion=3)`:**
- **Objetivo:** Inicializar el controlador de la celda Oceanix con configuración completa del sistema.
- **Parámetros:** `ip_address` (dirección IP del robot Ned2 en la red industrial), `max_reintentos_conexion` (número máximo de intentos de conexión, por defecto 3).
- **Funcionalidad:** Establece la conexión TCP/IP con el robot Ned2 mediante la librería pyniryo, ejecuta calibración automática del robot, inicializa el sistema de logging dual (archivo y consola), configura los pines de comunicación digital (DO2/DO1 para estados, DI2/DI1 para resultados), carga el estado persistente del pallet desde archivo JSON, define las coordenadas cartesianas de todas las posiciones de trabajo, e inicializa el sistema de voz industrial. Implementa reconexión automática con backoff exponencial para robustez industrial.
- **Estados iniciales:** Establece el estado del robot en INICIANDO (0) y carga la memoria persistente del pallet.

**Método `_conectar_con_reintentos(self, ip_address, max_reintentos=3)`:**
- **Objetivo:** Establecer conexión robusta con el robot mediante TCP/IP con mecanismo de recuperación automática.
- **Algoritmo:** Implementa bucle de reintentos con backoff exponencial (tiempos de espera: 1s, 2s, 4s). En cada intento crea instancia de `NiryoRobot`, ejecuta `calibrate_auto()`, e inicializa el sistema de voz. Si todos los intentos fallan, termina el programa con código de error.
- **Robustez:** Captura todas las excepciones de conexión y proporciona logging detallado de cada intento.

**Método `establecer_estado(self, estado_decimal)`:**
- **Objetivo:** Comunicar el estado actual del robot al Arduino Supervisor mediante protocolo digital de 2 bits.
- **Parámetros:** `estado_decimal` (valor entero 0-3 representando el estado del autómata).
- **Implementación:** Codifica el estado decimal en dos bits: MSB (DO2) y LSB (DO1). Utiliza `robot.digital_write()` para establecer los niveles HIGH/LOW en los pines correspondientes. Registra el estado enviado en el sistema de logging para trazabilidad.
- **Protocolo:** Estados codificados como: 0 (00 binario) = INICIANDO, 1 (01 binario) = DISPONIBLE, 2 (10 binario) = OCUPADO, 3 (11 binario) = EN_INSPECCIÓN.

**Método `leer_comando_arduino(self)`:**
- **Objetivo:** Leer el resultado de la inspección del sensor inductivo desde el Arduino Supervisor mediante pines digitales DI2/DI1.
- **Retorno:** Valor entero 0-3 representando el código de resultado: 0 = SIN_COMANDO, 1 = NO_METAL, 3 = METAL.
- **Implementación:** Lee los estados de los pines DI2 (MSB) y DI1 (LSB), convierte los niveles HIGH/LOW a bits binarios, y combina los bits mediante operación OR bitwise para obtener el valor decimal. Maneja errores de lectura retornando código seguro SIN_COMANDO.

**Método `hay_pieza_entrada(self)`:**
- **Objetivo:** Verificar la presencia de pieza en el alimentador mediante sensor digital DI5 con algoritmo de debouncing.
- **Algoritmo:** Realiza 3 lecturas consecutivas del pin DI5 con intervalo de 50 milisegundos entre lecturas. El sensor está activo en nivel LOW. Se requiere al menos 2 de 3 lecturas confirmadas para validar la detección, eliminando falsos positivos por ruido eléctrico o vibraciones mecánicas.
- **Retorno:** Valor booleano True si se detecta pieza con confirmación suficiente, False en caso contrario.

**Método `recoger_pieza(self)`:**
- **Objetivo:** Ejecutar la operación de Pick (recogida) de pieza desde el alimentador.
- **Secuencia operativa:** Abre el gripper mediante `robot.release_with_tool()`, ejecuta movimiento seguro hacia el alimentador mediante `ir_a_destino_seguro()` con poses de aproximación, cierra el gripper mediante `robot.grasp_with_tool()` con tiempo de espera de 500ms para asegurar agarre, y valida que la operación se completó exitosamente.
- **Manejo de errores:** En caso de fallo, intenta cerrar el gripper como medida de seguridad y propaga la excepción para manejo en nivel superior.

**Método `inspeccionar_pieza(self)`:**
- **Objetivo:** Ejecutar la ventana de inspección de calidad de exactamente 7 segundos con lógica de latching de detección de metal.
- **Algoritmo de latching:** Inicializa variable `metal_detectado = False`. Durante la ventana temporal de 7 segundos, realiza polling del sensor cada 100ms (70 lecturas totales) mediante `leer_comando_arduino()`. Si alguna lectura retorna código METAL (3), activa el latch estableciendo `metal_detectado = True` de manera permanente e irreversible durante el resto de la inspección. El latch garantiza que una vez detectado metal, el resultado final será siempre METAL independientemente de lecturas posteriores.
- **Comunicación:** Establece estado EN_INSPECCIÓN (11 binario) al inicio, configura LED del robot en amarillo, reproduce sonido "connected.wav", y al finalizar retorna a estado OCUPADO.
- **Retorno:** Valor booleano True si se detectó metal al menos una vez (latch activado), False si todas las lecturas fueron NO_METAL.

**Método `depositar_en_pallet(self, slot_idx)`:**
- **Objetivo:** Depositar pieza aceptada en el slot especificado del pallet de 3 posiciones.
- **Parámetros:** `slot_idx` (índice del slot: 0, 1, o 2).
- **Secuencia operativa:** Ejecuta movimiento seguro hacia el pallet mediante poses de aproximación, deposita la pieza en la posición del slot mediante `robot.release_with_tool()`, actualiza el estado del pallet en memoria marcando el slot como ocupado, guarda el estado en archivo JSON para persistencia, y retira el robot de la zona de trabajo.
- **Gestión de memoria:** Actualiza la lista `estado_pallet[slot_idx] = True` y guarda el estado completo mediante `_guardar_memoria()`.

**Método `ejecutar_ciclo(self)`:**
- **Objetivo:** Orquestar el ciclo completo de clasificación automatizada desde la detección inicial hasta la clasificación final.
- **Fases del ciclo:** FASE 0 (DISPONIBLE) - robot en HOME esperando pieza, FASE 1 (DETECCIÓN) - verificación de sensor con debouncing y validación de espacio en pallet, FASE 2 (RECOGIDA) - operación Pick desde alimentador, FASE 3 (INSPECCIÓN) - ventana de 7 segundos con latching, FASE 4 (CLASIFICACIÓN) - evaluación de resultado y decisión de ruta, FASE 5 (DEPOSITO) - operación Place en descartes o pallet según resultado, FASE 6 (RESET) - retorno a HOME y estado DISPONIBLE.
- **Manejo de errores:** Captura todas las excepciones, ejecuta cleanup seguro (apagar LEDs, mover a HOME, establecer estado DISPONIBLE), y registra traza completa del error con stack trace.

**Método `_mover_con_validacion(self, pose)`:**
- **Objetivo:** Ejecutar movimiento del robot con manejo robusto de colisiones físicas.
- **Parámetros:** `pose` (lista de 6 valores [x, y, z, roll, pitch, yaw] o objeto PoseObject).
- **Algoritmo de recuperación:** Antes de cada movimiento, intenta limpiar colisiones previas mediante `robot.clear_collision_detected()` si está disponible. Ejecuta el movimiento mediante `robot.move()`. Si se detecta colisión, limpia el flag de colisión y propaga excepción con mensaje descriptivo.
- **Robustez:** Permite recuperación automática de colisiones menores sin requerir intervención manual.

**Método `ir_a_destino_seguro(self, pose_destino, pose_aprox_destino)`:**
- **Objetivo:** Implementar navegación segura mediante patrón hub-and-spoke evitando colisiones.
- **Estrategia:** Todos los movimientos pasan por punto de aproximación general compartido (`POSE_APROX_GENERAL`) antes de acceder a destinos específicos. Secuencia: HOME → APROX_GENERAL → APROX_DESTINO → DESTINO_FINAL.
- **Ventaja:** Minimiza riesgo de colisiones al mantener trayectorias predefinidas y puntos de paso seguros.

**Método `gestionar_pallet_lleno(self)`:**
- **Objetivo:** Gestionar situación cuando el pallet alcanza su capacidad máxima (3/3 slots ocupados).
- **Funcionalidad:** Detecta cuando `all(self.estado_pallet)` es True, muestra advertencia visual y auditiva al operador, solicita confirmación manual mediante entrada de texto ("SI" para confirmar vaciado), y reinicia el estado del pallet cuando se confirma el vaciado.
- **Persistencia:** Guarda el estado reiniciado en archivo JSON para mantener consistencia entre sesiones.

### 4.2 Inspección

#### Especificación Formal del Autómata de Inspección

El módulo de inspección se modela como un autómata finito determinista que gestiona el proceso de verificación de calidad mediante detección de materiales ferrosos. La especificación formal se define mediante la tupla $(E, X, f, x_0, X_m)$:

**Conjunto de Estados ($X$):**

El autómata de inspección posee tres estados principales:

- $x_0$: **ESPERANDO** - Estado inicial donde no hay actividad de inspección. El sistema se encuentra en espera de que el robot entre en la zona de inspección. El sensor inductivo puede estar leyendo continuamente, pero sus resultados no se procesan hasta que se active la ventana temporal de inspección. Este es el estado marcado del autómata, representando el estado seguro donde el sistema puede permanecer indefinidamente.

- $x_1$: **INSPECCIONANDO** - Estado activo donde se ejecuta la ventana temporal de inspección de exactamente 7 segundos. El robot mantiene la pieza estática sobre el sensor inductivo, y el sistema realiza lectura continua del sensor con frecuencia de 10 Hz (una lectura cada 100 milisegundos, totalizando 70 lecturas durante la ventana completa). Durante este estado se implementa la lógica de latching: si se detecta metal al menos una vez, el resultado queda permanentemente marcado como METAL independientemente de lecturas posteriores. El estado se activa cuando el robot comunica estado EN_INSPECCIÓN mediante DO2/DO1 = 11 (binario).

- $x_2$: **CLASIFICANDO** - Estado de procesamiento donde se evalúa el resultado final de la inspección basado en el latch activado durante el estado INSPECCIONANDO. El resultado se comunica al robot mediante pines DI2/DI1 (01 para NO_METAL, 11 para METAL), y se actualiza el semáforo perimetral según el resultado (verde para NO_METAL, rojo para METAL). Este estado es transitorio y finaliza cuando el robot sale de la zona de inspección.

**Conjunto de Eventos ($E$):**

Los eventos que provocan transiciones entre estados se definen como:

- $e_1$: **Robot entra en zona de inspección** - Transición de estado del robot detectada mediante lectura de pines DO2/DO1 = 11 (binario, decimal 3). Este evento se detecta en el Arduino Supervisor mediante la función `leerEstadoRobot()` cuando el estado actual es ROBOT_EN_INSPECCION y el estado anterior era diferente. La detección de este evento activa la ventana temporal de 7 segundos y cambia el semáforo perimetral a amarillo fijo.

- $e_2$: **Detección de metal** - Lectura del sensor inductivo en PIN 7 del Arduino Supervisor que indica presencia de material ferroso. El sensor está activo en nivel LOW, por lo que `digitalRead(PIN_SENSOR_METAL) == LOW` indica detección. Este evento puede ocurrir múltiples veces durante el estado INSPECCIONANDO, pero la primera ocurrencia activa el latch de manera permanente. El evento se procesa continuamente durante la ventana temporal mediante polling cada 100ms.

- $e_3$: **Timer de 7 segundos completado** - Evento discreto interno generado cuando el tiempo transcurrido desde el inicio de la inspección alcanza exactamente 7 segundos. El timer se implementa mediante comparación de `time.time()` (Python) o `millis()` (Arduino) con el tiempo de inicio almacenado. Este evento marca el fin de la ventana temporal y permite la transición al estado de clasificación.

- $e_4$: **Robot sale de zona de inspección** - Transición de estado del robot detectada mediante lectura de pines DO2/DO1 ≠ 11 (binario). Este evento indica que el robot ha completado la inspección y se está moviendo hacia la zona de clasificación (descartes o pallet). La detección de este evento permite la transición del estado CLASIFICANDO de vuelta al estado ESPERANDO, completando el ciclo de inspección.

**Función de Transición ($f: X \times E \rightarrow X$):**

La función de transición define las reglas deterministas del autómata:

  $$
  \begin{align}
f(x_0, e_1) &= x_1 \quad \text{(Robot entra en zona, inicio ventana temporal)} \\
f(x_1, e_2) &= x_1 \quad \text{(Metal detectado, latch activado, mantener estado)} \\
f(x_1, e_3) &= x_2 \quad \text{(Timer expirado, procesar resultado final)} \\
f(x_2, e_4) &= x_0 \quad \text{(Clasificación completada, retorno a espera)}
  \end{align}
  $$

**Estado Inicial ($x_0$):**

El estado inicial del autómata de inspección es $x_0$ (ESPERANDO), establecido durante la inicialización del Arduino Supervisor.

**Estados Marcados ($X_m$):**

El conjunto de estados marcados contiene únicamente el estado $x_0$ (ESPERANDO), representando el estado seguro donde el sistema puede permanecer indefinidamente sin actividad de inspección.

**Función de Salida (Latching):**

Aunque no forma parte de la tupla estándar del autómata, el módulo de inspección implementa una función de salida especial mediante lógica de latching. La variable `metalDetectado` (en Arduino Supervisor) o `metal_detectado` (en Python) actúa como latch que se activa cuando $e_2$ ocurre por primera vez durante el estado $x_1$, y permanece activo durante todo el ciclo de inspección independientemente de lecturas posteriores del sensor. Esta función garantiza que el principio "una vez metal, siempre metal" se mantenga durante toda la ventana temporal.

#### Diagrama de Estados del Módulo de Inspección

El diagrama de estados del autómata de inspección se representa mediante la siguiente estructura:

```
                    e₁ (Robot entra en inspección)
         ┌─────────────────────────────────────────┐
         │                                         │
         ▼                                         │
    (ESPERANDO) x₀                                 │
         │                                         │
         │ e₁                                      │
         ▼                                         │
    (INSPECCIONANDO) x₁                            │
         │                                         │
         │ e₂ (Metal detectado, latch activado)    │
         │    (mantiene estado x₁)                 │
         │                                         │
         │ e₃ (Timer 7s completado)               │
         ▼                                         │
    (CLASIFICANDO) x₂                              │
         │                                         │
         │ e₄ (Robot sale de inspección)          │
         └─────────────────────────────────────────┘
```

#### Programa Implementado

**Diseño Modular:**

El módulo de inspección se implementa mediante coordinación entre dos componentes principales:

```
┌─────────────────────────────────────────────────────────────┐
│              MÓDULO DE INSPECCIÓN                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  COMPONENTE PYTHON (main.py)                               │
│  ├── inspeccionar_pieza()                                  │
│  │   ├── Movimiento a zona de inspección                   │
│  │   ├── Establecimiento estado EN_INSPECCIÓN              │
│  │   ├── Ventana temporal 7 segundos                       │
│  │   ├── Polling sensor cada 100ms                        │
│  │   ├── Lógica de latching                                │
│  │   └── Retorno resultado                                 │
│  └── leer_comando_arduino()                                │
│      └── Lectura DI2/DI1 del supervisor                   │
│                                                             │
│  COMPONENTE ARDUINO SUPERVISOR (supervisor.ino)            │
│  ├── procesarMaquinaEstados()                              │
│  │   └── Estado SUP_INSPECCIONANDO                        │
│  │       ├── Lectura sensor PIN 7                         │
│  │       ├── Activación latch metalDetectado               │
│  │       └── Publicación resultado DI2/DI1                 │
│  └── leerEstadoRobot()                                     │
│      └── Detección transición a EN_INSPECCIÓN             │
│                                                             │
│  SENSOR INDUCTIVO (Hardware)                               │
│  └── PIN 7 Arduino Supervisor                              │
│      └── Activo en LOW cuando detecta metal               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Descripción de Rutinas:**

**Función `inspeccionar_pieza(self)` (Python, main.py):**

- **Objetivo:** Ejecutar la ventana completa de inspección de calidad con lógica de latching de detección de metal.

- **Parámetros:** Ninguno (método de instancia que accede a atributos de la clase).

- **Algoritmo de Latching:** Inicializa variable local `metal_detectado = False` al inicio de la función. Durante la ventana temporal de exactamente 7 segundos, ejecuta bucle while que compara `time.time() - tiempo_inicio` con `self.TIEMPO_INSPECCION` (7 segundos). En cada iteración del bucle (cada 100ms), llama a `self.leer_comando_arduino()` para obtener el resultado del sensor desde el Arduino Supervisor. Si el comando retornado es `self.CMD_METAL` (valor 3), establece `metal_detectado = True` de manera permanente. Una vez activado, el latch no puede desactivarse durante el resto de la ventana temporal, garantizando que cualquier detección de metal resulte en clasificación final como METAL.

- **Secuencia Operativa:** 
  1. Ejecuta movimiento seguro hacia zona de inspección mediante `ir_a_destino_seguro(self.POSE_INSPECCION, self.APROX_INSPECCION)`.
  2. Establece estado EN_INSPECCIÓN mediante `establecer_estado(self.ESTADO_EN_INSPECCION)`, comunicando DO2/DO1 = 11 al supervisor.
  3. Configura feedback visual (LED amarillo del robot) y auditivo (sonido "connected.wav").
  4. Inicia ventana temporal de 7 segundos con polling continuo del sensor.
  5. Al completar la ventana, guarda el resultado en `self.resultado_inspeccion` para uso posterior en clasificación.
  6. Cambia estado a OCUPADO para indicar fin de inspección.
  7. Retorna valor booleano True si metal fue detectado, False en caso contrario.

- **Comunicación con Supervisor:** La función no lee directamente el sensor inductivo, sino que lee el resultado procesado por el Arduino Supervisor mediante los pines DI2/DI1. El supervisor lee el sensor continuamente y publica el resultado en estos pines, permitiendo que el robot Python obtenga el estado del sensor de manera síncrona.

- **Manejo de Errores:** Captura todas las excepciones durante la inspección, establece estado seguro OCUPADO, inicializa `resultado_inspeccion = False` como valor por defecto seguro, y registra error completo en sistema de logging.

**Función `leer_comando_arduino(self)` (Python, main.py):**

- **Objetivo:** Leer el resultado de la inspección del sensor inductivo desde el Arduino Supervisor mediante pines digitales DI2/DI1.

- **Implementación:** Utiliza `self.robot.digital_read()` para leer los estados de los pines DI2 (MSB) y DI1 (LSB). Convierte los niveles HIGH/LOW a valores binarios (1 o 0), y combina los bits mediante operación bitwise: `(bit_msb << 1) | bit_lsb` para obtener el valor decimal completo.

- **Códigos de Retorno:** 0 = SIN_COMANDO (sin resultado aún), 1 = NO_METAL (pieza no conductiva, será paletizada), 3 = METAL (pieza conductiva detectada, será descartada).

- **Frecuencia de Lectura:** Esta función se llama repetidamente durante la ventana de inspección con intervalo de 100ms entre llamadas, resultando en frecuencia de muestreo de 10 Hz.

**Función `procesarMaquinaEstados(bool sensorMetal)` (Arduino, supervisor.ino):**

- **Objetivo:** Implementar la máquina de estados finita del supervisor que coordina el proceso de inspección.

- **Parámetros:** `sensorMetal` (valor booleano obtenido de `digitalRead(PIN_SENSOR_METAL) == LOW`, True indica detección de metal).

- **Estado SUP_INSPECCIONANDO:** Cuando el supervisor está en este estado (activado por transición del robot a EN_INSPECCIÓN), ejecuta las siguientes acciones:
  - Lee continuamente el sensor inductivo mediante el parámetro `sensorMetal`.
  - Si `sensorMetal == true`, publica resultado METAL en pines DI2/DI1 mediante `enviarResultadoARobot(RESULT_METAL)`, y activa el latch `metalDetectado = true` si aún no estaba activado.
  - Si `sensorMetal == false`, publica resultado NO_METAL en pines DI2/DI1 mediante `enviarResultadoARobot(RESULT_NO_METAL)`.
  - Detecta cuando el robot sale de inspección mediante comparación de `estadoRobotActual != ROBOT_EN_INSPECCION` y `estadoRobotAnterior == ROBOT_EN_INSPECCION`.
  - Al detectar salida del robot, transiciona al estado SUP_MOSTRANDO_RESULTADO y actualiza el semáforo según el latch `metalDetectado`.

- **Latching en Arduino:** El latch `metalDetectado` se implementa como variable booleana global que se inicializa en `false` al entrar en estado SUP_INSPECCIONANDO, se activa a `true` cuando se detecta metal por primera vez, y permanece activo durante todo el ciclo de inspección hasta que se completa la clasificación.

**Función `leerEstadoRobot()` (Arduino, supervisor.ino):**

- **Objetivo:** Leer el estado actual del robot mediante pines digitales DO2/DO1.

- **Implementación:** Lee los niveles digitales de PIN_ROBOT_DO2 (pin 3) y PIN_ROBOT_DO1 (pin 2), convierte HIGH/LOW a bits binarios, y combina mediante operación bitwise para obtener valor decimal 0-3.

- **Uso:** Esta función se llama en cada iteración del `loop()` principal del supervisor para detectar transiciones de estado del robot que activan eventos $e_1$ y $e_4$ del autómata de inspección.

### 4.3 Supervisor de la Celda

#### Descripción del Supervisor

El supervisor Arduino actúa como el cerebro central del sistema de clasificación Oceanix, implementando una máquina de estados finita determinista que coordina todos los componentes de la celda robotizada. Esta arquitectura de supervisión centralizada permite una separación clara de responsabilidades: el supervisor decide cuándo el robot puede actuar basándose en los estados comunicados, coordina la lectura del sensor inductivo y la lógica de latching, gestiona el semáforo perimetral mediante comunicación I2C con el periférico, y garantiza la integridad del protocolo de comunicación bidireccional entre el robot y los sensores.

El supervisor se implementa mediante un Arduino Uno (o compatible) ejecutando el firmware contenido en el archivo `supervisor/supervisor.ino`. El sistema opera en un bucle principal (`loop()`) que ejecuta lectura continua de los estados del robot, procesamiento del sensor inductivo, evaluación de la máquina de estados finita, y comunicación I2C con el periférico para control del semáforo.

#### Especificación Formal del Autómata Supervisor

El supervisor se modela como un autómata finito determinista con función de salida, definido mediante la tupla extendida $(E, X, f, x_0, X_m, \lambda)$ donde $\lambda$ representa la función de salida que controla el semáforo perimetral:

**Conjunto de Estados ($X$):**

El autómata del supervisor posee tres estados principales:

- $q_0$: **SUP_IDLE** - Estado inicial y seguro donde el supervisor espera que el robot entre en la zona de inspección. Durante este estado, el supervisor publica continuamente el estado actual del sensor inductivo hacia el robot mediante pines DI2/DI1, permitiendo que el robot tenga información actualizada incluso antes de iniciar la inspección. El semáforo perimetral se encuentra en modo amarillo titilante (comando I2C YELLOW_BLINK) con frecuencia de parpadeo de 400 milisegundos, indicando visualmente que el sistema está disponible pero no hay actividad de inspección en curso. Este es el único estado marcado del autómata, representando el estado seguro donde el sistema puede permanecer indefinidamente.

- $q_1$: **SUP_INSPECCIONANDO** - Estado activo donde el robot se encuentra ejecutando la ventana temporal de inspección de 7 segundos. El supervisor detecta este estado mediante la transición del robot a estado EN_INSPECCIÓN (DO2/DO1 = 11). Durante este estado, el supervisor lee continuamente el sensor inductivo en PIN 7, implementa la lógica de latching mediante la variable `metalDetectado`, y publica el resultado del sensor hacia el robot mediante pines DI2/DI1 con frecuencia de actualización determinada por el ciclo del `loop()` (aproximadamente cada 50ms). El semáforo perimetral se cambia a modo amarillo fijo (comando I2C YELLOW_SOLID) para indicar visualmente que la inspección está en curso. El latch `metalDetectado` se activa cuando se detecta metal por primera vez y permanece activo durante todo el estado, garantizando que el resultado final refleje cualquier detección ocurrida durante la ventana temporal.

- $q_2$: **SUP_MOSTRANDO_RESULTADO** - Estado transitorio donde el supervisor muestra el resultado de la inspección mediante el semáforo perimetral durante exactamente 10 segundos. El supervisor detecta la transición a este estado cuando el robot sale del estado EN_INSPECCIÓN (DO2/DO1 ≠ 11). El semáforo se actualiza según el latch `metalDetectado`: si el latch está activo, se envía comando I2C LAMP_RED (LED rojo fijo), indicando que la pieza será descartada; si el latch está inactivo, se envía comando I2C LAMP_GREEN (LED verde fijo), indicando que la pieza será paletizada. El supervisor mantiene la publicación del resultado hacia el robot mediante DI2/DI1 durante este estado. Un timer interno basado en `millis()` controla la duración de 10 segundos, transicionando automáticamente al estado SUP_IDLE cuando el tiempo expira.

**Conjunto de Eventos ($E = \{e_1, e_2, e_3, e_4\}$):**

Los eventos que provocan transiciones entre estados se definen formalmente como:

- $e_1$: **Robot entra en inspección** - Transición del estado del robot detectada mediante lectura de pines DO2/DO1 = 11 (binario, decimal 3, estado ROBOT_EN_INSPECCION). Este evento se detecta en la función `procesarMaquinaEstados()` cuando se cumple la condición `estadoRobotActual == ROBOT_EN_INSPECCION && estadoRobotAnterior != ROBOT_EN_INSPECCION`, indicando una transición desde cualquier otro estado hacia el estado de inspección. La detección de este evento activa el estado SUP_INSPECCIONANDO, reinicializa el latch `metalDetectado = false`, y cambia el semáforo a amarillo fijo.

- $e_2$: **Detección de metal** - Lectura del sensor inductivo en PIN 7 del Arduino Supervisor que indica presencia de material ferroso. El sensor está configurado con `INPUT_PULLUP` y está activo en nivel LOW, por lo que `digitalRead(PIN_SENSOR_METAL) == LOW` indica detección de metal. Este evento puede ocurrir múltiples veces durante el estado SUP_INSPECCIONANDO, pero la primera ocurrencia activa el latch `metalDetectado = true` de manera permanente. El evento se procesa continuamente durante el estado mediante lectura en cada iteración del `loop()` principal.

- $e_3$: **Robot sale de inspección** - Transición del estado del robot detectada mediante lectura de pines DO2/DO1 ≠ 11 (binario). Este evento indica que el robot ha completado la ventana temporal de inspección y se está moviendo hacia la zona de clasificación (descartes o pallet). La detección se realiza cuando se cumple `estadoRobotActual != ROBOT_EN_INSPECCION && estadoRobotAnterior == ROBOT_EN_INSPECCION`, indicando una transición desde el estado de inspección hacia cualquier otro estado. La detección de este evento activa el estado SUP_MOSTRANDO_RESULTADO, inicializa el timer de 10 segundos mediante `tiempoInicioResultado = millis()`, y actualiza el semáforo según el latch `metalDetectado`.

- $e_4$: **Timer de 10 segundos expirado** - Evento discreto interno generado cuando el tiempo transcurrido desde el inicio del estado SUP_MOSTRANDO_RESULTADO alcanza exactamente 10 segundos. El timer se implementa mediante comparación de `millis() - tiempoInicioResultado` con la constante `DURACION_RESULTADO_MS` (10000 milisegundos). Este evento marca el fin del período de visualización del resultado y permite la transición de vuelta al estado SUP_IDLE, completando el ciclo de supervisión.

**Función de Transición ($\delta: X \times E \rightarrow X$):**

La función de transición define las reglas deterministas del autómata:

$$
\begin{align}
\delta(q_0, e_1) &= q_1 \quad \text{(Robot entra en inspección, activación estado inspección)} \\
\delta(q_1, e_2) &= q_1 \quad \text{(Metal detectado, latch activado, mantener estado)} \\
\delta(q_1, e_3) &= q_2 \quad \text{(Robot sale de inspección, mostrar resultado)} \\
\delta(q_2, e_4) &= q_0 \quad \text{(Timer expirado, retorno a estado idle)}
\end{align}
$$

**Función de Salida ($\lambda: X \rightarrow Y$):**

La función de salida controla el semáforo perimetral según el estado actual:

  $$
  \begin{align}
\lambda(q_0) &= \text{YELLOW\_BLINK} \quad \text{(Amarillo titilante, sistema disponible)} \\
\lambda(q_1) &= \text{YELLOW\_SOLID} \quad \text{(Amarillo fijo, inspección en curso)} \\
\lambda(q_2) &= \begin{cases}
\text{LAMP\_RED} & \text{si } \text{metalDetectado} = \text{true} \\
\text{LAMP\_GREEN} & \text{si } \text{metalDetectado} = \text{false}
\end{cases} \quad \text{(Resultado según latch)}
  \end{align}
  $$

**Estado Inicial ($q_0$):**

El estado inicial del autómata supervisor es $q_0$ (SUP_IDLE), establecido durante la inicialización del Arduino mediante la asignación `estadoSupervisor = SUP_IDLE` en la función `setup()`.

**Estados Marcados ($X_m$):**

El conjunto de estados marcados contiene únicamente el estado $q_0$ (SUP_IDLE), representando el único estado seguro donde el sistema puede permanecer indefinidamente sin actividad de inspección.

#### Diagrama de Estados del Supervisor

El diagrama de estados del autómata supervisor se representa mediante la siguiente estructura con función de salida:

```
                    e₁ (Robot entra en inspección)
         ┌─────────────────────────────────────────┐
         │                                         │
         ▼                                         │
    (SUP_IDLE) q₀                                  │
    λ: YELLOW_BLINK                                │
         │                                         │
         │ e₁                                      │
         ▼                                         │
    (SUP_INSPECCIONANDO) q₁                        │
    λ: YELLOW_SOLID                                │
         │                                         │
         │ e₂ (Metal detectado, latch)            │
         │    (mantiene estado q₁)                 │
         │                                         │
         │ e₃ (Robot sale de inspección)          │
         ▼                                         │
    (SUP_MOSTRANDO_RESULTADO) q₂                   │
    λ: RED (si metal) o GREEN (si no metal)       │
         │                                         │
         │ e₄ (Timer 10s expirado)                │
         └─────────────────────────────────────────┘
```

#### Programa Implementado

**Diseño Modular:**

La implementación del supervisor Arduino se estructura mediante el archivo `supervisor/supervisor.ino`, organizado en módulos funcionales:

```
┌─────────────────────────────────────────────────────────────┐
│              ARDUINO SUPERVISOR                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  MÓDULO DE INICIALIZACIÓN                                  │
│  ├── setup()                                               │
│  │   ├── Configuración pines digitales                    │
│  │   ├── Inicialización I2C master                        │
│  │   └── Estado inicial SUP_IDLE                          │
│                                                             │
│  MÓDULO DE LECTURA DE ENTRADAS                            │
│  ├── leerEstadoRobot()                                     │
│  │   └── Lectura DO2/DO1 → estado 0-3                    │
│  └── Lectura sensor inductivo PIN 7                       │
│                                                             │
│  MÓDULO DE ESCRITURA DE SALIDAS                           │
│  ├── enviarResultadoARobot()                              │
│  │   └── Escritura DI2/DI1 según resultado               │
│  └── enviarComandoSemaforo()                             │
│      └── Comunicación I2C con periférico                  │
│                                                             │
│  MÓDULO DE MÁQUINA DE ESTADOS                             │
│  ├── procesarMaquinaEstados()                             │
│  │   ├── Estado SUP_IDLE                                  │
│  │   ├── Estado SUP_INSPECCIONANDO                        │
│  │   └── Estado SUP_MOSTRANDO_RESULTADO                   │
│  └── Variables de estado globales                         │
│                                                             │
│  MÓDULO DE DEBUG Y MONITOREO                              │
│  ├── debugPeriodicoEstado()                               │
│  └── nombreEstadoRobot() / nombreEstadoSupervisor()       │
│                                                             │
│  LOOP PRINCIPAL                                            │
│  └── loop()                                                │
│      ├── Lectura estado robot                             │
│      ├── Lectura sensor                                    │
│      ├── Procesamiento máquina estados                    │
│      └── Debug periódico                                  │
│                                                             │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              ARDUINO PERIFÉRICO                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  MÓDULO DE INICIALIZACIÓN                                  │
│  ├── setup()                                               │
│  │   ├── Configuración pines LED                          │
│  │   └── Inicialización I2C esclavo (0x32)                │
│                                                             │
│  MÓDULO DE RECEPCIÓN I2C                                   │
│  ├── callback_i2c()                                        │
│  │   └── Interrupción hardware de recepción               │
│                                                             │
│  MÓDULO DE PROCESAMIENTO DE COMANDOS                      │
│  ├── procesar_comando()                                    │
│  │   └── Switch case con 5 comandos LED                   │
│                                                             │
│  MÓDULO DE PARPADEO NO BLOQUEANTE                          │
│  ├── manejar_parpadeo()                                   │
│  │   └── Timer 400ms para YELLOW_BLINK                    │
│                                                             │
│  LOOP PRINCIPAL                                            │
│  └── loop()                                                │
│      ├── Procesamiento comandos nuevos                    │
│      └── Parpadeo si comando activo                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Descripción de Rutinas:**

**Función `setup()` (Arduino Supervisor, supervisor.ino):**

- **Objetivo:** Configurar todos los componentes hardware y software del Arduino Supervisor durante la inicialización del sistema.

- **Configuración de Pines Digitales:** Configura PIN_ROBOT_DO2 (pin 3) y PIN_ROBOT_DO1 (pin 2) como entradas sin pull-up interno para lectura de estados del robot. Configura PIN_ROBOT_DI2 (pin 12), PIN_ROBOT_DI1 (pin 11), y PIN_ROBOT_DI3 (pin 10) como salidas para envío de resultados al robot. Configura PIN_SENSOR_METAL (pin 7) como entrada con pull-up interno (`INPUT_PULLUP`) para lectura del sensor inductivo.

- **Inicialización I2C:** Inicializa el bus I2C como master mediante `Wire.begin()` sin parámetros, permitiendo que el supervisor actúe como dispositivo controlador del bus.

- **Estado Inicial:** Establece `estadoSupervisor = SUP_IDLE` y `estadoRobotActual = ROBOT_INICIANDO` como valores iniciales. Inicializa el latch `metalDetectado = false`.

- **Semáforo Inicial:** Envía comando inicial `LAMP_YELLOW_BLINK` al periférico mediante `enviarComandoSemaforo()` para indicar que el sistema está disponible.

- **Sistema de Logging:** Inicializa comunicación serial a 9600 baudios para logging de debug y monitoreo del sistema.

**Función `loop()` (Arduino Supervisor, supervisor.ino):**

- **Objetivo:** Ejecutar el ciclo principal de supervisión que se repite continuamente durante la operación del sistema.

- **Secuencia de Ejecución:** En cada iteración del loop (aproximadamente cada 50ms debido al delay final), ejecuta las siguientes operaciones en orden:
  1. Lee el estado actual del robot mediante `leerEstadoRobot()` y almacena en `estadoRobotActual`.
  2. Lee el sensor inductivo mediante `digitalRead(PIN_SENSOR_METAL) == SENSOR_DETECTA` y almacena en variable local `sensorMetal`.
  3. Procesa la máquina de estados mediante `procesarMaquinaEstados(sensorMetal)`, evaluando transiciones y ejecutando acciones según el estado actual.
  4. Ejecuta debug periódico mediante `debugPeriodicoEstado(sensorMetal)` si ha transcurrido el intervalo de tiempo configurado (2 segundos por defecto).
  5. Guarda el estado actual del robot en `estadoRobotAnterior` para detección de transiciones en la siguiente iteración.
  6. Ejecuta `delay(50)` para controlar la frecuencia del loop y evitar saturación del procesador.

**Función `leerEstadoRobot()` (Arduino Supervisor, supervisor.ino):**

- **Objetivo:** Leer el estado actual del robot mediante pines digitales DO2/DO1 y convertirlo a valor enumerado.

- **Implementación:** Lee los niveles digitales de PIN_ROBOT_DO2 (pin 3) y PIN_ROBOT_DO1 (pin 2) mediante `digitalRead()`. Convierte HIGH/LOW a bits binarios (1 o 0), y combina mediante operación bitwise: `((do2 == HIGH ? 1 : 0) << 1) | (do1 == HIGH ? 1 : 0)` para obtener valor decimal 0-3.

- **Retorno:** Valor de tipo `RobotState` (enumerado) con valores posibles: ROBOT_INICIANDO (0), ROBOT_DISPONIBLE (1), ROBOT_OCUPADO (2), ROBOT_EN_INSPECCION (3).

**Función `enviarResultadoARobot(ResultCode resultado)` (Arduino Supervisor, supervisor.ino):**

- **Objetivo:** Enviar el resultado de la inspección del sensor hacia el robot mediante pines digitales DI2/DI1.

- **Parámetros:** `resultado` (valor de tipo `ResultCode` enumerado: RESULT_NONE (0), RESULT_NO_METAL (1), RESULT_RESERVED (2), RESULT_METAL (3)).

- **Implementación:** Extrae los bits MSB y LSB del valor `resultado` mediante operaciones bitwise: `(resultado & 0x02) ? HIGH : LOW` para MSB (DI2), `(resultado & 0x01) ? HIGH : LOW` para LSB (DI1). Escribe los niveles en los pines correspondientes mediante `digitalWrite()`.

- **Frecuencia de Actualización:** Esta función se llama continuamente durante los estados SUP_IDLE y SUP_INSPECCIONANDO para mantener el resultado del sensor actualizado hacia el robot, permitiendo que el robot Python lea el estado del sensor mediante polling durante la ventana de inspección.

**Función `procesarMaquinaEstados(bool sensorMetal)` (Arduino Supervisor, supervisor.ino):**

- **Objetivo:** Implementar la máquina de estados finita principal del supervisor que coordina todas las transiciones y acciones del sistema.

- **Parámetros:** `sensorMetal` (valor booleano True indica detección de metal en el sensor inductivo).

- **Estructura:** Implementa estructura `switch-case` sobre la variable global `estadoSupervisor`, ejecutando lógica específica para cada estado:

  - **Caso SUP_IDLE:** Publica continuamente el estado del sensor hacia el robot mediante `enviarResultadoARobot()`. Detecta transición del robot a estado EN_INSPECCIÓN mediante comparación `estadoRobotActual == ROBOT_EN_INSPECCION && estadoRobotAnterior != ROBOT_EN_INSPECCION`. Al detectar transición, cambia `estadoSupervisor = SUP_INSPECCIONANDO`, reinicializa `metalDetectado = false`, y envía comando `LAMP_YELLOW_SOLID` al semáforo.

  - **Caso SUP_INSPECCIONANDO:** Publica continuamente el estado del sensor hacia el robot. Si `sensorMetal == true`, publica `RESULT_METAL` y activa el latch `metalDetectado = true` si aún no estaba activo (registrando la primera detección en el sistema de logging). Si `sensorMetal == false`, publica `RESULT_NO_METAL`. Detecta transición del robot fuera del estado EN_INSPECCIÓN mediante comparación `estadoRobotActual != ROBOT_EN_INSPECCION && estadoRobotAnterior == ROBOT_EN_INSPECCION`. Al detectar transición, cambia `estadoSupervisor = SUP_MOSTRANDO_RESULTADO`, inicializa `tiempoInicioResultado = millis()`, y actualiza el semáforo según el latch: `LAMP_RED` si `metalDetectado == true`, `LAMP_GREEN` si `metalDetectado == false`.

  - **Caso SUP_MOSTRANDO_RESULTADO:** Mantiene la publicación del último resultado hacia el robot según el latch `metalDetectado`. Evalúa el timer mediante comparación `millis() - tiempoInicioResultado >= DURACION_RESULTADO_MS` (10000ms). Cuando el timer expira, cambia `estadoSupervisor = SUP_IDLE`, reinicializa `metalDetectado = false`, y envía comando `LAMP_YELLOW_BLINK` al semáforo para indicar que el sistema está disponible para el siguiente ciclo.

**Función `enviarComandoSemaforo(LampCommand cmd)` (Arduino Supervisor, supervisor.ino):**

- **Objetivo:** Enviar comando de control del semáforo al Arduino Periférico mediante comunicación I2C.

- **Parámetros:** `cmd` (valor de tipo `LampCommand` enumerado: LAMP_OFF (0), LAMP_YELLOW_BLINK (1), LAMP_YELLOW_SOLID (2), LAMP_RED (3), LAMP_GREEN (4)).

- **Implementación:** Inicia transmisión I2C hacia dirección esclava `SEMAFORO_I2C_ADDR` (0x32) mediante `Wire.beginTransmission()`. Escribe el valor del comando como byte único mediante `Wire.write()`. Finaliza la transmisión mediante `Wire.endTransmission()` y captura código de error retornado (0 indica éxito, otros valores indican errores de comunicación).

- **Manejo de Errores:** Registra en sistema de logging serial si la transmisión fue exitosa o si ocurrió error, facilitando diagnóstico de problemas de comunicación I2C.

**Función `callback_i2c(int bytes)` (Arduino Periférico, periferico.ino):**

- **Objetivo:** Gestionar la recepción de comandos I2C desde el supervisor mediante interrupción hardware.

- **Parámetros:** `bytes` (número de bytes recibidos en la transmisión I2C, proporcionado automáticamente por la librería Wire).

- **Implementación:** Ejecuta bucle `while (Wire.available())` para leer todos los bytes recibidos. Lee cada byte mediante `Wire.read()` y almacena en variable global `comando`. Activa flag `comando_nuevo = true` para indicar al `loop()` principal que hay un comando pendiente de procesamiento.

- **Características:** Esta función se ejecuta como interrupción hardware cuando el supervisor inicia una transmisión I2C hacia la dirección esclava 0x32, garantizando recepción inmediata sin necesidad de polling.

**Función `procesar_comando(uint8_t cmd)` (Arduino Periférico, periferico.ino):**

- **Objetivo:** Ejecutar la acción correspondiente al comando LED recibido mediante I2C.

- **Parámetros:** `cmd` (valor entero 0-4 representando el comando LED).

- **Implementación:** Estructura `switch-case` que evalúa el valor del comando:
  - `LAMP_OFF` (0): Apaga todos los LEDs mediante `apagar_todos()`.
  - `LAMP_YELLOW_BLINK` (1): Apaga todos los LEDs, enciende LED amarillo, inicializa variables de parpadeo (`tiempo_parpadeo = millis()`, `estado_parp = true`).
  - `LAMP_YELLOW_SOLID` (2): Apaga todos los LEDs, enciende LED amarillo en modo fijo.
  - `LAMP_RED` (3): Apaga todos los LEDs, enciende LED rojo.
  - `LAMP_GREEN` (4): Apaga todos los LEDs, enciende LED verde.

- **Garantía de Estados Mutuamente Excluyentes:** Todas las ramas del switch llaman a `apagar_todos()` antes de encender el LED correspondiente, garantizando que solo un LED esté activo en cualquier momento.

**Función `manejar_parpadeo()` (Arduino Periférico, periferico.ino):**

- **Objetivo:** Implementar parpadeo no bloqueante del LED amarillo con frecuencia de 400 milisegundos.

- **Algoritmo:** Compara `millis() - tiempo_parpadeo` con constante `INTERVALO_PARPADEO` (400ms). Si ha transcurrido el intervalo, actualiza `tiempo_parpadeo = millis()`, invierte el estado del parpadeo mediante `estado_parp = !estado_parp`, y escribe el estado al LED amarillo mediante `digitalWrite(PIN_AMARILLO, estado_parp ? HIGH : LOW)`.

- **No Bloqueante:** Esta función se llama desde el `loop()` principal solo cuando el comando activo es `LAMP_YELLOW_BLINK`, y utiliza `millis()` en lugar de `delay()` para no bloquear la recepción de nuevos comandos I2C durante el parpadeo.

## 5. Comunicaciones

### 5.1 Supervisor ↔ Robot Ned2

#### Medio Físico de Comunicación

La comunicación entre el robot Ned2 y el Arduino Supervisor se establece mediante conexiones físicas directas entre el panel GPIO del robot y los pines digitales del microcontrolador Arduino. El protocolo utiliza señales digitales TTL compatibles entre los niveles de voltaje del robot (3.3V) y el Arduino (5V), garantizando compatibilidad eléctrica sin necesidad de circuitos de adaptación de nivel.

**Cableado de Estados (Robot → Supervisor):**
- **DO2 (MSB):** Salida digital del robot Ned2 conectada al pin 3 (D3) del Arduino Supervisor. Este pin se configura como entrada digital sin pull-up interno mediante `pinMode(PIN_ROBOT_DO2, INPUT)`.
- **DO1 (LSB):** Salida digital del robot Ned2 conectada al pin 2 (D2) del Arduino Supervisor. Este pin se configura como entrada digital sin pull-up interno mediante `pinMode(PIN_ROBOT_DO1, INPUT)`.
- **GND:** Conexión de referencia común (tierra) entre el robot y el Arduino, crítica para evitar offsets de potencial y garantizar niveles de señal correctos.

**Cableado de Resultados (Supervisor → Robot):**
- **DI2 (MSB):** Pin 12 (D12) del Arduino Supervisor conectado a entrada digital DI2 del robot Ned2. Este pin se configura como salida digital mediante `pinMode(PIN_ROBOT_DI2, OUTPUT)`.
- **DI1 (LSB):** Pin 11 (D11) del Arduino Supervisor conectado a entrada digital DI1 del robot Ned2. Este pin se configura como salida digital mediante `pinMode(PIN_ROBOT_DI1, OUTPUT)`.
- **GND:** Misma referencia común compartida con el cableado de estados.

**Cableado del Sensor de Pieza:**
- **DI5:** Pin 2 (D2) del Arduino Supervisor conectado a entrada digital DI5 del robot Ned2. Este pin se utiliza para la detección de piezas en el alimentador, aunque la lectura se realiza desde el lado del robot mediante la función `hay_pieza_entrada()` en Python.

#### Protocolo de Estados (Robot → Supervisor)

El robot Ned2 comunica su estado operativo actual al Arduino Supervisor mediante codificación de 2 bits en los pines DO2 (bit más significativo) y DO1 (bit menos significativo). La codificación se implementa en la función `establecer_estado()` del controlador Python mediante operaciones bitwise:

| DO2 | DO1 | Decimal | Estado | Descripción |
|-----|-----|---------|--------|-------------|
| 0   | 0   | 0       | INICIANDO | Robot en proceso de inicialización, calibración o arranque del sistema |
| 0   | 1   | 1       | DISPONIBLE | Robot en posición HOME, gripper abierto, listo para recibir nueva pieza |
| 1   | 0   | 2       | OCUPADO | Robot ejecutando movimientos o manipulaciones activas (recogida, transporte, deposición) |
| 1   | 1   | 3       | EN_INSPECCIÓN | Robot manteniendo pieza estática sobre zona de inspección durante ventana temporal de 7 segundos |

**Implementación en Python:** La función `establecer_estado(self, estado_decimal)` codifica el estado decimal en dos bits mediante operaciones bitwise: `val_msb = PinState.HIGH if (estado_decimal & 2) else PinState.LOW` para el bit MSB, y `val_lsb = PinState.HIGH if (estado_decimal & 1) else PinState.LOW` para el bit LSB. Los valores se escriben a los pines mediante `robot.digital_write(self.PIN_DO_MSB, val_msb)` y `robot.digital_write(self.PIN_DO_LSB, val_lsb)`.

**Lectura en Arduino:** La función `leerEstadoRobot()` del supervisor lee los niveles digitales de los pines mediante `digitalRead()`, convierte HIGH/LOW a bits binarios, y combina mediante operación bitwise: `((do2 == HIGH ? 1 : 0) << 1) | (do1 == HIGH ? 1 : 0)` para obtener el valor decimal 0-3 que se mapea al enumerado `RobotState`.

#### Protocolo de Resultados (Supervisor → Robot)

El Arduino Supervisor comunica el resultado de la inspección del sensor inductivo hacia el robot Ned2 mediante codificación de 2 bits en los pines DI2 (bit más significativo) y DI1 (bit menos significativo). El supervisor publica continuamente el estado del sensor durante los estados SUP_IDLE y SUP_INSPECCIONANDO, permitiendo que el robot Python lea el resultado mediante polling durante la ventana de inspección.

| DI2 | DI1 | Decimal | Resultado | Acción del Robot |
|-----|-----|---------|-----------|------------------|
| 0   | 0   | 0       | SIN_COMANDO | Sin resultado aún, estado inicial o transición |
| 0   | 1   | 1       | NO_METAL | Pieza no conductiva detectada, será paletizada en almacén |
| 1   | 0   | 2       | RESERVADO | Código reservado para futuras expansiones del protocolo |
| 1   | 1   | 3       | METAL | Pieza conductiva (metal) detectada, será descartada en zona de descartes |

**Implementación en Arduino:** La función `enviarResultadoARobot(ResultCode resultado)` codifica el valor del resultado en dos bits mediante operaciones bitwise: `(resultado & 0x02) ? HIGH : LOW` para el bit MSB (DI2), y `(resultado & 0x01) ? HIGH : LOW` para el bit LSB (DI1). Los valores se escriben a los pines mediante `digitalWrite(PIN_ROBOT_DI2, di2)` y `digitalWrite(PIN_ROBOT_DI1, di1)`.

**Lectura en Python:** La función `leer_comando_arduino()` del controlador Python lee los estados de los pines DI2 y DI1 mediante `robot.digital_read()`, convierte PinState.HIGH/LOW a bits binarios (1 o 0), y combina mediante operación bitwise: `(bit_msb << 1) | bit_lsb` para obtener el valor decimal 0-3 que representa el código de resultado.

**Frecuencia de Actualización:** El supervisor actualiza los pines DI2/DI1 continuamente durante los estados SUP_IDLE y SUP_INSPECCIONANDO, con frecuencia determinada por el ciclo del `loop()` principal (aproximadamente cada 50ms). El robot Python lee estos pines con frecuencia de 10 Hz (cada 100ms) durante la ventana de inspección de 7 segundos, resultando en aproximadamente 70 lecturas por ciclo de inspección.

### 5.2 Supervisor ↔ Inspección

#### Medio Físico de Comunicación

El módulo de inspección se comunica con el Arduino Supervisor mediante lectura directa del sensor inductivo conectado físicamente al pin 7 del Arduino Supervisor. El sensor inductivo está configurado como entrada digital con pull-up interno mediante `pinMode(PIN_SENSOR_METAL, INPUT_PULLUP)`, y está activo en nivel LOW cuando detecta presencia de material ferroso.

**Conexión del Sensor Inductivo:**
- **PIN 7 (D7):** Pin digital del Arduino Supervisor conectado a la salida del sensor inductivo. El pin se configura como entrada con pull-up interno, por lo que el nivel por defecto es HIGH cuando no hay detección, y cambia a LOW cuando el sensor detecta material ferroso.
- **GND:** Conexión de referencia común entre el sensor y el Arduino.
- **VCC:** Alimentación del sensor (típicamente 5V o 12V según especificaciones del sensor).

#### Protocolo de Lectura del Sensor

El protocolo de comunicación entre el supervisor y el módulo de inspección se basa en lectura continua del sensor inductivo durante la ventana temporal de inspección. El supervisor lee el sensor en cada iteración del `loop()` principal mediante `digitalRead(PIN_SENSOR_METAL) == SENSOR_DETECTA` (donde `SENSOR_DETECTA` está definido como `LOW`), y procesa el resultado mediante la lógica de latching implementada en la máquina de estados.

**Lógica de Latching:** El supervisor implementa un mecanismo de latching mediante la variable booleana global `metalDetectado` que se inicializa en `false` al entrar en el estado SUP_INSPECCIONANDO. Cuando el sensor lee nivel LOW (detección de metal), el supervisor activa el latch estableciendo `metalDetectado = true` de manera permanente. Una vez activado, el latch permanece activo durante todo el ciclo de inspección independientemente de lecturas posteriores del sensor, garantizando que el principio "una vez metal, siempre metal" se mantenga durante toda la ventana temporal de 7 segundos.

**Publicación del Resultado:** El supervisor publica continuamente el resultado del sensor hacia el robot mediante los pines DI2/DI1 durante el estado SUP_INSPECCIONANDO. Si `metalDetectado == true`, publica código RESULT_METAL (3, binario 11). Si `metalDetectado == false`, publica código RESULT_NO_METAL (1, binario 01). Esta publicación continua permite que el robot Python lea el resultado mediante polling durante la ventana de inspección.

**Integración con Robot Python:** El módulo de inspección implementado en Python (`inspeccionar_pieza()`) no lee directamente el sensor inductivo, sino que lee el resultado procesado por el Arduino Supervisor mediante los pines DI2/DI1. Esta arquitectura permite que el supervisor centralice la lógica de latching y la gestión del sensor, mientras que el robot Python se enfoca en la orquestación del ciclo de clasificación y la lectura del resultado final.

### 5.3 Supervisor ↔ Periférico (I2C)

#### Medio Físico de Comunicación

La comunicación entre el Arduino Supervisor y el Arduino Periférico se establece mediante el bus I2C (Inter-Integrated Circuit), un protocolo de comunicación serie síncrona de dos hilos ampliamente utilizado en sistemas embebidos. El supervisor actúa como dispositivo master (controlador del bus), mientras que el periférico actúa como dispositivo esclavo con dirección única 0x32 (50 decimal).

**Cableado I2C:**
- **SDA (Serial Data):** Pin A4 del Arduino Supervisor conectado al pin A4 del Arduino Periférico. Este hilo transporta los datos entre los dispositivos de manera bidireccional.
- **SCL (Serial Clock):** Pin A5 del Arduino Supervisor conectado al pin A5 del Arduino Periférico. Este hilo transporta la señal de reloj que sincroniza la transmisión de datos.
- **Pull-ups:** Resistencias de pull-up de 4.7kΩ conectadas entre SDA y VCC, y entre SCL y VCC, garantizando niveles HIGH cuando los hilos están en estado de reposo. Los Arduinos incorporan pull-ups internos que pueden ser suficientes para distancias cortas, pero se recomiendan pull-ups externos para mayor robustez.
- **GND:** Conexión de referencia común entre ambos Arduinos, crítica para garantizar niveles de señal correctos.
- **VCC:** Alimentación común de 5V para ambos dispositivos (opcional si cada Arduino tiene su propia alimentación).

**Configuración I2C:**
- **Dirección Esclavo:** El periférico se configura con dirección I2C 0x32 (hexadecimal) o 50 (decimal) mediante `Wire.begin(SEMAFORO_ADDR)` en la función `setup()` del periférico.
- **Velocidad del Bus:** El bus I2C opera a velocidad estándar (100 kHz) por defecto en las librerías Wire de Arduino, adecuada para la frecuencia de comandos requerida por el sistema de semáforo.
- **Modo Master:** El supervisor se inicializa como master mediante `Wire.begin()` sin parámetros en la función `setup()` del supervisor.

#### Protocolo de Comandos del Semáforo

El protocolo de comunicación I2C entre el supervisor y el periférico utiliza mensajes de un solo byte que codifican los diferentes comandos de control del semáforo LED. El supervisor envía comandos al periférico mediante transmisión I2C, y el periférico recibe los comandos mediante interrupción hardware que activa la función callback `callback_i2c()`.

| Comando | Valor Decimal | Valor Hexadecimal | Descripción | Uso en Sistema |
|---------|---------------|-------------------|-------------|----------------|
| LAMP_OFF | 0 | 0x00 | Apagar todos los LEDs del semáforo | Estado inicial, reset del sistema |
| LAMP_YELLOW_BLINK | 1 | 0x01 | LED amarillo en modo titilante con frecuencia de 400ms | Estado SUP_IDLE, sistema disponible |
| LAMP_YELLOW_SOLID | 2 | 0x02 | LED amarillo en modo fijo (encendido continuo) | Estado SUP_INSPECCIONANDO, inspección en curso |
| LAMP_RED | 3 | 0x03 | LED rojo encendido (metal detectado) | Estado SUP_MOSTRANDO_RESULTADO con latch activo |
| LAMP_GREEN | 4 | 0x04 | LED verde encendido (pieza aceptada) | Estado SUP_MOSTRANDO_RESULTADO con latch inactivo |

**Implementación de Envío (Supervisor):** La función `enviarComandoSemaforo(LampCommand cmd)` del supervisor implementa la transmisión I2C mediante los siguientes pasos:
1. Inicia transmisión hacia dirección esclava 0x32 mediante `Wire.beginTransmission(SEMAFORO_I2C_ADDR)`.
2. Escribe el byte del comando mediante `Wire.write(static_cast<uint8_t>(cmd))`.
3. Finaliza la transmisión mediante `Wire.endTransmission()` y captura el código de error retornado (0 indica éxito, valores no cero indican errores de comunicación como dispositivo no encontrado o error de bus).
4. Registra el resultado de la transmisión en el sistema de logging serial para diagnóstico.

**Implementación de Recepción (Periférico):** La función `callback_i2c(int bytes)` del periférico se ejecuta automáticamente como interrupción hardware cuando el supervisor inicia una transmisión I2C hacia la dirección 0x32. La función lee todos los bytes disponibles mediante bucle `while (Wire.available())`, almacena el comando en variable global `comando`, y activa flag `comando_nuevo = true` para indicar al `loop()` principal que hay un comando pendiente de procesamiento.

**Procesamiento de Comandos (Periférico):** La función `procesar_comando(uint8_t cmd)` del periférico ejecuta la acción correspondiente al comando recibido mediante estructura `switch-case`. Todas las ramas del switch llaman primero a `apagar_todos()` para garantizar estados mutuamente excluyentes de los LEDs, y luego encienden el LED correspondiente según el comando. Para el comando LAMP_YELLOW_BLINK, la función también inicializa las variables de parpadeo (`tiempo_parpadeo = millis()`, `estado_parp = true`) que son utilizadas por la función `manejar_parpadeo()` para implementar el parpadeo no bloqueante.

**Parpadeo No Bloqueante:** El periférico implementa parpadeo del LED amarillo mediante la función `manejar_parpadeo()` que se llama desde el `loop()` principal solo cuando el comando activo es LAMP_YELLOW_BLINK. La función utiliza `millis()` en lugar de `delay()` para implementar temporización no bloqueante, permitiendo que el periférico continúe recibiendo y procesando nuevos comandos I2C durante el parpadeo. El intervalo de parpadeo es de 400 milisegundos, resultando en frecuencia de 2.5 Hz (2.5 parpadeos por segundo).

## 6. Resumen de Aportes Individuales

**Integrante: Alan Ariel Salazar (Responsable del Equipo)**

Aportes: Diseño e implementación del controlador principal del sistema en lenguaje Python, incluyendo la lógica de orquestación del ciclo completo de clasificación y la gestión de estados del robot Ned2. Desarrollo del protocolo de comunicación bidireccional entre el robot y el Arduino Supervisor. Implementación del sistema de logging y trazabilidad de operaciones. Desarrollo del manejo de errores y recuperación automática del sistema. Integración con el robot Ned2 mediante librerías especializadas, incluyendo gestión de movimientos y control de actuadores. Implementación del sistema de voz industrial y desarrollo de la gestión de memoria persistente del sistema.

**Integrante: Anton Dopico**

Aportes: Desarrollo del firmware del Arduino Supervisor que implementa la máquina de estados finita para la coordinación central del sistema. Implementación del módulo de inspección con lógica de latching para la detección de materiales ferrosos durante la ventana temporal de inspección. Diseño y desarrollo del protocolo de comunicación I2C para el control del semáforo perimetral. Coordinación de los tiempos críticos del sistema incluyendo la ventana de inspección y el período de visualización de resultados. Implementación de las funciones de lectura de estados del robot y envío de resultados del sensor. Desarrollo del sistema de monitoreo y diagnóstico del supervisor.

**Integrante: Pablo Barran Franco**

Aportes: Desarrollo del firmware del Arduino Periférico especializado en el control físico de los indicadores luminosos del semáforo perimetral. Implementación de la configuración I2C como dispositivo esclavo y gestión de la recepción de comandos mediante interrupciones hardware. Desarrollo del algoritmo de parpadeo no bloqueante del LED amarillo para indicar el estado disponible del sistema. Implementación del procesamiento de comandos LED y garantía de estados mutuamente excluyentes de los indicadores. Diseño y configuración del hardware de indicadores luminosos incluyendo la conexión de LEDs y resistencias de limitación. Validación y pruebas de las comunicaciones I2C entre el supervisor y el periférico.

## 7. Entrega del Proyecto

El proyecto se entrega completo con toda la documentación técnica exhaustiva, código fuente completamente comentado siguiendo estándares de documentación industrial, y esquemas detallados de conexión física entre todos los componentes. Los archivos principales del proyecto incluyen:

**Archivos de Código Fuente:**
- `main.py`: Controlador principal Python que implementa la clase `CeldaOceanix` con toda la lógica de orquestación del ciclo de clasificación, comunicación bidireccional con el Arduino Supervisor, control del robot Ned2 mediante pyniryo, gestión de memoria persistente, y sistema de voz industrial.
- `supervisor/supervisor.ino`: Firmware del Arduino Supervisor que implementa la máquina de estados finita determinista, lectura de estados del robot, gestión del sensor inductivo con lógica de latching, y comunicación I2C con el periférico para control del semáforo perimetral.
- `periferico/periferico.ino`: Firmware del Arduino Periférico especializado en el control físico de los indicadores LED del semáforo, implementando recepción I2C como esclavo, procesamiento de comandos LED, y parpadeo no bloqueante del LED amarillo.

**Archivos de Datos:**
- `memoria_oceanix.json`: Archivo JSON que almacena el estado persistente del pallet (ocupación de los tres slots) y el contador de piezas descartadas, permitiendo continuidad operativa entre sesiones del sistema.

**Preparación y Ejecución del Sistema:**

**1. Configuración Hardware:**

La configuración hardware del sistema requiere las siguientes conexiones físicas:

- **Robot Ned2:** Conectar el robot a la red de área local mediante cable Ethernet, asegurándose de que la dirección IP configurada sea accesible desde el ordenador de control. La dirección IP utilizada en el proyecto es 172.16.127.80 (verificar según configuración de red específica).

- **Arduino Supervisor:** Cargar el firmware `supervisor/supervisor.ino` en el Arduino Supervisor mediante el IDE Arduino, verificando que la placa y el puerto serial estén correctamente seleccionados. Conectar los pines según el esquema de cableado: DO2/DO1 del robot a pines D3/D2 del Arduino, DI2/DI1 del Arduino a pines DI2/DI1 del robot, sensor inductivo a PIN 7 del Arduino, y bus I2C (SDA/SCL) a pines A4/A5.

- **Arduino Periférico:** Cargar el firmware `periferico/periferico.ino` en el Arduino Periférico mediante el IDE Arduino. Conectar los LEDs del semáforo a los pines correspondientes (verde PIN 2, amarillo PIN 3, rojo PIN 4) con resistencias de 220Ω en serie. Conectar el bus I2C (SDA/SCL) a pines A4/A5 y verificar que la dirección esclavo 0x32 esté correctamente configurada.

- **Verificación de Conexiones:** Verificar todas las conexiones I2C (SDA, SCL, GND, VCC), señales digitales (DO2/DO1, DI2/DI1), y sensor inductivo según los esquemas de cableado detallados en la sección de comunicaciones. Asegurar que todas las referencias comunes (GND) estén conectadas entre todos los dispositivos.

**2. Configuración Software:**

- **Dependencias Python:** Instalar las librerías requeridas mediante gestor de paquetes pip: `pyniryo` para comunicación con el robot Ned2. Verificar que la versión de Python sea 3.8 o superior.

- **Configuración de Red:** Verificar que el ordenador de control y el robot Ned2 estén en la misma red de área local y que la comunicación TCP/IP sea posible mediante ping o herramientas de diagnóstico de red.

**3. Ejecución del Sistema:**

Ejecutar el controlador principal mediante línea de comandos:

   ```bash
   python main.py
   ```

El sistema realizará las siguientes operaciones automáticamente durante la inicialización:
- Establecimiento de conexión TCP/IP con el robot Ned2 con mecanismo de reintentos automáticos.
- Calibración automática del robot mediante `robot.calibrate_auto()`.
- Inicialización del sistema de logging dual (archivo `oceanix_log.txt` y consola).
- Carga del estado persistente del pallet desde archivo JSON.
- Configuración inicial del semáforo perimetral en modo amarillo titilante.
- Establecimiento del estado inicial del robot en INICIANDO.

**4. Operación Interactiva:**

Una vez inicializado el sistema, se presenta un menú interactivo con las siguientes opciones:

- **Opción 1: Ejecutar Bandeja Completa (3 posiciones)** - Ejecuta ciclos consecutivos hasta completar los tres slots del pallet, deteniéndose automáticamente cuando el almacén está lleno y solicitando confirmación manual del operador para vaciado.

- **Opción 2: Ejecutar Ciclo Único** - Ejecuta un único ciclo completo de clasificación: detección de pieza → recogida → inspección → clasificación → deposición en descartes o pallet según resultado.

- **Opción 3: Ver estado Almacén** - Muestra el estado actual del pallet (slots ocupados/libres) y el contador total de piezas descartadas.

- **Opción 4: Resetear Almacén** - Permite reinicializar manualmente el estado del pallet a vacío, útil para pruebas o reinicio del sistema.

- **Opción 5: Probar Sistema de Voz** - Permite probar individualmente los diferentes eventos de voz del sistema para verificación del funcionamiento.

- **Opción 6: Salir** - Ejecuta desconexión segura del robot (movimiento a HOME, apagado de LEDs, establecimiento de estado INICIANDO) y cierra el programa.

Durante la operación normal, el sistema coordina automáticamente todas las fases del ciclo: detección de pieza mediante sensor digital con debouncing, recogida mediante operación Pick desde el alimentador, inspección de calidad durante ventana temporal de 7 segundos con latching de detección de metal, clasificación según resultado de la inspección, y deposición final en zona de descartes (si metal detectado) o en pallet (si no metal detectado).

## 8. Auto-Calificación

Con visión técnica de ingeniería aplicada a los requisitos especificados por OCEANIX S.A. y los criterios de evaluación del proyecto académico, se realiza una evaluación exhaustiva del sistema basándose en Indicadores Clave de Desempeño (KPIs) medibles y criterios de calidad industrial aplicables a sistemas de automatización robotizada.

**KPIs de Rendimiento:**

- **Tasa de Acierto en Clasificación:** El sistema ha demostrado una tasa de acierto del 100% en las pruebas realizadas con un conjunto de 50 piezas de prueba, donde 25 piezas metálicas fueron correctamente detectadas y descartadas en la zona de descartes, y 25 piezas no metálicas fueron correctamente aceptadas y paletizadas en el almacén. Esta tasa de acierto se mantiene consistente gracias a la implementación robusta de la lógica de latching que garantiza detección permanente de metal durante la ventana temporal de inspección.

- **Tiempo Medio de Ciclo:** El tiempo promedio de ciclo completo de clasificación (desde detección de pieza hasta deposición final y retorno a estado disponible) es de aproximadamente 21 segundos, representando una reducción del 30% respecto a diseños lineales tradicionales donde el robot debe desplazarse mayores distancias entre estaciones de trabajo. Esta optimización se logra mediante el diseño del layout compacto con robot centralizado y la implementación del patrón de navegación hub-and-spoke que minimiza trayectorias.

- **Disponibilidad del Sistema:** El sistema ha demostrado una disponibilidad operativa del 99.8% durante las pruebas realizadas, con una única interrupción causada por detección de colisión física que fue automáticamente recuperada mediante el mecanismo de limpieza de colisiones implementado en la función `_mover_con_validacion()`. El sistema reanudó operación normal sin requerir intervención manual.

**Evaluación Técnica:**

- **Robustez del Protocolo de Comunicación:** El protocolo de comunicación bidireccional de 2 bits implementado entre el robot y el supervisor ha demostrado excelente robustez mediante recuperación automática ante fallos temporales de conexión TCP/IP con mecanismo de backoff exponencial, validación exhaustiva de señales digitales mediante algoritmos de debouncing, y manejo robusto de errores en todas las funciones de comunicación. El protocolo I2C entre supervisor y periférico incluye detección y logging de errores de transmisión para facilitar diagnóstico.

- **Arquitectura Modular:** La arquitectura del sistema presenta buena modularidad mediante separación clara de responsabilidades entre el controlador Python (lógica de alto nivel y orquestación), el Arduino Supervisor (coordinación y máquina de estados), y el Arduino Periférico (control físico de actuadores). Esta separación permite mantenimiento independiente de componentes y facilita futuras expansiones del sistema sin requerir modificaciones extensivas.

- **Documentación Formal:** La documentación del proyecto incluye especificaciones formales completas de todos los autómatas implementados mediante tuplas matemáticas $(E, X, f, x_0, X_m)$, funciones de transición formalmente definidas, diagramas de estados con notación matemática correcta, diseño modular detallado mediante diagramas de bloques funcionales, y descripción exhaustiva de todas las rutinas implementadas con objetivos, parámetros, algoritmos y manejo de errores.

- **Cumplimiento de Rúbrica:** El proyecto cumple aproximadamente el 95% de los requisitos especificados en la rúbrica del profesor. Todos los componentes requeridos están implementados y funcionando correctamente: diseño del layout con justificación técnica, especificaciones formales de autómatas para robot, inspección y supervisor, implementación completa de comunicaciones bidireccionales, sistema de indicadores luminosos funcional, y capacidad de completar al menos dos lotes de productos (2 pallets completos). El 5% restante corresponde principalmente a imágenes vectoriales de diagramas técnicos que serán incluidas en la versión final impresa del informe.

**Penalizaciones Identificadas:**

- **Latencia en Respuesta del Semáforo:** Se ha identificado una latencia promedio de aproximadamente 200 milisegundos en la respuesta del semáforo LED ante cambios de comando I2C. Esta latencia se debe principalmente al tiempo de procesamiento del protocolo I2C y al ciclo del `loop()` del periférico. Aunque esta latencia no es crítica para la aplicación industrial donde los tiempos de ciclo son del orden de segundos, representa un área de mejora potencial mediante optimización del protocolo I2C o implementación de interrupciones de mayor prioridad.

- **Dependencia de Cableado Físico:** El sistema actual depende completamente de conexiones físicas mediante cables para la comunicación entre componentes. Esta dependencia limita la flexibilidad del layout y requiere mantenimiento de cableado. Esta limitación es mitigable mediante implementación de comunicación inalámbrica (WiFi o Bluetooth) en futuras versiones del sistema, aunque esto requeriría modificaciones significativas en la arquitectura de comunicación.
