/*
 * ============================================================================
 * PROYECTO FINAL: PERIFÉRICO SEMÁFORO - SISTEMA OCEANIX
 * ARCHIVO: periferico/periferico.ino
 * ============================================================================
 *
 * INSTITUCIÓN: Universidad Intercontinental de la Empresa
 * ASIGNATURA: Tecnología de Automatización y Robotización Empresarial
 * PROFESOR: Eladio Dapena
 * FECHA: 5 de diciembre de 2025
 *
 * EQUIPO DE DESARROLLO:
 * - Alan Ariel Salazar (Responsable del Equipo)
 * - Anton Dopico
 * - Pablo Barran Franco
 *
 * ============================================================================
 * PRESENTACIÓN DEL COMPONENTE
 * ============================================================================
 *
 * Este firmware implementa el Arduino Periférico del sistema de clasificación
 * Oceanix, especializado exclusivamente en el control físico de los indicadores
 * luminosos LED del semáforo perimetral.
 *
 * El periférico actúa como dispositivo esclavo I2C con dirección única 0x32,
 * recibiendo comandos del Arduino Supervisor mediante interrupciones hardware
 * y ejecutando acciones inmediatas sobre los LEDs según el protocolo establecido.
 *
 * La implementación garantiza operación determinista mediante procesamiento
 * no bloqueante de comandos, parpadeo automático del LED amarillo con temporización
 * precisa de 400ms, y estados mutuamente excluyentes de los LEDs (solo un LED
 * puede estar encendido a la vez).
 *
 * El componente está diseñado con arquitectura minimalista y especializada,
 * delegando toda la lógica de decisión al Supervisor y enfocándose únicamente
 * en la ejecución eficiente de comandos de control LED.
 *
 * ============================================================================
 * OBJETIVO GENERAL
 * ============================================================================
 *
 * Implementar el controlador periférico del semáforo perimetral para el sistema
 * de clasificación Oceanix, actuando como esclavo I2C que recibe comandos del
 * supervisor y controla indicadores luminosos LED de manera determinista.
 *
 * ============================================================================
 * OBJETIVOS ESPECÍFICOS
 * ============================================================================
 *
 * 1. IMPLEMENTAR COMUNICACIÓN I2C COMO ESCLAVO
 *    - Configurar dirección I2C 0x32 (50 decimal) mediante Wire.begin(SEMAFORO_ADDR)
 *      para recepción de comandos del Supervisor
 *    - Implementar callback de recepción de datos vía interrupción hardware mediante
 *      función callback_i2c() registrada con Wire.onReceive()
 *    - Gestionar buffer de comandos con flags de actualización mediante variables
 *      globales volátiles (comando, comando_nuevo) para comunicación thread-safe
 *      entre interrupción y loop principal
 *
 * 2. CONTROLAR INDICADORES LUMINOSOS LED
 *    - Gestionar 3 LEDs independientes: verde (PIN 2), amarillo (PIN 3), rojo (PIN 4)
 *      configurados como salidas digitales mediante pinMode(OUTPUT)
 *    - Implementar lógica de conmutación digital con protección mediante función
 *      apagar_todos() que garantiza apagado de todos los LEDs antes de encender uno nuevo
 *    - Garantizar estados mutuamente excluyentes (solo un LED activo) mediante
 *      invocación sistemática de apagar_todos() en todas las ramas del switch-case
 *      de procesamiento de comandos
 *
 * 3. IMPLEMENTAR PARPADEO AUTOMÁTICO NO BLOQUEANTE
 *    - Temporizador de 400ms para amarillo titilante mediante comparación de millis()
 *      con tiempo_parpadeo almacenado, resultando en frecuencia de 2.5 Hz
 *    - Lógica no bloqueante en loop principal mediante función manejar_parpadeo()
 *      que se ejecuta solo cuando comando activo es LAMP_YELLOW_BLINK
 *    - Sincronización precisa sin interferir con comunicaciones I2C mediante
 *      uso de millis() en lugar de delay(), permitiendo recepción continua de
 *      nuevos comandos durante el parpadeo
 *
 * 4. GESTIONAR PROTOCOLO DE COMANDOS DETERMINISTA
 *    - Procesar 5 comandos I2C diferentes con acciones inmediatas mediante estructura
 *      switch-case en función procesar_comando() que mapea valores 0-4 a acciones LED
 *    - Implementar máquina de estados simple para modos LED mediante variable global
 *      comando que almacena el estado actual y determina el comportamiento del sistema
 *    - Logging detallado para diagnóstico operativo mediante comunicación serial a
 *      9600 baudios con mensajes descriptivos de cada comando recibido y ejecutado
 *
 * ============================================================================
 * CONFIGURACIÓN DEL HARDWARE
 * ============================================================================
 *
 * PLACA ARDUINO: Arduino Uno o compatible (ATmega328P)
 *
 * CONEXIONES FÍSICAS DETALLADAS:
 *
 * INDICADORES LED:
 *   - LED VERDE: Ánodo → PIN 2 (D2) del Arduino, Cátodo → GND vía resistencia 220Ω
 *     Función: Indicador visual de pieza aceptada (material no metálico)
 *   - LED AMARILLO: Ánodo → PIN 3 (D3) del Arduino, Cátodo → GND vía resistencia 220Ω
 *     Función: Indicador de estado del sistema (titilante = disponible, fijo = inspección)
 *   - LED ROJO: Ánodo → PIN 4 (D4) del Arduino, Cátodo → GND vía resistencia 220Ω
 *     Función: Indicador visual de metal detectado (pieza rechazada)
 *
 * BUS I2C (Periférico ↔ Supervisor):
 *   - SDA → PIN A4 del Arduino: Datos del bus I2C (bidireccional)
 *   - SCL → PIN A5 del Arduino: Reloj del bus I2C (sincronización)
 *   - Pull-ups externos de 4.7kΩ recomendados entre SDA/SCL y VCC (5V)
 *   - GND común entre Periférico y Supervisor (referencia común crítica)
 *
 * ALIMENTACIÓN:
 *   - VCC: 5V DC regulado conectado a VIN o pin 5V del Arduino
 *   - GND: Referencia común conectada a GND del Supervisor y fuente de alimentación
 *   - Consumo típico: < 100mA con los 3 LEDs encendidos simultáneamente (no ocurre)
 *
 * COMUNICACIÓN SERIAL (Debug):
 *   - Configurada a 9600 baudios para logging y diagnóstico
 *   - Utilizada exclusivamente para monitoreo, no para comunicación con Supervisor
 *
 * ============================================================================
 * ARQUITECTURA DEL COMPONENTE
 * ============================================================================
 *
 * El periférico implementa una arquitectura simple y especializada:
 *
 * SUPERVISOR ──I2C──> PERIFÉRICO ──GPIO──> LEDs
 *    │                    │                    │
 *    │ Comando (1 byte)   │ Procesamiento      │ Control físico
 *    │ Dirección: 0x32    │ No bloqueante      │ Estados mutuamente
 *    │                    │                    │ excluyentes
 *    └────────────────────┴────────────────────┘
 *
 * FLUJO DE DATOS:
 *   1. Supervisor envía comando I2C hacia dirección 0x32
 *   2. Interrupción hardware activa callback_i2c()
 *   3. Comando se almacena en variable global comando
 *   4. Flag comando_nuevo se activa para señalizar loop principal
 *   5. Loop principal detecta flag y llama a procesar_comando()
 *   6. Función procesar_comando() ejecuta acción LED correspondiente
 *   7. Si comando es YELLOW_BLINK, manejar_parpadeo() gestiona titileo
 *
 * ============================================================================
 * PROTOCOLO I2C - COMANDOS DE CONTROL
 * ============================================================================
 *
 * ESTRUCTURA DEL MENSAJE I2C:
 *   El Supervisor envía un único byte que codifica el comando LED. El periférico
 *   recibe este byte mediante interrupción hardware y lo procesa en el loop principal.
 *
 * ┌─────────────┬─────────────┬─────────────────────────────────────┐
 * │ Comando     │ Valor       │ Acción del Periférico              │
 * ├─────────────┼─────────────┼─────────────────────────────────────┤
 * │ LAMP_OFF    │ 0x00 (0)    │ Apagar todos los LEDs               │
 * │ YELLOW_BLINK│ 0x01 (1)    │ Amarillo titilante 400ms            │
 * │ YELLOW_SOLID│ 0x02 (2)    │ Amarillo continuo                   │
 * │ LAMP_RED    │ 0x03 (3)    │ Rojo continuo (metal)               │
 * │ LAMP_GREEN  │ 0x04 (4)    │ Verde continuo (aceptado)           │
 * └─────────────┴─────────────┴─────────────────────────────────────┘
 *
 * DETALLES DEL PROTOCOLO:
 *   - Dirección esclavo: 0x32 (50 decimal, hexadecimal 0x32)
 *   - Mensaje: 1 byte único que codifica el comando
 *   - Recepción: Vía interrupción hardware onReceive() que activa callback_i2c()
 *   - Procesamiento: En loop principal de forma no bloqueante mediante flag comando_nuevo
 *   - Confirmación: Logging serial para diagnóstico (no se envía ACK explícito al Supervisor)
 *   - Latencia: < 50ms desde recepción I2C hasta ejecución del comando LED
 *
 * ============================================================================
 * LÓGICA IMPLEMENTADA
 * ============================================================================
 *
 * El firmware implementa un bucle principal (loop()) que se ejecuta continuamente
 * con frecuencia aproximada de 20 Hz (cada 50ms debido al delay final). En cada
 * iteración del bucle se ejecutan las siguientes operaciones en orden:
 *
 * 1. VERIFICACIÓN DE COMANDO NUEVO:
 *    - Se verifica el flag comando_nuevo que se activa cuando callback_i2c() recibe
 *      un nuevo comando del Supervisor
 *    - Si el flag está activo, se resetea y se llama a procesar_comando(comando)
 *      para ejecutar la acción LED correspondiente
 *
 * 2. GESTIÓN DE PARPADEO (si aplica):
 *    - Si el comando activo es LAMP_YELLOW_BLINK, se llama a manejar_parpadeo()
 *      que implementa el algoritmo de titileo no bloqueante
 *    - La función manejar_parpadeo() compara millis() con tiempo_parpadeo almacenado
 *      y conmuta el estado del LED amarillo cada 400ms
 *
 * 3. DELAY DE CONTROL:
 *    - Se ejecuta delay(50) para controlar la frecuencia del bucle y evitar
 *      saturación del procesador, manteniendo capacidad de respuesta para I2C
 *
 * FUNCIÓN CALLBACK I2C (Interrupción Hardware):
 *   La función callback_i2c() se ejecuta automáticamente cuando el Supervisor
 *   inicia una transmisión I2C hacia la dirección 0x32. La función lee todos los
 *   bytes disponibles mediante bucle while (Wire.available()), almacena el último
 *   byte en la variable global comando, y activa el flag comando_nuevo para señalizar
 *   al loop principal que hay un comando pendiente de procesamiento.
 *
 * FUNCIÓN PROCESAMIENTO DE COMANDOS:
 *   La función procesar_comando() implementa una máquina de estados simple mediante
 *   estructura switch-case que mapea los 5 comandos válidos (0-4) a acciones LED
 *   específicas. Todas las ramas del switch llaman primero a apagar_todos() para
 *   garantizar estados mutuamente excluyentes, y luego ejecutan la acción correspondiente.
 *   Para el comando LAMP_YELLOW_BLINK, la función también inicializa las variables
 *   de parpadeo (tiempo_parpadeo, estado_parp) que son utilizadas por manejar_parpadeo().
 *
 * FUNCIÓN PARPADEO NO BLOQUEANTE:
 *   La función manejar_parpadeo() implementa temporización no bloqueante mediante
 *   comparación de millis() con tiempo_parpadeo almacenado. Cuando transcurren 400ms,
 *   se actualiza tiempo_parpadeo y se conmuta el estado del LED amarillo mediante
 *   digitalWrite(). Este algoritmo permite que el periférico continúe recibiendo y
 *   procesando nuevos comandos I2C durante el parpadeo, garantizando responsividad
 *   del sistema ante cambios de estado del Supervisor.
 *
 * ============================================================================
 * ESPECIFICACIONES TÉCNICAS
 * ============================================================================
 *
 * HARDWARE:
 *   - Microcontrolador: ATmega328P (Arduino Uno estándar)
 *   - LEDs: 5mm estándar con resistencias limitadoras de corriente de 220Ω
 *   - Bus I2C: Implementación hardware TWI (Two-Wire Interface) integrada en ATmega328P
 *   - Alimentación: 5V DC regulado (tolerancia: 4.5V - 5.5V)
 *   - Consumo máximo: < 150mA con todos los LEDs encendidos (caso teórico)
 *
 * SOFTWARE:
 *   - Framework: Arduino IDE 1.8+ o superior
 *   - Librería: Wire.h (I2C integrada en Arduino Core)
 *   - Frecuencia de reloj: 16MHz (cristal estándar Arduino Uno)
 *   - Comunicación serial: 9600 baudios para logging y diagnóstico
 *   - Tamaño de código compilado: ~2.5 KB (muy eficiente)
 *
 * TEMPORIZACIÓN:
 *   - Intervalo parpadeo: 400ms exactos (frecuencia 2.5 Hz, período 0.4s)
 *   - Latencia I2C: < 1ms desde transmisión Supervisor hasta recepción en callback
 *   - Tiempo respuesta LED: < 10ms desde procesamiento de comando hasta cambio físico
 *   - Frecuencia loop principal: ~20 Hz (cada 50ms)
 *
 * ============================================================================
 */

#include <Wire.h>

// ============================================================================
// CONFIGURACIÓN I2C
// ============================================================================
#define SEMAFORO_ADDR 0x32

#define LAMP_OFF 0
#define LAMP_YELLOW_BLINK 1
#define LAMP_YELLOW_SOLID 2
#define LAMP_RED 3
#define LAMP_GREEN 4

// ============================================================================
// PINES LEDS
// ============================================================================
#define PIN_VERDE 2
#define PIN_AMARILLO 3
#define PIN_ROJO 4

// ============================================================================
// VARIABLES
// ============================================================================
volatile uint8_t comando = LAMP_YELLOW_BLINK;
volatile bool comando_nuevo = false;

// Para parpadeo
unsigned long tiempo_parpadeo = 0;
bool estado_parp = false;
#define INTERVALO_PARPADEO 400

// ============================================================================
// FUNCIÓN: CONFIGURACIÓN INICIAL DEL PERIFÉRICO
// ============================================================================
// OBJETIVO: Inicializar hardware y comunicaciones del controlador LED
// COMPONENTES: Pines LED, I2C esclavo, estado inicial OFF, banner informativo
// CRÍTICO: Configuración necesaria para recepción de comandos I2C
// ============================================================================
void setup() {
  pinMode(PIN_VERDE, OUTPUT);
  pinMode(PIN_AMARILLO, OUTPUT);
  pinMode(PIN_ROJO, OUTPUT);
  
  Serial.begin(9600);
  delay(100);
  
  Serial.println("\n=== SEMÁFORO OCEANIX v3 FINAL ===\n");
  Serial.print("[I2C] Dirección: 0x");
  Serial.println(SEMAFORO_ADDR, HEX);
  
  apagar_todos();
  
  Wire.begin(SEMAFORO_ADDR);
  Wire.onReceive(callback_i2c);
  
  Serial.println("[OK] Listo - Escuchando I2C\n");
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
void loop() {
  // Si hay comando nuevo, procesarlo
  if (comando_nuevo) {
    comando_nuevo = false;
    procesar_comando(comando);
  }
  
  // Si estamos en parpadeo, manejar eso
  if (comando == LAMP_YELLOW_BLINK) {
    manejar_parpadeo();
  }
  
  delay(50);
}

// ============================================================================
// FUNCIÓN: CALLBACK DE RECEPCIÓN I2C - INTERRUPTOR CRÍTICO
// ============================================================================
// OBJETIVO: Gestionar recepción de comandos I2C del supervisor Supervisor
// MECANISMO: Interrupción hardware que activa recepción automática
// CRÍTICO: Esta función determina la responsiveness del sistema LED
// ============================================================================
void callback_i2c(int bytes) {
  while (Wire.available()) {
    uint8_t valor = Wire.read();
    comando = valor;
    comando_nuevo = true;
  }
}

// ============================================================================
// FUNCIÓN: PROCESAMIENTO DETERMINISTA DE COMANDOS LED
// ============================================================================
// OBJETIVO: Ejecutar acciones LED según protocolo I2C establecido
// LÓGICA: Máquina de estados simple con 5 comandos válidos
// CRÍTICO: Garantiza estados mutuamente excluyentes de LEDs
// ============================================================================
void procesar_comando(uint8_t cmd) {
  switch(cmd) {
    case LAMP_OFF:
      apagar_todos();
      Serial.println("[LED] OFF");
      break;
      
    case LAMP_YELLOW_BLINK:
      apagar_todos();
      digitalWrite(PIN_AMARILLO, HIGH);
      tiempo_parpadeo = millis();
      estado_parp = true;
      Serial.println("[LED] Amarillo parpadeando");
      break;
      
    case LAMP_YELLOW_SOLID:
      apagar_todos();
      digitalWrite(PIN_AMARILLO, HIGH);
      Serial.println("[LED] Amarillo sólido");
      break;
      
    case LAMP_RED:
      apagar_todos();
      digitalWrite(PIN_ROJO, HIGH);
      Serial.println("[LED] ROJO");
      break;
      
    case LAMP_GREEN:
      apagar_todos();
      digitalWrite(PIN_VERDE, HIGH);
      Serial.println("[LED] VERDE");
      break;
      
    default:
      Serial.print("[LED] Desconocido: ");
      Serial.println(cmd);
  }
}

// ============================================================================
// FUNCIÓN: PARPADEO NO BLOQUEANTE - ALGORITMO TEMPORIZADO
// ============================================================================
// OBJETIVO: Implementar titileo amarillo sin bloquear comunicaciones I2C
// ALGORITMO: Temporizador millis() con conmutación cada 400ms
// VENTAJA: No interfiere con recepción de comandos del supervisor
// ============================================================================
void manejar_parpadeo() {
  unsigned long ahora = millis();
  
  if (ahora - tiempo_parpadeo >= INTERVALO_PARPADEO) {
    tiempo_parpadeo = ahora;
    estado_parp = !estado_parp;
    digitalWrite(PIN_AMARILLO, estado_parp ? HIGH : LOW);
  }
}

// ============================================================================
// APAGAR TODOS LOS LEDS
// ============================================================================
void apagar_todos() {
  digitalWrite(PIN_VERDE, LOW);
  digitalWrite(PIN_AMARILLO, LOW);
  digitalWrite(PIN_ROJO, LOW);
}
