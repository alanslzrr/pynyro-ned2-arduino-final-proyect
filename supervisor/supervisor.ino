/*
 * ============================================================================
 * PROYECTO FINAL: SUPERVISOR ARDUINO - SISTEMA OCEANIX
 * ARCHIVO: supervisor/supervisor.ino
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
 * Este firmware implementa el Arduino Supervisor del sistema de clasificación
 * Oceanix, actuando como cerebro central que coordina todos los componentes de
 * la celda robotizada mediante protocolos de comunicación síncronos.
 *
 * El supervisor implementa una máquina de estados finita determinista con tres
 * estados principales (SUP_IDLE, SUP_INSPECCIONANDO, SUP_MOSTRANDO_RESULTADO)
 * que gestiona la coordinación entre el robot Ned2, el sensor inductivo para
 * detección de materiales ferrosos, y el semáforo perimetral mediante comunicación
 * I2C con el Arduino Periférico.
 *
 * La lógica implementada garantiza sincronización precisa entre todos los
 * componentes del sistema, implementa latching de detección de metal durante
 * la ventana temporal de inspección de 7 segundos, y controla la visualización
 * de resultados mediante el semáforo perimetral durante exactamente 10 segundos.
 *
 * ============================================================================
 * OBJETIVO GENERAL
 * ============================================================================
 *
 * Implementar el controlador Supervisor del sistema de clasificación Oceanix,
 * actuando como cerebro central que coordina robot Ned2, sensor inductivo
 * y semáforo perimetral mediante protocolos de comunicación síncronos.
 *
 * ============================================================================
 * OBJETIVOS ESPECÍFICOS
 * ============================================================================
 *
 * 1. IMPLEMENTAR MÁQUINA DE ESTADOS FINITA PARA SUPERVISIÓN
 *    - Gestionar tres estados principales: SUP_IDLE (sistema disponible),
 *      SUP_INSPECCIONANDO (ventana temporal de 7 segundos activa),
 *      SUP_MOSTRANDO_RESULTADO (visualización de resultado por 10 segundos)
 *    - Implementar lógica de transición basada en señales del robot leídas
 *      mediante pines digitales DO2/DO1
 *    - Gestionar temporizadores para estados con duración fija mediante función
 *      millis() para implementación no bloqueante
 *
 * 2. DESARROLLAR PROTOCOLO DE COMUNICACIÓN BIDIRECCIONAL
 *    - Leer estados del robot mediante pines digitales DO2 (PIN 3, MSB) y DO1
 *      (PIN 2, LSB), decodificando valores 0-3 que representan los 4 estados
 *      del autómata del robot
 *    - Enviar resultados de clasificación del sensor inductivo vía pines DI2
 *      (PIN 12, MSB) y DI1 (PIN 11, LSB), codificando valores 0-3 que representan
 *      los códigos de resultado (SIN_COMANDO, NO_METAL, RESERVADO, METAL)
 *    - Implementar protocolo 2-bit síncrono con validación mediante lectura
 *      continua en cada iteración del loop principal
 *
 * 3. INTEGRAR SENSOR INDUCTIVO CON LÓGICA DE LATCHING
 *    - Monitoreo continuo del sensor inductivo conectado a PIN 7 configurado
 *      como INPUT_PULLUP (sensor activo en nivel LOW cuando detecta metal)
 *    - Implementar latching durante fase de inspección mediante variable booleana
 *      global metalDetectado que se activa cuando se detecta metal por primera vez
 *      y permanece activa durante todo el ciclo de inspección
 *    - Distinguir entre detección momentánea (lecturas continuas del sensor) y
 *      resultado final (latch activado que determina la clasificación)
 *
 * 4. CONTROLAR SEMÁFORO PERIMETRAL VIA I2C
 *    - Comunicación master-esclavo con periférico Arduino mediante bus I2C
 *      (SDA en A4, SCL en A5) con dirección esclavo 0x32
 *    - Gestionar estados visuales del semáforo: amarillo titilante durante estado
 *      idle, amarillo fijo durante inspección, rojo cuando se detecta metal,
 *      verde cuando la pieza es aceptada
 *    - Temporización de exactamente 10 segundos para mostrar resultados mediante
 *      comparación de millis() con tiempo de inicio almacenado
 *
 * 5. IMPLEMENTAR SISTEMA DE DEBUG Y MONITOREO
 *    - Logging periódico de estados cada 2 segundos mediante función
 *      debugPeriodicoEstado() que muestra estado del robot, estado del sensor,
 *      y estado actual del supervisor
 *    - Detección de transiciones de estado del robot mediante comparación de
 *      estadoRobotActual con estadoRobotAnterior en cada iteración del loop
 *    - Reportes detallados para diagnóstico de fallos mediante comunicación
 *      serial a 9600 baudios con mensajes descriptivos de todas las transiciones
 *      y eventos críticos del sistema
 *
 * ============================================================================
 * CONFIGURACIÓN DEL HARDWARE
 * ============================================================================
 *
 * PLACA ARDUINO: Arduino Uno o compatible (ATmega328P)
 *
 * CONEXIONES FÍSICAS DETALLADAS:
 *
 * ROBOT NED2 ↔ ARDUINO SUPERVISOR:
 *   - DO2 (robot) → PIN 3 (D3) del Arduino: Estado MSB del robot
 *   - DO1 (robot) → PIN 2 (D2) del Arduino: Estado LSB del robot
 *   - DI2 (robot) ← PIN 12 (D12) del Arduino: Resultado MSB del sensor
 *   - DI1 (robot) ← PIN 11 (D11) del Arduino: Resultado LSB del sensor
 *   - DI3 (robot) ← PIN 10 (D10) del Arduino: Reservado para expansiones
 *   - GND común entre robot y Arduino (referencia común crítica)
 *
 * SENSOR INDUCTIVO:
 *   - Salida del sensor → PIN 7 (D7) del Arduino: Configurado como INPUT_PULLUP
 *   - Sensor activo en nivel LOW cuando detecta material ferroso
 *   - GND común entre sensor y Arduino
 *   - VCC del sensor conectado a alimentación adecuada (5V o 12V según especificaciones)
 *
 * BUS I2C (Supervisor ↔ Periférico):
 *   - SDA → PIN A4 del Arduino: Datos del bus I2C
 *   - SCL → PIN A5 del Arduino: Reloj del bus I2C
 *   - Pull-ups externos de 4.7kΩ recomendados entre SDA/SCL y VCC
 *   - GND común entre Supervisor y Periférico
 *
 * COMUNICACIÓN SERIAL (Debug):
 *   - Configurada a 9600 baudios para logging y monitoreo
 *   - Utilizada exclusivamente para diagnóstico, no para comunicación con robot
 *
 * ============================================================================
 * ARQUITECTURA DEL SISTEMA
 * ============================================================================
 *
 * ROBOT NED2 ↔ ARDUINO SUPERVISOR (Arduino Uno/Mega):
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │ ROBOT NED2              ARDUINO SUPERVISOR                               │
 * │ DO3 ──────────────────► PIN 1 (TX) - NO USAR, conflicto con Serial       │
 * │ DO2 ──────────────────► PIN 3 (Estado MSB) - Lectura estado robot        │
 * │ DO1 ──────────────────► PIN 2 (Estado LSB) - Lectura estado robot        │
 * │                                                                          │
 * │ DI1 ◄────────────────── PIN 11 (Resultado LSB) - Envío resultados        │
 * │ DI2 ◄────────────────── PIN 12 (Resultado MSB) - Envío resultados        │
 * │ DI3 ◄────────────────── PIN 10 (Reservado) - Futuras expansiones         │
 * │                                                                          │
 * │ SENSOR INDUCTIVO ─────► PIN 7 (INPUT_PULLUP) - Detección metal           │
 * │                                                                          │
 * │ SEMÁFORO (I2C 0x32) - Comunicación con periférico                        │
 * │ SDA ──────────────────► A4 (SDA) - Bus I2C datos                         │
 * │ SCL ──────────────────► A5 (SCL) - Bus I2C reloj                         │
 * │ GND ──────────────────► GND - Referencia común                           │
 * └──────────────────────────────────────────────────────────────────────────┘
 *
 * ============================================================================
 * PROTOCOLO DE COMUNICACIÓN FORMAL
 * ============================================================================
 *
 * PROTOCOLO DE ESTADOS (Robot → Supervisor vía DO2/DO1):
 * ┌─────────────┬─────────────┬─────────────────────────────────────┐
 * │ DO2 DO1     │ Decimal     │ Estado del Robot                   │
 * ├─────────────┼─────────────┼─────────────────────────────────────┤
 * │ 0   0       │ 0           │ INICIANDO - Robot arrancando        │
 * │ 0   1       │ 1           │ DISPONIBLE - Robot listo           │
 * │ 1   0       │ 2           │ OCUPADO - Robot en movimiento       │
 * │ 1   1       │ 3           │ EN_INSPECCIÓN - Inspección activa   │
 * └─────────────┴─────────────┴─────────────────────────────────────┘
 *
 * PROTOCOLO DE RESULTADOS (Supervisor → Robot vía DI2/DI1):
 * ┌─────────────┬─────────────┬─────────────────────────────────────┐
 * │ D12 D11     │ Decimal     │ Resultado de Clasificación          │
 * ├─────────────┼─────────────┼─────────────────────────────────────┤
 * │ 0   0       │ 0           │ SIN_COMANDO - Sin resultado         │
 * │ 0   1       │ 1           │ NO_METAL - Pieza no conductiva      │
 * │ 1   0       │ 2           │ RESERVADO - Para expansiones        │
 * │ 1   1       │ 3           │ METAL - Metal detectado             │
 * └─────────────┴─────────────┴─────────────────────────────────────┘
 *
 * ============================================================================
 * MÁQUINA DE ESTADOS DEL SUPERVISOR (SED - FORMAL)
 * ============================================================================
 *
 * DEFINICIÓN FORMAL DEL AUTÓMATA:
 *   Tupla extendida: (E, X, f, x₀, X_m, λ) donde λ es función de salida
 *
 * CONJUNTO DE EVENTOS: Σ = {e₁, e₂, e₃, e₄}
 *   e₁: Robot entra en inspección (DO2/DO1 = 11, estado ROBOT_EN_INSPECCION)
 *   e₂: Detección de metal (PIN 7 = LOW, sensor inductivo activo)
 *   e₃: Robot sale de inspección (DO2/DO1 ≠ 11, transición desde EN_INSPECCION)
 *   e₄: Timer 10 segundos expirado (millis() - tiempoInicioResultado >= 10000)
 *
 * CONJUNTO DE ESTADOS: Q = {q₀, q₁, q₂}
 *   q₀: SUP_IDLE - Esperando robot, semáforo amarillo titilando
 *   q₁: SUP_INSPECCIONANDO - Inspección activa, semáforo amarillo fijo
 *   q₂: SUP_MOSTRANDO_RESULTADO - Mostrando resultado por 10 segundos
 *
 * ESTADO INICIAL: q₀ (SUP_IDLE)
 * ESTADOS MARCADOS: {q₀} (único estado seguro donde sistema puede permanecer indefinidamente)
 *
 * FUNCIÓN DE TRANSICIÓN: δ: Q × E → Q
 *   δ(q₀, e₁) = q₁    // Robot entra en inspección, activación estado inspección
 *   δ(q₁, e₂) = q₁    // Metal detectado, latch activado, mantener estado
 *   δ(q₁, e₃) = q₂    // Robot sale de inspección, mostrar resultado
 *   δ(q₂, e₄) = q₀    // Timer expirado, retorno a estado idle
 *
 * FUNCIÓN DE SALIDA: λ: Q → Y (control del semáforo)
 *   λ(q₀) = YELLOW_BLINK    // Amarillo titilante, sistema disponible
 *   λ(q₁) = YELLOW_SOLID    // Amarillo fijo, inspección en curso
 *   λ(q₂) = RED si metalDetectado == true, GREEN si metalDetectado == false
 *
 * ============================================================================
 * COMANDOS I2C PARA SEMÁFORO PERIMETRAL
 * ============================================================================
 *
 * El supervisor envía comandos al periférico mediante comunicación I2C hacia
 * dirección esclava 0x32. Los comandos se codifican en un solo byte:
 *
 * ┌─────────────┬─────────────┬─────────────────────────────────────┐
 * │ Comando     │ Valor       │ Descripción                         │
 * ├─────────────┼─────────────┼─────────────────────────────────────┤
 * │ LAMP_OFF    │ 0           │ Apagar todos los LEDs               │
 * │ YELLOW_BLINK│ 1           │ Amarillo titilante (400ms)          │
 * │ YELLOW_SOLID│ 2           │ Amarillo fijo                       │
 * │ LAMP_RED    │ 3           │ Rojo (metal detectado)              │
 * │ LAMP_GREEN  │ 4           │ Verde (pieza aceptada)              │
 * └─────────────┴─────────────┴─────────────────────────────────────┘
 *
 * ============================================================================
 * LÓGICA IMPLEMENTADA
 * ============================================================================
 *
 * El firmware implementa un bucle principal (loop()) que se ejecuta continuamente
 * con frecuencia aproximada de 20 Hz (cada 50ms debido al delay final). En cada
 * iteración del bucle se ejecutan las siguientes operaciones en orden:
 *
 * 1. LECTURA DE ESTADO DEL ROBOT:
 *    - Se leen los pines DO2 (PIN 3) y DO1 (PIN 2) mediante digitalRead()
 *    - Se decodifica el valor binario a decimal 0-3 mediante operaciones bitwise
 *    - Se mapea el valor decimal al enumerado RobotState correspondiente
 *
 * 2. LECTURA DEL SENSOR INDUCTIVO:
 *    - Se lee el PIN 7 mediante digitalRead() y se compara con SENSOR_DETECTA (LOW)
 *    - El resultado se almacena en variable booleana sensorMetal para procesamiento
 *
 * 3. PROCESAMIENTO DE MÁQUINA DE ESTADOS:
 *    - Se llama a procesarMaquinaEstados(sensorMetal) que evalúa el estado actual
 *      del supervisor y ejecuta la lógica correspondiente según el estado
 *    - Se detectan transiciones de estado mediante comparación de estadoRobotActual
 *      con estadoRobotAnterior
 *    - Se actualiza el semáforo mediante comunicación I2C según la función de salida λ
 *
 * 4. DEBUG PERIÓDICO:
 *    - Se llama a debugPeriodicoEstado() si ha transcurrido el intervalo configurado
 *      (2 segundos por defecto) para mostrar información de diagnóstico en serial
 *
 * 5. ACTUALIZACIÓN DE ESTADO ANTERIOR:
 *    - Se guarda el estado actual del robot en estadoRobotAnterior para detección
 *      de transiciones en la siguiente iteración
 *
 * 6. DELAY DE CONTROL:
 *    - Se ejecuta delay(50) para controlar la frecuencia del bucle y evitar
 *      saturación del procesador
 *
 * ============================================================================
 */

#include <Wire.h>

// ============================================================================
// CONFIGURACIÓN I2C
// ============================================================================
const uint8_t SEMAFORO_I2C_ADDR = 0x32;

// Comandos para el semáforo
enum LampCommand : uint8_t {
  LAMP_OFF          = 0,
  LAMP_YELLOW_BLINK = 1,
  LAMP_YELLOW_SOLID = 2,
  LAMP_RED          = 3,
  LAMP_GREEN        = 4,
};

// ============================================================================
// PINES - ENTRADAS DESDE ROBOT
// ============================================================================
const int PIN_ROBOT_DO2 = 3;   // Estado MSB desde robot
const int PIN_ROBOT_DO1 = 2;   // Estado LSB desde robot
// Nota: DO3 en PIN 1 no se usa (conflicto con Serial TX)

// ============================================================================
// PINES - SALIDAS HACIA ROBOT
// ============================================================================
const int PIN_ROBOT_DI2 = 12;  // Resultado MSB hacia robot
const int PIN_ROBOT_DI1 = 11;  // Resultado LSB hacia robot
const int PIN_ROBOT_DI3 = 10;  // Reservado

// ============================================================================
// SENSOR INDUCTIVO
// ============================================================================
const int PIN_SENSOR_METAL = 7;
const int SENSOR_DETECTA = LOW;  // Activo en bajo

// ============================================================================
// ESTADOS DEL ROBOT (lectura desde DO2/DO1)
// ============================================================================
enum RobotState : uint8_t {
  ROBOT_INICIANDO     = 0,  // 00
  ROBOT_DISPONIBLE    = 1,  // 01
  ROBOT_OCUPADO       = 2,  // 10
  ROBOT_EN_INSPECCION = 3,  // 11
};

// ============================================================================
// CÓDIGOS DE RESULTADO (escritura hacia DI2/DI1)
// ============================================================================
enum ResultCode : uint8_t {
  RESULT_NONE     = 0,  // 00 - Sin resultado
  RESULT_NO_METAL = 1,  // 01 - Pieza limpia
  RESULT_RESERVED = 2,  // 10 - Reservado
  RESULT_METAL    = 3,  // 11 - Metal detectado
};

// ============================================================================
// MÁQUINA DE ESTADOS DEL SUPERVISOR
// ============================================================================
enum SupervisorState : uint8_t {
  SUP_IDLE,              // Esperando robot
  SUP_INSPECCIONANDO,    // Robot en zona, monitoreando sensor
  SUP_MOSTRANDO_RESULTADO, // Mostrando rojo/verde por 4s
};

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================
SupervisorState estadoSupervisor = SUP_IDLE;
RobotState estadoRobotActual = ROBOT_INICIANDO;
RobotState estadoRobotAnterior = ROBOT_INICIANDO;

// Detección de metal con latch
bool metalDetectado = false;

// Timer para mostrar resultado
unsigned long tiempoInicioResultado = 0;
const unsigned long DURACION_RESULTADO_MS = 10000;  // 10 segundos

// Debug throttling
unsigned long ultimoDebugEstado = 0;
const unsigned long INTERVALO_DEBUG_MS = 2000;

// ============================================================================
// FUNCIÓN: CONFIGURACIÓN INICIAL DEL SISTEMA
// ============================================================================
// OBJETIVO: Inicializar todos los componentes del supervisor Arduino
// COMPONENTES: Pines digitales, I2C, semáforo inicial, banner informativo
// IMPORTANCIA: Configuración crítica para operación correcta del sistema
// ============================================================================
void setup() {
  Serial.begin(9600);
  
  // Configurar pines de entrada desde robot
  pinMode(PIN_ROBOT_DO2, INPUT);  // Sin pullup - el robot debe manejar el nivel
  pinMode(PIN_ROBOT_DO1, INPUT);  // Sin pullup
  
  // Configurar pines de salida hacia robot
  pinMode(PIN_ROBOT_DI2, OUTPUT);
  pinMode(PIN_ROBOT_DI1, OUTPUT);
  pinMode(PIN_ROBOT_DI3, OUTPUT);
  
  // Sensor inductivo
  pinMode(PIN_SENSOR_METAL, INPUT_PULLUP);
  
  // Inicializar salidas
  enviarResultadoARobot(RESULT_NO_METAL);  // Por defecto: no metal
  
  // Iniciar I2C como Supervisor
  Wire.begin();
  
  // Banner de inicio
  Serial.println(F(""));
  Serial.println(F("╔════════════════════════════════════════════╗"));
  Serial.println(F("║  OCEANIX SUPERVISOR v5.0 - Supervisor REAL    ║"));
  Serial.println(F("╠════════════════════════════════════════════╣"));
  Serial.println(F("║  Entradas: D3(DO2), D2(DO1)                ║"));
  Serial.println(F("║  Salidas:  D12(DI2), D11(DI1), D10(DI3)    ║"));
  Serial.println(F("║  Sensor:   D7                              ║"));
  Serial.println(F("║  I2C:      0x32 (Semáforo)                 ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.println(F(""));
  
  // Semáforo inicial
  enviarComandoSemaforo(LAMP_YELLOW_BLINK);
  Serial.println(F("[INIT] Semaforo -> Amarillo titilando"));
  Serial.println(F("[INIT] Esperando robot..."));
  Serial.println(F(""));
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
void loop() {
  // 1. Leer estado actual del robot
  estadoRobotActual = leerEstadoRobot();
  
  // 2. Leer sensor inductivo
  bool sensorMetal = (digitalRead(PIN_SENSOR_METAL) == SENSOR_DETECTA);
  
  // 3. Procesar máquina de estados
  procesarMaquinaEstados(sensorMetal);
  
  // 4. Debug periódico
  debugPeriodicoEstado(sensorMetal);
  
  // 5. Guardar estado anterior
  estadoRobotAnterior = estadoRobotActual;
  
  delay(50);
}

// ============================================================================
// LEER ESTADO DEL ROBOT
// ============================================================================
RobotState leerEstadoRobot() {
  int do2 = digitalRead(PIN_ROBOT_DO2);
  int do1 = digitalRead(PIN_ROBOT_DO1);
  
  uint8_t estado = ((do2 == HIGH ? 1 : 0) << 1) | (do1 == HIGH ? 1 : 0);
  return static_cast<RobotState>(estado);
}

// ============================================================================
// ENVIAR RESULTADO AL ROBOT
// ============================================================================
void enviarResultadoARobot(ResultCode resultado) {
  int di2 = (resultado & 0x02) ? HIGH : LOW;  // Bit 1 (MSB)
  int di1 = (resultado & 0x01) ? HIGH : LOW;  // Bit 0 (LSB)
  
  digitalWrite(PIN_ROBOT_DI2, di2);
  digitalWrite(PIN_ROBOT_DI1, di1);
}

// ============================================================================
// FUNCIÓN: MÁQUINA DE ESTADOS FINITA - CORAZÓN DEL SUPERVISOR
// ============================================================================
// OBJETIVO: Implementar lógica de control basada en teoría de autómatas
// ALGORITMO: Evaluación de estados y eventos con transiciones deterministas
// CRÍTICO: Esta función determina todo el comportamiento del sistema
// ============================================================================
void procesarMaquinaEstados(bool sensorMetal) {
  
  switch (estadoSupervisor) {
    
    // =========================================
    // ESTADO IDLE: Esperando que robot entre en inspección
    // =========================================
    case SUP_IDLE:
      // Publicar continuamente el estado del sensor al robot
      if (sensorMetal) {
        enviarResultadoARobot(RESULT_METAL);
      } else {
        enviarResultadoARobot(RESULT_NO_METAL);
      }
      
      // Detectar entrada en inspección
      if (estadoRobotActual == ROBOT_EN_INSPECCION && 
          estadoRobotAnterior != ROBOT_EN_INSPECCION) {
        
        Serial.println(F(""));
        Serial.println(F("┌─────────────────────────────────────┐"));
        Serial.println(F("│  ROBOT ENTRÓ EN ZONA DE INSPECCIÓN  │"));
        Serial.println(F("└─────────────────────────────────────┘"));
        
        // Transición a estado inspección
        estadoSupervisor = SUP_INSPECCIONANDO;
        metalDetectado = false;
        
        // Semáforo amarillo fijo
        enviarComandoSemaforo(LAMP_YELLOW_SOLID);
        Serial.println(F("[SEMAFORO] Amarillo FIJO"));
      }
      break;
    
    // =========================================
    // ESTADO INSPECCIONANDO: Monitoreando sensor
    // =========================================
    case SUP_INSPECCIONANDO:
      // Publicar continuamente el estado del sensor al robot
      if (sensorMetal) {
        enviarResultadoARobot(RESULT_METAL);
        
        // Latch de detección
        if (!metalDetectado) {
          metalDetectado = true;
          Serial.println(F("[LATCH] ¡METAL DETECTADO!"));
        }
      } else {
        enviarResultadoARobot(RESULT_NO_METAL);
      }
      
      // Detectar salida de inspección
      if (estadoRobotActual != ROBOT_EN_INSPECCION && 
          estadoRobotAnterior == ROBOT_EN_INSPECCION) {
        
        Serial.println(F(""));
        Serial.println(F("┌─────────────────────────────────────┐"));
        Serial.println(F("│  ROBOT SALIÓ DE ZONA DE INSPECCIÓN  │"));
        Serial.println(F("└─────────────────────────────────────┘"));
        
        // Transición a mostrar resultado
        estadoSupervisor = SUP_MOSTRANDO_RESULTADO;
        tiempoInicioResultado = millis();
        
        // Mostrar resultado en semáforo
        if (metalDetectado) {
          enviarComandoSemaforo(LAMP_RED);
          Serial.println(F("[RESULTADO] >>> METAL <<< -> ROJO"));
        } else {
          enviarComandoSemaforo(LAMP_GREEN);
          Serial.println(F("[RESULTADO] >>> NO METAL <<< -> VERDE"));
        }
        Serial.println(F("[TIMER] Mostrando resultado por 10 segundos..."));
      }
      break;
    
    // =========================================
    // ESTADO MOSTRANDO RESULTADO: Timer de 4 segundos
    // =========================================
    case SUP_MOSTRANDO_RESULTADO:
      // Mantener el último resultado hacia el robot
      if (metalDetectado) {
        enviarResultadoARobot(RESULT_METAL);
      } else {
        enviarResultadoARobot(RESULT_NO_METAL);
      }
      
      // Timer de 4 segundos
      if (millis() - tiempoInicioResultado >= DURACION_RESULTADO_MS) {
        Serial.println(F(""));
        Serial.println(F("[TIMER] 10 segundos cumplidos"));
        
        // Transición a IDLE
        estadoSupervisor = SUP_IDLE;
        metalDetectado = false;
        
        // Semáforo vuelve a amarillo titilando
        enviarComandoSemaforo(LAMP_YELLOW_BLINK);
        Serial.println(F("[SEMAFORO] Amarillo TITILANDO"));
        Serial.println(F("[SUPERVISOR] Listo para siguiente ciclo"));
        Serial.println(F(""));
      }
      break;
  }
  
  // Log de cambio de estado del robot (independiente de máquina de estados)
  if (estadoRobotActual != estadoRobotAnterior) {
    Serial.print(F("[ROBOT] Estado: "));
    Serial.print(estadoRobotAnterior);
    Serial.print(F(" -> "));
    Serial.print(estadoRobotActual);
    Serial.print(F(" ("));
    Serial.print(nombreEstadoRobot(estadoRobotActual));
    Serial.println(F(")"));
  }
}

// ============================================================================
// DEBUG PERIÓDICO
// ============================================================================
void debugPeriodicoEstado(bool sensorMetal) {
  if (millis() - ultimoDebugEstado >= INTERVALO_DEBUG_MS) {
    ultimoDebugEstado = millis();
    
    int do2 = digitalRead(PIN_ROBOT_DO2);
    int do1 = digitalRead(PIN_ROBOT_DO1);
    
    Serial.print(F("[DEBUG] D3="));
    Serial.print(do2);
    Serial.print(F(" D2="));
    Serial.print(do1);
    Serial.print(F(" -> Robot="));
    Serial.print(estadoRobotActual);
    Serial.print(F(" | Sensor="));
    Serial.print(sensorMetal ? F("METAL") : F("limpio"));
    Serial.print(F(" | Sup="));
    Serial.println(nombreEstadoSupervisor(estadoSupervisor));
  }
}

// ============================================================================
// NOMBRES DE ESTADOS (para debug)
// ============================================================================
const char* nombreEstadoRobot(RobotState estado) {
  switch (estado) {
    case ROBOT_INICIANDO:     return "INICIANDO";
    case ROBOT_DISPONIBLE:    return "DISPONIBLE";
    case ROBOT_OCUPADO:       return "OCUPADO";
    case ROBOT_EN_INSPECCION: return "EN_INSPECCION";
    default:                  return "???";
  }
}

const char* nombreEstadoSupervisor(SupervisorState estado) {
  switch (estado) {
    case SUP_IDLE:               return "IDLE";
    case SUP_INSPECCIONANDO:     return "INSPECCIONANDO";
    case SUP_MOSTRANDO_RESULTADO: return "RESULTADO";
    default:                     return "???";
  }
}

// ============================================================================
// ENVIAR COMANDO AL SEMÁFORO (I2C)
// ============================================================================
void enviarComandoSemaforo(LampCommand cmd) {
  Wire.beginTransmission(SEMAFORO_I2C_ADDR);
  Wire.write(static_cast<uint8_t>(cmd));
  uint8_t err = Wire.endTransmission();
  
  Serial.print(F("[I2C] Enviando CMD="));
  Serial.print(static_cast<uint8_t>(cmd));
  if (err == 0) {
    Serial.println(F(" -> OK"));
  } else {
    Serial.print(F(" -> ERROR ("));
    Serial.print(err);
    Serial.println(F(")"));
  }
}
