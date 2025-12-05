/*
 * ============================================================================
 * PROYECTO FINAL: SUPERVISOR ARDUINO - SISTEMA OCEANIX
 * ============================================================================
 *
 * INSTITUCIÓN: Universidad Intercontinental de la Empresa
 * ASIGNATURA: Tecnología de Automatización y Robotización Empresarial
 * PROFESOR: Eladio Dapena
 * FECHA: 5 de diciembre de 2025
 *
 * EQUIPO DE DESARROLLO:
 * - Alan Ariel Salazar
 * - Anton Dopico
 * - Pablo Barran Franco
 *
 * ============================================================================
 *
 * OBJETIVO GENERAL:
 * ==============================================================================
 * Implementar el controlador maestro del sistema de clasificación Oceanix,
 * actuando como cerebro central que coordina robot Ned2, sensor inductivo
 * y semáforo perimetral mediante protocolos de comunicación síncronos.
 *
 * ============================================================================
 * OBJETIVOS ESPECÍFICOS:
 * ==============================================================================
 *
 * 1. IMPLEMENTAR MÁQUINA DE ESTADOS FINITA PARA SUPERVISIÓN
 *    - Gestionar tres estados principales: IDLE, INSPECCIONANDO, MOSTRANDO_RESULTADO
 *    - Implementar lógica de transición basada en señales del robot
 *    - Gestionar temporizadores para estados con duración fija
 *
 * 2. DESARROLLAR PROTOCOLO DE COMUNICACIÓN BIDIRECCIONAL
 *    - Leer estados del robot mediante pines digitales DO2/DO1
 *    - Enviar resultados de clasificación vía pines DI2/DI1
 *    - Implementar protocolo 2-bit síncrono con validación
 *
 * 3. INTEGRAR SENSOR INDUCTIVO CON LÓGICA DE LATCHING
 *    - Monitoreo continuo del sensor en PIN 7 (activo bajo)
 *    - Implementar latching durante fase de inspección
 *    - Distinguir entre detección momentánea y resultado final
 *
 * 4. CONTROLAR SEMÁFORO PERIMETRAL VIA I2C
 *    - Comunicación maestro-esclavo con periférico Arduino
 *    - Gestionar estados visuales: amarillo titilante/fijo, rojo, verde
 *    - Temporización de 10 segundos para mostrar resultados
 *
 * 5. IMPLEMENTAR SISTEMA DE DEBUG Y MONITOREO
 *    - Logging periódico de estados cada 2 segundos
 *    - Detección de transiciones de estado del robot
 *    - Reportes detallados para diagnóstico de fallos
 *
 * ============================================================================
 * ARQUITECTURA DEL SISTEMA:
 * ==============================================================================
 *
 * ┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
 * │   ROBOT NED2    │────│  SUPERVISOR      │────│   PERIFÉRICO    │
 * │   (Periférico)  │    │  (Maestro I2C)   │    │   (Esclavo)     │
 * └─────────────────┘    └──────────────────┘    └─────────────────┘
 *         │                        │                        │
 *         │ DO2/DO1 (Estados)      │ I2C (Comandos LED)     │
 *         │                        │                        │
 *         ▼ DI2/DI1 (Resultados)   ▼                        ▼
 *    ┌─────────────┐          ┌─────────────┐          ┌─────────────┐
 *    │  SENSOR      │          │  SEMÁFORO   │          │   CONTROL    │
 *    │ INDUCTIVO    │          │ PERIMETRAL │          │   LED        │
 *    └─────────────┘          └─────────────┘          └─────────────┘
 *
 * ============================================================================
 * CONEXIONES FÍSICAS DETALLADAS:
 * ==============================================================================
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
 * PROTOCOLO DE COMUNICACIÓN FORMAL:
 * ==============================================================================
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
 * MÁQUINA DE ESTADOS DEL SUPERVISOR (SED - FORMAL):
 * ==============================================================================
 *
 * DEFINICIÓN FORMAL DEL AUTÓMATA:
 * Σ = {e₁, e₂, e₃, e₄} donde:
 *   e₁: Robot entra en inspección (DO2/DO1 = 11)
 *   e₂: Detección de metal (PIN 7 = LOW)
 *   e₃: Robot sale de inspección (DO2/DO1 ≠ 11)
 *   e₄: Timer 10 segundos expirado
 *
 * ESTADOS: Q = {q₀, q₁, q₂}
 *   q₀: SUP_IDLE - Esperando robot, semáforo amarillo titilando
 *   q₁: SUP_INSPECCIONANDO - Inspección activa, semáforo amarillo fijo
 *   q₂: SUP_MOSTRANDO_RESULTADO - Mostrando resultado por 10 segundos
 *
 * ESTADO INICIAL: q₀
 * ESTADOS MARCADOS: {q₀} (estado seguro)
 *
 * FUNCIÓN DE TRANSICIÓN:
 * δ(q₀, e₁) = q₁    // Robot entra en inspección
 * δ(q₁, e₃) = q₂    // Robot sale de inspección
 * δ(q₂, e₄) = q₀    // Timer expirado, vuelta a idle
 *
 * FUNCIÓN DE SALIDA (Semáforo):
 * λ(q₀) = YELLOW_BLINK    // Estado idle
 * λ(q₁) = YELLOW_SOLID    // Inspección activa
 * λ(q₂) = RED/GREEN       // Resultado según detección
 *
 * ============================================================================
 * COMANDOS I2C PARA SEMÁFORO PERIMETRAL:
 * ==============================================================================
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
  
  // Iniciar I2C como maestro
  Wire.begin();
  
  // Banner de inicio
  Serial.println(F(""));
  Serial.println(F("╔════════════════════════════════════════════╗"));
  Serial.println(F("║  OCEANIX SUPERVISOR v5.0 - MAESTRO REAL    ║"));
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
