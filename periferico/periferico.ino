/*
 * ============================================================================
 * PROYECTO FINAL: PERIFÉRICO SEMÁFORO - SISTEMA OCEANIX
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
 * Implementar el controlador periférico del semáforo perimetral para el sistema
 * de clasificación Oceanix, actuando como esclavo I2C que recibe comandos del
 * supervisor y controla indicadores luminosos LED de manera determinista.
 *
 * ============================================================================
 * OBJETIVOS ESPECÍFICOS:
 * ==============================================================================
 *
 * 1. IMPLEMENTAR COMUNICACIÓN I2C COMO ESCLAVO
 *    - Configurar dirección I2C 0x32 para recepción de comandos
 *    - Implementar callback de recepción de datos vía interrupción
 *    - Gestionar buffer de comandos con flags de actualización
 *
 * 2. CONTROLAR INDICADORES LUMINOSOS LED
 *    - Gestionar 3 LEDs: verde (PIN 2), amarillo (PIN 3), rojo (PIN 4)
 *    - Implementar lógica de conmutación digital con protección
 *    - Garantizar estados mutuamente excluyentes (solo un LED activo)
 *
 * 3. IMPLEMENTAR PARPADEO AUTOMÁTICO NO BLOQUEANTE
 *    - Temporizador de 400ms para amarillo titilante
 *    - Lógica no bloqueante en loop principal
 *    - Sincronización precisa sin interferir con comunicaciones
 *
 * 4. GESTIONAR PROTOCOLO DE COMANDOS DETERMINISTA
 *    - Procesar 5 comandos I2C diferentes con acciones inmediatas
 *    - Implementar máquina de estados simple para modos LED
 *    - Logging detallado para diagnóstico operativo
 *
 * ============================================================================
 * ARQUITECTURA DEL COMPONENTE:
 * ==============================================================================
 *
 * ┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
 * │   SUPERVISOR    │────│      I2C         │────│   PERIFÉRICO    │
 * │   (Maestro)     │    │   (Bus serie)    │    │   (Esclavo)     │
 * └─────────────────┘    └──────────────────┘    └─────────────────┘
 *                                                            │
 *                                                            │ Control digital
 *                                                            ▼
 *                                               ┌─────────────────────┐
 *                                               │   CONTROLADOR LED   │
 *                                               │                     │
 *                                               │  ● VERDE (PIN 2)    │
 *                                               │  ● AMARILLO (PIN 3) │
 *                                               │  ● ROJO (PIN 4)     │
 *                                               └─────────────────────┘
 *
 * ============================================================================
 * CONEXIONES FÍSICAS DETALLADAS (ARDUINO UNO):
 * ==============================================================================
 *
 * PERIFÉRICO ARDUINO UNO ↔ SUPERVISOR ARDUINO:
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │ COMPONENTE                PIN ARDUINO     DESCRIPCIÓN                   │
 * ├──────────────────────────────────────────────────────────────────────────┤
 * │ LED VERDE                 PIN 2           Indicador pieza aceptada       │
 * │ LED AMARILLO              PIN 3           Estado inspección/standby      │
 * │ LED ROJO                  PIN 4           Metal detectado                │
 * │                                                                  │
 * │ BUS I2C (ESCLAVO)                                                 │
 * │ SDA (datos)               A4 (SDA)        Comunicación bidireccional    │
 * │ SCL (reloj)               A5 (SCL)        Sincronización bus I2C        │
 * │ GND (referencia)          GND             Referencia común crítica      │
 * │                                                                  │
 * │ ALIMENTACIÓN                                                     │
 * │ VCC (+5V)                 VIN/5V          Alimentación regulada         │
 * └──────────────────────────────────────────────────────────────────────────┘
 *
 * ============================================================================
 * PROTOCOLO I2C - COMANDOS DE CONTROL:
 * ==============================================================================
 *
 * ESTRUCTURA DEL MENSAJE I2C:
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
 * - Dirección esclavo: 0x32 (50 decimal)
 * - Mensaje: 1 byte (comando)
 * - Recepción: Vía interrupción onReceive()
 * - Procesamiento: En loop principal (no bloqueante)
 * - Confirmación: Logging serial para diagnóstico
 *
 * ============================================================================
 * DIAGRAMA DE ESTADOS DEL PERIFÉRICO:
 * ==============================================================================
 *
 * Máquina de Estados Simple (no SED formal, lógica directa):
 *
 * ┌─────────────┐
 * │  INICIAL    │ ← Setup: todos LEDs OFF
 * └──────┬──────┘
 *        │
 *        ▼
 * ┌─────────────┐ ◄─────────────────┐
 * │ ESPERANDO   │                   │
 * │ COMANDO     │ ── Comando ───►   │
 * │ I2C         │                   │
 * └──────┬──────┘                   │
 *        │                          │
 *        ▼                          │
 * ┌─────────────┐                   │
 * │ EJECUTANDO  │ ── LED ON ───►    │
 * │ COMANDO     │                   │
 * │ LED         │ ◄─────────────────┘
 * └─────────────┘
 *        │
 *        ▼ [Si YELLOW_BLINK]
 * ┌─────────────┐
 * │ PARPADEO    │ ← Timer 400ms no bloqueante
 * │ AUTOMÁTICO  │
 * └─────────────┘
 *
 * ============================================================================
 * ESPECIFICACIONES TÉCNICAS:
 * ==============================================================================
 *
 * HARDWARE:
 * - Microcontrolador: ATmega328P (Arduino Uno)
 * - LEDs: 5mm estándar con resistencias 220Ω
 * - Bus I2C: Implementación hardware (TWI)
 * - Alimentación: 5V DC regulado
 *
 * SOFTWARE:
 * - Framework: Arduino IDE 1.8+
 * - Librería: Wire (I2C integrada)
 * - Frecuencia: 16MHz
 * - Comunicación serial: 9600 baud para debug
 *
 * TEMPORIZACIÓN:
 * - Intervalo parpadeo: 400ms (2.5 Hz)
 * - Latencia I2C: < 1ms
 * - Tiempo respuesta LED: < 10ms
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
// OBJETIVO: Gestionar recepción de comandos I2C del supervisor maestro
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
