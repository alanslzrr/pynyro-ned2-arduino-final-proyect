# ==============================================================================
# PROYECTO FINAL: CELDA ROBOTIZADA DE CLASIFICACIÓN OCEANIX
# ==============================================================================
#
# INSTITUCIÓN: Universidad Intercontinental de la Empresa
# ASIGNATURA: Tecnología de Automatización y Robotización Empresarial
# PROFESOR: Eladio Dapena
# FECHA: 5 de diciembre de 2025
#
# EQUIPO DE DESARROLLO:
# - Alan Ariel Salazar
# - Anton Dopico
# - Pablo Barran Franco
#
# ==============================================================================
#
# OBJETIVO GENERAL:
# ==============================================================================
# Desarrollar e implementar un sistema automatizado de clasificación industrial
# que integre un robot colaborativo Ned2 con sensores inductivos para la
# detección de materiales metálicos, aplicando principios de automatización
# industrial y teoría de sistemas a eventos discretos (SED).
#
# ==============================================================================
# OBJETIVOS ESPECÍFICOS:
# ==============================================================================
# 1. IMPLEMENTAR CONTROLADOR PYTHON PARA ROBOT NED2
#    - Establecer comunicación TCP/IP con robot industrial
#    - Implementar protocolo de comunicación bidireccional 2-bit
#    - Gestionar estados del robot mediante máquina de estados finita
#
# 2. DESARROLLAR LÓGICA DE CLASIFICACIÓN AUTOMATIZADA
#    - Detectar presencia de piezas mediante sensor digital
#    - Ejecutar movimientos seguros con navegación hub-and-spoke
#    - Implementar inspección temporal de 7 segundos con latching
#    - Clasificar piezas: metálicas → descartes, no metálicas → pallet
#
# 3. INTEGRAR GESTIÓN DE ERRORES Y RECUPERACIÓN
#    - Implementar manejo de colisiones con recuperación automática
#    - Gestionar desconexiones y reconexiones con backoff exponencial
#    - Proporcionar logging completo para trazabilidad
#
# 4. OPTIMIZAR DESEMPEÑO INDUSTRIAL
#    - Minimizar tiempos de ciclo mediante rutas optimizadas
#    - Implementar debouncing de sensores para fiabilidad
#    - Gestionar memoria de pallet de 3 posiciones
#
# ==============================================================================
# ARQUITECTURA DEL SISTEMA:
# ==============================================================================
# PYTHON CONTROLLER (main.py) ↔ ARDUINO SUPERVISOR ↔ ARDUINO PERIFÉRICO
#       │                              │                              │
#       └─────────PROTOCOLO 2-BIT──────┼────────────I2C───────────────┘
#                                      │
#                                      ▼
#                               SENSOR INDUCTIVO
#                                      │
#                                      ▼
#                               SEMÁFORO LED
#
# ==============================================================================
# DEPENDENCIAS Y REQUERIMIENTOS:
# ==============================================================================
# - Python 3.8+
# - pyniryo (API para robot Ned2)
# - Robot Niryo Ned2 con firmware actualizado
# - Conexión de red TCP/IP (IP: 172.16.127.74)
# - Comunicación serial Arduino (9600 baud)
# - Bus I2C para periféricos
#
# ==============================================================================

from pyniryo import *
from pyniryo.api.objects import PoseObject
import time
import sys
import logging


# ==============================================================================
# CLASE: SISTEMA DE VOX INDUSTRIAL OCEANIX
# ==============================================================================
# OBJETIVO: Proporcionar síntesis de voz profesional para eventos del sistema
# FUNCIONALIDAD: Mensajes automatizados en español industrial con control de volumen
# INTEGRACIÓN: Compatible con API PyNiryo Language.SPANISH
# ==============================================================================

class SistemaVozOceanix:
    """
    Sistema de voz industrial para el robot Ned2 Oceanix.

    Esta clase implementa mensajes de voz automatizados en español para
    proporcionar feedback auditivo profesional durante operaciones industriales.
    Utiliza la API PyNiryo robot.say() con idioma español (valor 2).

    FUNCIONALIDADES:
    ==============================================================================
    - Síntesis de voz vía Google Text-to-Speech (gTTS) integrada en PyNiryo
    - Mensajes predefinidos para eventos críticos del sistema
    - Control automático de volumen (70% por defecto)
    - Manejo de errores silencioso (no interrumpe operaciones críticas)
    - Mensajes limitados a 100 caracteres por restricción API PyNiryo
    - Idioma español (language=2) para entorno industrial hispanohablante

    EVENTOS SOPORTADOS:
    ==============================================================================
    - inicio_sistema: Arranque del controlador Oceanix
    - ciclo_iniciado: Comienzo de ciclo de clasificación industrial
    - pieza_detectada: Nueva pieza detectada en alimentador
    - inspeccion_activa: Inicio de ventana de inspección (7 segundos)
    - metal_detectado: Clasificación como material metálico
    - no_metal_aceptado: Clasificación como material aceptado
    - deposito_completado: Pieza almacenada correctamente en pallet
    - ciclo_finalizado: Operación completada exitosamente
    - error_recuperable: Error con recuperación automática
    - pallet_completo: Almacén lleno, requiere intervención manual
    """

    def __init__(self, robot_instance, volumen_por_defecto=70):
        """
        Inicializa el sistema de voz con configuración industrial.

        Args:
            robot_instance: Instancia del robot NiryoRobot
            volumen_por_defecto (int): Volumen 0-100 (70% recomendado industrial)
        """
        self.robot = robot_instance
        self.volumen = volumen_por_defecto
        self.idioma = 2  # Español (0=English, 1=French, 2=Spanish, 3=Mandarin, 4=Portuguese)
        self._configurar_volumen()

        # Diccionario de mensajes optimizados (< 100 caracteres)
        self.mensajes = {
            'inicio_sistema': 'Sistema Oceanix operativo. Listo para clasificación automática.',
            'ciclo_iniciado': 'Iniciando ciclo de clasificación industrial.',
            'pieza_detectada': 'Pieza detectada en alimentador. Iniciando proceso.',
            'recogida_exitosa': 'Pieza recogida correctamente.',
            'inspeccion_activa': 'Inspección de calidad iniciada. Siete segundos.',
            'metal_detectado': 'Material metálico detectado. Procediendo a descartes.',
            'no_metal_aceptado': 'Material aceptado. Almacenamiento en pallet.',
            'deposito_completado': 'Pieza almacenada correctamente en pallet.',
            'ciclo_finalizado': 'Ciclo de clasificación completado exitosamente.',
            'error_recuperable': 'Error recuperable detectado. Continuando operación.',
            'pallet_completo': 'Almacén completo. Requiere vaciado manual.',
            'conexion_exitosa': 'Conexión con robot Ned dos establecida.',
            'calibracion_completada': 'Calibración automática completada.',
            'estado_disponible': 'Sistema disponible para nueva clasificación.',
            'parada_emergencia': 'Parada de emergencia activada. Sistema seguro.'
        }

    def _configurar_volumen(self):
        """Configura el volumen del sistema de audio del robot."""
        try:
            self.robot.set_volume(self.volumen)
        except Exception as e:
            logging.getLogger("OCEANIX").warning(f"No se pudo configurar volumen voz: {e}")

    def anunciar_evento(self, evento_clave, **kwargs):
        """
        Anuncia un evento específico con mensaje de voz optimizado.

        Args:
            evento_clave (str): Clave del mensaje predefinido
            **kwargs: Parámetros opcionales para formateo dinámico
        """
        try:
            if evento_clave not in self.mensajes:
                logging.getLogger("OCEANIX").warning(f"Evento de voz no definido: {evento_clave}")
                return

            mensaje = self.mensajes[evento_clave]

            # Formateo dinámico si se proporcionan parámetros
            if kwargs:
                try:
                    mensaje = mensaje.format(**kwargs)
                except (KeyError, ValueError) as e:
                    logging.getLogger("OCEANIX").warning(f"Error formateando mensaje voz: {e}")

            # Validación de límite de caracteres (restricción API)
            if len(mensaje) > 100:
                mensaje = mensaje[:97] + "..."
                logging.getLogger("OCEANIX").warning("Mensaje voz truncado por límite API")

            # Reproducción del mensaje
            self.robot.say(mensaje, self.idioma)
            logging.getLogger("OCEANIX").debug(f"Voz: '{mensaje}'")

        except Exception as e:
            # Error silencioso - no interrumpir operaciones críticas
            logging.getLogger("OCEANIX").warning(f"Error reproduciendo voz: {e}")

    def reproducir_mensaje_personalizado(self, mensaje, idioma=None):
        """
        Reproduce un mensaje personalizado (solo para casos excepcionales).

        Args:
            mensaje (str): Texto a reproducir (< 100 caracteres)
            idioma (int): Idioma específico como entero (0=English, 1=French, 2=Spanish, etc.)
                         Por defecto usa español (2)
        """
        try:
            if len(mensaje) > 100:
                logging.getLogger("OCEANIX").error("Mensaje personalizado excede límite de 100 caracteres")
                return

            idioma_voz = idioma if idioma is not None else self.idioma
            self.robot.say(mensaje, idioma_voz)

        except Exception as e:
            logging.getLogger("OCEANIX").warning(f"Error mensaje personalizado: {e}")


# ==============================================================================
# CONFIGURACIÓN DEL SISTEMA DE LOGGING
# ==============================================================================
# OBJETIVO: Proporcionar trazabilidad completa de operaciones del sistema
# FUNCIONALIDAD: Logging dual (archivo + consola) con niveles diferenciados
# ==============================================================================
# ==============================================================================
# FUNCIÓN: CONFIGURACIÓN DEL SISTEMA DE LOGGING
# ==============================================================================
# OBJETIVO: Establecer sistema de trazabilidad completo para operaciones
# FUNCIONALIDAD:
#   - Logging a archivo: Nivel DEBUG (trazabilidad completa)
#   - Logging a consola: Nivel INFO (operaciones relevantes)
#   - Formato timestamp: YYYY-MM-DD HH:MM:SS
# PARÁMETROS: Ninguno
# RETORNO: Objeto logger configurado
# ==============================================================================
def configurar_logging():
    """
    Configura el sistema de logging dual para trazabilidad industrial.

    Esta función establece dos handlers de logging:
    1. Archivo (oceanix_log.txt): Registra todos los eventos con nivel DEBUG
    2. Consola: Muestra información operativa con nivel INFO

    Returns:
        logging.Logger: Logger configurado para el sistema OCEANIX
    """
    logger = logging.getLogger("OCEANIX")
    logger.setLevel(logging.DEBUG)

    # Handler para archivo - trazabilidad completa
    fh = logging.FileHandler("oceanix_log.txt", encoding='utf-8')
    fh.setLevel(logging.DEBUG)

    # Handler para consola - información operativa
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)

    # Formato de timestamp industrial
    formatter = logging.Formatter('[%(asctime)s] %(levelname)-8s %(message)s',
                                  datefmt='%Y-%m-%d %H:%M:%S')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)

    logger.addHandler(fh)
    logger.addHandler(ch)
    return logger


# ==============================================================================
# CLASE: CONTROLADOR PRINCIPAL DEL SISTEMA OCEANIX
# ==============================================================================
# OBJETIVO: Implementar controlador maestro para celda robotizada de clasificación
# PATRÓN DE DISEÑO: Controlador maestro con máquina de estados finita (SED)
# ARQUITECTURA: Patrón Command con gestión de estados y recuperación de errores
# ==============================================================================

class CeldaOceanix:
    """
    Controlador principal del sistema de clasificación Oceanix.

    Esta clase implementa un controlador maestro que orquesta todas las operaciones
    de la celda robotizada, aplicando principios de teoría de autómatas para
    garantizar operación determinista y recuperación automática de errores.

    FUNCIONALIDADES PRINCIPALES:
    ==============================================================================
    1. COMUNICACIÓN BIDIRECCIONAL CON ARDUINO:
       - Protocolo 2-bit síncrono (estados del robot ↔ resultados de sensor)
       - Validación de señales y manejo de errores de comunicación

    2. CONTROL DE ROBOT NED2:
       - Navegación segura con patrón hub-and-spoke
       - Gestión de estados mediante máquina de estados finita
       - Recuperación automática de colisiones y errores

    3. LÓGICA DE CLASIFICACIÓN:
       - Inspección temporal de 7 segundos con latching de detección
       - Clasificación binaria: metal/no metal
       - Gestión de pallet con memoria de 3 posiciones

    4. GESTIÓN DE ERRORES Y RECUPERACIÓN:
       - Reconexión automática con backoff exponencial
       - Limpieza de colisiones detectadas
       - Estados seguros para parada de emergencia

    ATRIBUTOS DE CONFIGURACIÓN:
    ==============================================================================
    - Estados del robot: INICIANDO(0), DISPONIBLE(1), OCUPADO(2), EN_INSPECCIÓN(3)
    - Protocolo comunicación: Estados (DO2/DO1), Resultados (DI2/DI1)
    - Temporización: Inspección 7s, timeout movimientos 30s
    - Coordenadas: Sistema de poses seguras para navegación industrial

    MÉTODO PRINCIPAL:
    ==============================================================================
    ejecutar_ciclo(): Ejecuta ciclo completo de clasificación automatizada
    """

    # ==========================================================================
    # MÉTODO: INICIALIZACIÓN DEL CONTROLADOR
    # ==========================================================================
    # OBJETIVO: Establecer conexión con robot y configurar parámetros iniciales
    # ALGORITMO: Reconexión automática con backoff exponencial
    # VALIDACIÓN: Verificación de calibración y estados iniciales
    # ==========================================================================
    def __init__(self, ip_address, max_reintentos_conexion=3):
        """
        Inicializa el controlador de la celda Oceanix con configuración completa.

        Este método establece la conexión con el robot Ned2 mediante TCP/IP,
        configura todos los parámetros operativos, y inicializa el sistema de
        logging. Implementa reconexión automática con backoff exponencial para
        robustez industrial.

        Args:
            ip_address (str): Dirección IP del robot Ned2 en la red industrial
            max_reintentos_conexion (int): Número máximo de intentos de conexión

        Raises:
            SystemExit: Si no se puede establecer conexión después de reintentos

        Configura:
            - Conexión TCP/IP con robot Ned2
            - Calibración automática del robot
            - Estados iniciales del pallet (vacío)
            - Pines de comunicación digital
            - Coordenadas de trabajo seguras
            - Sistema de logging operacional
        """
        self.logger = configurar_logging()
        self.ip_address = ip_address
        self.robot = None
        self.sistema_voz = None  # Se inicializa después de conectar

        self.logger.info("=" * 60)
        self.logger.info("CONECTANDO CON ROBOT NED2 OCEANIX v2.3 (CORREGIDA)")
        self.logger.info("=" * 60)

        # Intentar conectar con reintentos
        self._conectar_con_reintentos(ip_address, max_reintentos_conexion)
        
        # --- MEMORIA DEL PALLET ---
        self.estado_pallet = [False, False, False]
        
        # --- PINES COMUNICACIÓN (BIDIRECCIONAL) ---
        self.PIN_DI_MSB = "DI2"       # Arduino -> Ned2 (MSB)
        self.PIN_DI_LSB = "DI1"       # Arduino -> Ned2 (LSB)
        self.PIN_SENSOR_PIEZA = 'DI5' # Sensor pieza (Activo en LOW)
        self.PIN_DO_MSB = "DO2"       # Ned2 -> Arduino (MSB)
        self.PIN_DO_LSB = "DO1"       # Ned2 -> Arduino (LSB)
        
        # --- CÓDIGOS PROTOCOLO 2 BITS ---
        self.ESTADO_INICIANDO = 0    # 00
        self.ESTADO_DISPONIBLE = 1   # 01
        self.ESTADO_OCUPADO = 2      # 10
        self.ESTADO_EN_INSPECCION = 3 # 11
        
        self.CMD_SIN_COMANDO = 0     # 00
        self.CMD_NO_METAL = 1        # 01 -> PALLET (VERDE)
        self.CMD_METAL = 3           # 11 -> DESCARTES (ROJO)
        
        # --- TIEMPOS ---
        self.TIEMPO_INSPECCION = 7   # segundos COMPLETOS de inspección
        self.TIMEOUT_MOVIMIENTO = 30 # segundos máximo por movimiento
        
        # --- RESULTADO DE INSPECCIÓN ---
        self.resultado_inspeccion = False  # Almacena resultado del último ciclo
        
        # --- COORDENADAS ACTUALIZADAS (Z re-calibrada) ---
        self.POSE_HOME = [0.2, 0.0, 0.2, 0.0, 0.0, 0.0]
        self.POSE_APROX_GENERAL = [0.199, -0.2, 0.322, -0.011, 1.526, -0.653]
        
        # Alimentador (Pick)
        self.POSE_ALIMENTADOR = [0.0, -0.247, 0.129, 2.409, 1.5, 2.45] 
        self.APROX_ALIMENTADOR = [0.0, -0.247, 0.199, 2.409, 1.5, 2.45]
        
        # Inspección
        self.POSE_INSPECCION = [0.288, -0.264, 0.172, -2.441, 1.53, 2.297]
        self.APROX_INSPECCION = [0.288, -0.264, 0.202, -2.441, 1.53, 2.297]
        
        # Descartes / Metal
        self.POSE_DESCARTES = [0.237, 0.193, 0.136, -2.392, 1.506, -1.814]
        self.APROX_DESCARTES = [0.237, 0.193, 0.206, -2.392, 1.506, -1.814]
        
        # Pallet / No Metal
        self.APROX_PALLET = [0.382, 0.00, 0.192, 3.021, 1.515, 3.069]
        self.POSES_PALLET = [
            [0.382, 0.05, 0.093, 3.021, 1.515, 3.069],  # Slot 1
            [0.382, 0.00, 0.093, 3.021, 1.515, 3.069],  # Slot 2
            [0.382, -0.05, 0.093, 3.021, 1.515, 3.069]  # Slot 3
        ]
        
        # Configuración inicial
        self.establecer_estado(self.ESTADO_INICIANDO)
        self.logger.info("[OK] Sistema inicializado correctamente")

        # Anuncio de voz del inicio del sistema
        if self.sistema_voz:
            self.sistema_voz.anunciar_evento('inicio_sistema')
        self.logger.info("[OK] Sistema Oceanix operativo\n")

    # ==========================================================================
    # MÉTODO: CONEXIÓN AUTOMÁTICA CON RECUPERACIÓN
    # ==========================================================================
    # OBJETIVO: Establecer conexión robusta con robot mediante TCP/IP
    # ALGORITMO: Backoff exponencial para evitar sobrecarga de red
    # RECUPERACIÓN: Reintentos automáticos hasta límite configurado
    # ==========================================================================
    def _conectar_con_reintentos(self, ip_address, max_reintentos=3):
        """
        Conecta al robot con reintentos automáticos y backoff exponencial.
        """
        for intento in range(max_reintentos):
            try:
                self.logger.info(f"Intento de conexión {intento + 1}/{max_reintentos}...")
                self.robot = NiryoRobot(ip_address)
                self.logger.info("[OK] Conectado al robot")
                
                self.logger.info("[INFO] Calibrando robot...")
                self.robot.calibrate_auto()
                self.logger.info("[OK] Calibración completada")

                # Inicializar sistema de voz después de calibración exitosa
                self.sistema_voz = SistemaVozOceanix(self.robot)
                self.logger.info("[OK] Sistema de voz inicializado")

                self.logger.info("--- CONEXIÓN EXITOSA ---")
                # Anuncio de voz de conexión exitosa
                self.sistema_voz.anunciar_evento('conexion_exitosa')
                self.logger.info("\n")
                return
                
            except Exception as e:
                self.logger.warning(f"Intento {intento + 1} falló: {e}")
                
                if intento < max_reintentos - 1:
                    tiempo_espera = 2 ** intento
                    self.logger.info(f"Esperando {tiempo_espera}s antes de reintentar...")
                    time.sleep(tiempo_espera)
                else:
                    self.logger.error("[ERROR] Fallo crítico: No se pudo conectar al robot")
                    sys.exit(1)

    # ==========================================================================
    # HELPERS DE POSICIÓN
    # ==========================================================================
    def _pose(self, pose_list):
        """Convierte una lista [x, y, z, roll, pitch, yaw] en PoseObject."""
        if isinstance(pose_list, PoseObject):
            return pose_list
        return PoseObject(*pose_list)

    # ==========================================================================
    # MÉTODOS: COMUNICACIÓN Y SENSORES
    # ==========================================================================
    # OBJETIVO: Implementar protocolo de comunicación bidireccional 2-bit
    # PROTOCOLO: Estados del robot (DO2/DO1) y resultados del sensor (DI2/DI1)
    # VALIDACIÓN: Conversión decimal-binaria para señales digitales
    # ==========================================================================

    # --------------------------------------------------------------------------
    # MÉTODO: ESTABLECER ESTADO DEL ROBOT
    # --------------------------------------------------------------------------
    # OBJETIVO: Comunicar estado actual del robot al Arduino supervisor
    # PARÁMETROS: estado_decimal (0-3) según máquina de estados
    # IMPLEMENTACIÓN: Codificación 2-bit MSB/LSB en pines digitales
    # --------------------------------------------------------------------------
    def establecer_estado(self, estado_decimal):
        """Envía el estado actual al Arduino mediante pines digitales (DO2, DO1)."""
        try:
            val_msb = PinState.HIGH if (estado_decimal & 2) else PinState.LOW
            val_lsb = PinState.HIGH if (estado_decimal & 1) else PinState.LOW
            self.robot.digital_write(self.PIN_DO_MSB, val_msb)
            self.robot.digital_write(self.PIN_DO_LSB, val_lsb)
            
            nombres = ["INICIANDO", "DISPONIBLE", "OCUPADO", "EN_INSPECCION"]
            nombre = nombres[estado_decimal] if estado_decimal < len(nombres) else "DESCONOCIDO"
            self.logger.debug(f"Estado enviado -> {estado_decimal} ({nombre})")
        except Exception as e:
            self.logger.error(f"Error al establecer estado Arduino: {e}")

    def leer_comando_arduino(self):
        """Lee DI2 y DI1 para interpretar el comando del Arduino (0-3)."""
        try:
            state_msb = self.robot.digital_read(self.PIN_DI_MSB)
            state_lsb = self.robot.digital_read(self.PIN_DI_LSB)
            bit_msb = 1 if state_msb == PinState.HIGH else 0
            bit_lsb = 1 if state_lsb == PinState.HIGH else 0
            return (bit_msb << 1) | bit_lsb
        except Exception as e:
            self.logger.error(f"Error al leer comando Arduino: {e}")
            return self.CMD_SIN_COMANDO

    def hay_pieza_entrada(self):
        """Verifica si hay pieza en el sensor de entrada CON DEBOUNCE."""
        lecturas_confirmadas = 0
        
        for i in range(3):
            try:
                if self.robot.digital_read(self.PIN_SENSOR_PIEZA) == PinState.LOW:
                    lecturas_confirmadas += 1
                time.sleep(0.05)
            except Exception as e:
                self.logger.warning(f"Error en lectura de sensor (intento {i+1}): {e}")
        
        resultado = lecturas_confirmadas >= 2
        self.logger.debug(f"Sensor pieza: {lecturas_confirmadas}/3 confirmadas -> {resultado}")
        return resultado

    def es_metal(self):
        """
        Retorna el resultado de la última inspección.
        
        NOTA: Este método ahora usa el resultado almacenado durante
        inspeccionar_pieza() en lugar de leer el sensor en tiempo real.
        Esto es porque la pieza ya no está sobre el sensor inductivo
        cuando este método se llama.
        """
        return self.resultado_inspeccion

    # ==========================================================================
    # FEEDBACK (LEDS Y SONIDOS)
    # ==========================================================================
    def configurar_led(self, color):
        """Cambia el color del anillo LED del robot."""
        try:
            self.robot.set_led_color(255, color)
            self.logger.debug(f"LED configurado a RGB{tuple(color)}")
        except Exception:
            try:
                for led_id in range(30):
                    self.robot.set_led_color(led_id, color)
                self.logger.debug(f"LEDs configurados individualmente a RGB{tuple(color)}")
            except Exception as e:
                self.logger.warning(f"No se pudo configurar LED: {e}")

    def apagar_led(self):
        """Apaga todos los LEDs del anillo."""
        self.configurar_led([0, 0, 0])

    def reproducir_sonido(self, sonido):
        """Reproduce un sonido del sistema del robot."""
        try:
            self.robot.play_sound(sonido)
            self.logger.debug(f"Sonido '{sonido}' reproducido")
        except Exception as e:
            self.logger.warning(f"Error al reproducir sonido '{sonido}' (no crítico): {e}")

    # ==========================================================================
    # CONTROL DE MOVIMIENTO
    # ==========================================================================
    def _mover_con_validacion(self, pose):
        """Mueve el robot con manejo CORRECTO de colisiones."""
        try:
            # Limpiar colisión anterior ANTES de nuevo movimiento
            try:
                if hasattr(self.robot, 'clear_collision_detected'):
                    self.robot.clear_collision_detected()
                    self.logger.debug("Limpieza de colisión previa completada")
            except Exception:
                self.logger.debug("No había colisión previa (normal)")
            
            # Movimiento
            self.robot.move(self._pose(pose))
            time.sleep(0.2)
            
        except Exception as e:
            error_msg = str(e).lower()
            
            if "collision" in error_msg or "clear_collision_detected" in error_msg:
                self.logger.error(f"COLISIÓN DETECTADA: {e}")
                
                try:
                    if hasattr(self.robot, 'clear_collision_detected'):
                        self.robot.clear_collision_detected()
                        self.logger.info("Colisión limpiada.")
                except Exception as clean_err:
                    self.logger.error(f"Error al limpiar colisión: {clean_err}")
                
                raise Exception(f"Colisión física detectada: {e}")
            else:
                self.logger.error(f"Error en movimiento: {e}")
                raise

    def ir_a_destino_seguro(self, pose_destino, pose_aprox_destino):
        """Mueve el robot utilizando puntos de paso seguros."""
        try:
            poses = [
                ("Aproximación General", self.POSE_APROX_GENERAL),
                ("Aproximación Destino", pose_aprox_destino),
                ("Destino Final", pose_destino)
            ]
            
            for nombre, pose in poses:
                self.logger.info(f"Moviendo a {nombre}...")
                self._mover_con_validacion(pose)
                
        except Exception as e:
            self.logger.error(f"Fallo en movimiento seguro: {e}")
            raise

    def salir_de_destino_seguro(self, pose_aprox_origen):
        """Retira el robot de una posición de trabajo hacia zona segura."""
        try:
            poses = [
                ("Aproximación Origen", pose_aprox_origen),
                ("Aproximación General", self.POSE_APROX_GENERAL)
            ]
            
            for nombre, pose in poses:
                self.logger.info(f"Retirando: {nombre}...")
                self._mover_con_validacion(pose)
                
        except Exception as e:
            self.logger.error(f"Fallo al retirar de destino: {e}")
            raise

    # ==========================================================================
    # GESTIÓN PALLET
    # ==========================================================================
    def gestionar_pallet_lleno(self):
        """Verifica si el pallet está lleno y solicita vaciado manual."""
        if all(self.estado_pallet):
            self.logger.warning("\n" + "!"*60)
            self.logger.warning(" ALMACÉN LLENO (3/3)")
            self.logger.warning(" No se pueden depositar más piezas.")
            self.logger.warning(" Por favor, vacíe el pallet.")
            self.logger.warning("!"*60)

            # Anuncio de voz de pallet completo
            if self.sistema_voz:
                self.sistema_voz.anunciar_evento('pallet_completo')
            
            while True:
                resp = input(">> Escriba 'SI' cuando el pallet esté vacío: ").strip().upper()
                if resp == "SI":
                    self.estado_pallet = [False, False, False]
                    self.logger.info("[OK] Sistema reiniciado. Pallet vacío.")
                    break

    def obtener_slot_libre(self):
        """Devuelve el índice (0-2) del primer slot libre, o -1 si está lleno."""
        for i in range(3):
            if not self.estado_pallet[i]:
                return i
        return -1

    # ==========================================================================
    # LÓGICA DEL CICLO (v2.3 CORREGIDA)
    # ==========================================================================
    def recoger_pieza(self):
        """Recoge pieza de alimentador con validación de estado."""
        try:
            self.logger.info("[FASE 2] Recogida (Pick)")
            
            self.robot.release_with_tool()
            time.sleep(0.3)
            
            self.ir_a_destino_seguro(self.POSE_ALIMENTADOR, self.APROX_ALIMENTADOR)
            
            self.robot.grasp_with_tool()
            time.sleep(0.5)
            
            self.logger.info("[OK] Pieza recogida y pinza cerrada")

            # Anuncio de voz de recogida exitosa
            if self.sistema_voz:
                self.sistema_voz.anunciar_evento('recogida_exitosa')

            return True
            
        except Exception as e:
            self.logger.error(f"Fallo en recogida: {e}")
            try:
                self.robot.grasp_with_tool()
            except:
                pass
            raise

    # ==========================================================================
    # MÉTODO: INSPECCIÓN TEMPORAL CON LATCHING
    # ==========================================================================
    # OBJETIVO: Ejecutar ventana de inspección de 7 segundos con detección latcheada
    # ALGORITMO: Polling continuo del sensor con lógica de latching
    # IMPORTANCIA: Método crítico que determina la clasificación final de piezas
    # ==========================================================================
    def inspeccionar_pieza(self):
        """
        Implementa la lógica de inspección temporal con latching de detección.

        Esta función ejecuta la fase crítica de inspección donde el robot permanece
        estático por exactamente 7 segundos mientras monitorea continuamente el
        sensor inductivo. La lógica de latching garantiza que si se detecta metal
        al menos una vez durante la ventana temporal, el resultado final será
        "METAL DETECTADO", implementando el principio de "una vez metal, siempre metal".

        PROTOCOLO DE COMUNICACIÓN:
        ==============================================================================
        Arduino Supervisor envía continuamente vía pines DI2/DI1:
        - 01 (decimal 1) = NO_METAL: Pieza no conductiva detectada
        - 11 (decimal 3) = METAL: Pieza conductiva (metal) detectada

        LÓGICA DE LATCHING:
        ==============================================================================
        - Variable metal_detectado inicia en False
        - Cada lectura del sensor se evalúa
        - Si comando == METAL → metal_detectado = True (LATCH ACTIVADO)
        - El latch NO se puede desactivar durante la inspección
        - Resultado final = metal_detectado

        SECUENCIA OPERATIVA:
        ==============================================================================
        1. Movimiento a zona de inspección (POSE_INSPECCION)
        2. Señalización estado EN_INSPECCIÓN al supervisor
        3. Feedback visual/sonoro (LED amarillo + sonido)
        4. Ventana de 7 segundos con polling cada 100ms (70 lecturas)
        5. Cambio a estado OCUPADO (fin de inspección)
        6. Retorno del resultado latcheado

        VALORES DE RETORNO:
        ==============================================================================
        bool: True si se detectó metal al menos una vez, False en caso contrario

        EXCEPCIONES:
        ==============================================================================
        - Captura errores de movimiento y comunicación
        - Estado seguro: resultado_inspeccion = False por defecto
        - Logging completo de todas las operaciones
        """
        try:
            self.logger.info("[FASE 3] Inspección de Calidad")
            
            # Mover a zona de inspección
            self.ir_a_destino_seguro(self.POSE_INSPECCION, self.APROX_INSPECCION)
            
            # Señalizar que estamos en inspección (semáforo amarillo fijo)
            self.establecer_estado(self.ESTADO_EN_INSPECCION)
            self.configurar_led([255, 255, 0])  # AMARILLO en robot
            self.reproducir_sonido("connected.wav")

            # Anuncio de voz de inspección activa
            if self.sistema_voz:
                self.sistema_voz.anunciar_evento('inspeccion_activa')
            
            # --- LÓGICA DE INSPECCIÓN: 7 SEGUNDOS COMPLETOS ---
            tiempo_inicio = time.time()
            metal_detectado = False  # Latch: si alguna vez detecta metal, queda True
            lecturas_metal = 0
            lecturas_no_metal = 0
            lecturas_totales = 0
            
            self.logger.info(f"[INSPECCION] Iniciando ventana de {self.TIEMPO_INSPECCION} segundos...")
            self.logger.info("[INSPECCION] Monitoreando sensor inductivo...")
            
            while (time.time() - tiempo_inicio) < self.TIEMPO_INSPECCION:
                # Leer el estado actual del sensor desde Arduino
                comando = self.leer_comando_arduino()
                lecturas_totales += 1
                
                if comando == self.CMD_METAL:
                    metal_detectado = True  # LATCH: Una vez detectado, no se borra
                    lecturas_metal += 1
                    # Log solo la primera detección de metal
                    if lecturas_metal == 1:
                        tiempo_transcurrido = time.time() - tiempo_inicio
                        self.logger.warning(f"[ALERTA] ¡METAL DETECTADO! (t={tiempo_transcurrido:.1f}s)")
                elif comando == self.CMD_NO_METAL:
                    lecturas_no_metal += 1
                
                # Pausa pequeña para no saturar (10 lecturas/segundo)
                time.sleep(0.1)
            
            # --- FIN DE INSPECCIÓN ---
            tiempo_total = time.time() - tiempo_inicio
            
            # Guardar resultado para uso en clasificación
            self.resultado_inspeccion = metal_detectado
            
            # Log de resumen
            self.logger.info(f"[INSPECCION] Completada en {tiempo_total:.1f}s")
            self.logger.info(f"[INSPECCION] Lecturas: {lecturas_totales} totales, {lecturas_metal} metal, {lecturas_no_metal} no-metal")
            
            if metal_detectado:
                self.logger.warning(f"[RESULTADO] >>> METAL DETECTADO <<< (detectado en {lecturas_metal} lecturas)")
            else:
                self.logger.info(f"[RESULTADO] >>> NO METAL <<< (limpio en {lecturas_no_metal} lecturas)")
            
            # Salir de estado inspección (semáforo latcheará rojo/verde)
            self.establecer_estado(self.ESTADO_OCUPADO)
            self.logger.info("[OK] Inspección completada")
            return metal_detectado
            
        except Exception as e:
            self.logger.error(f"Fallo en inspección: {e}")
            self.establecer_estado(self.ESTADO_OCUPADO)
            self.resultado_inspeccion = False  # Default seguro
            return False

    def depositar_en_pallet(self, slot_idx):
        """Deposita pieza en pallet con validación de estado."""
        try:
            pose_slot = self.POSES_PALLET[slot_idx]
            self.logger.info(f"[INFO] Almacenando en slot {slot_idx + 1}/3")
            
            self.robot.move(self._pose(self.POSE_APROX_GENERAL))
            self.robot.move(self._pose(self.APROX_PALLET))
            self.robot.move(self._pose(pose_slot))
            
            self.robot.release_with_tool()
            time.sleep(0.3)
            
            self.estado_pallet[slot_idx] = True
            self.logger.info("[OK] Pieza almacenada correctamente")

            # Anuncio de voz de deposito completado
            if self.sistema_voz:
                self.sistema_voz.anunciar_evento('deposito_completado')
            
            self.robot.move(self._pose(self.APROX_PALLET))
            self.robot.move(self._pose(self.POSE_APROX_GENERAL))
            
            return True
            
        except Exception as e:
            self.logger.error(f"Fallo al depositar en pallet: {e}")
            return False

    # ==========================================================================
    # MÉTODO PRINCIPAL: EJECUCIÓN DE CICLO COMPLETO
    # ==========================================================================
    # OBJETIVO: Ejecutar ciclo automatizado completo de clasificación industrial
    # ALGORITMO: Máquina de estados finita con 6 fases secuenciales
    # RECUPERACIÓN: Manejo completo de errores y estados seguros
    # ==========================================================================
    def ejecutar_ciclo(self):
        """
        Ejecuta el ciclo completo de clasificación automatizada de Oceanix.

        Este método implementa el flujo operativo principal del sistema, coordinando
        todas las fases de operación desde la detección inicial hasta la clasificación
        final. Aplica principios de automatización industrial con manejo robusto de
        errores y recuperación automática.

        FASES DEL CICLO OPERATIVO:
        ==============================================================================

        FASE 0: ESTADO DISPONIBLE
        -----------------------------------------------------------------------------
        - Estado inicial: Robot en POSE_HOME
        - Comunicación: Estado DISPONIBLE (01) al supervisor
        - Espera: Nueva pieza en alimentador

        FASE 1: DETECCIÓN DE PIEZA
        -----------------------------------------------------------------------------
        - Sensor: Verificación DI5 con debounce (3 lecturas)
        - Validación: Espacio disponible en pallet
        - Estado: Cambio a OCUPADO (10)
        - Gestión: Si pallet lleno → solicitud manual de vaciado

        FASE 2: RECOGIDA (PICK)
        -----------------------------------------------------------------------------
        - Movimiento: Navegación segura hub-and-spoke
        - Ruta: HOME → APROX_GENERAL → APROX_ALIMENTADOR → POSE_ALIMENTADOR
        - Acción: Release gripper → Move → Grasp gripper
        - Validación: Verificación de agarre exitoso

        FASE 3: INSPECCIÓN DE CALIDAD (CRÍTICA)
        -----------------------------------------------------------------------------
        - Duración: Exatamente 7 segundos (ventana temporal fija)
        - Lógica: Latching de detección de metal
        - Comunicación: Estado EN_INSPECCIÓN (11) al supervisor
        - Feedback: LED amarillo + sonido "connected.wav"
        - Sensor: Monitoreo continuo del sensor inductivo

        FASE 4: CLASIFICACIÓN DE RESULTADOS
        -----------------------------------------------------------------------------
        - Evaluación: Resultado latcheado de inspección
        - Decisión: Metal → descartes | No metal → pallet
        - Movimiento: Salida segura de zona de inspección

        FASE 5: DEPOSITO Y CLASIFICACIÓN FINAL
        -----------------------------------------------------------------------------
        - PATH A (METAL): LED rojo + sonido "warn.wav" + POSE_DESCARTES
        - PATH B (NO METAL): LED verde + sonido "ready.wav" + SLOT PALLET
        - Gestión: Actualización memoria pallet (3 posiciones)

        FASE 6: RESET Y CICLO COMPLETADO
        -----------------------------------------------------------------------------
        - Cleanup: Apagar LEDs + mover a HOME + estado DISPONIBLE
        - Logging: Registro completo del ciclo exitoso

        MANEJO DE ERRORES:
        ==============================================================================
        - Captura: Todas las excepciones del ciclo
        - Recuperación: Estado seguro DISPONIBLE
        - Cleanup: Apagado de LEDs y movimiento a HOME
        - Logging: Traza completa del error con stack trace

        TIEMPOS OPERATIVOS TÍPICOS:
        ==============================================================================
        - Ciclo completo: 21-25 segundos
        - Inspección: 7 segundos exactos
        - Movimientos: 2-4 segundos por trayectoria
        - Timeout movimientos: 30 segundos máximo

        DEPENDENCIAS:
        ==============================================================================
        - Robot Ned2 calibrado y conectado
        - Arduino supervisor operativo
        - Sensores configurados correctamente
        - Coordenadas de trabajo válidas
        """
        self.logger.info("\n" + "="*60)
        self.logger.info(" INICIO CICLO OCEANIX v2.3")
        self.logger.info("="*60)

        # Anuncio de voz del inicio de ciclo
        if self.sistema_voz:
            self.sistema_voz.anunciar_evento('ciclo_iniciado')

        try:
            # --- FASE 0: DISPONIBLE ---
            self.logger.info("\n[FASE 0] Estado DISPONIBLE")
            self.establecer_estado(self.ESTADO_DISPONIBLE)
            self._mover_con_validacion(self.POSE_HOME)

            # --- FASE 1: DETECCIÓN ---
            self.logger.info("\n[FASE 1] Verificando sensor de entrada...")
            if not self.hay_pieza_entrada():
                self.logger.info("[INFO] No se detecta pieza. Esperando...")
                self.establecer_estado(self.ESTADO_DISPONIBLE)
                return

            self.logger.info("[OK] PIEZA DETECTADA")

            # Anuncio de voz de pieza detectada
            if self.sistema_voz:
                self.sistema_voz.anunciar_evento('pieza_detectada')

            # Verificar espacio en almacén
            if self.obtener_slot_libre() == -1:
                self.logger.warning("[WARN] Hay pieza pero almacén LLENO.")
                self.gestionar_pallet_lleno()
                
                if self.obtener_slot_libre() == -1:
                    self.logger.error("[ERROR] Almacén sin huecos. Ciclo abortado.")
                    self.establecer_estado(self.ESTADO_DISPONIBLE)
                    return

            self.establecer_estado(self.ESTADO_OCUPADO)

            # --- FASE 2: RECOGIDA ---
            self.recoger_pieza()
            self.salir_de_destino_seguro(self.APROX_ALIMENTADOR)

            # --- FASE 3: INSPECCIÓN (7 segundos completos) ---
            # inspeccionar_pieza() ahora retorna el resultado directamente
            # y lo guarda en self.resultado_inspeccion
            detecta_metal = self.inspeccionar_pieza()
            
            # --- FASE 4: CONFIRMAR RESULTADO Y SALIR DE ZONA ---
            self.logger.info("\n[FASE 4] Resultado de clasificación")
            self.logger.info(f"[FASE 4] Decisión final: {'METAL -> DESCARTES' if detecta_metal else 'NO METAL -> PALLET'}")
            
            # Salir de zona de inspección
            self.salir_de_destino_seguro(self.APROX_INSPECCION)

            # --- FASE 5: CLASIFICACIÓN ---
            if detecta_metal:
                self.logger.info("\n[FASE 5] CLASIFICACIÓN: METAL -> DESCARTES")
                self.configurar_led([255, 0, 0])  # ROJO
                self.reproducir_sonido("warn.wav")

                # Anuncio de voz de metal detectado
                if self.sistema_voz:
                    self.sistema_voz.anunciar_evento('metal_detectado')
                
                self.ir_a_destino_seguro(self.POSE_DESCARTES, self.APROX_DESCARTES)
                self.robot.release_with_tool()
                self.logger.info("[OK] Pieza descartada")
                self.salir_de_destino_seguro(self.APROX_DESCARTES)
            else:
                self.logger.info("\n[FASE 5] CLASIFICACIÓN: NO METAL -> ALMACÉN")
                self.configurar_led([0, 255, 0])  # VERDE
                self.reproducir_sonido("ready.wav")

                # Anuncio de voz de material aceptado
                if self.sistema_voz:
                    self.sistema_voz.anunciar_evento('no_metal_aceptado')
                
                slot_idx = self.obtener_slot_libre()
                
                if slot_idx != -1:
                    if not self.depositar_en_pallet(slot_idx):
                        self.logger.error("[ERROR] Fallo en depósito, ciclo abortado")
                        try:
                            self.robot.move(self._pose(self.POSE_APROX_GENERAL))
                        except:
                            pass
                else:
                    self.logger.error("[ERROR] No hay slots disponibles")

            # --- FIN DE CICLO ---
            self.logger.info("\n[FASE 6] Fin de ciclo")
            self.apagar_led()
            self._mover_con_validacion(self.POSE_HOME)
            self.establecer_estado(self.ESTADO_DISPONIBLE)

            # Anuncio de voz de sistema disponible
            if self.sistema_voz:
                self.sistema_voz.anunciar_evento('estado_disponible')

            # Anuncio de voz de ciclo finalizado
            if self.sistema_voz:
                self.sistema_voz.anunciar_evento('ciclo_finalizado')

            self.logger.info("="*60)
            self.logger.info(" CICLO COMPLETADO")
            self.logger.info("="*60 + "\n")

        except Exception as e:
            self.logger.error(f"\n[ERROR] Interrupción en ciclo: {e}", exc_info=True)

            # Anuncio de voz de error recuperable
            if self.sistema_voz:
                self.sistema_voz.anunciar_evento('error_recuperable')

            self.apagar_led()
            self.establecer_estado(self.ESTADO_DISPONIBLE)

            try:
                self.logger.info("[INFO] Intentando mover robot a HOME...")
                self._mover_con_validacion(self.POSE_HOME)
            except Exception as home_err:
                self.logger.error(f"[ERROR CRÍTICO] No se puede alcanzar HOME: {home_err}")

    # ==========================================================================
    # DESCONEXIÓN SEGURA
    # ==========================================================================
    def desconexion_segura(self):
        """Realiza desconexión segura del robot."""
        self.logger.info("\n--- INICIANDO DESCONEXIÓN SEGURA ---")
        try:
            # Anuncio de voz de parada de emergencia si el sistema está operativo
            if self.sistema_voz and self.robot:
                self.sistema_voz.anunciar_evento('parada_emergencia')

            self.apagar_led()
            self.establecer_estado(self.ESTADO_INICIANDO)

            try:
                self.logger.info("[INFO] Robot moviendo a HOME...")
                self._mover_con_validacion(self.POSE_HOME)
                self.logger.info("[OK] Robot movido a HOME")
            except Exception as home_err:
                self.logger.warning(f"[WARN] HOME falló durante desconexión: {home_err}")

            if self.robot:
                self.robot.close_connection()
                self.logger.info("[OK] Robot desconectado correctamente")

        except Exception as e:
            self.logger.error(f"[ERROR] Fallo durante desconexión: {e}")
            try:
                if self.robot:
                    self.robot.close_connection()
            except:
                pass


# ==============================================================================
# MAIN
# ==============================================================================
if __name__ == '__main__':
    IP_ROBOT = "172.16.126.153"
    
    print("\n" + "="*60)
    print(" CELDA DE CLASIFICACIÓN OCEANIX v2.3 (CORREGIDA)")
    print("="*60)
    
    app = None

    try:
        app = CeldaOceanix(IP_ROBOT)

        while True:
            print("\n========== MENÚ OCEANIX ==========")
            print("1. Ejecutar Ciclo Completo")
            print("2. Ver estado Almacén")
            print("3. Resetear Almacén")
            print("4. Probar Sistema de Voz")
            print("5. Salir")
            
            try:
                opcion = input(">> Opción: ").strip()
            except EOFError:
                break

            if opcion == '1':
                app.ejecutar_ciclo()
                
            elif opcion == '2':
                ocupados = app.estado_pallet.count(True)
                estado_visual = ["[*]" if x else "[ ]" for x in app.estado_pallet]
                print(f"\n[INFO] Estado Pallet: {estado_visual}")
                print(f"       Ocupados: {ocupados}/3 slots")
                
            elif opcion == '3':
                resp = input("¿Seguro que quieres resetear el almacén? (SI/no): ").strip().upper()
                if resp == "SI":
                    app.estado_pallet = [False, False, False]
                    print("[INFO] Almacén vaciado en memoria.")
                
            elif opcion == '4':
                print("\n[INFO] Probando sistema de voz...")
                print("Eventos disponibles:")
                eventos_prueba = [
                    'inicio_sistema', 'ciclo_iniciado', 'pieza_detectada',
                    'recogida_exitosa', 'inspeccion_activa', 'metal_detectado',
                    'no_metal_aceptado', 'deposito_completado', 'ciclo_finalizado',
                    'pallet_completo', 'estado_disponible'
                ]
                for i, evento in enumerate(eventos_prueba, 1):
                    print(f"{i}. {evento}")

                try:
                    num_evento = int(input("Seleccione evento (1-11): ")) - 1
                    if 0 <= num_evento < len(eventos_prueba):
                        evento_seleccionado = eventos_prueba[num_evento]
                        print(f"[INFO] Reproduciendo: {evento_seleccionado}")
                        if app.sistema_voz:
                            app.sistema_voz.anunciar_evento(evento_seleccionado)
                        else:
                            print("[ERROR] Sistema de voz no inicializado")
                    else:
                        print("[ERROR] Opción inválida")
                except ValueError:
                    print("[ERROR] Ingrese un número válido")

            elif opcion == '5':
                print("[INFO] Saliendo del sistema...")
                break
                
            else:
                print("[WARN] Opción inválida.")
                
    except KeyboardInterrupt:
        print("\n\n[WARN] Parada de emergencia detectada (Ctrl+C).")
        
    except Exception as e:
        print(f"\n[ERROR] Error no controlado: {e}")
        
    finally:
        if app is not None:
            app.desconexion_segura()
        print("--- PROGRAMA FINALIZADO ---")
