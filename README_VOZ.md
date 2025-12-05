# Sistema de Voz Industrial Oceanix

## 🎯 **Visión General**

El sistema de voz industrial proporciona feedback auditivo profesional durante las operaciones automatizadas del robot Ned2. Utiliza la síntesis de voz integrada del robot vía Google Text-to-Speech (gTTS) para anunciar eventos críticos en español industrial.

## 🏗️ **Arquitectura Técnica**

### **Clase SistemaVozOceanix**
- **API**: PyNiryo `robot.say(texto, language=2)`
- **Límite**: 100 caracteres por mensaje (restricción API)
- **Volumen**: 70% por defecto (configurable)
- **Idioma**: Español industrial (language=2)

### **Eventos de Voz Integrados**

| Evento | Mensaje | Punto de Integración |
|--------|---------|---------------------|
| `inicio_sistema` | "Sistema Oceanix operativo. Listo para clasificación automática." | Inicialización completa |
| `conexion_exitosa` | "Conexión con robot Ned dos establecida." | Después de calibración |
| `ciclo_iniciado` | "Iniciando ciclo de clasificación industrial." | Inicio de `ejecutar_ciclo()` |
| `pieza_detectada` | "Pieza detectada en alimentador. Iniciando proceso." | Detección DI5 confirmada |
| `recogida_exitosa` | "Pieza recogida correctamente." | Gripper cerrado exitosamente |
| `inspeccion_activa` | "Inspección de calidad iniciada. Siete segundos." | Estado EN_INSPECCIÓN |
| `metal_detectado` | "Material metálico detectado. Procediendo a descartes." | Clasificación metal |
| `no_metal_aceptado` | "Material aceptado. Almacenamiento en pallet." | Clasificación no metal |
| `deposito_completado` | "Pieza almacenada correctamente en pallet." | Deposito en slot exitoso |
| `ciclo_finalizado` | "Ciclo de clasificación completado exitosamente." | Fin de ciclo normal |
| `pallet_completo` | "Almacén completo. Requiere vaciado manual." | Gestión pallet lleno |
| `estado_disponible` | "Sistema disponible para nueva clasificación." | Estado DISPONIBLE |
| `error_recuperable` | "Error recuperable detectado. Continuando operación." | Manejo de excepciones |
| `parada_emergencia` | "Parada de emergencia activada. Sistema seguro." | Desconexión segura |

## 🔧 **Integración en el Sistema**

### **Inicialización**
```python
# Después de calibración exitosa
self.sistema_voz = SistemaVozOceanix(self.robot)
```

### **Uso en Eventos**
```python
# Anuncio automático
if self.sistema_voz:
    self.sistema_voz.anunciar_evento('pieza_detectada')
```

### **Manejo de Errores**
- Errores de voz no interrumpen operaciones críticas
- Logging silencioso de fallos de síntesis
- Continuación normal del proceso productivo

## 🎮 **Modo de Prueba**

### **Acceso al Modo Prueba**
```
========== MENÚ OCEANIX ==========
1. Ejecutar Ciclo Completo
2. Ver estado Almacén
3. Resetear Almacén
4. Probar Sistema de Voz    ← Nueva opción
5. Salir
```

### **Funcionamiento del Modo Prueba**
1. Seleccionar opción 4
2. Elegir evento de la lista numerada (1-11)
3. El robot reproduce el mensaje de voz correspondiente

## 📊 **Características Técnicas**

### **Limitaciones de Diseño**
- **Longitud máxima**: 100 caracteres por mensaje
- **Idioma único**: Español (optimizado industrial)
- **Volumen fijo**: 70% (equilibrio entre audibilidad y no intrusivo)
- **No bloqueante**: Errores no detienen operaciones

### **Beneficios Industriales**
- **Feedback auditivo**: Operadores pueden trabajar sin mirar la consola
- **Confirmación de estados**: Verificación auditiva de transiciones críticas
- **Ambiente industrial**: Mensajes profesionales y concisos
- **Robustez**: Sistema falla silenciosamente sin afectar producción

## 🔍 **Diagnóstico y Monitoreo**

### **Logging Integrado**
```
[DEBUG] Voz: 'Sistema Oceanix operativo. Listo para clasificación automática.'
[WARNING] Mensaje voz truncado por límite API
[WARNING] Error reproduciendo voz: [detalles del error]
```

### **Estados del Sistema de Voz**
- **Inicializado**: Después de conexión exitosa
- **Operativo**: Durante ciclos de clasificación
- **Degradado**: Funciona parcialmente si hay errores
- **No disponible**: Sistema continúa sin voz

## 🚀 **Uso en Producción**

### **Activación Automática**
El sistema de voz se activa automáticamente en todos los eventos críticos sin intervención del operador.

### **Configuración Recomendada**
- Volumen: 70% (equilibrado para entornos industriales)
- Idioma: Español (language=2 en API PyNiryo)
- Mensajes: Predefinidos y optimizados

### **Mantenimiento**
- No requiere mantenimiento específico
- Auto-recuperación de errores
- Logging automático de problemas

## 📋 **Historial de Cambios**

### **Versión 1.0 - Integración Inicial**
- ✅ Clase SistemaVozOceanix implementada
- ✅ 13 eventos de voz integrados
- ✅ Modo de prueba funcional
- ✅ Manejo robusto de errores
- ✅ Documentación completa

---

**Nota**: El sistema de voz está diseñado para entornos industriales donde el feedback auditivo mejora la eficiencia operativa sin comprometer la seguridad o el rendimiento del sistema de clasificación automatizada.