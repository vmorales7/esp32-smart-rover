# Proyecto Vehículo Autónomo - ESP32

Este repositorio contiene el código fuente para un vehículo autónomo controlado por una ESP32, desarrollado como parte del curso **IEE2913 - Diseño Eléctrico**. El sistema implementa navegación autónoma con control de posición (X, Y), estimación de pose mediante encoders e IMU, evasión de obstáculos y estructura multitarea utilizando RTOS (FreeRTOS sobre ESP32).

---

## 🌐 Estructura del Proyecto

La estructura del código sigue una arquitectura modular, que separa claramente las funcionalidades por capas.

```text
📁 src/
├── motor_drive/         # Control de driver L298N por PWM, y control PI de velocidad de ruedas
├── sensors_firmware/    # Módulos para leer encoders, sensores ultrasónicos, e IMU
├── position_system/     # Estimadores de pose y controladores de posición
├── communication/       # Conexión a Firebase, WiFi y mensajería
├── vehicle_os/          # Módulo de lógica de operación del vehículo y lógica de evasión de obstáculos
├── entrypoints/         # Archivos main.cpp individuales para pruebas específicas
├── main.cpp             # Main por defecto (no usado)
└── main_selector.h      # Selector de archivo main activo para compilar desde PlatformIO
```
---

## 📁 Detalle por carpeta

### `/motor_drive/`
Contiene el `MotorController`, encargado de:
- Ejecutar control de velocidad de rueda mediante PI
- Traducir referencia angular (`w_ref`) a `duty cycle` para los motores
- Aplicar frenado activo o inversión controlada

### `/sensors_firmware/`
Incluye:
- Lectura de **encoders incrementales**
- Lectura de **sensores ultrasónicos (HC-SR04)** para detección de obstáculos
- Lectura de IMU (BNO)

### `/position_system/`
Implementa:
- Estimador de posición y velocidad (pose)
- Controlador de posición con dos modos:
  - `PID`: algoritmo clásico con PID en ángulo
  - `BACKS`: ley de control tipo backstepping

### `/communication/`
Reservado para futura implementación de conexión con Firebase (vía WiFi).

### `/vehicle_os/`
Implementa máquina de estados general del sistema. Incluye:
- Lógica de control
- Lógica de evasión.
- Archivo con configuraciones generales del sistema, constantes físicas y pines

### `/entrypoints/`
Cada archivo `main_*.cpp` corresponde a una **prueba específica**, como por ejemplo:
- `main_pose.cpp`: estimación de pose
- `main_wheel_speed.cpp`: prueba del control de velocidad por rueda
- `main_distance_sensors.cpp`: verificación de lógica para detectar obstáculos

---

## 🔀 Selección del archivo main (con `main_selector.h`)

El archivo `main_selector.h` permite compilar dinámicamente diferentes `main_*.cpp` ubicados en `/entrypoints/`, sin necesidad de cambiar el archivo `platformio.ini`. En main_selector.h, descomenta el #include del main_*.cpp que deseas compilar y comenta el resto.
