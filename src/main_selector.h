#ifndef MAIN_SELECTOR_H
#define MAIN_SELECTOR_H

// ===================================================
// ✅ Selección del archivo main activo
//    Solo uno debe estar descomentado a la vez
// ===================================================

// --- Main por defecto ---
// #define USE_MAIN_FINAL

// --- Otras opciones de prueba ---
// #define USE_MAIN_MOTOR_DUTY
// #define USE_MAIN_ENCODER_BASIC
 #define USE_MAIN_ENCODER
// #define USE_MAIN_WHEEL_CONTROL
// #define USE_MAIN_DISTANCE_SENSORS_BASIC
// #define USE_MAIN_DISTANCE_SENSORS

// ===================================================
// 🚀 Inclusión del archivo correspondiente
// ===================================================

#if defined(USE_MAIN_FINAL)
  #include "entrypoints/main_final.cpp"
#elif defined(USE_MAIN_MOTOR_DUTY)
  #include "entrypoints/main_motor_duty.cpp"
#elif defined(USE_MAIN_ENCODER_BASIC)
  #include "entrypoints/main_encoder_basic.cpp"
#elif defined(USE_MAIN_ENCODER)
  #include "entrypoints/main_encoder.cpp"
#elif defined(USE_MAIN_WHEEL_CONTROL)
  #include "entrypoints/main_wheel_control.cpp"
#elif defined(USE_MAIN_DISTANCE_SENSORS_BASIC)
  #include "entrypoints/main_distance_sensors_basic.cpp"
#elif defined(USE_MAIN_DISTANCE_SENSORS)
  #include "entrypoints/main_distance_sensors.cpp"
#else
  #error "⚠️ No se ha definido ningún main activo en main_selector.h"
#endif

#endif // MAIN_SELECTOR_H
