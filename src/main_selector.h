#ifndef MAIN_SELECTOR_H
#define MAIN_SELECTOR_H

// ===================================================
// ✅ Selección del archivo main activo
//    Solo uno debe estar descomentado a la vez
// ===================================================
// --- Main por defecto ---
// #define USE_MAIN_FINAL

// --- Otras opciones de prueba ---
// #define USE_MAIN_MOTOR_DUTY             // Rutina de desplazamiento por duty
// #define USE_MAIN_ENCODER_INTERRUPT_TEST
// #define USE_MAIN_ENCODER          // Lectura simple de encoder
// #define USE_MAIN_US_SENSOR_BASIC // Solo prueba de lectura de distancia
// #define USE_MAIN_AVANCE1       // Rutina de duty + detención por obtáculo + encoder
// #define USE_MAIN_WHEEL_SPEED      // Rutina para tuneo de controlador de rueda
// #define USE_MAIN_POSE             // Rutina de prueba para pose_estimator
// #define USE_MAIN_DISTANCE_SENSORS // RTOS con todos los sensores de distancia
#define USE_MAIN_AVANCE2
// #define USE_MAIN_DEBUG


// ===================================================
// 🚀 Inclusión del archivo correspondiente
// ===================================================
#if defined(USE_MAIN_FINAL)
  #include "entrypoints/main_final.cpp"
#elif defined(USE_MAIN_MOTOR_DUTY)
  #include "entrypoints/main_motor_duty.cpp"
#elif defined(USE_MAIN_ENCODER_INTERRUPT_TEST)
  #include "entrypoints/main_encoder_interrupt_test.cpp"
#elif defined(USE_MAIN_ENCODER)
  #include "entrypoints/main_encoder.cpp"
#elif defined(USE_MAIN_US_SENSOR_BASIC)
  #include "entrypoints/main_us_sensor_basic.cpp"
#elif defined(USE_MAIN_AVANCE1)
  #include "entrypoints/main_avance1.cpp"
#elif defined(USE_MAIN_WHEEL_SPEED)
  #include "entrypoints/main_wheel_speed.cpp"
#elif defined(USE_MAIN_POSE)
  #include "entrypoints/main_pose.cpp"
#elif defined(USE_MAIN_DISTANCE_SENSORS)
  #include "entrypoints/main_distance_sensors.cpp"
#elif defined(USE_MAIN_AVANCE2)
  #include "entrypoints/main_avance2.cpp"
#elif defined(USE_MAIN_DEBUG)
  #include "entrypoints/main_debug.cpp"
#elif defined(USE_MAIN_DEBUG2)
  #include "entrypoints/main_debug2.cpp"
#else
  #error "⚠️ No se ha definido ningún main activo en main_selector.h"
#endif

#endif // MAIN_SELECTOR_H