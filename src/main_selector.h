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
// #define USE_MAIN_ENCODER_BASIC          // Lectura simple de encoder
// #define USE_MAIN_ENCODER                // Rutina de desplazamiento por duty con lectura de encoder   
// #define USE_MAIN_DISTANCE_SENSORS_BASIC // Solo prueba de lectura de distancia
// #define USE_MAIN_DISTANCE_SENSORS       // Prueba de detención de rueda con la detección de obstáculo + encoder
#define USE_MAIN_DISTANCE_SENSORS_FULL  // Rutina de duty + detención por obtáculo + encoder
// #define USE_MAIN_POSE_BASIC             // Rutina de prueba para pose_estimator
// #define USE_MAIN_POSE                   // Rutina de duty + detención por obtáculo + encoder + pose estimation
// #define USE_MAIN_WHEEL_SPEED_BASIC      // Rutina para tuneo de controlador de rueda
// #define USE_MAIN_WHEEL_SPEED            // Rutina de wheel ref + detención por obstáculo + encoder + pose estimation
// #define USE_MAIN_AVANCE1


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
#elif defined(USE_MAIN_DISTANCE_SENSORS_BASIC)
  #include "entrypoints/main_distance_sensors_basic.cpp"
#elif defined(USE_MAIN_DISTANCE_SENSORS)
  #include "entrypoints/main_distance_sensors.cpp"
#elif defined(USE_MAIN_DISTANCE_SENSORS_FULL)
  #include "entrypoints/main_distance_sensors_full.cpp"
#elif defined(USE_MAIN_WHEEL_SPEED_BASIC)
  #include "entrypoints/main_wheel_speed_basic.cpp"
#elif defined(USE_MAIN_WHEEL_SPEED)
  #include "entrypoints/main_wheel_speed.cpp"
#elif defined(USE_MAIN_AVANCE1)
  #include "entrypoints/main_avance1.cpp"
#else
  #error "⚠️ No se ha definido ningún main activo en main_selector.h"
#endif

#endif // MAIN_SELECTOR_H