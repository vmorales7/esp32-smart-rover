#ifndef EVADE_CONTROLLER_H
#define EVADE_CONTROLLER_H

/* ------------------------ Mis librerías ------------------------*/

#include "project_config.h"
#include <cmath>

// Sensores
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
// #include "sensors_firmware/imu_reader.h"

// Estimación y control
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"
// #include "position_system/evade_controller.h"

// Control de motores
#include "motor_drive/motor_controller.h"


/// Define el mínimo espacio requerido para considerar "libre" (en cm)
constexpr float EVATION_MIN_SPACE = 30.0f; // Cambia este valor según el tamaño real del robot

/// Ángulo de giro por paso (en radianes)
constexpr float DELTA_THETA = 45.0f * (PI/180.0f); // 30 grados

/// Máximo ángulo de evasión permitido por lado (en radianes)
constexpr float MAX_EVASION_ANGLE = 90.0f * (PI/180.0f);

/// Distancia de avance luego de evasión (en metros)
constexpr float EVASION_ADVANCE_DIST = (OBSTACLE_THRESHOLD_CM * 0.1f); // avanza el largo de la zona de obstáculo

// Máquina de estados interna
enum class SubState {
    ENTER,
    SELECT_DIRECTION,
    TURN,
    CHECK_CLEAR,
    TURN_REVERSE,
    ADVANCE,
    SUCCESS,
    FAIL
};

#endif // EVADE_CONTROLLER_H