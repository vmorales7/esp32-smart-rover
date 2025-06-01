#ifndef EVADE_CONTROLLER_H
#define EVADE_CONTROLLER_H

/* ------------------------ Mis librerías ------------------------*/

#include "project_config.h"

// Sensores
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
// #include "sensors_firmware/imu_reader.h"

// Estimación y control
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

// Control de motores
#include "motor_drive/motor_controller.h"

// -------------- Parámetros para evasión --------------

constexpr float EVATION_MIN_SPACE = 40.0f;           // cm
constexpr float DELTA_THETA = 30.0f * (PI/180.0f);   // 30 grados en rad
constexpr float MAX_EVASION_ANGLE = 90.0f * (PI/180.0f); // 90 grados en rad
constexpr float EVASION_ADVANCE_DIST = 0.30f;        // metros (ajusta según zona segura)


// ------------- Funciones principales -------------

namespace EvadeController {
    void init(EvadeContext& ctx);
    void start_evade(EvadeContext& ctx, GlobalContext* global_ctx);
    void update(GlobalContext* global_ctx, EvadeContext& ctx);
    bool is_finished(const EvadeContext& ctx);
    bool has_failed(const EvadeContext& ctx);
}


#endif // EVADE_CONTROLLER_H