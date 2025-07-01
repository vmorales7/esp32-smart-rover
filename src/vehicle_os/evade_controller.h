#ifndef EVADE_CONTROLLER_H
#define EVADE_CONTROLLER_H

/* ------------------------ Mis librerías ------------------------*/

#include "vehicle_os/general_config.h"

// Máquina de estados del vehículo
#include "vehicle_os/vehicle_os.h"

// Parámetros para evasión (ajusta según tu robot)
constexpr float EVADE_MIN_SPACE = 40.0f;           // cm
constexpr float EVADE_DELTA_THETA = 45.0f * DEG_TO_RAD;   // 45 grados en rad
constexpr float MAX_EVADE_ANGLE = 150.0f * DEG_TO_RAD;    // 150 grados en rad
constexpr float EVADE_ADVANCE_DIST = 0.50f;        // metros

// Funciones para manejar la evasión desde el OS principal
namespace EvadeController {
    void reset_evade_state(GlobalContext* ctx_ptr);
    void update_evade(GlobalContext* ctx_ptr);
}

#endif // EVADE_CONTROLLER_H