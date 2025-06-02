#ifndef EVADE_CONTROLLER_H
#define EVADE_CONTROLLER_H

/* ------------------------ Mis librerías ------------------------*/

#include "project_config.h"

// Máquina de estados del vehículo
#include "vehicle_os/vehicle_os.h"

// Parámetros para evasión (ajusta según tu robot)
constexpr float EVADE_MIN_SPACE = 40.0f;           // cm
constexpr float EVADE_DELTA_THETA = 30.0f * (PI/180.0f);   // 30 grados en rad
constexpr float MAX_EVADE_ANGLE = 90.0f * (PI/180.0f); // 90 grados en rad
constexpr float EVADE_ADVANCE_DIST = 0.30f;        // metros

// Funciones para manejar la evasión desde el OS principal
namespace EvadeController {
    void reset_evade_state(GlobalContext* ctx_ptr);
    void start_evade(GlobalContext* ctx_ptr);
    void update_evade(GlobalContext* ctx_ptr);
}

#endif // EVADE_CONTROLLER_H