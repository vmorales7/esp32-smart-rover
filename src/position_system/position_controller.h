#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "project_config.h"

/**
 * @brief Módulo encargado de transformar referencias de velocidad lineal y angular
 * del cuerpo del vehículo (v, w) en velocidades angulares para cada rueda (ω_L, ω_R).
 */
namespace PositionController {

    /**
     * @brief Actualiza las referencias de velocidad angular para las ruedas en base
     * a las referencias de velocidad lineal y angular del vehículo.
     * 
     * Esta función implementa la cinemática inversa diferencial para calcular
     * las velocidades angulares requeridas en cada rueda para lograr las velocidades
     * deseadas del cuerpo.
     * 
     * @param v_ref_ptr Puntero a la velocidad lineal de referencia del vehículo [m/s]
     * @param w_ref_ptr Puntero a la velocidad angular de referencia del vehículo [rad/s]
     * @param wL_ref_ptr Puntero a la velocidad angular de referencia de la rueda izquierda [rad/s]
     * @param wR_ref_ptr Puntero a la velocidad angular de referencia de la rueda derecha [rad/s]
     */
    void update_wheel_speed_reference(
        volatile float* v_ref_ptr,
        volatile float* w_ref_ptr,
        volatile float* wL_ref_ptr,
        volatile float* wR_ref_ptr
    );

} // namespace PositionController

#endif // POSITION_CONTROLLER_H