#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "project_config.h"

/* ---------------- Constantes y variables sistema ------------------*/



/* ---------------- Funciones del sistema ------------------*/

namespace PositionController {

    /**
     * @brief Inicializa el controlador de posición.
     *
     * Establece todas las referencias (v_ref, w_ref, wL_ref, wR_ref) en cero
     * y configura el modo del controlador como SPEED_REF_INACTIVE.
     *
     * @param wL_ref Variable con referencia de rueda izquierda [rad/s]
     * @param wR_ref Variable con referencia de rueda derecha [rad/s]
     * @param control_mode Variable con modo actual del controlador de posición
     */
    void init(
        volatile uint8_t& control_mode,
        volatile float& wL_ref, volatile float& wR_ref
    );

    /**
     * @brief Cambia el modo de operación del controlador de posición.
     *
     * @param new_mode Nuevo modo deseado (SPEED_REF_INACTIVE, SPEED_REF_MANUAL, SPEED_REF_AUTO)
     * @param control_mode Variable con estado del controlador de posición
     */
    void set_control_mode(
        const uint8_t new_mode,
        volatile uint8_t& control_mode
    );

    /**
     * @brief Calcula la velocidad angular de referencia de una rueda,
     *        a partir de v_ref y w_ref.
     *
     * @param v_ref Velocidad lineal de referencia global [m/s]
     * @param w_ref Velocidad angular de referencia global [rad/s]
     * @param wheel_id WHEEL_LEFT (0) o WHEEL_RIGHT (1)
     * @return Velocidad angular de referencia para la rueda [rad/s]
     */
    float compute_wheel_speed_ref(
        const float v_ref, 
        const float w_ref, 
        const uint8_t wheel_id
    );

    /**
     * @brief Asigna referencias de velocidad angular a cada rueda.
     *
     * Si el modo del controlador está en SPEED_REF_INACTIVE, no se hace nada.
     *
     * @param wL Valor deseado para rueda izquierda [rad/s]
     * @param wR Valor deseado para rueda derecha [rad/s]
     * @param wL_ref_global Variable con referencia izquierda
     * @param wR_ref_global Variable con referencia derecha
     * @param control_mode Variable con modo del controlador
     */
    void set_wheel_speed_ref(
        const float wL, const float wR,
        volatile float& wL_ref_global, volatile float& wR_ref_global,
        volatile uint8_t& control_mode
    );
}

#endif // POSITION_CONTROLLER_H
