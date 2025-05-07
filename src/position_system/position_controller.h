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
     * @param v_ref_ptr Puntero a velocidad lineal de referencia global [m/s]
     * @param w_ref_ptr Puntero a velocidad angular de referencia global [rad/s]
     * @param wL_ref_ptr Puntero a referencia de rueda izquierda [rad/s]
     * @param wR_ref_ptr Puntero a referencia de rueda derecha [rad/s]
     * @param control_mode_ptr Puntero al modo actual del controlador de posición
     */
    void init(
        volatile uint8_t* control_mode_ptr,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr
    );

    /**
     * @brief Cambia el modo de operación del controlador de posición.
     *
     * @param mode Nuevo modo deseado (SPEED_REF_INACTIVE, SPEED_REF_MANUAL, SPEED_REF_AUTO)
     * @param control_mode_ptr Puntero al estado del controlador de posición
     */
    void set_control_mode(
        uint8_t mode,
        volatile uint8_t* control_mode_ptr
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
    float compute_wheel_speed_ref(float v_ref, float w_ref, uint8_t wheel_id);

    /**
     * @brief Asigna referencias de velocidad angular a cada rueda.
     *
     * Si el modo del controlador está en SPEED_REF_INACTIVE, no se hace nada.
     *
     * @param wL Valor deseado para rueda izquierda [rad/s]
     * @param wR Valor deseado para rueda derecha [rad/s]
     * @param wL_ref_ptr Puntero a referencia izquierda
     * @param wR_ref_ptr Puntero a referencia derecha
     * @param control_mode_ptr Puntero al modo del controlador
     */
    void set_wheel_speed_ref(
        float wL, float wR,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile uint8_t* control_mode_ptr
    );
}

#endif // POSITION_CONTROLLER_H
