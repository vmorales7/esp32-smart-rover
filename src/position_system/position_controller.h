#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "project_config.h"

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
    void init_position_controller(
        volatile float* v_ref_ptr, volatile float* w_ref_ptr,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile uint8_t* control_mode_ptr
    );

    /**
     * @brief Cambia el modo de operación del controlador de posición.
     *
     * @param mode Nuevo modo deseado (SPEED_REF_INACTIVE, SPEED_REF_MANUAL, SPEED_REF_AUTO)
     * @param control_mode_ptr Puntero al estado del controlador de posición
     */
    void set_position_control_mode(
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
     * @param value_left  Valor deseado para rueda izquierda [rad/s]
     * @param value_right Valor deseado para rueda derecha [rad/s]
     * @param wL_ref_ptr Puntero a referencia izquierda
     * @param wR_ref_ptr Puntero a referencia derecha
     * @param control_mode_ptr Puntero al modo del controlador
     */
    void set_wheel_speed_ref(
        float value_left, float value_right,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile uint8_t* control_mode_ptr
    );

    /**
     * @brief Establece referencias de velocidad global (v_ref, w_ref) y
     *        automáticamente calcula y asigna las referencias de rueda (wL_ref, wR_ref).
     *
     * Solo tiene efecto si el controlador no está en modo INACTIVE.
     *
     * @param v_ref Velocidad lineal deseada [m/s]
     * @param w_ref Velocidad angular deseada [rad/s]
     * @param v_ref_ptr Puntero a v_ref
     * @param w_ref_ptr Puntero a w_ref
     * @param wL_ref_ptr Puntero a wL_ref
     * @param wR_ref_ptr Puntero a wR_ref
     * @param control_mode_ptr Puntero al modo del controlador
     */
    void set_velocity_ref(
        float v_ref, float w_ref,
        volatile float* v_ref_ptr, volatile float* w_ref_ptr,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile uint8_t* control_mode_ptr
    );

}

#endif // POSITION_CONTROLLER_H
