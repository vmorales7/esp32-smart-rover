#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include "project_config.h"

/* ---------------- Constantes y variables sistema ------------------*/

enum class PoseEstimatorMode : uint8_t {
    ENCODER = 1U,
    FUSION  = 2U
};
constexpr PoseEstimatorMode POSE_ESTIMATOR_MODE = PoseEstimatorMode::ENCODER;

/**
 * @brief Estructura que representa la pose estimada del vehículo autónomo.
 *
 * Contiene la posición (X, Y), la orientación (θ), y las velocidades
 * lineal y angular estimadas en el plano.
 */
struct PoseData {
    float x;      ///< Posición X [m]
    float y;      ///< Posición Y [m]
    float theta;  ///< Orientación [rad]
    float v;      ///< Velocidad lineal [m/s]
    float w;      ///< Velocidad angular [rad/s]
};


/* ---------------- Funciones del sistema ------------------*/

namespace PoseEstimator {

    /**
     * @brief Calcula la nueva pose del vehículo a partir de deltas de giro de ruedas y estado anterior.
     *
     * Esta función realiza la integración cinemática diferencial de la pose, sin acceder a ningún dato global.
     * 
     * @param delta_phiL Giro de la rueda izquierda desde la última lectura [rad].
     * @param delta_phiR Giro de la rueda derecha desde la última lectura [rad].
     * @param wL_measured Velocidad angular de la rueda izquierda [rad/s].
     * @param wR_measured Velocidad angular de la rueda derecha [rad/s].
     * @param x_prev Posición previa en X [m].
     * @param y_prev Posición previa en Y [m].
     * @param theta_prev Orientación previa [rad].
     * @return PoseData Estimación de la nueva pose.
     */
    PoseData compute_encoder_pose(
        float delta_phiL, float delta_phiR,
        float wL_measured, float wR_measured,
        float x_prev, float y_prev, float theta_prev
    );

    /**
     * @brief Estima la nueva pose utilizando datos de los encoders y el estado actual del vehículo.
     *
     * Captura los pasos actuales, calcula los deltas desde la última lectura, y retorna
     * una estructura `PoseData` sin modificar el estado global.
     *
     * @param x_ptr Puntero a la posición actual en X.
     * @param y_ptr Puntero a la posición actual en Y.
     * @param theta_ptr Puntero a la orientación actual θ.
     * @param v_ptr Puntero a la velocidad lineal actual.
     * @param w_ptr Puntero a la velocidad angular actual.
     * @param wL_ptr Puntero a la velocidad angular izquierda medida.
     * @param wR_ptr Puntero a la velocidad angular derecha medida.
     * @param steps_left_ptr Puntero a los pasos acumulados del encoder izquierdo.
     * @param steps_right_ptr Puntero a los pasos acumulados del encoder derecho.
     * @return PoseData Estimación de la pose actual.
     */
    PoseData estimate_pose_from_encoder(
        volatile float* x_ptr, volatile float* y_ptr, volatile float* theta_ptr,
        volatile float* v_ptr, volatile float* w_ptr,
        volatile float* wL_ptr, volatile float* wR_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr
    );

    /**
     * @brief Aplica una estimación de pose a las variables globales del sistema.
     *
     * Esta función sobrescribe los valores actuales de posición, orientación y velocidades
     * con los de una estructura `PoseData` previamente estimada.
     *
     * @param pose_ptr Puntero a la estructura con la nueva pose estimada.
     * @param x_ptr Puntero a posición X.
     * @param y_ptr Puntero a posición Y.
     * @param theta_ptr Puntero a orientación θ.
     * @param v_ptr Puntero a velocidad lineal.
     * @param w_ptr Puntero a velocidad angular.
     * @param pose_estimator_state_ptr Puntero a la variable de estado del módulo
     */
    void update_pose(
        PoseData* pose_ptr,
        volatile float* x_ptr, volatile float* y_ptr, volatile float* theta_ptr,
        volatile float* v_ptr, volatile float* w_ptr,
        volatile uint8_t* pose_estimator_state_ptr
    );

    /**
     * @brief Activa o desactiva el estimador de pose.
     *
     * @param mode Debe ser ACTIVE o INACTIVE.
     * @param pose_estimator_state_ptr Puntero al estado del estimador.
     */
    void set_state(uint8_t mode, volatile uint8_t* pose_estimator_state_ptr);

    /**
     * @brief Reinicia la pose estimada del vehículo y los pasos acumulados de los encoders.
     *
     * Este reinicio también afecta a los acumuladores internos del estimador.
     *
     * @param x_ptr Puntero a posición X.
     * @param y_ptr Puntero a posición Y.
     * @param theta_ptr Puntero a orientación θ.
     * @param steps_left_ptr Puntero a acumulador de pasos de la rueda izquierda.
     * @param steps_right_ptr Puntero a acumulador de pasos de la rueda derecha.
     */
    void reset_pose_and_steps(
        volatile float* x_ptr, volatile float* y_ptr, volatile float* theta_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr
    );

    float wrap_to_pi(float angle);
    
    /**
     * @brief Tarea FreeRTOS que ejecuta periódicamente la estimación de pose por encoder.
     *
     * Si el estimador está activado, calcula la nueva pose y actualiza las variables globales.
     *
     * @param pvParameters Puntero a un `GlobalContext` con referencias a las estructuras del sistema.
     */
    void Task_PoseEstimatorEncoder(void* pvParameters);

}

#endif // POSE_ESTIMATOR_H
