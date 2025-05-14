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
     * @param x_var Variable con la posición actual en X.
     * @param y_var Variable con la posición actual en Y.
     * @param theta_var Variable con la orientación actual θ.
     * @param v_var Variable con la velocidad lineal actual.
     * @param w_var Variable con la velocidad angular actual.
     * @param wL_var Variable con la velocidad angular izquierda medida.
     * @param wR_var Variable con la velocidad angular derecha medida.
     * @param steps_left_var Variable con los pasos acumulados del encoder izquierdo.
     * @param steps_right_var Variable con los pasos acumulados del encoder derecho.
     * @return PoseData Estimación de la pose actual.
     */
    PoseData estimate_pose_from_encoder(
        volatile float& x_var, volatile float& y_var, volatile float& theta_var,
        volatile float& v_var, volatile float& w_var,
        volatile float& wL_var, volatile float& wR_var,
        volatile int64_t& steps_left_var, volatile int64_t& steps_right_var
    );

    /**
     * @brief Activa o desactiva el estimador de pose.
     *
     * @param new_state Debe ser ACTIVE o INACTIVE.
     * @param pose_estimator_state Variable con el estado del estimador.
     */
    void set_state(const uint8_t new_state, volatile uint8_t& pose_estimator_state);

    /**
     * @brief Reinicia la pose estimada del vehículo y los pasos acumulados de los encoders.
     *
     * Este reinicio también afecta a los acumuladores internos del estimador.
     *
     * @param x_var Variable con la posición X.
     * @param y_var Variable con la posición Y.
     * @param theta_var Variable con la orientación θ.
     * @param steps_left_var Variable con los pasos acumulados de la rueda izquierda.
     * @param steps_right_var Variable con los pasos acumulados de la rueda derecha.
     */
    void reset_pose_and_steps(
        volatile float& x_var, volatile float& y_var, volatile float& theta_var,
        volatile int64_t& steps_left_var, volatile int64_t& steps_right_var
    );

    /**
     * @brief Inicializa el estimador de pose y lo deja en estado INACTIVE.
     * 
     * @param x_var Variable con la posición X.
     * @param y_var Variable con la posición Y.
     * @param theta_var Variable con la orientación θ.
     * @param steps_left_var Variable con los pasos acumulados de la rueda izquierda.
     * @param steps_right_var Variable con los pasos acumulados de la rueda derecha.
     * @param pose_estimator_state Variable con el estado del estimador.
     */
    void init(
        volatile float& x_var, volatile float& y_var, volatile float& theta_var,
        volatile int64_t& steps_left_var, volatile int64_t& steps_right_var,
        volatile uint8_t& pose_estimator_state
    );

    /**
     * @brief Aplica una estimación de pose a las variables globales del sistema.
     *
     * Esta función sobrescribe los valores actuales de posición, orientación y velocidades
     * con los de una estructura `PoseData` previamente estimada.
     *
     * @param pose Estructura con la nueva pose estimada.
     * @param x_var Variable con la posición X.
     * @param y_var Variable con la posición Y.
     * @param theta_var Variable con la orientación θ.
     * @param v_var Variable con la velocidad lineal.
     * @param w_var Variable con la velocidad angular.
     * @param pose_estimator_state Variable con el estado del estimador.
     */
    void update_pose(
        PoseData& pose,
        volatile float& x_var, volatile float& y_var, volatile float& theta_var,
        volatile float& v_var, volatile float& w_var,
        volatile uint8_t& pose_estimator_state
    );

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
