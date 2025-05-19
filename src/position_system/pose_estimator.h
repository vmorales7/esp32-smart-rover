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
     * @param delta_phi_L Giro de la rueda izquierda desde la última lectura [rad].
     * @param delta_phi_R Giro de la rueda derecha desde la última lectura [rad].
     * @param wL_measured Velocidad angular de la rueda izquierda [rad/s].
     * @param wR_measured Velocidad angular de la rueda derecha [rad/s].
     * @param x_prev Posición previa en X [m].
     * @param y_prev Posición previa en Y [m].
     * @param theta_prev Orientación previa [rad].
     * @return PoseData Estimación de la nueva pose.
     */
    PoseData compute_encoder_pose(
        float delta_phi_L, float delta_phi_R,
        float w_L, float w_R,
        float x_prev, float y_prev, float theta_prev
    );

    /**
     * @brief Estima la nueva pose utilizando datos de los encoders y el estado actual del vehículo.
     *
     * Captura los pasos actuales, calcula los deltas desde la última lectura, y retorna
     * una estructura `PoseData` sin modificar el estado global.
     *
     * @param w_L Variable con la velocidad angular izquierda medida.
     * @param w_R Variable con la velocidad angular derecha medida.
     * @param steps_L Variable con los pasos acumulados del encoder izquierdo.
     * @param steps_R Variable con los pasos acumulados del encoder derecho.
     * @param x Variable con la posición actual en X.
     * @param y Variable con la posición actual en Y.
     * @param theta Variable con la orientación actual θ.
     * @param v Variable con la velocidad lineal actual.
     * @param w Variable con la velocidad angular actual.
     * @return PoseData Estimación de la pose actual.
     */
    PoseData estimate_pose_from_encoder(
        volatile float& w_L, volatile float& w_R,
        volatile int64_t& steps_L, volatile int64_t& steps_R,
        volatile float& x, volatile float& y, volatile float& theta,
        volatile float& v, volatile float& w
    );

    /**
     * @brief Activa o desactiva el estimador de pose.
     *
     * @param new_state Debe ser ACTIVE o INACTIVE.
     * @param pose_state Variable con el estado del estimador.
     */
    void set_state(const uint8_t new_state, volatile uint8_t& pose_state);

    /**
     * @brief Reinicia completamente la pose y los acumuladores de pasos del vehículo.
     *
     * Esta función pone en cero las variables de posición (x, y), orientación (theta),
     * velocidades (lineal y angular) y los pasos acumulados de ambos encoders.
     * También reinicia los acumuladores internos utilizados por el estimador de pose
     * para calcular los desplazamientos relativos.
     *
     * @param x Referencia a la posición actual en el eje X [m].
     * @param y Referencia a la posición actual en el eje Y [m].
     * @param theta Referencia a la orientación actual del vehículo [rad].
     * @param v Referencia a la velocidad lineal actual [m/s].
     * @param w Referencia a la velocidad angular actual [rad/s].
     * @param steps_L Referencia al número de pasos acumulados por el encoder izquierdo.
     * @param steps_R Referencia al número de pasos acumulados por el encoder derecho.
     */
    void reset_pose(
        volatile float& x, volatile float& y, volatile float& theta,
        volatile float& v, volatile float& w,
        volatile int64_t& steps_L, volatile int64_t& steps_R
    );

    /**
     * @brief Inicializa el estimador de pose del vehículo.
     *
     * Esta función reinicia completamente la pose estimada (posición, orientación, velocidades)
     * y los pasos acumulados de los encoders. Además, establece el estado del estimador como INACTIVE,
     * dejándolo preparado para activarse de forma controlada más adelante.
     *
     * @param x Referencia a la posición inicial en el eje X [m].
     * @param y Referencia a la posición inicial en el eje Y [m].
     * @param theta Referencia a la orientación inicial del vehículo [rad].
     * @param v Referencia a la velocidad lineal inicial [m/s].
     * @param w Referencia a la velocidad angular inicial [rad/s].
     * @param steps_L Referencia al número de pasos del encoder izquierdo.
     * @param steps_R Referencia al número de pasos del encoder derecho.
     * @param pose_state Variable que almacena el estado del estimador de pose (se establece como INACTIVE).
     */
    void init(
        volatile float& x, volatile float& y, volatile float& theta,
        volatile float& v, volatile float& w,
        volatile int64_t& steps_L, volatile int64_t& steps_R,
        volatile uint8_t& pose_state
    );


    /**
     * @brief Actualiza la pose estimada del vehículo en base a la odometría.
     *
     * Esta función estima la nueva posición, orientación y velocidades del vehículo
     * utilizando los datos de los encoders (velocidades angulares y pasos acumulados).
     * La estimación se realiza únicamente si el estimador de pose se encuentra activo.
     *
     * @param w_L Velocidad angular medida de la rueda izquierda [rad/s].
     * @param w_R Velocidad angular medida de la rueda derecha [rad/s].
     * @param steps_L Pasos acumulados del encoder izquierdo.
     * @param steps_R Pasos acumulados del encoder derecho.
     * @param x Referencia a la posición actual en el eje X [m] (actualizada con la nueva estimación).
     * @param y Referencia a la posición actual en el eje Y [m].
     * @param theta Referencia a la orientación actual del vehículo [rad].
     * @param v Referencia a la velocidad lineal actual [m/s].
     * @param w Referencia a la velocidad angular actual [rad/s].
     * @param pose_state Estado actual del estimador de pose (debe ser ACTIVE para ejecutar la estimación).
     */
    void update_pose(
        volatile float& w_L, volatile float& w_R,
        volatile int64_t& steps_L, volatile int64_t& steps_R,
        volatile float& x, volatile float& y, volatile float& theta,
        volatile float& v, volatile float& w,
        volatile uint8_t& pose_state
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
