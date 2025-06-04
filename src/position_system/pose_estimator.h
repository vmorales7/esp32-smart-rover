#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include "vehicle_os/general_config.h"

/* ---------------- Constantes y variables sistema ------------------*/

constexpr float FUSION_ALPHA_W = 0.1f; // IMU dominante (mide directo)
constexpr float FUSION_ALPHA_THETA = 0.1f; // IMU dominante (mide directo)

constexpr float IMU_OFFSET_X = 0.02f; // Offset de IMU en eje X [m]
constexpr float IMU_OFFSET_Y = -0.02f; // Offset de IMU en eje Y [m]
constexpr float IMU_OFFSET_R = IMU_OFFSET_X*IMU_OFFSET_X + IMU_OFFSET_Y*IMU_OFFSET_Y; // Offset de IMU en radio [m]
constexpr float IMU_CORRECTION_FACTOR = -0.05 / 0.011; 

/* ---------------- Funciones del sistema ------------------*/

namespace PoseEstimator {

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
 * Esta función pone en cero las variables de posición (x, y), orientación (theta), velocidades (lineal y angular), 
 * los ángulos acumulados de las ruedas (phi_L, phi_R) y el ángulo acumulado del IMU (imu_theta).
 * También reinicia los acumuladores internos utilizados por el estimador de pose
 * para calcular los desplazamientos relativos.
 *
 * @param[in,out] x      Referencia a la posición X global [m]
 * @param[in,out] y      Referencia a la posición Y global [m]
 * @param[in,out] theta  Referencia a la orientación global [rad]
 * @param[in,out] v      Referencia a la velocidad lineal global [m/s]
 * @param[in,out] w      Referencia a la velocidad angular global [rad/s]
 * @param[in,out] w_L    Referencia a la velocidad de rueda izquierda [rad/s]
 * @param[in,out] w_R    Referencia a la velocidad de rueda derecha [rad/s]
 * @param[in,out] phi_L  Referencia al ángulo acumulado de la rueda izquierda [rad]
 * @param[in,out] phi_R  Referencia al ángulo acumulado de la rueda derecha [rad]
 * @param[in,out] imu_theta Referencia al ángulo acumulado del IMU [rad]
 */
void reset_pose(
    volatile float& x, volatile float& y, volatile float& theta,
    volatile float& v, volatile float& w,
    volatile float& w_L, volatile float& w_R,
    volatile float& phi_L, volatile float& phi_R, 
    volatile float& imu_theta
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
 * @param w_L Referencia a la velocidad de la rueda izquierda [rad/s].
 * @param w_R Referencia a la velocidad de la rueda derecha [rad/s].
 * @param phi_L Referencia al ángulo acumulado de la rueda izquierda [rad].
 * @param phi_R Referencia al ángulo acumulado de la rueda derecha [rad].
 * @param imu_theta Referencia al ángulo acumulado del IMU [rad].
 * @param pose_state Variable que almacena el estado del estimador de pose (se establece como INACTIVE).
 */
void init(
    volatile float& x, volatile float& y, volatile float& theta,
    volatile float& v, volatile float& w,
    volatile float& w_L, volatile float& w_R,
    volatile float& phi_L, volatile float& phi_R, 
    volatile float& imu_theta,
    volatile uint8_t& pose_state
);

/**
 * @brief Actualiza la pose global usando odometría simple por encoders.
 *
 * Calcula avance y giro usando solo datos de encoder, y actualiza las referencias globales.
 * 
 * @param[in] encoder_stepsL   Pasos de encoder izquierdo acumulados
 * @param[in] encoder_stepsR   Pasos de encoder derecho acumulados
 * @param[in] encoder_wL       Velocidad angular de rueda izquierda [rad/s]
 * @param[in] encoder_wR       Velocidad angular de rueda derecha [rad/s]
 * @param[in,out] x            Referencia a la posición X global [m]
 * @param[in,out] y            Referencia a la posición Y global [m]
 * @param[in,out] theta        Referencia a la orientación global [rad]
 * @param[in,out] v            Referencia a la velocidad lineal global [m/s]
 * @param[in,out] w            Referencia a la velocidad angular global [rad/s]
 * @param[in,out] w_L          Referencia a la velocidad de rueda izquierda [rad/s]
 * @param[in,out] w_R          Referencia a la velocidad de rueda derecha [rad/s]
 * @param[in] pose_state       Estado actual del estimador (solo ejecuta si es ACTIVE)
 */
void update_pose_encoder(
    const float encoder_stepsL, const float encoder_stepsR, 
    const float encoder_wL, const float encoder_wR,         
    volatile float& x, volatile float& y, volatile float& theta,   
    volatile float& v, volatile float& w,                          
    volatile float& w_L, volatile float& w_R,                      
    const uint8_t pose_state                                   
);

/**
 * @brief Actualiza la pose global usando datos del IMU.
 *
 * Usa filtro complementario para estimar velocidad y orientación.
 * Integra pose por método trapezoidal.
 *
 * @param[in] imu_acc       Aceleración lineal medida por IMU [m/s²]
 * @param[in] imu_w         Velocidad angular medida por IMU [rad/s]
 * @param[in] imu_theta     Ángulo acumulado del IMU [rad]
 * @param[in,out] x        Referencia a la posición X global [m]
 * @param[in,out] y        Referencia a la posición Y global [m]
 * @param[in,out] theta    Referencia a la orientación global [rad]
 * @param[in,out] v        Referencia a la velocidad lineal global [m/s]
 * @param[in,out] w        Referencia a la velocidad angular global [rad/s]
 * @param[in,out] w_L      Referencia a la velocidad de rueda izquierda [rad/s]
 * @param[in,out] w_R      Referencia a la velocidad de rueda derecha [rad/s]
 * @param[in] pose_state   Estado actual del estimador (solo ejecuta si es ACTIVE)
 */
void update_pose_imu(
    const float imu_acc, const float imu_w, const float imu_theta, 
    volatile float& x, volatile float& y, volatile float& theta,             
    volatile float& v, volatile float& w, 
    volatile float& w_L, volatile float& w_R,                  
    const uint8_t pose_state                 
);

/**
 * @brief Actualiza la pose global usando fusión sensorial (encoder + IMU).
 *
 * Usa filtro complementario para estimar velocidad y orientación.
 * Integra pose por método trapezoidal.
 *
 * @param[in] encoder_phiL  Ángulo acumulado de la rueda izquierda [rad]
 * @param[in] encoder_phiR  Ángulo acumulado de la rueda derecha [rad]
 * @param[in] encoder_wL    Velocidad angular de rueda izquierda [rad/s]
 * @param[in] encoder_wR    Velocidad angular de rueda derecha [rad/s]
 * @param[in] imu_acc       Aceleración lineal medida por IMU [m/s²]
 * @param[in] imu_w         Velocidad angular medida por IMU [rad/s]
 * @param[in] imu_theta     Ángulo acumulado del IMU [rad]
 * @param[in,out] x        Referencia a la posición X global [m]
 * @param[in,out] y        Referencia a la posición Y global [m]
 * @param[in,out] theta    Referencia a la orientación global [rad]
 * @param[in,out] v        Referencia a la velocidad lineal global [m/s]
 * @param[in,out] w        Referencia a la velocidad angular global [rad/s]
 * @param[in,out] w_L      Referencia a la velocidad de rueda izquierda [rad/s]
 * @param[in,out] w_R      Referencia a la velocidad de rueda derecha [rad/s]
 * @param[in] pose_state   Estado actual del estimador (solo ejecuta si es ACTIVE)
 */
void update_pose_fusion(
    const float encoder_phiL, const float encoder_phiR,
    const float encoder_wL, const float encoder_wR,          
    const float imu_acc, const float imu_w, const float imu_theta, 
    volatile float& x, volatile float& y, volatile float& theta,             
    volatile float& v, volatile float& w,                           
    volatile float& w_L, volatile float& w_R,                       
    const uint8_t pose_state                 
);

/**
 * @brief Normaliza cualquier ángulo al rango (-π, π].
 * @param angle Ángulo en radianes.
 * @return Ángulo normalizado en (-π, π].
 */
float wrap_to_pi(float angle);

/**
 * @brief Tarea FreeRTOS que ejecuta periódicamente la estimación de pose por encoder.
 *
 * Si el estimador está activado, calcula la nueva pose y actualiza las variables globales.
 *
 * @param pvParameters Puntero a un `GlobalContext` con referencias a las estructuras del sistema.
 */
void Task_PoseEstimatorEncoder(void* pvParameters);

} // namespace PoseEstimator

#endif // POSE_ESTIMATOR_H
