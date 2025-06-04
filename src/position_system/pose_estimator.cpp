#include "pose_estimator.h"

namespace PoseEstimator {

// Variables auxiliares para buffer de datos pasados
static int64_t last_millis = 0;
static float last_phiL = 0.0f;
static float last_phiR = 0.0f;
static float last_imu_theta = 0.0f; 


void set_state(const uint8_t new_state, volatile uint8_t& pose_state) {
    if (new_state == pose_state) return;
    pose_state = (new_state == ACTIVE) ? ACTIVE : INACTIVE;
    // if (new_state == INACTIVE) reset_pose();
}


void reset_pose(
    volatile float& x, volatile float& y, volatile float& theta,
    volatile float& v, volatile float& w,
    volatile float& w_L, volatile float& w_R,
    volatile float& phi_L, volatile float& phi_R, 
    volatile float& imu_theta
) {
    // Reset de variables de pose
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
    v = 0.0f;
    w = 0.0f;
    w_L = 0.0f;
    w_R = 0.0f;

    // Reset de variables de acumulación (nadie más las puede reiniciar)
    phi_L = 0.0f;
    phi_R = 0.0f;
    imu_theta = 0.0f;

    // Reset de variables auxiliares
    last_millis = millis();
    last_phiL = 0.0f;
    last_phiR = 0.0f;
    last_imu_theta = 0.0f;
}


void init(
    volatile float& x, volatile float& y, volatile float& theta,
    volatile float& v, volatile float& w,
    volatile float& w_L, volatile float& w_R,
    volatile float& phi_L, volatile float& phi_R, 
    volatile float& imu_theta,
    volatile uint8_t& pose_state
) {
    reset_pose(x, y, theta, v, w, w_L, w_R, phi_L, phi_R, imu_theta);
    set_state(INACTIVE, pose_state);
}


void update_pose_encoder(
    const float encoder_phiL, const float encoder_phiR, 
    const float encoder_wL, const float encoder_wR,         
    volatile float& x, volatile float& y, volatile float& theta,   
    volatile float& v, volatile float& w,                          
    volatile float& w_L, volatile float& w_R,                      
    const uint8_t pose_state                                   
    ) {
    if (pose_state != ACTIVE) return;

    // 1. Establecer velocidades desde datos de los encoders
    w_L = encoder_wL;
    w_R = encoder_wR;
    v = WHEEL_RADIUS * (encoder_wR + encoder_wL) / 2.0f;
    w = (WHEEL_RADIUS / WHEELS_SEPARATION) * (encoder_wR - encoder_wL);

    // 2. Calcular variación de giro y avance
    const float delta_phiL = encoder_phiL - last_phiL;
    const float delta_phiR = encoder_phiR - last_phiR;
    const float ddist = WHEEL_RADIUS * (delta_phiR + delta_phiL) / 2.0f;
    const float dtheta = (WHEEL_RADIUS / WHEELS_SEPARATION) * (delta_phiR - delta_phiL);

    // 3. Integración numérica (con valor promedio de theta)
    const float theta_now = wrap_to_pi(theta + dtheta);
    const float avg_theta = wrap_to_pi((theta + theta_now) / 2.0f); // Mantener entre 0 y 2pi
    theta = theta_now;
    x += ddist * cosf(avg_theta);
    y += ddist * sinf(avg_theta);

    // 4. Guardar auxuliares para el próximo ciclo
    last_phiL = encoder_phiL;
    last_phiR = encoder_phiR;
}   


void update_pose_imu(      
    const float imu_acc, const float imu_w, const float imu_theta, 
    volatile float& x, volatile float& y, volatile float& theta,             
    volatile float& v, volatile float& w, 
    volatile float& w_L, volatile float& w_R,                  
    const uint8_t pose_state                 
    ) {
    if (pose_state != ACTIVE) return;

    // 1. Paso de tiempo
    const uint64_t now = millis();
    const float dt = (now - last_millis) * MS_TO_S; 
    if (dt <= 0.001f) return; // Protección contra dt muy pequeño
    last_millis = now;

    // 2. Velocidades desde IMU: integración de la velocidad y corrección de giro por sensor desalineado
    const float v_imu = v + imu_acc * dt; // Por integración simple
    const float w_imu_corr = imu_w - v_imu * IMU_CORRECTION_FACTOR*0;
    const float theta_imu = wrap_to_pi(imu_theta);

    // 3. Integración trapezoidal para la posición
    const float v_avg = (v + v_imu) / 2.0f;
    const float theta_avg = (theta + theta_imu) / 2.0f;
    x += v_avg * cosf(theta_avg) * dt;
    y += v_avg * sinf(theta_avg) * dt;

    // 4. Actualización de variables globales de velocidad
    v = v_imu; 
    w = w_imu_corr; 
    theta = theta_imu;
    Serial.println(theta);
    w_L = (v_imu - w_imu_corr * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;
    w_R = (v_imu + w_imu_corr * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;

    // 5. Guardar auxiliares para el próximo ciclo
    last_imu_theta = theta_imu;
}   


void update_pose_fusion(
    const float encoder_phiL, const float encoder_phiR,
    const float encoder_wL, const float encoder_wR,          
    const float imu_acc, const float imu_w, const float imu_theta, 
    volatile float& x, volatile float& y, volatile float& theta,             
    volatile float& v, volatile float& w,                           
    volatile float& w_L, volatile float& w_R,                       
    const uint8_t pose_state                 
    ) {
    if (pose_state != ACTIVE) return;

    // 0. Paso de tiempo
    const uint64_t now = millis();
    const float dt = (now - last_millis) * MS_TO_S; 
    if (dt <= 0.001f) return; // Protección contra dt muy pequeño

    // 1. Variables locales
    float w_fused = 0.0f;
    float theta_fused = 0.0f;

    // 2. Velocidades y dirección desde encoders
    const float v_encoder = WHEEL_RADIUS * (encoder_wR + encoder_wL) / 2.0f;
    const float w_encoder = (WHEEL_RADIUS / WHEELS_SEPARATION) * (encoder_wR - encoder_wL);
    const float delta_phiL = encoder_phiL - last_phiL;
    const float delta_phiR = encoder_phiR - last_phiR;
    const float dtheta_encoder = (WHEEL_RADIUS / WHEELS_SEPARATION) * (delta_phiR - delta_phiL);
    const float theta_encoder = wrap_to_pi(theta + dtheta_encoder); // Integración simple

    // 3. Velocidades desde IMU: integración de la velocidad y corrección de giro por sensor desalineado
    const float w_imu_corr = imu_w - v_encoder * IMU_CORRECTION_FACTOR;
    const float theta_imu_corr = wrap_to_pi(theta + (imu_theta-last_imu_theta));

    // 4. Fusión sensorial (complementary filter)
    w_fused = w_encoder * FUSION_ALPHA_W + w_imu_corr * (1.0f - FUSION_ALPHA_W);
    theta_fused = theta_encoder * FUSION_ALPHA_THETA + theta_imu_corr * (1.0f - FUSION_ALPHA_THETA);

    // 5. Integración trapezoidal para la posición
    const float v_avg = (v + v_encoder) / 2.0f;
    const float theta_avg = (theta + theta_fused) / 2.0f;
    x += v_avg * cosf(theta_avg) * dt;
    y += v_avg * sinf(theta_avg) * dt;

    // 6. Actualización de variables globales de velocidad
    v = v_encoder; 
    w = w_fused; 
    theta = theta_fused;
    w_L = (v_encoder - w_fused * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;
    w_R = (v_encoder + w_fused * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;

    // 7. Guardar auxiliares para el próximo ciclo
    last_phiL = encoder_phiL;
    last_phiR = encoder_phiR;
    last_imu_theta = imu_theta;
    last_millis = now;
}   


float wrap_to_pi(float angle) {
    angle = fmodf(angle + M_PI, 2.0f * M_PI);
    if (angle < 0.0f) angle += 2.0f * M_PI;
    return angle - PI;
}


void Task_PoseEstimatorEncoder(void* pvParameters) {
    // Configuración del periodo de muestreo
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(POSE_ESTIMATOR_PERIOD_MS);
    // Recuperar variables globales
    GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    volatile SystemStates& sts = *ctx_ptr->systems_ptr;
    volatile SensorsData& sens = *ctx_ptr->sensors_ptr;
    volatile PoseData& pose = *ctx_ptr->pose_ptr;
    // Ejecutar tarea periodicamente
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        if (pose.estimator_type == PoseEstimatorType::ENCODER) {
            update_pose_encoder(
                sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, 
                pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R, 
                sts.pose
            );
        } else if (pose.estimator_type == PoseEstimatorType::IMU) {
            update_pose_imu(
                sens.imu_acc, sens.imu_w, sens.imu_theta, 
                pose.x, pose.y, pose.theta, pose.v, pose.w, 
                pose.w_L, pose.w_R, sts.pose
            );
        } else if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY || 
                   pose.estimator_type == PoseEstimatorType::KALMAN) {
            update_pose_fusion(
                sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, 
                sens.imu_acc, sens.imu_w, sens.imu_theta,
                pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R,
                sts.pose
            );
        }
    }
}
    
} // namespace PoseEstimator
