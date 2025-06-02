#include "pose_estimator.h"

namespace PoseEstimator {

// Variables auxiliares para buffer de datos pasados
static int64_t last_millis = 0;
static int64_t last_steps_left = 0;
static int64_t last_steps_right = 0;


void set_state(const uint8_t new_state, volatile uint8_t& pose_state) {
    if (new_state == pose_state) return;
    pose_state = (new_state == ACTIVE) ? ACTIVE : INACTIVE;
    // if (new_state == INACTIVE) reset_pose();
}


void reset_pose(
    volatile float& x, volatile float& y, volatile float& theta,
    volatile float& v, volatile float& w,
    volatile float& w_L, volatile float& w_R,
    volatile int64_t& steps_L, volatile int64_t& steps_R
) {
    // Reset de variables de pose
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
    v = 0.0f;
    w = 0.0f;

    // Reset de variables de velocidad de ruedas
    w_L = 0.0f;
    w_R = 0.0f;
    steps_L = 0;
    steps_R = 0;

    // Locales
    last_millis = 0;
    last_steps_left = 0;
    last_steps_right = 0;
}


void init(
    volatile float& x, volatile float& y, volatile float& theta,
    volatile float& v, volatile float& w,
    volatile float& w_L, volatile float& w_R,
    volatile int64_t& steps_L, volatile int64_t& steps_R,
    volatile uint8_t& pose_state
) {
    reset_pose(x, y, theta, v, w, w_L, w_R, steps_L, steps_R);
    set_state(INACTIVE, pose_state);
}


void update_pose_encoder(
    const float encoder_stepsL, const float encoder_stepsR, 
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
    const float delta_phiL = (encoder_stepsL - last_steps_left) * RAD_PER_PULSE;
    const float delta_phiR = (encoder_stepsR - last_steps_right) * RAD_PER_PULSE;
    const float ddist = WHEEL_RADIUS * (delta_phiR + delta_phiL) / 2.0f;
    const float dtheta = (WHEEL_RADIUS / WHEELS_SEPARATION) * (delta_phiR - delta_phiL);

    // 3. Integración numérica (con valor promedio de theta)
    const float theta_now = wrap_to_pi(theta + dtheta);
    const float avg_theta = wrap_to_pi((theta + theta_now) / 2.0f); // Mantener entre 0 y 2pi
    theta = theta_now;
    x += ddist * cosf(avg_theta);
    y += ddist * sinf(avg_theta);

    // 4. Guardar auxuliares para el próximo ciclo
    last_steps_left = encoder_stepsL;
    last_steps_right = encoder_stepsR;
}   


void update_pose_fusion(
    const float wL_encoder, const float wR_encoder,          
    const float ax_imu, const float wz_imu, const float theta_imu, 
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
    last_millis = now;

    // 1. Variables locales
    float v_fused = 0.0f;
    float w_fused = 0.0f;
    float theta_fused = 0.0f;

    // 2. Velocidades desde encoders y desde IMU
    const float v_encoder = WHEEL_RADIUS * (wR_encoder + wL_encoder) / 2.0f;
    const float w_encoder = (WHEEL_RADIUS / WHEELS_SEPARATION) * (wR_encoder - wL_encoder);
    const float theta_encoder = wrap_to_pi(theta + (w + w_encoder)/2.0f * dt); // Integración simple
    const float v_imu = v + ax_imu * dt; // Por integración simple, asume LPF aplicado en ax_imu

    // 3. Fusión sensorial (complementary filter)
    v_fused = v_encoder * FUSION_ALPHA_V + v_imu * (1.0f - FUSION_ALPHA_V);
    w_fused = w_encoder * FUSION_ALPHA_W + wz_imu * (1.0f - FUSION_ALPHA_W);
    theta_fused = theta_encoder * FUSION_ALPHA_THETA + theta_imu * (1.0f - FUSION_ALPHA_THETA);

    // 4. Integración trapezoidal para la posición
    const float v_avg = (v + v_fused) / 2.0f;
    const float theta_avg = (theta + theta_fused) / 2.0f;
    x += v_avg * cosf(theta_avg) * dt;
    y += v_avg * sinf(theta_avg) * dt;

    // 5. Actualización de variables globales de velocidad
    v = v_fused; 
    w = w_fused; 
    theta = theta_fused;
    w_L = (v_fused - w_fused * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;
    w_R = (v_fused + w_fused * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;
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
                sens.enc_stepsL, sens.enc_stepsR, sens.enc_wL, sens.enc_wR, 
                pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R, 
                sts.pose
            );
        } else if (pose.estimator_type == PoseEstimatorType::FUSION) {
            update_pose_fusion(
                sens.enc_wL, sens.enc_wR, sens.imu_ax, sens.imu_wz, sens.imu_theta,
                pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R,
                sts.pose
            );
        }
    }
}
    
}

// PoseData compute_encoder_pose(
//     float delta_phi_L, float delta_phi_R,
//     float w_L, float w_R,
//     float x_prev, float y_prev, float theta_prev
// ) {
//     PoseData pose;

//     // Calcular variación de giro y avance
//     float dtheta = (WHEEL_RADIUS / WHEELS_SEPARATION) * (delta_phi_R - delta_phi_L);
//     float ddist = (WHEEL_RADIUS / 2.0f) * (delta_phi_R + delta_phi_L);

//     // Integración numérica del giro global
//     float theta = wrap_to_pi(theta_prev + dtheta);
    
//     // Integración numérica (trapezoidal simple) de la posición global
//     float avg_theta = (theta_prev + theta) / 2.0f; // Promediamos los valores de theta para mas exactitud
//     avg_theta = wrap_to_pi(avg_theta); // Mantener entre 0 y 2pi
//     float dx = ddist * cosf(avg_theta);
//     float dy = ddist * sinf(avg_theta);

//     // Actualizar posición
//     pose.theta = theta;
//     pose.x = x_prev + dx;
//     pose.y = y_prev + dy;
    
//     // Se actualiza la velocidad lineal y angular (ojo que se usa la velocidad que viene con un filtro)
//     pose.v = (w_R + w_L) * WHEEL_RADIUS / 2.0f;
//     pose.w = (w_R - w_L) * WHEEL_RADIUS / WHEELS_SEPARATION;
//     // Futuro yo: quizás considerar calcular las velocidades a partir de delta_phiL/R para que no sea filtrado

//     return pose;
// }


// PoseData estimate_pose_from_encoder(
//     volatile float& w_L, volatile float& w_R,
//     volatile int64_t& steps_L, volatile int64_t& steps_R,
//     volatile float& x, volatile float& y, volatile float& theta,
//     volatile float& v, volatile float& w
// ) {
//     // Partimos guardando una foto de cada variable
//     float x_prev = x;
//     float y_prev = y;
//     float theta_prev = theta;
//     float wL_measured = w_L;
//     float wR_measured = w_R;
//     int64_t steps_left = steps_L;
//     int64_t steps_right = steps_R;
            
//     // Calcular cambio en el giro de cada rueda usando la variable guardada
//     float delta_phiL = (steps_left - last_steps_left) * RAD_PER_PULSE;
//     float delta_phiR = (steps_right - last_steps_right) * RAD_PER_PULSE;

//     // Calcular la pose solo de los datos de encoder
//     PoseData pose = compute_encoder_pose(
//         delta_phiL, delta_phiR, wL_measured, wR_measured, x_prev, y_prev, theta_prev
//     );

//     // Actualizar últimos steps para el próximo ciclo
//     last_steps_left = steps_left;
//     last_steps_right = steps_right;

//     return pose;
// }


// void update_pose(
//     volatile float& w_L, volatile float& w_R,
//     volatile int64_t& steps_L, volatile int64_t& steps_R,
//     volatile float& x, volatile float& y, volatile float& theta,
//     volatile float& v, volatile float& w,
//     volatile uint8_t& pose_state
// ) {
//     if (pose_state != ACTIVE) return;

//     // Cálculo desde encoder
//     PoseData encoder_pose = estimate_pose_from_encoder(w_L, w_R, steps_L, steps_R, x, y, theta, v, w);

//     // Acá iría la llamada al pose desde imu

//     // Llamar función de fusión sensorial
//     PoseData pose = encoder_pose;

//     // Actualizar salida
//     x     = pose.x;
//     y     = pose.y;
//     theta = pose.theta;
//     v     = pose.v;
//     w     = pose.w;
// }

// void Task_PoseEstimatorEncoder(void* pvParameters) {
//     // Configuración del periodo de muestreo
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t period = pdMS_TO_TICKS(POSE_ESTIMATOR_PERIOD_MS);

//     // Acceso al contexto global
//     GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
//     auto& whl   = *ctx_ptr->wheels_ptr;
//     auto& kin   = *ctx_ptr->kinematic_ptr;
//     auto& state = *ctx_ptr->systems_ptr;

//     for (;;) {
//         vTaskDelayUntil(&xLastWakeTime, period);
//         update_pose(
//             whl.w_L, whl.w_R, whl.steps_L, whl.steps_R,
//             kin.x, kin.y, kin.theta, kin.v, kin.w,
//             state.pose
//         );
//     }
// }
