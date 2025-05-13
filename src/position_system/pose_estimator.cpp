#include "pose_estimator.h"

namespace PoseEstimator {

    // Variable auxiliar para acumular lectura previa de steps
    static int64_t last_steps_left = 0;
    static int64_t last_steps_right = 0;

    static float wrap_to_pi(float angle) {
        angle = fmodf(angle + PI, 2.0f * PI);
        if (angle < 0.0f) angle += 2.0f * PI;
        return angle - PI;
    }

    PoseData compute_encoder_pose(
        float delta_phiL, float delta_phiR,
        float wL_measured, float wR_measured,
        float x_prev, float y_prev, float theta_prev
    ) {
        PoseData pose;

        // Calcular variación de giro y avance
        float dtheta = (WHEEL_RADIUS / WHEEL_DISTANCE) * (delta_phiR - delta_phiL);
        float ddist = (WHEEL_RADIUS / 2.0f) * (delta_phiR + delta_phiL);

        // Integración numérica del giro global
        float theta = wrap_to_pi(theta_prev + dtheta);
        
        // Integración numérica (trapezoidal simple) de la posición global
        float avg_theta = (theta_prev + theta) / 2.0f; // Promediamos los valores de theta para mas exactitud
        avg_theta = wrap_to_pi(avg_theta); // Mantener entre 0 y 2pi
        float dx = ddist * cosf(avg_theta);
        float dy = ddist * sinf(avg_theta);

        // Actualizar posición
        pose.theta = theta;
        pose.x = x_prev + dx;
        pose.y = y_prev + dy;
        
        // Se actualiza la velocidad lineal y angular (ojo que se usa la velocidad que viene con un filtro)
        pose.v = (wR_measured + wL_measured) * WHEEL_RADIUS / 2.0f;
        pose.w = (wR_measured - wL_measured) * WHEEL_RADIUS / WHEEL_DISTANCE;
        // Futuro yo: quizás considerar calcular las velocidades a partir de delta_phiL/R para que no sea filtrado

        return pose;
    }

    PoseData estimate_pose_from_encoder(
        volatile float& x_var, volatile float& y_var, volatile float& theta_var,
        volatile float& v_var, volatile float& w_var,
        volatile float& wL_var, volatile float& wR_var,
        volatile int64_t& steps_left_var, volatile int64_t& steps_right_var
    ) {
        // Partimos guardando una foto de cada variable
        float x_prev = x_var;
        float y_prev = y_var;
        float theta_prev = theta_var;
        float wL_measured = wL_var;
        float wR_measured = wR_var;
        int64_t steps_left = steps_left_var;
        int64_t steps_right = steps_right_var;
                
        // Calcular cambio en el giro de cada rueda usando la variable guardada
        float delta_phiL = (steps_left - last_steps_left) * RAD_PER_PULSE;
        float delta_phiR = (steps_right - last_steps_right) * RAD_PER_PULSE;

        // Calcular la pose solo de los datos de encoder
        PoseData pose = compute_encoder_pose(
            delta_phiL, delta_phiR, wL_measured, wR_measured, x_prev, y_prev, theta_prev
        );

        // Actualizar últimos steps para el próximo ciclo
        last_steps_left = steps_left;
        last_steps_right = steps_right;

        return pose;
    }

    void set_state(const uint8_t new_state, volatile uint8_t& pose_estimator_state) {
        pose_estimator_state = (new_state == ACTIVE) ? ACTIVE : INACTIVE;
    }

    void reset_pose_and_steps(
        volatile float& x_var, volatile float& y_var, volatile float& theta_var,
        volatile int64_t& steps_left_var, volatile int64_t& steps_right_var
    ) {
        x_var = 0.0f;
        y_var = 0.0f;
        theta_var = 0.0f;
        steps_left_var = 0;
        steps_right_var = 0;
        last_steps_left = 0;
        last_steps_right = 0;
    }

    void init(
        volatile float& x_var, volatile float& y_var, volatile float& theta_var,
        volatile int64_t& steps_left_var, volatile int64_t& steps_right_var,
        volatile uint8_t& pose_estimator_state
    ) {
        reset_pose_and_steps(x_var, y_var, theta_var, steps_left_var, steps_right_var);
        set_state(INACTIVE, pose_estimator_state);
    }

    void update_pose(
        PoseData& pose,
        volatile float& x_var, volatile float& y_var, volatile float& theta_var,
        volatile float& v_var, volatile float& w_var,
        volatile uint8_t& pose_estimator_state
    ) {
        if (pose_estimator_state != ACTIVE) return;
        x_var     = pose.x;
        y_var     = pose.y;
        theta_var = pose.theta;
        v_var     = pose.v;
        w_var     = pose.w;
    }
    
    void Task_PoseEstimatorEncoder(void* pvParameters) {
        // Datos de RTOS
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(POSE_ESTIMATOR_PERIOD_MS);

        // Obtener referencias directas a las estructuras globales
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        auto& k = *ctx_ptr->kinematic_ptr;     // kinematic_state
        auto& w = *ctx_ptr->wheels_ptr;        // wheels_data
        auto& s = *ctx_ptr->systems_ptr;       // system_states

        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            if (s.pose_estimator == ACTIVE) {
                PoseData pose = estimate_pose_from_encoder(
                    k.x, k.y, k.theta, k.v, k.w,
                    w.wL_measured, w.wR_measured, w.steps_left, w.steps_right
                );
                update_pose(pose, k.x, k.y, k.theta, k.v, k.w, s.pose_estimator);
            }
        }
    }
    
}


    // float normalize_angle(float angle) {
    //     if (angle > PI)  {angle -= 2.0f * PI;}
    //     else if (angle < -PI) {angle += 2.0f * PI;}
    //     return angle;
    // }