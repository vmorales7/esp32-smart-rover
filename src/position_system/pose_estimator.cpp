#include "pose_estimator.h"

namespace PoseEstimator {

    // Variable auxiliar para acumular lectura previa de steps
    static int64_t last_steps_left = 0;
    static int64_t last_steps_right = 0;

    PoseData compute_encoder_pose(
        float delta_phi_L, float delta_phi_R,
        float w_L, float w_R,
        float x_prev, float y_prev, float theta_prev
    ) {
        PoseData pose;

        // Calcular variación de giro y avance
        float dtheta = (WHEEL_RADIUS / WHEEL_DISTANCE) * (delta_phi_R - delta_phi_L);
        float ddist = (WHEEL_RADIUS / 2.0f) * (delta_phi_R + delta_phi_L);

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
        pose.v = (w_R + w_L) * WHEEL_RADIUS / 2.0f;
        pose.w = (w_R - w_L) * WHEEL_RADIUS / WHEEL_DISTANCE;
        // Futuro yo: quizás considerar calcular las velocidades a partir de delta_phiL/R para que no sea filtrado

        return pose;
    }

    
    PoseData estimate_pose_from_encoder(
        volatile float& w_L, volatile float& w_R,
        volatile int64_t& steps_L, volatile int64_t& steps_R,
        volatile float& x, volatile float& y, volatile float& theta,
        volatile float& v, volatile float& w
    ) {
        // Partimos guardando una foto de cada variable
        float x_prev = x;
        float y_prev = y;
        float theta_prev = theta;
        float wL_measured = w_L;
        float wR_measured = w_R;
        int64_t steps_left = steps_L;
        int64_t steps_right = steps_R;
                
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


    void set_state(const uint8_t new_state, volatile uint8_t& pose_state) {
        if (new_state == pose_state) return;
        pose_state = (new_state == ACTIVE) ? ACTIVE : INACTIVE;

        // if (new_state == INACTIVE) reset_pose();
    }


    void reset_pose(
        volatile float& x, volatile float& y, volatile float& theta,
        volatile float& v, volatile float& w,
        volatile int64_t& steps_L, volatile int64_t& steps_R
    ) {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        v = 0.0f;
        w = 0.0f;

        steps_L = 0;
        steps_R = 0;
        last_steps_left = 0;
        last_steps_right = 0;
    }


    void init(
        volatile float& x, volatile float& y, volatile float& theta,
        volatile float& v, volatile float& w,
        volatile int64_t& steps_L, volatile int64_t& steps_R,
        volatile uint8_t& pose_state
    ) {
        reset_pose(x, y, theta, v, w, steps_L, steps_R);
        set_state(INACTIVE, pose_state);
    }


    void update_pose(
        volatile float& w_L, volatile float& w_R,
        volatile int64_t& steps_L, volatile int64_t& steps_R,
        volatile float& x, volatile float& y, volatile float& theta,
        volatile float& v, volatile float& w,
        volatile uint8_t& pose_state
    ) {
        if (pose_state != ACTIVE) return;

        // Cálculo desde encoder
        PoseData encoder_pose = estimate_pose_from_encoder(w_L, w_R, steps_L, steps_R, x, y, theta, v, w);

        // Acá iría la llamada al pose desde imu

        // Llamar función de fusión sensorial
        PoseData pose = encoder_pose;

        // Actualizar salida
        x     = pose.x;
        y     = pose.y;
        theta = pose.theta;
        v     = pose.v;
        w     = pose.w;
    }
    

    void Task_PoseEstimatorEncoder(void* pvParameters) {
        // Configuración del periodo de muestreo
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(POSE_ESTIMATOR_PERIOD_MS);

        // Acceso al contexto global
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        auto& whl   = *ctx_ptr->wheels_ptr;
        auto& kin   = *ctx_ptr->kinematic_ptr;
        auto& state = *ctx_ptr->systems_ptr;

        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            update_pose(
                whl.w_L, whl.w_R, whl.steps_L, whl.steps_R,
                kin.x, kin.y, kin.theta, kin.v, kin.w,
                state.pose
            );
        }
    }

    
    static float wrap_to_pi(float angle) {
        angle = fmodf(angle + PI, 2.0f * PI);
        if (angle < 0.0f) angle += 2.0f * PI;
        return angle - PI;
    }
    
}
