#include "pose_estimator.h"

namespace PoseEstimator {

    // Variable auxiliar para acumular lectura previa de steps
    static int64_t last_steps_left = 0;
    static int64_t last_steps_right = 0;

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
        float theta = theta_prev + dtheta;

        // Integración numérica de la posición global
        float avg_theta = (theta_prev + theta) / 2.0f;
        float dx = ddist * cosf(avg_theta);
        float dy = ddist * sinf(avg_theta);

        // Actualizar posición
        pose.theta = fmodf(theta + PI, 2.0f * PI) - PI; // Mantener entre -pi y +pi
        pose.x = x_prev + dx;
        pose.y = y_prev + dy;
        
        // Se actualiza la velocidad lineal y angular
        pose.v = (wR_measured + wL_measured) * WHEEL_RADIUS / 2.0f;
        pose.w = (wR_measured - wL_measured) * WHEEL_RADIUS / WHEEL_DISTANCE;

        return pose;
    }

    PoseData estimate_pose_from_encoder(
        volatile float* x_ptr, volatile float* y_ptr, volatile float* theta_ptr,
        volatile float* v_ptr, volatile float* w_ptr,
        volatile float* wL_ptr, volatile float* wR_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr
    ) {
        // Partimos guardando una foto de cada variable
        float x_prev = *x_ptr;
        float y_prev = *y_ptr;
        float theta_prev = *theta_ptr;
        float wL_measured = *wL_ptr;
        float wR_measured = *wR_ptr;
        int64_t steps_left = *steps_left_ptr;
        int64_t steps_right = *steps_right_ptr;
                
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

    void set_pose_estimator_state(uint8_t mode, volatile uint8_t* pose_estimator_state_ptr) {
        *pose_estimator_state_ptr = (mode == ACTIVE) ? ACTIVE : INACTIVE;
    }

    void reset_pose_and_steps(
        volatile float* x_ptr, volatile float* y_ptr, volatile float* theta_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr
    ) {
        *x_ptr = 0.0f;
        *y_ptr = 0.0f;
        *theta_ptr = 0.0f;
        *steps_left_ptr = 0;
        *steps_right_ptr = 0;
        last_steps_left = 0;
        last_steps_right = 0;
    }

    void update_pose(
        PoseData* pose_ptr,
        volatile float* x_ptr, volatile float* y_ptr, volatile float* theta_ptr,
        volatile float* v_ptr, volatile float* w_ptr
    ) {
        *x_ptr   = pose_ptr->x;
        *y_ptr   = pose_ptr->y;
        *theta_ptr = pose_ptr->theta;
        *v_ptr   = pose_ptr->v;
        *w_ptr   = pose_ptr->w;
    }
    

    void Task_PoseEstimatorEncoder(void* pvParameters) {
        // Datos de RTOS
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(POSE_ESTIMATOR_PERIOD_MS);
    
        // Recuperar la data pasada como input al Task (estructura de punteros)
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    
        // Punteros individuales
        volatile uint8_t* pose_state_ptr  = &ctx_ptr->systems_ptr->pose_estimator;
        volatile float* x_ptr             = &ctx_ptr->kinematic_ptr->x;
        volatile float* y_ptr             = &ctx_ptr->kinematic_ptr->y;
        volatile float* theta_ptr         = &ctx_ptr->kinematic_ptr->theta;
        volatile float* v_ptr             = &ctx_ptr->kinematic_ptr->v;
        volatile float* w_ptr             = &ctx_ptr->kinematic_ptr->w;
        volatile float* wL_ptr            = &ctx_ptr->wheels_ptr->w_measured_left;
        volatile float* wR_ptr            = &ctx_ptr->wheels_ptr->w_measured_right;
        volatile int64_t* steps_left_ptr  = &ctx_ptr->wheels_ptr->steps_left;
        volatile int64_t* steps_right_ptr = &ctx_ptr->wheels_ptr->steps_right;
    
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            if (*pose_state_ptr == ACTIVE) {
                PoseData pose = estimate_pose_from_encoder(
                    x_ptr, y_ptr, theta_ptr, v_ptr, w_ptr, wL_ptr, wR_ptr, steps_left_ptr, steps_right_ptr
                );
                update_pose(&pose, x_ptr, y_ptr, theta_ptr, v_ptr, w_ptr);
            }
        }
    }
    
}
