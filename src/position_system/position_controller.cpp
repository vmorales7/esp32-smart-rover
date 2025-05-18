#include "position_controller.h"

float normalize_angle(float a) {
    // fuerza a ∈ (−π, π]
    while (a >  PI) a -= 2.0f * PI;
    while (a <= -PI) a += 2.0f * PI;
    return a;
}

float saturate(float v, float limit) {
    if      (v >  limit) return  limit;
    else if (v < -limit) return -limit;
    else                 return  v;
}


namespace PositionController {

    // estado interno del PID
    static float integral_alpha   = 0.0f;
    static float last_error_alpha = 0.0f;
    static float last_millis = 0.0f;

    void init(
        volatile uint8_t& control_mode,
        volatile float& wL_ref, volatile float& wR_ref
    ) {
        control_mode = SPEED_REF_INACTIVE;
        wL_ref = 0.0f;
        wR_ref = 0.0f;
    }

    void set_control_mode(
        const uint8_t new_mode,
        volatile uint8_t& control_mode,
        volatile float& wL_ref, volatile float& wR_ref
    ) {
        // Si se entrega el mismo modo, no se hace nada
        if (new_mode == control_mode) return;

        // Siempre se reincian las referencias
        control_mode = new_mode;
        wL_ref = 0.0f;
        wR_ref = 0.0f;

        // Acá irían los condicionales según caso
    }

    float compute_wheel_speed_ref(const float v_ref, const float w_ref, const uint8_t wheel_id) {
        float rotation_term = w_ref * WHEEL_DISTANCE / 2.0f;
        if (wheel_id == WHEEL_LEFT)
            return (v_ref - rotation_term) / WHEEL_RADIUS;
        else if (wheel_id == WHEEL_RIGHT)
            return (v_ref + rotation_term) / WHEEL_RADIUS;
        else
            return 0.0f; // default / error
    }

    void set_wheel_speed_ref(
        const float wL, const float wR,
        volatile float& wL_ref_global, volatile float& wR_ref_global,
        volatile uint8_t& control_mode
    ) {
        if (control_mode == SPEED_REF_MANUAL) {
            wL_ref_global = constrain(wL, -WM_NOM, WM_NOM);
            wR_ref_global = constrain(wR, -WM_NOM, WM_NOM);

        // El controlador internamente debe hacer el ajuste
        } else if (control_mode == SPEED_REF_AUTO_BASIC || control_mode == SPEED_REF_AUTO_ADVANCED){ 
            wL_ref_global = wL;
            wR_ref_global = wR;
        }
    }

    void compute_auto_wheel_speed(
        const float x,
        const float y,
        const float theta,
        const float x_d,
        const float y_d,
        volatile uint8_t& control_mode,
        volatile float& wL_ref,
        volatile float& wR_ref
    ) {
        if (control_mode != SPEED_REF_AUTO_BASIC) return;

        float now = millis() * MS_TO_S;
        float dt = now - last_millis;

        // 1) errores en X-Y
        float dx    = x_d - x;
        float dy    = y_d - y;
        float rho   = sqrtf(dx*dx + dy*dy);
        float beta  = atan2f(dy, dx);
        float alpha = normalize_angle(beta - theta);

        float v_ref = 0.0f, w_ref = 0.0f;

        if(fabsf(alpha) > ANGLE_TOLERANCE) {
            // Controlamos solo el ángulo
            if (rho > DISTANCE_TOLERANCE) {
                w_ref = Kp_alpha * alpha;
                integral_alpha = 0.0f;
                last_error_alpha = alpha;
            }
            else {
                // reset PID
                v_ref = 0.0f;
                w_ref = 0.0f;
            }

        }

        else if (rho > DISTANCE_TOLERANCE) {
            // PID en α
                integral_alpha   += alpha * dt;
                float derivative = (alpha - last_error_alpha) / dt;
                w_ref = Kp_alpha * alpha + Ki_alpha * integral_alpha + Kd_alpha * derivative;
                last_error_alpha = alpha;
                // P en ρ
                v_ref = Kv_rho * rho;

        } else {
            // reset PID
            integral_alpha   = 0.0f;
            last_error_alpha = 0.0f;
            v_ref = 0.0f;   
            w_ref = 0.0f;
        }

        // saturar dentro de limites físicos
        v_ref = saturate(v_ref, 0.5f);
        w_ref = saturate(w_ref, 2.0f);

        // 2) convertir a ruedas
        float wL = compute_wheel_speed_ref(v_ref, w_ref, WHEEL_LEFT);
        float wR = compute_wheel_speed_ref(v_ref, w_ref, WHEEL_RIGHT);

        set_wheel_speed_ref(wL, wR, wL_ref, wR_ref, control_mode);
        last_millis = now;
    }

    void compute_auto_wheel_speed_advanced(
        const float x, const float y, const float theta,
        const float x_d, const float y_d,
        volatile uint8_t& control_mode,
        volatile float& wL_ref, volatile float& wR_ref
    ) {
        if (control_mode != SPEED_REF_AUTO_ADVANCED) return;

        // 1) errores en marco vehículo
        float dx   = x_d - x;
        float dy   = y_d - y;

        float e1   =  cosf(theta)*dx + sinf(theta)*dy;
        float e2   = -sinf(theta)*dx + cosf(theta)*dy;
        float e3   = normalize_angle(atan2f(dy, dx) - theta);

        float rho  = sqrtf(e1*e1 + e2*e2);
        bool stop = (rho <= DISTANCE_TOLERANCE); 

        // 2) Control
        float v_des = 0.0f;
        float w_des = 0.0f;
        if (stop) {
            v_des = 0.0f;
            w_des = 0.0f;
        } else if (fabsf(e3) > ANGLE_TOLERANCE) {
            v_des = 0.0f;
            w_des = Kp_alpha * e3;
        } else {
            v_des = K1 * e1;
            w_des = K2 * e2 + K3 * e1 * e2 * e3;
        }

        // 3) saturar
        v_des = saturate(v_des, 0.5f);
        w_des = saturate(w_des, 1.0f);

        // 4) convertir a ruedas
        wL_ref = compute_wheel_speed_ref(v_des, w_des, WHEEL_LEFT);
        wR_ref = compute_wheel_speed_ref(v_des, w_des, WHEEL_RIGHT);
    }

    void Task_PositionControl(void* pvParameters) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period  = pdMS_TO_TICKS(POSITION_CONTROL_PERIOD_MS);

        GlobalContext* ctx   = static_cast<GlobalContext*>(pvParameters);
        auto*           sys   = ctx->systems_ptr;
        auto*           kin   = ctx->kinematic_ptr;
        auto*           whl   = ctx->wheels_ptr;

        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            uint8_t mode = sys->position;

            if (mode == SPEED_REF_AUTO_BASIC) {
                compute_auto_wheel_speed(
                    kin->x,   kin->y,   kin->theta,
                    kin->x_d, kin->y_d,
                    sys->position,
                    whl->wL_ref, whl->wR_ref
                );
            }
            else if (mode == SPEED_REF_AUTO_ADVANCED) {
                compute_auto_wheel_speed_advanced(
                    kin->x,   kin->y,   kin->theta,
                    kin->x_d, kin->y_d,
                    sys->position,                
                    whl->wL_ref, whl->wR_ref
                );
            }
        }
    }

}
