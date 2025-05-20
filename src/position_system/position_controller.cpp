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
    static float last_alpha = 0.0f;
    static float last_millis = 0.0f;


    void init(
        volatile PositionControlMode& control_mode,
        volatile float& w_L_ref, volatile float& w_R_ref
    ) {
        control_mode = PositionControlMode::INACTIVE;
        w_L_ref = 0.0f;
        w_R_ref = 0.0f;
    }


    void set_control_mode(
        const PositionControlMode new_mode,
        volatile PositionControlMode& control_mode,
        volatile float& w_L_ref, volatile float& w_R_ref
    ) {
        // Si se entrega el mismo modo, no se hace nada
        if (new_mode == control_mode) return;

        // Siempre se reincian las referencias
        control_mode = new_mode;
        w_L_ref = 0.0f;
        w_R_ref = 0.0f;

        // Acá irían los condicionales según caso
    }


    void set_wheel_speed_ref(
        const float w_L, const float w_R,
        volatile float& w_L_ref_global, volatile float& w_R_ref_global,
        volatile PositionControlMode& control_mode
    ) {
        if (control_mode == PositionControlMode::MANUAL) {
            w_L_ref_global = constrain(w_L, -WM_NOM, WM_NOM);
            w_R_ref_global = constrain(w_R, -WM_NOM, WM_NOM);

        // El controlador internamente debe hacer el ajuste
        } else if (control_mode != PositionControlMode::INACTIVE){ 
            w_L_ref_global = w_L;
            w_R_ref_global = w_R;
        }
    }


    void update_position_control_pid(
        const float x, const float y, const float theta,
        const float x_d, const float y_d,
        volatile float& w_L_ref,  volatile float& w_R_ref,
        volatile PositionControlMode& control_mode
    ) {
        if (control_mode != PositionControlMode::MOVE_BASIC) return;

        // 0) Paso de tiempo
        const float now = millis() * MS_TO_S;
        const float dt = now - last_millis;
        last_millis = now;
        if (dt < MIN_POS_DT) return; // Protección: solo actualizar cada cierto tiempo

        // 1) Cálculo de errores en X-Y
        const float dx = x_d - x;
        const float dy = y_d - y;
        const float rho   = sqrtf(dx*dx + dy*dy);
        const float alpha = normalize_angle(atan2f(dy, dx) - theta);

        // 2) Control
        float v_ref = 0.0f, w_ref = 0.0f;
        float v_ref_raw = 0.0f, w_ref_raw = 0.0f;
        float wL = 0.0f, wR = 0.0f;

        if (rho > DISTANCE_TOLERANCE) { // Controlamos solo si estamos lejos del objetivo
            // Control PID en α
            const float derivative = (alpha - last_alpha) / dt;
            last_alpha = alpha;
            w_ref_raw = Kp_alpha * alpha + Ki_alpha * integral_alpha + Kd_alpha * derivative;

            // Si el ángulo es muy grande, solo girar en el lugar sino control P en ρ
            if (fabsf(alpha) > ANGLE_NAVIGATION_TOLERANCE) {
                v_ref_raw = 0.0f;
            } else {
                v_ref_raw = Kp_rho * rho;
            }

            // Saturar referencias de velocidad para respetar límites de vel. de rueda
            const VelocityData data = constrain_velocity(v_ref_raw, w_ref_raw);
            v_ref = data.v;
            w_ref = data.w;
            wL = data.wL;
            wR = data.wR;

            // Anti-windup del integrador (por back-calculation)
            const float anti_wp = (w_ref_raw - w_ref) * Kw_alpha;
            integral_alpha += (alpha - anti_wp) * dt;
        } 
        else {
            // Objetivo alcanzado: reseteo
            integral_alpha = 0.0f;
            last_alpha = 0.0f;
        }
        // 3) Asignar velocidad de referencia
        set_wheel_speed_ref(wL, wR, w_L_ref, w_R_ref, control_mode);
    }


    void update_position_control_backs(
        const float x, const float y, const float theta,
        const float x_d, const float y_d,
        volatile float& wL_ref, volatile float& wR_ref,
        volatile PositionControlMode& control_mode
    ) {
        if (control_mode != PositionControlMode::MOVE_ADVANCED) return;

        // 1) errores en marco vehículo
        float dx = x_d - x;
        float dy = y_d - y;

        float e1 =  cosf(theta)*dx + sinf(theta)*dy;
        float e2 = -sinf(theta)*dx + cosf(theta)*dy;
        float e3 = normalize_angle(atan2f(dy, dx) - theta);

        float rho = sqrtf(e1*e1 + e2*e2);
        bool stop = (rho <= DISTANCE_TOLERANCE); 

        // 2) Control
        float v_des = 0.0f;
        float w_des = 0.0f;
        if (stop) {
            v_des = 0.0f;
            w_des = 0.0f;
        } else if (fabsf(e3) > ANGLE_NAVIGATION_TOLERANCE) {
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
        wL_ref = compute_wheel_speed(v_des, w_des, WHEEL_LEFT);
        wR_ref = compute_wheel_speed(v_des, w_des, WHEEL_RIGHT);
    }

    
    void update_wheel_speed_ref(
        const float x, const float y, const float theta,
        const float x_d, const float y_d,
        volatile float& wL_ref, volatile float& wR_ref,
        volatile PositionControlMode& control_mode
    ) {
        if (control_mode == PositionControlMode::MOVE_BASIC || control_mode == PositionControlMode::TURN_BASIC) {
            update_position_control_pid(x, y, theta, x_d, y_d, wL_ref, wR_ref, control_mode);
        } else if (control_mode == PositionControlMode::MOVE_ADVANCED || control_mode == PositionControlMode::TURN_ADVANCED) {
            update_position_control_backs(x, y, theta, x_d, y_d, wL_ref, wR_ref, control_mode);
        }
    }


    float compute_wheel_speed(const float v, const float w, const uint8_t wheel_id) {
        const float rotation_term = w * WHEEL_DISTANCE / 2.0f;
        if (wheel_id == WHEEL_LEFT)
            return (v - rotation_term) / WHEEL_RADIUS;
        else if (wheel_id == WHEEL_RIGHT)
            return (v + rotation_term) / WHEEL_RADIUS;
        else
            return 0.0f; // default / error
    }


    VelocityData constrain_velocity(float v_raw, float w_raw) {
        VelocityData data;
        data.v = v_raw;
        data.w = w_raw;

        // Directamente el cálculo de cada rueda:
        data.wL = (data.v - data.w * WHEEL_DISTANCE / 2.0f) / WHEEL_RADIUS;
        data.wR = (data.v + data.w * WHEEL_DISTANCE / 2.0f) / WHEEL_RADIUS;

        const float max_wheel_speed = fmaxf(fabsf(data.wL), fabsf(data.wR));
        if (max_wheel_speed > WM_NOM) {
            const float scale = WM_NOM / max_wheel_speed;
            data.v *= scale;
            data.w *= scale;

            // Recalcular tras el escalado:
            data.wL = (data.v - data.w * WHEEL_DISTANCE / 2.0f) / WHEEL_RADIUS;
            data.wR = (data.v + data.w * WHEEL_DISTANCE / 2.0f) / WHEEL_RADIUS;
        }
        return data;
    }


    void Task_PositionControl(void* pvParameters) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period  = pdMS_TO_TICKS(POSITION_CONTROL_PERIOD_MS);

        GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
        auto* sys = ctx->systems_ptr;
        auto* kin = ctx->kinematic_ptr;
        auto* whl = ctx->wheels_ptr;

        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            PositionControlMode mode = sys->position;
            update_wheel_speed_ref(
                kin->x, kin->y, kin->theta, kin->x_d, kin->y_d, whl->w_L_ref, whl->w_R_ref, mode
            );
        }
    }

}
