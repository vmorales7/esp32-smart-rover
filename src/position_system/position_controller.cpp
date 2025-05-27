#include "position_controller.h"

namespace PositionController {

    // Estado interno del PID
    static float integral_alpha = 0.0f;
    static float last_alpha = 0.0f;
    static float integral_rho = 0.0f; 
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

        // Modificación del modo de operación
        control_mode = new_mode;
        w_L_ref = 0.0f;
        w_R_ref = 0.0f;

        // Acá irían los condicionales según caso
        if (new_mode != PositionControlMode::INACTIVE && new_mode != PositionControlMode::MANUAL) {
            // Resetear los integradores de PID al pasar a AUTO
            integral_alpha = 0.0f;
            integral_rho = 0.0f;
            last_alpha = 0.0f;
        }
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


    bool update_control_pid(
        const float x, const float y, const float theta,
        const float x_d, const float y_d, const float theta_d,
        volatile float& w_L_ref,  volatile float& w_R_ref,
        volatile PositionControlMode& control_mode
    ) {
        // 0) Paso de tiempo
        const float now = millis() * MS_TO_S;
        const float dt = now - last_millis;
        last_millis = now;
        if (dt < MIN_POS_DT) return false; // Protección: solo actualizar cada cierto tiempo

        // 1) Inicialización de variables
        float v_ref = 0.0f, w_ref = 0.0f;
        float v_ref_raw = 0.0f, w_ref_raw = 0.0f;
        float wL = 0.0f, wR = 0.0f;
        float rho = 0.0f, alpha = 0.0f;
        bool reached = false;

        // 2) Condicional de operación -> define alpha, rho, y los criterios de convergencia
        const float angle_tolerance = (control_mode == PositionControlMode::MOVE_PID) 
                                        ? ANGLE_NAVIGATION_TOLERANCE : ANGLE_ROTATION_TOLERANCE;
        if (control_mode == PositionControlMode::MOVE_PID) { // Caso de control de movimiento
            const float dx = x_d - x;
            const float dy = y_d - y;
            rho = sqrtf(dx*dx + dy*dy);
            alpha = wrap_to_pi(atan2f(dy, dx) - theta); // Error respecto al ángulo del punto objetivo
            reached = (rho <= DISTANCE_TOLERANCE);
        } else if (control_mode == PositionControlMode::TURN_PID) { // Caso de rotación pura
            // rho no se usa y se deja en cero
            alpha = wrap_to_pi(theta_d - theta); // Error respecto al ángulo objetivo
            reached = (fabsf(alpha) <= angle_tolerance);
        }

        // 3) Control
        if (!reached) { // Controlamos solo si estamos lejos del objetivo
            // Control PID en α
            const float derivative = (alpha - last_alpha) / dt;
            w_ref_raw = KP_ALPHA * alpha + KI_ALPHA * integral_alpha + KD_ALPHA * derivative;
            last_alpha = alpha;

            // Control tipo PI para la distancia. Si el ángulo es muy grande, solo gira en el lugar (mantiene vref en 0)
            if (fabsf(alpha) < angle_tolerance) {
                v_ref_raw = KP_RHO * rho + KI_RHO * integral_rho;
            }

            // Saturar referencias de velocidad para respetar límites de vel. de rueda
            const VelocityData data = constrain_velocity(v_ref_raw, w_ref_raw);
            v_ref = data.v;
            w_ref = data.w;
            wL = data.wL;
            wR = data.wR;

            // Anti-windup del integrador (por back-calculation)
            const float anti_wp_alpha = (w_ref_raw - w_ref) * KW_ALPHA;
            const float anti_wp_rho = (rho - v_ref) * KW_RHO;
            integral_alpha += (alpha - anti_wp_alpha) * dt;
            integral_rho += (rho - anti_wp_rho) * dt;
        } 
        else {
            // Objetivo alcanzado: reseteo
            integral_alpha = 0.0f;
            integral_rho = 0.0f;
            last_alpha = 0.0f;
        }

        // 4) Asignar velocidad de referencia
        set_wheel_speed_ref(wL, wR, w_L_ref, w_R_ref, control_mode);
        return reached; // Retorna true si se alcanzó el objetivo
    }


    // bool update_control_backstepping(
    //     const float x, const float y, const float theta,
    //     const float x_d, const float y_d, const float theta_d,
    //     volatile float& wL_ref, volatile float& wR_ref,
    //     volatile PositionControlMode& control_mode
    // ) {

    //     // 1) errores en marco vehículo
    //     float dx = x_d - x;
    //     float dy = y_d - y;

    //     float e1 =  cosf(theta)*dx + sinf(theta)*dy;
    //     float e2 = -sinf(theta)*dx + cosf(theta)*dy;
    //     float e3 = wrap_to_pi(atan2f(dy, dx) - theta);

    //     float rho = sqrtf(e1*e1 + e2*e2);
    //     bool stop = (rho <= DISTANCE_TOLERANCE); 

    //     // 2) Control
    //     float v_des = 0.0f;
    //     float w_des = 0.0f;
    //     if (stop) {
    //         v_des = 0.0f;
    //         w_des = 0.0f;
    //     } else if (fabsf(e3) > ANGLE_NAVIGATION_TOLERANCE) {
    //         v_des = 0.0f;
    //         w_des = KP_ALPHA * e3;
    //     } else {
    //         v_des = K1 * e1;
    //         w_des = K2 * e2 + K3 * e1 * e2 * e3;
    //     }

    //     // 3) saturar
    //     v_des = saturate(v_des, 0.5f);
    //     w_des = saturate(w_des, 1.0f);

    //     // 4) convertir a ruedas
    //     wL_ref = compute_wheel_speed(v_des, w_des, WHEEL_LEFT);
    //     wR_ref = compute_wheel_speed(v_des, w_des, WHEEL_RIGHT);

    //     return stop; // Retorna si se está controlando o si se alcanzó el objetivo
    // }

    bool update_control_backstepping(
        const float x, const float y, const float theta,
        const float x_d, const float y_d, const float theta_d,
        volatile float& wL_ref, volatile float& wR_ref,
        volatile PositionControlMode& control_mode
    ) {
        // 0) Inicialización de variables
        float v_ref = 0.0f, w_ref = 0.0f;
        bool reached = false;

        // 1) Errores cinemáticos en marco del robot
        const float dx = x_d - x;
        const float dy = y_d - y;
        const float e1 = cosf(theta)*dx + sinf(theta)*dy;
        const float e2 = -sinf(theta)*dx + cosf(theta)*dy;
        const float rho = sqrtf(e1*e1 + e2*e2);
        float e3 = 0.0f;

        // 2) Condiciones de control y referencias crudas
        const float angle_tolerance = (control_mode == PositionControlMode::MOVE_BACKS) 
                                        ? ANGLE_NAVIGATION_TOLERANCE : ANGLE_ROTATION_TOLERANCE;
        if (control_mode == PositionControlMode::TURN_BACKS) { // Giro puro → solo se usa control angular
            e3 = wrap_to_pi(theta_d - theta); 
            reached = (fabs(e3) <= angle_tolerance);
            if (!reached) {
                w_ref = K2*e2 + K3*e1*e2*e3;
            }
        } 
        else if (control_mode == PositionControlMode::MOVE_BACKS) { // Movimiento normal
            reached = (rho <= DISTANCE_TOLERANCE);
            if (!reached) {
                e3 = wrap_to_pi(atan2f(dy, dx) - theta);
                if (fabs(e3) <= angle_tolerance) v_ref = K1*e1;
                w_ref = K2*e2 + K3*e1*e2*e3;
            }
        }
        // 3) Aplicar saturación y aplicar a las ruedas
        const VelocityData data = constrain_velocity(v_ref, w_ref);
        set_wheel_speed_ref(data.wL, data.wR, wL_ref, wR_ref, control_mode);
        return reached;
    }

    
    bool update_control(
        const float x, const float y, const float theta,
        const float x_d, const float y_d, const float theta_d,
        volatile float& v, volatile float& w,
        volatile float& wL_ref, volatile float& wR_ref,
        volatile bool& target_reached,
        volatile PositionControlMode& control_mode
    ) {
        bool reached = false;
        if (control_mode == PositionControlMode::MOVE_PID || control_mode == PositionControlMode::TURN_PID) {
            reached =  update_control_pid(x, y, theta, x_d, y_d, theta_d, wL_ref, wR_ref, control_mode);
        } else if (control_mode == PositionControlMode::MOVE_BACKS || control_mode == PositionControlMode::TURN_BACKS) {
            reached = update_control_backstepping(x, y, theta, x_d, y_d, theta_d, wL_ref, wR_ref, control_mode);
        }
        target_reached = (reached && v <= V_STOP_THRESHOLD && w <= W_STOP_THRESHOLD);
        return target_reached; // Retorna true si se alcanzó el objetivo y el vehículo está detenido
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


    bool stop_movement(
        volatile float& v, volatile float& w,
        volatile float& w_L_ref, volatile float& w_R_ref,
        volatile PositionControlMode& control_mode
    ) {
        // Fijar duty a cero
        set_control_mode(PositionControlMode::MANUAL, control_mode, w_L_ref, w_R_ref);
        set_wheel_speed_ref(0.0f, 0.0f, w_L_ref, w_R_ref, control_mode);

        // Verificar detención de motores
        return (v <= V_STOP_THRESHOLD && w <= W_STOP_THRESHOLD);
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
            update_control(
                kin->x, kin->y, kin->theta, kin->x_d, kin->y_d, kin->theta_d,
                kin->v, kin->w, whl->w_L_ref, whl->w_R_ref,
                kin->target_reached, mode
            );
        }
    }


    float wrap_to_pi(float angle) {
        angle = fmodf(angle + PI, 2.0f * PI);
        if (angle < 0.0f) angle += 2.0f * PI;
        return angle - PI;
    }

}

// float normalize_angle(float a) {
//     // fuerza a ∈ (−π, π]
//     while (a >  PI) a -= 2.0f * PI;
//     while (a <= -PI) a += 2.0f * PI;
//     return a;
// }

// float saturate(float v, float limit) {
//     if      (v >  limit) return  limit;
//     else if (v < -limit) return -limit;
//     else                 return  v;
// }