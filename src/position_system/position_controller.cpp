#include "position_controller.h"

namespace PositionController {
   
// Estado de movimiento anterior 
static MovingState last_moving_state = MovingState::STOPPED; 

// Estado interno del PID
static float integral_alpha = 0.0f;
static float last_alpha = 0.0f;
static float integral_rho = 0.0f; 
static float last_millis = 0.0f;

static float e1= 0.0f;
static float e2= 0.0f;
static float e3= 0.0f;

void init(
    volatile PositionControlMode& control_mode,
    volatile float& x_d_global, volatile float& y_d_global, volatile float& theta_d_global,
    volatile bool& waypoint_reached,
    volatile float& w_L_ref, volatile float& w_R_ref
) {
    control_mode = PositionControlMode::INACTIVE;
    waypoint_reached = false; 
    reset_pid_state();
    w_L_ref = 0.0f;
    w_R_ref = 0.0f;
    x_d_global = 0.0f; 
    y_d_global = 0.0f;
    theta_d_global = 0.0f;
}


bool set_control_mode(
    const PositionControlMode new_mode,
    volatile PositionControlMode& control_mode,
    volatile float& wL_ref, volatile float& wR_ref
) {
    // Si se entrega el mismo modo, no se hace nada
    if (new_mode == control_mode) return SUCCESS;
    control_mode = new_mode;
    reset_pid_state(); // Reiniciar el estado del PID
    last_moving_state = MovingState::IDLE;
    if (new_mode == PositionControlMode::INACTIVE) {
        // Si se pone en modo inactivo, se dejan las referencias de velocidad en cero
        wL_ref = 0.0f;
        wR_ref = 0.0f;
    }
    return SUCCESS; // Indicar que se cambió el modo correctamente
}


bool set_controller_type(
    const ControlType new_type,
    volatile ControlType& controller_type
) {
    // Si se entrega el mismo tipo o si el control está desactivado, no se hace nada
    if (new_type == controller_type) return ERROR;

    // Modificación del tipo de controlador
    controller_type = new_type;
    reset_pid_state(); // Reiniciar el estado del PID
    return SUCCESS; // Indicar que se cambió el tipo correctamente
}


bool set_waypoint(
    const float x_d, const float y_d, const float theta_d,
    volatile float& x_d_global, volatile float& y_d_global, volatile float& theta_d_global,
    volatile bool& waypoint_reached, volatile PositionControlMode& control_mode
) {
    if (control_mode == PositionControlMode::INACTIVE) {
        return ERROR; // No se actualiza si el modo es inactivo o manual
    }
    // Actualizar las coordenadas del destino
    x_d_global = x_d;
    y_d_global = y_d;
    theta_d_global = wrap_to_pi(theta_d); // Normalizar el ángulo al rango (-π, π]
    reset_pid_state(); // Reiniciar el estado del PID
    reset_backs();
    waypoint_reached = false; // Reiniciar la bandera de waypoint alcanzado
    // Serial.printf("waypoint_reached = %d\n", waypoint_reached);
    // Serial.println();
    return SUCCESS; // Indicar que se estableció el waypoint correctamente
}


bool set_diferential_waypoint(
    const float dist, const float delta_theta,
    volatile float& x_d_global, volatile float& y_d_global, volatile float& theta_d_global,
    volatile bool& waypoint_reached, const PositionControlMode control_mode
) {
    if (control_mode == PositionControlMode::INACTIVE) {
        return ERROR; // No se actualiza si el modo es inactivo o manual
    }
    // Actualizar las coordenadas del destino
    theta_d_global = wrap_to_pi(theta_d_global + delta_theta); 
    x_d_global +=  dist* cosf(theta_d_global);
    y_d_global += dist * sinf(theta_d_global);
    reset_pid_state(); // Reiniciar el estado del PID
    reset_backs();
    waypoint_reached = false; // Reiniciar la bandera de waypoint alcanzado
    return SUCCESS;
}


void set_wheel_speed_ref(
    const float w_L, const float w_R,
    volatile float& w_L_ref_global, volatile float& w_R_ref_global,
    const PositionControlMode control_mode
) {
    // Para el caso manual, igual se asegura que no superen el máximo
    if (control_mode == PositionControlMode::MANUAL) {
        w_L_ref_global = constrain(w_L, -WM_NOM, WM_NOM);
        w_R_ref_global = constrain(w_R, -WM_NOM, WM_NOM);

    // El controlador internamente debe hacer la saturación
    } else if (control_mode != PositionControlMode::INACTIVE){ 
        w_L_ref_global = w_L;
        w_R_ref_global = w_R;
    }
}


MovingState update_control_pid(
    const float x, const float y, const float theta,
    const float x_d, const float y_d, const float theta_d,
    volatile float& wL_ref,  volatile float& wR_ref,
    const PositionControlMode control_mode
) {
    // 0) Inicialización de variables
    const float dx = x_d - x;
    const float dy = y_d - y;
    float rho = 0.0f, alpha = 0.0f;
    float wL_ref_local = 0.0f, wR_ref_local = 0.0f;
    MovingState move_state = last_moving_state;

    // 1) Paso de tiempo
    const float now = millis() * MS_TO_S;
    const float dt = now - last_millis;
    if (dt < 0.001) return move_state; // Protección de tiempo mínimo
    last_millis = now;

    // 2) Condicional de operación: define alpha, rho, y aplica criterio de convergencia
    if (control_mode == PositionControlMode::MOVE) { // Caso de control de movimiento
        rho = sqrtf(dx*dx + dy*dy);                 // Distancia al objetivo
        alpha = wrap_to_pi(atan2f(dy, dx) - theta); // Error de orientación hacia el objetivo
        move_state = (rho < DISTANCE_TOLERANCE) ? MovingState::STOPPING : MovingState::ADVANCING;
    }       
    else if (control_mode == PositionControlMode::ALIGN) { // Caso de alineación hacia el objetivo
        // rho no se usa y se deja en cero
        alpha = wrap_to_pi(atan2f(dy, dx) - theta); // Error de orientación hacia el objetivo
        move_state = (fabsf(alpha) < ANGLE_TOLERANCE) ? MovingState::STOPPING : MovingState::ROTATING;
    } 
    else if (control_mode == PositionControlMode::ROTATE) { // Caso de rotación pura
        // rho no se usa y se deja en cero
        alpha = wrap_to_pi(theta_d - theta); // Error respecto al ángulo objetivo
        move_state = (fabsf(alpha) < ANGLE_TOLERANCE) ? MovingState::STOPPING : MovingState::ROTATING;
    }

    // 3) Control
    if (move_state != MovingState::STOPPING) { // Controlamos solo si estamos lejos del objetivo
        // Control PID en α
        const float derivative = (alpha - last_alpha) / dt;
        const float w_ref_raw = KP_ALPHA * alpha + KI_ALPHA * integral_alpha + KD_ALPHA * derivative;
        last_alpha = alpha;

        // Control tipo PI para la distancia -> se mantiene v_ref = 0.0 cuando: 
            // 1. Modo rotating
            // 2. Modo advancing + el ángulo > MAX_ANGLE_DEVIATION -> gira en el lugar hasta alinearse (ANGLE_TOLERANCE)
        float v_ref_raw = 0.0f;
        const float max_angle = (last_moving_state == MovingState::ROTATING) ? ANGLE_TOLERANCE : MAX_ANGLE_DEVIATION;
        if (move_state != MovingState::ADVANCING || fabsf(alpha) > max_angle) {
            move_state = MovingState::ROTATING; // El estado de movimiento será de rotación
            integral_rho = 0.0f; // Reiniciar integral de rho
        } else {
            v_ref_raw = KP_RHO * rho + KI_RHO * integral_rho;
        }

        // Saturar referencias de velocidad para respetar límites de vel. de rueda
        const VelocityData sat_vel = constrain_velocity(v_ref_raw, w_ref_raw);
        const float v_ref_sat = sat_vel.v;
        const float w_ref_sat = sat_vel.w;
        wL_ref_local = sat_vel.wL;
        wR_ref_local = sat_vel.wR;

        // Anti-windup del integrador por back-calculation
        const float anti_wp_rho = (v_ref_raw - v_ref_sat) * KW_RHO;
        const float anti_wp_alpha = (w_ref_raw - w_ref_sat) * KW_ALPHA;
        integral_alpha += (alpha - anti_wp_alpha) * dt;
        integral_rho += (rho - anti_wp_rho) * dt;

        // Anti-windup del integrador por clamping
        integral_alpha = constrain(integral_alpha, -INTEGRAL_ALPHA_MAX, INTEGRAL_ALPHA_MAX);
        integral_rho = constrain(integral_rho, -INTEGRAL_RHO_MAX, INTEGRAL_RHO_MAX);
    } 
    else { // Si estamos deteniendo el vehículo las referencias de velocidad se dejan en cero
        // Además los integradores se reinician (no se usan por pasar directo la referencia de rueda)
        integral_alpha = 0.0f;
        integral_rho = 0.0f;
        last_alpha = 0.0f;
    }

    // 4) Asignar velocidad de referencia
    set_wheel_speed_ref(wL_ref_local, wR_ref_local, wL_ref, wR_ref, control_mode);
    last_moving_state = move_state; 
    return move_state; // Retorna true si se alcanzó el objetivo
}


void reset_pid_state() {
    // Reiniciar el estado del PID
    integral_alpha = 0.0f;
    integral_rho = 0.0f;
    last_alpha = 0.0f;
    last_millis = millis() * MS_TO_S;
}

void reset_backs() {
    
    e1= 0.0f;
    e2= 0.0f;
    e3= 0.0f;
    last_millis = millis() * MS_TO_S;
}



MovingState update_control_backstepping(
    const float x, const float y, const float theta,
    const float x_d, const float y_d, const float theta_d,
    volatile float& wL_ref, volatile float& wR_ref,
    const PositionControlMode control_mode
) {
    // 0) Inicialización de variables
    const float dx = x_d - x;
    const float dy = y_d - y;
    float v_ref_local = 0.0f, w_ref_local = 0.0f;
    MovingState move_state = last_moving_state;

    // 1) Errores cinemáticos en marco del robot
    float e1 = cosf(theta)*dx + sinf(theta)*dy;
    float e2 = -sinf(theta)*dx + cosf(theta)*dy;
    float rho = sqrtf(e1*e1 + e2*e2);
    float e3 = 0.0f; // Se sobreescribe más adelante

    // 2) Condiciones de control y referencias crudas
    if (control_mode == PositionControlMode::MOVE) { // Movimiento normal
        move_state = (rho < DISTANCE_TOLERANCE) ? MovingState::STOPPING : MovingState::ADVANCING;
        if (move_state == MovingState::ADVANCING) {
            const float max_angle = (last_moving_state == MovingState::ROTATING) ? ANGLE_TOLERANCE : MAX_ANGLE_DEVIATION;
            e3 = wrap_to_pi(atan2f(dy, dx) - theta); // Error angular respecto al objetivo
            if (fabs(e3) < max_angle) {
                v_ref_local = K1*e1;           // Si el ángulo es pequeño, avanzar
            } else {
                // Si el ángulo es grande, solo rotar y se mantiene la referencia de velocidad lineal en cero
                move_state = MovingState::ROTATING; // Cambiar el estado de movimiento a alineación
            }
            w_ref_local = K2*e3 + K3*e1*e2; // Siempre hay control angular
        }
    } 
    else if (control_mode == PositionControlMode::ALIGN || control_mode == PositionControlMode::ROTATE) {
        // La alineación deseada dependerá del modo de control
        float theta_d_local = (control_mode == PositionControlMode::ALIGN) ? atan2f(dy, dx) : theta_d; 
        e3 = wrap_to_pi(theta_d_local - theta); // Error angular respecto al objetivo
        if (fabs(e3) < ANGLE_TOLERANCE) {
            move_state = MovingState::STOPPING; // Si el error es pequeño, detener
        } else {
            move_state = MovingState::ROTATING; // Si no, rotar con control angular
            w_ref_local = K2*e3 + K3*e1*e2;
        }
    } 
    // 3) Aplicar saturación y aplicar a las ruedas
    const VelocityData sat_vel = constrain_velocity(v_ref_local, w_ref_local);
    set_wheel_speed_ref(sat_vel.wL, sat_vel.wR, wL_ref, wR_ref, control_mode);
    last_moving_state = move_state; // Actualizar el estado de movimiento global
    return move_state;
}



bool update_control(
    const float x, const float y, const float theta,
    const float x_d, const float y_d, const float theta_d,
    const float v, const float w,
    volatile float& wL_ref, volatile float& wR_ref,
    volatile bool& waypoint_reached,
    const ControlType controller_type,
    const PositionControlMode control_mode
) {
    // Verificar si el modo de control es válido
    if (control_mode == PositionControlMode::INACTIVE || control_mode == PositionControlMode::MANUAL) {
        return ERROR; // No se actualiza el control si está inactivo o en modo manual
    }

    // Llamar al controlador según el modo de control
    MovingState state = last_moving_state;
    if (controller_type == ControlType::PID) {
        state = update_control_pid(x, y, theta, x_d, y_d, theta_d, wL_ref, wR_ref, control_mode);
    } 
    else if (controller_type == ControlType::BACKS) {
        state = update_control_backstepping(x, y, theta, x_d, y_d, theta_d, wL_ref, wR_ref, control_mode);
    }
    // Verificar si se alcanzó el objetivo
    if (state == MovingState::STOPPING && fabsf(v) < V_STOP_THRESHOLD && fabsf(w) < W_STOP_THRESHOLD) {
        state = MovingState::STOPPED; // Si el vehículo está detenido, se considera que se alcanzó el objetivo
        waypoint_reached = true; // Marcar que se alcanzó el waypoint
    } else {
        waypoint_reached = false; // Si no se detuvo, no se alcanzó el waypoint
    }
    last_moving_state = state; // Actualizar el estado de movimiento global
    // if (waypoint_reached) Serial.printf("Objetivo alcanzado: (x = %.2f, y = %.2f). Estamos: (x = %.2f, y = %.2f)\n", x_d, y_d, x, y);
    return waypoint_reached; // Retorna true si se alcanzó el objetivo y el vehículo está detenido
}


bool stop_movement(
    const float v, const float w,
    volatile float& w_L_ref, volatile float& w_R_ref,
    volatile PositionControlMode& control_mode
) {
    // Fijar duty a cero
    set_control_mode(PositionControlMode::MANUAL, control_mode, w_L_ref, w_R_ref);
    set_wheel_speed_ref(0.0f, 0.0f, w_L_ref, w_R_ref, control_mode);

    // Verificar detención de motores
    const bool stopped = (v <= V_STOP_THRESHOLD && w <= W_STOP_THRESHOLD);
    last_moving_state = (stopped) ? MovingState::STOPPED : MovingState::STOPPING;
    return stopped;
}


VelocityData constrain_velocity(float v, float w) {
    // Cálculo de velocidad de rueda según modelo cinemático
    float wL = (v - w * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;
    float wR = (v + w * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;

    // Se revisa si alguna de las velocidades supera el límite técnico
    const float max_wheel_speed = fmaxf(fabsf(wL), fabsf(wR));
    if (max_wheel_speed > WM_NOM) {

        // Si supera alguna, se rescalan ambas para que la mayor esté a máxima
        // Esto permite que el vehículo tenga la velocidad angular adecuada
        const float scale = WM_NOM / max_wheel_speed;
        v *= scale;
        w *= scale;

        // Recalcular las velocidades de rueda considerando el escalamiento
        wL = (v - w * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;
        wR = (v + w * WHEEL_TO_MID_DISTANCE) / WHEEL_RADIUS;
    }
    VelocityData data = {v, w, wL, wR};
    return data;
}


void Task_PositionControl(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period  = pdMS_TO_TICKS(POSITION_CONTROL_PERIOD_MS);

    // Obtener el contexto global
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SystemStates& sts = *(ctx->systems_ptr);
    volatile PoseData& pose = *(ctx->pose_ptr); 
    volatile ControllerData& ctrl = *(ctx->control_ptr);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        update_control(
            pose.x, pose.y, pose.theta, ctrl.x_d, ctrl.y_d, ctrl.theta_d,
            pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, 
            ctrl.waypoint_reached, ctrl.controller_type, sts.position);
    }
}


float wrap_to_pi(float angle) {
    angle = fmodf(angle + PI, 2.0f * PI);
    if (angle < 0.0f) angle += 2.0f * PI;
    return angle - PI;
}

} // namespace PositionController
