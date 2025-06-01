#include "evade_controller.h"

// ---- Implementación EvadeController ----
namespace EvadeController {

// --- Inicializa el contexto (llamar en enter_evade) ---
void init(GlobalContext* global_ctx_ptr) {
    EvadeContext ctx = *(global_ctx_ptr->evade_ptr);
    ctx.state = EvadeState::SELECT_DIR;
    ctx.direction = 0;
    ctx.current_angle = 0.0f;
    ctx.tried_both_sides = false;
    ctx.start_theta = 0.0f;
    ctx.last_side_tried = 0;
    ctx.saved_waypoint = {0.0f, 0.0f};
    ctx.waypoint_active = false;
    ctx.evade_failed = false;
}

void start_evade(GlobalContext* global_ctx_ptr) {
    volatile PoseData& pose = *(global_ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(global_ctx_ptr->control_ptr);
    EvadeContext ctx = *(global_ctx_ptr->evade_ptr);

    // Guarda el waypoint original y orientación inicial
    ctx.saved_waypoint.x = ctrl.x_d;
    ctx.saved_waypoint.y = ctrl.y_d;
    ctx.start_theta = pose.theta;

    init(ctx);
}

int8_t choose_best_direction(const SensorsData& sens) {
    if (sens.us_left_dist > sens.us_right_dist) return +1; // izquierda
    else return -1; // derecha (o igual)
}


bool has_free_space(const SensorsData& sens) {
    // Considera el sensor frontal (puedes ajustar a lateral si giraste mucho)
    return (sens.us_mid_dist >= EVATION_MIN_SPACE);
}

// --- Fija waypoint diferencial para girar ---
void set_differential_theta(GlobalContext* ctx, float delta_theta) {
    volatile SystemStates& sts = *(ctx->systems_ptr);
    volatile ControllerData& ctrl = *(ctx->control_ptr);

    // Gira delta_theta relativo a orientación actual
    PositionController::set_diferential_waypoint(
        0.0f, delta_theta, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
}

// --- Fija waypoint diferencial para avanzar ---
void set_differential_advance(GlobalContext* ctx, float dist) {
    volatile SystemStates& sts = *(ctx->systems_ptr);
    volatile ControllerData& ctrl = *(ctx->control_ptr);

    PositionController::set_diferential_waypoint(
        dist, 0.0f, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
}

// --- Restaura el waypoint original ---
void restore_original_waypoint(GlobalContext* ctx, const TargetPoint& wp) {
    volatile SystemStates& sts = *(ctx->systems_ptr);
    volatile ControllerData& ctrl = *(ctx->control_ptr);

    PositionController::set_waypoint(
        wp.x, wp.y, 0.0f, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
}

// --- Actualiza la mini-máquina de estados de evasión ---
void update(GlobalContext* global_ctx, EvadeContext& ctx) {
    volatile SensorsData& sens = *(global_ctx->sensors_ptr);
    volatile ControllerData& ctrl = *(global_ctx->control_ptr);

    switch (ctx.state) {
        case EvadeState::SELECT_DIR: {
            ctx.direction = choose_best_direction(sens);
            ctx.current_angle = 0.0f;
            ctx.tried_both_sides = false;
            ctx.state = EvadeState::ALIGN_TO_FREE_PATH;
            break;
        }

        case EvadeState::ALIGN_TO_FREE_PATH: {
            // Fija nuevo theta diferencial (giro en el sentido elegido)
            set_differential_theta(global_ctx, ctx.direction * DELTA_THETA);
            ctx.waypoint_active = true;
            ctx.state = EvadeState::CHECK_SPACE;
            break;
        }

        case EvadeState::CHECK_SPACE: {
            // Espera a que termine el align (control principal)
            if (ctrl.waypoint_reached && ctx.waypoint_active) {
                // Lectura forzada de sensor en nueva dirección
                DistanceSensors::force_check_all_sensors(global_ctx);

                if (has_free_space(sens)) {
                    ctx.state = EvadeState::ADVANCE;
                } else {
                    // Aumenta el ángulo
                    ctx.current_angle += DELTA_THETA;
                    if (ctx.current_angle + DELTA_THETA > MAX_EVASION_ANGLE) {
                        // Intentó demasiado de este lado, probar el otro (si no lo ha hecho)
                        if (!ctx.tried_both_sides) {
                            ctx.direction = -ctx.direction; // cambia sentido
                            ctx.current_angle = 0.0f;
                            ctx.tried_both_sides = true;
                            ctx.state = EvadeState::ALIGN_TO_FREE_PATH;
                        } else {
                            ctx.state = EvadeState::FAIL; // No pudo evadir
                        }
                    } else {
                        ctx.state = EvadeState::ALIGN_TO_FREE_PATH;
                    }
                }
                ctx.waypoint_active = false;
            }
            break;
        }

        case EvadeState::ADVANCE: {
            // Fija waypoint para avanzar adelante
            set_differential_advance(global_ctx, EVASION_ADVANCE_DIST);
            ctx.state = EvadeState::RESTORE_PATH;
            ctx.waypoint_active = true;
            break;
        }

        case EvadeState::RESTORE_PATH: {
            if (ctrl.waypoint_reached && ctx.waypoint_active) {
                // Volver a ruta original
                restore_original_waypoint(global_ctx, ctx.saved_waypoint);
                ctx.state = EvadeState::FINISHED;
                ctx.waypoint_active = false;
            }
            break;
        }

        case EvadeState::FAIL: {
            // No pudo evadir: marca bandera y sale
            ctx.evade_failed = true;
            ctx.state = EvadeState::FINISHED;
            break;
        }

        case EvadeState::FINISHED:
        case EvadeState::IDLE:
        default:
            // No hacer nada, espera a que el OS principal saque del estado EVADE
            break;
    }
}

// --- Retorna true si la evasión terminó (éxito o no) ---
bool is_finished(const EvadeContext& ctx) {
    return (ctx.state == EvadeState::FINISHED);
}

// --- Retorna true si la evasión falló ---
bool has_failed(const EvadeContext& ctx) {
    return ctx.evade_failed;
}

} // namespace EvadeController
