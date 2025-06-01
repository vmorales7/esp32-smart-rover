#include "evade_controller.h"

namespace EvadeController {

bool run_evation(GlobalContext* ctx) {
    static SubState substate = SubState::SELECT_DIRECTION;
    static float origin_theta = 0.0f;
    static float theta_accum = 0.0f;     // Ángulo acumulado girado
    static int8_t direction = 0;         // +1 = derecha, -1 = izquierda
    static bool reverse_attempted = false;    // Flag para saber si ya probó reversa
    static PositionControlMode turn_controller = PositionControlMode::INACTIVE;
    static PositionControlMode move_controller = PositionControlMode::INACTIVE;

    auto& kin = *ctx->kinematic_ptr;
    auto& sts = *ctx->systems_ptr;
    auto& whl = *ctx->wheels_ptr;
    auto& dis = *ctx->distance_ptr;
    auto& os  = *ctx->os_ptr;

    switch (substate) {
        case SubState::SELECT_DIRECTION: {
            // Reiniciar variables internas cuando se entra en evasión
            origin_theta = kin.theta;
            theta_accum = 0.0f;
            reverse_attempted = false;

            // Revisar sensores laterales y elegir dirección con más espacio
            if (dis.left_dist >= dis.right_dist) {
                direction = +1; // Gira a la izquierda (theta aumenta)
            } else {
                direction = -1; // Gira a la derecha (theta disminuye)
            }

            // Seleccionar el modo de control
            if(os.control_type == ControlType::PID) {
                turn_controller = PositionControlMode::TURN_PID;
                move_controller = PositionControlMode::MOVE_PID;
            } else {
                turn_controller = PositionControlMode::TURN_BACKS;
                move_controller = PositionControlMode::MOVE_BACKS;
            }

            // Define el primer objetivo de giro: gira hacia la dirección elegida
            kin.theta_d = PositionController::wrap_to_pi(origin_theta + direction * DELTA_THETA);
            kin.x_d = kin.x; // Mantiene la posición X actual (en realidad el controlador no usa x_d)
            kin.y_d = kin.y; // Mantiene la posición Y actual (en realidad el controlador no usa y_d)

            // Se comienza el giro inicial
            PositionController::set_control_mode(turn_controller, sts.position, whl.w_L_ref, whl.w_R_ref, kin.moving_state);
            substate = SubState::TURN;
            break;
        }

        case SubState::TURN: {
            if (kin.moving_state == MovingState::KIN_REACHED) { // Espera a que termine el giro
                theta_accum += DELTA_THETA;
                substate = SubState::CHECK_CLEAR;
            }
            break;
        }

        case SubState::CHECK_CLEAR: {
            // Revisar si hay suficiente espacio para avanzar
            
            bool path_clear = (
                dis.left_dist >= EVATION_MIN_SPACE &&
                dis.mid_dist  >= EVATION_MIN_SPACE &&
                dis.right_dist >= EVATION_MIN_SPACE
            );
            if (path_clear) {
                // Fijar un objetivo de avance diferencial
                float dx = EVASION_ADVANCE_DIST * std::cos(kin.theta_d);
                float dy = EVASION_ADVANCE_DIST * std::sin(kin.theta_d);
                kin.x_d = kin.x + dx;
                kin.y_d = kin.y + dy;
                // Cambia modo a MOVE
                PositionController::set_control_mode(
                    (os.control_type == ControlType::PID) ? PositionControlMode::MOVE_PID : PositionControlMode::MOVE_BACKS,
                    sts.position, whl.w_L_ref, whl.w_R_ref, kin.moving_state
                );
                substate = SubState::ADVANCE;
            } else {
                // Intentar otro giro
                if (theta_accum + DELTA_THETA > MAX_EVASION_ANGLE) {
                    if (!reverse_attempted) {
                        // Volver al origen y probar el lado contrario
                        direction *= -1;
                        theta_accum = 0.0f;
                        kin.theta_d = PositionController::wrap_to_pi(origin_theta + direction * DELTA_THETA);
                        PositionController::set_control_mode(
                            (os.control_type == ControlType::PID) ? PositionControlMode::TURN_PID : PositionControlMode::TURN_BACKS,
                            sts.position, whl.w_L_ref, whl.w_R_ref, kin.moving_state
                        );
                        reverse_attempted = true;
                        substate = SubState::TURN_REVERSE;
                    } else {
                        // Fracaso: no hay ruta libre en ningún sentido
                        substate = SubState::FAIL;
                    }
                } else {
                    // Gira otro delta_theta en la misma dirección
                    kin.theta_d = PositionController::wrap_to_pi(kin.theta_d + direction * DELTA_THETA);
                    PositionController::set_control_mode(
                        (os.control_type == ControlType::PID) ? PositionControlMode::TURN_PID : PositionControlMode::TURN_BACKS,
                        sts.position, whl.w_L_ref, whl.w_R_ref, kin.moving_state
                    );
                    substate = SubState::TURN;
                }
            }
            break;
        }

        case SubState::TURN_REVERSE: {
            if (kin.moving_state == MovingState::KIN_REACHED) {
                theta_accum += DELTA_THETA;
                substate = SubState::CHECK_CLEAR;
            }
            break;
        }

        case SubState::ADVANCE: {
            // Esperar hasta avanzar la distancia indicada
            if (kin.moving_state == MovingState::KIN_REACHED) {
                substate = SubState::SUCCESS;
            }
            break;
        }

        case SubState::SUCCESS: {
            // Reset de variables internas
            substate = SubState::SELECT_DIRECTION;
            direction = 0;
            theta_accum = 0.0f;
            reverse_attempted = false;
            return true; // Evasión exitosa
        }

        case SubState::FAIL: {
            substate = SubState::SELECT_DIRECTION;
            direction = 0;
            theta_accum = 0.0f;
            reverse_attempted = false;
            return false; // No se pudo evadir
        }
    }
    return false; // Sigue en evasión
}

// Extra: función para fijar theta_d o x_d, y_d en forma diferencial
float set_theta_d_relative(KinematicState& kin, float delta_theta_rad) {
    kin.theta_d = PositionController::wrap_to_pi(kin.theta + delta_theta_rad);
    return kin.theta_d;
}

void set_xy_d_relative(KinematicState& kin, float delta_x, float delta_y) {
    kin.x_d = kin.x + delta_x;
    kin.y_d = kin.y + delta_y;
}


} // namespace EvadeController
