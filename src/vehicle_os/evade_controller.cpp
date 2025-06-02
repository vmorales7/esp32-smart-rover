#include "vehicle_os/evade_controller.h"

// --------- Helpers internos ---------
bool has_free_space(GlobalContext* ctx_ptr) {
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    // Requiere update de flags antes de leer
    DistanceSensors::force_check_sensors(ctx_ptr);
    return (sens.us_left_dist >= EVADE_MIN_SPACE &&
            sens.us_mid_dist >= EVADE_MIN_SPACE &&
            sens.us_right_dist >= EVADE_MIN_SPACE);
}

// --------- Máquina de estados de evasión principal ---------

namespace EvadeController {

void reset_evade_state(GlobalContext* ctx_ptr) {
    volatile EvadeContext& evade = *(ctx_ptr->evade_ptr);
    evade.state = EvadeState::IDLE;
    evade.direction = 0;
    evade.current_angle = 0.0f;
    evade.tried_both_sides = false;
    evade.saved_waypoint.x = NULL_WAYPOINT_XY;
    evade.saved_waypoint.y = NULL_WAYPOINT_XY;
}


void start_evade(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile EvadeContext& evade = *(ctx_ptr->evade_ptr);

    evade.saved_waypoint.x = ctrl.x_d;
    evade.saved_waypoint.y = ctrl.y_d;

    evade.state = EvadeState::SELECT_DIR;

    DistanceSensors::set_state(INACTIVE, sts.distance, 
        sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
}

void update_evade(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile EvadeContext& evade = *(ctx_ptr->evade_ptr);

    switch (evade.state) {
        case EvadeState::SELECT_DIR: {    
            evade.tried_both_sides = false;
            DistanceSensors::force_check_sensors(ctx_ptr);
            if (evade.direction == 0) {
                evade.direction = (sens.us_left_dist >= sens.us_right_dist) ? +1 : -1; // +1: izquierda, -1: derecha
            } else {
                 // Mantener ultima dirección para evitar regresar al punto inicial en caso de obstáculos grandes
                evade.direction = evade.direction;
            }
            evade.current_angle = evade.direction * EVADE_DELTA_THETA;
            PositionController::set_diferential_waypoint(0.0f, evade.direction * EVADE_DELTA_THETA, 
                ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
            PositionController::set_control_mode(PositionControlMode::ROTATE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
            evade.state = EvadeState::WAIT_ALIGN;
            break;
        }
        case EvadeState::WAIT_ALIGN: {
            if (ctrl.waypoint_reached) {
                const bool free = has_free_space(ctx_ptr);
                if (free) {
                    PositionController::set_diferential_waypoint(EVADE_ADVANCE_DIST, 0.0f, 
                        ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                    OS::enter_move(ctx_ptr);
                    evade.state = EvadeState::WAIT_ADVANCE;
                } else {
                    const float next_angle = evade.current_angle + evade.direction * EVADE_DELTA_THETA;
                    if (next_angle > MAX_EVADE_ANGLE) {
                        if (!evade.tried_both_sides) { // Podemos intentar el otro lado
                            evade.direction = -evade.direction;
                            evade.tried_both_sides = true;
                            PositionController::set_diferential_waypoint(
                                0.0f, -1*evade.current_angle + evade.direction * EVADE_DELTA_THETA, 
                                ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                            evade.current_angle = evade.direction * EVADE_DELTA_THETA;
                        } else {
                            evade.state = EvadeState::FAIL;
                        }
                    } else {
                        evade.current_angle = next_angle;
                        PositionController::set_diferential_waypoint(
                            0.0f, evade.direction * EVADE_DELTA_THETA, 
                            ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                    }
                }
            }
            break;
        }
        case EvadeState::WAIT_ADVANCE: {
            if (ctrl.waypoint_reached) {
                PositionController::set_waypoint(evade.saved_waypoint.x, evade.saved_waypoint.y, 0.0f, 
                    ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, 
                    ctrl.w_L_ref, ctrl.w_R_ref);
                PositionController::set_wheel_speed_ref(0.0f, 0.0f, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                evade.state = EvadeState::FINISHED;
            } 
            else if (sens.us_obstacle) {
                // Obstáculo temporal durante avance: detener hasta liberar
                PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, 
                    ctrl.w_L_ref, ctrl.w_R_ref);
                PositionController::set_wheel_speed_ref(0.0f, 0.0f, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                evade.state = EvadeState::WAIT_FREE_PATH;
            }
            break;
        }
        case EvadeState::WAIT_FREE_PATH: {
            DistanceSensors::update_global_obstacle_flag(
                sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
            if (!sens.us_obstacle) {
                PositionController::set_control_mode(PositionControlMode::MOVE, sts.position, 
                    ctrl.w_L_ref, ctrl.w_R_ref);
                evade.state = EvadeState::WAIT_ADVANCE;
            }
            break;
        }
        default:
            // No hacer nada, espera a que el OS principal saque del estado EVADE
            break;
    }
}

} // namespace EvadeController
