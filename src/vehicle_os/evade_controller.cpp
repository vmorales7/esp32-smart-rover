#include "vehicle_os/evade_controller.h"

// --------- Helpers internos ---------
bool has_free_space(GlobalContext* ctx_ptr) {
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    // Requiere update de flags antes de leer
    DistanceSensors::force_measure_distances(ctx_ptr);
    return (sens.us_left_dist > EVADE_MIN_SPACE &&
            sens.us_mid_dist > EVADE_MIN_SPACE &&
            sens.us_right_dist > EVADE_MIN_SPACE);
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
    evade.saved_waypoint.ts = NULL_TIMESTAMP;
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
}

void update_evade(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile EvadeContext& evade = *(ctx_ptr->evade_ptr);
    //Serial.printf("Estado de evasion: %d\n", static_cast<int>(evade.state));
    //Serial.printf("Pose punto actual: (%f, %f)\n", pose.x, pose.y);
    // Serial.printf("Pose objetivo: (%f, %f)\n", ctrl.x_d, ctrl.y_d);
    switch (evade.state) {

        case EvadeState::SELECT_DIR: {    
            evade.tried_both_sides = false;
            DistanceSensors::force_measure_distances(ctx_ptr);
            if (evade.direction == 0) {
                evade.direction = (sens.us_left_dist >= sens.us_right_dist) ? +1 : -1; // +1: izquierda, -1: derecha
            } else {
                 // Mantener ultima dirección para evitar regresar al punto inicial en caso de obstáculos grandes
                evade.direction = evade.direction;
            }
            evade.current_angle = evade.direction * EVADE_DELTA_THETA;
            ctrl.theta_d= pose.theta;
            ctrl.x_d= pose.x;
            ctrl.y_d- pose.y;
            PositionController::set_diferential_waypoint(0.0f, evade.current_angle, ctrl.x_d, ctrl.y_d, ctrl.theta_d, 
                ctrl.waypoint_reached, sts.position);
            OS::enter_rotate(ctx_ptr);
            evade.state = EvadeState::WAIT_ALIGN;
            break;
        }
        case EvadeState::WAIT_ALIGN: {
            if (ctrl.waypoint_reached) {
                PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                const bool free = has_free_space(ctx_ptr);
                if (free) {
                    ctrl.theta_d= pose.theta;
                    ctrl.x_d= pose.x;
                    ctrl.y_d= pose.y;
                    PositionController::set_diferential_waypoint(EVADE_ADVANCE_DIST, 0.0f, 
                        ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                    // Serial.printf("Punto objetivo: (%f,%f,%f)\n",ctrl.x_d,ctrl.y_d,ctrl.theta_d);
                    OS::enter_move(ctx_ptr);
                    evade.state = EvadeState::WAIT_ADVANCE;
                } else {
                    const float next_angle = evade.current_angle + evade.direction * EVADE_DELTA_THETA;
                    if (fabsf(next_angle) > MAX_EVADE_ANGLE) {
                        if (!evade.tried_both_sides) { // Podemos intentar el otro lado
                            evade.direction = -evade.direction;
                            evade.tried_both_sides = true;
                            PositionController::set_diferential_waypoint(
                                0.0f, -1*evade.current_angle + evade.direction * EVADE_DELTA_THETA, 
                                ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                            evade.current_angle = evade.direction * EVADE_DELTA_THETA;
                        } else {
                            OS::enter_evade(ctx_ptr);
                            PositionController::set_waypoint(evade.saved_waypoint.x, evade.saved_waypoint.y, 0.0f, 
                                ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                            evade.state = EvadeState::FAIL;
                        }
                    } else {
                        evade.current_angle = next_angle;
                        PositionController::set_diferential_waypoint(
                            0.0f, evade.direction * EVADE_DELTA_THETA, 
                            ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                        OS::enter_rotate(ctx_ptr);
                    }
                }
            }
            break;
        }
        case EvadeState::WAIT_ADVANCE: {
            if (ctrl.waypoint_reached) {
                // Avance exitoso, retomar el waypoint, dejar el vehículo detenido, y volver a máquina principal
                OS::enter_evade(ctx_ptr);
                
                PositionController::set_waypoint(evade.saved_waypoint.x, evade.saved_waypoint.y, 0.0f, 
                    ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                evade.state = EvadeState::FINISHED;
            } 
            else if (sens.us_obstacle) {
                // Obstáculo temporal durante avance: detener hasta liberar
                OS::enter_wait_free_path(ctx_ptr);
                evade.state = EvadeState::WAIT_FREE_PATH;
            }
            break;
        }
        case EvadeState::WAIT_FREE_PATH: {
            DistanceSensors::update_global_obstacle_flag(
                sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
            if (!sens.us_obstacle) {
                OS::enter_move(ctx_ptr);
                evade.state = EvadeState::WAIT_ADVANCE;
            }
            break;
        }
    }
}

} // namespace EvadeController
