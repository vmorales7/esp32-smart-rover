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
    evade.state = EvadeState::SELECT_DIR;
    evade.direction = 0;
    evade.current_angle = 0.0f;
    evade.tried_both_sides = false;
    // No reiniciar puntos: no es necesario, y genera problemas si se reinicia durante una evasión 
    // Además siempre se inicia llamando start_evade, lo cual ya reinicia los waypoint 
    // evade.saved_waypoint.x = NULL_WAYPOINT_XY;
    // evade.saved_waypoint.y = NULL_WAYPOINT_XY;
    // evade.saved_waypoint.ts = NULL_TIMESTAMP;
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
    DistanceSensors::update_global_obstacle_flag(
        sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);

    switch (evade.state) 
    {
        case EvadeState::SELECT_DIR: 
        {   // Primera etapa: decidir en qué dirección comenzar la evasión -> se ejecuta solo una vez
            // Guardar waypoint objetivo actual
            evade.saved_waypoint.x = ctrl.x_d;
            evade.saved_waypoint.y = ctrl.y_d;

            // Inicializar estado de evasión
            evade.current_angle = 0.0f;
            // evade.direction = 0;         // No reiniciar para no regresar al punto inicial con obstáculos grandes
            evade.tried_both_sides = false; // Quizás este tampoco se debiera reiniciar

            // Primer giro hacia donde haya mas espacio libre o repetir última dirección
            if (evade.direction == 0) {
                DistanceSensors::force_measure_distances(ctx_ptr);
                evade.direction = (sens.us_left_dist >= sens.us_right_dist) ? +1 : -1; // +1: izquierda, -1: derecha
            }
            // Fijar el punto objetivo de giro respecto a la posición actual
            evade.current_angle = evade.direction * EVADE_DELTA_THETA;
            const float delta_theta = evade.current_angle;
            PositionController::set_diferential_waypoint(0.0f, delta_theta, pose.x, pose.y, pose.theta,
                ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);

            // Vamos al siguiente estado
            OS::enter_rotate(ctx_ptr);
            evade.state = EvadeState::WAIT_ALIGN;
            break;
        }
        case EvadeState::WAIT_ALIGN: 
        {   // Segunda etapa: alinear el vehículo en la dirección de evasión e intentar múltiples direcciones
            // Asgurarse del modo rotate por si se usó stop_movement externo (si ya era rotate, no hace nada)
            PositionController::set_control_mode(
                PositionControlMode::ROTATE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref); 
            if (ctrl.waypoint_reached) 
            {   // Alineación exitosa, ver si hay espacio libre para avanzar
                PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                const bool free = has_free_space(ctx_ptr);
                if (free) 
                {   // Si hay espacio libre, se fija el nuevo setpoint con una distancia respecto al punto actual
                    PositionController::set_diferential_waypoint(EVADE_ADVANCE_DIST, 0.0f, pose.x, pose.y, pose.theta,
                        ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                    // Activa los sensores de distancia para detectar obstáculos y liberar inicialmente la flag
                    OS::enter_move(ctx_ptr); 
                    evade.state = EvadeState::WAIT_ADVANCE;
                } 
                else 
                {   // No hay espacio libre, intentar girar más. 
                    const float next_angle = evade.current_angle + evade.direction * EVADE_DELTA_THETA;
                    if (fabsf(next_angle) > MAX_EVADE_ANGLE) 
                    {   // Es más del máximo ángulo que queremos usar, ya que no queremos retroceder
                        if (!evade.tried_both_sides) 
                        {   // No hemos verificado ambos lados, intentar el otro lado
                            evade.direction *= -1; 
                            evade.tried_both_sides = true;

                            // Fijar un giro diferencial para llegar a delta_theta en la dirección opuesta
                            const float delta_theta = evade.direction * EVADE_DELTA_THETA - evade.current_angle;
                            PositionController::set_diferential_waypoint(0.0f, delta_theta, pose.x, pose.y, pose.theta,
                                 ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                            
                            // Actualizar el estado de evasión
                            evade.current_angle = evade.direction * EVADE_DELTA_THETA;
                        } 
                        else 
                        {   // FAIL: ya verificamos ambos y no hubo espacio libre. 
                            // Devolver al waypoint original para terminar
                            OS::enter_evade(ctx_ptr); // Detiene y apaga sensores de distancia
                            reset_evade_state(ctx_ptr);
                            PositionController::set_waypoint(evade.saved_waypoint.x, evade.saved_waypoint.y, 0.0f, 
                                ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                            evade.state = EvadeState::FAIL;
                        }
                    } 
                    else 
                    {   // Aún podemos seguir girando en la misma dirección, fijar el nuevo waypoint de giro
                        evade.current_angle = next_angle;
                        const float delta_theta = evade.direction * EVADE_DELTA_THETA;
                        PositionController::set_diferential_waypoint(0.0f, delta_theta, pose.x, pose.y, pose.theta,
                            ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                    }
                }
            }
            break;
        }
        case EvadeState::WAIT_ADVANCE: 
        {   // Tercera etapa: avanzar en la dirección de evasión -> nos aseguramos de estar en modo MOVE
            PositionController::set_control_mode(
                PositionControlMode::MOVE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
            if (ctrl.waypoint_reached) 
            {   // Avance exitoso, retomar el waypoint, dejar el vehículo detenido, y volver a máquina principal
                OS::enter_evade(ctx_ptr); // Solo se usa para detener el vehículo y dejar en modo manual
                PositionController::set_waypoint(evade.saved_waypoint.x, evade.saved_waypoint.y, 0.0f, 
                    ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                evade.state = EvadeState::FINISHED;
            } 
            else if (sens.us_obstacle) 
            {   // Obstáculo temporal durante avance: detener hasta liberar
                OS::enter_wait_free_path(ctx_ptr);
                evade.state = EvadeState::WAIT_FREE_PATH;
            }
            break;
        }
        case EvadeState::WAIT_FREE_PATH: 
        {   // Actualizar la bandera de obstáculos globales hasta que se libere el camino
            PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
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
