#include "vehicle_os.h"

namespace OS {

    void update(GlobalContext* ctx_ptr) {
        // Acceso a datos globales
        auto& sts = *ctx_ptr->systems_ptr;
        auto& kin = *ctx_ptr->kinematic_ptr;
        auto& dis = *ctx_ptr->distance_ptr;
        auto& whl = *ctx_ptr->wheels_ptr;
        auto& os  = *ctx_ptr->os_ptr;

        switch (os.state) {
            case OS_State::INIT:
                os.state = OS_State::IDLE;
                enter_idle(ctx_ptr);
                break;

            case OS_State::IDLE:
                // Por ahora no se hace nada en IDLE, pero luego se hará lectura de Firebase
                enter_stand_by(ctx_ptr);
                os.state = OS_State::STAND_BY;
                break;

            case OS_State::STAND_BY:
                os.last_command = RemoteCommand::START; // Por ahora se asume modo START (ya que no hay Firebase)
                
                const bool pending_targets = (os.total_targets > 0);
                if (os.last_command == RemoteCommand::START) {
                    if (pending_targets) {
                        // Si hay puntos pendientes, se entra al estado MOVE
                        // Acá habría que actualizar el Firebase con el nuevo waypoint actual
                        enter_move(ctx_ptr); // Selecciona nuevo waypoint y limpia flag de objetivo alcanzado
                        os.state = OS_State::MOVE;
                        // Primera revisión de presencia de obstáculos (evita que comience a moverse con un obstáculo)
                        DistanceSensors::check_all_sensors_obstacle(
                            dis.left_dist, dis.left_obst, dis.mid_dist, dis.mid_obst,
                            dis.right_dist, dis.right_obst, dis.obstacle_detected, sts.distance
                        );
                    }
                }
                else if (os.last_command == RemoteCommand::IDLE) {
                    if (!pending_targets) {
                        // Si no hay puntos pendientes, se vuelve a IDLE
                        enter_idle(ctx_ptr);
                        os.state = OS_State::IDLE;
                    }
                }
                break;

            case OS_State::MOVE:
                if (kin.moving_state == MovingState::KIN_REACHED) {
                    // Si se alcanza el objetivo y las ruedas están detenidas, se completa el waypoint
                    bool ok = complete_current_waypoint(os);
                    if (!ok) {
                        // Si no se pudo completar el waypoint (# targets = 0), se manda todo a stand-by
                        enter_stand_by(ctx_ptr);
                        os.state = OS_State::STAND_BY;
                        break;
                    }
                    // Futuro: avisar a FB que se alcanzó el waypoint

                    if (os.total_targets > 0 && os.last_command == RemoteCommand::START) { 
                        // Si hay más puntos, se establece el siguiente waypoint solo si se está en START
                        set_waypoint(ctx_ptr); 
                        // Futuro: avisar a FB que se estableció el nuevo waypoint
                    } 
                    else { // Si no hay más puntos o se dio el stop, se vuelve a STAND_BY
                        os.state = OS_State::STAND_BY;
                        enter_stand_by(ctx_ptr);
                    }
                }
                else if (os.last_command == RemoteCommand::STOP) {
                    // Si se recibe el comando STOP, se detiene el movimiento
                    const bool stop_flag = PositionController::stop_movement(
                        kin.v, kin.w, whl.w_L_ref, whl.w_R_ref, kin.moving_state, sts.position);
                    if (stop_flag) {
                        enter_stand_by(ctx_ptr);
                        os.state = OS_State::STAND_BY;
                    }
                }
                else if (dis.obstacle_detected) {
                    // Se pasa a EVADE solo en caso de detección de obstáculo cuando el vehículo está en movimiento
                    if (kin.moving_state != MovingState::KIN_MOVING){
                        const bool stop_flag = PositionController::stop_movement(
                            kin.v, kin.w, whl.w_L_ref, whl.w_R_ref, kin.moving_state, sts.position);
                        if (stop_flag) {
                            enter_evade(ctx_ptr);
                            os.state = OS_State::EVADE;
                        }
                    }
                    else{ // En otros casos solo se limpia la bandera global de obstáculos
                        DistanceSensors::update_global_obstacle_flag(
                            dis.left_obst, dis.mid_obst, dis.right_obst, dis.obstacle_detected);
                    }
                }
                break;

            case OS_State::EVADE:
                if (os.last_command == RemoteCommand::STOP) {
                    // Si se recibe el comando STOP, se detiene el movimiento
                    const bool stop_flag = PositionController::stop_movement(
                        kin.v, kin.w, whl.w_L_ref, whl.w_R_ref, kin.moving_state, sts.position);
                    if (stop_flag) {
                        enter_stand_by(ctx_ptr);
                        os.state = OS_State::STAND_BY;
                    }
                }
                else if (!dis.obstacle_detected) { // La flag de obstáculo se mantiene en true hasta que se resuelva la evasión
                    os.state = OS_State::MOVE;
                    enter_move(ctx_ptr); // Internamente llama a set_waypoint
                } 
                else {
                    // Aquí se puede implementar la lógica de evasión
                    // Por ahora solo se actualiza la flag de obstáculos hasta que se libera
                    // evation_routine(ctx_ptr);
                    DistanceSensors::update_global_obstacle_flag(
                        dis.left_obst, dis.mid_obst, dis.right_obst, dis.obstacle_detected);
                }
                break;
        }
    }


    bool set_waypoint(GlobalContext* ctx_ptr) {
        auto& os = *ctx_ptr->os_ptr;
        auto& kin = *ctx_ptr->kinematic_ptr;

        // Fijar el punto objetivo a partir del primer punto en la trayectoria
        if (os.total_targets > 0) {
            kin.x_d = os.trajectory[0].x;
            kin.y_d = os.trajectory[0].y;
            // Actualizar estado de movimiento
            PositionController::set_moving_state(MovingState::KIN_MOVING, kin.moving_state); 
            kin.moving_state = MovingState::KIN_STOPPING; // Reiniciar flag de objetivo alcanzado
            return SUCCESS;
        }
        return ERROR; // No hay puntos en la trayectoria 
    }


    bool enter_init(GlobalContext* ctx_ptr) {
        auto& sts = *ctx_ptr->systems_ptr;
        auto& kin = *ctx_ptr->kinematic_ptr;
        auto& whl = *ctx_ptr->wheels_ptr;
        auto& dis = *ctx_ptr->distance_ptr;

        // 1. Inicialización de módulos individuales
        EncoderReader::init(whl.steps_L, whl.steps_R, whl.w_L, whl.w_R, sts.encoders);
        PoseEstimator::init(kin.x, kin.y, kin.theta, kin.v, kin.w, whl.steps_L, whl.steps_R, sts.pose); 
        MotorController::init(sts.motors, whl.duty_L, whl.duty_R);
        DistanceSensors::init(dis.left_dist, dis.left_obst, dis.mid_dist, dis.mid_obst, 
            dis.right_dist, dis.right_obst, dis.obstacle_detected, sts.distance);
        // IMUReader::init(imu_data, sts.imu); // A futuro
        PositionController::init(sts.position, whl.w_L_ref, whl.w_R_ref);

        // 2. Lanzar tareas RTOS núcleo 1
        xTaskCreatePinnedToCore(
            EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2*BASIC_STACK_SIZE, ctx_ptr, 3, nullptr, 1);
        // xTaskCreatePinnedToCore(
        //     PoseEstimator::Task_ImuUpdate, "UpdateIMU", 2*BASIC_STACK_SIZE, ctx_ptr, 3, nullptr, 1);
        xTaskCreatePinnedToCore(
            PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 4*BASIC_STACK_SIZE, ctx_ptr, 2, nullptr, 1);
        xTaskCreatePinnedToCore(
            MotorController::Task_WheelControl, "WheelControl", 2*BASIC_STACK_SIZE, ctx_ptr, 2, nullptr, 1);
        xTaskCreatePinnedToCore(
            PositionController::Task_PositionControl, "PositionControl", 3*BASIC_STACK_SIZE, ctx_ptr, 1, nullptr, 1);


        // 3. Lanzar tareas RTOS núcleo 0
        xTaskCreatePinnedToCore(
            OS::Task_VehicleOS, "VehicleOS", 2*BASIC_STACK_SIZE, ctx_ptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(
            DistanceSensors::Task_CheckLeftObstacle, "USLeft", BASIC_STACK_SIZE, ctx_ptr, 2, nullptr, 0);
        xTaskCreatePinnedToCore(
            DistanceSensors::Task_CheckMidObstacle,  "USMid",  BASIC_STACK_SIZE, ctx_ptr, 2, nullptr, 0);
        xTaskCreatePinnedToCore(
            DistanceSensors::Task_CheckRightObstacle, "USRight", BASIC_STACK_SIZE, ctx_ptr, 2, nullptr, 0);

        return SUCCESS; // Estado INIT alcanzado
    }


    bool enter_idle(GlobalContext* ctx) {
        auto& sts = *ctx->systems_ptr;
        auto& kin = *ctx->kinematic_ptr;
        auto& whl = *ctx->wheels_ptr;
        auto& dis = *ctx->distance_ptr;

        // 🛑 Dejar motores inactivos
        MotorController::set_motors_mode(MotorMode::IDLE, sts.motors, whl.duty_L, whl.duty_R);

        // 🚫 Desactivar sensores de obstáculos -> se fuerza la limpieza de las flag de obstáculo
        DistanceSensors::set_state(INACTIVE, sts.distance, dis.left_dist, dis.left_obst, 
            dis.mid_dist, dis.mid_obst, dis.right_dist, dis.right_obst, dis.obstacle_detected);
 
        // Detener control de posición
        PositionController::set_control_mode(PositionControlMode::INACTIVE, sts.position, whl.w_L_ref, whl.w_R_ref);

        // ⏸️ Pausar encoders (congelar pasos y velocidades)
        EncoderReader::pause(whl.steps_L, whl.steps_R, whl.w_L, whl.w_R, sts.encoders);

        // 🧭 Pausar IMU (a futuro)
        // IMUReader::pause(sts.imu); // ← implementar luego

        // 🔻 Desactivar estimador de pose y resetear posición y orientación a cero
        PoseEstimator::set_state(INACTIVE, sts.pose);
        PoseEstimator::reset_pose(kin.x, kin.y, kin.theta, kin.v, kin.w, whl.steps_L, whl.steps_R);

        return SUCCESS; // Estado IDLE alcanzado
    }


    bool enter_stand_by(GlobalContext* ctx) {
        auto& sts = *ctx->systems_ptr;
        auto& kin = *ctx->kinematic_ptr;
        auto& whl = *ctx->wheels_ptr;
        auto& dis = *ctx->distance_ptr;

        // 🧭 Activar lectura de encoders
        EncoderReader::resume(sts.encoders);

        // 🧭 Activar IMU (a futuro)
        // IMUReader::resume(...); // ← implementar luego

        // 🧠 Activar estimador de pose (para no perder seguimiento del vehículo)
        PoseEstimator::set_state(ACTIVE, sts.pose);

        // 🧷 Mantener control de posición en modo pasivo, con velocidad de referencia 0
        PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, whl.w_L_ref, whl.w_R_ref);
        PositionController::set_wheel_speed_ref(0.0f, 0.0f, whl.w_L_ref, whl.w_R_ref, sts.position);
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, whl.duty_L, whl.duty_R);

        // 🚫 Desactivar sensores de obstáculos -> se fuerza la limpieza de las flag de obstáculo
        DistanceSensors::set_state(INACTIVE, sts.distance, dis.left_dist, dis.left_obst, 
            dis.mid_dist, dis.mid_obst, dis.right_dist, dis.right_obst, dis.obstacle_detected);
        
        return SUCCESS; 
    }


    bool enter_move(GlobalContext* ctx) {
        auto& sts = *ctx->systems_ptr;
        auto& kin = *ctx->kinematic_ptr;
        auto& whl = *ctx->wheels_ptr;
        auto& dis = *ctx->distance_ptr;
        auto& os  = *ctx->os_ptr;

        // 🟢 Activar motores en modo automático
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, whl.duty_L, whl.duty_R);

        // 🟢 Reanudar sensores y estimadores
        EncoderReader::resume(sts.encoders);
        // IMUReader::resume(...);
        PoseEstimator::set_state(ACTIVE, sts.pose);
        DistanceSensors::set_state(ACTIVE, sts.distance, dis.left_dist, dis.left_obst, 
            dis.mid_dist, dis.mid_obst, dis.right_dist, dis.right_obst, dis.obstacle_detected);

        // ✅ Fijar el punto objetivo a partir del primer punto en la trayectoria
        const bool ok = set_waypoint(ctx);

        // 🟢 Activar control de posición (v_ref y w_ref)
        if (os.control_type == ControlType::PID) {
            PositionController::set_control_mode(PositionControlMode::MOVE_PID, sts.position, whl.w_L_ref, whl.w_R_ref);
        } else {
            PositionController::set_control_mode(PositionControlMode::MOVE_BACKS, sts.position, whl.w_L_ref, whl.w_R_ref);
        }
        return ok;
    }


    bool enter_evade(GlobalContext* ctx) {
        auto& sts = *ctx->systems_ptr;
        auto& kin = *ctx->kinematic_ptr;
        auto& whl = *ctx->wheels_ptr;
        auto& dis = *ctx->distance_ptr;

        // En teoría bastaría con dejar la referencia de velocidad en cero, 
        // pero por robustez nos aseguramos de tener todos los modulos activos y luego dar la detención
        // De todas formas, ninguno debería ejecutarse, ya que están hechos para que no se ejecuten si se entrega el mismo estado en que ya están

        // 🟡 Activar motores en modo automático
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, whl.duty_L, whl.duty_R);

        // 🟡 Reanudar sensores y estimadores
        EncoderReader::resume(sts.encoders);
        // IMUReader::resume(...);
        PoseEstimator::set_state(ACTIVE, sts.pose);
        DistanceSensors::set_state(ACTIVE, sts.distance, dis.left_dist, dis.left_obst, 
            dis.mid_dist, dis.mid_obst, dis.right_dist, dis.right_obst, dis.obstacle_detected);

        // Se fija la velocidad de referencia a cero
        PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, whl.w_L_ref, whl.w_R_ref);
        PositionController::set_wheel_speed_ref(0.0f, 0.0f, whl.w_L_ref, whl.w_R_ref, sts.position);

        return SUCCESS;
    }

    
    void clear_trajectory_with_null(volatile OperationData& os) {
        for (uint8_t i = 0; i < MAX_TRAJECTORY_POINTS; ++i) {
            os.trajectory[i].x = NULL_WAYPOINT_XY;
            os.trajectory[i].y = NULL_WAYPOINT_XY;
        }
        os.total_targets = 0U;
    }


    bool complete_current_waypoint(volatile OperationData& os) {
        if (os.total_targets > 0) {
            os.total_targets--;
            for (uint8_t i = 1; i < MAX_TRAJECTORY_POINTS; ++i) {
                os.trajectory[i-1].x = os.trajectory[i].x;
                os.trajectory[i-1].y = os.trajectory[i].y;
            }
            os.trajectory[MAX_TRAJECTORY_POINTS-1].x = NULL_WAYPOINT_XY;
            os.trajectory[MAX_TRAJECTORY_POINTS-1].y = NULL_WAYPOINT_XY;
            return SUCCESS; // Se completó el waypoint
        }
        return ERROR; // No hay waypoints para completar
    }


    bool add_waypoint(volatile OperationData& os, float x, float y) {
        if (os.total_targets < MAX_TRAJECTORY_POINTS) {
            os.trajectory[os.total_targets].x = x;
            os.trajectory[os.total_targets].y = y;
            os.total_targets++;
            return SUCCESS;
        }
        return ERROR; // No se pudo agregar el waypoint, lista llena
    }


    void Task_VehicleOS(void* pvParameters) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(OS_UPDATE_PERIOD_MS);
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            update(ctx_ptr);
        }
    }

}
