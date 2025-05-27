#include "vehicle_os.h"

namespace OS {

    OperationData os_data = {
        OS_State::INIT,  // Estado inicial
        0,               // Número de puntos en la trayectoria
        {},              // Trajectory vacía
        RemoteCommand::NONE, // Comando remoto inicial
        false,               // Bandera de objetivo alcanzado
        ControlType::PID     // Tipo de control inicial
    };

    void update(GlobalContext* ctx_ptr) {
        // Acceso a datos globales
        auto& sts = *ctx_ptr->systems_ptr;
        auto& kin = *ctx_ptr->kinematic_ptr;
        auto& dis = *ctx_ptr->distance_ptr;
        auto& whl = *ctx_ptr->wheels_ptr;
        auto& os  = *ctx_ptr->os_ptr;

        switch (os.state) {
            case OS_State::INIT:
                enter_init(ctx_ptr);
                os.state = OS_State::IDLE;
                clear_trajectory_with_null(os);
                enter_idle(ctx_ptr);
                break;

            case OS_State::IDLE:
                // Por ahora no se hace nada en IDLE, pero luego se hará lectura de Firebase
                enter_stand_by(ctx_ptr);
                os.state = OS_State::STAND_BY;
                break;

            case OS_State::STAND_BY:

                os.last_command = RemoteCommand::START; // Por ahora se pasa siempre a START

                if (os.last_command == RemoteCommand::START) {

                }
                break;

            case OS_State::MOVE:
                if (kin.target_reached) {
                    // Si se alcanza el objetivo y las ruedas están detenidas, se completa el waypoint
                    complete_current_waypoint(os);
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
                        kin.v, kin.w, whl.w_L_ref, whl.w_R_ref, sts.position);
                    if (stop_flag) {
                        enter_stand_by(ctx_ptr);
                        os.state = OS_State::STAND_BY;
                    }
                }
                else if (dis.obstacle_detected) {
                    const bool stop_flag = PositionController::stop_movement(
                        kin.v, kin.w, whl.w_L_ref, whl.w_R_ref, sts.position);
                    if (stop_flag) {
                        enter_evade(ctx_ptr);
                        os.state = OS_State::EVADE;
                    }
                }
                break;

            case OS_State::EVADE:
                if (os.last_command == RemoteCommand::STOP) {
                    // Si se recibe el comando STOP, se detiene el movimiento
                    const bool stop_flag = PositionController::stop_movement(
                        kin.v, kin.w, whl.w_L_ref, whl.w_R_ref, sts.position);
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
                    // Aquí se puede implementar la lógica de evasión, por ahora solo se revisa el obstáculo
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
            kin.target_reached = false; // Reiniciar flag de objetivo alcanzado
            return SUCCESS;
        }
        return ERROR; // No hay puntos en la trayectoria 
    }


    bool enter_init(GlobalContext* ctx_ptr) {
        auto& sts = *ctx_ptr->systems_ptr;
        auto& kin = *ctx_ptr->kinematic_ptr;
        auto& whl = *ctx_ptr->wheels_ptr;
        auto& dis = *ctx_ptr->distance_ptr;

        // Inicialización de módulos individuales
        EncoderReader::init(whl.steps_L, whl.steps_R, whl.w_L, whl.w_R, sts.encoders);
        PoseEstimator::init(kin.x, kin.y, kin.theta, kin.v, kin.w, whl.steps_L, whl.steps_R, sts.pose); 
        MotorController::init(sts.motors, whl.duty_L, whl.duty_R);
        DistanceSensors::init(dis.left_dist, dis.left_obst, dis.mid_dist, dis.mid_obst, 
            dis.right_dist, dis.right_obst, dis.obstacle_detected, sts.distance);
        // IMUReader::init(imu_data, sts.imu); // A futuro
        PositionController::init(sts.position, whl.w_L_ref, whl.w_R_ref);

        return SUCCESS; // Estado INIT alcanzado
    }


    bool enter_idle(GlobalContext* ctx) {
        auto& sts = *ctx->systems_ptr;
        auto& kin = *ctx->kinematic_ptr;
        auto& whl = *ctx->wheels_ptr;
        auto& dis = *ctx->distance_ptr;

        // 🛑 Dejar motores inactivos
        MotorController::set_motors_mode(MotorMode::IDLE, sts.motors, whl.duty_L, whl.duty_R);

        // Pausar y resetear sensores de distancia
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

        // 🧱 Frenar vehículo
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, whl.duty_L, whl.duty_R);

        // 🚫 Desactivar sensores de obstáculos
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
