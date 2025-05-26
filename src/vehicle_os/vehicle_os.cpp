#include "vehicle_os.h"

namespace OS {

    static OperationData os_data = {
        OS_State::INIT,  // Estado inicial
        0,             // Número de puntos en la trayectoria
        {},              // Trajectory vacía
        RemoteCommand::NONE, // Comando remoto inicial
        false,         // Bandera de objetivo alcanzado
        ControlType::PID // Tipo de control inicial
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
                init_trajectory_with_null(os);
                enter_idle(ctx_ptr);
                break;

            case OS_State::IDLE:
                break;

            case OS_State::STAND_BY:
            if (os.last_command == RemoteCommand::START) {

            }
                break;

            case OS_State::MOVE:
                if (kin.target_reached) {
                    os.state = OS_State::STAND_BY;
                    enter_stand_by(ctx_ptr);
                    complete_current_waypoint(os);
                }
                if (dis.obstacle_detected) {
                    os.state = OS_State::EVADE;
                    enter_evade(ctx_ptr);
                }
                break;

            case OS_State::EVADE:
                // La flag de obstáculo se mantiene en true hasta que se resuelva la evasión
                if (!dis.obstacle_detected) {
                    os.state = OS_State::MOVE;
                    enter_move(ctx_ptr);
                } else {
                    // Aquí se puede implementar la lógica de evasión, por ahora solo se revisa el obstáculo
                    // evation_routine(ctx_ptr);
                    DistanceSensors::update_global_obstacle_flag(
                        dis.left_obst, dis.mid_obst, dis.right_obst, dis.obstacle_detected);
                }
                break;
        }
    }


    void set_waypoint(GlobalContext* ctx_ptr) {
        auto& os = *ctx_ptr->os_ptr;
        auto& kin = *ctx_ptr->kinematic_ptr;

        // Fijar el punto objetivo a partir del primer punto en la trayectoria
        if (os.total_targets > 0) {
            kin.x_d = os.trajectory[0].x;
            kin.y_d = os.trajectory[0].y;
        }
    }

    void enter_init(GlobalContext* ctx_ptr) {
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
    }


    void enter_idle(GlobalContext* ctx) {
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
    }


    void enter_stand_by(GlobalContext* ctx) {
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

        // 🚫 Desactivar sensores de obstáculos (por completitud)
        DistanceSensors::set_state(INACTIVE, sts.distance, dis.left_dist, dis.left_obst, 
            dis.mid_dist, dis.mid_obst, dis.right_dist, dis.right_obst, dis.obstacle_detected);
    }


    void enter_move(GlobalContext* ctx) {
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
        if (os.total_targets > 0) {
            kin.x_d = os.trajectory[0].x;
            kin.y_d = os.trajectory[0].y;
            kin.theta_d = 0.0; // No relevante en este caso
        }
        // 🟢 Activar control de posición (v_ref y w_ref)
        if (os.control_type == ControlType::PID) {
            PositionController::set_control_mode(PositionControlMode::MOVE_PID, sts.position, whl.w_L_ref, whl.w_R_ref);
        } else {
            PositionController::set_control_mode(PositionControlMode::MOVE_BACKS, sts.position, whl.w_L_ref, whl.w_R_ref);
        }
    }


    void enter_evade(GlobalContext* ctx) {
        auto& sts = *ctx->systems_ptr;
        auto& kin = *ctx->kinematic_ptr;
        auto& whl = *ctx->wheels_ptr;
        auto& dis = *ctx->distance_ptr;

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
    }


    void init_trajectory_with_null(volatile OperationData& os) {
        for (uint8_t i = 0; i < MAX_TRAJECTORY_POINTS; ++i) {
            os.trajectory[i].x = NULL_WAYPOINT_XY;
            os.trajectory[i].y = NULL_WAYPOINT_XY;
        }
        os.total_targets = 0;
    }


    void complete_current_waypoint(volatile OperationData& os) {
        for (uint8_t i = 1; i < MAX_TRAJECTORY_POINTS; ++i) {
            os.trajectory[i-1].x = os.trajectory[i].x;
            os.trajectory[i-1].y = os.trajectory[i].y;
        }
        os.trajectory[MAX_TRAJECTORY_POINTS-1].x = NULL_WAYPOINT_XY;
        os.trajectory[MAX_TRAJECTORY_POINTS-1].y = NULL_WAYPOINT_XY;
        if (os.total_targets > 0) os.total_targets--;
    }


    void add_waypoint(volatile OperationData& os, float x, float y) {
        if (os.total_targets < MAX_TRAJECTORY_POINTS) {
            os.trajectory[os.total_targets].x = x;
            os.trajectory[os.total_targets].y = y;
            os.total_targets++;
        }
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
