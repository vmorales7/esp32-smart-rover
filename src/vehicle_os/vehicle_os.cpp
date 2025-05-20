#include "vehicle_os.h"

namespace OS {

    static OS_State current_state = OS_State::INIT;

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
        DistanceSensors::init(dis.left_dist, dis.left_obst, dis.mid_dist, dis.mid_obst, 
            dis.right_dist, dis.right_obst, dis.obstacle_detected, sts.distance);
 
        // Detener control de posición
        PositionController::set_control_mode(PositionControlMode::INACTIVE, sts.position, whl.w_L_ref, whl.w_R_ref);

        // ⏸️ Pausar encoders (congelar pasos y velocidades)
        EncoderReader::pause(whl.steps_L, whl.steps_R, whl.w_L, whl.w_R, sts.encoders);

        // 🧭 Pausar IMU (a futuro)
        // IMUReader::pause(sys.imu); // ← implementar luego

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

        // 🚫 Desactivar sensores de obstáculos por completitud
        DistanceSensors::set_state(INACTIVE, sts.distance, dis.left_dist, dis.left_obst, 
            dis.mid_dist, dis.mid_obst, dis.right_dist, dis.right_obst, dis.obstacle_detected);
    }


    void enter_move(GlobalContext* ctx) {
        auto& sts = *ctx->systems_ptr;
        auto& kin = *ctx->kinematic_ptr;
        auto& whl = *ctx->wheels_ptr;
        auto& dis = *ctx->distance_ptr;
        auto& os  = *ctx->os_ptr;

        // ✅ Fijar el punto objetivo a partir del primer punto en la trayectoria
        if (os.total_targets > 0) {
            kin.x_d = os.trajectory[0].x;
            kin.y_d = os.trajectory[0].y;
            // La orientación deseada puede ser calculada luego
        }

        // 🟢 Activar motores en modo automático
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, whl.duty_L, whl.duty_R);

        // 🟢 Reanudar sensores y estimadores
        EncoderReader::resume(sts.encoders);
        // IMUReader::resume(...);
        PoseEstimator::set_state(ACTIVE, sts.pose);
        //DistanceSensors::set_state(ACTIVE, sts.distance);
        
        // 🟢 Activar control de posición básico (v_ref y w_ref)
        PositionController::set_control_mode(PositionControlMode::MOVE_BASIC, sts.position, whl.w_L_ref, whl.w_R_ref);
    }


    void update(GlobalContext* ctx_ptr) {
        // Acceso a datos globales
        auto& sys = *ctx_ptr->systems_ptr;
        auto& kin = *ctx_ptr->kinematic_ptr;
        auto& dis = *ctx_ptr->distance_ptr;
        auto& whl = *ctx_ptr->wheels_ptr;

        switch (current_state) {
            case OS_State::INIT:
                enter_init(ctx_ptr);
                current_state = OS_State::IDLE;
                enter_idle(ctx_ptr);
                break;

            case OS_State::IDLE:
                break;

            case OS_State::STAND_BY:
                break;

            case OS_State::MOVE:
                if (dis.obstacle_detected) {
                    current_state = OS_State::EVADE;
                }
                break;

            case OS_State::EVADE:
                break;
        }
    }

    void Task_VehicleOS(void* pvParameters) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(100);
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            update(ctx_ptr);
        }
    }

}
