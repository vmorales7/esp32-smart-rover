#include "vehicle_os.h"

namespace OS {

void update(GlobalContext* ctx_ptr) {
    // Acceso a datos globales
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile OperationData& os = *(ctx_ptr->os_ptr);

    bool ok = true; // Se usará para verificar si se pudo entrar a un estado
    os.last_command = RemoteCommand::START; // Por ahora se asume modo START (ya que no hay Firebase)

    switch (os.state) {
        case OS_State::INIT: {// Solo se ejecuta como paso hacia IDLE
            enter_idle(ctx_ptr);
            break;
        }
        case OS_State::IDLE: {
            // Por ahora no se hace nada en IDLE, pero luego se hará lectura de Firebase
            enter_stand_by(ctx_ptr);
            break;
        }
        case OS_State::STAND_BY: {            
            const bool pending_targets = (os.total_targets > 0);
            if (os.last_command == RemoteCommand::START) {    
                // Se establece el primer waypoint de la trayectoria y limpiar la flag de waypoint alcanzado
                ok = set_waypoint(ctx_ptr);
                if (ok) { // Si hay puntos pendientes, se entra al estado ALIGN
                    enter_align(ctx_ptr); // El controlador intentará alinear el vehículo hacia el objetivo
                    // Avisar a FB que se estableció el waypoint
                } else { // Si no se pudo, nos quedamos en STAND_BY
                    enter_stand_by(ctx_ptr);
                }
            } else if (os.last_command == RemoteCommand::IDLE) {                    
                enter_idle(ctx_ptr);
            }
            break;
        }
        case OS_State::ALIGN: { // Este estado se usa para alinear el vehículo hacia el objetivo
            if (ctrl.waypoint_reached) {
                // Si se alinea, podemos pasar al estado MOVE
                ok = set_waypoint(ctx_ptr); // Checkeo ante error y limpiar flag de waypoint alcanzado
                if (ok) { // Si se pudo entrar al estado MOVE, se actualiza el estado
                    enter_move(ctx_ptr);
                } else {// Si no se pudo entrar al estado MOVE, se vuelve a STAND_BY
                    enter_stand_by(ctx_ptr);
                }
            } // Si se da el STOP, se frena el movimiento y se vuelve a STAND_BY cuando se detiene
            else if (os.last_command == RemoteCommand::STOP) { 
                const bool stop_flag = PositionController::stop_movement(
                    pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                if (stop_flag) {
                    enter_stand_by(ctx_ptr);
                }
            }
            break;
        }
        case OS_State::MOVE: {
            if (ctrl.waypoint_reached) {
                // Si se alcanza el objetivo y las ruedas están detenidas, se completa el waypoint
                ok = complete_current_waypoint(os);
                if (!ok) {// Si no se pudo completar el waypoint (# targets = 0), se manda todo a STAND_BY
                    enter_stand_by(ctx_ptr);
                } 
                else if (os.last_command == RemoteCommand::START) { 
                    // Se establece el siguiente waypoint solo si se está en START
                    // Futuro: avisar a FB que se alcanzó el waypoint
                    ok = set_waypoint(ctx_ptr); 
                    if (!ok) {// Si no se pudo establecer el siguiente waypoint se vuelve a STAND_BY
                        enter_stand_by(ctx_ptr);
                    } else { // Si se pudo establecer el siguiente waypoint, se vuelve a ALIGN
                        // Futuro: avisar a FB que se estableció el nuevo waypoint
                        enter_align(ctx_ptr);
                    }
                } 
                else { // Si no hay más puntos o se dio el stop, se vuelve a STAND_BY
                    // Futuro: avisar a FB que se alcanzó el waypoint
                    enter_stand_by(ctx_ptr);
                }
            } 
            else if (os.last_command == RemoteCommand::STOP || sens.us_obstacle) {
                // Si se recibe el comando STOP, se detiene el movimiento
                const bool stop_flag = PositionController::stop_movement(
                    pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                if (stop_flag) {
                    if (sens.us_obstacle) {
                        // Si se detecta un obstáculo, se entra al estado EVADE
                        enter_evade(ctx_ptr);
                    } else { // Si no hay obstáculo, se vuelve a STAND_BY
                        enter_stand_by(ctx_ptr);
                    }
                }
            }
        }
        case OS_State::EVADE: {
            if (os.last_command == RemoteCommand::STOP) {
                // Si se recibe el comando STOP, se detiene el movimiento
                const bool stop_flag = PositionController::stop_movement(
                    pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                if (stop_flag) {
                    enter_stand_by(ctx_ptr);
                }
            } 
            else {
                // Aquí se puede implementar la lógica de evasión
                // Por ahora solo se actualiza la flag de obstáculos hasta que se libera
                DistanceSensors::update_global_obstacle_flag(
                    sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
                if (!sens.us_obstacle) enter_stand_by(ctx_ptr);
            }
            break;
        }
    }
}


bool set_waypoint(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile OperationData& os = *(ctx_ptr->os_ptr);

    // Fijar el punto objetivo a partir del primer punto en la trayectoria
    if (os.total_targets > 0) {
        PositionController::set_waypoint(os.trajectory[0].x, os.trajectory[0].y, 0.0f,
            ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
        // PositionController::set_control_mode(PositionControlMode::ALIGN, sts.position);
        return SUCCESS;
    }
    return ERROR; // No hay puntos en la trayectoria 
}


bool enter_init(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile OperationData& os = *(ctx_ptr->os_ptr);
    TaskHandlers& task_handlers = *(ctx_ptr->rtos_task_ptr);

    // 0. Actualizar el estado del sistema a INIT
    os.state = OS_State::INIT;

    // 1. Inicialización de módulos individuales
    EncoderReader::init(sens.enc_stepsL, sens.enc_stepsR, sens.enc_wL, sens.enc_wR, sts.encoders);
    PoseEstimator::init(pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R, 
        sens.enc_stepsL, sens.enc_stepsR, sts.pose); 
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    DistanceSensors::init(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst, 
        sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, sts.distance);
    // IMUReader::init(imu_data, sts.imu); // A futuro
    PositionController::init(sts.position, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, 
        ctrl.w_L_ref, ctrl.w_R_ref);

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
        DistanceSensors::Task_CheckLeftObstacle, "USLeft", BASIC_STACK_SIZE, ctx_ptr, 2, &(task_handlers.usLeftHandle), 0);
    xTaskCreatePinnedToCore(
        DistanceSensors::Task_CheckMidObstacle,  "USMid",  BASIC_STACK_SIZE, ctx_ptr, 2, &(task_handlers.usMidHandle), 0);
    xTaskCreatePinnedToCore(
        DistanceSensors::Task_CheckRightObstacle, "USRight", BASIC_STACK_SIZE, ctx_ptr, 2, &(task_handlers.usRightHandle), 0);

    return SUCCESS; // Estado INIT alcanzado
}


bool enter_idle(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile OperationData& os = *(ctx_ptr->os_ptr);

    // Actualizar el estado del sistema a IDLE
    os.state = OS_State::IDLE;

    // Detener control de posición y dejar motores inactivos (libres)
    PositionController::set_waypoint(0.0f, 0.0f, 0.0f, ctrl.x_d, ctrl.y_d, ctrl.theta_d, 
        ctrl.waypoint_reached, sts.position);
    PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position);
    PositionController::set_wheel_speed_ref(0.0f, 0.0f, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
    PositionController::set_control_mode(PositionControlMode::INACTIVE, sts.position);
    MotorController::set_motors_mode(MotorMode::IDLE, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // ⏸️ Pausar encoders, IMU, y estimación de pose -> resetar posición y orientación a cero
    EncoderReader::pause(sens.enc_stepsL, sens.enc_stepsR, sens.enc_wL, sens.enc_wR, sts.encoders);
    // IMUReader::pause(sts.imu); // ← implementar luego
    PoseEstimator::set_state(INACTIVE, sts.pose);
    PoseEstimator::reset_pose(pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R, 
        sens.enc_stepsL, sens.enc_stepsR);

    // 🚫 Desactivar sensores de obstáculos -> se fuerza la limpieza de las flag de obstáculo
    DistanceSensors::set_state(INACTIVE, sts.distance, sens.us_left_dist, sens.us_left_obst, 
        sens.us_mid_dist, sens.us_mid_obst, sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);

    // Limpiar la trayectoria
    clear_trajectory_with_null(os);

    return SUCCESS; // Estado IDLE alcanzado
}


bool enter_stand_by(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile OperationData& os = *(ctx_ptr->os_ptr);

    // Actualizar el estado del sistema a STAND_BY
    os.state = OS_State::STAND_BY;

    // 🧭 Activar lectura de sensores y estimador de pose (para no perder seguimiento del vehículo)
    EncoderReader::resume(sts.encoders);
    // IMUReader::resume(...);
    PoseEstimator::set_state(ACTIVE, sts.pose);

    // 🧷 Mantener control de posición en modo pasivo, con velocidad de referencia 0
    PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position);
    PositionController::set_wheel_speed_ref(0.0f, 0.0f, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
    MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // 🚫 Desactivar sensores de obstáculos -> se fuerza la limpieza de las flag de obstáculo
    DistanceSensors::set_state(INACTIVE, sts.distance, sens.us_left_dist, sens.us_left_obst, 
        sens.us_mid_dist, sens.us_mid_obst, sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);
    
    return SUCCESS; 
}


bool enter_align(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile OperationData& os = *(ctx_ptr->os_ptr);
    bool ok = true;

    // Actualizar el estado del sistema a ALIGN
    os.state = OS_State::ALIGN;

    // 🟢 Reanudar sensores y estimador de posición
    EncoderReader::resume(sts.encoders);
    // IMUReader::resume(...);
    PoseEstimator::set_state(ACTIVE, sts.pose);

    // 🟢 Activar control de posición (v_ref y w_ref)
    // PositionController::set_controller_type();
    PositionController::set_control_mode(PositionControlMode::ALIGN, sts.position);
    MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // 🚫 Mantener desactivada la detección de obstáculos
    DistanceSensors::set_state(INACTIVE, sts.distance, sens.us_left_dist, sens.us_left_obst, 
        sens.us_mid_dist, sens.us_mid_obst, sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);

    return ok;
}


bool enter_move(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile OperationData& os = *(ctx_ptr->os_ptr);
    bool ok = true;

    // Actualizar el estado del sistema a MOVE
    os.state = OS_State::MOVE;

    // 🟢 Reanudar sensores y estimador de posición
    EncoderReader::resume(sts.encoders);
    // IMUReader::resume(...);
    PoseEstimator::set_state(ACTIVE, sts.pose);

    // 🟢 Activar control de posición (v_ref y w_ref)
    // PositionController::set_controller_type();
    PositionController::set_control_mode(PositionControlMode::MOVE, sts.position);
    MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // 🟢 Activar sensores de distancia y realizar una primera lectura forzada para estar bien actualizados
    DistanceSensors::set_state(ACTIVE, sts.distance, sens.us_left_dist, sens.us_left_obst, 
        sens.us_mid_dist, sens.us_mid_obst, sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);
    DistanceSensors::force_check_all_sensors(ctx_ptr);

    return ok;
}


bool enter_evade(GlobalContext* ctx_ptr) {
    volatile SystemStates& sts = *(ctx_ptr->systems_ptr);
    volatile SensorsData& sens = *(ctx_ptr->sensors_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    volatile OperationData& os = *(ctx_ptr->os_ptr);
    bool ok = true;

    // Actualizar el estado del sistema a EVADE
    os.state = OS_State::EVADE;

    // 🟡 Reanudar sensores y estimadores
    EncoderReader::resume(sts.encoders);
    // IMUReader::resume(...);
    PoseEstimator::set_state(ACTIVE, sts.pose);

    // Se fija la velocidad de referencia a cero
    // PositionController::set_controller_type();
    PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position);
    PositionController::set_wheel_speed_ref(0.0f, 0.0f, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
    MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // Se utilizarán los sensores de distancia
    DistanceSensors::set_state(ACTIVE, sts.distance, sens.us_left_dist, sens.us_left_obst, 
        sens.us_mid_dist, sens.us_mid_obst, sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);

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


bool add_waypoint(const float x, const float y, volatile OperationData& os) {
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

} // namespace OS
