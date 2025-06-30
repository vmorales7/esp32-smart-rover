#include "vehicle_os.h"

namespace OS
{

    void update_local(GlobalContext *ctx_ptr)
    {
        // Acceso a datos globales
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        volatile EvadeContext &evade = *(ctx_ptr->evade_ptr);
        bool ok = true;    // Se usará para verificar si se pudo entrar a un estado

        switch (os.state)
        {
        case OS_State::INIT:
        { // Solo se ejecuta como paso hacia IDLE
            enter_idle(ctx_ptr);
            os.state = OS_State::IDLE;
            set_operation_log(OS_State::IDLE, OS_State::INIT, ctx_ptr);
            break;
        }
        case OS_State::IDLE: 
        {   // No se hace nada en IDLE
            if (os.local_total_targets > 0)
            {
                enter_stand_by(ctx_ptr);
                reset_local_status(ctx_ptr);
                os.state = OS_State::STAND_BY;
                set_operation_log(OS_State::STAND_BY, OS_State::IDLE, ctx_ptr);
            }
            break;
        }
        case OS_State::STAND_BY: 
        {   // Se establece el primer waypoint de la trayectoria y limpiar la flag de waypoint alcanzado
            PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
            ok = set_local_waypoint(ctx_ptr);
            if (ok) 
            {                                 // Si hay puntos pendientes, se entra al estado ALIGN
                enter_align(ctx_ptr);         // El controlador intentará alinear el vehículo hacia el objetivo
                os.state = OS_State::ALIGN;
                set_operation_log(OS_State::ALIGN, OS_State::STAND_BY, ctx_ptr);
                EvadeController::reset_evade_state(ctx_ptr);
            }
            else 
            {   // Si no se poner punto nuevo, nos quedamos en STAND_BY
                set_operation_log(OS_State::STAND_BY, OS_State::STAND_BY, ctx_ptr);
            }
            break;
        }
        case OS_State::ALIGN: 
        {   // Este estado se usa para alinear el vehículo hacia el objetivo
            if (ctrl.waypoint_reached) // Revisar si el controlador ya alineó
            {
                // Poner controlador en modo manual, checkeo de wp faltantes y limpiar flag de waypoint alcanzado
                ok = set_local_waypoint(ctx_ptr);
                if (ok)
                {  // Si se pudo entrar al estado MOVE, se actualiza el estado
                    enter_move(ctx_ptr);
                    os.state = OS_State::MOVE;
                    set_operation_log(OS_State::MOVE, OS_State::ALIGN, ctx_ptr);
                }
                else
                {   // Si no se pudo entrar al estado MOVE, se vuelve a STAND_BY
                    enter_stand_by(ctx_ptr);
                    os.state = OS_State::STAND_BY;
                    set_operation_log(OS_State::STAND_BY, OS_State::ALIGN, ctx_ptr);
                }
            }
            break;
        }
        case OS_State::MOVE: 
        {
            if (ctrl.waypoint_reached)
            {
                // Si se alcanza el objetivo, se completa el waypoint
                complete_local_waypoint(os);
                ok = set_local_waypoint(ctx_ptr);
                if (!ok) { // Si se acabaron los waypoints, se vuelve a STAND_BY
                    enter_stand_by(ctx_ptr);
                    os.state = OS_State::STAND_BY;
                    set_operation_log(OS_State::STAND_BY, OS_State::MOVE, ctx_ptr);
                } else { // Si se pudo establecer el siguiente waypoint, se vuelve a ALIGN
                    enter_align(ctx_ptr);
                    EvadeController::reset_evade_state(ctx_ptr); // Reiniciar el estado de evasión en cada waypoint nuevo
                    os.state = OS_State::ALIGN;
                    set_operation_log(OS_State::ALIGN, OS_State::MOVE, ctx_ptr);
                }
            }
            else if (sens.us_obstacle)
            {// Si  hay obstáculo, primero se detiene el movimiento
                const bool stop_flag = PositionController::stop_movement(
                    pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                if (stop_flag) 
                {   // Movimiento detenido
                    if (check_skip_evade(ctx_ptr) == true) 
                    { // Si se puede saltar el waypoint, se completa y se fija el siguiente
                        sens.us_obstacle = false; // Se limpia la flag de obstáculo
                        if (OS_DEBUG_MODE) Serial.println("Waypoint tapado por obstáculo, se pasa al siguiente waypoint");
                        complete_local_waypoint(os); // Se completa el waypoint actual y se pasa al siguiente
                        ok = set_local_waypoint(ctx_ptr); // Checkeo ante error y limpiar flag de waypoint alcanzado
                        if (!ok) { // Si no se pudo fijar, se vuelve a STAND_BY
                            enter_stand_by(ctx_ptr);
                            os.state = OS_State::STAND_BY;
                            set_operation_log(OS_State::STAND_BY, OS_State::MOVE, ctx_ptr);
                        } else { // Si se pudo completar el waypoint, se vuelve a ALIGN
                            enter_align(ctx_ptr);
                            os.state = OS_State::ALIGN;
                            set_operation_log(OS_State::ALIGN, OS_State::MOVE, ctx_ptr);
                            EvadeController::reset_evade_state(ctx_ptr); // Reiniciar el estado de evasión
                        }
                    } 
                    else { // Si es necesario evadir, se entra al estado EVADE
                        if (evade.include_evade) {// Caso con evasión incluida
                            EvadeController::start_evade(ctx_ptr);
                            enter_evade(ctx_ptr);
                        } else { // Caso sin evasión, se espera a que el camino esté libre
                            enter_wait_free_path(ctx_ptr);
                        }
                        set_operation_log(OS_State::EVADE, OS_State::MOVE, ctx_ptr);
                        os.state = OS_State::EVADE;
                    }
                }
            }
            break;
        }
        case OS_State::EVADE: 
        {
            if (evade.include_evade) // Caso en que está activa la evasión
            {   
                EvadeController::update_evade(ctx_ptr);
                if (evade.state == EvadeState::FINISHED) // Si se completó la evasión, se vuelve a ALIGN
                {   
                    enter_align(ctx_ptr);
                    os.state = OS_State::ALIGN;
                    set_operation_log(OS_State::ALIGN, OS_State::EVADE, ctx_ptr);
                }
                else if (evade.state == EvadeState::FAIL) // Reiniciar evasión y establecer un nuevo waypoint
                {   
                    complete_local_waypoint(os);
                    EvadeController::reset_evade_state(ctx_ptr);
                    ok = set_local_waypoint(ctx_ptr);
                    if (!ok)
                    {   // No hay más waypoints disponibles, volver a STAND_BY
                        enter_stand_by(ctx_ptr);
                        os.state = OS_State::STAND_BY;
                        set_operation_log(OS_State::STAND_BY, OS_State::EVADE, ctx_ptr);
                    }
                    else
                    {   // Waypoint siguiente fijado con éxito, ir a ALIGN
                        enter_align(ctx_ptr);
                        os.state = OS_State::ALIGN;
                        set_operation_log(OS_State::ALIGN, OS_State::EVADE, ctx_ptr);
                    }
                }
            }
            else // Caso en que no está activa la evasión: espera a que el camino esté libre
            {   
                DistanceSensors::update_global_obstacle_flag(
                    sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
                if (!sens.us_obstacle) 
                {   // Si el camino está libre, se vuelve a MOVE
                    enter_move(ctx_ptr);
                    os.state = OS_State::MOVE;
                    set_operation_log(OS_State::MOVE, OS_State::EVADE, ctx_ptr);
                }
            }
            break;
        }
        }
    }

    void update_online(GlobalContext *ctx_ptr)
    {
        // Acceso a datos globales
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        volatile EvadeContext &evade = *(ctx_ptr->evade_ptr);

        bool ok = true;                    // Se usará para verificar si se pudo entrar a un estado
        FB_State fb_result = FB_State::OK; // Estado de la comunicación con Firebase

        switch (os.state)
        {
        case OS_State::INIT:
        { // Solo se ejecuta como paso hacia IDLE
            enter_idle(ctx_ptr);
            os.state = OS_State::IDLE;
            set_operation_log(OS_State::IDLE, OS_State::INIT, ctx_ptr);
            break;
        }
        case OS_State::IDLE:
        {
            if (CheckOnlineStatus(ctx_ptr) == false)
            {   // Si no hay conexión WiFi o Firebase no está verificado, se mantiene en IDLE
                set_operation_log(OS_State::IDLE, OS_State::IDLE, ctx_ptr);
            }
            else if (os.fb_last_command == UserCommand::START)
            {   // Si se da un start desde Firebase, se entra a STAND_BY y se resetea el estado
                fb_result = reset_online_status(ctx_ptr);
                if (fb_result == FB_State::OK)
                { // Si se pudo resetear el estado online, se entra a STAND_BY
                    enter_stand_by(ctx_ptr);
                    os.state = OS_State::STAND_BY;
                    set_operation_log(OS_State::STAND_BY, OS_State::IDLE, ctx_ptr);
                }
                else
                { // Si no se pudo resetear, se queda en IDLE
                    set_operation_log(OS_State::IDLE, OS_State::IDLE, ctx_ptr);
                }
            }
            break;
        }
        case OS_State::STAND_BY:
        {
            // Si no hay conexión WiFi o Firebase no está verificado, se espera a que se conecte
            if (CheckOnlineStatus(ctx_ptr) == false)
            {   
                if (OS_DEBUG_MODE) Serial.println("STAND_BY: No hay conexión WiFi o Firebase no verificado.");
                set_operation_log(OS_State::STAND_BY, OS_State::STAND_BY, ctx_ptr);
            }
            // Siempre se verifica si el último punto completado fue enviado a Firebase
            // Si hay error, se queda en STAND_BY por no poder remover el pendiente
            else if (os.fb_completed_but_not_sent)
            {   // Completar el waypoint pendiente antes de seguir
                SendReachedWaypoint(ctx_ptr); 
                // Sí o sí debe estar completado, sino no se elimina el waypoint pendiente y quedaría pegado 
            }
            // Último punto alcanzado fue enviado + comando START
            else if (os.fb_last_command == UserCommand::START)
            {   // Hay que pedir el waypoint pendiente mas antiguo a Firebase
                fb_result = FirebaseComm::UpdatePendingWaypoint(
                    os.fb_target_buffer.x, os.fb_target_buffer.y, os.fb_target_buffer.ts, os.fb_state);
                // Si se recibe un punto invalido, internamente se tratará de eliminar y pedir el siguiente
                // Si se recibe podemos fijar el waypoint en el controlador y pasar a ALIGN
                if (fb_result == FB_State::OK)
                {
                    set_online_waypoint(ctx_ptr);
                    enter_align(ctx_ptr);
                    os.state = OS_State::ALIGN;
                    set_operation_log(OS_State::ALIGN, OS_State::STAND_BY, ctx_ptr);
                    EvadeController::reset_evade_state(ctx_ptr);
                }
            } // Último punto alcanzado fue enviado + comando IDLE
            else if (os.fb_last_command == UserCommand::IDLE)
            {
                fb_result = FirebaseComm::ClearPendingWaypoints(os.fb_state);
                if (fb_result != FB_State::PENDING) 
                {   // Si se pudo eliminar los waypoints pendiente o fue error, se vuelve a IDLE
                    enter_idle(ctx_ptr);
                    os.state = OS_State::IDLE;
                    set_operation_log(OS_State::IDLE, OS_State::STAND_BY, ctx_ptr);
                }
            }
            break;
        }
        case OS_State::ALIGN:
        {   // Este estado se usa para alinear el vehículo hacia el objetivo
            ok = CheckOnlineStatus(ctx_ptr);
            if (!ok || os.fb_last_command != UserCommand::START)
            { // Si hay error de conexión o STOP/IDLE, se frena el movimiento y se vuelve a STAND_BY cuando se detiene
                bool stop_flag = PositionController::stop_movement(
                    pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                if (stop_flag)
                {
                    enter_stand_by(ctx_ptr);
                    os.state = OS_State::STAND_BY;
                    set_operation_log(OS_State::STAND_BY, OS_State::ALIGN, ctx_ptr);
                    if (OS_DEBUG_MODE) {
                        if (!ok) Serial.println("ALIGN: Error de conexión, se vuelve a STAND_BY.");
                        else Serial.println("ALIGN: Comando STOP/IDLE recibido, se vuelve a STAND_BY.");
                    }
                }
            }
            else if (ctrl.waypoint_reached)
            {   // Si ya se alcanza el waypoint se pasa a MOVE
                ctrl.waypoint_reached = false; // Limpiar la flag para el controlador de posición
                enter_move(ctx_ptr);
                os.state = OS_State::MOVE;
                set_operation_log(OS_State::MOVE, OS_State::ALIGN, ctx_ptr);
                if (OS_DEBUG_MODE) Serial.println("ALIGN: Alineación completada. Transición a MOVE.");
            }
            else
            {   // Se vuelve a fijar el modo de control, ya que un stop_movement lo deja en modo manual
                // Pero podría haber vuelto a conectar antes de hacer la transición a STAND_BY
                // Si no ocurrió stop_movement, set_control_mode no hace nada porque ya está en ALIGN
                PositionController::set_control_mode(
                    PositionControlMode::ALIGN, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
            }
            break;
        }
        case OS_State::MOVE:
        {
            ok = CheckOnlineStatus(ctx_ptr);
            if (ctrl.waypoint_reached)
            {   // Si se alcanza el waypoint, se detiene, se registra y se pasa a STAND_BY
                register_finished_waypoint_data(true, ctx_ptr); // Registrar datos del waypoint alcanzado
                enter_stand_by(ctx_ptr);
                os.state = OS_State::STAND_BY;
                set_operation_log(OS_State::STAND_BY, OS_State::MOVE, ctx_ptr);
            }
            // Si se da el STOP/IDLE o falla la conexión, se detiene el movimiento y se pasa a STAND_BY
            else if (!ok || os.fb_last_command != UserCommand::START)
            { // Si hay error de conexión o STOP/IDLE, se frena el movimiento y se vuelve a STAND_BY cuando se detiene
                bool stop_flag = PositionController::stop_movement(
                    pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                if (stop_flag)
                {
                    enter_stand_by(ctx_ptr);
                    os.state = OS_State::STAND_BY;
                    set_operation_log(OS_State::STAND_BY, OS_State::MOVE, ctx_ptr);
                    if (OS_DEBUG_MODE) {
                        if (!ok) Serial.println("MOVE: Error de conexión, se vuelve a STAND_BY.");
                        else Serial.println("MOVE: Comando STOP/IDLE recibido, se vuelve a STAND_BY.");
                    }
                }
            }
            else if (sens.us_obstacle)
            {   // Si hay obstáculo y seguimos en START + conexión OK, se detiene y entra a evasión
                bool stop_flag = PositionController::stop_movement(
                    pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                if (stop_flag)
                {   // Esperar a que el movimiento se detenga
                    if (evade.include_evade)
                    {   // Está activa la evasión, se inicia y se entra al estado EVADE
                        EvadeController::start_evade(ctx_ptr);
                        enter_evade(ctx_ptr);
                    }
                    else
                    {   // No está activa la evasión, se espera a que el camino esté libre
                        enter_wait_free_path(ctx_ptr);
                    }
                    set_operation_log(OS_State::EVADE, OS_State::MOVE, ctx_ptr);
                    os.state = OS_State::EVADE;
                }
            }
            else
            {   // Se vuelve a fijar el modo de control, ya que un stop_movement lo deja en modo manual
                // Pero podría haber vuelto a conectar antes de hacer la transición a STAND_BY
                // Si no ocurrió stop_movement, set_control_mode no hace nada porque ya está en MOVE
                PositionController::set_control_mode(
                    PositionControlMode::MOVE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
            }
            break;
        }
        case OS_State::EVADE:
        {
            ok = CheckOnlineStatus(ctx_ptr);
            if (os.fb_last_command != UserCommand::START || !ok)
            {   // Si se recibe el comando STOP/IDLE o hay error de conexión, se detiene el movimiento
                bool stop_flag = PositionController::stop_movement(
                    pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                if (stop_flag)
                {
                    enter_stand_by(ctx_ptr);
                    os.state = OS_State::STAND_BY;
                    set_operation_log(OS_State::STAND_BY, OS_State::MOVE, ctx_ptr);
                    if (OS_DEBUG_MODE) {
                        if (!ok) Serial.println("EVADE: Error de conexión, se vuelve a STAND_BY.");
                        else Serial.println("EVADE: Comando STOP/IDLE recibido, se vuelve a STAND_BY.");
                    }
                }
            }
            else if (evade.include_evade)
            {
                EvadeController::update_evade(ctx_ptr);
                // Si se completó la evasión, se vuelve a ALIGN
                if (evade.state == EvadeState::FINISHED)
                {
                    enter_align(ctx_ptr);
                    os.state = OS_State::ALIGN;
                    set_operation_log(OS_State::ALIGN, OS_State::EVADE, ctx_ptr);
                }
                else if (evade.state == EvadeState::FAIL)
                {   // Si falla la evasión, se vuelve a STAND_BY y se avisa punto fallido
                    PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                    register_finished_waypoint_data(false, ctx_ptr); // Registrar datos del waypoint fallado
                    enter_stand_by(ctx_ptr);
                    os.state = OS_State::STAND_BY;
                    set_operation_log(OS_State::STAND_BY, OS_State::EVADE, ctx_ptr);
                }
            }
            else
            {   // Caso en que no está activa la evasión: espera a que el camino esté libre
                DistanceSensors::update_global_obstacle_flag(
                    sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
                if (!sens.us_obstacle)
                {
                    enter_move(ctx_ptr);
                    os.state = OS_State::MOVE;
                    set_operation_log(OS_State::MOVE, OS_State::EVADE, ctx_ptr);
                }
            }
            break;
        }
        }
    }

    bool enter_init(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        TaskHandlers &task_handlers = *(ctx_ptr->rtos_task_ptr);

        // 0. Actualizar el estado del sistema a INIT
        os.state = OS_State::INIT;

        // 1. Iniciar serial para depuración y retardo de inicio
        Serial.begin(115200);
        delay(1000);
        Serial.println("\nIniciando operación del vehículo...");
        delay(5000);

        // 1. Inicializar WiFi, tiempo y firebase si es necesario. No permiten avanzar hasta que se completen.
        if (ONLINE_MODE)
        {
            Serial.println("\nIniciando WiFi...");
            begin_wifi();
            Serial.println("\nIniciando conexión a time...");
            init_time();
            Serial.println("\nIniciando Firebase...");
            FirebaseComm::ConnectFirebase();
            delay(10000);
        } // Ambas son operaciones bloqueantes, por lo que el sistema no avanzará hasta que se completen
        Serial.println("\n COsas wifi listas...");
        delay(1000);

        // 2. Inicialización de módulos individuales
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            IMUSensor::init(sens.imu_acc, sens.imu_w, sens.imu_theta, sts.imu);
        }
        EncoderReader::init(sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, sts.encoders);
        PoseEstimator::init(pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R,
                            sens.enc_phiL, sens.enc_phiR, sens.imu_theta, sts.pose);
        MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
        DistanceSensors::init(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst,
                              sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, sts.distance);
        PositionController::init(sts.position, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached,
                                 ctrl.w_L_ref, ctrl.w_R_ref);

        // 3. Lanzar tareas RTOS núcleo 1
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            xTaskCreatePinnedToCore(IMUSensor::Task_IMUData, "UpdateIMU", 2 * BASIC_STACK_SIZE, ctx_ptr, 3, nullptr, 1);
        }
        xTaskCreatePinnedToCore(
            EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2 * BASIC_STACK_SIZE, ctx_ptr, 3, nullptr, 1);
        xTaskCreatePinnedToCore(
            PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 4 * BASIC_STACK_SIZE, ctx_ptr, 2, nullptr, 1);
        xTaskCreatePinnedToCore(
            MotorController::Task_WheelControl, "WheelControl", 2 * BASIC_STACK_SIZE, ctx_ptr, 2, nullptr, 1);
        xTaskCreatePinnedToCore(
            PositionController::Task_PositionControl, "PositionControl", 3 * BASIC_STACK_SIZE, ctx_ptr, 1, nullptr, 1);

        // 4. Lanzar tareas RTOS núcleo 0
        xTaskCreatePinnedToCore(OS::Task_VehicleOS, "VehicleOS", 3 * BASIC_STACK_SIZE, ctx_ptr, 0, nullptr, 0);
        xTaskCreatePinnedToCore(
            DistanceSensors::Task_CheckObstacle, "CheckObstacles", 2 * BASIC_STACK_SIZE, ctx_ptr, 2, &(task_handlers.obstacle_handle), 0);
        xTaskCreatePinnedToCore(Task_StopOnRiskFlags, "StopOnRiskFlags", BASIC_STACK_SIZE, ctx_ptr, 1, nullptr, 0);
        if (ONLINE_MODE)
        {
            xTaskCreatePinnedToCore(Task_CheckWifi, "CheckWifi", BASIC_STACK_SIZE, ctx_ptr, 1, nullptr, 0);
            xTaskCreatePinnedToCore(FirebaseComm::Task_PushStatus, "FirebasePushStatus", 4 * BASIC_STACK_SIZE, ctx_ptr, 1, nullptr, 0);
            xTaskCreatePinnedToCore(FirebaseComm::Task_GetCommands, "FirebaseGetCommands", 4 * BASIC_STACK_SIZE, ctx_ptr, 1, nullptr, 0);
            xTaskCreatePinnedToCore(FirebaseComm::Task_Loop, "FirebaseLoop", BASIC_STACK_SIZE, ctx_ptr, 1, nullptr, 0);
        }
        if (OS_DEBUG_MODE)
            Serial.println("Tareas RTOS iniciadas.");
        return SUCCESS; // Estado INIT alcanzado
    }

    bool enter_idle(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);

        // Detener control de posición y dejar motores inactivos (libres)
        PositionController::set_control_mode(PositionControlMode::INACTIVE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
        MotorController::set_motors_mode(MotorMode::IDLE, sts.motors, ctrl.duty_L, ctrl.duty_R);

        // ⏸️ Pausar encoders, IMU, y estimación de pose -> resetar posición y orientación a cero
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            IMUSensor::set_state(INACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);
        }
        EncoderReader::pause(sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, sts.encoders);
        PoseEstimator::set_state(INACTIVE, sts.pose);

        // 🚫 Desactivar sensores de obstáculos -> se fuerza la limpieza de las flag de obstáculo
        DistanceSensors::set_state(INACTIVE, sts.distance,
                                   sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);

        return SUCCESS; // Estado IDLE alcanzado
    }

    bool reset_local_status(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);

        // Resetear datos de movimiento y posición
        PoseEstimator::reset_pose(pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R,
                                  sens.enc_phiL, sens.enc_phiR, sens.imu_theta);
        // Resetear datos de control
        PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
        PositionController::set_waypoint(0.0f, 0.0f, 0.0f, ctrl.x_d, ctrl.y_d, ctrl.theta_d,
                                         ctrl.waypoint_reached, sts.position);
        // Restear flag de obstáculo
        DistanceSensors::reset_system(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst,
                                sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);                    
        // Reiniciar el estado de evasión
        EvadeController::reset_evade_state(ctx_ptr);
        return SUCCESS;
    }

    bool enter_stand_by(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);

        // Activar lectura de sensores y estimador de pose (para no perder seguimiento del vehículo)
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            IMUSensor::set_state(ACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);
        }
        EncoderReader::resume(sts.encoders);
        PoseEstimator::set_state(ACTIVE, sts.pose);

        // Mantener control de posición en modo manual, con velocidad de referencia 0
        PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

        // Desactivar sensores de obstáculos -> se fuerza la limpieza de las flag de obstáculo
        DistanceSensors::set_state(INACTIVE, sts.distance,
                                   sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
        DistanceSensors::reset_system(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst,
                                      sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);

        // Reset del estado de evasión
        EvadeController::reset_evade_state(ctx_ptr);

        return SUCCESS;
    }

    bool enter_align(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        bool ok = true;

        // 🟢 Reanudar sensores y estimador de posición
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            IMUSensor::set_state(ACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);
        }
        EncoderReader::resume(sts.encoders);
        PoseEstimator::set_state(ACTIVE, sts.pose);

        // 🟢 Activar control de posición (v_ref y w_ref) y actualizar el tipo de controlador
        PositionController::set_controller_type(os.fb_controller_type, ctrl.controller_type);
        PositionController::set_control_mode(PositionControlMode::ALIGN, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

        // 🚫 Mantener desactivada la detección de obstáculos
        DistanceSensors::set_state(INACTIVE, sts.distance,
                                   sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
        DistanceSensors::reset_system(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst,
                                      sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);

        return ok;
    }

    bool enter_move(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        bool ok = true;

        // 🟢 Reanudar sensores y estimador de posición
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            IMUSensor::set_state(ACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);
        }
        EncoderReader::resume(sts.encoders);
        PoseEstimator::set_state(ACTIVE, sts.pose);

        // 🟢 Activar sensores de distancia y realizar una primera lectura forzada para estar bien actualizados
        DistanceSensors::set_state(ACTIVE, sts.distance,
                                   sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
        DistanceSensors::force_check_sensors(ctx_ptr);

        // 🟢 Activar control de posición (v_ref y w_ref) y actualizar el tipo de controlador
        PositionController::set_controller_type(os.fb_controller_type, ctrl.controller_type);
        PositionController::set_control_mode(PositionControlMode::MOVE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

        return ok;
    }

    bool enter_evade(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        bool ok = true;

        // 🟡 Reanudar sensores y estimadores
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            IMUSensor::set_state(ACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);
        }
        EncoderReader::resume(sts.encoders);
        PoseEstimator::set_state(ACTIVE, sts.pose);

        // Se fija la velocidad de referencia a cero
        // PositionController::set_controller_type();
        PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

        // Sensores de distancia desactivados
        DistanceSensors::set_state(INACTIVE, sts.distance,
                                   sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);

        return SUCCESS;
    }

    bool enter_rotate(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        bool ok = true;

        // 🟢 Reanudar sensores y estimador de posición
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            IMUSensor::set_state(ACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);
        }
        EncoderReader::resume(sts.encoders);
        PoseEstimator::set_state(ACTIVE, sts.pose);

        // 🟢 Activar control de posición (v_ref y w_ref)
        // PositionController::set_controller_type();
        PositionController::set_control_mode(PositionControlMode::ROTATE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

        // 🚫 Mantener desactivada la detección de obstáculos
        DistanceSensors::set_state(INACTIVE, sts.distance,
                                   sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
        DistanceSensors::reset_system(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst,
                                      sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);

        return ok;
    }

    bool enter_wait_free_path(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        bool ok = true;

        // 🟢 Reanudar sensores y estimador de posición
        if (pose.estimator_type == PoseEstimatorType::COMPLEMENTARY)
        {
            IMUSensor::set_state(ACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);
        }
        EncoderReader::resume(sts.encoders);
        PoseEstimator::set_state(ACTIVE, sts.pose);

        // 🟢 Activar control de posición (v_ref y w_ref)
        PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
        MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

        // Mantener activada la detección de obstáculos
        DistanceSensors::set_state(ACTIVE, sts.distance,
                                   sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);

        return ok;
    }

    bool set_local_waypoint(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);

        // Fijar el punto objetivo a partir del primer punto en la trayectoria
        PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
        if (os.local_total_targets > 0)
        {
            PositionController::set_waypoint(os.local_trajectory[0].x, os.local_trajectory[0].y, 0.0f,
                                             ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
            return SUCCESS;
        }
        return ERROR; // No hay puntos en la trayectoria
    }

    void clear_local_trajectory(volatile OperationData &os)
    {
        for (uint8_t i = 0; i < MAX_TRAJECTORY_POINTS; ++i)
        {
            os.local_trajectory[i].x = NULL_WAYPOINT_XY;
            os.local_trajectory[i].y = NULL_WAYPOINT_XY;
            os.local_trajectory[i].ts = NULL_TIMESTAMP;
        }
        os.local_total_targets = 0U;
    }

    bool complete_local_waypoint(volatile OperationData &os)
    {
        if (os.local_total_targets > 0)
        {
            os.local_total_targets--;
            for (uint8_t i = 1; i < MAX_TRAJECTORY_POINTS; ++i)
            { // Mover los puntos hacia adelante
                os.local_trajectory[i - 1].x = os.local_trajectory[i].x;
                os.local_trajectory[i - 1].y = os.local_trajectory[i].y;
                os.local_trajectory[i - 1].ts = os.local_trajectory[i].ts;
            }
            os.local_trajectory[MAX_TRAJECTORY_POINTS - 1].x = NULL_WAYPOINT_XY;
            os.local_trajectory[MAX_TRAJECTORY_POINTS - 1].y = NULL_WAYPOINT_XY;
            return SUCCESS; // Se completó el waypoint
        }
        return ERROR; // No hay waypoints para completar
    }

    bool add_local_waypoint(const float x, const float y, const float ts, volatile OperationData &os)
    {
        if (os.local_total_targets < MAX_TRAJECTORY_POINTS)
        {
            os.local_trajectory[os.local_total_targets].x = x;
            os.local_trajectory[os.local_total_targets].y = y;
            os.local_trajectory[os.local_total_targets].ts = ts;
            os.local_total_targets++;
            return SUCCESS;
        }
        return ERROR; // No se pudo agregar el waypoint, lista llena
    }

    bool CheckOnlineStatus(GlobalContext *ctx_ptr)
    {
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        return ((os.wifi_status != WifiStatus::TIMEOUT) || os.fb_state != FB_State::CONNECTION_ERROR);
        // return ((os.fb_state == FB_State::ERROR) || (check_wifi() != WifiStatus::TIMEOUT) || !FirebaseComm::ready());
    }

    FB_State reset_online_status(GlobalContext *ctx_ptr)
    {
        static bool local_reset_done = false;

        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);

        // Solo ejecutar el reset local una vez por intento completo
        if (!local_reset_done) {
            reset_local_status(ctx_ptr); // Resetear datos locales solo al comienzo
            os.fb_target_buffer.reset();
            os.fb_waypoint_data.reset();
            os.fb_completed_but_not_sent = false;
            local_reset_done = true;
        }

        const FB_State result = FirebaseComm::ClearAllLogs(os.fb_state);

        if (result == FB_State::OK || result == FB_State::ERROR) {
            local_reset_done = false;  // Permitir nuevo reset en la próxima llamada
        }

        if (OS_DEBUG_MODE) {
            if (result == FB_State::OK)
                Serial.println("Historial de status y waypoints borrado en Firebase.");
            else if (result == FB_State::ERROR)
                Serial.println("Error al borrar historial de status y waypoints en Firebase.");
        }

        return result;
    }

    bool check_skip_evade(GlobalContext *ctx_ptr)
    {
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        volatile SensorsData &sens = *(ctx_ptr->sensors_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);

        float dx = ctrl.x_d - pose.x; // Distancia al waypoint en X
        float dy = ctrl.y_d - pose.y; // Distancia al waypoint en Y
        float dist_to_waypoint = sqrtf(dx*dx + dy*dy);
        float min_dist_sensor = fminf(sens.us_left_dist, 
            fminf(sens.us_mid_dist, sens.us_right_dist)) / 100; // Distancia mínima del sensor
        if (OS_DEBUG_MODE) Serial.printf(
            "Distancia al waypoint: %.2f, Distancia mínima del sensor: %.2f\n", dist_to_waypoint, min_dist_sensor);
        
        // Si el waypoint está muy cerca del obstáculo, se puede saltar el waypoint
        const bool skip_waypoint = (dist_to_waypoint < MAX_EVADE_SKIP_DIST && 
            min_dist_sensor < dist_to_waypoint && dist_to_waypoint - min_dist_sensor < MIN_EVADE_BEHIND_DIST);

        return skip_waypoint;
    }

    FB_State SendReachedWaypoint(GlobalContext *ctx_ptr)
    {   
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        FB_State result = FirebaseComm::CompleteWaypoint(
            os.fb_waypoint_data.input_ts,
            os.fb_waypoint_data.wp_x,
            os.fb_waypoint_data.wp_y,
            os.fb_waypoint_data.start_ts,
            os.fb_waypoint_data.end_ts,
            os.fb_waypoint_data.reached_flag,
            os.fb_waypoint_data.pos_x,
            os.fb_waypoint_data.pos_y,
            os.fb_waypoint_data.controller_type,
            os.fb_waypoint_data.iae,
            os.fb_waypoint_data.rmse,
            os.fb_state);
        if (result == FB_State::OK) 
        {
            os.fb_completed_but_not_sent = false;
            if (OS_DEBUG_MODE) Serial.println("SendReachedWaypoint: waypoint alcanzado enviado correctamente.");
        }
        if (result == FB_State::ERROR && OS_DEBUG_MODE)
            Serial.println("SendReachedWaypoint: error permanente al completar waypoint.");
        return result;
    }

    void set_online_waypoint(GlobalContext *ctx_ptr)
    {
        volatile SystemStates &sts = *(ctx_ptr->systems_ptr);
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);

        // Guardar datos iniciales del waypoint en el buffer
        os.fb_waypoint_data.input_ts = os.fb_target_buffer.ts;
        os.fb_waypoint_data.wp_x = os.fb_target_buffer.x;
        os.fb_waypoint_data.wp_y = os.fb_target_buffer.y;
        os.fb_waypoint_data.start_ts = get_unix_timestamp();

        // Establecer el objetivo del controlador: pasar a manual para poder fijar el waypoint
        PositionController::set_control_mode(
            PositionControlMode::MANUAL, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
        PositionController::set_waypoint(
            os.fb_target_buffer.x, os.fb_target_buffer.y, 0.0f,
            ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
    }

    void register_finished_waypoint_data(const bool reached_flag, GlobalContext *ctx_ptr)
    {
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);

        // Guardar datos del waypoint alcanzado
        os.fb_waypoint_data.end_ts = get_unix_timestamp();
        os.fb_waypoint_data.reached_flag = reached_flag;
        os.fb_waypoint_data.pos_x = pose.x;
        os.fb_waypoint_data.pos_y = pose.y;
        os.fb_waypoint_data.controller_type = static_cast<uint8_t>(ctrl.controller_type);
        os.fb_waypoint_data.iae = ctrl.iae;
        os.fb_waypoint_data.rmse = ctrl.rmse;

        // Avisar que se alcanzó un waypoint y que falta enviar
        os.fb_completed_but_not_sent = true; // Marcar que se alcanzó un waypoint pero no se envió
    }

    void Task_VehicleOS(void *pvParameters) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(OS_UPDATE_PERIOD_MS);
        GlobalContext *ctx_ptr = static_cast<GlobalContext *>(pvParameters);
        for (;;)  {
            vTaskDelayUntil(&xLastWakeTime, period);
            if (ONLINE_MODE) {
                update_online(ctx_ptr); // Actualizar Firebase si está en modo online
            } else {
                update_local(ctx_ptr);
            }
        }
    }

    void Task_StopOnRiskFlags(void *pvParameters) {
        GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
        volatile OperationData& os = *(ctx->os_ptr);
        volatile SensorsData& sens = *(ctx->sensors_ptr);
        volatile PoseData& pose = *(ctx->pose_ptr);
        volatile ControllerData& ctrl = *(ctx->control_ptr);
        volatile SystemStates& sts = *(ctx->systems_ptr);

        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(OS_CHECK_STOP_PERIOD_MS);

        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            const bool is_moving = (os.state == OS_State::MOVE);
            const bool is_aligning = (os.state == OS_State::ALIGN);
            const bool stop_cmd = (os.fb_last_command != UserCommand::START);
            const bool offline = ONLINE_MODE && (!CheckOnlineStatus(ctx)); 
            if ((is_moving && sens.us_obstacle) || ((is_moving || is_aligning) && (stop_cmd || offline)))
            {
                if (sts.position != PositionControlMode::MANUAL && sts.position != PositionControlMode::INACTIVE)
                    PositionController::stop_movement(pose.v, pose.w, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
            }
        }
    }

    void set_operation_log(const OS_State new_state, const OS_State old_state, GlobalContext *ctx_ptr)
    {
        volatile OperationData &os = *(ctx_ptr->os_ptr);
        volatile PoseData &pose = *(ctx_ptr->pose_ptr);
        volatile ControllerData &ctrl = *(ctx_ptr->control_ptr);
        if (new_state == OS_State::INIT)
        {
            strncpy(const_cast<char *>(os.last_log), "Entrando a estado INIT", sizeof(os.last_log));
        }
        else if (new_state == OS_State::IDLE)
        {
            strncpy(const_cast<char *>(os.last_log), "Entrando a estado IDLE", sizeof(os.last_log));
        }
        else if (new_state == OS_State::STAND_BY)
        {
            if (old_state == OS_State::ALIGN)
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado STAND-BY desde ALIGN en (x=%.2f, y=%.2f)", pose.x, pose.y);
            }
            else if (old_state == OS_State::MOVE)
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado STAND-BY desde MOVE en (x=%.2f, y=%.2f)", pose.x, pose.y);
            }
            else if (old_state == OS_State::EVADE)
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado STAND-BY desde EVADE en (x=%.2f, y=%.2f)", pose.x, pose.y);
            }
            else if (old_state == OS_State::STAND_BY)
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado STAND-BY desde STAND-BY en (x=%.2f, y=%.2f)", pose.x, pose.y);
            }
            else if (old_state == OS_State::IDLE)
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado STAND-BY desde IDLE en (x=%.2f, y=%.2f)", pose.x, pose.y);
            }
            else
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado STAND-BY en (x=%.2f, y=%.2f)", pose.x, pose.y);
            }
        }
        else if (new_state == OS_State::ALIGN)
        {
            if (old_state == OS_State::STAND_BY)
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado ALIGN desde STAND-BY hacia (x=%.2f, y=%.2f)", ctrl.x_d, ctrl.y_d);
            }
            else if (old_state == OS_State::MOVE)
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado ALIGN desde MOVE hacia (x=%.2f, y=%.2f)", ctrl.x_d, ctrl.y_d);
            }
            else if (old_state == OS_State::EVADE)
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado ALIGN desde EVADE hacia (x=%.2f, y=%.2f)", ctrl.x_d, ctrl.y_d);
            }
            else
            {
                snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                         "Entrando a estado ALIGN hacia (x=%.2f, y=%.2f)", ctrl.x_d, ctrl.y_d);
            }
        }
        else if (new_state == OS_State::MOVE)
        {
            snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                     "Entrando a estado MOVE desde ALIGN hacia (x=%.2f, y=%.2f)", ctrl.x_d, ctrl.y_d);
        }
        else if (new_state == OS_State::EVADE)
        {
            snprintf(const_cast<char *>(os.last_log), sizeof(os.last_log),
                     "Entrando a estado EVADE desde MOVE en (x=%.2f, y=%.2f)", pose.x, pose.y);
        }
        os.last_log[sizeof(os.last_log) - 1] = '\0'; // Seguridad de terminador null
        if (new_state != old_state && OS_DEBUG_MODE)
        {
            Serial.printf("Log de transición: %s\n", os.last_log); // Imprimir log de transición
        }
    }

} // namespace OS
