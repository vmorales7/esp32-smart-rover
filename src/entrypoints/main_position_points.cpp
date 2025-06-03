#include "vehicle_os/general_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

#warning "Compilando main_position_points.cpp"


// ====================== VARIABLES GLOBALES ======================
volatile SystemStates sts;
volatile SensorsData sens;
volatile ControllerData ctrl;
volatile PoseData pose;

GlobalContext ctx = {
    .systems_ptr     = &sts,
    .sensors_ptr     = &sens,
    .pose_ptr        = &pose,
    .control_ptr     = &ctrl,
    .os_ptr          = nullptr, 
    .rtos_task_ptr   = nullptr,
    .evade_ptr       = nullptr
};

// Memoria para retomar post-obstáculo
PositionControlMode last_control_mode = PositionControlMode::INACTIVE; 
bool en_pausa_por_obstaculo = false; // Indica si se pausó por un obstáculo


// ====================== CONFIGURACIÓN PUNTOS ======================

constexpr ControlType CONTROLLER_TYPE = ControlType::PID; // Tipo de controlador a utilizar
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::COMPLEMENTARY; // Tipo de estimador de pose

struct Waypoint {
    float x;
    float y;
};
constexpr uint8_t NUM_PUNTOS = 3;
Waypoint trayectoria[NUM_PUNTOS] = {
    {1.0f, 1.0f},
    {2.0f, 1.0f},
    {2.0f, 2.0f}
};
uint8_t punto_actual = 0;


// ====================== TAREA PRINCIPAL ======================
void Task_ControlPorPuntos(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(200);

    volatile GlobalContext& ctx = *static_cast<GlobalContext*>(pvParameters);
    volatile PoseData& pose = *ctx.pose_ptr;
    volatile SystemStates& sts = *ctx.systems_ptr;
    volatile ControllerData& ctrl = *ctx.control_ptr;
    volatile OperationData& os = *ctx.os_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        // Verificar si ya llegó
        if (!ctrl.waypoint_reached) {
            Serial.printf("Posición actual: (x=%.2f, y=%.2f)\n", pose.x, pose.y);
            // Detención por obstáculo si está avanzando
            if (sens.us_obstacle && sts.position == PositionControlMode::MOVE) {
                Serial.println("Obstáculo detectado, deteniendo motores.");
                PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, 
                    ctrl.w_L_ref, ctrl.w_R_ref);
                PositionController::set_wheel_speed_ref(0.0f, 0.0f, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
                en_pausa_por_obstaculo = true;
            } 
            else if (!sens.us_obstacle && en_pausa_por_obstaculo) {
                Serial.printf("Obstáculo despejado, reanudando movimiento hacia el punto (x=%.2f, y=%.2f)\n", 
                    ctrl.x_d, ctrl.y_d);
                PositionController::set_control_mode(last_control_mode, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
                en_pausa_por_obstaculo = false;
            }
            DistanceSensors::update_global_obstacle_flag(
                sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
        } 
        else if (sts.position == PositionControlMode::ALIGN) {
            Serial.printf("Punto %d alineado. Punto actual (x=%.2f, y=%.2f)\n", punto_actual, pose.x, pose.y);
            Serial.println("Iniciando movimiento hacia el punto objetivo...");
            PositionController::set_control_mode(PositionControlMode::MOVE, sts.position, 
                ctrl.w_L_ref, ctrl.w_R_ref);
            last_control_mode = PositionControlMode::MOVE;
        } 
        else if (sts.position == PositionControlMode::MOVE) {
            Serial.printf("Punto %d alcanzado.\n", punto_actual);
            punto_actual++;
            if (punto_actual < NUM_PUNTOS) {
                PositionController::set_control_mode(PositionControlMode::ALIGN, sts.position, 
                    ctrl.w_L_ref, ctrl.w_R_ref);
                last_control_mode = PositionControlMode::ALIGN;
                PositionController::set_waypoint(trayectoria[punto_actual].x, trayectoria[punto_actual].y, 0.0f,
                    ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
                Serial.printf("Nuevo punto objetivo: (x=%.2f, y=%.2f)\n", ctrl.x_d, ctrl.y_d);
                Serial.println("Alineando hacia el nuevo punto...");
            }
            else {
                Serial.println("Todos los puntos alcanzados. Deteniendo motores.");
                ctrl.waypoint_reached = false; // Reiniciar flag para evitar reacciones inesperadas
                PositionController::set_control_mode(PositionControlMode::INACTIVE, sts.position, 
                    ctrl.w_L_ref, ctrl.w_R_ref);
                vTaskSuspend(nullptr); // Suspender tarea si se completaron todos los puntos
            }
        }
    }
}


// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicio: Seguimiento de puntos");

    // Inicialización de sistemas
    EncoderReader::init(sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, sts.encoders);
    PoseEstimator::init(pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R,
        sens.enc_phiL, sens.enc_phiR, sens.imu_theta, sts.pose);
    pose.estimator_type = POSE_ESTIMATOR_TYPE; // Establecer tipo de estimador de pose
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    DistanceSensors::init(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst, 
        sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, sts.distance);
    PositionController::init(sts.position, ctrl.x_d, ctrl.y_d, ctrl.theta_d, 
        ctrl.waypoint_reached, ctrl.w_L_ref, ctrl.w_R_ref);

    // Puesta en marcha de todo
    EncoderReader::resume(sts.encoders);
    // IMUReader::resume(sts.imu); // A futuro
    PoseEstimator::set_state(ACTIVE, sts.pose);
    DistanceSensors::set_state(ACTIVE, sts.distance, 
        sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
    PositionController::set_controller_type(CONTROLLER_TYPE, ctrl.controller_type);
    PositionController::set_control_mode(PositionControlMode::ALIGN, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
    MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // Tareas de sistema (core 1)
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(PositionController::Task_PositionControl, "PositionControl", 2048, &ctx, 1, nullptr, 1);

    // Sensores de distancia (core 0)
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckObstacle, "CheckObstacles", 2048, &ctx, 1, nullptr, 0);

    // Tarea principal de control por puntos (core 0)
    xTaskCreatePinnedToCore(Task_ControlPorPuntos, "ControlPorPuntos", 2048, &ctx, 2, nullptr, 0);
}

void loop() {
    // No se usa
}