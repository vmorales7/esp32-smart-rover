#include "vehicle_os/general_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

#warning "Compilando main_position_basic.cpp"

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


// ====================== CONFIGURACIÓN OBJETIVO ======================

constexpr ControlType CONTROLLER_TYPE = ControlType::PID;
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::ENCODER;

// Escoger desplazamiento / alineación / rotación pura
PositionControlMode control_state = PositionControlMode::MOVE; 
constexpr float X_OBJETIVO = 1.0f; 
constexpr float Y_OBJETIVO = 1.0f;
constexpr float Q_OBJETIVO = 0.0f * PI/180.f;  // modificar solo si se quiere un giro puro


// ====================== PROTOTIPOS ======================

void Task_PrintPose(void* pvParameters);
void Task_PrintXY(void* pvParameters);


// ====================== SETUP ======================
void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.println("Main: Position Control — Basic");

    // Inicialización de módulos
    EncoderReader::init(sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, sts.encoders);
    // IMUReader::init(sens.imu_ax, sens.imu_wz, sens.imu_theta, sts.imu); // A futuro
    PoseEstimator::init(pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R,
        sens.enc_phiL, sens.enc_phiR, sens.imu_theta, sts.pose);
    pose.estimator_type = POSE_ESTIMATOR_TYPE; // Establecer tipo de estimador
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    PositionController::init(sts.position, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, ctrl.w_L_ref, ctrl.w_R_ref); 

    // Establecer modos
    EncoderReader::resume(sts.encoders);
    // IMUReader::resume(sts.imu); // A futuro
    PoseEstimator::set_state(ACTIVE, sts.pose);
    PositionController::set_controller_type(CONTROLLER_TYPE, ctrl.controller_type);
    PositionController::set_control_mode(control_state, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
    MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // Asignar punto objetivo
    PositionController::set_waypoint(X_OBJETIVO, Y_OBJETIVO, Q_OBJETIVO, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);

    // Crear tareas principales
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(PositionController::Task_PositionControl, "PositionControl", 2048, &ctx, 2, nullptr, 1);

    // Debug
    xTaskCreatePinnedToCore(Task_PrintPose, "PrintPose", 2048, &ctx, 1, nullptr, 0);
    //xTaskCreatePinnedToCore(Task_PrintXY, "PrintXY", 2048, &ctx, 1, nullptr, 0);
}

void loop() {
    // No se usa
}

// ====================== TAREA DE IMPRESIÓN ======================
void Task_PrintPose(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);
    volatile GlobalContext& ctx = *static_cast<GlobalContext*>(pvParameters);
    volatile PoseData& pose = *ctx.pose_ptr;
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("x: %.2f  y: %.2f  θ: %.1f  |  v: %.2f  w: %.2f\n", 
            pose.x, pose.y, (pose.theta * 180.0f / M_PI), pose.v, pose.w);
    }
}

void Task_PrintXY(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(250);
    volatile GlobalContext& ctx = *static_cast<GlobalContext*>(pvParameters);
    volatile PoseData& pose = *ctx.pose_ptr;
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("%.2f %.2f\n", pose.x, pose.y);
    }
}
