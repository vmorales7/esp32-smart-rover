#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

#warning "Compilando main_position_basic.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates states;
volatile WheelsData wheels = {0};
volatile KinematicState kinem = {0};
volatile PoseData pose = {0};

GlobalContext ctx = {
    .systems_ptr     = &states,
    .os_ptr          = nullptr,
    .kinematic_ptr   = &kinem,
    .wheels_ptr      = &wheels,
    .imu_ptr         = nullptr,
    .distance_ptr    = nullptr
};

// ====================== CONFIGURACIÓN OBJETIVO ======================
constexpr float X_OBJETIVO = 1.0f;  // metros
constexpr float Y_OBJETIVO = 1.0f;  // metros

// ====================== PROTOTIPOS ======================
void Task_PrintPose(void* pvParameters);
void Task_PrintXY(void* pvParameters);

// ====================== SETUP ======================
void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.println("Debug: Position Control — Basic");

    // Inicialización de módulos
    EncoderReader::init(wheels.steps_L, wheels.steps_R, wheels.w_L, wheels.w_R, states.encoders);
    PoseEstimator::init(kinem.x, kinem.y, kinem.theta, kinem.v, kinem.w, wheels.steps_L, wheels.steps_R, states.pose);
    MotorController::init(states.motors, wheels.duty_L, wheels.duty_R);
    PositionController::init(states.position, wheels.w_L_ref, wheels.w_R_ref);

    // Establecer modos
    PositionController::set_control_mode(PositionControlMode::MOVE_PID, states.position, wheels.w_L_ref, wheels.w_R_ref);
    EncoderReader::resume(states.encoders);
    MotorController::set_motors_mode(MotorMode::AUTO, states.motors, wheels.duty_L, wheels.duty_R);
    PoseEstimator::set_state(ACTIVE, states.pose);

    // Asignar punto objetivo
    kinem.x_d = X_OBJETIVO;
    kinem.y_d = Y_OBJETIVO;

    // Crear tareas principales
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(PositionController::Task_PositionControl, "PositionControl", 2048, &ctx, 2, nullptr, 1);

    // Debug
    xTaskCreatePinnedToCore(Task_PrintPose, "PrintPose", 2048, nullptr, 1, nullptr, 0);
    //xTaskCreatePinnedToCore(Task_PrintXY, "PrintXY", 2048, &ctx, 1, nullptr, 0);
}

void loop() {
    // No se usa
}

// ====================== TAREA DE IMPRESIÓN ======================
void Task_PrintPose(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("x: %.2f  y: %.2f  θ: %.1f  |  v: %.2f  w: %.2f\n", 
            kinem.x, kinem.y, (kinem.theta * PI), kinem.v, kinem.w);
    }
}

void Task_PrintXY(void* pvParameters) {
    auto& k = *static_cast<GlobalContext*>(pvParameters)->kinematic_ptr;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(250);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("%.2f %.2f\n", k.x, k.y);
    }
}
