#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

#warning "Compilando main_pose.cpp"

// ====================== VARIABLES GLOBALES ======================

volatile SystemStates states = {0};
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

constexpr float Wref = 9.0f;

// ====================== Tarea auxiliar ======================

void Task_PrintPose(void* pvParameters);
void Task_PrintXY(void* pvParameters);


// ====================== SETUP ======================

void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.println("Pose Debug — Wheel speed cte 50% Wm nom");
    delay(1000);

    // Inicializar encoder
    EncoderReader::init(wheels.steps_L, wheels.steps_R, wheels.w_L, wheels.w_R, states.encoders);

    // Inicializar estimador de pose
    PoseEstimator::init(kinem.x, kinem.y, kinem.theta, kinem.v, kinem.w, wheels.steps_L, wheels.steps_R, states.pose);

    // Inicializar motores
    MotorController::init(states.motors, wheels.duty_L, wheels.duty_R);
    MotorController::set_motors_mode(MOTOR_AUTO, states.motors, wheels.duty_L, wheels.duty_R);

    // Inicializar controlador de posición
    PositionController::init(states.position, wheels.w_L_ref, wheels.w_R_ref);
    PositionController::set_control_mode(SPEED_REF_MANUAL, states.position, wheels.w_L_ref, wheels.w_R_ref);

    // Instrucciones de inicio
    PositionController::set_wheel_speed_ref(Wref, Wref, wheels.w_L_ref, wheels.w_R_ref, states.position);
    PoseEstimator::set_state(ACTIVE, states.pose);
    EncoderReader::resume(states.encoders);

    // Crear tareas RTOS
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 2048, &ctx, 1, nullptr, 1);

    // Debug
    xTaskCreatePinnedToCore(Task_PrintPose, "PrintPose", 2048, &ctx, 1, nullptr, 0);

    // Performance tests
    // MotorController::set_motors_mode(MOTOR_IDLE, states.motors, wheels.duty_L, wheels.duty_R);
    // xTaskCreatePinnedToCore(Task_PrintXY, "PrintXY", 2048, &ctx, 1, nullptr, 0);
}

void loop() {
    // Todo se maneja en tareas
}


// ====================== Para print ======================

void Task_PrintPose(void* pvParameters) {
    auto& k = *static_cast<GlobalContext*>(pvParameters)->kinematic_ptr;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("Pose => x: %.2f | y: %.2f | θ: %.2f || Vel => v: %.2f | w: %.2f\n",
                        k.x, k.y, k.theta, k.v, k.w);
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
