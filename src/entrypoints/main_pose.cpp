#include "vehicle_os/general_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

#warning "Compilando main_pose.cpp"

// ====================== VARIABLES GLOBALES ======================

volatile SystemStates sts;
volatile SensorsData sens;
volatile ControllerData ctrl;
volatile PoseData pose;

GlobalContext ctx = {
    .systems_ptr   = &sts,
    .sensors_ptr   = &sens,
    .pose_ptr      = &pose,
    .control_ptr   = &ctrl,
    .os_ptr        = nullptr, 
    .rtos_task_ptr = nullptr,
    .evade_ptr     = nullptr
};

constexpr float Wref = 9.0f;
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::ENCODER;

// ====================== Tareas auxiliares ======================

void Task_PrintPose(void* pvParameters);
void Task_PrintXY(void* pvParameters);


// ====================== SETUP ======================

void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.println("Pose Debug — Wheel speed cte 50% Wm nom");
    delay(1000);

    // Inicializar encoder
    EncoderReader::init(sens.enc_stepsL, sens.enc_stepsR, sens.enc_wL, sens.enc_wR, sts.encoders);

    // Inicializar estimador de pose
    PoseEstimator::init(pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R, 
        sens.enc_stepsL, sens.enc_stepsR, sts.pose);
    pose.estimator_type = POSE_ESTIMATOR_TYPE; // Establecer tipo de estimador

    // Inicializar motores
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // Inicializar controlador de posición
    PositionController::init(sts.position, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, 
        ctrl.w_L_ref, ctrl.w_R_ref);
    PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);

    // Instrucciones de inicio
    PositionController::set_wheel_speed_ref(Wref, Wref, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
    PoseEstimator::set_state(ACTIVE, sts.pose);
    EncoderReader::resume(sts.encoders);

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
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile PoseData& pose = *ctx->pose_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("Pose => x: %.2f | y: %.2f | θ: %.2f || Vel => v: %.2f | w: %.2f\n",
                        pose.x, pose.y, pose.theta, pose.v, pose.w);
    }
}

void Task_PrintXY(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(250);

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile PoseData& pose = *ctx->pose_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("%.2f %.2f\n", pose.x, pose.y);
    }
}
