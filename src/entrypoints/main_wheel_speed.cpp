#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/position_controller.h"
#warning "Compilando main_wheel_speed.cpp"

// Constantes
constexpr float W1 = 0.5f * WM_NOM;
constexpr float W2 = 0.7f * WM_NOM;
constexpr uint32_t TOGGLE_INTERVAL_MS = 10000;
constexpr uint32_t PRINT_INTERVAL_MS = 250;

// Variables globales del sistema
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


// ------------------------ Tareas RTOS adicionales -------------------------

void Task_ToggleReference(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TOGGLE_INTERVAL_MS);
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SystemStates& sts = *ctx->systems_ptr;
    volatile SensorsData& sens = *ctx->sensors_ptr;
    volatile ControllerData& ctrl = *ctx->control_ptr;

    float currentWref = W1;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        currentWref = (currentWref == W1) ? W2 : W1;
        PositionController::set_wheel_speed_ref(
            currentWref, currentWref, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
        Serial.printf("Nueva w_ref: %.2f rad/s\n", currentWref);
    }
}

void Task_Printer(void* pvParameters);
void Task_PrintPerformance(void* pvParameters);

// --------------------------- Setup y loop -----------------------------

void setup() {
    Serial.begin(115200);

    // Inicialización
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);
    EncoderReader::init(sens.enc_stepsL, sens.enc_stepsR, sens.enc_wL, sens.enc_wR, sts.encoders);
    PositionController::init(sts.position, ctrl.x_d, ctrl.y_d, ctrl.theta_d, 
        ctrl.waypoint_reached, ctrl.w_L_ref, ctrl.w_R_ref);
    PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
    EncoderReader::resume(sts.encoders);
    PositionController::set_wheel_speed_ref(W1, W1, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);

    // Lanzar todas las tareas RTOS
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 3, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(Task_ToggleReference, "ToggleRef", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(Task_Printer, "Printer", 2048, &ctx, 1, nullptr, 0);
    // xTaskCreatePinnedToCore(Task_PrintPerformance, "PrintPerformance", 2048, &ctx, 1, nullptr, 0);
}

void loop() {
    // No se usa, todo se maneja con RTOS
}


// ------------------------ Tareas RTOS adicionales -------------------------
void Task_Printer(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_INTERVAL_MS);
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SensorsData& sens = *ctx->sensors_ptr;
    volatile ControllerData& ctrl = *ctx->control_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("wL %.2f dutyL %.2f | wR %.2f dutyR %.2f\n",
                      sens.enc_wL, ctrl.duty_L, sens.enc_wR, ctrl.duty_R);
    }
}

void Task_PrintPerformance(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_INTERVAL_MS);
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SensorsData& sens = *ctx->sensors_ptr;
    volatile ControllerData& ctrl = *ctx->control_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        // Imprimir solo datos de la rueda izquierda, separados por tabulaciones
        Serial.printf("%.1f\t%.1f\t%.2f\n",
                      ctrl.w_L_ref,      // referencia angular
                      sens.enc_wL, // velocidad medida
                      ctrl.duty_L);  // duty aplicado
    }
}
