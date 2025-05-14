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
volatile SystemStates states = {0};
volatile WheelsData wheels_data = {0};
volatile KinematicState kinematic_data = {0};

GlobalContext global_ctx = {
    .systems_ptr = &states,
    .kinematic_ptr = &kinematic_data,
    .wheels_ptr = &wheels_data,
    .distance_ptr = nullptr  // no usado en este ejemplo
};


// ------------------------ Tareas RTOS adicionales -------------------------

void Task_ToggleReference(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TOGGLE_INTERVAL_MS);
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile float* wL_ref = &ctx->wheels_ptr->wL_ref;
    volatile float* wR_ref = &ctx->wheels_ptr->wR_ref;
    volatile uint8_t* control_mode_ptr = &ctx->systems_ptr->position_controller;

    float currentWref = W1;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        currentWref = (currentWref == W1) ? W2 : W1;
        PositionController::set_wheel_speed_ref(
            currentWref, currentWref, *wL_ref, *wR_ref, *control_mode_ptr);
        Serial.printf("Nueva w_ref: %.2f rad/s\n", currentWref);
    }
}

void Task_Printer(void* pvParameters);
void Task_PrintPerformance(void* pvParameters);
void Task_SerialPlot(void* pvParameters);

// --------------------------- Setup y loop -----------------------------

void setup() {
    Serial.begin(115200);

    // Inicialización
    MotorController::init(states.motor_operation, wheels_data.duty_left, wheels_data.duty_right);
    MotorController::set_motors_mode(MOTOR_AUTO, states.motor_operation, wheels_data.duty_left, wheels_data.duty_right);
    EncoderReader::init(wheels_data, states.encoder);
    PositionController::init(states.position_controller, wheels_data.wL_ref, wheels_data.wR_ref);
    PositionController::set_control_mode(SPEED_REF_MANUAL, states.position_controller);
    EncoderReader::resume(states.encoder);
    PositionController::set_wheel_speed_ref(W1, W1, wheels_data.wL_ref, wheels_data.wR_ref, states.position_controller);

    // Lanzar todas las tareas RTOS
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &global_ctx, 3, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &global_ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(Task_ToggleReference, "ToggleRef", 2048, &global_ctx, 1, nullptr, 1);
    // xTaskCreatePinnedToCore(Task_Printer, "Printer", 2048, &global_ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(Task_PrintPerformance, "PrintPerformance", 2048, &global_ctx, 1, nullptr, 0);
    // xTaskCreatePinnedToCore(Task_SerialPlot, "SerialPlot", 2048, &global_ctx, 1, nullptr, 0);

}

void loop() {
    // No se usa, todo se maneja con RTOS
}




// ------------------------ Tareas RTOS adicionales -------------------------
void Task_Printer(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_INTERVAL_MS);
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile WheelsData* wheels = ctx->wheels_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("wL %.2f dutyL %.2f | wR %.2f dutyR %.2f\n",
                      wheels->wL_measured, wheels->duty_left,
                      wheels->wR_measured, wheels->duty_right);
    }
}

void Task_PrintPerformance(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_INTERVAL_MS);
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile WheelsData* wheels = ctx->wheels_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        // Imprimir solo datos de la rueda izquierda, separados por tabulaciones
        Serial.printf("%.1f\t%.1f\t%.2f\n",
                      wheels->wL_ref,      // referencia angular
                      wheels->wL_measured, // velocidad medida
                      wheels->duty_left);  // duty aplicado
    }
}

void Task_SerialPlot(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_INTERVAL_MS);
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile WheelsData* wheels = ctx->wheels_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        Serial.print("wL_ref:");
        Serial.print(wheels->wL_ref, 2);
        Serial.print("\t");

        Serial.print("wL_meas:");
        Serial.print(wheels->wL_measured, 2);
        Serial.print("\t");

        Serial.print("dutyL:");
        Serial.println(wheels->duty_left, 2);
    }
}
