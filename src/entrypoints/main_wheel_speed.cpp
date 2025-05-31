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
volatile SystemStates states;
volatile WheelsData wheels_data;
volatile KinematicState kinematic_data;

GlobalContext ctx = {
    .systems_ptr     = &states,
    .os_ptr          = nullptr,
    .kinematic_ptr   = &kinematic_data,
    .wheels_ptr      = &wheels_data,
    .imu_ptr         = nullptr,
    .distance_ptr    = nullptr
};


// ------------------------ Tareas RTOS adicionales -------------------------

void Task_ToggleReference(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TOGGLE_INTERVAL_MS);
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile float* w_L_ref = &ctx->wheels_ptr->w_L_ref;
    volatile float* w_R_ref = &ctx->wheels_ptr->w_R_ref;
    volatile PositionControlMode* control_mode_ptr = &ctx->systems_ptr->position;

    float currentWref = W1;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        currentWref = (currentWref == W1) ? W2 : W1;
        PositionController::set_wheel_speed_ref(
            currentWref, currentWref, *w_L_ref, *w_R_ref, *control_mode_ptr);
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
    MotorController::init(states.motors, wheels_data.duty_L, wheels_data.duty_R);
    MotorController::set_motors_mode(MotorMode::AUTO, states.motors, wheels_data.duty_L, wheels_data.duty_R);
    EncoderReader::init(wheels_data.steps_L, wheels_data.steps_R,wheels_data.w_L, wheels_data.w_R, states.encoders);
    PositionController::init(states.position, wheels_data.w_L_ref, wheels_data.w_R_ref);
    PositionController::set_control_mode(PositionControlMode::MANUAL, states.position, wheels_data.w_L_ref, wheels_data.w_R_ref);
    EncoderReader::resume(states.encoders);
    PositionController::set_wheel_speed_ref(W1, W1, wheels_data.w_L_ref, wheels_data.w_R_ref, states.position);

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
    volatile WheelsData* wheels = ctx->wheels_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("wL %.2f dutyL %.2f | wR %.2f dutyR %.2f\n",
                      wheels->w_L, wheels->duty_L,
                      wheels->w_R, wheels->duty_R);
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
                      wheels->w_L_ref,      // referencia angular
                      wheels->w_L, // velocidad medida
                      wheels->duty_L);  // duty aplicado
    }
}
