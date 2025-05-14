#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#warning "Compilando main_encoder.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
GlobalContext ctx = {
    .systems_ptr = &system_states,
    .kinematic_ptr = nullptr,
    .wheels_ptr = &wheels_data,
    .distance_ptr = nullptr
};

constexpr uint16_t PRINT_PERIOD_MS = 250;

// ====================== TAREA DE MONITOREO SERIAL ======================
void Task_SerialPrint(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_PERIOD_MS); // Cada 250 ms
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.print("Steps: ");
        Serial.print(wheels_data.steps_left);
        uint64_t delta = wheels_data.wL_measured * PRINT_PERIOD_MS;
        Serial.print(" | delta steps: ");
        Serial.print(delta);
        Serial.print(" | w (rad/s): ");
        Serial.println(wheels_data.wL_measured, 2);
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicio: Test con encoder + tarea RTOS");

    // Inicializar motor
    MotorController::init(
        system_states.motor_operation, wheels_data.duty_left, wheels_data.duty_right);
    MotorController::set_motors_mode(
        MOTOR_ACTIVE, system_states.motor_operation, wheels_data.duty_left, wheels_data.duty_right);

    // Inicializar encoder
    EncoderReader::init(wheels_data, system_states.encoder);
    EncoderReader::resume(system_states.encoder);

    // Crear tareas
    xTaskCreatePinnedToCore(
        EncoderReader::Task_EncoderUpdate,
        "EncoderUpdate",
        2048, &ctx, 2, nullptr, APP_CPU_NUM
    );
    xTaskCreatePinnedToCore(
        Task_SerialPrint,
        "SerialMonitor",
        2048, nullptr, 1, nullptr, APP_CPU_NUM
    );

    // Empezamos la operación
    MotorController::set_motors_duty(
        0.5f, 0.5f, wheels_data.duty_left, wheels_data.duty_right, system_states.motor_operation);
}

// ====================== LOOP ======================
void loop() {
    // Nada
}
