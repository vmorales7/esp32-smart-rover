#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#warning "Compilando main_encoder.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates systems;
volatile WheelsData wheels = {0};
GlobalContext ctx = {
    .systems_ptr     = &systems,
    .os_ptr          = nullptr,
    .kinematic_ptr   = nullptr,
    .wheels_ptr      = &wheels,
    .imu_ptr         = nullptr,
    .distance_ptr    = nullptr
};

constexpr uint16_t PRINT_PERIOD_MS = 250;

// ====================== TAREA DE MONITOREO SERIAL ======================
void Task_SerialPrint(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_PERIOD_MS); // Cada 250 ms
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.print("Steps: ");
        Serial.print(wheels.steps_L);
        uint64_t delta = static_cast<uint64_t>(
            wheels.w_L * static_cast<float>(PRINT_PERIOD_MS) * MS_TO_S / RAD_PER_PULSE);
        Serial.print(" | delta steps: ");
        Serial.print(delta);
        Serial.print(" | w (rad/s): ");
        Serial.println(wheels.w_L, 2);
    }
}

void Task_SerialPrintPlot(void* pvParameters) { 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_PERIOD_MS);

    static uint32_t t0_ms = millis();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        // Tiempo actual en segundos
        float t = (millis() - t0_ms) * MS_TO_S;

        // Imprimir: tiempo | steps | velocidad
        Serial.print(t, 2);
        Serial.print(" ");
        Serial.print(wheels.steps_L);
        Serial.print(" ");
        Serial.println(wheels.w_L, 2);
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicio: Test con encoder + tarea RTOS");

    // Inicializar motor
    MotorController::init(systems.motors, wheels.duty_L, wheels.duty_R);
    MotorController::set_motors_mode(MotorMode::ACTIVE, systems.motors, wheels.duty_L, wheels.duty_R);

    // Inicializar encoder
    EncoderReader::init(wheels.steps_L, wheels.steps_R, wheels.w_L, wheels.w_R, systems.encoders);
    EncoderReader::resume(systems.encoders);

    // Crear tareas
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 2, nullptr, APP_CPU_NUM);
    xTaskCreatePinnedToCore(Task_SerialPrint, "SerialMonitor", 2048, nullptr, 1, nullptr, APP_CPU_NUM);

    // Empezamos la operación
    MotorController::set_motors_duty(0.5f, 0.5f, wheels.duty_L, wheels.duty_R, systems.motors);

    // Pruebas de desempeño
    // MotorController::set_motors_mode(MOTOR_IDLE, systems.motor_operation, wheels.duty_L, wheels.duty_R);
    // xTaskCreatePinnedToCore(Task_SerialPrintPlot, "PrintPlot", 2048, nullptr, 1, nullptr, APP_CPU_NUM);
}

// ====================== LOOP ======================
void loop() {
    // Nada
}
