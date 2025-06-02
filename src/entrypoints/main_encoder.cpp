#include "vehicle_os/general_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#warning "Compilando main_encoder.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SensorsData sens;
volatile SystemStates sts;
volatile ControllerData ctrl;
GlobalContext ctx = {
    .systems_ptr = &sts,
    .sensors_ptr = &sens,
    .pose_ptr = nullptr,
    .control_ptr = nullptr,
    .os_ptr = nullptr,
    .rtos_task_ptr = nullptr,
    .evade_ptr = nullptr
};

constexpr uint16_t PRINT_PERIOD_MS = 250;

// ====================== TAREA DE MONITOREO SERIAL ======================
void Task_SerialPrint(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_PERIOD_MS); // Cada 250 ms
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.print("Steps: ");
        Serial.print(sens.enc_stepsL);
        uint64_t delta = static_cast<uint64_t>(
            sens.enc_wL * static_cast<float>(PRINT_PERIOD_MS) * MS_TO_S / RAD_PER_PULSE);
        Serial.print(" | delta steps: ");
        Serial.print(delta);
        Serial.print(" | w (rad/s): ");
        Serial.println(sens.enc_wL, 2);
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
        Serial.print(sens.enc_stepsL);
        Serial.print(" ");
        Serial.println(sens.enc_wL, 2);
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicio: Test con encoder + tarea RTOS");

    // Inicializar motor
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    MotorController::set_motors_mode(MotorMode::ACTIVE, sts.motors, ctrl.duty_L, ctrl.duty_R);

    // Inicializar encoder
    EncoderReader::init(sens.enc_stepsL, sens.enc_stepsR, sens.enc_wL, sens.enc_wR, sts.encoders);
    EncoderReader::resume(sts.encoders);

    // Crear tareas
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 2, nullptr, APP_CPU_NUM);
    xTaskCreatePinnedToCore(Task_SerialPrint, "SerialMonitor", 2048, nullptr, 1, nullptr, APP_CPU_NUM);

    // Empezamos la operación
    MotorController::set_motors_duty(0.5f, 0.5f, ctrl.duty_L, ctrl.duty_R, sts.motors);

    // Pruebas de desempeño
    // MotorController::set_motors_mode(MOTOR_IDLE, systems.motor_operation, wheels.duty_L, wheels.duty_R);
    // xTaskCreatePinnedToCore(Task_SerialPrintPlot, "PrintPlot", 2048, nullptr, 1, nullptr, APP_CPU_NUM);
}

// ====================== LOOP ======================
void loop() {
    // Nada
}
