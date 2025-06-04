#include "vehicle_os/general_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#warning "Compilando main_encoder.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SensorsData sens;
volatile SystemStates sts;
volatile ControllerData ctrl;
volatile PoseData pose;
GlobalContext ctx = {
    .systems_ptr = &sts,
    .sensors_ptr = &sens,
    .pose_ptr = &pose,
    .control_ptr = nullptr,
    .os_ptr = nullptr,
    .rtos_task_ptr = nullptr,
    .evade_ptr = nullptr
};

constexpr uint16_t PRINT_PERIOD_MS = 250;
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::ENCODER;

// ====================== TAREA DE MONITOREO SERIAL ======================
void Task_SerialPrint(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_PERIOD_MS); // Cada 250 ms
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.print("phi (°): ");
        Serial.print(sens.enc_phiL * RAD_TO_DEG, 2);
        float delta = static_cast<float>(
            sens.enc_wL * static_cast<float>(PRINT_PERIOD_MS) * MS_TO_S);
        Serial.print(" | delta phi (°): ");
        Serial.print(delta * RAD_TO_DEG, 2);
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

        // Imprimir: tiempo | phi | velocidad
        Serial.print(t, 2);
        Serial.print(" ");
        Serial.print(sens.enc_phiL, 2);
        Serial.print(" ");
        Serial.println(sens.enc_wL, 2);
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicio: Test con encoder + tarea RTOS");

    // Inicialización
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    EncoderReader::init(sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, sts.encoders);

    // Crear tareas
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 1, nullptr, 1);


    // Empezamos la operación
    EncoderReader::resume(sts.encoders);
    MotorController::set_motors_mode(MotorMode::MANUAL, sts.motors, ctrl.duty_L, ctrl.duty_R);
    MotorController::set_motors_duty(0.5f, 0.5f, ctrl.duty_L, ctrl.duty_R, sts.motors);

    // Pruebas de desempeño
    xTaskCreatePinnedToCore(Task_SerialPrint, "SerialMonitor", 2048, nullptr, 1, nullptr, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(Task_SerialPrintPlot, "PrintPlot", 2048, nullptr, 1, nullptr, APP_CPU_NUM);
    // MotorController::set_motors_mode(MotorMode::IDLE, sts.motors, ctrl.duty_L, ctrl.duty_R);
}

// ====================== LOOP ======================
void loop() {
    // Nada
}
