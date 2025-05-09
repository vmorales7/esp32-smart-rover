#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/position_controller.h"
#warning "Compilando main_wheel_speed_basic.cpp"

constexpr float W1 = 0.5f * WM_NOM;
constexpr float W2 = 0.6f * WM_NOM;
constexpr uint32_t TOGGLE_INTERVAL_MS = 10000;
constexpr uint32_t PRINT_INTERVAL_MS = 250;

volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile KinematicState kinematic_data = {0};
volatile uint8_t control_mode = 0;

void setup() {
    Serial.begin(115200);
    MotorController::init(
        &system_states.motor_operation, &wheels_data.duty_left, &wheels_data.duty_right);
    EncoderReader::init(&wheels_data, &system_states.encoder);
    PositionController::init(
        &control_mode, &wheels_data.wL_ref, &wheels_data.wR_ref);
    EncoderReader::resume(&system_states.encoder);
    MotorController::set_motors_mode(MOTOR_AUTO,
        &system_states.motor_operation, &wheels_data.duty_left, &wheels_data.duty_right);
    PositionController::set_control_mode(SPEED_REF_MANUAL, &control_mode);
}

void loop() {
    static uint32_t lastToggle = millis();
    static uint32_t lastPrint = millis();
    static float currentWref = W1;

    // Alternar velocidad de referencia cada 10 segundos
    if (millis() - lastToggle > TOGGLE_INTERVAL_MS) {
        currentWref = (currentWref == W1) ? W2 : W1;
        PositionController::set_wheel_speed_ref(
            currentWref, currentWref,
            &wheels_data.wL_ref, &wheels_data.wR_ref,
            &control_mode
        );
        Serial.printf("wref %.1f\n", currentWref);  // Imprimir nueva referencia
        lastToggle = millis();
    }

    // Actualizar encoder y control
    EncoderReader::update_encoder_data(&wheels_data, &system_states.encoder);
    MotorController::update_motors_control(&wheels_data, &system_states.motor_operation);

    // Imprimir velocidad y duty cada PRINT_INTERVAL_MS
    if (millis() - lastPrint > PRINT_INTERVAL_MS) {
        Serial.printf("wL %.1f dutyL %.2f\n", wheels_data.wL_measured, wheels_data.duty_left);
        lastPrint = millis();
    }

    delay(WHEEL_CONTROL_PERIOD_MS);  // mantiene periodo de control constante
}
