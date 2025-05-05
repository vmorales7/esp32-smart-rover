#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/position_controller.h"
#warning "Compilando main_wheel_speed_basic.cpp"

constexpr float W1 = 0.5f * WM_NOM;
constexpr float W2 = 0.6f * WM_NOM;
constexpr uint32_t TOGGLE_INTERVAL_MS = 10000;
constexpr uint32_t PRINT_INTERVAL_MS = 1000;

volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile KinematicState kinematic_data = {0};

void setup() {
    Serial.begin(115200);

    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    EncoderReader::init(
        &system_states.encoder,
        &wheels_data.steps_left,
        &wheels_data.steps_right,
        &wheels_data.wL_measured,
        &wheels_data.wR_measured
    );

    EncoderReader::resume(&system_states.encoder);

    MotorController::set_motors_mode(MOTOR_AUTO,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );
}

void loop() {
    static uint32_t lastToggle = millis();
    static uint32_t lastPrint = millis();
    static float currentWref = W1;

    // Alternar velocidad de referencia cada 10 segundos
    if (millis() - lastToggle > TOGGLE_INTERVAL_MS) {
        currentWref = (currentWref == W1) ? W2 : W1;
        wheels_data.wL_ref = currentWref;
        wheels_data.wR_ref = currentWref;
        lastToggle = millis();
    }

    // Actualizar encoder y control
    EncoderReader::update_encoder_data(
        &system_states.encoder,
        &wheels_data.steps_left,
        &wheels_data.steps_right,
        &wheels_data.wL_measured,
        &wheels_data.wR_measured
    );

    MotorController::update_motors_control(
        &wheels_data.wL_ref,
        &wheels_data.wR_ref,
        &wheels_data.wL_measured,
        &wheels_data.wR_measured,
        &wheels_data.duty_left,
        &wheels_data.duty_right,
        &system_states.motor_operation
    );

    // Imprimir velocidad cada segundo
    if (millis() - lastPrint > PRINT_INTERVAL_MS) {
        float wref = currentWref;
        float wL = wheels_data.wL_measured;
        float wR = wheels_data.wR_measured;

        Serial.printf("wref: %.1f  wL: %.1f  wR: %.1f\n", wref, wL, wR);
        lastPrint = millis();
    }

    delay(WHEEL_CONTROL_PERIOD_MS);  // mantiene periodo de control constante
}
