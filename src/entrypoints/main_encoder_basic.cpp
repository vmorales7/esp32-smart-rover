
#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include <ESP32Encoder.h>
#warning "Compilando main_encoder_basic.cpp"

// ====================== VARIABLES GLOBALES ======================

volatile SystemStates system_states = {
    .motor_operation      = MOTOR_IDLE,
    .encoder              = ACTIVE,
    .imu                  = INACTIVE,
    .distance             = INACTIVE,
    .pose_estimator       = INACTIVE,
    .position_controller  = SPEED_REF_INACTIVE,
    .evade_controller     = INACTIVE
};

volatile WheelsData wheels_data = {0};

ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test básico: ESP32Encoder lectura directa");

    // Inicializar motor y encoder
    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left, &wheels_data.duty_right
    );

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLeft.attachHalfQuad(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    encoderRight.attachHalfQuad(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);

    MotorController::set_motor_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left, &wheels_data.duty_right
    );

    // Fijar un duty bajo constante para mover lentamente
    MotorController::set_motor_duty(
        0.5f, 0.5f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
}

// ====================== LOOP ======================

void loop() {
    static unsigned long t_last = 0;
    unsigned long t = millis();

    // Leer encoder
    int64_t currentCountLeft = encoderLeft.getCount();
    int64_t currentCountRight = encoderRight.getCount();

    // Imprimir cada 500 ms
    if ((t - t_last) >= 500) {
        t_last = t;

        Serial.print("Steps L: ");
        Serial.print(-currentCountLeft);
        Serial.print(" | Steps R: ");
        Serial.println(currentCountRight);
    }
}