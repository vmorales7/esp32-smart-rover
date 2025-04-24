#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include <ESP32Encoder.h>
#warning "Compilando main_encoder_basic.cpp"

// ====================== VARIABLES GLOBALES ======================

volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};

ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

int64_t lastCountLeft = 0;
int64_t lastCountRight = 0;

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test básico: ESP32Encoder lectura directa + velocidad");

    // Inicializar motor
    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left, &wheels_data.duty_right
    );

    MotorController::set_motor_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left, &wheels_data.duty_right
    );

    // Inicializar encoder
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLeft.attachHalfQuad(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);

    // Fijar un duty bajo constante
    MotorController::set_motor_duty(
        0.5f, 0.5f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
}

// ====================== LOOP ======================

void loop() {
    static unsigned long t_last = 0;
    unsigned long t_now = millis();

    if ((t_now - t_last) >= 500) {
        float dt = (t_now - t_last) * MS_TO_S;  // en segundos
        t_last = t_now;

        // Leer pasos actuales
        int64_t countLeft = encoderLeft.getCount();

        // Calcular deltas
        int64_t deltaLeft = countLeft - lastCountLeft;
        lastCountLeft = countLeft;

        // Calcular velocidad angular
        float wLeft = deltaLeft * RAD_PER_PULSE / dt;

        // Mostrar resultados
        Serial.print("Steps: ");
        Serial.print(countLeft);
        Serial.print(" | delta steps: ");
        Serial.print(deltaLeft);
        Serial.print(" | w (rad/s): ");
        Serial.println(wLeft, 2);
    }
}
