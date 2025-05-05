#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#warning "Compilando main_encoder.cpp"

// ====================== VARIABLES GLOBALES ======================

volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
constexpr uint32_t PRINT_PERIOD = 250; //ms

// ====================== FUNCIONES AUXILIARES ======================

void print_encoder_state() {
    Serial.print("Steps L/R: ");
    Serial.print(wheels_data.steps_left);
    Serial.print(" / ");
    Serial.print(wheels_data.steps_right);
    Serial.print(" | wL: ");
    Serial.print(wheels_data.wL_measured, 2);
    Serial.print(" | wR: ");
    Serial.println(wheels_data.wR_measured, 2);
}

void ejecutar_fase(const char* msg, float dutyL, float dutyR, uint32_t duracion_ms) {
    Serial.println(msg);
    MotorController::set_motors_duty(
        dutyL, dutyR,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );

    unsigned long t0 = millis();
    unsigned long t_print = t0;

    while (millis() - t0 < duracion_ms) {
        EncoderReader::update_encoder_data(
            &system_states.encoder,
            &wheels_data.steps_left, &wheels_data.steps_right,
            &wheels_data.wL_measured, &wheels_data.wR_measured
        );

        if (millis() - t_print >= PRINT_PERIOD) {
            print_encoder_state();
            t_print += PRINT_PERIOD;
        }

        delay(10);  // ← Espera para garantizar al menos 10 ms entre actualizaciones
    }
}

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: Encoder + Duty Control");

    // Inicialización de motor y encoder
    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );
    EncoderReader::init(
        &system_states.encoder,
        &wheels_data.steps_left, &wheels_data.steps_right,
        &wheels_data.wL_measured, &wheels_data.wR_measured
    );
    MotorController::set_motors_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );
    EncoderReader::resume(&system_states.encoder);
    Serial.println();
    
    // Secuencia de prueba
    ejecutar_fase("Avanzando recto (70%)", 0.7f, 0.7f, 10000);
    ejecutar_fase("Frenando antes de girar", 0.0f, 0.0f, 1000);
    ejecutar_fase("Girando en el lugar (izq, 40%)", 0.0f, 0.4f, 10000);
    ejecutar_fase("Frenando antes de avanzar", 0.0f, 0.0f, 1000);
    ejecutar_fase("Avanzando recto (50%)", 0.5f, 0.5f, 10000);

    MotorController::set_motors_mode(
        MOTOR_IDLE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );
    Serial.println("Secuencia completada. Motores en IDLE.");
}

void loop() {
    // Nada
}
