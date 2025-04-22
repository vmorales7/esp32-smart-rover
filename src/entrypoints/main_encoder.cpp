#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#warning "Compilando main_encoder.cpp"

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


// ====================== FUNCIONES AUXILIARES ======================

void print_encoder_state() {
    Serial.print("Steps L/R: ");
    Serial.print(wheels_data.steps_left);
    Serial.print(" / ");
    Serial.print(wheels_data.steps_right);
    Serial.print(" | wL: ");
    Serial.print(wheels_data.w_measured_left, 3);
    Serial.print(" rad/s | wR: ");
    Serial.println(wheels_data.w_measured_right, 3);
}

void ejecutar_fase(const char* msg, float dutyL, float dutyR, uint32_t duracion_ms) {
    Serial.println(msg);
    MotorController::set_motor_duty(
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
            &wheels_data.w_measured_left, &wheels_data.w_measured_right
        );

        if (millis() - t_print >= 500) {
            print_encoder_state();
            t_print += 500;
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
        &wheels_data.w_measured_left, &wheels_data.w_measured_right
    );
    MotorController::set_motor_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );
    EncoderReader::resume(&system_states.encoder);
    Serial.println();
    
    // Secuencia de prueba
    ejecutar_fase("Avanzando recto (70%)", 0.7f, 0.7f, 5000);
    ejecutar_fase("Frenando antes de girar", 0.0f, 0.0f, 2000);
    ejecutar_fase("Girando en el lugar (izq, 30%)", -0.3f, 0.3f, 2000);
    ejecutar_fase("Frenando antes de avanzar", 0.0f, 0.0f, 2000);
    ejecutar_fase("Avanzando recto (50%)", 0.5f, 0.5f, 5000);

    MotorController::set_motor_mode(
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
