#include "project_config.h"
#include "motor_drive/motor_controller.h"
#warning "Compilando main_motor_duty.cpp"

// ====================== VARIABLES GLOBALES ======================

volatile SystemStates system_states = {
    .motor_operation      = MOTOR_IDLE,
    .encoder              = INACTIVE,
    .imu                  = INACTIVE,
    .distance             = INACTIVE,
    .pose_estimator       = INACTIVE,
    .position_controller  = SPEED_REF_INACTIVE,
    .evade_controller     = INACTIVE
};

volatile WheelsData wheels_data = {
    .steps_left = 0,
    .steps_right = 0,
    .w_measured_left = 0.0f,
    .w_measured_right = 0.0f,
    .w_ref_left = 0.0f,
    .w_ref_right = 0.0f,
    .duty_left = 0.0f,
    .duty_right = 0.0f
};


// ====================== FUNCIONES AUXILIARES ======================

void print_duty_state(const char* msg) {
    Serial.print(msg);
    Serial.print(" | duty_left: ");
    Serial.print(wheels_data.duty_left, 3);
    Serial.print(" | duty_right: ");
    Serial.println(wheels_data.duty_right, 3);
}

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);

    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    MotorController::set_motor_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    delay(3000);
    Serial.println("Test: Duty Cycle Motor - Iniciando secuencia...");

    // Avance recto 70% por 3 segundos
    MotorController::set_motor_duty(
        0.7f, 0.7f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Avanzando recto (90%)");
    delay(5000);

    // Freno antes de girar
    MotorController::set_motor_duty(
        0.0f, 0.0f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Frenando antes de girar");
    delay(2000);

    // Giro en el lugar hacia la izquierda por 2 segundos
    MotorController::set_motor_duty(
        -0.4f, 0.4f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Girando en el lugar (izquierda, 50%)");
    delay(2000);

    // Freno antes de avanzar nuevamente
    MotorController::set_motor_duty(
        0.0f, 0.0f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Frenando antes de avanzar");
    delay(2000);

    // Nuevamente avance recto 50% por 3 segundos
    MotorController::set_motor_duty(
        0.7f, 0.7f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Avanzando recto nuevamente (70%)");
    delay(5000);

    // Detener motores
    MotorController::set_motor_mode(
        MOTOR_IDLE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );
    Serial.println("Secuencia completada. Detenido.");
}

void loop() {
    // Nada, prueba no se repite
}
