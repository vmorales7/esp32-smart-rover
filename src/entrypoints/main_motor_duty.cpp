#include "project_config.h"
#include "motor_drive/motor_controller.h"
#warning "Compilando main_motor_duty.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};


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

    MotorController::set_motors_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    delay(3000);
    Serial.println("Test: Duty Cycle Motor - Iniciando secuencia...");

    MotorController::set_motors_duty(
        0.5f, 0.5f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Avanzando recto (50%)");
    delay(5000);

    // Freno antes de girar
    MotorController::set_motors_duty(
        0.0f, 0.0f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Frenando antes de girar");
    delay(2000);

    // Giro en el lugar hacia la izquierda por 2 segundos
    MotorController::set_motors_duty(
        -0.4f, 0.4f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Girando en el lugar (izquierda, 40%)");
    delay(2000);

    // Freno antes de avanzar nuevamente
    MotorController::set_motors_duty(
        0.0f, 0.0f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Frenando antes de avanzar");
    delay(2000);

    // Nuevamente avance recto 50% por 3 segundos
    MotorController::set_motors_duty(
        0.4f, 0.4f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    print_duty_state("Avanzando recto nuevamente (40%)");
    delay(5000);

    // Detener motores
    MotorController::set_motors_mode(
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
