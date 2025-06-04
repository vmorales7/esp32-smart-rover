#include "vehicle_os/general_config.h"
#include "motor_drive/motor_controller.h"
#warning "Compilando main_motor_duty.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile MotorMode motors_op = MotorMode::IDLE;
volatile float dutyL = 0.0f;
volatile float dutyR = 0.0f;


// ====================== FUNCIONES AUXILIARES ======================
void print_duty_state(const char* msg) {
    Serial.print(msg);
    Serial.print(" | duty_left: ");
    Serial.print(dutyL, 3);
    Serial.print(" | duty_right: ");
    Serial.println(dutyR, 3);
}

// ====================== SETUP Y LOOP ======================
void setup() {
    Serial.begin(115200);

    // Inicializar
    MotorController::init(motors_op, dutyL, dutyR);
    MotorController::set_motors_mode(MotorMode::MANUAL, motors_op, dutyL, dutyR);
    delay(3000);
    Serial.println("Test: Duty Cycle Motor - Iniciando secuencia...");

    // Avance recto 60% por 5 segundos
    MotorController::set_motors_duty(0.6f, 0.6f, dutyL, dutyR, motors_op);
    print_duty_state("Avanzando recto (60%)");
    delay(5000);

    // Freno antes de girar
    MotorController::set_motors_duty(0.0f, 0.0f, dutyL, dutyR, motors_op);
    print_duty_state("Frenando antes de girar");
    delay(2000);

    // Giro en el lugar hacia la izquierda por 2 segundos
    MotorController::set_motors_duty(-0.4f, 0.4f, dutyL, dutyR, motors_op);
    print_duty_state("Girando en el lugar (izquierda, 40%)");
    delay(2000);

    // Freno antes de avanzar nuevamente
    MotorController::set_motors_duty(0.0f, 0.0f, dutyL, dutyR, motors_op);
    print_duty_state("Frenando antes de avanzar");
    delay(2000);

    // Nuevamente avance recto 50% por 3 segundos
    MotorController::set_motors_duty(0.5f, 0.5f, dutyL, dutyR, motors_op);
    print_duty_state("Avanzando recto nuevamente (50%)");
    delay(5000);

    // Detener motores
    MotorController::set_motors_mode(MotorMode::IDLE, motors_op, dutyL, dutyR);
    Serial.println("Secuencia completada. Detenido.");

    // MotorController::set_motors_duty(0.5f, 0.5f, dutyL, dutyR, motors_op);
}

void loop() {
    // Nada, prueba no se repite 
}
