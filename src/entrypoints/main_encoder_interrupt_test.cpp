#include "project_config.h"
#include "motor_drive/motor_controller.h"
#warning "Compilando main_encoder_interrupt_test.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile MotorMode motors_op = MotorMode::IDLE;
volatile float dutyL = 0.0f;
volatile float dutyR = 0.0f;
volatile long encoder_steps = 0;

// ======================== INTERRUPCIÓN ========================

void IRAM_ATTR encoder_isr() {
    encoder_steps++;
}

// ======================== SETUP ==============================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Prueba básica: lectura de encoder por interrupción");
    
    // Inicializar motor
    MotorController::init(motors_op, dutyL, dutyR);
    MotorController::set_motors_mode(MotorMode::ACTIVE, motors_op, dutyL, dutyR);
    MotorController::set_motors_duty(0.5f, 0.5f, dutyL, dutyR, motors_op);
    
    // Inicializar encoder
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoder_isr, RISING);
}

// ======================== LOOP ===============================

void loop() {
    static unsigned long last_print = millis();
    static long last_steps = 0;

    unsigned long now = millis();
    if (now - last_print >= 500) {
        long current_steps = encoder_steps;
        long delta = current_steps - last_steps;
        last_steps = current_steps;

        float dt = (now - last_print) / 1000.0f; // en segundos
        float w = delta * (2*PI) / RAW_ENCODER_PPR / dt;

        Serial.print("Steps: ");
        Serial.print(current_steps);
        Serial.print(" | delta steps: ");
        Serial.print(delta);
        Serial.print(" | w (rad/s): ");
        Serial.println(w, 2);

        last_print = now;
    }
}
