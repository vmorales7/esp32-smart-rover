#include <Arduino.h>

constexpr uint8_t MOTOR_LEFT_PWM_PIN = 26;
constexpr uint8_t MOTOR_LEFT_DIR_PIN1  = 25;
constexpr uint8_t MOTOR_LEFT_DIR_PIN2  = 33;

constexpr uint8_t ENCODER_LEFT_A_PIN = 14; // Amarillo
constexpr uint8_t ENCODER_LEFT_B_PIN = 27; // Verde

constexpr float PULSES_PER_REV = 11*21.3;  // pasos reales por vuelta (HalfQuad)
constexpr float RAD_PER_STEP = (2.0f * PI) / PULSES_PER_REV;

volatile long encoder_steps = 0;

// ======================== INTERRUPCIÓN ========================

void IRAM_ATTR encoder_isr() {
    encoder_steps++;
}

// ======================== MOTOR ==============================

void init_motor() {
    pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_LEFT_DIR_PIN2, OUTPUT);

    ledcSetup(0, 1000, 8);  // canal 0, 1kHz, 8 bits
    ledcAttachPin(MOTOR_LEFT_PWM_PIN, 0);

    digitalWrite(MOTOR_LEFT_DIR_PIN1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR_PIN2, LOW);
    ledcWrite(0, 128);  // 50% duty (128 / 255)
}

// ======================== ENCODER =============================

void init_encoder() {
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoder_isr, RISING);
}

// ======================== SETUP ==============================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Prueba básica: lectura de encoder por interrupción");
    
    init_motor();
    init_encoder();
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
        float w = delta * RAD_PER_STEP / dt;

        Serial.print("Steps: ");
        Serial.print(current_steps);
        Serial.print(" | delta steps: ");
        Serial.print(delta);
        Serial.print(" | w (rad/s): ");
        Serial.println(w, 2);

        last_print = now;
    }
}
