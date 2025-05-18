#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_avance1.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile DistanceSensorData distance_data = {0};
constexpr uint16_t PRINT_PERIOD_MS = 500;

// ====================== FUNCIONES AUXILIARES ======================

void print_encoder_state() {
    Serial.print("Steps L/R: ");
    Serial.print(wheels_data.steps_left);
    Serial.print(" / ");
    Serial.print(wheels_data.steps_right);
    Serial.print(" | wL: ");
    Serial.print(wheels_data.wL_measured, 3);
    Serial.print(" rad/s | wR: ");
    Serial.println(wheels_data.wR_measured, 3);
}

void ejecutar_fase_con_obstaculo(const char* msg, float dutyL, float dutyR, uint32_t duracion_ms) {
    Serial.println(msg);
    
    uint32_t tiempo_acumulado = 0;
    uint32_t t_anterior = millis();
    uint32_t t_print = millis();

    bool en_movimiento = false;

    while (tiempo_acumulado < duracion_ms) {
        // Leer distancia
        uint8_t distancia = DistanceSensors::read_distance(US_MID_TRIG_PIN, US_MID_ECHO_PIN);
        distance_data.us_mid_distance = distancia;

        // Condición de obstáculo
        bool obstaculo = distancia < OBSTACLE_THRESHOLD_CM;
        distance_data.obstacle_detected = obstaculo;

        if (obstaculo) {
            if (en_movimiento) {
                Serial.print("Obstáculo detectado a ");
                Serial.print(distancia);
                Serial.println(" cm — deteniendo motores");
                MotorController::set_motors_duty(
                    0.0f, 0.0f, wheels_data.duty_left, wheels_data.duty_right, system_states.motors);
                en_movimiento = false;
            }
        } else {
            if (!en_movimiento) {
                Serial.println("Obstáculo despejado — reanudando movimiento");
                MotorController::set_motors_duty(
                    dutyL, dutyR, wheels_data.duty_left, wheels_data.duty_right, system_states.motors);
                t_anterior = millis(); // reinicia referencia temporal tras pausa
                en_movimiento = true;
            }

            // Avance normal
            uint32_t t_actual = millis();
            tiempo_acumulado += (t_actual - t_anterior);
            t_anterior = t_actual;
        }
        EncoderReader::update_encoder_data(wheels_data, system_states.encoders);
        if (millis() - t_print >= PRINT_PERIOD_MS) {
            print_encoder_state();
            t_print += PRINT_PERIOD_MS;
        }
        delay(10);
    }
    // Detener al final de la fase
    MotorController::set_motors_duty(
        0.0f, 0.0f, wheels_data.duty_left, wheels_data.duty_right, system_states.motors);
}


// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: duty + encoder + obstacle");

    // Inicialización de motor y encoder
    MotorController::init(
        system_states.motors, wheels_data.duty_left, wheels_data.duty_right);
    MotorController::set_motors_mode(
        MOTOR_ACTIVE, system_states.motors, wheels_data.duty_left, wheels_data.duty_right);
    DistanceSensors::init_system(system_states.distance, distance_data);
    DistanceSensors::set_state(ACTIVE, system_states.distance);  
    EncoderReader::init(wheels_data, system_states.encoders);
    EncoderReader::resume(system_states.encoders);
    
    Serial.println();
    
    // Secuencia de prueba
    ejecutar_fase_con_obstaculo("Avanzando recto (50%)", 0.5f, 0.5f, 5000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Girando en el lugar (izq, 60%)", 0.0f, 0.6f, 2000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Avanzando recto (50%)", 0.5f, 0.5f, 5000);
    delay(1000);
    Serial.println("Secuencia completada. Motores en IDLE.");
    MotorController::set_motors_mode(MOTOR_IDLE, system_states.motors, wheels_data.duty_left, wheels_data.duty_right);

}

void loop() {
    // Nada
}
