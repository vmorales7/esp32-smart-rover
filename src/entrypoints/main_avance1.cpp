#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_avance1.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates systems;
volatile WheelsData wheels = {0};
volatile DistanceSensorData distances = {0};
constexpr uint16_t PRINT_PERIOD_MS = 500;

// ====================== FUNCIONES AUXILIARES ======================

void print_encoder_state() {
    Serial.print("Steps L/R: ");
    Serial.print(wheels.steps_L);
    Serial.print(" / ");
    Serial.print(wheels.steps_R);
    Serial.print(" | wL: ");
    Serial.print(wheels.w_L, 3);
    Serial.print(" rad/s | wR: ");
    Serial.println(wheels.w_R, 3);
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
        distances.mid_dist = distancia;

        // Condición de obstáculo
        bool obstaculo = distancia < OBSTACLE_THRESHOLD_CM;
        distances.obstacle_detected = obstaculo;

        if (obstaculo) {
            if (en_movimiento) {
                Serial.print("Obstáculo detectado a ");
                Serial.print(distancia);
                Serial.println(" cm — deteniendo motores");
                MotorController::set_motors_duty(0.0f, 0.0f, wheels.duty_L, wheels.duty_R, systems.motors);
                en_movimiento = false;
            }
        } else {
            if (!en_movimiento) {
                Serial.println("Obstáculo despejado — reanudando movimiento");
                MotorController::set_motors_duty(dutyL, dutyR, wheels.duty_L, wheels.duty_R, systems.motors);
                t_anterior = millis(); // reinicia referencia temporal tras pausa
                en_movimiento = true;
            }

            // Avance normal
            uint32_t t_actual = millis();
            tiempo_acumulado += (t_actual - t_anterior);
            t_anterior = t_actual;
        }
        EncoderReader::update_encoder_data(wheels.steps_L, wheels.steps_R, wheels.w_L, wheels.w_R, systems.encoders);
        if (millis() - t_print >= PRINT_PERIOD_MS) {
            print_encoder_state();
            t_print += PRINT_PERIOD_MS;
        }
        delay(10);
    }
    // Detener al final de la fase
    MotorController::set_motors_duty(0.0f, 0.0f, wheels.duty_L, wheels.duty_R, systems.motors);
}


// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: duty + encoder + obstacle");

    // Inicialización de motor y encoder
    MotorController::init(systems.motors, wheels.duty_L, wheels.duty_R);
    MotorController::set_motors_mode(MotorMode::ACTIVE, systems.motors, wheels.duty_L, wheels.duty_R);
    DistanceSensors::init(
        distances.left_dist, distances.left_obst, distances.mid_dist, distances.mid_obst, 
        distances.right_dist, distances.right_obst, distances.obstacle_detected, systems.distance);
    DistanceSensors::set_state(ACTIVE, systems.distance,
        distances.left_dist, distances.left_obst, distances.mid_dist, distances.mid_obst, 
        distances.right_dist, distances.right_obst, distances.obstacle_detected 
    );  
    EncoderReader::init(wheels.steps_L, wheels.steps_R, wheels.w_L, wheels.w_R, systems.encoders);
    EncoderReader::resume(systems.encoders);
    
    Serial.println();
    
    // Secuencia de prueba
    ejecutar_fase_con_obstaculo("Avanzando recto (50%)", 0.5f, 0.5f, 5000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Girando en el lugar (izq, 60%)", 0.0f, 0.6f, 2000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Avanzando recto (50%)", 0.5f, 0.5f, 5000);
    delay(1000);
    Serial.println("Secuencia completada. Motores en IDLE.");
    MotorController::set_motors_mode(MotorMode::IDLE, systems.motors, wheels.duty_L, wheels.duty_R);
}

void loop() {
    // Nada
}
