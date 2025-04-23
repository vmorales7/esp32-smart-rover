#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensors_full.cpp"

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

volatile WheelsData wheels_data = {0};

volatile DistanceSensorData distance_data = {
    .obstacle_detected = false,
    .left_distance = US_MAX_DISTANCE_CM,
    .right_distance = US_MAX_DISTANCE_CM
};


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
        uint8_t distancia = DistanceSensors::us_read_distance(
            US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN, &system_states.distance
        );
        distance_data.left_distance = distancia;

        // Condición de obstáculo
        bool obstaculo = distancia < OBSTACLE_THRESHOLD_CM;
        distance_data.obstacle_detected = obstaculo;

        if (obstaculo) {
            if (en_movimiento) {
                Serial.print("Obstáculo detectado a ");
                Serial.print(distancia);
                Serial.println(" cm — deteniendo motores");
                MotorController::set_motor_mode(MOTOR_IDLE, &system_states.motor_operation,
                                                &wheels_data.duty_left, &wheels_data.duty_right);
                en_movimiento = false;
            }
        } else {
            if (!en_movimiento) {
                Serial.println("Obstáculo despejado — reanudando movimiento");
                MotorController::set_motor_mode(MOTOR_ACTIVE, &system_states.motor_operation,
                                                &wheels_data.duty_left, &wheels_data.duty_right);
                MotorController::set_motor_duty(
                    dutyL, dutyR,
                    &wheels_data.duty_left, &wheels_data.duty_right,
                    &system_states.motor_operation
                );
                t_anterior = millis(); // reinicia referencia temporal tras pausa
                en_movimiento = true;
            }

            // Avance normal
            EncoderReader::update_encoder_data(
                &system_states.encoder,
                &wheels_data.steps_left, &wheels_data.steps_right,
                &wheels_data.wL_measured, &wheels_data.wR_measured
            );

            uint32_t t_actual = millis();
            tiempo_acumulado += (t_actual - t_anterior);
            t_anterior = t_actual;
        }

        if (millis() - t_print >= 500) {
            print_encoder_state();
            t_print += 500;
        }

        delay(10);
    }

    // Detener al final de la fase
    MotorController::set_motor_mode(MOTOR_IDLE, &system_states.motor_operation,
                                    &wheels_data.duty_left, &wheels_data.duty_right);
}


// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: duty + encoder + obstacle");

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
    MotorController::set_motor_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );
    EncoderReader::resume(&system_states.encoder);
    Serial.println();
    
    // Secuencia de prueba
    ejecutar_fase_con_obstaculo("Avanzando recto (70%)", 0.7f, 0.7f, 4000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Girando en el lugar (izq, 30%)", -0.3f, 0.3f, 2000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Avanzando recto (50%)", 0.5f, 0.5f, 4000);
    delay(1000);
    Serial.println("Secuencia completada. Motores en IDLE.");
}

void loop() {
    // Nada
}
