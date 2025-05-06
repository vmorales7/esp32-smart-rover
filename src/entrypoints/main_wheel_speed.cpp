#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#include "position_system/position_controller.h"
#warning "Compilando main_distance_speed_ref.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile DistanceSensorData distance_data = {
    .obstacle_detected = false,
    .left_distance = US_MAX_DISTANCE_CM,
    .right_distance = US_MAX_DISTANCE_CM
};
volatile KinematicState kinematic_data = {0};
volatile uint8_t control_mode = 0;
constexpr uint32_t PRINT_INTERVAL_MS = 250;

// ====================== FUNCIONES AUXILIARES ======================

void print_encoder_state() {
    Serial.print("wL ");
    Serial.print(wheels_data.wL_measured, 2);
    Serial.print("  dutyL ");
    Serial.print(wheels_data.duty_left, 2);
    Serial.print("  |  wR ");
    Serial.print(wheels_data.wR_measured, 2);
    Serial.print("  dutyR ");
    Serial.println(wheels_data.duty_right, 2);
}

void ejecutar_fase_con_obstaculo_speed(const char* msg, float wrefL, float wrefR, uint32_t duracion_ms) {
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
                MotorController::set_motors_mode(MOTOR_IDLE, &system_states.motor_operation,
                                                 &wheels_data.duty_left, &wheels_data.duty_right);
                en_movimiento = false;
            }
        } else {
            if (!en_movimiento) {
                Serial.println("Obstáculo despejado — reanudando movimiento");
                MotorController::set_motors_mode(MOTOR_AUTO, &system_states.motor_operation,
                                                 &wheels_data.duty_left, &wheels_data.duty_right);
                PositionController::set_wheel_speed_ref(
                    wrefL, wrefR,
                    &wheels_data.wL_ref, &wheels_data.wR_ref,
                    &control_mode
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

            MotorController::update_motors_control(
                &wheels_data.wL_ref, &wheels_data.wR_ref,
                &wheels_data.wL_measured, &wheels_data.wR_measured,
                &wheels_data.duty_left, &wheels_data.duty_right,
                &system_states.motor_operation
            );

            uint32_t t_actual = millis();
            tiempo_acumulado += (t_actual - t_anterior);
            t_anterior = t_actual;
        }

        if (millis() - t_print >= PRINT_INTERVAL_MS) {
            print_encoder_state();
            t_print += PRINT_INTERVAL_MS;
        }

        delay(WHEEL_CONTROL_PERIOD_MS);
    }

    // Detener al final de la fase
    MotorController::set_motors_mode(MOTOR_IDLE, &system_states.motor_operation,
                                     &wheels_data.duty_left, &wheels_data.duty_right);
}

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: wheel speed ref + encoder + obstacle");

    // Inicialización
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
    EncoderReader::resume(&system_states.encoder);

    DistanceSensors::init(&system_states.distance);
    DistanceSensors::set_state(ACTIVE, &system_states.distance);

    PositionController::init(
        &kinematic_data.v_ref,
        &kinematic_data.w_ref,
        &wheels_data.wL_ref,
        &wheels_data.wR_ref,
        &control_mode
    );
    PositionController::set_control_mode(SPEED_REF_MANUAL, &control_mode);

    // Prueba
    ejecutar_fase_con_obstaculo_speed("Avanzando recto (0.5 wn)", 0.5f * WM_NOM, 0.5f * WM_NOM, 10000);
    delay(1000);
    ejecutar_fase_con_obstaculo_speed("Girando en el lugar (der, 0.4 wn)", -0.4f * WM_NOM, 0.4f * WM_NOM, 5000);
    delay(1000);
    ejecutar_fase_con_obstaculo_speed("Avanzando recto (0.5 wn)", 0.5f * WM_NOM, 0.5f * WM_NOM, 10000);
    delay(1000);
    Serial.println("Secuencia completada. Motores en IDLE.");
}

void loop() {
    // Nada
}
