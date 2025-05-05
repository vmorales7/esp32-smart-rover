#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#include "position_system/position_controller.h"
#warning "Compilando main_wheel_speed.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile KinematicState kinematic_data = {0};
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
    Serial.print(wheels_data.wR_measured, 3);
    Serial.print(" rad/s | dutyL: ");
    Serial.print(wheels_data.duty_left, 2);
    Serial.print(" | dutyR: ");
    Serial.println(wheels_data.duty_right, 2);
}

void ejecutar_fase_con_obstaculo(const char* msg, float wL_ref, float wR_ref, uint32_t duracion_ms) {
    Serial.println(msg);

    uint32_t tiempo_acumulado = 0;
    uint32_t t_anterior = millis();
    uint32_t t_print = millis();

    bool en_movimiento = false;

    while (tiempo_acumulado < duracion_ms) {
        // Leer sensor
        uint8_t distancia = DistanceSensors::us_read_distance(
            US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN, &system_states.distance
        );
        distance_data.left_distance = distancia;

        bool obstaculo = distancia < OBSTACLE_THRESHOLD_CM;
        distance_data.obstacle_detected = obstaculo;

        if (obstaculo) {
            if (en_movimiento) {
                Serial.print("Obstáculo detectado a ");
                Serial.print(distancia);
                Serial.println(" cm — deteniendo referencia");

                PositionController::set_wheel_speed_ref(
                    0.0f, 0.0f,
                    &wheels_data.wL_ref, &wheels_data.wR_ref,
                    &system_states.position_controller
                );
                en_movimiento = false;
            }
        } else {
            if (!en_movimiento) {
                Serial.println("Obstáculo despejado — reanudando movimiento");

                PositionController::set_wheel_speed_ref(
                    wL_ref, wR_ref,
                    &wheels_data.wL_ref, &wheels_data.wR_ref,
                    &system_states.position_controller
                );
                t_anterior = millis();
                en_movimiento = true;
            }

            // Actualización de encoders y control
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

        if (millis() - t_print >= 500) {
            print_encoder_state();
            t_print += 500;
        }

        delay(10);
    }

    // Detener al final de la fase
    PositionController::set_wheel_speed_ref(
        0.0f, 0.0f,
        &wheels_data.wL_ref, &wheels_data.wR_ref,
        &system_states.position_controller
    );
    MotorController::update_motors_control(
        &wheels_data.wL_ref, &wheels_data.wR_ref,
        &wheels_data.wL_measured, &wheels_data.wR_measured,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: velocidad de ruedas + obstáculo");

    PositionController::init(
        &kinematic_data.v_ref, &kinematic_data.w_ref, &wheels_data.wL_ref, &wheels_data.wR_ref, &system_states.position_controller
    );
    PositionController::set_control_mode(SPEED_REF_MANUAL, &system_states.position_controller);

    MotorController::init(&system_states.motor_operation,
                          &wheels_data.duty_left, &wheels_data.duty_right);
    MotorController::set_motors_mode(MOTOR_AUTO, &system_states.motor_operation,
                                    &wheels_data.duty_left, &wheels_data.duty_right);

    DistanceSensors::init(&system_states.distance);
    DistanceSensors::set_state(ACTIVE, &system_states.distance);

    EncoderReader::init(&system_states.encoder,
        &wheels_data.steps_left, &wheels_data.steps_right,
        &wheels_data.wL_measured, &wheels_data.wR_measured);
    EncoderReader::resume(&system_states.encoder);
}

// ====================== LOOP ======================
void loop() {
    ejecutar_fase_con_obstaculo("Avanzando recto (70%)", 0.7f * WM_NOM, 0.7f * WM_NOM, 4000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Girando en el lugar (30%)", -0.3f * WM_NOM, 0.3f * WM_NOM, 2000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Avanzando recto (50%)", 0.5f * WM_NOM, 0.5f * WM_NOM, 4000);
    delay(1000);
    Serial.println("Secuencia completada.");
}
