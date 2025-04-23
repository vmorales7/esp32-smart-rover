#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensor.cpp"

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

// ====================== CONFIG ======================

constexpr float DUTY_FORWARD = 0.5f;
constexpr uint16_t LOOP_DELAY_MS = 200;

// ====================== SETUP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: Movimiento con evasión básica de obstáculos");

    // Inicialización de motores y sensores
    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    // Activar motores
    MotorController::set_motor_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    // Inicializar y activar sensor de distancia
    DistanceSensors::init(&system_states.distance);
    DistanceSensors::set_distance_sensor_state(ACTIVE, &system_states.distance);

    // Comenzar movimiento recto
    MotorController::set_motor_duty(
        DUTY_FORWARD, DUTY_FORWARD,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
}

// ====================== LOOP ======================

void loop() {
    // Leer sensor ultrasónico izquierdo
    uint8_t distancia = DistanceSensors::us_read_distance(
        US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN, &system_states.distance
    );
    distance_data.left_distance = distancia;

    // Verificar obstáculo
    if (distancia < OBSTACLE_THRESHOLD_CM) {
        if (!distance_data.obstacle_detected) {
            distance_data.obstacle_detected = true;
            Serial.print("Obstáculo detectado a ");
            Serial.print(distancia);
            Serial.println(" cm — deteniendo motores");

            MotorController::set_motor_mode(
                MOTOR_IDLE,
                &system_states.motor_operation,
                &wheels_data.duty_left,
                &wheels_data.duty_right
            );
        }
    } else {
        if (distance_data.obstacle_detected) {
            distance_data.obstacle_detected = false;
            Serial.println("Obstáculo despejado — reanudando movimiento");

            MotorController::set_motor_mode(
                MOTOR_ACTIVE,
                &system_states.motor_operation,
                &wheels_data.duty_left,
                &wheels_data.duty_right
            );
            MotorController::set_motor_duty(
                DUTY_FORWARD, DUTY_FORWARD,
                &wheels_data.duty_left, &wheels_data.duty_right,
                &system_states.motor_operation
            );
        }
    }

    delay(LOOP_DELAY_MS);
}
