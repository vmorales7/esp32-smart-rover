#include <Arduino.h>
#include "project_config.h"
#include "motor_drive/motor_controller.h"

// ====================== VARIABLES GLOBALES ======================

volatile SystemStates system_states = {
    .motor_operation      = MOTOR_IDLE,
    .encoder              = INACTIVE,
    .imu                  = INACTIVE,
    .distance             = INACTIVE,
    .pose_estimator       = INACTIVE,
    .position_controller  = INACTIVE,
    .evade_controller     = INACTIVE
};

volatile WheelsData wheels_data = {
    .steps_left = 0,
    .steps_right = 0,
    .w_measured_left = 0.0f,
    .w_measured_right = 0.0f,
    .w_ref_left = 0.0f,
    .w_ref_right = 0.0f,
    .duty_left = 0.0f,
    .duty_right = 0.0f
};

volatile KinematicState kinematic_data = {
    .x = 0.0f, .y = 0.0f, .theta = 0.0f,
    .x_d = 0.0f, .y_d = 0.0f, .theta_d = 0.0f,
    .v = 0.0f, .w = 0.0f,
    .v_ref = 0.0f, .w_ref = 0.0f
};

volatile DistanceSensorData sensor_data = {
    .obstacle_detected = false,
    .left_distance = 0,
    .right_distance = 0
};

GlobalContext global_ctx = {
    .systems_ptr = &system_states,
    .kinematic_ptr = &kinematic_data,
    .wheels_ptr = &wheels_data
};

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);

    // Inicializar controlador de motor
    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    // Activar subsistema de motores
    MotorController::set_motor_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    Serial.println("Test: Duty Cycle Motor - Iniciando secuencia...");

    // Avance recto 50% por 3 segundos
    Serial.println("Avanzando recto (50%)");
    MotorController::set_motor_duty(
        0.5f, 0.5f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    delay(3000);

    // Giro en el lugar hacia la izquierda por 2 segundos
    Serial.println("Girando en el lugar (izquierda, 30%)");
    MotorController::set_motor_duty(
        -0.3f, 0.3f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    delay(2000);

    // Nuevamente avance recto 50% por 3 segundos
    Serial.println("Avanzando recto nuevamente (50%)");
    MotorController::set_motor_duty(
        0.5f, 0.5f,
        &wheels_data.duty_left, &wheels_data.duty_right,
        &system_states.motor_operation
    );
    delay(3000);

    // Detener motores
    MotorController::set_motor_mode(
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
