#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#warning "Compilando main_motor_duty_encoder.cpp"

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
    .wheels_ptr = &wheels_data,
    .distance_ptr = &sensor_data
};

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);

    // Inicializar motores y encoders
    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    EncoderReader::init(
        &system_states.encoder,
        &wheels_data.steps_left,
        &wheels_data.steps_right,
        &wheels_data.w_measured_left,
        &wheels_data.w_measured_right
    );

    MotorController::set_motor_mode(
        MOTOR_ACTIVE,
        &system_states.motor_operation,
        &wheels_data.duty_left,
        &wheels_data.duty_right
    );

    EncoderReader::resume(&system_states.encoder);

    Serial.println("Motores activos - Encoders inicializados");

    MotorController::set_motor_duty(
        0.25f, 0.25f,
        &wheels_data.duty_left,
        &wheels_data.duty_right,
        &system_states.motor_operation
    );

    Serial.println("Avance a 0.25 iniciado...");
}

void loop() {
    // Actualización de encoders cada 1000 ms (manual, no RTOS)
    EncoderReader::update_encoder_data(
        &system_states.encoder,
        &wheels_data.steps_left,
        &wheels_data.steps_right,
        &wheels_data.w_measured_left,
        &wheels_data.w_measured_right
    );

    Serial.print("wL [rad/s]: ");
    Serial.print(wheels_data.w_measured_left);
    Serial.print("\t wR [rad/s]: ");
    Serial.println(wheels_data.w_measured_right);

    delay(1000);
} 
