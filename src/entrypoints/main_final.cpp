#include <Arduino.h>
#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/position_controller.h"
#include "position_system/pose_estimator.h"
#warning "Compilando main_final.cpp"


void setup() {
    //Serial.begin(115200);

    // Inicialización de sistemas
    MotorController::init(
        &(system_states.motor_operation), 
        &(wheels_data.duty_left), &(wheels_data.duty_right)
    );
    EncoderReader::init(
        &(system_states.encoder), 
        &(wheels_data.steps_left), &(wheels_data.steps_right),
        &(wheels_data.w_measured_left), &(wheels_data.w_measured_right)
    );

    // Crear tarea de control de ruedas
    TaskHandle_t WheelControlTaskHandle = nullptr;
    xTaskCreatePinnedToCore(
        MotorController::Task_WheelControl, // Task function
        "WheelControl",                     // Name
        3000,                               // Stack size
        &global_ctx,                        // Parameters
        10,                                 // Priority
        &WheelControlTaskHandle,            // task handle
        1                                   // Pin to core 1
    );

    // Crear tarea de lectura de encoders
    TaskHandle_t EncoderTaskHandle = nullptr;
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderReader",
        3000, &global_ctx, 8, &EncoderTaskHandle, 1);

    // Crear tarea de estimación de pose
    TaskHandle_t PoseEstimatorTaskHandle = nullptr;
    xTaskCreatePinnedToCore(PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 
        3000, &global_ctx, 7, &PoseEstimatorTaskHandle, 1 );

}

void loop() {
    // Nada aquí, todo se hace en las tareas
}


/* -------- Inicialización de variables globales ---------- */
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
