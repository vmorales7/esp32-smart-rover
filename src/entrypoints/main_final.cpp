#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
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
        &(wheels_data.wL_measured), &(wheels_data.wR_measured)
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
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile KinematicState kinematic_data = {0};
volatile DistanceSensorData distance_data = {
    .obstacle_detected = false,
    .left_distance = US_MAX_DISTANCE_CM,
    .right_distance = US_MAX_DISTANCE_CM
};
GlobalContext global_ctx = {
    .systems_ptr = &system_states,
    .kinematic_ptr = &kinematic_data,
    .wheels_ptr = &wheels_data,
    .distance_ptr = &distance_data
};
