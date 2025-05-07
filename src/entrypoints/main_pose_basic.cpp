#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

#warning "Compilando main_pose_basic.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile KinematicState kinematic_state = {0};
volatile PoseData pose_data = {0};

GlobalContext ctx = {
    .systems_ptr   = &system_states,
    .kinematic_ptr = &kinematic_state,
    .wheels_ptr    = &wheels_data,
    .distance_ptr  = nullptr
};

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Pose Debug — Duty fijo 50% (RTOS)");

    // Inicializar posición
    PoseEstimator::set_state(ACTIVE, &system_states.pose_estimator);
    PoseEstimator::reset_pose_and_steps(
        &kinematic_state.x, &kinematic_state.y, &kinematic_state.theta,
        &wheels_data.steps_left, &wheels_data.steps_right
    );

    // Inicializar encoder
    EncoderReader::init(&system_states.encoder,
        &wheels_data.steps_left, &wheels_data.steps_right,
        &wheels_data.wL_measured, &wheels_data.wR_measured);
    EncoderReader::resume(&system_states.encoder);

    // Inicializar motores
    MotorController::init(&system_states.motor_operation,
                          &wheels_data.duty_left, &wheels_data.duty_right);
    MotorController::set_motors_mode(MOTOR_AUTO,
                                     &system_states.motor_operation,
                                     &wheels_data.duty_left, &wheels_data.duty_right);

    // Generar referencia de velocidad
    PositionController::init(
        &system_states.position_controller,
        &wheels_data.wL_ref, &wheels_data.wR_ref
    );
    PositionController::set_control_mode(SPEED_REF_MANUAL, &system_states.position_controller);
    PositionController::set_wheel_speed_ref(
        10.0f, 10.0f,
        &wheels_data.wL_ref, &wheels_data.wR_ref,
        &system_states.position_controller
    );

    // Crear tareas RTOS
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 2048, &ctx, 1, nullptr, 1);
}


// ====================== LOOP ======================
void loop() {
    static unsigned long last_print = 0;
    unsigned long now = millis();

    if (now - last_print >= 500) {
        last_print = now;

        Serial.print("Pose => x: ");
        Serial.print(kinematic_state.x, 2);
        Serial.print(" | y: ");
        Serial.print(kinematic_state.y, 2);
        Serial.print(" | θ: ");
        Serial.print(kinematic_state.theta, 2);
        Serial.print(" || Vel => v: ");
        Serial.print(kinematic_state.v, 1);
        Serial.print(" | w: ");
        Serial.println(kinematic_state.w, 1);
    }
}
