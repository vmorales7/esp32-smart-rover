#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/pose_estimator.h"
#warning "Compilando main_pose_basic.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile KinematicState kinematic_state = {0};
volatile PoseData pose_data = {0};

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Pose Debug — Duty fijo 50%");

    // Inicializar posición
    PoseEstimator::set_state(ACTIVE,&system_states.pose_estimator);
    PoseEstimator::reset_pose_and_steps(
        &kinematic_state.x, &kinematic_state.y, &kinematic_state.theta,
        &wheels_data.steps_left, &wheels_data.steps_left
    );

    // Inicializar encoder
    EncoderReader::init(&system_states.encoder,
        &wheels_data.steps_left, &wheels_data.steps_right,
        &wheels_data.wL_measured, &wheels_data.wR_measured);
    EncoderReader::resume(&system_states.encoder);

    // Echar a andar motores
    MotorController::init(&system_states.motor_operation,
                          &wheels_data.duty_left, &wheels_data.duty_right);
    MotorController::set_motor_mode(MOTOR_ACTIVE,
                                    &system_states.motor_operation,
                                    &wheels_data.duty_left, &wheels_data.duty_right);
    MotorController::set_motor_duty(0.5f, 0.5f,
                                    &wheels_data.duty_left, &wheels_data.duty_right,
                                    &system_states.motor_operation);
}

// ====================== LOOP ======================
void loop() {
    static unsigned long last_control = 0;
    static unsigned long last_print = 0;

    unsigned long now = millis();

    // Actualiza encoder y pose cada 10 ms
    if (now - last_control >= 10) {
        last_control = now;

        EncoderReader::update_encoder_data(
            &system_states.encoder,
            &wheels_data.steps_left, &wheels_data.steps_right,
            &wheels_data.wL_measured, &wheels_data.wR_measured
        );

        PoseData pose = PoseEstimator::estimate_pose_from_encoder(
            &kinematic_state.x, &kinematic_state.y, &kinematic_state.theta,
            &kinematic_state.v, &kinematic_state.w,
            &wheels_data.wL_measured, &wheels_data.wR_measured,
            &wheels_data.steps_left, &wheels_data.steps_right
        );
        PoseEstimator::update_pose(
            &pose,
            &kinematic_state.x, &kinematic_state.y, &kinematic_state.theta,
            &kinematic_state.v, &kinematic_state.w,
            &system_states.pose_estimator
        );
    }

    // Imprime cada 500 ms
    if (now - last_print >= 500) {
        last_print = now;

        Serial.print("Pose => x: ");
        Serial.print(kinematic_state.x, 3);
        Serial.print(" | y: ");
        Serial.print(kinematic_state.y, 3);
        Serial.print(" | θ: ");
        Serial.println(kinematic_state.theta, 3);

        Serial.print("Vel  => v: ");
        Serial.print(kinematic_state.v, 3);
        Serial.print(" m/s | w: ");
        Serial.print(kinematic_state.w, 3);
        Serial.println(" rad/s");
    }
}
