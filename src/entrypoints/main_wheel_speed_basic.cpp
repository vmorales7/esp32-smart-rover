#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/position_controller.h"
#warning "Compilando main_wheel_speed_basic.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile KinematicState kinematic_data = {0};

// ====================== FUNCIONES AUXILIARES ======================

void print_wheel_speeds() {
    Serial.print("wL_measured: ");
    Serial.print(wheels_data.wL_measured, 1);
    Serial.print(" rad/s | wR_measured: ");
    Serial.print(wheels_data.wR_measured, 1);
    Serial.print(" rad/s");

    Serial.print(" | dutyL: ");
    Serial.print(wheels_data.duty_left, 2);
    Serial.print(" | dutyR: ");
    Serial.println(wheels_data.duty_right, 2);
}


void set_speed_reference(float w_ref) {
    Serial.print("Nueva referencia de velocidad: ");
    Serial.print(w_ref, 1);
    Serial.println(" rad/s");

    PositionController::set_wheel_speed_ref(
        w_ref, w_ref,
        &wheels_data.wL_ref,
        &wheels_data.wR_ref,
        &system_states.position_controller
    );
}

// ====================== SETUP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: Motor PI Control (ajuste Kp/Ki/Kw)");

    // Inicializar módulos
    PositionController::init(
        &kinematic_data.v_ref, &kinematic_data.w_ref, &wheels_data.wL_ref, &wheels_data.wR_ref, &system_states.position_controller
    );
    PositionController::set_control_mode(SPEED_REF_MANUAL, &system_states.position_controller);
    MotorController::init(
        &system_states.motor_operation,
        &wheels_data.duty_left, &wheels_data.duty_right
    );
    MotorController::set_motor_mode(
        MOTOR_AUTO,
        &system_states.motor_operation,
        &wheels_data.duty_left, &wheels_data.duty_right
    );
    EncoderReader::init(
        &system_states.encoder,
        &wheels_data.steps_left, &wheels_data.steps_right,
        &wheels_data.wL_measured, &wheels_data.wR_measured
    );
    EncoderReader::resume(&system_states.encoder);
}

// ====================== LOOP ======================

void loop() {
    static unsigned long t0 = millis();
    static unsigned long t_last_print = 0;
    static unsigned long t_last_control = 0;
    static int fase = 0;

    unsigned long t = millis();

    // Cambio de fase: cada 5 segundos
    if ((t - t0) >= 5000) {
        t0 = t;
        fase = (fase + 1) % 2;
        float ref = (fase == 0) ? 0.5f * WM_NOM : 0.2f * WM_NOM;
        set_speed_reference(ref);
    }

    // Actualizar lectura de encoder cada 10ms
    if ((t - t_last_control) >= 10) {
        t_last_control = t;
        EncoderReader::update_encoder_data(
            &system_states.encoder,
            &wheels_data.steps_left, &wheels_data.steps_right,
            &wheels_data.wL_measured, &wheels_data.wR_measured
        );

        MotorController::update_motor_control(
            &wheels_data.wL_ref, &wheels_data.wR_ref,
            &wheels_data.wL_measured, &wheels_data.wR_measured,
            &wheels_data.duty_left, &wheels_data.duty_right,
            &system_states.motor_operation
        );
    }

    // Print de estado cada 500ms
    if ((t - t_last_print) >= 500) {
        t_last_print = t;
        print_wheel_speeds();
    }
}
