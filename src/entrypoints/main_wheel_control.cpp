#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "position_system/position_controller.h"
#warning "Compilando main_motor_wheel_control.cpp"

// ====================== DEFINICIONES ======================

constexpr float WHEEL_W_NOM = WM_NOM;  // Velocidad nominal para pruebas

// ====================== VARIABLES GLOBALES ======================

volatile SystemStates system_states = {
    .motor_operation      = MOTOR_IDLE,
    .encoder              = INACTIVE,
    .imu                  = INACTIVE,
    .distance             = INACTIVE,
    .pose_estimator       = INACTIVE,
    .position_controller  = SPEED_REF_MANUAL,  // Activamos el módulo de referencia manual
    .evade_controller     = INACTIVE
};

volatile WheelsData wheels_data = {0};

// ====================== FUNCIONES AUXILIARES ======================

void print_wheel_speeds() {
    Serial.print("wL_measured: ");
    Serial.print(wheels_data.w_measured_left, 1);
    Serial.print(" rad/s | wR_measured: ");
    Serial.println(wheels_data.w_measured_right, 1);
}

void set_speed_reference(float w_ref) {
    Serial.print("Nueva referencia de velocidad: ");
    Serial.print(w_ref, 1);
    Serial.println(" rad/s");

    PositionController::set_wheel_speed_ref(
        w_ref, w_ref,
        &wheels_data.w_ref_left,
        &wheels_data.w_ref_right,
        &system_states.position_controller
    );
}

// ====================== SETUP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: Motor PI Control (ajuste Kp/Ki/Kw)");

    // Inicializar módulos
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
        &wheels_data.w_measured_left, &wheels_data.w_measured_right
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
        float ref = (fase == 0) ? 0.5f * WHEEL_W_NOM : 0.2f * WHEEL_W_NOM;
        set_speed_reference(ref);
    }

    // Actualizar lectura de encoder cada 10ms
    if ((t - t_last_control) >= 10) {
        t_last_control = t;
        EncoderReader::update_encoder_data(
            &system_states.encoder,
            &wheels_data.steps_left, &wheels_data.steps_right,
            &wheels_data.w_measured_left, &wheels_data.w_measured_right
        );

        MotorController::update_motor_control(
            &wheels_data.w_ref_left, &wheels_data.w_ref_right,
            &wheels_data.w_measured_left, &wheels_data.w_measured_right,
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
