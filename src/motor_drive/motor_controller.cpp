#include "motor_controller.h"

/* ---------------- Funciones auxiliares generales ------------------*/

DutyProfile init_duty_profile(const float rawDuty) {
    DutyProfile duty_profile = {0};
    duty_profile.duty_val = rawDuty;
    duty_profile.abs_duty = fabsf(rawDuty);
    const bool fw = (rawDuty >= 0.0f);
    duty_profile.forward = fw;
    duty_profile.duty_dir = fw ? 1 : -1;
    return duty_profile;
}

void check_max_duty(DutyProfile& duty) {
    if (duty.abs_duty > MAX_DUTY) {
        duty.abs_duty = MAX_DUTY; // Limitar al duty máximo (100%)
        duty.duty_val = MAX_DUTY * duty.duty_dir;   
    }
}

void check_inversion_duty(DutyProfile& duty, const float w_measured) {
    // Caracterizar la velocidad
    const float abs_wm = fabsf(w_measured);
    const int8_t sign_wm = (w_measured >= 0) ? 1 : -1;
    bool break_flag = false;

    // ¿Intenta invertir respecto al giro actual yendo muy rápido?
    if (duty.duty_dir != sign_wm && abs_wm > WHEEL_INVERT_THRESHOLD) { 
        // Detener y evaluar frenado
        duty.duty_val = 0.0f;
        duty.abs_duty = 0.0f;
        break_flag = (abs_wm < WHEEL_BRAKE_THRESHOLD); // Suficientemente lento para aplicar freno?
    }
    duty.break_flag = break_flag;
}

void check_min_duty(DutyProfile& duty) {
    if (duty.abs_duty < ZERO_DUTY_THRESHOLD) {
        duty.abs_duty = 0.0f;                     // Duty demasiado bajo se toma como cero
        duty.duty_val = 0.0f;
    }
}

void check_starting_duty(DutyProfile& duty, const float w_measured) {
    // ¿Está partiendo el vehículo desde una posición de detención? -> El duty debe ser mayor al mínimo
    const float abs_wm = fabsf(w_measured);
    if (duty.abs_duty > ZERO_DUTY_THRESHOLD && abs_wm < WHEEL_STOP_THRESHOLD) {
        const float new_abs =  fmaxf(duty.abs_duty, MIN_START_DUTY);
        duty.abs_duty = new_abs;
        duty.duty_val = new_abs * duty.duty_dir;
    }
}


/* ------------------ Funciones de control PI --------------------*/

WheelSpeedPID::WheelSpeedPID(float kp, float ki, float kw)
    : Kp(kp), Ki(ki), Kw(kw), integral(0), lastDuty(0), lastTime(0) {}

    DutyProfile WheelSpeedPID::compute(const float setpoint, const float measured) {
        // Tiempo actual y delta tiempo
        float now = millis() * MS_TO_S;
        float dt = now - lastTime;
    
        // Protección: solo actualizar cada cierto tiempo
        if (dt < 0.001) return init_duty_profile(lastDuty);
    
        // Control PI
        float error = setpoint - measured;
        float rawDuty = Kp * error + Ki * integral;

        // Se limita el duty según límites máximos y evitando inversión
        DutyProfile duty_data = init_duty_profile(rawDuty);
        check_max_duty(duty_data);

        // Anti-windup por back-calculation + clamping
        const float dutyError = rawDuty - duty_data.duty_val;
        const float anti_wp = Kw * dutyError;
        const float integral_leak = LAMBDA_WHEEL * integral;
        integral += (error - anti_wp - integral_leak) * dt;
        integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX); // Clamping del integrador

        // Correciones al duty
        check_min_duty(duty_data);
        // check_starting_duty(duty_data, measured);
        check_inversion_duty(duty_data, measured);
    
        // Finalizar
        lastTime = now;
        lastDuty = duty_data.duty_val;
        return duty_data;
    }
    
    void WheelSpeedPID::reset() {
        integral = 0.0f;
        lastTime = millis() * MS_TO_S;
        lastDuty = 0.0f;
    }    

// ----- Funciones y variables local (static) de motor_controller -------
static WheelSpeedPID pidLeft(KP_WHEEL, KI_WHEEL, KW_WHEEL); // Instancia PID motor izq.
static WheelSpeedPID pidRight(KP_WHEEL, KI_WHEEL, KW_WHEEL); // Instancia PID motor der.


/* ---------------- MotorController ------------------*/

namespace MotorController {

void init(
    volatile MotorMode& motor_state_global,
    volatile float& dutyL_global, volatile float& dutyR_global
) {
    // Configurar pines de control de L298N
    pinMode(MOTOR_LEFT_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_LEFT_DIR_PIN2, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR_PIN2, OUTPUT);

    // Configurar PWM para motor izquierdo
    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQUENCY, PWM_RES_BITS); // Canal 0
    ledcAttachPin(MOTOR_LEFT_PWM_PIN, PWM_CHANNEL_LEFT);      // Asocia el pin físico al canal 0

    // Configurar PWM para motor derecho
    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQUENCY, PWM_RES_BITS);  // Canal 1
    ledcAttachPin(MOTOR_RIGHT_PWM_PIN, PWM_CHANNEL_RIGHT);

    // Los motores se dejan libres
    motor_state_global = MotorMode::IDLE; 
    set_motor_idle(WHEEL_LEFT);
    set_motor_idle(WHEEL_RIGHT);
    dutyL_global = 0.0;
    dutyR_global = 0.0;
}


void set_motor_pwm(const uint8_t wheel, const float abs_duty, const bool forward){
    const uint32_t pwm = abs_duty * PWM_MAX + 0.5f;
    bool fw = forward;
    if (wheel == WHEEL_LEFT) {
        if constexpr (INVERT_MOTOR_LEFT) fw = !forward;     
        digitalWrite(MOTOR_LEFT_DIR_PIN1, fw ? HIGH : LOW);
        digitalWrite(MOTOR_LEFT_DIR_PIN2, fw ? LOW : HIGH);
        ledcWrite(PWM_CHANNEL_LEFT, pwm);
    } else if (wheel == WHEEL_RIGHT){
        if constexpr (INVERT_MOTOR_RIGHT) fw = !forward; 
        digitalWrite(MOTOR_RIGHT_DIR_PIN1, fw ? HIGH : LOW);
        digitalWrite(MOTOR_RIGHT_DIR_PIN2, fw ? LOW : HIGH);
        ledcWrite(PWM_CHANNEL_RIGHT, pwm);
    }
}


void set_motor_break(const uint8_t wheel) {
    // Los pines de control se fuerzan a HIGH para frenado            
    if (wheel == WHEEL_LEFT) {
        digitalWrite(MOTOR_LEFT_DIR_PIN1, HIGH);
        digitalWrite(MOTOR_LEFT_DIR_PIN2, HIGH);
        ledcWrite(PWM_CHANNEL_LEFT, PWM_MAX);    
    } else if (wheel == WHEEL_RIGHT){
        digitalWrite(MOTOR_RIGHT_DIR_PIN1, HIGH);
        digitalWrite(MOTOR_RIGHT_DIR_PIN2, HIGH);
        ledcWrite(PWM_CHANNEL_RIGHT, PWM_MAX); 
    }
}


void set_motor_idle(const uint8_t wheel) {
    // Los pines de control se fuerzan a LOW
    if (wheel == WHEEL_LEFT) {
        digitalWrite(MOTOR_LEFT_DIR_PIN1, LOW);
        digitalWrite(MOTOR_LEFT_DIR_PIN2, LOW);
        ledcWrite(PWM_CHANNEL_LEFT, 0);    
    } else if (wheel == WHEEL_RIGHT){
        digitalWrite(MOTOR_RIGHT_DIR_PIN1, LOW);
        digitalWrite(MOTOR_RIGHT_DIR_PIN2, LOW);
        ledcWrite(PWM_CHANNEL_RIGHT, 0); 
    }
}


void set_motors_mode(
    const MotorMode mode_new, volatile MotorMode& motor_state_global,
    volatile float& dutyL_global, volatile float& dutyR_global
) {
    // Si se entrega el mismo modo, no se hace nada
    if (mode_new == motor_state_global) return;

    // Modificación del modo de operación
    motor_state_global = mode_new; 
    dutyL_global = 0.0;
    dutyR_global = 0.0;

    // Clasificación según cambio
    if (mode_new == MotorMode::AUTO) { // Resetear los integradores de PID al pasar a AUTO
        pidLeft.reset();
        pidRight.reset();
    } else if (mode_new == MotorMode::BREAK) {
        set_motor_break(WHEEL_LEFT);
        set_motor_break(WHEEL_RIGHT);

    } else { // Cualquier otro modo se considera IDLE (safe)
        set_motor_idle(WHEEL_LEFT);
        set_motor_idle(WHEEL_RIGHT);
    }
}


void apply_duty_profile(
    const uint8_t wheel_id, const DutyProfile duty_data,
    volatile float& global_duty, const MotorMode motor_state
) {
    // Verificar que el sistema esté en un modo válido
    if (motor_state != MotorMode::MANUAL && motor_state != MotorMode::AUTO) return;

    // Caso especial: aplicar freno
    if (duty_data.break_flag) {
        set_motor_break(wheel_id);
        global_duty = 0.0f;
        return;
    }

    // Aplicar PWM: el valor debe haber sido checkeado previamente
    set_motor_pwm(wheel_id, duty_data.abs_duty, duty_data.forward);
    global_duty = duty_data.duty_val;
}


void set_motors_duty(
    const float duty_left, const float duty_right, 
    volatile float& dutyL_global, volatile float& dutyR_global,
    const MotorMode motor_state
) {
    if (motor_state != MotorMode::MANUAL && motor_state != MotorMode::AUTO) return;

    DutyProfile dutyL_data = init_duty_profile(duty_left);
    DutyProfile dutyR_data = init_duty_profile(duty_right);

    check_max_duty(dutyL_data);
    check_max_duty(dutyR_data);

    set_motor_pwm(WHEEL_LEFT, dutyL_data.abs_duty, dutyL_data.forward);
    set_motor_pwm(WHEEL_RIGHT, dutyR_data.abs_duty, dutyR_data.forward);

    dutyL_global = dutyL_data.duty_val;
    dutyR_global = dutyR_data.duty_val;
}


void update_motors_control(
    const float w_L, const float w_R, 
    const float w_L_ref, const float w_R_ref,
    volatile float& duty_L, volatile float& duty_R, 
    const MotorMode state
) {
    if (state != MotorMode::AUTO) return; // Se opera solo en modo auto
    // Cálculo del PI
    DutyProfile dutyL = pidLeft.compute(w_L_ref, w_L);
    DutyProfile dutyR = pidRight.compute(w_R_ref, w_R);
    // Aplicar duty resultante
    apply_duty_profile(WHEEL_LEFT, dutyL, duty_L, state);
    apply_duty_profile(WHEEL_RIGHT, dutyR, duty_R, state);
}


void Task_WheelControl(void* pvParameters) {
    // Datos de RTOS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(WHEEL_CONTROL_PERIOD_MS);

    // Variables globales
    GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    volatile SystemStates& sts = *ctx_ptr->systems_ptr;
    volatile PoseData& pose = *ctx_ptr->pose_ptr;
    volatile ControllerData& ctrl = *ctx_ptr->control_ptr;

    // Llamar periódicamente a la función de control de motores
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        update_motors_control(
            pose.w_L, pose.w_R, ctrl.w_L_ref, ctrl.w_R_ref, ctrl.duty_L, ctrl.duty_R, sts.motors);
    }
}

} // namespace MotorController
