#include "motor_controller.h"

/* ---------------- Funciones auxiliares generales ------------------*/

void check_duty_limits(DutyProfile& duty) {
    if (duty.abs_duty >= MIN_MOVE_DUTY) {                 
        if (duty.abs_duty > MAX_DUTY) {
            duty.abs_duty = MAX_DUTY; // Limitar al duty máximo (100%)
            duty.duty_val = MAX_DUTY * duty.duty_dir;
        } 
    } else if (duty.abs_duty > ZERO_DUTY_THRESHOLD) {
        duty.abs_duty = MIN_MOVE_DUTY;            // Duty es válido pero menor al mínimo útil
        duty.duty_val = MIN_MOVE_DUTY * duty.duty_dir;
    } else {
        duty.abs_duty = 0.0f;                     // Duty demasiado bajo se toma como cero
        duty.duty_val = 0.0f;
    }
}

void check_duty_speed(DutyProfile& duty, const float w_measured, const float w_ref) {
    // Revisar si se quiere realizar frenado o inversión
    int8_t w_measured_sign = (w_measured>=0) ? 1 : -1;
    int8_t w_ref_sign = (w_ref>=0) ? 1 : -1;
    float w_measured_abs = fabsf(w_measured);
    float w_ref_abs = fabsf(w_ref);
    duty.break_flag = false; // predeterminadamente se devuelve a false

    // Caso en que se desea invertir o frenar activamente
    if ((duty.duty_dir != w_measured_sign) && (w_measured_abs < w_ref_abs || w_measured_sign != w_ref_sign)) {
        if (w_measured_abs < W_INVERT_THRESHOLD) return; // Se puede aplicar la inversión!
    else { // Caso en que la velocidad es demasiado alta para invertir el duty                                                          
            duty.duty_val = 0.0f;
            duty.abs_duty = 0.0f;
            if (w_measured_abs < W_BRAKE_THRESHOLD) duty.break_flag = true; // Frenado activo
        }
    }
}

DutyProfile init_duty_profile(const float rawDuty) {
    DutyProfile duty_profile = {0};
    duty_profile.duty_val = rawDuty;
    duty_profile.abs_duty = fabsf(rawDuty);
    bool fw = (rawDuty >= 0.0f);
    duty_profile.forward = fw;
    duty_profile.duty_dir = fw ? 1 : -1;
    return duty_profile;
}

// void update_duty_profile(const float rawDuty, DutyProfile& duty_profile) {
//     duty_profile.duty_val = rawDuty;
//     duty_profile.abs_duty = fabsf(rawDuty);
//     bool fw = rawDuty >= 0.0f;
//     duty_profile.forward = fw;
//     duty_profile.duty_dir = fw ? 1 : -1;
// }


/* ------------------ Funciones de control PI --------------------*/

WheelSpeedPID::WheelSpeedPID(float kp, float ki, float kw)
    : Kp(kp), Ki(ki), Kw(kw), integral(0), lastDuty(0), lastTime(0) {}

    DutyProfile WheelSpeedPID::compute(const float setpoint, const float measured) {
        // Tiempo actual y delta tiempo
        float now = millis() * MS_TO_S;
        float dt = now - lastTime;
    
        // Protección: solo actualizar cada cierto tiempo
        if (dt < MIN_PID_DT) return init_duty_profile(lastDuty);
    
        // Control PI
        float error = setpoint - measured;
        float rawDuty = Kp * error + Ki * integral;
        DutyProfile duty_data = init_duty_profile(rawDuty);

        // Se limita el duty según límites prácticos y técnicos por velocidad
        check_duty_limits(duty_data);
        check_duty_speed(duty_data, measured, setpoint);

        // Anti-windup clásico
        float dutyError = rawDuty - duty_data.duty_val;
        float anti_wp = Kw * dutyError;
        integral += (error - anti_wp) * dt;
    
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
        volatile uint8_t& motor_state_global,
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
        motor_state_global = MOTOR_IDLE; 
        set_motor_idle(WHEEL_LEFT);
        set_motor_idle(WHEEL_RIGHT);
        dutyL_global = 0.0;
        dutyR_global = 0.0;
    }

    void set_motor_pwm(const uint8_t wheel, const float abs_duty, bool forward){
        uint32_t pwm = abs_duty * PWM_MAX + 0.5f;
        if (wheel == WHEEL_LEFT) {
            if constexpr (INVERT_MOTOR_LEFT) forward = !forward;     
            digitalWrite(MOTOR_LEFT_DIR_PIN1, forward ? HIGH : LOW);
            digitalWrite(MOTOR_LEFT_DIR_PIN2, forward ? LOW : HIGH);
            ledcWrite(PWM_CHANNEL_LEFT, pwm);
        } else if (wheel == WHEEL_RIGHT){
            if constexpr (INVERT_MOTOR_RIGHT) forward = !forward;
            digitalWrite(MOTOR_RIGHT_DIR_PIN1, forward ? HIGH : LOW);
            digitalWrite(MOTOR_RIGHT_DIR_PIN2, forward ? LOW : HIGH);
            ledcWrite(PWM_CHANNEL_RIGHT, pwm);
        }
    }
    
    void set_motor_break(uint8_t wheel) {
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
    
    void set_motor_idle(uint8_t wheel) {
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
        volatile uint8_t mode_new, volatile uint8_t& motor_state_global,
        volatile float& dutyL_global, volatile float& dutyR_global
    ) {
        // Si se entrega el mismo modo, no se hace nada
        if (mode_new == motor_state_global) return;

        // Modificación del modo de operación
        motor_state_global = mode_new; 
        if (mode_new == MOTOR_AUTO) { // Resetear los integradores de PID al pasar a AUTO
            pidLeft.reset();
            pidRight.reset();
        } else if (mode_new == MOTOR_BREAK) {
            set_motor_break(WHEEL_LEFT);
            set_motor_break(WHEEL_RIGHT);
            dutyL_global = 0.0;
            dutyR_global = 0.0;

        } else { // Cualquier otro modo se considera IDLE (safe)
            set_motor_idle(WHEEL_LEFT);
            set_motor_idle(WHEEL_RIGHT);
            dutyL_global = 0.0;
            dutyR_global = 0.0;
        }
    }

    void apply_duty_profile(
        const uint8_t wheel_id,
        const DutyProfile& duty_data,
        volatile float& global_duty,
        volatile uint8_t& motor_state
    ) {
        // Verificar que el sistema esté en un modo válido
        if (motor_state != MOTOR_ACTIVE && motor_state != MOTOR_AUTO) return;

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
        volatile float duty_left, volatile float duty_right, 
        volatile float& dutyL_global, volatile float& dutyR_global,
        volatile uint8_t& motor_state
    ) {
        if (motor_state != MOTOR_ACTIVE && motor_state != MOTOR_AUTO) return;

        DutyProfile dutyL_data = init_duty_profile(duty_left);
        DutyProfile dutyR_data = init_duty_profile(duty_right);

        check_duty_limits(dutyL_data);
        check_duty_limits(dutyR_data);

        set_motor_pwm(WHEEL_LEFT, dutyL_data.abs_duty, dutyL_data.forward);
        set_motor_pwm(WHEEL_RIGHT, dutyR_data.abs_duty, dutyR_data.forward);

        dutyL_global = dutyL_data.duty_val;
        dutyR_global = dutyR_data.duty_val;
    }

    void update_motors_control(volatile WheelsData& wheels_data, volatile uint8_t& motor_state) {
        if (motor_state != MOTOR_AUTO) return; // Se opera solo en modo auto
        // Cálculo del PI
        DutyProfile dutyL = pidLeft.compute(wheels_data.wL_ref, wheels_data.wL_measured);
        DutyProfile dutyR = pidRight.compute(wheels_data.wR_ref, wheels_data.wR_measured);

        // Aplicar
        apply_duty_profile(WHEEL_LEFT, dutyL, wheels_data.duty_left, motor_state);
        apply_duty_profile(WHEEL_RIGHT, dutyR, wheels_data.duty_right, motor_state);
    }

    void Task_WheelControl(void* pvParameters) {
        // Datos de RTOS
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(WHEEL_CONTROL_PERIOD_MS);
    
        // Cast del parámetro entregado a GlobalContext
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    
        // Punteros a las variables necesarias
        volatile uint8_t* motor_state_ptr = &ctx_ptr->systems_ptr->motor_operation;
        volatile WheelsData* wheels_data_ptr = ctx_ptr->wheels_ptr;

        // Llamar periódicamente a la función
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            update_motors_control(*wheels_data_ptr, *motor_state_ptr);
        }
    }  

}
