#include "motor_controller.h"

WheelSpeedPID::WheelSpeedPID(float kp, float ki, float kw)
    : Kp(kp), Ki(ki), Kw(kw), integral(0), lastDuty(0), lastTime(0) {}

    float WheelSpeedPID::compute(float setpoint, float measured) {
        // Tiempo actual y delta tiempo
        float now = millis() * MS_TO_S;
        float dt = now - lastTime;
    
        // Protección: solo actualizar cada cierto tiempo
        if (dt < MIN_PID_DT) return lastDuty;
    
        // Control PI
        float error = setpoint - measured;
        float rawDuty = Kp * error + Ki * integral;
    
        // Saturar dentro de [-MAX_DUTY, MAX_DUTY]
        float duty_output = constrain(rawDuty, -MAX_DUTY, MAX_DUTY);
    
        // ------------------- Protección contra cambio brusco de signo -------------------
        if ((duty_output * measured < 0) && abs(measured) > W_INVERT_THRESHOLD) duty_output = 0.0f;

        // ------------------- Anti-windup clásico -------------------
        float dutyError = rawDuty - duty_output;
        float anti_wp = Kw * dutyError;
        integral += (error - anti_wp) * dt;
    
        // Finalizar
        lastTime = now;
        lastDuty = duty_output;
        return duty_output;
    }
    
    void WheelSpeedPID::reset() {
        integral = 0.0f;
        lastTime = millis() * MS_TO_S;
        lastDuty = 0.0f;
    }    

// ----- Funciones y variables local (static) de motor_controller -------
static WheelSpeedPID pidLeft(KP_WHEEL, KI_WHEEL, KW_WHEEL); // Instancia PID motor izq.
static WheelSpeedPID pidRight(KP_WHEEL, KI_WHEEL, KW_WHEEL); // Instancia PID motor der.


// ----- Funciones y variables globales al sistema -------
namespace MotorController {

    void init(
        volatile uint8_t* motor_state_ptr,
        volatile float* dutyL_ptr, volatile float* dutyR_ptr
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
        set_motors_mode(MOTOR_IDLE, motor_state_ptr, dutyL_ptr, dutyR_ptr);
    }

    void set_motor_pwm(uint8_t wheel, float duty, bool forward){
        uint32_t pwm = duty * PWM_MAX + 0.5f;
        if (wheel == WHEEL_LEFT) {    
            digitalWrite(MOTOR_LEFT_DIR_PIN1, forward ? HIGH : LOW);
            digitalWrite(MOTOR_LEFT_DIR_PIN2, forward ? LOW : HIGH);
            ledcWrite(PWM_CHANNEL_LEFT, pwm);
        } else if (wheel == WHEEL_RIGHT){
            digitalWrite(MOTOR_RIGHT_DIR_PIN1, forward ? HIGH : LOW);
            digitalWrite(MOTOR_RIGHT_DIR_PIN2, forward ? LOW : HIGH);
            ledcWrite(PWM_CHANNEL_RIGHT, pwm);
        }
    }
    
    void set_motor_break(int wheel) {
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
    
    void set_motor_idle(int wheel) {
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
        volatile uint8_t mode, volatile uint8_t* motor_state_ptr,
        volatile float* dutyL_ptr, volatile float* dutyR_ptr
    ) {
        // Si se entrega el mismo modo, no se hace nada
        if (mode == *motor_state_ptr) return;

        // Modificación del modo de operación
        *motor_state_ptr = mode; 
        if (mode == MOTOR_AUTO) { // Resetear los integradores de PID al pasar a AUTO
            pidLeft.reset();
            pidRight.reset();
        } else if (mode == MOTOR_BREAK) {
            set_motor_break(WHEEL_LEFT);
            set_motor_break(WHEEL_RIGHT);
            *dutyL_ptr = 0.0;
            *dutyR_ptr = 0.0;

        } else { // Cualquier otro modo se considera IDLE (safe)
            set_motor_idle(WHEEL_LEFT);
            set_motor_idle(WHEEL_RIGHT);
            *dutyL_ptr = 0.0;
            *dutyR_ptr = 0.0;
        }
    }

    void set_motors_duty(
        volatile float duty_left, volatile float duty_right, 
        volatile float* dutyL_ptr, volatile float* dutyR_ptr,
        volatile uint8_t* motor_state_ptr
    ) {
        // En IDLE o BREAK no se permite set duty
        if (*motor_state_ptr != MOTOR_ACTIVE && *motor_state_ptr != MOTOR_AUTO) return; 

        { // Motor izquierdo
            bool forward = duty_left >= 0.0f;   // dirección según el signo de duty
            float duty = roundf(abs(duty_left) * 100.0f) * 0.01f; // Aproximar al segundo decimal
            // Clasificación por bandas
            if (duty < ZERO_DUTY_THRESHOLD) {  // Duty tan pequeño que se considera 0 → apagar motor
                set_motor_idle(WHEEL_LEFT);
                *dutyL_ptr = 0.0f;
            } else {
                if (duty < MIN_MOVE_DUTY) duty = MIN_MOVE_DUTY; // duty pequeño pero insuficiente para avanzar
                if (duty > MAX_DUTY) duty = MAX_DUTY; // limitar al duty máximo (100%)
                *dutyL_ptr = duty;
                
                // Lógica de dirección y escritura de valores
                if (INVERT_MOTOR_LEFT) forward = !forward; // invertir dirección si pines están mal conectados
                set_motor_pwm(WHEEL_LEFT, duty, forward);
            }
        }
        { // Motor derecho
            bool forward = duty_right >= 0.0f;
            float duty = roundf(abs(duty_right) * 100.0f) * 0.01f;
            if (duty < ZERO_DUTY_THRESHOLD) {
                set_motor_idle(WHEEL_RIGHT);
                *dutyR_ptr = 0.0f;
            } else {
                if (duty < MIN_MOVE_DUTY) duty = MIN_MOVE_DUTY;
                if (duty > MAX_DUTY) duty = MAX_DUTY;
                *dutyR_ptr = duty;
                if (INVERT_MOTOR_RIGHT) forward = !forward;
                set_motor_pwm(WHEEL_RIGHT, duty, forward);
            }
        }
    }

    void update_motors_control(
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile float* wL_measured_ptr, volatile float* wR_measured_ptr,
        volatile float* dutyL_ptr, volatile float* dutyR_ptr,
        volatile uint8_t* motor_state_ptr
    ) {
        if (*motor_state_ptr != MOTOR_AUTO) return; // Se opera solo en modo auto

        // Cálculo del PI
        float dutyL = pidLeft.compute(*wL_ref_ptr, *wL_measured_ptr);
        float dutyR = pidRight.compute(*wR_ref_ptr, *wR_measured_ptr);
        // set_motors_duty(dutyL, dutyR, dutyL_ptr, dutyR_ptr, motor_state_ptr);

        // Si no hay freno, se aplica el duty como de costumbre
        if (dutyL != 0.0f || dutyR != 0.0f) {
            dutyL = protect_motor_duty(dutyL,*wL_measured_ptr);
            dutyR = protect_motor_duty(dutyR,*wR_measured_ptr);
            set_motors_duty(dutyL, dutyR, dutyL_ptr, dutyR_ptr, motor_state_ptr);
        }
    
        // Lógica por rueda: si se pide duty = 0 y la rueda sigue girando, frenar activamente
        if (dutyL == 0.0f) { 
            if (abs(*wL_measured_ptr) < W_BRAKE_THRESHOLD) set_motor_break(WHEEL_LEFT);
            else set_motor_idle(WHEEL_LEFT);
            *dutyL_ptr = 0.0f;
        }
        if (dutyR == 0.0f) { 
            if (abs(*wR_measured_ptr) < W_BRAKE_THRESHOLD) set_motor_break(WHEEL_RIGHT);
            else set_motor_idle(WHEEL_RIGHT);
            *dutyR_ptr = 0.0f;
        }
    }
    
    float protect_motor_duty(float duty, float w_measured) {
        // Prevenir inversión de sentido si la rueda aún gira rápido
        if ((duty * w_measured < 0.0f) && abs(w_measured) > W_INVERT_THRESHOLD) {
            return 0.0f;
        }
    
        // Si la rueda está completamente detenida y se quiere mover, duty debe ser al menos MIN_START_DUTY
        if (abs(w_measured) < W_STOP_THRESHOLD && abs(duty) > 0.0f && abs(duty) < MIN_START_DUTY) {
            duty = (duty > 0.0f ? 1.0f : -1.0f) * MIN_START_DUTY;
        }
    
        return duty;
    }

    void Task_WheelControl(void* pvParameters) {
        // Datos de RTOS
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(WHEEL_CONTROL_PERIOD_MS);
    
        // Cast del parámetro entregado a GlobalContext
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    
        // Punteros a las variables necesarias
        volatile uint8_t* motor_state_ptr = &ctx_ptr->systems_ptr->motor_operation;
        volatile float* v_ref_ptr         = &ctx_ptr->kinematic_ptr->v_ref;
        volatile float* w_ref_ptr         = &ctx_ptr->kinematic_ptr->w_ref;
        volatile float* wL_ref_ptr        = &ctx_ptr->wheels_ptr->wL_ref;
        volatile float* wR_ref_ptr        = &ctx_ptr->wheels_ptr->wR_ref;
        volatile float* wL_measured_ptr   = &ctx_ptr->wheels_ptr->wL_measured;
        volatile float* wR_measured_ptr   = &ctx_ptr->wheels_ptr->wR_measured;
        volatile float* dutyL_ptr         = &ctx_ptr->wheels_ptr->duty_left;
        volatile float* dutyR_ptr         = &ctx_ptr->wheels_ptr->duty_right;
    
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            update_motors_control(
                wL_ref_ptr, wR_ref_ptr, 
                wL_measured_ptr, wR_measured_ptr, 
                dutyL_ptr, dutyR_ptr, 
                motor_state_ptr
            );
        }
    }  

}
