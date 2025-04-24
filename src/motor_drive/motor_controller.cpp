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
        bool inversion = (duty_output * measured < 0);
        if (inversion && abs(measured) > W_INVERT_THRESHOLD) {
            duty_output = 0.0f;  // Evitar el cambio brusco de sentido
        }
    
        // ------------------- Anti-windup clásico (con duty saturado final) -------------------
        float dutyError = rawDuty - duty_output;
        float anti_wp = Kw * dutyError;
        integral += (error - anti_wp) * dt;
    
        // Finalizar
        lastTime = now;
        lastDuty = duty_output;
        return duty_output;
    }
    

void WheelSpeedPID::reset() {
    integral = 0.0;
    lastTime = millis() * MS_TO_S;
    //lastDuty = 0.0;
}    

// ----- Funciones y variables local (static) de motor_controller -------
/* 
MOTOR_IDLE = se dejan libres los motores, alta impedancia entre los bornes del motor
MOTOR_BREAK = motores bloqueados, frena el motor
MOTOR_ACTIVE = se controla la velocidad con el duty y los pines de control
*/
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
        set_motor_mode(MOTOR_IDLE, motor_state_ptr, dutyL_ptr, dutyR_ptr);
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

    void set_motor_mode(
        volatile uint8_t mode, volatile uint8_t* motor_state_ptr,
        volatile float* dutyL_ptr, volatile float* dutyR_ptr
    ) {
        // Si se entrega el mismo modo, no se hace nada
        if (mode == *motor_state_ptr) return;
        *motor_state_ptr = mode; 

        if (mode == MOTOR_AUTO) { // Resetear los integradores de PID al pasar a AUTO
            pidLeft.reset();
            pidRight.reset();
        } else if (mode == MOTOR_BREAK) {
            set_motor_break(WHEEL_LEFT);
            set_motor_break(WHEEL_RIGHT);
            // Los duty se dejan en cero
            *dutyL_ptr = 0.0;
            *dutyR_ptr = 0.0;

        } else { // Cualquier otro modo se considera IDLE (safe)
            set_motor_idle(WHEEL_LEFT);
            set_motor_idle(WHEEL_RIGHT);
            ledcWrite(PWM_CHANNEL_LEFT, 0);  
            ledcWrite(PWM_CHANNEL_RIGHT, 0);
            
            // Los duty se dejan en cero
            *dutyL_ptr = 0.0;
            *dutyR_ptr = 0.0;
        }
    }

    void set_motor_duty(
        volatile float duty_left, volatile float duty_right, 
        volatile float* dutyL_ptr, volatile float* dutyR_ptr,
        volatile uint8_t* motor_state_ptr
    ) {
        // En IDLE o BREAK no se permite set duty
        if (*motor_state_ptr != MOTOR_ACTIVE && *motor_state_ptr != MOTOR_AUTO) return; 

        { // Motor izquierdo
            bool forward = duty_left >= 0;   // dirección según el signo de duty
            float duty = roundf(abs(duty_left) * 100.0f) * 0.01f; // Aproximar al segundo decimal
            // Clasificación por bandas
            if (duty < ZERO_DUTY_THRESHOLD) {
                // Duty tan pequeño que se considera 0 → apagar motor
                digitalWrite(MOTOR_LEFT_DIR_PIN1, LOW);
                digitalWrite(MOTOR_LEFT_DIR_PIN2, LOW);
                ledcWrite(PWM_CHANNEL_LEFT, 0U);
                *dutyL_ptr = 0.0f;
            } else {
                // duty pequeño pero insuficiente para avanzar -> se fija el mínimo duty que genera movimiento
                if (duty < MIN_EFFECTIVE_DUTY) duty = MIN_EFFECTIVE_DUTY;
                if (duty > MAX_DUTY) duty = MAX_DUTY; // limitar al duty máximo (100%)
                uint32_t pwm = duty * PWM_MAX + 0.5f;
                
                // Escribir los valores
                if (INVERT_MOTOR_LEFT) forward = !forward; // invertir dirección por reflejo físico de las ruedas
                digitalWrite(MOTOR_LEFT_DIR_PIN1, forward ? HIGH : LOW);
                digitalWrite(MOTOR_LEFT_DIR_PIN2, forward ? LOW : HIGH);
                ledcWrite(PWM_CHANNEL_LEFT, pwm);
                *dutyL_ptr = duty_left;
            }
        }
        { // Motor derecho
            bool forward = duty_right >= 0.0f;
            float duty = round(abs(duty_right) * 100.0f) / 100.0f;
            if (duty < ZERO_DUTY_THRESHOLD) {
                digitalWrite(MOTOR_RIGHT_DIR_PIN1, LOW);
                digitalWrite(MOTOR_RIGHT_DIR_PIN2, LOW);
                ledcWrite(PWM_CHANNEL_RIGHT, 0U);
                *dutyR_ptr = 0.0f;
            } else {
                // Limitar el duty
                if (duty < MIN_EFFECTIVE_DUTY) duty = MIN_EFFECTIVE_DUTY;
                if (duty > MAX_DUTY) duty = MAX_DUTY;
                uint32_t pwm = duty * PWM_MAX + 0.5f;
                
                // Escribir los valores
                if (INVERT_MOTOR_RIGHT) forward = !forward; // invertir dirección por reflejo físico de las ruedas
                digitalWrite(MOTOR_RIGHT_DIR_PIN1, forward ? HIGH : LOW);
                digitalWrite(MOTOR_RIGHT_DIR_PIN2, forward ? LOW : HIGH);
                ledcWrite(PWM_CHANNEL_RIGHT, pwm);
                *dutyR_ptr = duty_right;
            }
        }
    }

    void update_motor_control(
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile float* wL_measured_ptr, volatile float* wR_measured_ptr,
        volatile float* dutyL_ptr, volatile float* dutyR_ptr,
        volatile uint8_t* motor_state_ptr
    ) {
        if (*motor_state_ptr != MOTOR_AUTO) return; // Se opera solo en modo auto

        // Cálculo del PI
        float dutyL = pidLeft.compute(*wL_ref_ptr, *wL_measured_ptr);
        float dutyR = pidRight.compute(*wR_ref_ptr, *wR_measured_ptr);

        // Si no hay freno, se aplica el duty como de costumbre
        if (dutyL != 0.0f || dutyR != 0.0f) {
            set_motor_duty(dutyL, dutyR, dutyL_ptr, dutyR_ptr, motor_state_ptr);
        }
    
        // Lógica por rueda: si se pide duty = 0 y la rueda sigue girando, frenar activamente
        if (dutyL == 0.0f && abs(*wL_measured_ptr) < W_BRAKE_THRESHOLD) {
            set_motor_break(WHEEL_LEFT);
            *dutyL_ptr = 0.0f;
        } else {
            set_motor_idle(WHEEL_LEFT);  // o se reactiva con el nuevo duty más abajo
        }
    
        if (dutyR == 0.0f && abs(*wR_measured_ptr) < W_BRAKE_THRESHOLD) {
            set_motor_break(WHEEL_RIGHT);
            *dutyR_ptr = 0.0f;
        } else {
            set_motor_idle(WHEEL_RIGHT);
        }
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
            update_motor_control(
                wL_ref_ptr, wR_ref_ptr, 
                wL_measured_ptr, wR_measured_ptr, 
                dutyL_ptr, dutyR_ptr, 
                motor_state_ptr
            );
        }
    }  

}
