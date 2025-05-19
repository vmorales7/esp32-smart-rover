#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "project_config.h"

/* ---------------- Constantes del sistema ------------------*/

// PWM parameters
constexpr uint32_t PWM_FREQUENCY = 1000;               // Frecuencia en Hz
constexpr uint8_t PWM_RES_BITS = 8;                    // Resolución en bits (8 → rango 0–255)
constexpr uint32_t PWM_MAX = (1 << PWM_RES_BITS) - 1;  // Duty máximo según resolución
constexpr uint8_t PWM_CHANNEL_LEFT = 0U;
constexpr uint8_t PWM_CHANNEL_RIGHT = 1U;

// Motor duty limits
constexpr float ZERO_DUTY_THRESHOLD = 0.1f; // por debajo de esto se considera 0
constexpr float MIN_MOVE_DUTY = 0.29f;      // mínimo con movimiento para pwm de 1000Hz (0.17 sin carga)
constexpr float MIN_START_DUTY = 0.33f;     // mínimo para partida para pwm de 1000Hz (0.22 sin carga)
constexpr float MAX_DUTY = 1.0f;

// Speed controller PI parameters
constexpr float KP_WHEEL = 0.09f;
constexpr float KI_WHEEL = 0.07f;
constexpr float KW_WHEEL = 0.01f;

// Speed control limits
constexpr float MIN_PID_DT = 0.001f;  // Tiempo mínimo entre ejecuciones [s]
constexpr float W_INVERT_THRESHOLD = 0.1 * WM_NOM;        // No invertir el signo del duty si va muy rápido [rad/s]
constexpr float W_BRAKE_THRESHOLD  = 0.5 * WM_NOM;        // Threshold de velocidad para freno activo [rad/s]
constexpr float W_STOP_THRESHOLD   = (2.0f * PI / 15.0f); // Threshold de velocidad para considerar stop [rad/s]

// Corrección de pines
constexpr bool INVERT_MOTOR_LEFT = true;
constexpr bool INVERT_MOTOR_RIGHT = true;


/* ---------------- Estructura de perfil de duty ------------------*/

/**
 * @brief Representa el perfil de un duty calculado para una rueda.
 */
struct DutyProfile {
    float duty_val;     ///< Valor del duty (con signo)
    float abs_duty;     ///< Valor absoluto del duty
    int8_t duty_dir;    ///< Dirección: +1 o -1
    bool forward;       ///< Dirección lógica: true = adelante
    bool break_flag;    ///< true si se debe aplicar freno activo
};

/* ---------------- Operación ------------------*/

/**
 * @brief Clase para el control de velocidad de rueda usando un controlador PI con anti-windup.
 */
class WheelSpeedPID {
public:
    /**
     * @brief Constructor del controlador PI de rueda.
     * 
     * @param kp Ganancia proporcional.
     * @param ki Ganancia integral.
     * @param kw Ganancia anti-windup (usada para limitar el crecimiento del término integral).
     */
    WheelSpeedPID(float kp, float ki, float kw);

    /**
     * @brief Calcula un nuevo duty a partir del setpoint y la velocidad medida.
     * @param setpoint Velocidad de referencia [rad/s].
     * @param measured Velocidad medida [rad/s].
     * @return Perfil de duty calculado, ya protegido.
     */
    DutyProfile compute(float setpoint, float measured);

    /**
     * @brief Reinicia el integrador y el tiempo anterior del controlador.
     */
    void reset();

private:
    float Kp, Ki, Kw;
    float integral;
    float lastDuty;
    float lastTime;
};


/* ---------------- Funciones auxiliares generales ------------------*/

/**
 * @brief Inicializa un perfil de duty a partir de un valor crudo.
 * @param rawDuty Valor crudo del duty [-1.0, 1.0]
 * @return DutyProfile inicializado
 */
DutyProfile init_duty_profile(float rawDuty);

/**
 * @brief Aplica límites prácticos y técnicos al duty.
 * @param duty Referencia al perfil a proteger
 */
void check_duty_limits(DutyProfile& duty);

/**
 * @brief Verifica si debe aplicarse freno o impedir inversión.
 * @param duty Perfil de duty a actualizar
 * @param w_measured Velocidad medida [rad/s]
 * @param w_ref Velocidad de referencia [rad/s]
 */
void check_duty_speed(DutyProfile& duty, float w_measured, float w_ref);


/* ---------------- MotorController ------------------*/

/**
 * @brief Espacio de nombres que agrupa las funciones de inicialización, control y tareas de motor.
 */
namespace MotorController {

    /**
     * @brief Inicializa el sistema de control de motores.
     * @param motor_state_global Referencia al estado actual del motor.
     * @param dutyL_global Referencia al duty aplicado a rueda izquierda.
     * @param dutyR_global Referencia al duty aplicado a rueda derecha.
     */
    void init(
        volatile uint8_t& motor_state_global,
        volatile float& dutyL_global,
        volatile float& dutyR_global
    );
    
    /**
     * @brief Configura el PWM y pines de dirección de un motor individual.
     * @param wheel Identificador de rueda.
     * @param abs_duty Valor absoluto del duty [0.0 – 1.0]
     * @param forward Sentido del giro: true = adelante
     */
    void set_motor_pwm(uint8_t wheel, float abs_duty, bool forward);

    /**
     * @brief Aplica freno activo a un motor utilizando el L298N.
     *
     * Esta función pone ambos pines de dirección en HIGH y aplica PWM máximo
     * para forzar el frenado eléctrico (modo `BREAK` del L298N).
     *
     * @param wheel Identificador del motor (`WHEEL_LEFT` o `WHEEL_RIGHT`).
     */
    void set_motor_break(uint8_t wheel);

    /**
     * @brief Coloca un motor en modo libre (alta impedancia).
     *
     * Desactiva la señal PWM y pone ambos pines de dirección en LOW, permitiendo que la rueda
     * gire libremente sin torque aplicado (modo `IDLE` del L298N).
     *
     * @param wheel Identificador del motor (`WHEEL_LEFT` o `WHEEL_RIGHT`).
     */
    void set_motor_idle(uint8_t wheel);

    /**
     * @brief Cambia el modo global de los motores.
     * @param mode_new Modo deseado (IDLE, ACTIVE, BREAK, AUTO).
     * @param motor_state_global Referencia al estado global del sistema.
     * @param dutyL_global Referencia al duty izquierdo.
     * @param dutyR_global Referencia al duty derecho.
     */
    void set_motors_mode(
        volatile uint8_t mode_new,
        volatile uint8_t& motor_state_global,
        volatile float& dutyL_global,
        volatile float& dutyR_global
    );

    /**
     * @brief Aplica directamente un perfil de duty a un motor.
     * @param wheel_id Rueda a controlar (WHEEL_LEFT o WHEEL_RIGHT)
     * @param duty_data Perfil de duty ya calculado y protegido.
     * @param global_duty Referencia al duty efectivo.
     * @param motor_state Estado actual del sistema de motor.
     */
    void apply_duty_profile(
        uint8_t wheel_id,
        const DutyProfile& duty_data,
        volatile float& global_duty,
        volatile uint8_t& motor_state
    );
            
    /**
     * @brief Aplica un duty al PWM y actualiza la estructura correspondiente con el duty efectivo aplicado.
     * 
     * @param duty_left Duty para la rueda izquierda.
     * @param duty_right Duty para la rueda derecha.
     * @param dutyL_global Variable global con la variable donde se almacenará el duty efectivo aplicado a la rueda izquierda.
     * @param dutyR_global Variable global con la variable donde se almacenará el duty efectivo aplicado a la rueda derecha.
     * @param motor_state Variable global con la variable con el estado de operación del encoder
     */
    void set_motors_duty(
        volatile float duty_left,
        volatile float duty_right,
        volatile float& dutyL_global,
        volatile float& dutyR_global,
        volatile uint8_t& motor_state
    );

    /**
     * @brief Ejecuta un paso de control para ambos motores en modo automático.
     *
     * Esta función calcula el duty necesario para que cada rueda alcance su velocidad angular
     * de referencia, utilizando un controlador PI con protección contra inversión abrupta,
     * arranques en parado y anti-windup. Si se detecta que el sentido deseado difiere del real
     * y la velocidad aún es alta, se aplica frenado activo.
     *
     * @param w_L Velocidad angular medida de la rueda izquierda [rad/s].
     * @param w_R Velocidad angular medida de la rueda derecha [rad/s].
     * @param w_L_ref Velocidad angular de referencia para la rueda izquierda [rad/s].
     * @param w_R_ref Velocidad angular de referencia para la rueda derecha [rad/s].
     * @param duty_L Referencia al duty aplicado al motor izquierdo (salida) [-1.0, 1.0].
     * @param duty_R Referencia al duty aplicado al motor derecho (salida) [-1.0, 1.0].
     * @param state Estado del controlador de motores (debe estar en MOTOR_AUTO para ejecutar control).
     */
    void update_motors_control(
        volatile float& w_L, volatile float& w_R, 
        volatile float& w_L_ref, volatile float& w_R_ref,
        volatile float& duty_L, volatile float& duty_R, 
        volatile uint8_t& state
    );

    /**
     * @brief Tarea de FreeRTOS que ejecuta periódicamente el control de velocidad de ruedas.
     * Usa punteros extraídos desde una estructura `GlobalContext`.
     * 
     * @param pvParameters Puntero a un `GlobalContext*` que contiene referencias al estado del sistema.
     */
    void Task_WheelControl(void* pvParameters);

} // namespace MotorController

#endif // MOTOR_CONTROLLER_H
