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
constexpr float MIN_MOVE_DUTY = 0.17f;   // duty mínimo con efecto real (0.17 sin carga para 1000Hz)
constexpr float MIN_START_DUTY = 0.22f;   // duty mínimo con efecto real (0.22 sin carga para 1000Hz)
constexpr float MAX_DUTY = 1.0f;

// Speed controller PI parameters
constexpr float KP_WHEEL = 0.09f;
constexpr float KI_WHEEL = 0.07f;
constexpr float KW_WHEEL = 0.01f;

// Speed control limits
constexpr float MIN_PID_DT = 0.001f;  // Tiempo mínimo entre ejecuciones [s]
constexpr float W_INVERT_THRESHOLD = 0.1 * WM_NOM; // No invertir el signo del duty si va muy rápido [rad/s]
constexpr float W_BRAKE_THRESHOLD = 0.5 * WM_NOM;  // Threshold de velocidad para freno activo [rad/s]
constexpr float W_STOP_THRESHOLD = 0.01 * WM_NOM;  // Threshold de velocidad para considerar stop [rad/s]

// Corrección de pines
constexpr bool INVERT_MOTOR_LEFT = true;
constexpr bool INVERT_MOTOR_RIGHT = true;


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
     * @brief Calcula el duty necesario a partir del setpoint y la velocidad medida.
     * 
     * @param setpoint Velocidad deseada de la rueda (rad/s).
     * @param measured Velocidad medida de la rueda (rad/s).
     * @return Duty saturado en el rango [-MAX_DUTY, MAX_DUTY].
     */
    float compute(float setpoint, float measured);

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


/**
 * @brief Espacio de nombres que agrupa las funciones de inicialización, control y tareas de motor.
 */
namespace MotorController {

    /**
     * @brief Inicializa el controlador de motores: pines, PWM y estado inicial.
     * 
     * @param motor_state_ptr Puntero al estado de operación del sistema de motores.
     * @param dutyL_ptr Puntero al duty actual de la rueda izquierda.
     * @param dutyR_ptr Puntero al duty actual de la rueda derecha.
     */
    void init(
        volatile uint8_t* motor_state_ptr,
        volatile float* dutyL_ptr,
        volatile float* dutyR_ptr
    );
    
    /**
     * @brief Establece la señal PWM y dirección para un motor individual.
     *
     * Esta función configura los pines de dirección según el sentido (`forward`) y aplica
     * el valor de PWM correspondiente al canal asociado.
     *
     * @param wheel Identificador del motor (`WHEEL_LEFT` o `WHEEL_RIGHT`).
     * @param duty Valor del duty cycle (0.0–1.0) ya procesado.
     * @param forward Sentido de giro: `true` para adelante, `false` para reversa.
     */
    void set_motor_pwm(uint8_t wheel, float duty, bool forward);

    /**
     * @brief Aplica freno activo a un motor utilizando el L298N.
     *
     * Esta función pone ambos pines de dirección en HIGH y aplica PWM máximo
     * para forzar el frenado eléctrico (modo `BREAK` del L298N).
     *
     * @param wheel Identificador del motor (`WHEEL_LEFT` o `WHEEL_RIGHT`).
     */
    void set_motor_break(int wheel);

    /**
     * @brief Coloca un motor en modo libre (alta impedancia).
     *
     * Desactiva la señal PWM y pone ambos pines de dirección en LOW, permitiendo que la rueda
     * gire libremente sin torque aplicado (modo `IDLE` del L298N).
     *
     * @param wheel Identificador del motor (`WHEEL_LEFT` o `WHEEL_RIGHT`).
     */
    void set_motor_idle(int wheel);

    /**
     * @brief Cambia el modo de operación de los motores (IDLE, ACTIVE, BREAK).
     * También reinicia los integradores del PID y actualiza el PWM según corresponda.
     * 
     * @param mode Nuevo modo de operación.
     * @param motor_state_ptr Puntero al estado actual del sistema de motores.
     * @param dutyL_ptr Puntero al duty actual de la rueda izquierda.
     * @param dutyR_ptr Puntero al duty actual de la rueda derecha.
     */
    void set_motors_mode(
        volatile uint8_t mode,
        volatile uint8_t* motor_state_ptr,
        volatile float* dutyL_ptr,
        volatile float* dutyR_ptr
    );

    /**
     * @brief Aplica un duty al PWM y actualiza la estructura correspondiente con el duty efectivo aplicado.
     * 
     * @param duty_left Duty para la rueda izquierda.
     * @param duty_right Duty para la rueda derecha.
     * @param dutyL_ptr Puntero a la variable donde se almacenará el duty efectivo aplicado a la rueda izquierda.
     * @param dutyR_ptr Puntero a la variable donde se almacenará el duty efectivo aplicado a la rueda derecha.
     * @param motor_state_ptr Puntero a la variable con el estado de operación del encoder
     */
    void set_motors_duty(
        volatile float duty_left,
        volatile float duty_right,
        volatile float* dutyL_ptr,
        volatile float* dutyR_ptr,
        volatile uint8_t* motor_state_ptr
    );

    /**
     * @brief Protege y ajusta el duty calculado antes de aplicarlo al motor.
     *
     * Esta función implementa dos protecciones comunes en el control de velocidad de motores:
     * 
     * 1. **Protección contra inversión brusca de sentido:** Si se intenta aplicar un duty con signo
     *    opuesto a la velocidad actual de la rueda (y esta aún gira más rápido que un umbral),
     *    el duty se anula para evitar choques mecánicos.
     * 
     * 2. **Duty mínimo para arranque:** Si la rueda está detenida (velocidad medida ≈ 0) y se
     *    desea aplicar un duty demasiado bajo para superar la fricción estática, se fuerza
     *    a un mínimo necesario (`MIN_START_DUTY`).
     * 
     * @param duty Duty calculado por el controlador PI (valor normalizado entre -1 y 1).
     * @param w_measured Velocidad angular actual de la rueda [rad/s].
     * @return Duty corregido que puede aplicarse de forma segura al motor.
     */
    float protect_motor_duty(float duty, float w_measured);

    /**
     * @brief Ejecuta un paso de control para ambos motores en modo automático.
     *
     * Esta función calcula el duty necesario para alcanzar la velocidad angular
     * de referencia en cada rueda, usando un controlador PI con lógica de protección.
     * Si el duty calculado es cero y la rueda aún gira, se aplica freno activo.
     *
     * @param wheels_data_ptr Puntero a la estructura que contiene la información de referencia,
     *        velocidad medida y duty aplicado para cada rueda.
     * @param motor_state_ptr Puntero al estado actual del sistema de motores (debe ser MOTOR_AUTO para operar).
     */
    void update_motors_control(
        volatile WheelsData* wheels_data_ptr,
        volatile uint8_t* motor_state_ptr
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
