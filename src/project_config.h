#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

/* -------------- Definiciones de los pines de la ESP32 --------------*/

// Para control del L298N
constexpr uint8_t MOTOR_LEFT_PWM_PIN = 26;  // Azul
constexpr uint8_t MOTOR_LEFT_DIR_PIN1 = 25; // Blanco
constexpr uint8_t MOTOR_LEFT_DIR_PIN2 = 33; // Verde

constexpr uint8_t MOTOR_RIGHT_PWM_PIN = 18;  // Azul
constexpr uint8_t MOTOR_RIGHT_DIR_PIN1 = 19; // Blanco
constexpr uint8_t MOTOR_RIGHT_DIR_PIN2 = 21; // Verde

// Para los encoders
// Azul = Vcc & Negro = GND 
constexpr uint8_t ENCODER_LEFT_A_PIN = 14; // Amarillo
constexpr uint8_t ENCODER_LEFT_B_PIN = 27; // Verde
constexpr uint8_t ENCODER_RIGHT_A_PIN = 16; // Amarillo
constexpr uint8_t ENCODER_RIGHT_B_PIN = 17; // Verde

// Sensores ultrasónicos
constexpr uint8_t US_LEFT_TRIG_PIN  = 13;
constexpr uint8_t US_LEFT_ECHO_PIN  = 12;

constexpr uint8_t US_RIGHT_TRIG_PIN = 22;
constexpr uint8_t US_RIGHT_ECHO_PIN = 23;

// Sensor infrarrojo
constexpr uint8_t IR_LEFT_SENSOR_PIN = 15;
constexpr uint8_t IR_RIGHT_SENSOR_PIN = 10;


/* -------------- Constantes generales --------------*/

// Para los estados de los sistemas
constexpr uint8_t ACTIVE   = 1U;
constexpr uint8_t INACTIVE = 0U;

// Para los condicionales con las ruedas
constexpr uint8_t WHEEL_LEFT  = 0U;
constexpr uint8_t WHEEL_RIGHT = 1U;

// Parámetros físicos del vegículo
constexpr float WHEEL_RADIUS = 0.066f / 2.0f;    // en metros
constexpr float WHEEL_DISTANCE = 0.194f / 2.0f;   // distancia entre ruedas (L)

// Auxiliares
constexpr float MS_TO_S = 0.001f;


/* -------------- Constantes del Motor Controller --------------*/


// Motor speed characteristics
constexpr uint16_t RPM_NOM = 215U;            // rpm nominales bajo carga (son 280 sin carga)       
constexpr float WM_NOM = RPM_NOM * 2*PI/60.0; // rad/s nominales bajo carga = 22.51 (29.3 sin carga)

// Motor modes
enum MotorMode : uint8_t {
    MOTOR_IDLE = 0U,    // Se dejan libres los motores, alta impedancia entre los bornes del motor
    MOTOR_ACTIVE = 1U,  // Se controla la velocidad con el duty y los pines de control
    MOTOR_AUTO = 2U,    // Modo de control automático
    MOTOR_BREAK = 3U    // Motores bloqueados, frena el motor
};

// RTOS
constexpr uint16_t WHEEL_CONTROL_PERIOD_MS = 10;


/* -------------- Constantes del Encoder Reader --------------*/

// Según tipo de configuración de encoder
enum class EncoderMode {
    SINGLE_EDGE = 1,  // 1x
    HALF_QUAD   = 2,  // 2x
    FULL_QUAD   = 4   // 4x
};
constexpr EncoderMode ENCODER_MODE = EncoderMode::HALF_QUAD; // Escoger el formato de lectura de encoder
constexpr float get_encoder_multiplier(EncoderMode mode) {
    return (mode == EncoderMode::SINGLE_EDGE) ? 1.0f :
           (mode == EncoderMode::HALF_QUAD)   ? 2.0f :
           (mode == EncoderMode::FULL_QUAD)   ? 4.0f :
           1.0f;
}

// Variables usadas en cálculos (usado en varios módulos)
constexpr float RAW_ENCODER_PPR = 11.0f * 21.3f; // Steps del encoder x razón de engranaje de motor
constexpr float ENCODER_PPR = RAW_ENCODER_PPR * get_encoder_multiplier(ENCODER_MODE);
constexpr float RAD_PER_PULSE = (2.0f * PI) / ENCODER_PPR;

// RTOS
constexpr uint16_t ENCODER_READ_PERIOD_MS = 10;


/* -------------- Constantes del Pose Estimator --------------*/
constexpr uint16_t POSE_ESTIMATOR_PERIOD_MS = 200; 


/* -------------- Constantes de sensores de distancia --------------*/
constexpr uint16_t IR_SENSOR_READ_PERIOD_MS = 1000;
constexpr uint16_t US_SENSOR_READ_PERIOD_MS = 1000;


/* -------------- Constantes del control de posición --------------*/
enum PositionControlMode : uint8_t {
    SPEED_REF_INACTIVE = 0U,
    SPEED_REF_MANUAL = 1U,
    SPEED_REF_AUTO_BASIC = 2U,   
    SPEED_REF_AUTO_ADVANCED = 3U
};


/* -------------------- Estructuras con la data del sistema --------------------*/
/**
 * @brief Representa el estado cinemático del vehículo autónomo en el plano 2D.
 *
 * Contiene la pose estimada actual, la pose deseada como objetivo, las velocidades actuales
 * del vehículo y las referencias de velocidad lineal y angular generadas por el controlador de posición.
 *
 * La data es usada por los estimadores de pose, el controlador de posición, evasión y la lógica de navegación.
 * No debe modificarse directamente fuera de los módulos responsables.
 */
struct KinematicState {
    /// Posición actual en el eje X [m]
    float x;

    /// Posición actual en el eje Y [m]
    float y;

    /// Orientación actual del vehículo respecto al eje X [rad].
    /// θ = 0 indica orientación hacia el eje X positivo.
    float theta;

    /// Posición objetivo en el eje X [m].
    /// Esta coordenada es actualizada desde Firebase o secuencia de navegación.
    float x_d;

    /// Posición objetivo en el eje Y [m].
    float y_d;

    /// Orientación del objetivo respecto a los ejes de referencia [rad].
    float theta_d;

    /// Velocidad lineal actual del vehículo [m/s].
    /// Calculada por el estimador de pose.
    float v;

    /// Velocidad angular actual del vehículo [rad/s].
    /// Calculada por el estimador de pose.
    float w;
};


/**
 * @brief Indica el estado de activación de los distintos subsistemas del vehículo autónomo.
 *
 * Esta estructura es usada como bandera de control para suspender o reanudar módulos en ejecución
 * de forma segura desde la lógica central (`vehicle_os`) o al cambiar entre estados operacionales.
 *
 * Solo debe ser modificada por `vehicle_os` u otras tareas de sistema.
 */
struct SystemStates {
    /// Estado del subsistema de motores (MOTOR_IDLE, MOTOR_ACTIVE, etc.).
    /// Define el comportamiento general del controlador de motores y PWM.
    uint8_t motor_operation;

    /// Estado del lector de encoders (ACTIVE o INACTIVE).
    /// Si está inactivo, los encoders se pausan y la velocidad se congela en 0.
    uint8_t encoder;

    /// Estado del lector de IMU (ACTIVE o INACTIVE).
    /// Aún no implementado, pero reservado para integración futura.
    uint8_t imu;

    /// Estado del módulo de sensores de distancia (ultrasonido + IR).
    /// Controla la ejecución periódica de `distance_sensors`.
    uint8_t distance;

    /// Estado del estimador de pose (ACTIVE o INACTIVE).
    /// Determina si se actualiza la pose del vehículo en cada ciclo.
    uint8_t pose_estimator;

    /// Estado del controlador de posición (ACTIVE o INACTIVE).
    /// Define si se está generando v_ref/w_ref activamente.
    uint8_t position_controller;

    /// Estado del controlador de evasión de obstáculos (ACTIVE o INACTIVE).
    /// Si está activo, el vehículo ignora la referencia normal y ejecuta maniobras evasivas.
    uint8_t evade_controller;
};


/**
 * @brief Almacena la información de velocidad y control para cada rueda del vehículo.
 *
 * La data solo debe ser modificada por funciones hechas para ello y nunca directamente.
 */
struct WheelsData {
    /// Pasos/pulsos acumulados del encoder izquierdo.
    /// Se actualiza periódicamente por el módulo `encoder_reader`.
    /// Solo una lectura por medio del estimador de pose puede reiniciar su valor para evitar perder datos.
    int64_t steps_left;

    /// Pasos acumulados del encoder derecho.
    /// Incrementado de forma sincronizada con el encoder izquierdo.
    /// Solo una lectura por medio del estimador de pose puede reiniciar su valor para evitar perder datos.
    int64_t steps_right;

    /// Velocidad angular medida de la rueda izquierda [rad/s].
    /// Calculada a partir del número de pasos y el intervalo de muestreo.
    /// Se utiliza en el controlador PI realimentar y ajustar la velocidad.
    float wL_measured;

    /// Velocidad angular medida de la rueda derecha [rad/s].
    /// Calculada a partir del número de pasos y el intervalo de muestreo.
    /// Se utiliza en el controlador PI realimentar y ajustar la velocidad.
    float wR_measured;

    /// Velocidad angular de referencia para la rueda izquierda [rad/s].
    /// Calculada desde la velocidad lineal/angular deseada. Se ajusta por el controlador de posición.
    float wL_ref;

    /// Velocidad angular de referencia para la rueda derecha [rad/s].
    /// Calculada desde la velocidad lineal/angular deseada. Se ajusta por el controlador de posición.
    float wR_ref;

    /// Duty aplicado al PWM del motor izquierdo (normalizado en [-1, 1]).
    /// Su signo determina el sentido de giro de la rueda.
    /// Se ajusta automáticamente por el controlador de velocidad de rueda.
    float duty_left;

    /// Duty aplicado al PWM del motor derecho (normalizado en [-1, 1]).
    /// Se ajusta automáticamente por el controlador de velocidad.
    /// Se ajusta automáticamente por el controlador de velocidad de rueda.
    float duty_right;
};

/**
 * @brief Estructura que almacena el estado consolidado de sensores de distancia.
 *
 * Contiene:
 * - Bandera del sensor infrarrojo (detección de obstáculo)
 * - Distancia lateral izquierda (sensor ultrasónico)
 * - Distancia lateral derecha (sensor ultrasónico)
 */
struct DistanceSensorData {
    bool obstacle_detected;     ///< true si el sensor IR detecta un obstáculo
    uint8_t us_left_distance;   ///< Distancia medida por el sensor US izquierdo [cm]
    uint8_t us_right_distance;  ///< Distancia medida por el sensor US derecho [cm]
    bool ir_left_obstacle;
    bool ir_right_obstacle;
};

/**
 * @brief Estructura global de contexto que agrupa punteros a las principales estructuras de estado del sistema.
 * Esta estructura se utiliza para facilitar el paso de datos compartidos entre diferentes tareas RTOS o módulos del sistema.
 */
struct GlobalContext {
    volatile SystemStates* systems_ptr;
    volatile KinematicState* kinematic_ptr;
    volatile WheelsData* wheels_ptr;
    volatile DistanceSensorData* distance_ptr;
};

#endif
