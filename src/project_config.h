#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

/* -------------- Definiciones de los pines de la ESP32 --------------*/

// Para control del L298N
constexpr uint8_t MOTOR_LEFT_PWM_PIN  = 26;  // Azul
constexpr uint8_t MOTOR_LEFT_DIR_PIN1 = 25; // Blanco
constexpr uint8_t MOTOR_LEFT_DIR_PIN2 = 33; // Verde

constexpr uint8_t MOTOR_RIGHT_PWM_PIN  = 18;  // Azul
constexpr uint8_t MOTOR_RIGHT_DIR_PIN1 = 19; // Blanco
constexpr uint8_t MOTOR_RIGHT_DIR_PIN2 = 21; // Verde

// Para los encoders
// Azul = Vcc & Negro = GND 
constexpr uint8_t ENCODER_LEFT_A_PIN  = 14; // Amarillo
constexpr uint8_t ENCODER_LEFT_B_PIN  = 27; // Verde
constexpr uint8_t ENCODER_RIGHT_A_PIN = 16; // Amarillo
constexpr uint8_t ENCODER_RIGHT_B_PIN = 17; // Verde

// Sensores ultrasónicos
constexpr uint8_t US_LEFT_TRIG_PIN = 32;
constexpr uint8_t US_LEFT_ECHO_PIN = 34;

constexpr uint8_t US_MID_TRIG_PIN = 13;  
constexpr uint8_t US_MID_ECHO_PIN = 12;

constexpr uint8_t US_RIGHT_TRIG_PIN = 22;
constexpr uint8_t US_RIGHT_ECHO_PIN = 23;

// I2C para IMU
constexpr uint8_t IMU_SDA_PIN = 4;
constexpr uint8_t IMU_SCL_PIN = 5;

// Aux: LED para debug
constexpr uint8_t ESP32_LED = 2;


/* -------------- Constantes generales --------------*/

// Parámetros físicos del vehículo
constexpr float WHEEL_RADIUS = 0.067f / 2.0f;    // en metros
constexpr float WHEEL_DISTANCE = 0.194f / 2.0f;   // distancia entre ruedas (L)

// Motor speed characteristics
constexpr uint16_t RPM_NOM = 215U;            // rpm nominales bajo carga (son 280 sin carga)       
constexpr float WM_NOM = RPM_NOM * 2*PI/60.0; // rad/s nominales bajo carga = 22.51 (29.3 sin carga)

// Auxiliares
constexpr uint8_t WHEEL_LEFT  = 0U;
constexpr uint8_t WHEEL_RIGHT = 1U;
constexpr float MS_TO_S = 0.001f;

constexpr uint8_t ACTIVE   = 1U;
constexpr uint8_t INACTIVE = 0U;

constexpr bool SUCCESS   = 1U;
constexpr bool ERROR = 0U;

// Motor modes
enum class MotorMode : uint8_t {
    IDLE = 0U,    // Se dejan libres los motores, alta impedancia entre los bornes del motor
    ACTIVE = 1U,  // Se controla la velocidad con el duty y los pines de control
    AUTO = 2U,    // Modo de control automático
    BREAK = 3U    // Motores bloqueados, frena el motor
};

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

// Constantes para interpretar data de los encoders
constexpr float RAW_ENCODER_PPR = 11.0f * 21.3f; // Steps del encoder x razón de engranaje de motor
constexpr float ENCODER_PPR = RAW_ENCODER_PPR * get_encoder_multiplier(ENCODER_MODE);
constexpr float RAD_PER_PULSE = (2.0f * PI) / ENCODER_PPR;

// Opciones de control de posición
constexpr float W_STOP_THRESHOLD = 2.0f * PI / 30.0f; // Threshold de velocidad para considerar stop (1 vuelta en 30 segundos)
constexpr float V_STOP_THRESHOLD = 0.1 / 30.0f;         // Velocidad lineal mínima para considerar stop (10 cm en 30 segundos)

enum class PositionControlMode : uint8_t {
    INACTIVE = 0U,
    MANUAL,
    MOVE_PID,   
    TURN_PID,  
    MOVE_BACKS,  
    TURN_BACKS
};

enum class ControlType : uint8_t {
    PID = 0U,
    BACKS
};


/* ------------------------ Constantes OS ------------------------*/

enum class OS_State : uint8_t {
    INIT = 0,          // Estado inicial al encender
    IDLE,              // Espera sin movimiento
    STAND_BY,          // Espera sin movimiento
    MOVE,              // Desplazamiento hacia objetivo
    EVADE,             // Evasión de obstáculo
};

// Estructura de punto
struct TargetPoint {
    float x;
    float y;
};

// Instrucciones posibles desde Firebase o interfaz web
enum class RemoteCommand : uint8_t {
    NONE = 0,
    START,
    STOP,
    IDLE
};


constexpr uint8_t MAX_TRAJECTORY_POINTS = 100; // Define máximo de puntos
constexpr float NULL_WAYPOINT_XY = 99.9f;

/* -------------- Tiempos de poleo para tareas RTOS --------------*/

constexpr uint16_t WHEEL_CONTROL_PERIOD_MS = 10;
constexpr uint16_t ENCODER_READ_PERIOD_MS = 10;
constexpr uint16_t IMU_READ_PERIOD_MS = 20;
constexpr uint16_t OBSTACLE_CHECK_PERIOD_MS = 250;
constexpr uint16_t POSE_ESTIMATOR_PERIOD_MS = 100; 
constexpr uint16_t POSITION_CONTROL_PERIOD_MS = 200;
constexpr uint16_t OS_UPDATE_PERIOD_MS = 50; 
constexpr uint16_t BASIC_STACK_SIZE = 2048; // Tamaño de stack básico para tareas RTOS


/* -------------------- Estructuras con la data del sistema --------------------*/

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
    MotorMode motors;

    /// Estado del lector de encoders (ACTIVE o INACTIVE).
    /// Si está inactivo, los encoders se pausan y la velocidad se congela en 0.
    uint8_t encoders;

    /// Estado del lector de IMU (ACTIVE o INACTIVE).
    /// Aún no implementado, pero reservado para integración futura.
    uint8_t imu;

    /// Estado del módulo de sensores de distancia (ultrasonido + IR).
    /// Controla la ejecución periódica de `distance_sensors`.
    uint8_t distance;

    /// Estado del estimador de pose (ACTIVE o INACTIVE).
    /// Determina si se actualiza la pose del vehículo en cada ciclo.
    uint8_t pose;

    /// Estado del controlador de posición (ACTIVE o INACTIVE).
    /// Define si se está generando v_ref/w_ref activamente.
    PositionControlMode position;

    // /// Estado del controlador de evasión de obstáculos (ACTIVE o INACTIVE).
    // /// Si está activo, el vehículo ignora la referencia normal y ejecuta maniobras evasivas.
    // uint8_t evation;

    // Constructor por defecto (C++11+)
    SystemStates() :
        motors(MotorMode::IDLE), 
        encoders(INACTIVE), imu(INACTIVE), distance(INACTIVE), pose(INACTIVE),
        position(PositionControlMode::INACTIVE)
    {}
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
    int64_t steps_L;

    /// Pasos acumulados del encoder derecho.
    /// Incrementado de forma sincronizada con el encoder izquierdo.
    /// Solo una lectura por medio del estimador de pose puede reiniciar su valor para evitar perder datos.
    int64_t steps_R;

    /// Velocidad angular medida de la rueda izquierda [rad/s].
    /// Calculada a partir del número de pasos y el intervalo de muestreo.
    /// Se utiliza en el controlador PI realimentar y ajustar la velocidad.
    float w_L;

    /// Velocidad angular medida de la rueda derecha [rad/s].
    /// Calculada a partir del número de pasos y el intervalo de muestreo.
    /// Se utiliza en el controlador PI realimentar y ajustar la velocidad.
    float w_R;

    /// Velocidad angular de referencia para la rueda izquierda [rad/s].
    /// Calculada desde la velocidad lineal/angular deseada. Se ajusta por el controlador de posición.
    float w_L_ref;

    /// Velocidad angular de referencia para la rueda derecha [rad/s].
    /// Calculada desde la velocidad lineal/angular deseada. Se ajusta por el controlador de posición.
    float w_R_ref;

    /// Duty aplicado al PWM del motor izquierdo (normalizado en [-1, 1]).
    /// Su signo determina el sentido de giro de la rueda.
    /// Se ajusta automáticamente por el controlador de velocidad de rueda.
    float duty_L;

    /// Duty aplicado al PWM del motor derecho (normalizado en [-1, 1]).
    /// Se ajusta automáticamente por el controlador de velocidad.
    /// Se ajusta automáticamente por el controlador de velocidad de rueda.
    float duty_R;

    WheelsData()
        : steps_L(0), steps_R(0),
        w_L(0.0f), w_R(0.0f),
        w_L_ref(0.0f), w_R_ref(0.0f),
        duty_L(0.0f), duty_R(0.0f)
    {}
};

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

    /// Flag que indica si el objetivo fue alcanzado.
    uint8_t moving_state;

    KinematicState()
    : x(0.0f), y(0.0f), theta(0.0f),
      x_d(0.0f), y_d(0.0f), theta_d(0.0f),
      v(0.0f), w(0.0f),
      moving_state(MovingState::KIN_STOPPING) // Inicialmente detenido
    {}
};

/**
 * @brief Estructura que almacena el estado consolidado de sensores ultrasónicos frontales.
 *
 * Contiene:
 * - Distancias individuales medidas por sensores US (izq, medio, der)
 * - Flags de detección de obstáculo por cada sensor
 * - Bandera general combinada para uso global del sistema
 */
struct DistanceSensorData {
    bool obstacle_detected;        ///< true si cualquier sensor US detecta obstáculo (< threshold)

    uint8_t left_dist;      ///< Distancia medida por sensor US izquierdo [cm]
    uint8_t mid_dist;       ///< Distancia medida por sensor US medio/frontal [cm]
    uint8_t right_dist;     ///< Distancia medida por sensor US derecho [cm]

    bool left_obst;         ///< true si el sensor izquierdo detecta obstáculo (< threshold)
    bool mid_obst;          ///< true si el sensor medio detecta obstáculo
    bool right_obst;        ///< true si el sensor derecho detecta obstáculo

    DistanceSensorData()
    : obstacle_detected(false),
      left_dist(0), mid_dist(0), right_dist(0),
      left_obst(false), mid_obst(false), right_obst(false)
    {}
};

/**
 * @brief Estructura que almacena el estado consolidado del IMU.
 *
 * Contiene:
 * - Distancias individuales medidas por integracion en distancia (movimiento en ejes)
 * - Angulos de movimiento segun giroscopio
 * - Movimiento de angulos segun magnetometro
 */
struct IMUSensorData{
    float ax; ///< aceleración en el eje x
    float ay; ///< aceleración en el eje y
    float vx; ///< velocidad en el eje x
    float vy; ///< velocidad en el eje y
    float w_gyro; ///< velocidad angular con respecto al eje z
    float mag_angle; ///< ángulo rotación respecto al norte magnetico de output total (orientacion absoluta)

    IMUSensorData()
        : ax(0.0f), ay(0.0f),
        vx(0.0f), vy(0.0f),
        w_gyro(0.0f),
        mag_angle(0.0f)
    {}
};

/**
 * @brief Contiene la información de alto nivel relacionada con la operación del vehículo autónomo.
 *
 * Esta estructura centraliza el estado actual del sistema, la trayectoria de navegación cargada,
 * y la última instrucción remota recibida (por ejemplo, desde Firebase). Se utiliza por la máquina
 * de estados principal (`vehicle_os`) para coordinar la lógica de navegación y comportamiento del vehículo.
 *
 * Campos clave:
 * - current_state: Estado operativo actual del sistema (ej. INIT, NAVIGATING, etc.)
 * - trajectory: Lista de puntos objetivo a seguir, en orden.
 * - total_targets: Cantidad de puntos cargados actualmente en `trajectory`.
 * - last_remote_command: Última instrucción recibida por interfaz remota (START, STOP, etc.)
 */
struct OperationData {
    OS_State state;
    uint8_t total_targets;
    TargetPoint trajectory[MAX_TRAJECTORY_POINTS];
    RemoteCommand last_command;
    bool waypoint_reached;
    ControlType control_type;

    OperationData()
        : state(OS_State::INIT),
          total_targets(0),
          last_command(RemoteCommand::NONE),
          waypoint_reached(false),
          control_type(ControlType::PID)
    {
        for (uint8_t i = 0; i < MAX_TRAJECTORY_POINTS; ++i) {
            trajectory[i] = {NULL_WAYPOINT_XY, NULL_WAYPOINT_XY};
        }
    }
};

/**
 * @brief Estructura global de contexto que agrupa punteros a las principales estructuras de estado del sistema.
 * Esta estructura se utiliza para facilitar el paso de datos compartidos entre diferentes tareas RTOS o módulos del sistema.
 */
struct GlobalContext {
    volatile SystemStates* systems_ptr;
    volatile OperationData* os_ptr;
    volatile KinematicState* kinematic_ptr;
    volatile WheelsData* wheels_ptr;
    volatile IMUSensorData* imu_ptr;
    volatile DistanceSensorData* distance_ptr;
    volatile IMUSensorData* imu_ptr;
};

#endif
